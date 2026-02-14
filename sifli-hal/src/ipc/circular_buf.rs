//! Rust reimplementation of SDK `circular_buf` (shared memory wire protocol).
//!
//! Note: This struct resides in cross-core shared RAM; fields may be asynchronously
//! modified by the other core. The Rust side must avoid creating `&`/`&mut` references
//! (which would trigger aliasing/data-race UB), so raw pointers + volatile reads/writes
//! are used to match SDK behavior.

use core::ptr;
use core::sync::atomic::{fence, Ordering};

const CB_PTR_MIRROR_MASK: u32 = 0xFFFF;
const CB_PTR_IDX_OFFSET: u32 = 16;
const CB_PTR_IDX_MASK: u32 = 0xFFFF;

#[inline]
const fn cb_make_ptr_idx_mirror(idx: u16, mirror: u16) -> u32 {
    ((idx as u32) << CB_PTR_IDX_OFFSET) | ((mirror as u32) & CB_PTR_MIRROR_MASK)
}

#[inline]
const fn cb_get_ptr_idx(ptr_idx_mirror: u32) -> u16 {
    ((ptr_idx_mirror >> CB_PTR_IDX_OFFSET) & CB_PTR_IDX_MASK) as u16
}

#[inline]
const fn cb_get_ptr_mirror(ptr_idx_mirror: u32) -> u16 {
    (ptr_idx_mirror & CB_PTR_MIRROR_MASK) as u16
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CircularBufState {
    Empty,
    Full,
    HalfFull,
}

#[inline]
fn circular_buf_status(rd_ptr: u32, wr_ptr: u32) -> CircularBufState {
    let rd_idx = cb_get_ptr_idx(rd_ptr);
    let rd_mirror = cb_get_ptr_mirror(rd_ptr);
    let wr_idx = cb_get_ptr_idx(wr_ptr);
    let wr_mirror = cb_get_ptr_mirror(wr_ptr);

    if rd_idx == wr_idx {
        if rd_mirror == wr_mirror {
            CircularBufState::Empty
        } else {
            CircularBufState::Full
        }
    } else {
        CircularBufState::HalfFull
    }
}

/// Rust reimplementation of SiFli SDK `struct circular_buf` (shared memory wire protocol).
#[repr(C)]
pub(crate) struct CircularBuf {
    pub(crate) rd_buffer_ptr: *mut u8,
    pub(crate) wr_buffer_ptr: *mut u8,
    pub(crate) read_idx_mirror: u32,
    pub(crate) write_idx_mirror: u32,
    pub(crate) buffer_size: i16,
}

pub(crate) trait CircularBufPtrExt {
    unsafe fn rd_buffer_ptr(self) -> *mut u8;
    unsafe fn wr_buffer_ptr(self) -> *mut u8;
    unsafe fn buffer_size(self) -> usize;
    unsafe fn read_idx_mirror(self) -> u32;
    unsafe fn write_idx_mirror(self) -> u32;

    unsafe fn data_len(self) -> usize;
    unsafe fn space_len(self) -> usize;
}

pub(crate) trait CircularBufMutPtrExt {
    unsafe fn set_read_idx_mirror(self, v: u32);
    unsafe fn set_write_idx_mirror(self, v: u32);

    unsafe fn wr_init(self, pool: *mut u8, size: i16);
    unsafe fn rd_init(self, pool: *mut u8);

    unsafe fn put(self, data: &[u8]) -> usize;
    unsafe fn get(self, out: &mut [u8]) -> usize;
}

impl CircularBufPtrExt for *const CircularBuf {
    #[inline]
    unsafe fn rd_buffer_ptr(self) -> *mut u8 {
        ptr::read_volatile(ptr::addr_of!((*self).rd_buffer_ptr))
    }

    #[inline]
    unsafe fn wr_buffer_ptr(self) -> *mut u8 {
        ptr::read_volatile(ptr::addr_of!((*self).wr_buffer_ptr))
    }

    #[inline]
    unsafe fn buffer_size(self) -> usize {
        (ptr::read_volatile(ptr::addr_of!((*self).buffer_size)) as i32)
            .max(0) as usize
    }

    #[inline]
    unsafe fn read_idx_mirror(self) -> u32 {
        ptr::read_volatile(ptr::addr_of!((*self).read_idx_mirror))
    }

    #[inline]
    unsafe fn write_idx_mirror(self) -> u32 {
        ptr::read_volatile(ptr::addr_of!((*self).write_idx_mirror))
    }

    unsafe fn data_len(self) -> usize {
        let buf_size = self.buffer_size();
        if buf_size == 0 {
            return 0;
        }

        let rd_ptr = self.read_idx_mirror();
        let wr_ptr = self.write_idx_mirror();
        match circular_buf_status(rd_ptr, wr_ptr) {
            CircularBufState::Empty => 0,
            CircularBufState::Full => buf_size,
            CircularBufState::HalfFull => {
                let rd_idx = cb_get_ptr_idx(rd_ptr) as usize;
                let wr_idx = cb_get_ptr_idx(wr_ptr) as usize;
                let len = if wr_idx > rd_idx {
                    wr_idx - rd_idx
                } else {
                    buf_size.saturating_sub(rd_idx.saturating_sub(wr_idx))
                };
                len.min(buf_size)
            }
        }
    }

    #[inline]
    unsafe fn space_len(self) -> usize {
        self.buffer_size().saturating_sub(self.data_len())
    }
}

impl CircularBufMutPtrExt for *mut CircularBuf {
    #[inline]
    unsafe fn set_read_idx_mirror(self, v: u32) {
        ptr::write_volatile(ptr::addr_of_mut!((*self).read_idx_mirror), v);
    }

    #[inline]
    unsafe fn set_write_idx_mirror(self, v: u32) {
        ptr::write_volatile(ptr::addr_of_mut!((*self).write_idx_mirror), v);
    }

    unsafe fn wr_init(self, pool: *mut u8, size: i16) {
        assert!(size > 0);

        self.set_read_idx_mirror(0);
        self.set_write_idx_mirror(0);
        ptr::write_volatile(ptr::addr_of_mut!((*self).wr_buffer_ptr), pool);
        ptr::write_volatile(
            ptr::addr_of_mut!((*self).buffer_size),
            (size as i32 & !3) as i16,
        );
    }

    #[inline]
    unsafe fn rd_init(self, pool: *mut u8) {
        ptr::write_volatile(ptr::addr_of_mut!((*self).rd_buffer_ptr), pool);
    }

    unsafe fn put(self, data: &[u8]) -> usize {
        let mut length = data.len();
        if length == 0 {
            return 0;
        }

        let buf_size = (self as *const CircularBuf).buffer_size();
        if buf_size == 0 {
            return 0;
        }

        let space = (self as *const CircularBuf).space_len();
        if space == 0 {
            return 0;
        }
        if space < length {
            length = space;
        }

        let write_idx_mirror = (self as *const CircularBuf).write_idx_mirror();
        let mut wr_idx = cb_get_ptr_idx(write_idx_mirror) as usize;
        let mut wr_mirror = cb_get_ptr_mirror(write_idx_mirror);
        let pool = (self as *const CircularBuf).wr_buffer_ptr();

        if wr_idx > buf_size {
            return 0;
        }

        if buf_size.saturating_sub(wr_idx) > length {
            ptr::copy_nonoverlapping(data.as_ptr(), pool.add(wr_idx), length);
            fence(Ordering::SeqCst);
            wr_idx += length;
            self.set_write_idx_mirror(cb_make_ptr_idx_mirror(wr_idx as u16, wr_mirror));
            return length;
        }

        let first = buf_size - wr_idx;
        let second = length - first;

        ptr::copy_nonoverlapping(data.as_ptr(), pool.add(wr_idx), first);
        ptr::copy_nonoverlapping(data.as_ptr().add(first), pool, second);

        fence(Ordering::SeqCst);
        wr_mirror = (!wr_mirror) & 0xFFFF;
        wr_idx = second;
        self.set_write_idx_mirror(cb_make_ptr_idx_mirror(wr_idx as u16, wr_mirror));

        length
    }

    unsafe fn get(self, out: &mut [u8]) -> usize {
        let mut length = out.len();
        if length == 0 {
            return 0;
        }

        let size = (self as *const CircularBuf).data_len();
        if size == 0 {
            return 0;
        }
        if size < length {
            length = size;
        }

        let read_idx_mirror = (self as *const CircularBuf).read_idx_mirror();
        let mut rd_idx = cb_get_ptr_idx(read_idx_mirror) as usize;
        let mut rd_mirror = cb_get_ptr_mirror(read_idx_mirror);
        let buf_size = (self as *const CircularBuf).buffer_size();
        let pool = (self as *const CircularBuf).rd_buffer_ptr();

        if buf_size == 0 || pool.is_null() || rd_idx > buf_size {
            return 0;
        }

        if buf_size.saturating_sub(rd_idx) > length {
            ptr::copy_nonoverlapping(pool.add(rd_idx), out.as_mut_ptr(), length);
            fence(Ordering::SeqCst);
            rd_idx += length;
            self.set_read_idx_mirror(cb_make_ptr_idx_mirror(rd_idx as u16, rd_mirror));
            return length;
        }

        let first = buf_size - rd_idx;
        let second = length - first;

        ptr::copy_nonoverlapping(pool.add(rd_idx), out.as_mut_ptr(), first);
        ptr::copy_nonoverlapping(pool, out.as_mut_ptr().add(first), second);

        fence(Ordering::SeqCst);
        rd_mirror = (!rd_mirror) & 0xFFFF;
        rd_idx = second;
        self.set_read_idx_mirror(cb_make_ptr_idx_mirror(rd_idx as u16, rd_mirror));

        length
    }
}

