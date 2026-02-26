//! Bounds-checked RAM slice accessor for block operations.
//!
//! Provides safe `clear()`, `copy_from_slice()`, and `write()` methods
//! that confine `unsafe` to this single type, replacing scattered
//! `write_bytes` / `copy_nonoverlapping` calls throughout HAL and radio code.

/// A slice of RAM defined by base address and length in bytes.
///
/// Wraps a raw address + length pair, providing bounds-checked block operations.
/// All `unsafe` pointer access is confined within these methods.
///
/// # Usage
///
/// ```ignore
/// use sifli_hal::ram::RamSlice;
///
/// let em = RamSlice::new(0x2040_8000, 0x5000);
/// em.clear();
///
/// let nvds = RamSlice::new(0x2040_FE00, 0x200);
/// nvds.write(0, header);
/// nvds.copy_at(8, &data[..len]);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct RamSlice {
    addr: usize,
    len: usize,
}

impl RamSlice {
    /// Create a new RAM slice.
    ///
    /// # Safety contract
    ///
    /// Caller must ensure `addr..addr+len` is a valid, writable memory
    /// range (e.g., LCPU shared memory, Exchange Memory, NVDS buffer).
    #[inline]
    pub const fn new(addr: usize, len: usize) -> Self {
        Self { addr, len }
    }

    /// Base address.
    #[inline]
    pub const fn addr(&self) -> usize {
        self.addr
    }

    /// Length in bytes.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Clear the entire region to zero.
    #[inline]
    pub fn clear(&self) {
        unsafe {
            core::ptr::write_bytes(self.addr as *mut u8, 0, self.len);
        }
    }

    /// Copy a byte slice into this region at offset 0.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() > len`.
    #[inline]
    pub fn copy_from_slice(&self, data: &[u8]) {
        assert!(
            data.len() <= self.len,
            "RamSlice copy overflow: {} bytes into {} byte region",
            data.len(),
            self.len,
        );
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), self.addr as *mut u8, data.len());
        }
    }

    /// Copy a byte slice into this region at the given byte offset.
    ///
    /// # Panics
    ///
    /// Panics if `offset + data.len() > len`.
    #[inline]
    pub fn copy_at(&self, offset: usize, data: &[u8]) {
        assert!(
            offset + data.len() <= self.len,
            "RamSlice copy_at overflow: offset {} + {} bytes > len {}",
            offset,
            data.len(),
            self.len,
        );
        unsafe {
            core::ptr::copy_nonoverlapping(
                data.as_ptr(),
                (self.addr + offset) as *mut u8,
                data.len(),
            );
        }
    }

    /// Read a value at the given byte offset using `read_volatile`.
    ///
    /// # Panics
    ///
    /// Panics if `offset + size_of::<T>() > len` or if the resulting address
    /// is not aligned for `T`.
    #[inline]
    pub fn read<T: Copy>(&self, offset: usize) -> T {
        assert!(
            offset + core::mem::size_of::<T>() <= self.len,
            "RamSlice read overflow: offset {} + {} bytes > len {}",
            offset,
            core::mem::size_of::<T>(),
            self.len,
        );
        let addr = self.addr + offset;
        assert!(
            addr % core::mem::align_of::<T>() == 0,
            "RamSlice: unaligned read at {:#x} (required alignment: {})",
            addr,
            core::mem::align_of::<T>(),
        );
        unsafe { core::ptr::read_volatile(addr as *const T) }
    }

    /// Create a sub-slice at `offset` bytes with `len` bytes.
    ///
    /// # Panics
    ///
    /// Panics if `offset + len > self.len`.
    #[inline]
    pub fn slice(&self, offset: usize, len: usize) -> Self {
        assert!(
            offset + len <= self.len,
            "RamSlice slice overflow: offset {} + {} > len {}",
            offset,
            len,
            self.len,
        );
        Self {
            addr: self.addr + offset,
            len,
        }
    }

    /// Write a value at the given byte offset using `write_volatile`.
    ///
    /// # Panics
    ///
    /// Panics if `offset + size_of::<T>() > len` or if the resulting address
    /// is not aligned for `T`.
    #[inline]
    pub fn write<T: Copy>(&self, offset: usize, value: T) {
        assert!(
            offset + core::mem::size_of::<T>() <= self.len,
            "RamSlice write overflow: offset {} + {} bytes > len {}",
            offset,
            core::mem::size_of::<T>(),
            self.len,
        );
        let addr = self.addr + offset;
        assert!(
            addr % core::mem::align_of::<T>() == 0,
            "RamSlice: unaligned write at {:#x} (required alignment: {})",
            addr,
            core::mem::align_of::<T>(),
        );
        unsafe {
            core::ptr::write_volatile(addr as *mut T, value);
        }
    }
}
