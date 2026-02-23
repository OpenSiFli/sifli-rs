//! NVDS (Non-Volatile Data Store) shared memory initialization.
//!
//! Writes default BLE parameters to LCPU shared memory at `0x2040_FE00`.

const NVDS_BUFF_START: usize = crate::memory_map::shared::NVDS_BUFF_START;
const NVDS_PATTERN: u32 = 0x4E56_4453; // "NVDS"

mod tag {
    pub const BD_ADDRESS: u8 = 0x01;
    pub const PRE_WAKEUP_TIME: u8 = 0x0D;
    pub const EXT_WAKEUP_ENABLE: u8 = 0x12;
    pub const SCHEDULING: u8 = 0x15;
    pub const TRACER_CONFIG: u8 = 0x2F;
}

#[repr(C)]
struct NvdsHeader {
    pattern: u32,
    used_mem: u16,
    writing: u16,
}

/// Write default NVDS data to LCPU shared memory.
pub(crate) fn write_default(bd_addr: &[u8; 6], use_lxt: bool) {
    let mut buf = [0u8; 64];
    let mut pos = 0;

    if !use_lxt {
        buf[pos..pos + 4].copy_from_slice(&[tag::PRE_WAKEUP_TIME, 0x02, 0x64, 0x19]);
        pos += 4;
        buf[pos..pos + 3].copy_from_slice(&[tag::EXT_WAKEUP_ENABLE, 0x01, 0x01]);
        pos += 3;
    }

    buf[pos..pos + 6].copy_from_slice(&[tag::TRACER_CONFIG, 0x04, 0x20, 0x00, 0x00, 0x00]);
    pos += 6;

    buf[pos] = tag::BD_ADDRESS;
    buf[pos + 1] = 0x06;
    buf[pos + 2..pos + 8].copy_from_slice(bd_addr);
    pos += 8;

    buf[pos..pos + 3].copy_from_slice(&[tag::SCHEDULING, 0x01, 0x01]);
    pos += 3;

    unsafe {
        let dst = NVDS_BUFF_START as *mut u8;
        let header = NvdsHeader {
            pattern: NVDS_PATTERN,
            used_mem: pos as u16,
            writing: 0,
        };
        core::ptr::write_volatile(dst as *mut NvdsHeader, header);
        core::ptr::copy_nonoverlapping(buf.as_ptr(), dst.add(8), pos);
    }

    debug!(
        "NVDS written: {} bytes, bd_addr={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        pos, bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]
    );
}
