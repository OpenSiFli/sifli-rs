//! NVDS (Non-Volatile Data Store) shared memory initialization.
//!
//! Writes default BLE parameters (BD address, tracer config, etc.) to the
//! LCPU shared memory region at `0x2040_FE00`. LCPU ROM reads this area
//! during initialization to configure the BLE stack.
//!
//! SDK equivalent: `bt_stack_nvds_init()` in `bf0_bt_common.c:318`.

const NVDS_BUFF_START: usize = super::memory_map::shared::NVDS_BUFF_START;
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

/// Write default NVDS data to LCPU shared memory at `0x2040_FE00`.
///
/// SDK: `bt_stack_nvds_init()` → `sifli_nvds_get_default_vaule()` → memcpy to `NVDS_BUFF_START`.
///
/// Must be called before LCPU boot (before `power_on()`), with LCPU SRAM
/// accessible (i.e. after `wake_lcpu()`).
pub(crate) fn write_default(bd_addr: &[u8; 6], use_lxt: bool) {
    let mut buf = [0u8; 64];
    let mut pos = 0;

    // RC10K mode extra parameters (SDK: bf0_bt_nvds.c:119-125)
    if !use_lxt {
        // Tag 0x0D: pre-wakeup time = 0x1964 (6500)
        buf[pos..pos + 4].copy_from_slice(&[tag::PRE_WAKEUP_TIME, 0x02, 0x64, 0x19]);
        pos += 4;
        // Tag 0x12: ext wakeup enable = 1
        buf[pos..pos + 3].copy_from_slice(&[tag::EXT_WAKEUP_ENABLE, 0x01, 0x01]);
        pos += 3;
    }

    // Tag 0x2F: tracer config = [0x20, 0x00, 0x00, 0x00]
    buf[pos..pos + 6].copy_from_slice(&[tag::TRACER_CONFIG, 0x04, 0x20, 0x00, 0x00, 0x00]);
    pos += 6;

    // Tag 0x01: BD address (6 bytes)
    buf[pos] = tag::BD_ADDRESS;
    buf[pos + 1] = 0x06;
    buf[pos + 2..pos + 8].copy_from_slice(bd_addr);
    pos += 8;

    // Tag 0x15: scheduling = 1
    buf[pos..pos + 3].copy_from_slice(&[tag::SCHEDULING, 0x01, 0x01]);
    pos += 3;

    // Write header + TLV data to shared memory
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
