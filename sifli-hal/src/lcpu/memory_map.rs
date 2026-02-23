//! LCPU subsystem memory map constants (single source of truth).

/// Addresses shared across all chip revisions.
pub mod shared {
    pub const LPSYS_RAM_BASE: usize = 0x2040_0000;
    pub const NVDS_BUFF_START: usize = 0x2040_FE00;
    pub const EM_START: usize = 0x2040_8000;
    pub const EM_SIZE: usize = 0x5000;
    pub const HCPU2LCPU_MB_CH1: usize = 0x2007_FE00;
    pub const HCPU2LCPU_MB_CH2: usize = 0x2007_FC00;
    pub const HCPU_TO_LCPU_OFFSET: usize = 0x0A00_0000;
}

/// Bluetooth RF peripheral addresses.
pub mod rf {
    pub const BT_RFC_MEM_BASE: u32 = 0x4008_2000;
    pub const CFO_PHASE_ADDR: u32 = 0x4008_2790;
    pub const PHY_RX_DUMP_ADDR: u32 = 0x400C_0000;
}

/// A3 revision specific addresses.
pub mod a3 {
    pub const ROM_CONFIG_BASE: usize = 0x2040_FDC0;
    pub const LCPU2HCPU_CH1: usize = 0x2040_5C00;
    pub const LCPU2HCPU_CH2: usize = 0x2040_5E00;
    pub const PATCH_CODE_START: usize = 0x2040_6000;
    pub const PATCH_RECORD_ADDR: usize = 0x2040_7F00;
    pub const RWIP_PROG_DELAY: usize = 0x2040_FA94;
    pub const G_ROM_CONFIG: usize = 0x2040_E48C;
}

/// Letter Series ROM runtime variable addresses.
///
/// Source: `lcpu_rom_micro_rev7.axf` symbol table, verified by ROM fingerprint.
/// `_rwip_sleep` at 0x4430 matches hardware observation at `RWIP_SLEEP_HANDLER`.
pub mod letter_rom {
    /// `_rwip_sleep` handler function pointer (DATA segment).
    /// Read this to fingerprint the ROM version: expected value 0x4431.
    pub const RWIP_SLEEP_HANDLER: usize = 0x2040_00AC;

    /// `idle_hook_list[4]` array base.
    /// RT-Thread idle thread iterates this array, calling non-NULL function pointers.
    /// ROM pre-populates: [0]=rt_hw_watchdog_pet, [1]=bluetooth_idle_hook_func (weak empty).
    pub const IDLE_HOOK_LIST: usize = 0x2040_0DAC;

    /// `bluetooth_idle_hook_func` — weak empty stub at idle_hook_list[1].
    /// Used to verify the expected value before overwriting.
    pub const BLUETOOTH_IDLE_HOOK_FUNC: u32 = 0x0000_5481;

    /// Expected fingerprint value at RWIP_SLEEP_HANDLER.
    pub const FINGERPRINT: u32 = 0x4431;

    /// `rwip_env` base address (BSS segment).
    /// RivieraWaves BLE stack runtime environment structure.
    /// Verified by observing `prevent_sleep` at +0x00 and literal pool in `_rwip_sleep`.
    pub const RWIP_ENV: usize = 0x2040_1384;

    /// Offset of `lp_ref_cycle` within `rwip_env`.
    ///
    /// `_rwip_sleep()` reads this via `rwip_get_lp_ref_cycle()` to convert
    /// half-slot durations to LP clock cycles. Formula:
    ///   `lp_cycles = (delta_hs × rc_cycle × 15000) / lp_ref_cycle`
    ///
    /// If zero (uninitialized), the division by zero causes sleep to never activate.
    pub const RWIP_ENV_LP_REF_CYCLE_OFFSET: usize = 0x1d8;
}

/// Letter Series (A4/B4) specific addresses.
pub mod letter {
    pub const ROM_CONFIG_BASE: usize = 0x2040_2A00;
    pub const LCPU2HCPU_CH1: usize = 0x2040_2800;
    pub const LCPU2HCPU_CH2: usize = 0x2040_2A00;
    pub const PATCH_BUF_START: usize = 0x2040_5000;
    pub const PATCH_CODE_START: usize = 0x2040_500C;
    pub const PATCH_CODE_START_LCPU: usize = 0x0040_500C;
}
