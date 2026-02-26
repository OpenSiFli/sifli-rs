//! LCPU subsystem memory map constants.
//!
//! Hardware-derived constants (`shared`, `rf`, `a3`, `letter`) are generated
//! by `build.rs` from `data/sf32lb52x/sram_layout.toml`.
//!
//! ROM reverse-engineered symbols (`a3_rom`, `letter_rom`) are maintained
//! manually below.

// ── Generated from sram_layout.toml ─────────────────────────────────────────

include!(concat!(env!("OUT_DIR"), "/memory_map_generated.rs"));

// ── ROM reverse-engineered symbols (hand-maintained) ────────────────────────

/// A3 revision ROM symbol addresses.
///
/// Source: `lcpu_rom_micro.axf` symbol table.
pub mod a3_rom {
    /// `rwip_prog_delay` ROM variable address.
    pub const RWIP_PROG_DELAY: usize = 0x2040_FA94;
    /// `g_rom_config` ROM variable address (in ROM Private RAM region).
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
    pub const BLUETOOTH_IDLE_HOOK_FUNC: u32 = 0x5481;
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
    pub const RWIP_ENV_LP_REF_CYCLE_OFFSET: usize = 0x1D8;
}
