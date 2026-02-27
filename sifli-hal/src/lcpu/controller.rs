//! Post-boot BLE controller initialization.
//!
//! SDK equivalent: `bluetooth_init()` in
//! `SiFli-SDK/middleware/bluetooth/service/bluetooth.c:563`.
//!
//! The SDK runs this on the LCPU side after boot. Since the Rust HAL
//! doesn't run LCPU middleware, we replicate critical steps from HCPU
//! by writing directly to LCPU shared memory and peripheral registers.
//!
//! ## SDK `bluetooth_init()` steps
//!
//! | # | SDK function              | Rust function              | Notes                        |
//! |---|---------------------------|----------------------------|------------------------------|
//! | 1 | `rf_ptc_config(1)`        | `setup_cfo_tracking()`     | PTC2 CFO phase tracking      |
//! | 2 | `bluetooth_isr_init()`    | —                          | LCPU NVIC, ROM handles       |
//! | 3 | `ble_xtal_less_init()`    | `configure_sleep_timing()` | lld_prog_delay + clock cfg   |
//! | 4 | `bluetooth_pm_init()`     | —                          | LCPU PM, we disable sleep    |
//! | 5 | `bluetooth_config()`      | (controller_enable_bit)    | Partial: enable BLE+BT       |
//! | 6 | `HAL_RCC_SetMacFreq()`    | `configure_mac_clock()`    | LPSYS_RCC MACDIV (8MHz)      |
//! | 7 | `wlan_coex_config()`      | —                          | Not needed                   |

use super::config::ControllerConfig;
use super::ram::BtRomConfig;
use crate::syscfg;

/// LCPU ROM runtime variable addresses.
///
/// Source: `SiFli-SDK/example/rom_bin/lcpu_boot_loader/lcpu_rom_micro.map`
/// and `SiFli-SDK/example/rom_bin/lcpu_general_ble_img/lcpu_img_56x_52.axf`
mod addr {
    use super::super::memory_map::a3;
    use super::BtRomConfig;

    /// `rwip_prog_delay` address for A3 revision.
    pub const RWIP_PROG_DELAY_A3: *mut u8 = a3::RWIP_PROG_DELAY as _;

    /// `g_rom_config` base address for A3 revision (21 bytes).
    pub const G_ROM_CONFIG_A3: *mut BtRomConfig = a3::G_ROM_CONFIG as _;
}

/// `g_rom_config.bit_valid` bit definitions.
///
/// Each bit indicates the corresponding field has been explicitly configured.
/// ROM uses internal defaults for fields whose valid bit is NOT set.
///
/// Source: SDK `rom_config_set_*()` functions in `ble_rom_config.c`.
mod bit {
    /// `controller_enable_bit` (field +8).
    pub const CONTROLLER_ENABLE: u32 = 1 << 1;
    /// `lld_prog_delay` (field +9).
    pub const LLD_PROG_DELAY: u32 = 1 << 2;
    /// `default_sleep_mode` (field +11).
    pub const SLEEP_MODE: u32 = 1 << 4;
    /// `default_sleep_enabled` (field +12).
    pub const SLEEP_ENABLED: u32 = 1 << 5;
    /// `default_xtal_enabled` (field +13).
    pub const XTAL_ENABLED: u32 = 1 << 6;
    /// `default_rc_cycle` (field +14).
    pub const RC_CYCLE: u32 = 1 << 7;
}

/// BT_PKTDET trigger source index for PTC2 (SDK: `PTC_LCPU_BT_PKTDET`).
const PTC_LCPU_BT_PKTDET: u8 = 105;

/// CFO phase storage address in BT_RFC SRAM region.
///
/// SDK: `cfo_phase_t *pt_cfo = (cfo_phase_t *)0x40082790` in `bluetooth_misc.c:282`.
const CFO_PHASE_ADDR: u32 = super::memory_map::rf::CFO_PHASE_ADDR;

/// CFO phase storage size in bytes (SDK: `sizeof(cfo_phase_t)` = 10).
const CFO_PHASE_SIZE: usize = 10;

use crate::pac::ptc::vals::Op as PtcOp;

/// SDK `bluetooth_init()` equivalent.
///
/// Configures the BLE controller runtime environment after LCPU boot.
/// Must be called after warmup event consumption and before BLE connections.
pub(crate) fn init(config: &ControllerConfig) {
    // SDK step 3: ble_xtal_less_init() — BLE scheduler sleep timing parameters
    configure_sleep_timing(config);

    // SDK step 6: HAL_RCC_SetMacFreq() — MAC peripheral clock to 8MHz
    configure_mac_clock();

    // SDK step 1: rf_ptc_config(1) — PTC2 packet-detect trigger for CFO tracking
    setup_cfo_tracking();

    // SDK step 4 replacement: bt_sleep_control(0) — disable BLE controller sleep
    //
    // We lack ble_standby_sleep_after_handler() (which reconfigures MAC clock + PTC
    // after every wakeup). Without it, sleep causes MAC clock loss → 0x3E timeout.
    disable_ble_sleep();
}

/// Configure BLE scheduler sleep timing parameters.
///
/// SDK equivalent: `ble_xtal_less_init()` in `bluetooth.c:79-101`.
///
/// For A3: writes `rwip_prog_delay` runtime variable and `g_rom_config` fields
/// that control BLE scheduler timing (sleep mode, XTAL, RC cycle).
/// For Letter Series: skipped entirely — ROM reads these from `RomControlBlock.bt_config`.
fn configure_sleep_timing(config: &ControllerConfig) {
    let is_letter = syscfg::read_idr().revision().is_letter_series();

    // ── A3 only: rwip_prog_delay + g_rom_config ──
    //
    // For A3: write rwip_prog_delay runtime variable and g_rom_config fields directly.
    // For Letter Series: rwip_prog_delay is set by ROM from RomControlBlock.bt_config
    // (configured in ram::rom_config() with LLD_PROG_DELAY bit), and g_rom_config
    // is also handled by ROM. Writing to the A3 address (0x2040_FA94) on Letter Series
    // is harmless but pointless, while the SF32LB56x address (0x2041_6338) is out of
    // bounds on SF32LB52x Letter Series and would cause a bus fault.
    if !is_letter {
        // rwip_prog_delay: direct runtime variable read by sch_arb_insert(), etc.
        unsafe {
            core::ptr::write_volatile(addr::RWIP_PROG_DELAY_A3, config.lld_prog_delay);
        }
        debug!(
            "Controller init: rwip_prog_delay={} written to 0x{:08X}",
            config.lld_prog_delay,
            addr::RWIP_PROG_DELAY_A3 as usize
        );
        let cfg_ptr = addr::G_ROM_CONFIG_A3;

        // Dump before modification.
        let rom_cfg = unsafe { core::ptr::read_volatile(cfg_ptr) };
        debug!(
            "g_rom_config BEFORE: bit_valid=0x{:08X} prog_delay={} sleep_mode={} sleep_enabled={} xtal_enabled={} rc_cycle={}",
            rom_cfg.bit_valid,
            rom_cfg.lld_prog_delay,
            rom_cfg.default_sleep_mode,
            rom_cfg.default_sleep_enabled,
            rom_cfg.default_xtal_enabled,
            rom_cfg.default_rc_cycle,
        );

        unsafe {
            // Write fields via struct references (mirrors SDK rom_config_set_* functions).
            let cfg = &mut *cfg_ptr;

            // controller_enable_bit = BLE(1) | BT(2) = 3
            // SDK: rom_config_set_controller_enabled(BT_CONTROLLER_ENABLE_MASK | BLE_CONTROLLER_ENABLE_MASK)
            core::ptr::write_volatile(&mut cfg.controller_enable_bit, 0x03);

            // lld_prog_delay
            core::ptr::write_volatile(&mut cfg.lld_prog_delay, config.lld_prog_delay);

            // default_sleep_mode = 0 (no sleep)
            core::ptr::write_volatile(&mut cfg.default_sleep_mode, 0);

            // default_sleep_enabled = 0 (disable sleep)
            // Without bluetooth_pm_init(), sleep would cause missed connection events.
            core::ptr::write_volatile(&mut cfg.default_sleep_enabled, 0);

            // default_xtal_enabled
            core::ptr::write_volatile(&mut cfg.default_xtal_enabled, config.xtal_enabled as u8);

            // default_rc_cycle
            core::ptr::write_volatile(&mut cfg.default_rc_cycle, config.rc_cycle);

            // Update bit_valid — tell ROM these fields are explicitly configured.
            let old_valid = core::ptr::read_volatile(&cfg.bit_valid);
            let new_valid = old_valid
                | bit::CONTROLLER_ENABLE
                | bit::LLD_PROG_DELAY
                | bit::SLEEP_MODE
                | bit::SLEEP_ENABLED
                | bit::XTAL_ENABLED
                | bit::RC_CYCLE;
            core::ptr::write_volatile(&mut cfg.bit_valid, new_valid);
        }

        // Verify.
        let rom_cfg = unsafe { core::ptr::read_volatile(cfg_ptr) };
        debug!(
            "g_rom_config AFTER: bit_valid=0x{:08X} prog_delay={} sleep_mode={} sleep_enabled={} xtal_enabled={} rc_cycle={}",
            rom_cfg.bit_valid,
            rom_cfg.lld_prog_delay,
            rom_cfg.default_sleep_mode,
            rom_cfg.default_sleep_enabled,
            rom_cfg.default_xtal_enabled,
            rom_cfg.default_rc_cycle,
        );
    } else {
        debug!("Controller init: Letter Series, rwip_prog_delay set via RomControlBlock");
    }
}

/// Configure MAC peripheral clock to 8MHz.
///
/// SDK equivalent: `HAL_RCC_SetMacFreq()` in `bf0_hal_rcc.c:2047`.
/// Also called in `ble_standby_sleep_after_handler()` after every BLE sleep wakeup.
fn configure_mac_clock() {
    use crate::rcc;
    let hclk = rcc::get_lpsys_hclk_freq().unwrap_or(crate::time::Hertz(24_000_000));
    let mac_div = (hclk.0 / 8_000_000) as u8;
    if mac_div == 0 {
        warn!(
            "LPSYS HCLK {}Hz < 8MHz, MAC clock divider would be 0, using 1",
            hclk.0
        );
        rcc::config_lpsys_mac_clock(1, 0x08);
    } else {
        rcc::config_lpsys_mac_clock(mac_div, 0x08);
    }
    debug!(
        "MAC clock configured: HCLK={}Hz, MACDIV={}, MACFREQ=8",
        hclk.0,
        mac_div.max(1)
    );
}

/// Configure PTC2 for CFO (Carrier Frequency Offset) phase tracking.
///
/// SDK equivalent: `rf_ptc_config(1)` in `bluetooth_misc.c:356`.
///
/// On SF32LB52X, this configures PTC2 task 1 to trigger on BT_PKTDET
/// (packet detection hardware signal), performing an OR write to the
/// CFO phase storage address. The PTC2 ISR on LCPU reads `BT_PHY.RX_STATUS1.CFO_PHASE`
/// and saves it — HCPU only sets up the hardware trigger.
fn setup_cfo_tracking() {
    let ptc2 = crate::pac::PTC2;

    // 1. Enable PTC2 clock (LPSYS_RCC.ENR1.PTC2)
    crate::pac::LPSYS_RCC.enr1().modify(|w| w.set_ptc2(true));

    // 2. Clear CFO phase storage (SDK: memset(pt_cfo, 0, 10))
    unsafe {
        core::ptr::write_bytes(CFO_PHASE_ADDR as *mut u8, 0, CFO_PHASE_SIZE);
    }

    // 3. Configure task 1 (SDK g_ptc_task[0] = 1, i.e. hardware channel 1)
    //    TAR = CFO phase address
    ptc2.tar1().write(|w| w.set_addr(CFO_PHASE_ADDR));
    //    TDR = 0
    ptc2.tdr1().write(|w| w.set_data(0));
    //    TCR = trigsel(105=BT_PKTDET) | op(OR) | trigpol(0=rising edge)
    ptc2.tcr1().write(|w| {
        w.set_trigsel(PTC_LCPU_BT_PKTDET);
        w.set_op(PtcOp::Or);
        w.set_trigpol(false);
    });

    // 4. Clear pending interrupts
    ptc2.icr().write(|w| {
        w.set_ctcif1(true);
        w.set_cteif(true);
    });

    // 5. Enable task 1 completion + transfer error interrupts
    ptc2.ier().modify(|w| {
        w.set_tcie1(true);
        w.set_teie(true);
    });

    debug!(
        "CFO tracking configured: PTC2 task1, trigger=BT_PKTDET({}), target=0x{:08X}",
        PTC_LCPU_BT_PKTDET, CFO_PHASE_ADDR
    );
}

/// Disable BLE controller sleep.
///
/// SDK equivalent: `bt_sleep_control(0)` which writes `LPSYS_AON.RESERVE0 = 1`.
///
/// Without `ble_standby_sleep_after_handler()` (which reconfigures MAC clock
/// and PTC after every wakeup), sleep causes MAC clock loss and subsequently
/// 0x3E Connection Timeout disconnects.
fn disable_ble_sleep() {
    crate::pac::LPSYS_AON.reserve0().write(|w| w.set_data(1));
    debug!("BLE sleep disabled (LPSYS_AON.RESERVE0=1)");
}
