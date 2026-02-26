//! Post-boot BLE controller runtime initialization.
//!
//! SDK equivalent: `bluetooth_init()` in
//! `middleware/bluetooth/service/bluetooth.c:563`.
//!
//! Called after LCPU boot + warmup event. Configures:
//! - Sleep timing parameters (`ble_xtal_less_init()`, `bluetooth.c:79`)
//! - MAC baseband clock (`HAL_RCC_SetMacFreq()`, `bluetooth.c:652`)
//! - CFO phase tracking via PTC2 (`rf_ptc_config()`, `bluetooth_misc.c:356`)
//! - BLE power management: idle hook + LP ref cycle (`bluetooth_pm_init()`, `bluetooth.c:333`)
//!
//! ## SDK `bluetooth_pm_init()` coverage (`bluetooth.c:333`)
//!
//! | SDK operation                        | Status      | Notes                                    |
//! |--------------------------------------|-------------|------------------------------------------|
//! | `rt_pm_device_register()`            | N/A         | No OS PM framework on bare-metal         |
//! | `rt_thread_idle_sethook()`           | Implemented | `register_idle_hook()` — direct RAM      |
//! | `rt_pm_override_mode_select()`       | TODO        | Multi-level sleep mode selection          |
//! | `bluetooth_pm_suspend()` / resume    | TODO        | Sleep gate check before system deep-sleep |

use super::config::ControllerConfig;
use super::rom_config::BtRomConfig;
use sifli_hal::syscfg::ChipRevision;

/// LCPU ROM runtime variable addresses.
mod addr {
    pub const RWIP_PROG_DELAY_A3: *mut u8 = crate::memory_map::a3_rom::RWIP_PROG_DELAY as _;
    pub const G_ROM_CONFIG_A3: usize = crate::memory_map::a3_rom::G_ROM_CONFIG;
}

const PTC_LCPU_BT_PKTDET: u8 = 105;
const CFO_PHASE_ADDR: u32 = crate::memory_map::rf::CFO_PHASE_ADDR;
const CFO_PHASE_SIZE: usize = crate::memory_map::rf::CFO_PHASE_SIZE;

use crate::pac::ptc::vals::Op as PtcOp;

/// Initialize BLE controller after boot.
///
/// SDK: `bluetooth_init()` in `bluetooth.c:563`.
/// Called after `ble_boot()` / warmup event consumption.
pub(crate) fn init(rev: ChipRevision, config: &ControllerConfig) {
    configure_sleep_timing(rev, config);
    configure_mac_clock();
    setup_cfo_tracking();

    if config.pm_enabled {
        pm_init(rev, config);
    } else {
        disable_ble_sleep();
    }
}

/// Initialize BLE power management.
///
/// SDK: `bluetooth_pm_init()` in `bluetooth.c:333`.
///
/// Enables LCPU low-power sleep between radio events:
/// - Opens sleep gate (`LPSYS_AON.RESERVE0 = 0`)
/// - Registers `_rwip_sleep` as idle hook so ROM idle loop calls sleep scheduling
/// - Writes LP clock reference cycle to fix ROM division-by-zero bug
///
/// TODO: SDK also does:
/// - `rt_pm_device_register()` — `bluetooth.c:340`
///   Registers `bluetooth_pm_suspend()` callback so PM framework can ask
///   "is BLE stack ready to sleep?" before entering system-level sleep.
/// - `rt_pm_override_mode_select()` — `bluetooth.c:343`
///   Selects IDLE/DEEP/STANDBY based on tick + `bluetooth_stack_suspend()` result.
///   Needed for HCPU-side deep sleep coordination.
fn pm_init(rev: ChipRevision, config: &ControllerConfig) {
    enable_ble_sleep();
    register_idle_hook(rev);
    configure_lp_ref_cycle(rev, config);
}

/// Configure sleep timing parameters in LCPU shared memory.
///
/// SDK: `ble_xtal_less_init()` in `bluetooth.c:79`.
/// - A3: writes `rwip_prog_delay` + `g_rom_config` fields directly in RAM.
///   Uses `rom_config_set_lld_prog_delay()`, `rom_config_set_default_*()`.
/// - Letter Series: already configured via `RomControlBlock` before boot.
fn configure_sleep_timing(rev: ChipRevision, config: &ControllerConfig) {
    if let ChipRevision::A3OrEarlier(_) = rev {
        // SDK: `rom_config_set_lld_prog_delay(3)` — bluetooth.c:97
        unsafe {
            core::ptr::write_volatile(addr::RWIP_PROG_DELAY_A3, config.lld_prog_delay);
        }

        // SDK: `rom_config_set_default_*()` series — bluetooth.c:86-97
        // addr::G_ROM_CONFIG_A3 corresponds to SDK global `g_rom_config`
        let bt_rom_cfg = sifli_hal::ram::RamSlice::new(
            addr::G_ROM_CONFIG_A3,
            core::mem::size_of::<BtRomConfig>(),
        );
        BtRomConfig::from_controller(config).apply(&bt_rom_cfg);
    }
}

/// Configure BLE MAC baseband clock to 8 MHz.
///
/// SDK: `HAL_RCC_SetMacFreq()` in `drivers/hal/bf0_hal_rcc.c:2047`.
/// Called at `bluetooth_init()` end — `bluetooth.c:652`.
/// MAC clock = LPSYS_HCLK / mac_div, target 8 MHz.
fn configure_mac_clock() {
    use crate::rcc;
    let hclk = rcc::get_lpsys_hclk_freq().unwrap_or(sifli_hal::time::Hertz(24_000_000));
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
}

/// Configure PTC2 for CFO (Carrier Frequency Offset) phase tracking.
///
/// SDK: `rf_ptc_config(1)` in `bluetooth_misc.c:356`, called at `bluetooth_init()` — `bluetooth.c:601`.
///
/// Flow: BLE packet detected → `PTC_LCPU_BT_PKTDET` trigger → PTC2 fires →
///       `PTC2_IRQHandler()` (`bluetooth_misc.c:318`) → `ptc_save_phase()` (`bluetooth_misc.c:284`)
///       reads `BT_PHY.RX_STATUS1.CFO_PHASE` and stores to `cfo_phase_t` at 0x40082790.
///
/// We configure PTC2 channel 1 with target address = `cfo_phase_t*`, operation = OR,
/// matching `ptc_config(0, PTC_LCPU_BT_PKTDET, 0, 0)` in `bluetooth_misc.c:365`.
fn setup_cfo_tracking() {
    let ptc2 = crate::pac::PTC2;

    // SDK: `HAL_RCC_EnableModule(RCC_MOD_PTC2)` — bluetooth_misc.c:363
    crate::pac::LPSYS_RCC.enr1().modify(|w| w.set_ptc2(true));

    // SDK: `memset((void *)pt_cfo, 0, 10)` — bluetooth_misc.c:361
    sifli_hal::ram::RamSlice::new(CFO_PHASE_ADDR as usize, CFO_PHASE_SIZE).clear();

    // SDK: `ptc_config(0, PTC_LCPU_BT_PKTDET, 0, 0)` — bluetooth_misc.c:335-355
    // Sets PTC2 channel: target=&cfo_phase[0], data=0, op=OR, trigger=BT_PKTDET
    ptc2.tar1().write(|w| w.set_addr(CFO_PHASE_ADDR));
    ptc2.tdr1().write(|w| w.set_data(0));
    ptc2.tcr1().write(|w| {
        w.set_trigsel(PTC_LCPU_BT_PKTDET);
        w.set_op(PtcOp::Or);
        w.set_trigpol(false);
    });

    ptc2.icr().write(|w| {
        w.set_ctcif1(true);
        w.set_cteif(true);
    });

    ptc2.ier().modify(|w| {
        w.set_tcie1(true);
        w.set_teie(true);
    });
}

/// Enable BLE controller sleep (open sleep gate).
///
/// SDK: `LPSYS_AON.RESERVE0 = 0` — used as sleep gate by ROM's `_rwip_sleep()`.
pub(crate) fn enable_ble_sleep() {
    crate::pac::LPSYS_AON.reserve0().write(|w| w.set_data(0));
}

/// Disable BLE controller sleep (close sleep gate).
pub(crate) fn disable_ble_sleep() {
    crate::pac::LPSYS_AON.reserve0().write(|w| w.set_data(1));
}

/// Register `_rwip_sleep` as idle hook to enable LCPU low-power sleep.
///
/// SDK: `bluetooth_pm_init()` in `bluetooth.c:326` calls
/// `rt_thread_idle_sethook(bluetooth_idle_hook_func)` — `bluetooth.c:341`.
///
/// In the SDK, `bluetooth_idle_hook_func` is a `__WEAK` empty stub (`bluetooth.c:280`).
/// On bare-metal (no RT-Thread), we directly replace `idle_hook_list[1]`
/// (which ROM initializes to `bluetooth_idle_hook_func`) with the real
/// `_rwip_sleep` function pointer, so the ROM idle loop calls sleep scheduling.
fn register_idle_hook(rev: ChipRevision) {
    use crate::memory_map::letter_rom;

    if let ChipRevision::A3OrEarlier(_) = rev {
        return;
    }

    // Verify ROM version by checking known function pointer
    let rwip_sleep = unsafe {
        core::ptr::read_volatile(letter_rom::RWIP_SLEEP_HANDLER as *const u32)
    };
    if rwip_sleep != letter_rom::FINGERPRINT {
        warn!(
            "Unknown ROM version (rwip_sleep_handler={:#010x}, expected {:#010x}), \
             skipping idle hook registration",
            rwip_sleep, letter_rom::FINGERPRINT
        );
        return;
    }

    // idle_hook_list[1] should currently point to bluetooth_idle_hook_func (empty stub)
    let hook_slot = (letter_rom::IDLE_HOOK_LIST + 4) as *mut u32;
    let current = unsafe { core::ptr::read_volatile(hook_slot) };

    if current != letter_rom::BLUETOOTH_IDLE_HOOK_FUNC {
        warn!(
            "idle_hook_list[1] = {:#010x}, expected {:#010x} (bluetooth_idle_hook_func), \
             skipping override",
            current, letter_rom::BLUETOOTH_IDLE_HOOK_FUNC
        );
        return;
    }

    // Replace empty stub with _rwip_sleep so idle loop actually enters sleep
    let rwip_sleep_fn = unsafe {
        core::ptr::read_volatile(letter_rom::RWIP_SLEEP_HANDLER as *const u32)
    };

    unsafe {
        core::ptr::write_volatile(hook_slot, rwip_sleep_fn);
    }

    info!("BLE idle hook: _rwip_sleep registered");
}

/// Write LP clock reference cycle to `rwip_env.lp_ref_cycle`.
///
/// Fixes a ROM bug: `_rwip_sleep()` computes LP sleep cycles as
/// `lp_cycles = (delta_hs * rc_cycle * 15000) / lp_ref_cycle`.
/// If `lp_ref_cycle == 0` (uninitialized), division by zero → result 0 →
/// subtract 17 → negative → below minimum sleep threshold → never sleeps.
///
/// SDK: `HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_LP_CYCLE, ...)` writes
/// `lp_ref_cycle` to ROM config area. ROM's `HAL_LCPU_CONFIG_get(2)` reads it
/// into `rwip_env+0x1d8`. When the value is 0, ROM skips the write.
/// We write directly to `rwip_env+0x1d8` to bypass the ROM config indirection.
fn configure_lp_ref_cycle(rev: ChipRevision, config: &ControllerConfig) {
    use crate::memory_map::letter_rom;

    if let ChipRevision::A3OrEarlier(_) = rev {
        return;
    }

    let lp_ref_addr =
        (letter_rom::RWIP_ENV + letter_rom::RWIP_ENV_LP_REF_CYCLE_OFFSET) as *mut u32;

    let current = unsafe { core::ptr::read_volatile(lp_ref_addr) };
    if current != 0 {
        return;
    }

    // Formula: lp_ref_cycle = rc_cycle * 15000 * 100 / 1024
    // Matches ROM's expected unit for LP clock period calculation
    let lp_ref_cycle = (config.rc_cycle as u32 * 15000 * 100) / 1024;
    unsafe {
        core::ptr::write_volatile(lp_ref_addr, lp_ref_cycle);
    }

    info!("LP ref cycle: {} (rc_cycle={})", lp_ref_cycle, config.rc_cycle);
}
