//! LCPU ROM configuration block write logic.
//!
//! Writes boot-time parameters to the LCPU ROM configuration area in shared memory.

use super::config::{ActConfig, ControllerConfig, EmConfig, RomConfig};
use crate::syscfg;
use core::{mem, ptr};

/// LCPU ROM Configuration Block Layout.
#[repr(C)]
struct RomControlBlock {
    magic: u32,
    lpcycle_curr: u32,
    lpcycle_ave: u32,
    wdt_time: u32,
    wdt_status: u32,
    _pad1: [u8; 4],
    wdt_clk: u16,
    is_xtal_enable: u8,
    is_rccal_in_l: u8,
    _pad2: [u8; 144],
    bt_config: BtRomConfig,
    _pad3: [u8; 7],
    hcpu_ipc_addr: u32,
}

/// BT/BLE specific configuration (A4+).
#[repr(C)]
#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct BtRomConfig {
    pub(crate) bit_valid: u32,
    pub(crate) max_sleep_time: u32,
    pub(crate) controller_enable_bit: u8,
    pub(crate) lld_prog_delay: u8,
    pub(crate) lld_prog_delay_min: u8,
    pub(crate) default_sleep_mode: u8,
    pub(crate) default_sleep_enabled: u8,
    pub(crate) default_xtal_enabled: u8,
    pub(crate) default_rc_cycle: u8,
    pub(crate) default_swprofiling_cfg: u8,
    pub(crate) boot_mode: u8,
    pub(crate) is_fpga: u8,
    pub(crate) en_inq_filter: u8,
    pub(crate) support_3m: u8,
    pub(crate) sco_cfg: u8,
}

impl RomControlBlock {
    const ADDR_A3: usize = crate::memory_map::a3::ROM_CONFIG_BASE;
    const ADDR_LETTER: usize = crate::memory_map::letter::ROM_CONFIG_BASE;
    const HCPU2LCPU_MB_CH1_BUF_START_ADDR: usize = crate::memory_map::shared::HCPU2LCPU_MB_CH1;
    const MAGIC: u32 = 0x4545_7878;

    fn address() -> usize {
        if syscfg::read_idr().revision().is_letter_series() {
            Self::ADDR_LETTER
        } else {
            Self::ADDR_A3
        }
    }
}

/// Write ROM configuration to LCPU shared memory.
pub(crate) fn write(config: &RomConfig, ctrl: &ControllerConfig) {
    let base = RomControlBlock::address();
    let is_letter = syscfg::read_idr().revision().is_letter_series();

    let size = if is_letter {
        mem::size_of::<RomControlBlock>()
    } else {
        0x40
    };

    debug!(
        "ROM config: base=0x{:08X}, size={}, letter={}",
        base, size, is_letter
    );

    unsafe {
        ptr::write_bytes(base as *mut u8, 0, size);
        let block = &mut *(base as *mut RomControlBlock);

        ptr::write_volatile(&mut block.magic, RomControlBlock::MAGIC);
        ptr::write_volatile(&mut block.is_xtal_enable, config.enable_lxt as u8);
        ptr::write_volatile(&mut block.is_rccal_in_l, (!config.enable_lxt) as u8);
        ptr::write_volatile(&mut block.wdt_status, 0xFF);
        ptr::write_volatile(&mut block.wdt_time, config.wdt_time);
        ptr::write_volatile(&mut block.wdt_clk, config.wdt_clk);

        if is_letter {
            ptr::write_volatile(
                &mut block.hcpu_ipc_addr,
                RomControlBlock::HCPU2LCPU_MB_CH1_BUF_START_ADDR as u32,
            );

            let bt_cfg = BtRomConfig {
                bit_valid: (1 << 10) | (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 2) | (1 << 1),
                controller_enable_bit: 0x03,
                lld_prog_delay: ctrl.lld_prog_delay,
                default_sleep_mode: 0,
                default_sleep_enabled: ctrl.sleep_enabled as u8,
                default_xtal_enabled: ctrl.xtal_enabled as u8,
                default_rc_cycle: ctrl.rc_cycle,
                is_fpga: 0,
                ..Default::default()
            };
            ptr::write_volatile(&mut block.bt_config, bt_cfg);

            if let Some(ref em) = config.em_config {
                let dst = (base + EmConfig::ROM_OFFSET) as *mut EmConfig;
                ptr::write_volatile(dst, *em);
            }
            if let Some(ref act) = config.act_config {
                let dst = (base + ActConfig::ROM_OFFSET) as *mut ActConfig;
                ptr::write_volatile(dst, *act);
            }
        }
    }
}

/// Write BT TX power parameters to LCPU ROM configuration area.
pub(crate) fn set_bt_tx_power(tx_pwr: u32) {
    const BT_TXPWR_OFFSET: usize = 20;
    let base = RomControlBlock::address();
    let addr = base + BT_TXPWR_OFFSET;
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, tx_pwr);
    }
}
