//! LCPU ROM configuration block write logic.
//!
//! Writes boot-time parameters to the LCPU ROM configuration area in shared memory.
//! Uses `RamSlice` + generated SDK offsets from `rom_config_layout.toml`.

use super::config::{ControllerConfig, RomConfig};
use crate::memory_map::rom_config;
use crate::memory_map::rom_config::bt::valid;
pub(crate) use crate::memory_map::rom_config::bt::BtRomConfig;
use sifli_hal::syscfg::ChipRevision;

impl BtRomConfig {
    /// Construct BtRomConfig from ControllerConfig.
    ///
    /// Single source of truth for BLE controller parameters.
    /// Used by both Letter (pre-boot) and A3 (post-boot) paths.
    pub(crate) fn from_controller(config: &ControllerConfig) -> Self {
        Self {
            bit_valid: valid::CONTROLLER_ENABLE_BIT
                | valid::LLD_PROG_DELAY
                | valid::DEFAULT_SLEEP_MODE
                | valid::DEFAULT_SLEEP_ENABLED
                | valid::DEFAULT_XTAL_ENABLED
                | valid::DEFAULT_RC_CYCLE,
            controller_enable_bit: 0x03,
            lld_prog_delay: config.lld_prog_delay,
            default_sleep_mode: 0,
            default_sleep_enabled: config.pm_enabled as u8,
            default_xtal_enabled: config.xtal_enabled as u8,
            default_rc_cycle: config.rc_cycle,
            ..Default::default()
        }
    }

    /// Write fields to an existing BtRomConfig in shared memory.
    ///
    /// Writes each field individually with `write_volatile` for fine-grained
    /// shared memory access. OR's `bit_valid` to preserve existing flags.
    pub(crate) fn apply(&self, region: &sifli_hal::ram::RamSlice) {
        region.write(rom_config::bt::CONTROLLER_ENABLE_BIT, self.controller_enable_bit);
        region.write(rom_config::bt::LLD_PROG_DELAY, self.lld_prog_delay);
        region.write(rom_config::bt::DEFAULT_SLEEP_MODE, self.default_sleep_mode);
        region.write(rom_config::bt::DEFAULT_SLEEP_ENABLED, self.default_sleep_enabled);
        region.write(rom_config::bt::DEFAULT_XTAL_ENABLED, self.default_xtal_enabled);
        region.write(rom_config::bt::DEFAULT_RC_CYCLE, self.default_rc_cycle);

        let old_valid: u32 = region.read(rom_config::bt::BIT_VALID);
        region.write(rom_config::bt::BIT_VALID, old_valid | self.bit_valid);
    }
}

/// Get ROM config base address and size for the given chip revision.
fn config_base_and_size(rev: ChipRevision) -> (usize, usize) {
    match rev {
        ChipRevision::A3OrEarlier(_) => {
            (crate::memory_map::a3::ROM_CONFIG_BASE, rom_config::SIZE_A3)
        }
        _ => (crate::memory_map::letter::ROM_CONFIG_BASE, rom_config::SIZE_LETTER),
    }
}

/// Write ROM configuration to LCPU shared memory.
pub(crate) fn init(rev: ChipRevision, config: &RomConfig, ctrl: &ControllerConfig) {
    let (base, size) = config_base_and_size(rev);
    let rom_cfg = sifli_hal::ram::RamSlice::new(base, size);
    rom_cfg.clear();

    rom_cfg.write(rom_config::MAGIC, rom_config::MAGIC_VALUE);
    rom_cfg.write(rom_config::IS_XTAL_ENABLE, config.enable_lxt as u8);
    rom_cfg.write(rom_config::IS_RCCAL_IN_L, (!config.enable_lxt) as u8);
    rom_cfg.write(rom_config::WDT_STATUS, 0xFFu32);
    rom_cfg.write(rom_config::WDT_TIME, config.wdt_time);
    rom_cfg.write(rom_config::WDT_CLK, config.wdt_clk);

    if rev.is_letter_series() {
        rom_cfg.write(
            rom_config::HCPU_IPC_ADDR,
            crate::memory_map::shared::HCPU2LCPU_MB_CH1 as u32,
        );

        let mut bt_cfg = BtRomConfig::from_controller(ctrl);
        bt_cfg.bit_valid |= valid::IS_FPGA;
        rom_cfg.write(rom_config::BT_ROM_CONFIG, bt_cfg);

        if let Some(ref em) = config.em_config {
            rom_cfg.write(rom_config::EM_BUF, *em);
        }
        if let Some(ref act) = config.act_config {
            rom_cfg.write(rom_config::BT_ACT_CONFIG, *act);
        }
    }
}

/// Write BT TX power parameters to LCPU ROM configuration area.
pub(crate) fn set_bt_tx_power(rev: ChipRevision, tx_pwr: u32) {
    let (base, size) = config_base_and_size(rev);
    sifli_hal::ram::RamSlice::new(base, size).write(rom_config::BT_TXPWR, tx_pwr);
}
