//! BLE configuration types.
//!
//! User-facing configuration via [`BleInitConfig`] builder pattern.
//! Advanced users can customize [`EmConfig`] and [`ActConfig`] for
//! Exchange Memory layout and activity limits.

use crate::memory_map::rom_config::act::valid as act_valid;

/// BLE controller runtime parameters.
///
/// Applied after LCPU boot to configure BLE scheduling and timing.
/// Not user-facing — configured via [`BleInitConfig`] builders.
#[derive(Debug, Clone, Copy)]
pub struct ControllerConfig {
    /// Link Layer Driver programming delay (625us slots). SDK default: 3.
    pub(crate) lld_prog_delay: u8,
    /// Whether LXT is used for BLE sleep timing. Follows `RomConfig::enable_lxt`.
    pub(crate) xtal_enabled: bool,
    /// RC oscillator cycle count for BLE sleep timing. SDK default: 20.
    pub(crate) rc_cycle: u8,
    /// Enable BLE power management (idle hook, sleep scheduling, LP clock).
    pub(crate) pm_enabled: bool,
}

impl ControllerConfig {
    pub(crate) const fn new() -> Self {
        Self {
            lld_prog_delay: 3,
            xtal_enabled: true, // matches RomConfig.enable_lxt default
            rc_cycle: 20,
            pm_enabled: false,
        }
    }
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// ROM boot parameters written to LCPU shared memory before boot.
/// Not user-facing — configured via [`BleInitConfig`] builders.
#[derive(Debug, Clone, Copy)]
pub struct RomConfig {
    /// Watchdog timeout (in seconds, default 10).
    pub(crate) wdt_time: u32,
    /// Watchdog clock frequency (Hz). Always 32768.
    pub(crate) wdt_clk: u16,
    /// Enable external low-speed crystal (default true).
    pub(crate) enable_lxt: bool,
    /// BLE Exchange Memory buffer layout (Letter Series only).
    pub(crate) em_config: Option<EmConfig>,
    /// BLE/BT activity configuration (Letter Series only).
    pub(crate) act_config: Option<ActConfig>,
}

impl RomConfig {
    pub(crate) const fn new() -> Self {
        Self {
            wdt_time: 10,
            wdt_clk: 32_768,
            enable_lxt: true,
            em_config: Some(EmConfig::DEFAULT),
            act_config: Some(ActConfig::DEFAULT),
        }
    }
}

impl Default for RomConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// BLE Exchange Memory buffer configuration.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct EmConfig {
    /// Validity flag (1 = valid, 0 = use ROM defaults).
    pub is_valid: u8,
    /// EM buffer offset table (up to 40 entries).
    pub em_buf: [u16; Self::MAX_NUM],
}

impl EmConfig {
    pub const MAX_NUM: usize = 40;

    pub const DEFAULT: Self = Self {
        is_valid: 1,
        em_buf: [
            0x178, 0x178, 0x740, 0x7A0, 0x810, 0x880, 0xA00, 0xBB0, 0xD48, 0x133C, 0x13A4, 0x19BC,
            0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x263C, 0x265C, 0x2734,
            0x2784, 0x28D4, 0x28E8, 0x28FC, 0x29EC, 0x29FC, 0x2BBC, 0x2BD8, 0x3BE8, 0x5804, 0x5804,
            0x5804, 0, 0, 0, 0, 0,
        ],
    };
}

impl Default for EmConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

pub use crate::memory_map::rom_config::act::ActConfig;

impl ActConfig {
    pub const DEFAULT: Self = Self {
        bit_valid: act_valid::BT_MAX_ACL
            | act_valid::BT_MAX_SCO
            | act_valid::BLE_MAX_ACT
            | act_valid::BLE_MAX_RAL
            | act_valid::BLE_MAX_ISO,
        bt_max_acl: 7,
        bt_max_sco: 0,
        ble_max_act: 6,
        ble_max_ral: 3,
        ble_max_iso: 0,
        ble_rx_desc: 0,
        bt_rx_desc: 0,
        bt_name_len: 0,
    };
}

impl Default for ActConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// BLE-specific configuration (post-boot controller params + BD address).
/// Not user-facing — configured via [`BleInitConfig`] builders.
#[derive(Debug, Clone, Copy)]
pub struct BleConfig {
    pub(crate) controller: ControllerConfig,
    pub(crate) bd_addr: [u8; 6],
}

impl BleConfig {
    pub(crate) const fn new() -> Self {
        Self {
            controller: ControllerConfig::new(),
            bd_addr: [0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD],
        }
    }
}

impl Default for BleConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Patch data (entry list + code binary).
#[derive(Debug, Clone, Copy)]
pub(crate) struct PatchData {
    pub(crate) list: &'static [u8],
    pub(crate) bin: &'static [u8],
}

/// Firmware binary data for LCPU.
#[cfg(feature = "sf32lb52x-lcpu")]
mod sf32lb52x_lcpu_data {
    pub const FIRMWARE: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/lcpu_firmware.bin");
    pub const PATCH_A3_LIST: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/patch_a3_list.bin");
    pub const PATCH_A3_BIN: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/patch_a3_bin.bin");
    pub const PATCH_LETTER_LIST: &[u8] =
        include_bytes!("../../data/sf32lb52x/lcpu/patch_letter_list.bin");
    pub const PATCH_LETTER_BIN: &[u8] =
        include_bytes!("../../data/sf32lb52x/lcpu/patch_letter_bin.bin");
}

/// Complete BLE initialization configuration.
///
/// Use builder methods to customize. Internal fields are not directly accessible.
///
/// # Example
///
/// ```ignore
/// use sifli_radio::bluetooth::BleInitConfig;
/// let config = BleInitConfig::default().pm_enabled(true);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct BleInitConfig {
    pub(crate) rom: RomConfig,
    pub(crate) ble: BleConfig,
    pub(crate) patch_a3: Option<PatchData>,
    pub(crate) patch_letter: Option<PatchData>,
    pub(crate) firmware: Option<&'static [u8]>,
    pub(crate) disable_rf_cal: bool,
    pub(crate) skip_frequency_check: bool,
}

impl BleInitConfig {
    /// Create a new config with defaults (no firmware/patch data).
    pub const fn new() -> Self {
        Self {
            rom: RomConfig::new(),
            ble: BleConfig::new(),
            patch_a3: None,
            patch_letter: None,
            firmware: None,
            disable_rf_cal: false,
            skip_frequency_check: false,
        }
    }

    /// Set public BD address.
    pub const fn bd_addr(mut self, addr: [u8; 6]) -> Self {
        self.ble.bd_addr = addr;
        self
    }

    /// Enable or disable BLE power management.
    pub const fn pm_enabled(mut self, enabled: bool) -> Self {
        self.ble.controller.pm_enabled = enabled;
        self
    }

    /// Enable or disable external low-speed crystal (LXT).
    ///
    /// When enabled (default), LXT is used for BLE sleep timing (more accurate).
    /// When disabled, RC oscillator is used instead.
    /// Depends on board hardware — set to `false` if no 32kHz crystal is present.
    pub const fn enable_lxt(mut self, enable: bool) -> Self {
        self.rom.enable_lxt = enable;
        self.ble.controller.xtal_enabled = enable;
        self
    }

    /// Set LCPU watchdog timeout in seconds (default 10).
    pub const fn wdt_time(mut self, seconds: u32) -> Self {
        self.rom.wdt_time = seconds;
        self
    }

    /// Disable RF calibration (for faster startup during development).
    pub const fn disable_rf_cal(mut self, disable: bool) -> Self {
        self.disable_rf_cal = disable;
        self
    }

    /// Skip LPSYS HCLK frequency check during firmware loading.
    pub const fn skip_frequency_check(mut self, skip: bool) -> Self {
        self.skip_frequency_check = skip;
        self
    }

    /// Set BLE Exchange Memory buffer configuration (Letter Series only).
    pub const fn em_config(mut self, config: EmConfig) -> Self {
        self.rom.em_config = Some(config);
        self
    }

    /// Set BLE/BT activity configuration (Letter Series only).
    pub const fn act_config(mut self, config: ActConfig) -> Self {
        self.rom.act_config = Some(config);
        self
    }
}

impl Default for BleInitConfig {
    fn default() -> Self {
        #[allow(unused_mut)]
        let mut cfg = Self::new();

        #[cfg(feature = "sf32lb52x-lcpu")]
        {
            cfg.firmware = Some(sf32lb52x_lcpu_data::FIRMWARE);
            cfg.patch_a3 = Some(PatchData {
                list: sf32lb52x_lcpu_data::PATCH_A3_LIST,
                bin: sf32lb52x_lcpu_data::PATCH_A3_BIN,
            });
            cfg.patch_letter = Some(PatchData {
                list: sf32lb52x_lcpu_data::PATCH_LETTER_LIST,
                bin: sf32lb52x_lcpu_data::PATCH_LETTER_BIN,
            });
        }

        cfg
    }
}
