//! BLE configuration types.
//!
//! User-facing configuration for ROM parameters, Exchange Memory layout,
//! BLE/BT activity limits, and controller runtime parameters.

/// BLE controller runtime parameters.
///
/// Applied after LCPU boot to configure BLE scheduling and timing.
#[derive(Debug, Clone, Copy)]
pub struct ControllerConfig {
    /// Link Layer Driver programming delay (625us slots).
    pub lld_prog_delay: u8,
    /// Whether external 32kHz crystal (LXT) is enabled for BLE sleep timing.
    pub xtal_enabled: bool,
    /// RC oscillator cycle count for BLE sleep timing.
    pub rc_cycle: u8,
    /// Enable BLE controller sleep between radio events.
    pub sleep_enabled: bool,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            lld_prog_delay: 3,
            xtal_enabled: false,
            rc_cycle: 20,
            sleep_enabled: false,
        }
    }
}

/// User-configurable ROM parameters.
#[derive(Debug, Clone, Copy)]
pub struct RomConfig {
    /// Watchdog timeout (in seconds, default 10).
    pub wdt_time: u32,
    /// Watchdog clock frequency (Hz, default 32768).
    pub wdt_clk: u16,
    /// Enable external low-speed crystal (default true).
    pub enable_lxt: bool,
    /// BLE Exchange Memory buffer layout (Letter Series only).
    pub em_config: Option<EmConfig>,
    /// BLE/BT activity configuration (Letter Series only).
    pub act_config: Option<ActConfig>,
}

impl Default for RomConfig {
    fn default() -> Self {
        Self {
            wdt_time: 10,
            wdt_clk: 32_768,
            enable_lxt: true,
            em_config: Some(EmConfig::DEFAULT),
            act_config: Some(ActConfig::DEFAULT),
        }
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
    pub(crate) const ROM_OFFSET: usize = 32;

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

/// BLE/BT activity configuration.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ActConfig {
    pub bit_valid: u32,
    pub bt_max_acl: u8,
    pub bt_max_sco: u8,
    pub ble_max_act: u8,
    pub ble_max_ral: u8,
    pub ble_max_iso: u8,
    pub ble_rx_desc: u8,
    pub bt_rx_desc: u8,
    pub bt_name_len: u8,
}

impl ActConfig {
    pub(crate) const ROM_OFFSET: usize = 116;

    pub const DEFAULT: Self = Self {
        bit_valid: (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4),
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
#[derive(Debug, Clone, Copy)]
pub struct BleConfig {
    /// BLE controller runtime parameters.
    pub controller: ControllerConfig,
    /// Public BD address.
    pub bd_addr: [u8; 6],
}

impl BleConfig {
    pub const fn new() -> Self {
        Self {
            controller: ControllerConfig {
                lld_prog_delay: 3,
                xtal_enabled: false,
                rc_cycle: 20,
                sleep_enabled: false,
            },
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
pub struct PatchData {
    /// Patch entry list array.
    pub list: &'static [u8],
    /// Patch code bytes.
    pub bin: &'static [u8],
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
/// # Example
///
/// ```ignore
/// use sifli_radio::bluetooth::BleInitConfig;
/// let config = BleInitConfig::default().sleep_enabled(true);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct BleInitConfig {
    /// ROM parameters.
    pub rom: RomConfig,
    /// BLE controller runtime parameters + BD address.
    pub ble: BleConfig,
    /// Patch data for A3 and earlier.
    pub patch_a3: Option<PatchData>,
    /// Patch data for Letter Series (A4/B4).
    pub patch_letter: Option<PatchData>,
    /// LCPU firmware image bytes (needed for A3 and earlier).
    pub firmware: Option<&'static [u8]>,
    /// Disable RF calibration.
    pub disable_rf_cal: bool,
    /// Skip LPSYS HCLK frequency check during image loading.
    pub skip_frequency_check: bool,
}

impl BleInitConfig {
    /// Create a new config with all options unset.
    pub const fn new() -> Self {
        Self {
            rom: RomConfig {
                wdt_time: 10,
                wdt_clk: 32_768,
                enable_lxt: true,
                em_config: Some(EmConfig::DEFAULT),
                act_config: Some(ActConfig::DEFAULT),
            },
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

    /// Enable or disable BLE controller sleep between radio events.
    pub const fn sleep_enabled(mut self, enabled: bool) -> Self {
        self.ble.controller.sleep_enabled = enabled;
        self
    }

    /// Disable RF calibration.
    pub const fn disable_rf_cal(mut self, disable: bool) -> Self {
        self.disable_rf_cal = disable;
        self
    }

    /// Skip LPSYS HCLK frequency check during image loading.
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
