use crate::pac::hpsys_cfg::regs::Ulpmcr;
use crate::pac::lpsys_cfg::regs::Ulpmcr as LpUlpmcr;
use crate::pac::{HPSYS_CFG, LPSYS_CFG, PMUC};
use crate::time::Hertz;

// Constants for DVFS mode limits
pub const HPSYS_DVFS_MODE_D0_LIMIT: u32 = 24;
pub const HPSYS_DVFS_MODE_D1_LIMIT: u32 = 48;
pub const HPSYS_DVFS_MODE_S0_LIMIT: u32 = 144;
pub const HPSYS_DVFS_MODE_S1_LIMIT: u32 = 240;

const fn build_ulpmcr(ram_rm: u8, ram_ra: u8, ram_wa: u8, rom_rm: u8) -> Ulpmcr {
    let mut v = Ulpmcr(0);
    v.set_ram_rm(ram_rm);
    v.set_ram_rme(true);
    v.set_ram_ra(ram_ra);
    v.set_ram_wa(ram_wa);
    v.set_rom_rm(rom_rm);
    v.set_rom_rme(true);
    v
}

const fn build_lp_ulpmcr(ram_rm: u8, ram_ra: u8, ram_wa: u8, rom_rm: u8) -> LpUlpmcr {
    let mut v = LpUlpmcr(0);
    v.set_ram_rm(ram_rm);
    v.set_ram_rme(true);
    v.set_ram_ra(ram_ra);
    v.set_ram_wa(ram_wa);
    v.set_rom_rm(rom_rm);
    v.set_rom_rme(true);
    v
}

pub const HPSYS_DVFS_CONFIG: [HpsysDvfsConfig; 4] = [
    // LDO: 0.9V, BUCK: 1.0V
    HpsysDvfsConfig { ldo_offset: -5, ldo: 0x6, buck: 0x9, ulpmcr: build_ulpmcr(0, 1, 6, 0) },
    // LDO: 1.0V, BUCK: 1.1V
    HpsysDvfsConfig { ldo_offset: -3, ldo: 0x8, buck: 0xA, ulpmcr: build_ulpmcr(1, 1, 6, 1) },
    // LDO: 1.1V, BUCK: 1.25V
    HpsysDvfsConfig { ldo_offset:  0, ldo: 0xB, buck: 0xD, ulpmcr: build_ulpmcr(3, 0, 4, 3) },
    // LDO: 1.2V, BUCK: 1.35V
    HpsysDvfsConfig { ldo_offset:  2, ldo: 0xD, buck: 0xF, ulpmcr: build_ulpmcr(3, 0, 4, 3) },
];

pub const HPSYS_DLL2_LIMIT: [u32; 4] = [
    0,           // D0 Mode
    0,           // D1 Mode
    288_000_000, // S0 Mode
    288_000_000, // S1 Mode
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HpsysDvfsMode {
    D0 = 0,
    D1 = 1,
    S0 = 2,
    S1 = 3,
}

#[cfg(feature = "defmt")]
impl defmt::Format for HpsysDvfsMode {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HpsysDvfsMode::D0 => defmt::write!(f, "D0"),
            HpsysDvfsMode::D1 => defmt::write!(f, "D1"),
            HpsysDvfsMode::S0 => defmt::write!(f, "S0"),
            HpsysDvfsMode::S1 => defmt::write!(f, "S1"),
        }
    }
}

pub fn is_hpsys_dvfs_mode_s() -> bool {
    // HPSYS: LDO_VSEL=0 means S-mode, LDO_VSEL=1 means D-mode
    !HPSYS_CFG.syscr().read().ldo_vsel()
}

impl HpsysDvfsMode {
    pub fn from_frequency(freq_mhz: u32) -> Result<Self, &'static str> {
        match freq_mhz {
            0..=HPSYS_DVFS_MODE_D0_LIMIT => Ok(HpsysDvfsMode::D0),
            25..=HPSYS_DVFS_MODE_D1_LIMIT => Ok(HpsysDvfsMode::D1),
            49..=HPSYS_DVFS_MODE_S0_LIMIT => Ok(HpsysDvfsMode::S0),
            145..=HPSYS_DVFS_MODE_S1_LIMIT => Ok(HpsysDvfsMode::S1),
            _ => Err("Frequency out of valid range"),
        }
    }

    pub fn from_hertz(freq: Hertz) -> Result<Self, &'static str> {
        Self::from_frequency(freq.0 / 1_000_000)
    }

    pub fn get_dll2_limit(self) -> Hertz {
        Hertz(HPSYS_DLL2_LIMIT[self as usize])
    }

    pub fn get_config(self) -> HpsysDvfsConfig {
        HPSYS_DVFS_CONFIG[self as usize]
    }

    pub fn get_frequency_limit(self) -> Hertz {
        Hertz(match self {
            HpsysDvfsMode::D0 => HPSYS_DVFS_MODE_D0_LIMIT,
            HpsysDvfsMode::D1 => HPSYS_DVFS_MODE_D1_LIMIT,
            HpsysDvfsMode::S0 => HPSYS_DVFS_MODE_S0_LIMIT,
            HpsysDvfsMode::S1 => HPSYS_DVFS_MODE_S1_LIMIT,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HpsysDvfsConfig {
    pub ldo_offset: i8,
    pub ldo: u8,
    pub buck: u8,
    pub ulpmcr: Ulpmcr,
}

pub(crate) fn config_hcpu_dvfs<F>(
    current_hclk_freq: Hertz,
    target_hclk_freq: Hertz,
    config_clock: F,
) where
    F: FnOnce(),
{ 
    let current_dvfs_mode = HpsysDvfsMode::from_hertz(current_hclk_freq).unwrap();
    let target_dvfs_mode = HpsysDvfsMode::from_hertz(target_hclk_freq).unwrap();

    use HpsysDvfsMode::*;
    match (current_dvfs_mode, target_dvfs_mode) {
        (D0, D0) | (D1, D1) | (S0, S0) | (S1, S1) => config_clock(),
        (D0, S0) | (D1, S0) | (D1, S1) | (D0, S1) => switch_hcpu_dvfs_d2s(target_dvfs_mode, config_clock),
        (S0, D0) | (S0, D1) | (S1, D0) | (S1, D1) => switch_hcpu_dvfs_s2d(target_dvfs_mode, config_clock),
        (S0, S1) | (S1, S0) => {
            // switch between different S mode
            // TODO: HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);

            config_hcpu_sx_mode_volt(target_dvfs_mode);
            // buck need 250us to settle
            crate::cortex_m_blocking_delay_us(250);
            config_clock();
        },
        (D0, D1) | (D1, D0) => {
            // TODO:
            // switch between different D mode
            // Why?
            // switch_hcpu_dvfs_d2s(target_dvfs_mode, todo_set_to_144);
            switch_hcpu_dvfs_s2d(target_dvfs_mode, config_clock);
        },
    }
}

fn config_hcpu_sx_mode_volt(target_dvfs_mode: HpsysDvfsMode) {
    let dvfs_config = target_dvfs_mode.get_config();

    // configure BUCK voltage
    PMUC.buck_vout().modify(| w| {
        w.set_vout(dvfs_config.buck);
    });

    // configure LDO voltage
    // TODO: use efuse value (HAL_PMU_GetHpsysVoutRef[2])
    let vout_ref = dvfs_config.ldo;
    PMUC.hpsys_vout().modify(| w| {
        w.set_vout(vout_ref);
    });
}

fn switch_hcpu_dvfs_d2s<F>(
    target_dvfs_mode: HpsysDvfsMode,
    config_clock: F,
) where
    F: FnOnce(),
{
    config_hcpu_sx_mode_volt(target_dvfs_mode);
    // Switch to S mode (HPSYS: ldo_vsel=0 selects S-mode voltage rails)
    HPSYS_CFG.syscr().modify(|w| {
        w.set_ldo_vsel(false);
    });

    // buck need 250us to settle
    crate::cortex_m_blocking_delay_us(250);

    // Set memory timing parameters for S-mode before switching to high frequency
    HPSYS_CFG.ulpmcr().write_value(target_dvfs_mode.get_config().ulpmcr);

    config_clock();
}

fn switch_hcpu_dvfs_s2d<F>(
    target_dvfs_mode: HpsysDvfsMode,
    config_clock: F,
) where
    F: FnOnce(),
{
    let dvfs_config = target_dvfs_mode.get_config();

    // configure BUCK voltage
    PMUC.buck_cr2().modify(|w| {
        w.set_set_vout_m(dvfs_config.buck);
    });

    // configure LDO voltage for D mode
    // SDK uses: HAL_PMU_GetHpsysVoutRef() + ldo_offset
    // eFuse default reference is ~0xB (1.1V). TODO: read actual eFuse value.
    let efuse_ref: i8 = 0xB;
    let vref = (efuse_ref + dvfs_config.ldo_offset) as u8;
    PMUC.hpsys_ldo().modify(|w| {
        w.set_vref(vref);
    });

    config_clock();

    // configure memory param
    HPSYS_CFG.ulpmcr().write_value(dvfs_config.ulpmcr);
    // Switch to D mode (HPSYS: ldo_vsel=1 selects D-mode voltage rails)
    HPSYS_CFG.syscr().modify(|w| {
        w.set_ldo_vsel(true);
    });
}

// =============================================================================
// LPSYS DVFS (Low-Power System Dynamic Voltage and Frequency Scaling)
// =============================================================================

/// LPSYS DVFS mode limits (MHz)
pub const LPSYS_DVFS_MODE_D_LIMIT: u32 = 24;
pub const LPSYS_DVFS_MODE_S_LIMIT: u32 = 48;

/// LPSYS DVFS configuration table
pub const LPSYS_DVFS_CONFIG: [LpsysDvfsConfig; 2] = [
    // D Mode: LDO 0.9V, ≤24MHz
    LpsysDvfsConfig {
        ldo: 0x6,
        ulpmcr: build_lp_ulpmcr(0, 1, 6, 2),
    },
    // S Mode: LDO 1.0V, ≤48MHz
    LpsysDvfsConfig {
        ldo: 0x8,
        ulpmcr: build_lp_ulpmcr(1, 1, 6, 2),
    },
];

/// LPSYS DVFS mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpsysDvfsMode {
    /// D Mode: Low power, ≤24MHz
    D = 0,
    /// S Mode: Standard, ≤48MHz
    S = 1,
}

#[cfg(feature = "defmt")]
impl defmt::Format for LpsysDvfsMode {
    fn format(&self, f: defmt::Formatter) {
        match self {
            LpsysDvfsMode::D => defmt::write!(f, "D"),
            LpsysDvfsMode::S => defmt::write!(f, "S"),
        }
    }
}

impl LpsysDvfsMode {
    /// Determine the appropriate DVFS mode for a given frequency (in MHz)
    pub fn from_frequency(freq_mhz: u32) -> Result<Self, &'static str> {
        match freq_mhz {
            0..=LPSYS_DVFS_MODE_D_LIMIT => Ok(LpsysDvfsMode::D),
            25..=LPSYS_DVFS_MODE_S_LIMIT => Ok(LpsysDvfsMode::S),
            _ => Err("Frequency out of valid LPSYS range (max 48MHz)"),
        }
    }

    /// Determine the appropriate DVFS mode for a given frequency (in Hertz)
    pub fn from_hertz(freq: Hertz) -> Result<Self, &'static str> {
        Self::from_frequency(freq.0 / 1_000_000)
    }

    /// Get the DVFS configuration for this mode
    pub fn get_config(self) -> LpsysDvfsConfig {
        LPSYS_DVFS_CONFIG[self as usize]
    }

    /// Get the maximum frequency limit for this mode
    pub fn get_frequency_limit(self) -> Hertz {
        Hertz(
            match self {
                LpsysDvfsMode::D => LPSYS_DVFS_MODE_D_LIMIT,
                LpsysDvfsMode::S => LPSYS_DVFS_MODE_S_LIMIT,
            } * 1_000_000,
        )
    }
}

/// LPSYS DVFS configuration
#[derive(Debug, Clone, Copy)]
pub struct LpsysDvfsConfig {
    /// LDO voltage setting
    pub ldo: u8,
    /// ULP memory control register value
    pub ulpmcr: LpUlpmcr,
}

/// Check if LPSYS is currently in S mode (higher voltage)
pub fn is_lpsys_dvfs_mode_s() -> bool {
    LPSYS_CFG.syscr().read().ldo_vsel()
}

/// Configure LPSYS DVFS mode with automatic voltage scaling.
///
/// This function handles the DVFS transition between D mode (≤24MHz) and S mode (≤48MHz).
///
/// # Safety
///
/// This function modifies hardware registers and should be called with interrupts disabled
/// or in a critical section if LPSYS peripherals are in use.
///
/// # Arguments
///
/// * `current` - Current LPSYS HCLK frequency
/// * `target` - Target LPSYS HCLK frequency
/// * `config_fn` - Function to configure the clock dividers (called at the appropriate point in the transition)
pub unsafe fn config_lpsys_dvfs<F>(current: Hertz, target: Hertz, config_fn: F)
where
    F: FnOnce(),
{
    let current_mode = LpsysDvfsMode::from_hertz(current).unwrap_or(LpsysDvfsMode::S);
    let target_mode = LpsysDvfsMode::from_hertz(target).unwrap_or(LpsysDvfsMode::D);

    match (current_mode, target_mode) {
        (LpsysDvfsMode::D, LpsysDvfsMode::D) | (LpsysDvfsMode::S, LpsysDvfsMode::S) => {
            // Same mode, just configure clock
            config_fn();
        }
        (LpsysDvfsMode::D, LpsysDvfsMode::S) => {
            // D -> S: Increase voltage first, then frequency
            switch_lpsys_dvfs_d2s(target_mode, config_fn);
        }
        (LpsysDvfsMode::S, LpsysDvfsMode::D) => {
            // S -> D: Decrease frequency first, then voltage
            switch_lpsys_dvfs_s2d(target_mode, config_fn);
        }
    }
}

fn config_lpsys_s_mode_volt(target_dvfs_mode: LpsysDvfsMode) {
    let dvfs_config = target_dvfs_mode.get_config();

    // Configure LDO voltage
    // TODO: use efuse value if available
    PMUC.lpsys_vout().modify(|w| {
        w.set_vout(dvfs_config.ldo);
    });
}

fn switch_lpsys_dvfs_d2s<F>(target_dvfs_mode: LpsysDvfsMode, config_clock: F)
where
    F: FnOnce(),
{
    config_lpsys_s_mode_volt(target_dvfs_mode);

    // Switch to S mode (ldo_vsel=1 selects S-mode voltage rails)
    LPSYS_CFG.syscr().modify(|w| {
        w.set_ldo_vsel(true);
    });

    // Wait for voltage to settle (250us)
    crate::cortex_m_blocking_delay_us(250);

    config_clock();
}

fn switch_lpsys_dvfs_s2d<F>(target_dvfs_mode: LpsysDvfsMode, config_clock: F)
where
    F: FnOnce(),
{
    let dvfs_config = target_dvfs_mode.get_config();

    // Configure LDO voltage for D mode
    PMUC.lpsys_ldo().modify(|w| {
        w.set_vref(dvfs_config.ldo);
    });

    // First reduce frequency
    config_clock();

    // Configure memory parameters
    LPSYS_CFG.ulpmcr().write_value(dvfs_config.ulpmcr);

    // Switch to D mode (ldo_vsel=0 selects D-mode voltage rails)
    LPSYS_CFG.syscr().modify(|w| {
        w.set_ldo_vsel(false);
    });
}
