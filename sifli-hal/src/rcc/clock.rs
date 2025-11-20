use crate::pac::{HPSYS_CFG, PMUC};

use crate::cortex_m_blocking_delay_us;
use crate::pac::hpsys_rcc::vals::{SelPeri, SelSys, SelTick};
use crate::pac::{HPSYS_AON, HPSYS_RCC};
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU8, Ordering, compiler_fence};

use crate::rcc::mux::ClockMux;
use crate::time::{Hertz, MaybeHertz};
use core::ops;

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
static mut CLOCK_FREQS: MaybeUninit<Clocks> = MaybeUninit::uninit();

// Power on mode - stored globally
// static POWER_ON_MODE: AtomicU8 = AtomicU8::new(PowerOnMode::ColdBoot as u8);

/// LCPU wakeup reference counter (SF32LB52X specific)
/// Used to track nested wakeup requests
static LCPU_WAKEUP_REF_COUNT: AtomicU8 = AtomicU8::new(0);

/// Sets the clock frequencies
///
/// Safety: Sets a mutable global.
pub(crate) unsafe fn set_freqs(freqs: Clocks) {
    debug!("rcc: {:?}", freqs);
    unsafe { CLOCK_FREQS = MaybeUninit::new(freqs) };
}

/// Safety: Reads a mutable global.
pub(crate) unsafe fn get_freqs() -> &'static Clocks {
    unsafe { (*core::ptr::addr_of_mut!(CLOCK_FREQS)).assume_init_ref() }
}

/// Get the current clock configuration of the chip.
pub fn clocks() -> &'static Clocks {
    unsafe { get_freqs() }
}

/// Get current HCLK frequency from hardware registers
/// This is used internally during initialization before clocks() is available
pub(crate) fn get_hclk_freq() -> Option<Hertz> {
    let clk_sys = get_clk_sys_freq()?;
    Some(clk_sys / HPSYS_RCC.cfgr().read().hdiv())
}

/// Get current sysclk frequency from hardware registers
pub(crate) fn get_clk_sys_freq() -> Option<Hertz> {
    match HPSYS_RCC.csr().read().sel_sys() {
        SelSys::Hrc48 => get_hrc48_freq(),
        SelSys::Hxt48 => get_hxt48_freq(),
        SelSys::Dbl96 => None, // Not implemented
        SelSys::Dll1 => get_clk_dll1_freq(),
    }
}

/// Get HXT48 status and frequency
pub(crate) fn get_hxt48_freq() -> Option<Hertz> {
    if HPSYS_AON.acr().read().hxt48_rdy() {
        Some(Hertz(48_000_000))
    } else {
        None
    }
}

/// Get HRC48 status and frequency
pub(crate) fn get_hrc48_freq() -> Option<Hertz> {
    if HPSYS_AON.acr().read().hrc48_rdy() {
        Some(Hertz(48_000_000))
    } else {
        None
    }
}

/// Get DLL1 frequency from hardware registers
pub(crate) fn get_clk_dll1_freq() -> Option<Hertz> {
    let dllcr = HPSYS_RCC.dllcr(0).read();
    if dllcr.en() {
        Some(Hertz(
            24_000_000 * (dllcr.stg() + 1) as u32 / (dllcr.out_div2_en() as u32 + 1),
        ))
    } else {
        None
    }
}

/// Get DLL2 frequency from hardware registers
pub(crate) fn get_clk_dll2_freq() -> Option<Hertz> {
    let dllcr = HPSYS_RCC.dllcr(1).read();
    if dllcr.en() {
        Some(Hertz(
            24_000_000 * (dllcr.stg() + 1) as u32 / (dllcr.out_div2_en() as u32 + 1),
        ))
    } else {
        None
    }
}

/// Get USB clock source from hardware registers
pub(crate) fn get_clk_usb_source() -> crate::pac::hpsys_rcc::vals::SelUsbc {
    HPSYS_RCC.csr().read().sel_usbc()
}

/// Get USB clock divider from hardware registers
pub(crate) fn get_clk_usb_div() -> u8 {
    HPSYS_RCC.usbcr().read().div()
}

pub const CLK_LRC10_FREQ: Hertz = Hertz(100_000);
pub const CLK_LRC32_FREQ: Hertz = Hertz(32_000);
pub const CLK_LXT32_FREQ: Hertz = Hertz(32_768);
// NOTE: requires calibration to be accurate
pub const CLK_HRC48_FREQ: Hertz = Hertz(48_000_000);
pub const CLK_HXT48_FREQ: Hertz = Hertz(48_000_000);

// - MARK: hard limits
const DLL_STG_STEP: Hertz = Hertz(24_000_000);
const DLL_MIN_FREQ: Hertz = Hertz(24_000_000);
const DLL_MAX_FREQ: Hertz = Hertz(384_000_000);

// - MARK: Impls

/// System clock source
#[repr(u8)]
pub enum Sysclk {
    HRC48 = 0,
    HXT48 = 1,
    DBL96 = 2,
    DLL1 = 3,
    WDT = 4,
}

/// hclk_hpsys = clk_hpsys / HDIV
/// if HDIV=0, hclk_hpsys = clk_hpsys
#[derive(Clone, Copy)]
pub struct HclkPrescaler(pub u8);

impl Default for HclkPrescaler {
    fn default() -> Self {
        HclkPrescaler(1)
    }
}

impl ops::Div<HclkPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: HclkPrescaler) -> Self::Output {
        if rhs.0 == 0 {
            self
        } else {
            Hertz(self.0 / rhs.0 as u32)
        }
    }
}

/// PDIV, 3bit
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PclkPrescaler {
    DIV1 = 0,
    DIV2 = 1,
    DIV4 = 2,
    DIV8 = 3,
    DIV16 = 4,
    DIV32 = 5,
    DIV64 = 6,
    DIV128 = 7,
}

impl ops::Div<PclkPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: PclkPrescaler) -> Self::Output {
        Hertz(self.0 >> (rhs as u32))
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DllStage {
    MUL1 = 0,
    MUL2 = 1,
    MUL3 = 2,
    MUL4 = 3,
    MUL5 = 4,
    MUL6 = 5,
    MUL7 = 6,
    MUL8 = 7,
    MUL9 = 8,
    MUL10 = 9,
    MUL11 = 10,
    MUL12 = 11,
    MUL13 = 12,
    MUL14 = 13,
    MUL15 = 14,
    MUL16 = 15,
}

impl ops::Mul<DllStage> for Hertz {
    type Output = Hertz;
    fn mul(self, rhs: DllStage) -> Self::Output {
        Hertz(self.0 * (rhs as u32 + 1))
    }
}

// HPSYS_RCC.DLL1CR and HPSYS_RCC.DLL2CR
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Dll {
    // pub in_div2: bool,
    pub out_div2: bool,
    pub stg: DllStage,
}

impl Default for Dll {
    fn default() -> Self {
        Self {
            out_div2: false,
            stg: DllStage::MUL1,
        }
    }
}

/// Clock configuration
///
/// hdiv, pdiv1, pdiv2 = 1, 1, 6 by default SDK settings
/// - HCLK = 240 MHz / 1 = 240 MHz
/// - PCLK1 = 240 MHz / 2 = 120 MHz
/// - PCLK2 = 240 MHz / 64 = 3.75 MHz
pub struct Config {
    /// System clock source
    pub sys: Sysclk,
    /// HCLK prescaler
    pub hdiv: HclkPrescaler,
    /// PCLK prescaler
    pub pdiv1: PclkPrescaler,
    /// PCLK2 prescaler
    pub pdiv2: PclkPrescaler,

    /// DLL1 configuration
    pub dll1: Option<Dll>,
    /// DLL2 configuration
    pub dll2: Option<Dll>,

    /// Whether to calibrate HRC48 against HXT48
    ///
    /// HRC48 is the internal RC oscillator that runs at ~48MHz (uncalibrated).
    /// Calibration requires HXT48 to be enabled and system clock switched away from HRC48.
    /// After calibration, HRC48 can be used as accurate clock source for low-power modes.
    pub hrc48_calibrate: bool,

    // pub audpll: Option<AudPll>,
    // Clock mux configuration
    pub mux: ClockMux,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sys: Sysclk::DLL1,
            hdiv: HclkPrescaler(1),
            pdiv1: PclkPrescaler::DIV2,
            pdiv2: PclkPrescaler::DIV64,
            dll1: Some(Dll {
                out_div2: false,
                stg: DllStage::MUL10,
            }),
            dll2: Some(Dll {
                out_div2: false,
                stg: DllStage::MUL12,
            }),
            hrc48_calibrate: true, // Calibrate HRC48 by default for better accuracy
            mux: ClockMux::default(),
        }
    }
}

impl Config {
    /// Validate the clock configuration for consistency
    ///
    /// Returns `Ok(())` if the configuration is valid, or `Err(&'static str)`
    /// with an error message if there are inconsistencies.
    ///
    /// Checks include:
    /// - If sysclk uses DLL1, dll1 must be Some
    /// - If MPI uses DLL2, dll2 must be Some
    /// - Frequency limits are within hardware specifications
    pub fn validate(&self) -> Result<(), &'static str> {
        // Check sysclk source consistency
        match self.sys {
            Sysclk::DLL1 => {
                if self.dll1.is_none() {
                    return Err("sysclk is set to DLL1, but dll1 is None");
                }
            }
            Sysclk::HRC48 => {
                // HRC48 doesn't need any DLL
            }
            Sysclk::HXT48 => {
                // HXT48 doesn't need any DLL
            }
            Sysclk::DBL96 => {
                return Err("DBL96 is not implemented yet");
            }
            Sysclk::WDT => {
                return Err("WDT as sysclk is not supported");
            }
        }

        // Check MPI (Flash/PSRAM) clock source consistency
        if self.mux.mpi1sel == crate::rcc::mux::MpiSel::DLL2
            || self.mux.mpi2sel == crate::rcc::mux::MpiSel::DLL2
        {
            if self.dll2.is_none() {
                return Err("MPI clock source is set to DLL2, but dll2 is None");
            }
        }

        // Check USB clock source consistency
        if self.mux.usbsel == crate::rcc::mux::Usbsel::DLL2 {
            if self.dll2.is_none() {
                return Err("USB clock source is set to DLL2, but dll2 is None");
            }
        }

        // Check USB clock can be divided to 60MHz
        const USB_TARGET_FREQ: u32 = 60_000_000;
        let usb_source_freq = match self.mux.usbsel {
            crate::rcc::mux::Usbsel::SYSCLK => self.get_sysclk_freq().0,
            crate::rcc::mux::Usbsel::DLL2 => {
                if let Some(dll2) = self.dll2 {
                    (DLL_STG_STEP * dll2.stg).0
                } else {
                    return Err("USB clock source is DLL2, but dll2 is not configured");
                }
            }
        };

        if usb_source_freq % USB_TARGET_FREQ != 0 {
            return Err("USB clock source cannot be divided to exact 60MHz");
        }

        let usb_div = usb_source_freq / USB_TARGET_FREQ - 1;
        if usb_div > 7 {
            return Err("USB clock divider exceeds maximum value (7)");
        }

        // Check frequency limits
        let sysclk_freq = self.get_sysclk_freq();
        if sysclk_freq > DLL_MAX_FREQ {
            return Err("sysclk frequency exceeds maximum limit (384 MHz)");
        }
        if sysclk_freq < DLL_MIN_FREQ && matches!(self.sys, Sysclk::DLL1) {
            return Err("DLL1 frequency below minimum limit (24 MHz)");
        }

        let hclk_freq = self.get_hclk_freq();
        if hclk_freq.0 > 240_000_000 {
            return Err("HCLK frequency exceeds maximum DVFS limit (240 MHz)");
        }

        // Check DLL1 frequency range if configured
        if let Some(dll1) = self.dll1 {
            let dll1_freq = DLL_STG_STEP * dll1.stg;
            if dll1_freq < DLL_MIN_FREQ || dll1_freq > DLL_MAX_FREQ {
                return Err("DLL1 frequency out of valid range (24-384 MHz)");
            }
        }

        // Check DLL2 frequency range if configured
        if let Some(dll2) = self.dll2 {
            let dll2_freq = DLL_STG_STEP * dll2.stg;
            if dll2_freq < DLL_MIN_FREQ || dll2_freq > DLL_MAX_FREQ {
                return Err("DLL2 frequency out of valid range (24-384 MHz)");
            }

            // Check DLL2 vs DVFS mode limit
            use crate::pmu::dvfs::HpsysDvfsMode;
            if let Ok(dvfs_mode) = HpsysDvfsMode::from_hertz(hclk_freq) {
                let dll2_limit = dvfs_mode.get_dll2_limit();
                if dll2_freq > dll2_limit {
                    return Err("DLL2 frequency exceeds current DVFS mode limit");
                }
            }
        }

        Ok(())
    }

    fn get_sysclk_freq(&self) -> Hertz {
        match self.sys {
            Sysclk::HRC48 => CLK_HRC48_FREQ,
            Sysclk::HXT48 => CLK_HXT48_FREQ,
            Sysclk::DLL1 => {
                if let Some(dll1) = self.dll1 {
                    let out = DLL_STG_STEP * dll1.stg;
                    if dll1.out_div2 { out / 2_u32 } else { out }
                } else {
                    // If DLL1 is not configured, read from hardware
                    // crate::rcc::get_clk_dll1_freq().unwrap_or(CLK_HXT48_FREQ)
                    panic!("DLL1 is not configured");
                }
            }
            _ => unimplemented!(),
        }
    }

    fn get_hclk_freq(&self) -> Hertz {
        self.get_sysclk_freq() / self.hdiv
    }
}

/// Clocks configuration
#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    // HPSYS
    pub sysclk: MaybeHertz,
    // AHB
    pub hclk: MaybeHertz,
    // APB
    pub pclk: MaybeHertz,
    pub pclk2: MaybeHertz,

    /// DLL1 frequency (None if not enabled)
    pub dll1: MaybeHertz,
    /// DLL2 frequency (None if not enabled)
    pub dll2: MaybeHertz,

    // always 48M
    pub clk_peri: MaybeHertz,
    // For BTIM2, GPTIM2, etc.
    pub clk_peri_div2: MaybeHertz,

    /// USB clock (must be 60MHz for USB PHY)
    pub clk_usb: MaybeHertz,
    /// WDT clock (from PMUC.CR.SEL_LPCLK: LRC10K or LRC32K)
    pub clk_wdt: MaybeHertz,
    /// RTC clock (from RTC.CR.LPCKSEL: LRC10K or LXT32K)
    pub clk_rtc: MaybeHertz,
    
    /// MPI1/Flash1 clock
    pub clk_mpi1: MaybeHertz,
    /// MPI2/Flash2 clock
    pub clk_mpi2: MaybeHertz,
    
    /// Audio PLL clock (managed by AUDCODEC driver, not RCC)
    pub clk_aud_pll: MaybeHertz,
    /// Audio PLL / 16 clock
    pub clk_aud_pll_div16: MaybeHertz,
}

// 1. HAL_PreInit
pub(crate) unsafe fn init(config: Config) {
    // Validate configuration before applying
    config.validate().expect("Invalid clock configuration");

    // Initialize power-on mode detection
    //unsafe {
    //    system_power_on_mode_init();
    //}

    // not switch back to XT48 if other clock source has been selected already
    if HPSYS_RCC.csr().read().sel_sys() == SelSys::Hxt48 {
        // HAL_HPAON_EnableXT48
        HPSYS_AON.acr().modify(|w| w.set_hxt48_req(true));
        while HPSYS_AON.acr().read().hxt48_rdy() != true {
            // wait until HXT48 ready
        }
    }

    HPSYS_RCC.csr().modify(|w| w.set_sel_peri(SelPeri::Hxt48));

    const PM_STANDBY_BOOT: u8 = 3;

    if HPSYS_AON.pmr().read().mode() != PM_STANDBY_BOOT {
        // TODO: wake LCPU
        // TODO: reset and halt LCPU

        // TODO: BSP_System_Config, get system configure from EFUSE

        // TODO: HAL_HPAON_StartGTimer

        // HAL_PMU_EnableRC32K
        PMUC.lrc32_cr().modify(|w| w.set_en(true));

        // HAL_PMU_LpCLockSelect(PMU_LPCLK_RC32);
        PMUC.cr().modify(|w| w.set_sel_lpclk(true));

        // HAL_PMU_EnableDLL(1);
        PMUC.hxt_cr1().modify(|w| w.set_buf_dll_en(true));

        // TODO: LXT init

        // Calibrate HRC48 if requested
        // IMPORTANT: This must be done AFTER switching to HXT48 or DLL1,
        // because HRC48 frequency is unknown before calibration
        if config.hrc48_calibrate {
            // Ensure we're not running on HRC48
            let current_sysclk = HPSYS_RCC.csr().read().sel_sys();
            if current_sysclk == SelSys::Hrc48 {
                // Must switch away from HRC48 before calibration
                if HPSYS_AON.acr().read().hxt48_rdy() {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Hxt48));
                } else {
                    // Cannot calibrate without a stable reference clock
                    panic!(
                        "Cannot calibrate HRC48: currently running on HRC48 and HXT48 is not available"
                    );
                }
            }

            if let Err(e) = calibrate_hrc48() {
                // Calibration failed, but we can continue if not using HRC48
                debug!("HRC48 calibration warning: {}", e);
            }
        }

        HPSYS_RCC.cfgr().modify(|w| {
            w.set_hdiv(config.hdiv.0);
            w.set_pdiv1(config.pdiv1 as u8);
            w.set_pdiv2(config.pdiv2 as u8);
        });

        let sysclk = config.get_sysclk_freq();
        let hclk = config.get_hclk_freq();
        let current_hclk = get_hclk_freq().unwrap_or(Hertz(48_000_000));

        crate::pmu::dvfs::config_hcpu_dvfs(current_hclk, hclk, || {
            // Safe Clock Switching: If current sysclk is DLL1, switch to HXT/HRC first
            if HPSYS_RCC.csr().read().sel_sys() == SelSys::Dll1 {
                if HPSYS_AON.acr().read().hxt48_rdy() {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Hxt48));
                } else {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Hrc48));
                }
            }

            // HAL_RCC_HCPU_EnableDLL1(240000000);
            if let Some(dll1) = config.dll1 {
                if !HPSYS_CFG.cau2_cr().read().hpbg_en() {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_en(true));
                }
                if !HPSYS_CFG.cau2_cr().read().hpbg_vddpsw_en()  {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_vddpsw_en(true));
                }

                HPSYS_RCC.dllcr(0).modify(|w| w.set_en(false));
                compiler_fence(Ordering::SeqCst);

                HPSYS_RCC.dllcr(0).modify(|w| {
                    w.set_in_div2_en(true); // always true
                    w.set_out_div2_en(dll1.out_div2);
                    w.set_stg(dll1.stg as u8);
                    w.set_en(true);
                });

                // wait for DLL ready, 5us at least
                cortex_m_blocking_delay_us(10);

                while HPSYS_RCC.dllcr(0).read().ready() == false {
                    // wait for DLL ready
                }
            }

            // HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
            match config.sys {
                Sysclk::HRC48 => {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Hrc48));
                }
                Sysclk::HXT48 => {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Hxt48));
                }
                Sysclk::DLL1 => {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(SelSys::Dll1));
                }
                _ => unimplemented!(),
            }
        });

        if let Some(ref dll2) = config.dll2 {
            let current_dll2 = HPSYS_RCC.dllcr(1).read();
            let need_reconfig = !current_dll2.en()
                || current_dll2.stg() != dll2.stg as u8
                || current_dll2.out_div2_en() != dll2.out_div2;

            if need_reconfig {
                // Safe DLL2 reconfiguration procedure:
                // Step 1: Switch Flash/PSRAM (MPI) clocks away from DLL2 to prevent XIP crash
                let old_mpi1_sel = HPSYS_RCC.csr().read().sel_mpi1();
                let old_mpi2_sel = HPSYS_RCC.csr().read().sel_mpi2();

                // Temporarily switch to clk_peri_hpsys
                HPSYS_RCC.csr().modify(|w| {
                    w.set_sel_mpi1(crate::rcc::mux::MpiSel::PERI as u8);
                    w.set_sel_mpi2(crate::rcc::mux::MpiSel::PERI as u8);
                });

                // Step 2: Now safe to reconfigure DLL2
                if HPSYS_CFG.cau2_cr().read().hpbg_en() == false {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_en(true));
                }
                if HPSYS_CFG.cau2_cr().read().hpbg_vddpsw_en() == false {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_vddpsw_en(true));
                }

                HPSYS_RCC.dllcr(1).modify(|w| w.set_en(false));

                HPSYS_RCC.dllcr(1).modify(|w| {
                    w.set_in_div2_en(true); // always true
                    w.set_out_div2_en(dll2.out_div2);
                    w.set_stg(dll2.stg as u8);
                    w.set_en(true);
                });

                // wait for DLL ready, 5us at least
                cortex_m_blocking_delay_us(10);

                while HPSYS_RCC.dllcr(1).read().ready() == false {
                    // wait for DLL ready
                }

                // Step 3: Restore MPI clock sources (if they were using DLL2, switch them back)
                // Only restore if the old source was DLL2, otherwise keep clk_peri_hpsys for safety
                HPSYS_RCC.csr().modify(|w| {
                    if old_mpi1_sel == crate::rcc::mux::MpiSel::DLL2 as u8 {
                        w.set_sel_mpi1(crate::rcc::mux::MpiSel::DLL2 as u8);
                    }
                    if old_mpi2_sel == crate::rcc::mux::MpiSel::DLL2 as u8 {
                        w.set_sel_mpi2(crate::rcc::mux::MpiSel::DLL2 as u8);
                    }
                });
            }
        }

        // other MUX configuration

        // Configure USB clock according to config.mux.usbsel
        // USB PHY requires exactly 60MHz
        const USB_TARGET_FREQ: u32 = 60_000_000;
        let usb_source_freq = match config.mux.usbsel {
            crate::rcc::mux::Usbsel::SYSCLK => config.get_sysclk_freq(),
            crate::rcc::mux::Usbsel::DLL2 => {
                if let Some(dll2) = config.dll2 {
                    DLL_STG_STEP * dll2.stg
                } else {
                    // Fallback to reading from hardware
                    panic!("DLL2 is not configured, cannot configure USB clock");
                }
            }
        };

        let usb_div = (usb_source_freq.0 / USB_TARGET_FREQ).saturating_sub(1) as u8;
        HPSYS_RCC.usbcr().modify(|w| w.set_div(usb_div));
        HPSYS_RCC.csr().modify(|w| {
            w.set_sel_usbc(match config.mux.usbsel {
                crate::rcc::mux::Usbsel::SYSCLK => crate::pac::hpsys_rcc::vals::SelUsbc::ClkSys,
                crate::rcc::mux::Usbsel::DLL2 => crate::pac::hpsys_rcc::vals::SelUsbc::Dll2,
            });
        });

        // Configure peripheral clock according to config.mux.perisel
        HPSYS_RCC.csr().modify(|w| {
            w.set_sel_peri(match config.mux.perisel {
                crate::rcc::mux::PeriSel::HRC48 => SelPeri::Hrc48,
                crate::rcc::mux::PeriSel::HXT48 => SelPeri::Hxt48,
            });
        });

        // Configure low-power clock (for WDT) according to config.mux.wdtsel
        // PMUC.CR.SEL_LPCLK controls the LP clock source for WDT
        PMUC.cr().modify(|w| {
            w.set_sel_lpclk(match config.mux.wdtsel {
                crate::rcc::mux::Wdtsel::LRC10 => false, // 0: LRC10K
                crate::rcc::mux::Wdtsel::LRC32 => true,  // 1: LRC32K
            });
        });

        // Configure RTC clock (for RTC, LPTIM1/2, BT sleep) according to config.mux.rtcsel
        // RTC.CR.LPCKSEL controls the clock source for RTC and low-power timers
        // TODO: Implement RTC clock selection when RTC peripheral is added to PAC
        // crate::pac::RTC.cr().modify(|w| {
        //     w.set_lpcksel(match config.mux.rtcsel {
        //         crate::rcc::mux::Rtcsel::LRC10 => false,  // 0: LRC10K
        //         crate::rcc::mux::Rtcsel::LXT32 => true,   // 1: LXT32K
        //     });
        // });

        // Configure system tick clock according to config.mux.ticksel
        HPSYS_RCC.csr().modify(|w| {
            w.set_sel_tick(match config.mux.ticksel {
                crate::rcc::mux::Ticksel::RTC => SelTick::ClkRtc,
                crate::rcc::mux::Ticksel::_RESERVED_1 => SelTick::_RESERVED_1,
                crate::rcc::mux::Ticksel::HRC48 => SelTick::Hrc48,
                crate::rcc::mux::Ticksel::HXT48 => SelTick::Hxt48,
            })
        });

        // Store the final clock frequencies for later access via clocks()
        let final_clocks = Clocks {
            sysclk: config.get_sysclk_freq().into(),
            hclk: config.get_hclk_freq().into(),
            pclk: (config.get_hclk_freq() / config.pdiv1).into(),
            pclk2: (config.get_hclk_freq() / config.pdiv2).into(),
            dll1: config.dll1.map(|dll1| DLL_STG_STEP * dll1.stg).into(),
            dll2: config.dll2.map(|dll2| DLL_STG_STEP * dll2.stg).into(),
            clk_peri: (match config.mux.perisel {
                crate::rcc::mux::PeriSel::HRC48 => CLK_HRC48_FREQ,
                crate::rcc::mux::PeriSel::HXT48 => CLK_HXT48_FREQ,
            }).into(),
            clk_peri_div2: (match config.mux.perisel {
                crate::rcc::mux::PeriSel::HRC48 => CLK_HRC48_FREQ / 2u32,
                crate::rcc::mux::PeriSel::HXT48 => CLK_HXT48_FREQ / 2u32,
            }).into(),
            clk_usb: {
                let usb_source_freq = match config.mux.usbsel {
                    crate::rcc::mux::Usbsel::SYSCLK => config.get_sysclk_freq(),
                    crate::rcc::mux::Usbsel::DLL2 => {
                        if let Some(dll2) = config.dll2 {
                            DLL_STG_STEP * dll2.stg
                        } else {
                            crate::rcc::get_clk_dll2_freq().unwrap_or(Hertz(240_000_000))
                        }
                    }
                };
                let usb_div = (usb_source_freq.0 / 60_000_000).saturating_sub(1) as u8;
                Some(usb_source_freq / (usb_div as u32 + 1)).into()
            },
            clk_wdt: Some(match config.mux.wdtsel {
                crate::rcc::mux::Wdtsel::LRC10 => CLK_LRC10_FREQ,
                crate::rcc::mux::Wdtsel::LRC32 => CLK_LRC32_FREQ,
            }).into(),
            clk_rtc: Some(match config.mux.rtcsel {
                crate::rcc::mux::Rtcsel::LRC10 => CLK_LRC10_FREQ,
                crate::rcc::mux::Rtcsel::LXT32 => CLK_LXT32_FREQ,
            }).into(),
            clk_mpi1: Some({
                let mpi_source = match config.mux.mpi1sel {
                    crate::rcc::mux::MpiSel::PERI => match config.mux.perisel {
                        crate::rcc::mux::PeriSel::HRC48 => CLK_HRC48_FREQ,
                        crate::rcc::mux::PeriSel::HXT48 => CLK_HXT48_FREQ,
                    },
                    crate::rcc::mux::MpiSel::DLL2 => {
                        if let Some(dll2) = config.dll2 {
                            DLL_STG_STEP * dll2.stg
                        } else {
                            CLK_HXT48_FREQ // Fallback
                        }
                    },
                    crate::rcc::mux::MpiSel::DLL3 => {
                        // DLL3 not supported on SF32LB52X
                        CLK_HXT48_FREQ
                    },
                };
                mpi_source
            }).into(),
            clk_mpi2: Some({
                let mpi_source = match config.mux.mpi2sel {
                    crate::rcc::mux::MpiSel::PERI => match config.mux.perisel {
                        crate::rcc::mux::PeriSel::HRC48 => CLK_HRC48_FREQ,
                        crate::rcc::mux::PeriSel::HXT48 => CLK_HXT48_FREQ,
                    },
                    crate::rcc::mux::MpiSel::DLL2 => {
                        if let Some(dll2) = config.dll2 {
                            DLL_STG_STEP * dll2.stg
                        } else {
                            CLK_HXT48_FREQ // Fallback
                        }
                    },
                    crate::rcc::mux::MpiSel::DLL3 => {
                        // DLL3 not supported on SF32LB52X
                        CLK_HXT48_FREQ
                    },
                };
                mpi_source
            }).into(),
            // Audio PLL is managed by AUDCODEC driver, default to None here
            clk_aud_pll: None.into(),
            clk_aud_pll_div16: None.into(),
        };

        set_freqs(final_clocks);
    }
}

/// Calibrate HRC48 (48MHz internal RC oscillator) against HXT48 (external crystal)
///
/// Uses binary search algorithm to adjust HRC48 frequency trim to match HXT48.
///
/// # Returns
/// - `Ok(())` if calibration succeeds (frequency difference < 160 counts)
/// - `Err(&str)` if calibration fails or HXT48 is not ready
///
/// # Safety
/// - HXT48 must be enabled and ready before calling this function
/// - This function takes ~1ms to complete (11 iterations × ~100us each)
pub unsafe fn calibrate_hrc48() -> Result<(), &'static str> {
    // Check HXT48 is ready
    if !HPSYS_AON.acr().read().hxt48_rdy() {
        return Err("HXT48 is not ready, cannot calibrate HRC48");
    }

    // Set calibration length
    HPSYS_RCC.hrccal1().modify(|w| w.set_cal_length(0x3fff));

    // Configure HRC clock selection for calibration
    PMUC.hrc_cr().modify(|w| {
        w.set_clkhp_sel(3);
        w.set_clklp_sel(3);
    });

    // Binary search for optimal trim value
    let mut ct_val: u16 = 0x200; // Start at mid-point
    let mut step_unit: u16 = 0x100;
    let mut cnt_diff: u16 = 0;

    for _ in 0..11 {
        // Set trim value
        PMUC.hrc_cr().modify(|w| w.set_freq_trim(ct_val));

        // Wait for stabilization
        crate::cortex_m_blocking_delay_us(3);

        // Enable calibration
        HPSYS_RCC.hrccal1().modify(|w| w.set_cal_en(true));

        // Wait for calibration done
        while !HPSYS_RCC.hrccal1().read().cal_done() {}

        // Read calibration results
        let cal2 = HPSYS_RCC.hrccal2().read();
        let hxt_cnt = cal2.hxt_cnt();
        let hrc_cnt = cal2.hrc_cnt();

        // Disable calibration
        HPSYS_RCC.hrccal1().modify(|w| w.set_cal_en(false));

        // Adjust trim value based on results
        if hxt_cnt > hrc_cnt {
            ct_val = ct_val.wrapping_sub(step_unit);
            cnt_diff = hxt_cnt - hrc_cnt;
        } else {
            ct_val = ct_val.wrapping_add(step_unit);
            cnt_diff = hrc_cnt - hxt_cnt;
        }

        // Check if calibration is good enough
        if cnt_diff < 64 {
            break;
        }

        step_unit >>= 1;
    }

    // Validate final result
    if cnt_diff > 160 {
        Err("HRC48 calibration failed: frequency difference too large")
    } else {
        Ok(())
    }
}

/// Wake up LCPU
///
/// Sends wakeup request to LCPU and waits for acknowledgment.
/// Maintains reference counter for nested calls (SF32LB52X specific).
pub unsafe fn wake_lcpu() {
    // Set HP2LP_REQ bit
    HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(true));

    // Wait for LCPU to see the request and respond
    cortex_m::asm::delay(48 * 230); // 230us @ 48MHz
    while !HPSYS_AON.issr().read().lp_active() {}
    cortex_m::asm::delay(48 * 30); // 30us @ 48MHz
    while !HPSYS_AON.issr().read().lp_active() {}

    // Increment reference counter
    LCPU_WAKEUP_REF_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// Cancel LCPU active request (paired with wake_lcpu)
pub unsafe fn cancel_lcpu_active_request() {
    let count = LCPU_WAKEUP_REF_COUNT.fetch_sub(1, Ordering::Relaxed);

    // Clear HP2LP_REQ when count reaches 0
    if count == 1 {
        HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(false));
    }
}

pub struct ConfigBuilder {
    config: Config,
}

impl ConfigBuilder {
    pub fn new() -> Self {
        Self {
            config: Config::default(),
        }
    }

    pub fn build(self) -> Config {
        self.config
    }

    pub fn config_hclk_mhz(self, hclk_mhz: u32) -> Self {
        todo!()
    }
}
