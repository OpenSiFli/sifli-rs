//! Clock configuration, initialization, and runtime reconfiguration.

use crate::cortex_m_blocking_delay_us;
use crate::pac::hpsys_rcc::vals::mux::Perisel;
use crate::pac::{HPSYS_AON, HPSYS_RCC};
use crate::pac::{HPSYS_CFG, PMUC};
use crate::time::Hertz;
use core::sync::atomic::{compiler_fence, Ordering};

use super::{modify_freqs, set_freqs};
use super::{get_hclk_freq, read_hpsys_clocks_from_hw};
use super::{
    ClockMux, Dll, DllStage, HclkPrescaler, Lpsel, Mpisel, PclkPrescaler, Rtcsel, Sysclk,
    Ticksel, Usbsel, Wdtsel,
};

/// Clock configuration
///
/// hdiv, pdiv1, pdiv2 = 1, 1, 6 by default SDK settings
/// - HCLK = 240 MHz / 1 = 240 MHz
/// - PCLK1 = 240 MHz / 2 = 120 MHz
/// - PCLK2 = 240 MHz / 64 = 3.75 MHz
#[non_exhaustive]
pub struct ConfigBuilder {
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

    /// Whether USB clock is required (must be exactly 60MHz).
    ///
    /// When `true` (default), `check()` validates that the USB clock source can be
    /// divided to exactly 60MHz, and `init()` configures the USB clock divider.
    /// Set to `false` if USB is not used, allowing more flexible clock configurations.
    pub usb: bool,

    // pub audpll: Option<AudPll>,
    // Clock mux configuration
    pub mux: ClockMux,
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self {
            hdiv: HclkPrescaler(1),
            pdiv1: PclkPrescaler::Div2,
            pdiv2: PclkPrescaler::Div64,
            sys: Sysclk::Dll1,
            dll1: Some(Dll {
                out_div2: false,
                stg: DllStage::Mul10,
            }),
            dll2: Some(Dll {
                out_div2: false,
                stg: DllStage::Mul12,
            }),
            hrc48_calibrate: true, // Calibrate HRC48 by default for better accuracy
            usb: true,
            mux: ClockMux::default(),
        }
    }
}

impl ConfigBuilder {
    pub const fn new() -> Self {
        Self {
            hdiv: HclkPrescaler(1),
            pdiv1: PclkPrescaler::Div2,
            pdiv2: PclkPrescaler::Div64,
            sys: Sysclk::Dll1,
            dll1: Some(Dll {
                out_div2: false,
                stg: DllStage::Mul10,
            }),
            dll2: Some(Dll {
                out_div2: false,
                stg: DllStage::Mul12,
            }),
            hrc48_calibrate: true,
            usb: true,
            mux: ClockMux::new(),
        }
    }

    /// Create a builder for runtime sysclk reconfiguration.
    ///
    /// Defaults to HXT48, no DLL, no USB, no DLL2 — safe for runtime switching
    /// where only the sysclk/hclk/pclk tree matters.
    pub const fn sysclk() -> Self {
        Self {
            sys: Sysclk::Hxt48,
            hdiv: HclkPrescaler(1),
            pdiv1: PclkPrescaler::Div1,
            pdiv2: PclkPrescaler::Div1,
            dll1: None,
            dll2: None,
            hrc48_calibrate: false,
            usb: false,
            mux: ClockMux {
                // MPI defaults to Peri (no DLL dependency) for safe validation
                mpi1sel: Mpisel::Peri,
                mpi2sel: Mpisel::Peri,
                rtcsel: Rtcsel::Lrc10,
                wdtsel: Wdtsel::Lrc32,
                usbsel: Usbsel::Sysclk,
                perisel: Perisel::Hxt48,
                ticksel: Ticksel::ClkRtc,
                lpsel: Lpsel::SelSys,
            },
        }
    }

    pub const fn with_sys(mut self, sys: Sysclk) -> Self {
        self.sys = sys;
        self
    }

    pub const fn with_hdiv(mut self, hdiv: HclkPrescaler) -> Self {
        self.hdiv = hdiv;
        self
    }

    pub const fn with_pdiv1(mut self, pdiv1: PclkPrescaler) -> Self {
        self.pdiv1 = pdiv1;
        self
    }

    pub const fn with_pdiv2(mut self, pdiv2: PclkPrescaler) -> Self {
        self.pdiv2 = pdiv2;
        self
    }

    pub const fn with_dll1(mut self, dll1: Dll) -> Self {
        self.dll1 = Some(dll1);
        self
    }

    pub const fn with_dll2(mut self, dll2: Dll) -> Self {
        self.dll2 = Some(dll2);
        self
    }

    pub const fn with_hrc48_calibrate(mut self, cal: bool) -> Self {
        self.hrc48_calibrate = cal;
        self
    }

    pub const fn with_usb(mut self, usb: bool) -> Self {
        self.usb = usb;
        self
    }

    pub const fn with_mux(mut self, mux: ClockMux) -> Self {
        self.mux = mux;
        self
    }

    /// Validate the clock configuration at compile time.
    ///
    /// Panics with a descriptive message if the configuration is invalid.
    /// Use inside `const { }` blocks to get compile-time errors.
    ///
    /// Note: Uses `::core::panic!` to bypass defmt's panic override,
    /// which is not const-compatible.
    pub const fn check(&self) {
        // Check sysclk source consistency
        match self.sys {
            Sysclk::Dll1 => {
                if self.dll1.is_none() {
                    ::core::panic!("sysclk is set to DLL1, but dll1 is None");
                }
            }
            Sysclk::Hrc48 | Sysclk::Hxt48 => {}
            Sysclk::Dbl96 => {
                ::core::panic!("DBL96 is not implemented yet");
            }
        }

        // Check MPI (Flash/PSRAM) clock source consistency
        match self.mux.mpi1sel {
            Mpisel::Dll2 => {
                if self.dll2.is_none() {
                    ::core::panic!("MPI1 clock source is set to DLL2, but dll2 is None");
                }
            }
            Mpisel::Dll1 => {
                if self.dll1.is_none() {
                    ::core::panic!("MPI1 clock source is set to DLL1, but dll1 is None");
                }
            }
            _ => {}
        }
        match self.mux.mpi2sel {
            Mpisel::Dll2 => {
                if self.dll2.is_none() {
                    ::core::panic!("MPI2 clock source is set to DLL2, but dll2 is None");
                }
            }
            Mpisel::Dll1 => {
                if self.dll1.is_none() {
                    ::core::panic!("MPI2 clock source is set to DLL1, but dll1 is None");
                }
            }
            _ => {}
        }

        // Check USB clock constraints (only when USB is enabled)
        if self.usb {
            if let Usbsel::Dll2 = self.mux.usbsel {
                if self.dll2.is_none() {
                    ::core::panic!("USB clock source is set to DLL2, but dll2 is None");
                }
            }

            let usb_source_freq = match self.mux.usbsel {
                Usbsel::Sysclk => self.get_sysclk_freq_hz(),
                Usbsel::Dll2 => match self.dll2 {
                    Some(dll2) => dll2.freq_hz(),
                    None => ::core::panic!("USB clock source is DLL2, but dll2 is not configured"),
                },
            };

            if usb_source_freq % 60_000_000 != 0 {
                ::core::panic!("USB clock source cannot be divided to exact 60MHz");
            }

            let usb_div = usb_source_freq / 60_000_000;
            if usb_div > 7 {
                ::core::panic!("USB clock divider exceeds maximum value (7)");
            }
        }

        // Check frequency limits (bypassed when `unchecked-overclocking` is enabled)
        let sysclk_hz = self.get_sysclk_freq_hz();
        const_rcc_assert!(
            sysclk_hz <= 384_000_000,
            "sysclk frequency exceeds maximum limit (384 MHz)"
        );
        if let Sysclk::Dll1 = self.sys {
            const_rcc_assert!(
                sysclk_hz >= 24_000_000,
                "DLL1 sysclk frequency below minimum limit (24 MHz)"
            );
        }

        let hclk_hz = self.get_hclk_freq_hz();
        const_rcc_assert!(
            hclk_hz <= 240_000_000,
            "HCLK frequency exceeds maximum DVFS limit (240 MHz)"
        );

        // Check PCLK frequency limits based on DVFS mode
        let pclk1_hz = hclk_hz >> (self.pdiv1 as u32);
        let pclk2_hz = hclk_hz >> (self.pdiv2 as u32);
        if hclk_hz > 48_000_000 {
            // S mode (enhanced): pclk1 ≤ 120 MHz, pclk2 ≤ 7.5 MHz
            const_rcc_assert!(
                pclk1_hz <= 120_000_000,
                "PCLK1 exceeds S-mode limit (120 MHz), increase pdiv1"
            );
            const_rcc_assert!(
                pclk2_hz <= 7_500_000,
                "PCLK2 exceeds S-mode limit (7.5 MHz), increase pdiv2"
            );
        } else {
            // D mode (basic): pclk1 ≤ 48 MHz, pclk2 ≤ 6 MHz
            const_rcc_assert!(
                pclk1_hz <= 48_000_000,
                "PCLK1 exceeds D-mode limit (48 MHz), increase pdiv1"
            );
            const_rcc_assert!(
                pclk2_hz <= 6_000_000,
                "PCLK2 exceeds D-mode limit (6 MHz), increase pdiv2"
            );
        }

        // Check DLL1 frequency range if configured
        if let Some(dll1) = self.dll1 {
            let dll1_freq = 24_000_000 * (dll1.stg.to_bits() as u32 + 1);
            const_rcc_assert!(
                dll1_freq >= 24_000_000 && dll1_freq <= 384_000_000,
                "DLL1 frequency out of valid range (24-384 MHz)"
            );
        }

        // Check DLL2 frequency range if configured
        if let Some(dll2) = self.dll2 {
            let dll2_freq = 24_000_000 * (dll2.stg.to_bits() as u32 + 1);
            const_rcc_assert!(
                dll2_freq >= 24_000_000 && dll2_freq <= 384_000_000,
                "DLL2 frequency out of valid range (24-384 MHz)"
            );

            // Check DLL2 vs DVFS mode limit
            let hclk_mhz = hclk_hz / 1_000_000;
            if hclk_mhz > 48 {
                const_rcc_assert!(
                    dll2_freq <= 288_000_000,
                    "DLL2 frequency exceeds DVFS S-mode limit (288 MHz)"
                );
            }
        }
    }

    /// Validate and return a [`Config`]. Use in `const { }` blocks for compile-time checking.
    ///
    /// ```rust,ignore
    /// const { rcc::ConfigBuilder::new().with_sys(Sysclk::Dll1).with_dll1(...).checked() }
    /// ```
    pub const fn checked(self) -> Config {
        self.check();
        Config(self)
    }

    pub(crate) const fn get_sysclk_freq_hz(&self) -> u32 {
        match self.sys {
            Sysclk::Hrc48 | Sysclk::Hxt48 => 48_000_000,
            Sysclk::Dll1 => match self.dll1 {
                Some(dll1) => dll1.freq_hz(),
                None => ::core::panic!("DLL1 is not configured"),
            },
            Sysclk::Dbl96 => ::core::panic!("DBL96 is not implemented"),
        }
    }

    pub(crate) const fn get_hclk_freq_hz(&self) -> u32 {
        let s = self.get_sysclk_freq_hz();
        if self.hdiv.0 == 0 {
            s
        } else {
            s / self.hdiv.0 as u32
        }
    }

    // Non-const versions for use in init() where Hertz operators are needed
    pub(crate) fn get_sysclk_freq(&self) -> Hertz {
        Hertz(self.get_sysclk_freq_hz())
    }

    pub(crate) fn get_hclk_freq(&self) -> Hertz {
        Hertz(self.get_hclk_freq_hz())
    }
}

/// A validated clock configuration.
///
/// Can only be constructed via [`ConfigBuilder::checked()`], which validates at
/// compile time when used inside a `const { }` block.
pub struct Config(pub(crate) ConfigBuilder);

// =============================================================================
// DLL1 Configuration Helpers
// =============================================================================

/// Safely switch sysclk away from DLL1 (to HXT48 or HRC48).
///
/// If the current sysclk source is not DLL1, this is a no-op.
fn switch_away_from_dll1() {
    if HPSYS_RCC.csr().read().sel_sys() == Sysclk::Dll1 {
        if HPSYS_AON.acr().read().hxt48_rdy() {
            HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Hxt48));
        } else {
            HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Hrc48));
        }
    }
}

/// Ensure HPBG is enabled (required for DLL operation) and configure DLL1.
///
/// Disables DLL1, reconfigures it with the given parameters, and waits until ready.
fn configure_dll1(dll1: &Dll) {
    if !HPSYS_CFG.cau2_cr().read().hpbg_en() {
        HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_en(true));
    }
    if !HPSYS_CFG.cau2_cr().read().hpbg_vddpsw_en() {
        HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_vddpsw_en(true));
    }

    HPSYS_RCC.dllcr(0).modify(|w| w.set_en(false));
    compiler_fence(Ordering::SeqCst);

    HPSYS_RCC.dllcr(0).modify(|w| {
        w.set_in_div2_en(true); // always true
        w.set_out_div2_en(dll1.out_div2);
        w.set_stg(dll1.stg);
        w.set_en(true);
    });

    // wait for DLL ready, 5us at least
    cortex_m_blocking_delay_us(10);
    while !HPSYS_RCC.dllcr(0).read().ready() {}
}

/// Set HPSYS dividers and switch sysclk source.
fn apply_dividers_and_sysclk(config: &ConfigBuilder) {
    HPSYS_RCC.cfgr().modify(|w| {
        w.set_hdiv(config.hdiv.0);
        w.set_pdiv1(config.pdiv1);
        w.set_pdiv2(config.pdiv2);
    });

    match config.sys {
        Sysclk::Hrc48 => HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Hrc48)),
        Sysclk::Hxt48 => HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Hxt48)),
        Sysclk::Dll1 => HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Dll1)),
        _ => unimplemented!(),
    }
}

// =============================================================================
// Runtime Reconfiguration
// =============================================================================

/// Reconfigure sysclk at runtime.
///
/// Requires exclusive access to `hclk`, `pclk`, and `pclk2` tokens,
/// which guarantees no peripheral currently borrows these clocks.
///
/// When the `time-driver-gptim1` feature is enabled this function is unavailable
/// because GPTIM1 uses `pclk` and switching sysclk would break timekeeping.
///
/// Use [`ConfigBuilder::sysclk()`] to create a configuration suitable for runtime switching.
///
/// // TODO: once peripheral drivers start borrowing `&'d Hclk` / `&'d Pclk` tokens,
/// // this function should take `&mut Hclk, &mut Pclk, &mut Pclk2` to enforce at
/// // compile time that no peripheral is using those clock domains during reconfiguration.
/// // Currently no driver borrows these tokens, so the mechanism is not yet active.
#[cfg(not(feature = "time-driver-gptim1"))]
pub fn reconfigure_sysclk(config: Config) {
    let config = &config.0;
    let current_hclk = get_hclk_freq().unwrap_or(Hertz(48_000_000));
    let target_hclk = config.get_hclk_freq();

    crate::pmu::dvfs::config_hcpu_dvfs(current_hclk, target_hclk, || {
        switch_away_from_dll1();

        if let Some(dll1) = config.dll1 {
            configure_dll1(&dll1);
        }

        apply_dividers_and_sysclk(config);
    });

    // Update global frequency state from hardware
    let hw = read_hpsys_clocks_from_hw();
    modify_freqs(|clocks| {
        // Preserve audio PLL state (managed by AUDCODEC driver)
        let aud_pll = clocks.clk_aud_pll;
        let aud_pll_div16 = clocks.clk_aud_pll_div16;
        *clocks = hw;
        clocks.clk_aud_pll = aud_pll;
        clocks.clk_aud_pll_div16 = aud_pll_div16;
    });
}

// =============================================================================
// Initialization
// =============================================================================

// 1. HAL_PreInit
pub(crate) unsafe fn init(config: Config) {
    let config = &config.0;

    // not switch back to XT48 if other clock source has been selected already
    if HPSYS_RCC.csr().read().sel_sys() == Sysclk::Hxt48 {
        // HAL_HPAON_EnableXT48
        HPSYS_AON.acr().modify(|w| w.set_hxt48_req(true));
        while !HPSYS_AON.acr().read().hxt48_rdy() {
            // wait until HXT48 ready
        }
    }

    HPSYS_RCC.csr().modify(|w| w.set_sel_peri(Perisel::Hxt48));

    const PM_STANDBY_BOOT: u8 = 3;

    if HPSYS_AON.pmr().read().mode() != PM_STANDBY_BOOT {
        // TODO: wake LCPU
        // TODO: reset and halt LCPU

        // TODO: BSP_System_Config, get system configure from EFUSE

        // TODO: HAL_HPAON_StartGTimer

        // HAL_PMU_EnableRC32K
        PMUC.lrc32_cr().modify(|w| w.set_en(true));

        // HAL_PMU_LpCLockSelect(PMU_LPCLK_RC32);
        PMUC.cr().modify(|w| w.set_sel_lpclk(Wdtsel::Lrc32));

        // HAL_PMU_EnableDLL(1);
        PMUC.hxt_cr1().modify(|w| w.set_buf_dll_en(true));

        // TODO: LXT init

        // Calibrate HRC48 if requested
        // IMPORTANT: This must be done AFTER switching to HXT48 or DLL1,
        // because HRC48 frequency is unknown before calibration
        if config.hrc48_calibrate {
            // Ensure we're not running on HRC48
            let current_sysclk = HPSYS_RCC.csr().read().sel_sys();
            if current_sysclk == Sysclk::Hrc48 {
                // Must switch away from HRC48 before calibration
                if HPSYS_AON.acr().read().hxt48_rdy() {
                    HPSYS_RCC.csr().modify(|w| w.set_sel_sys(Sysclk::Hxt48));
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

        // Set dividers first (before DVFS transition)
        HPSYS_RCC.cfgr().modify(|w| {
            w.set_hdiv(config.hdiv.0);
            w.set_pdiv1(config.pdiv1);
            w.set_pdiv2(config.pdiv2);
        });

        let hclk = config.get_hclk_freq();
        let current_hclk = get_hclk_freq().unwrap_or(Hertz(48_000_000));

        crate::pmu::dvfs::config_hcpu_dvfs(current_hclk, hclk, || {
            switch_away_from_dll1();

            if let Some(dll1) = config.dll1 {
                configure_dll1(&dll1);
            }

            apply_dividers_and_sysclk(config);
        });

        if let Some(ref dll2) = config.dll2 {
            let current_dll2 = HPSYS_RCC.dllcr(1).read();
            let need_reconfig = !current_dll2.en()
                || current_dll2.stg() != dll2.stg
                || current_dll2.out_div2_en() != dll2.out_div2;

            if need_reconfig {
                // Safe DLL2 reconfiguration procedure:
                // Step 1: Switch Flash/PSRAM (MPI) clocks away from DLL2 to prevent XIP crash
                let old_mpi1_sel = HPSYS_RCC.csr().read().sel_mpi1();
                let old_mpi2_sel = HPSYS_RCC.csr().read().sel_mpi2();

                // Temporarily switch to clk_peri_hpsys
                HPSYS_RCC.csr().modify(|w| {
                    w.set_sel_mpi1(Mpisel::Peri);
                    w.set_sel_mpi2(Mpisel::Peri);
                });

                // Step 2: Now safe to reconfigure DLL2
                if !HPSYS_CFG.cau2_cr().read().hpbg_en() {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_en(true));
                }
                if !HPSYS_CFG.cau2_cr().read().hpbg_vddpsw_en() {
                    HPSYS_CFG.cau2_cr().modify(|w| w.set_hpbg_vddpsw_en(true));
                }

                HPSYS_RCC.dllcr(1).modify(|w| w.set_en(false));

                HPSYS_RCC.dllcr(1).modify(|w| {
                    w.set_in_div2_en(true); // always true
                    w.set_out_div2_en(dll2.out_div2);
                    w.set_stg(dll2.stg);
                    w.set_en(true);
                });

                // wait for DLL ready, 5us at least
                cortex_m_blocking_delay_us(10);

                while !HPSYS_RCC.dllcr(1).read().ready() {
                    // wait for DLL ready
                }

                // Step 3: Restore MPI clock sources
                HPSYS_RCC.csr().modify(|w| {
                    if old_mpi1_sel == Mpisel::Dll2 {
                        w.set_sel_mpi1(Mpisel::Dll2);
                    }
                    if old_mpi2_sel == Mpisel::Dll2 {
                        w.set_sel_mpi2(Mpisel::Dll2);
                    }
                });
            }
        }

        // other MUX configuration

        // Configure USB clock only when USB is enabled
        if config.usb {
            const USB_TARGET_FREQ: u32 = 60_000_000;
            let usb_source_freq = match config.mux.usbsel {
                Usbsel::Sysclk => config.get_sysclk_freq(),
                Usbsel::Dll2 => {
                    if let Some(dll2) = config.dll2 {
                        Hertz(dll2.freq_hz())
                    } else {
                        panic!("DLL2 is not configured, cannot configure USB clock");
                    }
                }
            };

            let usb_div = (usb_source_freq.0 / USB_TARGET_FREQ) as u8;
            HPSYS_RCC.usbcr().modify(|w| w.set_div(usb_div));
            HPSYS_RCC.csr().modify(|w| {
                w.set_sel_usbc(config.mux.usbsel);
            });
        }

        // Configure peripheral clock according to config.mux.perisel
        HPSYS_RCC.csr().modify(|w| {
            w.set_sel_peri(config.mux.perisel);
        });

        // Configure low-power clock (for WDT) according to config.mux.wdtsel
        PMUC.cr().modify(|w| {
            w.set_sel_lpclk(config.mux.wdtsel);
        });

        // Configure RTC clock
        // TODO: Uncomment when RTC peripheral is added to PAC
        // crate::pac::RTC.cr().modify(|w| {
        //     w.set_lpcksel(config.mux.rtcsel);
        // });

        // Configure system tick clock according to config.mux.ticksel
        HPSYS_RCC
            .csr()
            .modify(|w| w.set_sel_tick(config.mux.ticksel));
    }

    // Store the final clock frequencies for later access via clocks()
    // This must run on both normal boot and standby boot paths,
    // because CLOCK_FREQS lives in RAM which is lost during standby.
    let final_clocks = read_hpsys_clocks_from_hw();

    set_freqs(final_clocks);
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
            ct_val = ct_val.wrapping_add(step_unit);
            cnt_diff = hxt_cnt - hrc_cnt;
        } else {
            ct_val = ct_val.wrapping_sub(step_unit);
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
