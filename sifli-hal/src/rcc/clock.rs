//! Clock types, constants, and global state.

pub use crate::pac::hpsys_rcc::vals::mux::{
    Lpsel, Mpisel, Perisel, Rtcsel, Ticksel, Usbsel, Wdtsel,
};
pub use crate::pac::hpsys_rcc::vals::{Dllstg as DllStage, Pdiv as PclkPrescaler, Sysclk};

use crate::time::{Hertz, MaybeHertz};
use core::ops;
use core::sync::atomic::{AtomicBool, Ordering};

// =============================================================================
// Global Clock State
// =============================================================================

/// Whether `CLOCK_FREQS` has been initialized by `set_freqs()`.
static CLOCK_FREQS_INIT: AtomicBool = AtomicBool::new(false);

/// Cached HPSYS clock frequencies (LPSYS clocks are always read from hardware).
static mut CLOCK_FREQS: Clocks = Clocks::ZERO;

/// Sets the clock frequencies.
///
/// Safety: Sets a mutable global.
pub(crate) unsafe fn set_freqs(freqs: Clocks) {
    debug!("rcc: {:?}", freqs);
    unsafe { CLOCK_FREQS = freqs };
    CLOCK_FREQS_INIT.store(true, Ordering::Release);
}

/// Safety: Reads a mutable global. Must be called after `set_freqs()`.
pub(crate) unsafe fn get_freqs() -> &'static Clocks {
    unsafe { &*core::ptr::addr_of!(CLOCK_FREQS) }
}

/// Get the current HPSYS clock configuration.
///
/// # Panics
///
/// Panics if called before `init()`.
pub fn clocks() -> &'static Clocks {
    assert!(
        CLOCK_FREQS_INIT.load(Ordering::Acquire),
        "rcc: clocks() called before init()"
    );
    unsafe { get_freqs() }
}

// =============================================================================
// Constants
// =============================================================================

pub const CLK_LRC10_FREQ: Hertz = Hertz(100_000);
pub const CLK_LRC32_FREQ: Hertz = Hertz(32_000);
pub const CLK_LXT32_FREQ: Hertz = Hertz(32_768);
// NOTE: requires calibration to be accurate
pub const CLK_HRC48_FREQ: Hertz = Hertz(48_000_000);
pub const CLK_HXT48_FREQ: Hertz = Hertz(48_000_000);

// =============================================================================
// Clock Mux
// =============================================================================

/// Clock multiplexer configuration
#[non_exhaustive]
pub struct ClockMux {
    pub rtcsel: Rtcsel,
    pub wdtsel: Wdtsel,
    pub usbsel: Usbsel,
    pub perisel: Perisel,
    pub mpi1sel: Mpisel,
    pub mpi2sel: Mpisel,
    pub ticksel: Ticksel,
    pub lpsel: Lpsel,
}

impl Default for ClockMux {
    fn default() -> Self {
        Self {
            rtcsel: Rtcsel::Lrc10,
            wdtsel: Wdtsel::Lrc32,
            usbsel: Usbsel::Sysclk,
            perisel: Perisel::Hxt48,
            mpi1sel: Mpisel::Peri,
            mpi2sel: Mpisel::Dll2,
            ticksel: Ticksel::ClkRtc,
            lpsel: Lpsel::SelSys,
        }
    }
}

impl ClockMux {
    pub const fn new() -> Self {
        Self {
            rtcsel: Rtcsel::Lrc10,
            wdtsel: Wdtsel::Lrc32,
            usbsel: Usbsel::Sysclk,
            perisel: Perisel::Hxt48,
            mpi1sel: Mpisel::Peri,
            mpi2sel: Mpisel::Dll2,
            ticksel: Ticksel::ClkRtc,
            lpsel: Lpsel::SelSys,
        }
    }

    pub const fn with_rtcsel(mut self, rtcsel: Rtcsel) -> Self {
        self.rtcsel = rtcsel;
        self
    }

    pub const fn with_wdtsel(mut self, wdtsel: Wdtsel) -> Self {
        self.wdtsel = wdtsel;
        self
    }

    pub const fn with_usbsel(mut self, usbsel: Usbsel) -> Self {
        self.usbsel = usbsel;
        self
    }

    pub const fn with_perisel(mut self, perisel: Perisel) -> Self {
        self.perisel = perisel;
        self
    }

    pub const fn with_mpi1sel(mut self, mpi1sel: Mpisel) -> Self {
        self.mpi1sel = mpi1sel;
        self
    }

    pub const fn with_mpi2sel(mut self, mpi2sel: Mpisel) -> Self {
        self.mpi2sel = mpi2sel;
        self
    }

    pub const fn with_ticksel(mut self, ticksel: Ticksel) -> Self {
        self.ticksel = ticksel;
        self
    }

    pub const fn with_lpsel(mut self, lpsel: Lpsel) -> Self {
        self.lpsel = lpsel;
        self
    }
}

// =============================================================================
// Prescaler / DLL types and operator impls
// =============================================================================

/// hclk_hpsys = clk_hpsys / HDIV
/// if HDIV=0, hclk_hpsys = clk_hpsys
#[derive(Clone, Copy)]
pub struct HclkPrescaler(pub u8);

impl HclkPrescaler {
    pub const fn new(div: u8) -> Self {
        Self(div)
    }
}

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

#[allow(clippy::suspicious_arithmetic_impl)]
impl ops::Div<PclkPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: PclkPrescaler) -> Self::Output {
        Hertz(self.0 >> (rhs as u32))
    }
}

impl ops::Mul<DllStage> for Hertz {
    type Output = Hertz;
    fn mul(self, rhs: DllStage) -> Self::Output {
        Hertz(self.0 * (rhs as u32 + 1))
    }
}

// HPSYS_RCC.DLL1CR and HPSYS_RCC.DLL2CR
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Dll {
    // pub in_div2: bool,
    pub out_div2: bool,
    pub stg: DllStage,
}

impl Default for Dll {
    fn default() -> Self {
        Self {
            out_div2: false,
            stg: DllStage::Mul1,
        }
    }
}

impl Dll {
    pub const fn new() -> Self {
        Self {
            out_div2: false,
            stg: DllStage::Mul1,
        }
    }

    pub const fn freq_hz(&self) -> u32 {
        let base = 24_000_000 * (self.stg.to_bits() as u32 + 1);
        if self.out_div2 {
            base / 2
        } else {
            base
        }
    }

    pub const fn with_out_div2(mut self, out_div2: bool) -> Self {
        self.out_div2 = out_div2;
        self
    }

    pub const fn with_stg(mut self, stg: DllStage) -> Self {
        self.stg = stg;
        self
    }
}

// =============================================================================
// Clocks struct
// =============================================================================

/// Clocks configuration
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

impl Clocks {
    const ZERO: Self = Self {
        sysclk: MaybeHertz::NONE,
        hclk: MaybeHertz::NONE,
        pclk: MaybeHertz::NONE,
        pclk2: MaybeHertz::NONE,
        dll1: MaybeHertz::NONE,
        dll2: MaybeHertz::NONE,
        clk_peri: MaybeHertz::NONE,
        clk_peri_div2: MaybeHertz::NONE,
        clk_usb: MaybeHertz::NONE,
        clk_wdt: MaybeHertz::NONE,
        clk_rtc: MaybeHertz::NONE,
        clk_mpi1: MaybeHertz::NONE,
        clk_mpi2: MaybeHertz::NONE,
        clk_aud_pll: MaybeHertz::NONE,
        clk_aud_pll_div16: MaybeHertz::NONE,
    };
}
