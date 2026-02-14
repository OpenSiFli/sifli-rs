//! Hardware clock frequency read functions (HPSYS).
//!
//! These functions read the current clock frequencies directly from hardware registers.

use super::{
    Clocks, Mpisel, Sysclk, Usbsel, Wdtsel,
    CLK_HRC48_FREQ, CLK_HXT48_FREQ, CLK_LRC10_FREQ, CLK_LRC32_FREQ,
};
use crate::pac::hpsys_rcc::vals::mux::Perisel;
use crate::pac::{HPSYS_AON, HPSYS_RCC, PMUC};
use crate::time::Hertz;

/// Get current sysclk frequency from hardware registers.
pub(crate) fn get_clk_sys_freq() -> Option<Hertz> {
    match HPSYS_RCC.csr().read().sel_sys() {
        Sysclk::Hrc48 => get_hrc48_freq(),
        Sysclk::Hxt48 => get_hxt48_freq(),
        Sysclk::Dbl96 => None, // Not implemented
        Sysclk::Dll1 => get_clk_dll1_freq(),
    }
}

/// Get current HCLK frequency from hardware registers.
pub(crate) fn get_hclk_freq() -> Option<Hertz> {
    let clk_sys = get_clk_sys_freq()?;
    let hdiv = HPSYS_RCC.cfgr().read().hdiv();
    // HDIV=0 means no division (same as HDIV=1)
    if hdiv == 0 { Some(clk_sys) } else { Some(clk_sys / hdiv) }
}

/// Get current PCLK (APB1) frequency from hardware registers.
pub(crate) fn get_pclk_freq() -> Option<Hertz> {
    let hclk = get_hclk_freq()?;
    Some(hclk / (1u32 << HPSYS_RCC.cfgr().read().pdiv1().to_bits()))
}

/// Get current PCLK2 (APB2) frequency from hardware registers.
pub(crate) fn get_pclk2_freq() -> Option<Hertz> {
    let hclk = get_hclk_freq()?;
    Some(hclk / (1u32 << HPSYS_RCC.cfgr().read().pdiv2().to_bits()))
}

/// Get HXT48 status and frequency.
pub(crate) fn get_hxt48_freq() -> Option<Hertz> {
    if HPSYS_AON.acr().read().hxt48_rdy() {
        Some(Hertz(48_000_000))
    } else {
        None
    }
}

/// Get HRC48 status and frequency.
pub(crate) fn get_hrc48_freq() -> Option<Hertz> {
    if HPSYS_AON.acr().read().hrc48_rdy() {
        Some(Hertz(48_000_000))
    } else {
        None
    }
}

/// Get DLL1 frequency from hardware registers.
pub(crate) fn get_clk_dll1_freq() -> Option<Hertz> {
    let dllcr = HPSYS_RCC.dllcr(0).read();
    if dllcr.en() {
        Some(Hertz(
            24_000_000 * (dllcr.stg().to_bits() + 1) as u32 / (dllcr.out_div2_en() as u32 + 1),
        ))
    } else {
        None
    }
}

/// Get DLL2 frequency from hardware registers.
pub(crate) fn get_clk_dll2_freq() -> Option<Hertz> {
    let dllcr = HPSYS_RCC.dllcr(1).read();
    if dllcr.en() {
        Some(Hertz(
            24_000_000 * (dllcr.stg().to_bits() + 1) as u32 / (dllcr.out_div2_en() as u32 + 1),
        ))
    } else {
        None
    }
}

/// Get peripheral clock frequency from hardware registers.
pub(crate) fn get_clk_peri_freq() -> Option<Hertz> {
    Some(match HPSYS_RCC.csr().read().sel_peri() {
        Perisel::Hrc48 => CLK_HRC48_FREQ,
        Perisel::Hxt48 => CLK_HXT48_FREQ,
    })
}

/// Get peripheral clock / 2 frequency from hardware registers.
pub(crate) fn get_clk_peri_div2_freq() -> Option<Hertz> {
    get_clk_peri_freq().map(|f| f / 2u32)
}

/// Get USB clock frequency from hardware registers.
pub(crate) fn get_clk_usb_freq() -> Option<Hertz> {
    let source_freq = match get_clk_usb_source() {
        Usbsel::Sysclk => get_clk_sys_freq(),
        Usbsel::Dll2 => get_clk_dll2_freq(),
    };
    let usb_div = get_clk_usb_div();
    source_freq.map(|f| if usb_div > 0 { f / usb_div as u32 } else { f })
}

/// Get USB clock source from hardware registers.
pub(crate) fn get_clk_usb_source() -> Usbsel {
    HPSYS_RCC.csr().read().sel_usbc()
}

/// Get USB clock divider from hardware registers.
pub(crate) fn get_clk_usb_div() -> u8 {
    HPSYS_RCC.usbcr().read().div()
}

/// Get WDT clock frequency from hardware registers.
pub(crate) fn get_clk_wdt_freq() -> Option<Hertz> {
    Some(match PMUC.cr().read().sel_lpclk() {
        Wdtsel::Lrc10 => CLK_LRC10_FREQ,
        Wdtsel::Lrc32 => CLK_LRC32_FREQ,
    })
}

/// Get RTC clock frequency from hardware registers.
pub(crate) fn get_clk_rtc_freq() -> Option<Hertz> {
    // TODO: read from RTC.CR.LPCKSEL when available
    Some(CLK_LRC10_FREQ)
}

/// Get MPI1 clock frequency from hardware registers.
pub(crate) fn get_clk_mpi1_freq() -> Option<Hertz> {
    match HPSYS_RCC.csr().read().sel_mpi1() {
        Mpisel::Peri => get_clk_peri_freq(),
        Mpisel::Dll2 => get_clk_dll2_freq(),
        Mpisel::Dll1 => get_clk_dll1_freq(),
        _ => Some(CLK_HXT48_FREQ), // Reserved values, fallback
    }
}

/// Get MPI2 clock frequency from hardware registers.
pub(crate) fn get_clk_mpi2_freq() -> Option<Hertz> {
    match HPSYS_RCC.csr().read().sel_mpi2() {
        Mpisel::Peri => get_clk_peri_freq(),
        Mpisel::Dll2 => get_clk_dll2_freq(),
        Mpisel::Dll1 => get_clk_dll1_freq(),
        _ => Some(CLK_HXT48_FREQ), // Reserved values, fallback
    }
}

/// Read current HPSYS clock frequencies from hardware and build a `Clocks` struct.
///
/// Used by both `init()` and `reconfigure_sysclk()` to build `Clocks` from hardware state.
pub(crate) fn read_hpsys_clocks_from_hw() -> Clocks {
    Clocks {
        sysclk: get_clk_sys_freq().into(),
        hclk: get_hclk_freq().into(),
        pclk: get_pclk_freq().into(),
        pclk2: get_pclk2_freq().into(),
        dll1: get_clk_dll1_freq().into(),
        dll2: get_clk_dll2_freq().into(),
        clk_peri: get_clk_peri_freq().into(),
        clk_peri_div2: get_clk_peri_div2_freq().into(),
        clk_usb: get_clk_usb_freq().into(),
        clk_wdt: get_clk_wdt_freq().into(),
        clk_rtc: get_clk_rtc_freq().into(),
        clk_mpi1: get_clk_mpi1_freq().into(),
        clk_mpi2: get_clk_mpi2_freq().into(),
        // Audio PLL is managed by AUDCODEC driver, default to None here
        clk_aud_pll: None.into(),
        clk_aud_pll_div16: None.into(),
    }
}
