//! Hardware clock frequency read functions (HPSYS).
//!
//! These functions read the current clock frequencies directly from hardware registers.

use super::{Sysclk, Usbsel};
use crate::pac::{HPSYS_AON, HPSYS_RCC};
use crate::time::Hertz;

/// Get current HCLK frequency from hardware registers.
///
/// This is used internally during initialization before `clocks()` is available.
pub(crate) fn get_hclk_freq() -> Option<Hertz> {
    let clk_sys = get_clk_sys_freq()?;
    let hdiv = HPSYS_RCC.cfgr().read().hdiv();
    // HDIV=0 means no division (same as HDIV=1)
    if hdiv == 0 { Some(clk_sys) } else { Some(clk_sys / hdiv) }
}

pub(crate) fn get_pclk_freq() -> Option<Hertz> {
    let hclk = get_hclk_freq()?;
    Some(hclk / (1u32 << HPSYS_RCC.cfgr().read().pdiv1().to_bits()))
}

/// Get current sysclk frequency from hardware registers.
pub(crate) fn get_clk_sys_freq() -> Option<Hertz> {
    match HPSYS_RCC.csr().read().sel_sys() {
        Sysclk::Hrc48 => get_hrc48_freq(),
        Sysclk::Hxt48 => get_hxt48_freq(),
        Sysclk::Dbl96 => None, // Not implemented
        Sysclk::Dll1 => get_clk_dll1_freq(),
    }
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

/// Get USB clock source from hardware registers.
pub(crate) fn get_clk_usb_source() -> Usbsel {
    HPSYS_RCC.csr().read().sel_usbc()
}

/// Get USB clock divider from hardware registers.
pub(crate) fn get_clk_usb_div() -> u8 {
    HPSYS_RCC.usbcr().read().div()
}
