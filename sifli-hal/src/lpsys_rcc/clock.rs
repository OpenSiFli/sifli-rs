use crate::pac::LPSYS_RCC;
use crate::rcc::{get_hxt48_freq, get_hrc48_freq};
use crate::time::Hertz;

// Reuse PAC enums directly so values match register fields.
pub use crate::pac::lpsys_rcc::vals::{
    SelPeri as ClkPeriSel,
    SelSys as ClkSysSel,
    SelTick as TickSel,
};

/// High-level view of the LPSYS main system clock source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LpsysSysClkSel {
    /// 48 MHz internal RC oscillator.
    Hrc48,
    /// 48 MHz external crystal.
    Hxt48,
    /// Low-speed WDT domain clock.
    Wdt,
}

#[cfg(feature = "defmt")]
impl defmt::Format for LpsysSysClkSel {
    fn format(&self, f: defmt::Formatter) {
        match self {
            LpsysSysClkSel::Hrc48 => defmt::write!(f, "Hrc48"),
            LpsysSysClkSel::Hxt48 => defmt::write!(f, "Hxt48"),
            LpsysSysClkSel::Wdt => defmt::write!(f, "Wdt"),
        }
    }
}

fn get_clk_wdt_freq() -> Option<Hertz> {
    // TODO: derive WDT clock frequency from actual hardware.
    // Keep in sync with the approximation used in `lcpu::check_lcpu_frequency`.
    Some(Hertz(32_768))
}

/// Get LPSYS main clock source (including WDT domain).
pub fn get_clk_lpsys_source() -> LpsysSysClkSel {
    let csr = LPSYS_RCC.csr().read();

    if csr.sel_sys_lp() {
        LpsysSysClkSel::Wdt
    } else {
        match csr.sel_sys() {
            ClkSysSel::Hrc48 => LpsysSysClkSel::Hrc48,
            ClkSysSel::Hxt48 => LpsysSysClkSel::Hxt48,
        }
    }
}

/// Get LPSYS main clock frequency (clk_lpsys).
pub fn get_clk_lpsys_freq() -> Option<Hertz> {
    match get_clk_lpsys_source() {
        LpsysSysClkSel::Hrc48 => get_hrc48_freq(),
        LpsysSysClkSel::Hxt48 => get_hxt48_freq(),
        LpsysSysClkSel::Wdt => get_clk_wdt_freq(),
    }
}

/// Get LPSYS HCLK divider (HDIV1).
pub fn get_hclk_lpsys_div() -> u8 {
    LPSYS_RCC.cfgr().read().hdiv1()
}

/// Get LPSYS HCLK frequency (hclk_lpsys).
pub fn get_hclk_lpsys_freq() -> Option<Hertz> {
    let clk_lpsys = get_clk_lpsys_freq()?;
    let div = get_hclk_lpsys_div();

    if div == 0 {
        Some(clk_lpsys)
    } else {
        Some(clk_lpsys / div as u32)
    }
}

/// Get LPSYS PCLK1 divider (PDIV1).
pub fn get_pclk1_lpsys_div() -> u8 {
    LPSYS_RCC.cfgr().read().pdiv1()
}

/// Get LPSYS PCLK2 divider (PDIV2).
pub fn get_pclk2_lpsys_div() -> u8 {
    LPSYS_RCC.cfgr().read().pdiv2()
}

/// Get LPSYS PCLK1 frequency.
pub fn get_pclk1_lpsys_freq() -> Option<Hertz> {
    let hclk = get_hclk_lpsys_freq()?;
    Some(hclk / (1 << get_pclk1_lpsys_div()) as u32)
}

/// Get LPSYS PCLK2 frequency.
pub fn get_pclk2_lpsys_freq() -> Option<Hertz> {
    let hclk = get_hclk_lpsys_freq()?;
    Some(hclk / (1 << get_pclk2_lpsys_div()) as u32)
}

/// Get peripheral clock source for LPSYS (clk_peri_lpsys).
pub fn get_clk_peri_lpsys_source() -> ClkPeriSel {
    LPSYS_RCC.csr().read().sel_peri()
}

/// Get peripheral clock frequency for LPSYS (clk_peri_lpsys).
pub fn get_clk_peri_lpsys_freq() -> Option<Hertz> {
    match get_clk_peri_lpsys_source() {
        ClkPeriSel::Hrc48 => get_hrc48_freq(),
        ClkPeriSel::Hxt48 => get_hxt48_freq(),
    }
}

/// Get BT MAC clock frequency (MACCLK).
pub fn get_macclk_freq() -> Option<Hertz> {
    let hclk = get_hclk_lpsys_freq()?;
    let div = LPSYS_RCC.cfgr().read().macdiv();

    if div == 0 {
        None
    } else {
        Some(hclk / div as u32)
    }
}

/// Set/clear the LCPU reset bit (`LPSYS_RCC.RSTR1.LCPU`).
///
/// Primitive write to RSTR1; does not include any waits or side effects.
#[inline]
pub fn set_lcpu_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_lcpu(enable));
}

/// Set/clear the BT MAC reset bit (`LPSYS_RCC.RSTR1.MAC`).
///
/// Primitive write to RSTR1; does not include any waits or side effects.
#[inline]
pub fn set_mac_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_mac(enable));
}

/// Set/clear the BT RF reset bit (`LPSYS_RCC.RSTR1.RFC`).
///
/// Primitive write to RSTR1; does not include any waits or side effects.
#[inline]
pub fn set_rfc_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_rfc(enable));
}

/// Check if the BT RF reset bit (`LPSYS_RCC.RSTR1.RFC`) is set.
#[inline]
pub fn get_rfc_reset() -> bool {
    LPSYS_RCC.rstr1().read().rfc()
}

/// Get the raw value of the RSTR1 register.
#[inline]
pub fn get_rstr1() -> u32 {
    LPSYS_RCC.rstr1().read().0
}

/// Get the LPSYS systick clock source.
pub fn get_lpsys_tick_source() -> TickSel {
    LPSYS_RCC.csr().read().sel_tick()
}

/// Get the LPSYS systick divider.
pub fn get_lpsys_tick_div() -> u8 {
    LPSYS_RCC.cfgr().read().tickdiv()
}

/// Get the LPSYS systick reference clock frequency.
pub fn get_lpsys_tick_freq() -> Option<Hertz> {
    let base = match get_lpsys_tick_source() {
        // RTC frequency is not fully modeled in this HAL yet; return None for now.
        TickSel::ClkRtc => return None,
        TickSel::Hrc48 => get_hrc48_freq()?,
        TickSel::Hxt48 => get_hxt48_freq()?,
        _ => return None,
    };

    let div = get_lpsys_tick_div();
    if div == 0 {
        Some(base)
    } else {
        Some(base / div as u32)
    }
}

/// Log LPSYS-related clock frequencies for debugging.
pub fn test_print_clocks() {
    info!("LPSYS Clock frequencies:");

    let clocks = [
        ("clk_lpsys", get_clk_lpsys_freq()),
        ("hclk_lpsys", get_hclk_lpsys_freq()),
        ("pclk1_lpsys", get_pclk1_lpsys_freq()),
        ("pclk2_lpsys", get_pclk2_lpsys_freq()),
        ("clk_peri_lpsys", get_clk_peri_lpsys_freq()),
        ("macclk", get_macclk_freq()),
        ("lp_tick", get_lpsys_tick_freq()),
    ];

    for (name, freq) in clocks {
        if let Some(f) = freq {
            let freq_khz = f.0 / 1_000;
            let mhz_part = freq_khz / 1_000;
            let khz_part = freq_khz % 1_000;

            if khz_part == 0 {
                info!("  - {}: {} MHz", name, mhz_part);
            } else if mhz_part == 0 {
                info!("  - {}: {} kHz", name, khz_part);
            } else {
                info!("  - {}: {}.{:03} MHz", name, mhz_part, khz_part);
            }
        } else {
            info!("  - {}: disabled/unknown", name);
        }
    }
}
