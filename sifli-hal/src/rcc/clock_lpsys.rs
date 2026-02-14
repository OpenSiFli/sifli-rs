//! LPSYS/LCPU clock management.
//!
//! All functions related to the Low-Power Subsystem (LPSYS) clock configuration,
//! frequency reading, reset control, and LCPU wake management.

use super::{get_hrc48_freq, get_hxt48_freq};
use crate::pac::lpsys_rcc::vals as lpsys_vals;
use crate::pac::{HPSYS_AON, LPSYS_RCC};
use crate::time::Hertz;
use core::sync::atomic::{AtomicU8, Ordering};

/// LCPU wakeup reference counter (SF32LB52X specific).
/// Used to track nested wakeup requests.
static LCPU_WAKEUP_REF_COUNT: AtomicU8 = AtomicU8::new(0);

// =============================================================================
// LPSYS Frequency Read
// =============================================================================

/// Read LPSYS HCLK divider (HDIV1).
pub fn get_lpsys_hclk_div() -> u8 {
    LPSYS_RCC.cfgr().read().hdiv1().to_bits()
}

/// Get LPSYS system clock frequency (clk_lpsys).
pub fn get_lpsys_sysclk_freq() -> Option<Hertz> {
    let csr = LPSYS_RCC.csr().read();
    match csr.sel_sys() {
        lpsys_vals::Sysclk::Hrc48 => get_hrc48_freq(),
        lpsys_vals::Sysclk::Hxt48 => get_hxt48_freq(),
    }
}

/// Get current LPSYS HCLK frequency from hardware registers.
pub fn get_lpsys_hclk_freq() -> Option<Hertz> {
    let clk_lpsys = get_lpsys_sysclk_freq()?;

    let hdiv = LPSYS_RCC.cfgr().read().hdiv1().to_bits();
    let divisor = if hdiv == 0 { 1 } else { hdiv as u32 };

    Some(clk_lpsys / divisor)
}

/// Get LPSYS PCLK1 frequency.
pub fn get_lpsys_pclk1_freq() -> Option<Hertz> {
    let hclk = get_lpsys_hclk_freq()?;
    let pdiv1 = LPSYS_RCC.cfgr().read().pdiv1().to_bits();
    Some(hclk / (1u32 << pdiv1))
}

/// Get LPSYS PCLK2 frequency.
pub fn get_lpsys_pclk2_freq() -> Option<Hertz> {
    let hclk = get_lpsys_hclk_freq()?;
    let pdiv2 = LPSYS_RCC.cfgr().read().pdiv2().to_bits();
    Some(hclk / (1u32 << pdiv2))
}

/// Get LPSYS peripheral clock frequency (clk_peri_lpsys).
pub fn get_lpsys_clk_peri_freq() -> Option<Hertz> {
    use crate::pac::lpsys_rcc::vals::mux::Perisel;
    let csr = LPSYS_RCC.csr().read();
    match csr.sel_peri() {
        Perisel::Hrc48 => get_hrc48_freq(),
        Perisel::Hxt48 => get_hxt48_freq(),
    }
}

/// Get LPSYS MAC clock frequency.
pub fn get_lpsys_mac_clk_freq() -> Option<Hertz> {
    let hclk = get_lpsys_hclk_freq()?;
    let cfgr = LPSYS_RCC.cfgr().read();
    let macdiv = cfgr.macdiv().to_bits();
    if macdiv == 0 {
        Some(hclk)
    } else {
        Some(hclk / macdiv as u32)
    }
}

// =============================================================================
// LPSYS Reset Control
// =============================================================================

/// Assert or deassert LCPU reset in LPSYS domain.
pub fn set_lp_lcpu_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_lcpu(enable));
}

/// Assert or deassert MAC reset in LPSYS domain.
pub fn set_lp_mac_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_mac(enable));
}

/// Assert or deassert RFC reset in LPSYS domain.
pub fn set_lp_rfc_reset(enable: bool) {
    LPSYS_RCC.rstr1().modify(|w| w.set_rfc(enable));
}

/// Check whether LCPU reset bit is asserted.
pub fn lp_lcpu_reset_asserted() -> bool {
    LPSYS_RCC.rstr1().read().lcpu()
}

/// Check whether MAC reset bit is asserted.
pub fn lp_mac_reset_asserted() -> bool {
    LPSYS_RCC.rstr1().read().mac()
}

/// Check whether RFC reset bit is asserted.
pub fn lp_rfc_reset_asserted() -> bool {
    LPSYS_RCC.rstr1().read().rfc()
}

// =============================================================================
// LPSYS Clock Configuration API
// =============================================================================

/// Configure LPSYS HCLK frequency (in MHz).
///
/// This function automatically handles DVFS mode transitions when crossing
/// the 24MHz boundary between D mode and S mode.
///
/// # Arguments
///
/// * `freq_mhz` - Target frequency in MHz (1-48)
///
/// # Returns
///
/// * `Ok(())` - Configuration successful
/// * `Err(&str)` - Error message if configuration fails
///
/// # Safety
///
/// This function modifies hardware registers and should be called with care.
/// Ensure no LPSYS peripherals are actively in use during frequency changes.
pub unsafe fn config_lpsys_hclk_mhz(freq_mhz: u32) -> Result<(), &'static str> {
    use crate::pmu::dvfs::config_lpsys_dvfs;

    // Validate frequency range
    if freq_mhz == 0 || freq_mhz > 48 {
        return Err("LPSYS HCLK frequency must be 1-48 MHz");
    }

    let target_freq = Hertz(freq_mhz * 1_000_000);
    let current_freq = get_lpsys_hclk_freq().unwrap_or(Hertz(48_000_000));

    // Calculate divider (LPSYS sysclk is 48MHz from HRC48 or HXT48)
    let sysclk_freq = get_lpsys_sysclk_freq().unwrap_or(Hertz(48_000_000));
    let hdiv = if target_freq >= sysclk_freq {
        0 // No division
    } else {
        (sysclk_freq.0 / target_freq.0) as u8
    };

    // Use DVFS API to handle voltage transitions
    config_lpsys_dvfs(current_freq, target_freq, || {
        set_lpsys_hdiv(hdiv);
    });

    Ok(())
}

/// Set LPSYS HCLK divider (HDIV1).
///
/// hclk_lpsys = clk_lpsys / HDIV (if HDIV=0, no division)
///
/// # Arguments
///
/// * `hdiv` - Divider value (0 = no division, 1-63 = divide by hdiv)
pub fn set_lpsys_hdiv(hdiv: u8) {
    LPSYS_RCC.cfgr().modify(|w| {
        w.set_hdiv1(lpsys_vals::Hdiv::from_bits(hdiv));
    });
}

/// Set LPSYS clock dividers.
///
/// # Arguments
///
/// * `hdiv` - HCLK divider (0 = no division, 1-63)
/// * `pdiv1` - PCLK1 prescaler (0-7, divides by 2^pdiv1)
/// * `pdiv2` - PCLK2 prescaler (0-7, divides by 2^pdiv2)
pub fn set_lpsys_div(hdiv: u8, pdiv1: u8, pdiv2: u8) {
    LPSYS_RCC.cfgr().modify(|w| {
        w.set_hdiv1(lpsys_vals::Hdiv::from_bits(hdiv));
        w.set_pdiv1(lpsys_vals::Pdiv::from_bits(pdiv1));
        w.set_pdiv2(lpsys_vals::Pdiv::from_bits(pdiv2));
    });
}

/// Configure LPSYS MAC clock.
///
/// # Arguments
///
/// * `macdiv` - MAC clock divider (MACCLK = hclk_lpsys / MACDIV)
/// * `macfreq` - MAC frequency hint (for hardware use)
pub fn config_lpsys_mac_clock(macdiv: u8, macfreq: u8) {
    LPSYS_RCC.cfgr().modify(|w| {
        w.set_macdiv(lpsys_vals::Macdiv::from_bits(macdiv));
        w.set_macfreq(macfreq);
    });
}

/// Select LPSYS system clock source.
///
/// # Arguments
///
/// * `source` - Clock source (HRC48 or HXT48)
pub fn select_lpsys_sysclk(source: lpsys_vals::Sysclk) {
    LPSYS_RCC.csr().modify(|w| {
        w.set_sel_sys(source);
    });
}

// =============================================================================
// LCPU Wake Management
// =============================================================================

/// Wake up LCPU
///
/// Sends wakeup request to LCPU and waits for acknowledgment.
/// Maintains reference counter for nested calls (SF32LB52X specific).
///
/// # Safety
/// Must be called with HPSYS_AON peripheral access. Caller must ensure
/// paired calls with [`cancel_lcpu_active_request`].
pub unsafe fn wake_lcpu() {
    // Set HP2LP_REQ bit
    HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(true));

    // Wait for LCPU to see the request and respond
    crate::cortex_m_blocking_delay_us(230);
    while !HPSYS_AON.issr().read().lp_active() {}
    crate::cortex_m_blocking_delay_us(30);
    while !HPSYS_AON.issr().read().lp_active() {}

    // Increment reference counter
    LCPU_WAKEUP_REF_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// Cancel LCPU active request (paired with wake_lcpu)
///
/// # Safety
/// Must be paired with a prior [`wake_lcpu`] call. Caller must ensure
/// the reference count does not underflow.
pub unsafe fn cancel_lcpu_active_request() {
    let count = LCPU_WAKEUP_REF_COUNT.fetch_sub(1, Ordering::Relaxed);

    // Clear HP2LP_REQ when count reaches 0
    if count == 1 {
        HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(false));
    }
}

// =============================================================================
// LCPU Frequency Safety
// =============================================================================

/// Ensure LPSYS HCLK stays ≤ 24 MHz while loading the LCPU image.
///
/// If the current frequency exceeds the limit, this function will automatically
/// reduce it to 24 MHz. The original frequency is returned so the caller can
/// optionally restore it later.
///
/// Mirrors `bf0_lcpu_init.c:170-176` in the SDK.
///
/// # Returns
///
/// The original HCLK frequency before any changes (in Hz).
pub fn ensure_safe_lcpu_frequency() -> Result<u32, &'static str> {
    const MAX_LOAD_FREQ_HZ: u32 = 24_000_000;
    const MAX_LOAD_FREQ_MHZ: u32 = 24;

    // 1. Compute current LPSYS HCLK frequency.
    let hclk_hz = get_lpsys_hclk_freq()
        .ok_or("failed to read LPSYS HCLK frequency")?
        .0;

    // 2. If exceeds limit, automatically reduce to safe frequency.
    if hclk_hz > MAX_LOAD_FREQ_HZ {
        debug!(
            "LPSYS HCLK {} Hz exceeds {} Hz limit, reducing to {} MHz for LCPU loading",
            hclk_hz, MAX_LOAD_FREQ_HZ, MAX_LOAD_FREQ_MHZ
        );

        unsafe {
            config_lpsys_hclk_mhz(MAX_LOAD_FREQ_MHZ)?;
        }

        // Verify the change took effect
        let new_hclk_hz = get_lpsys_hclk_freq()
            .ok_or("failed to read LPSYS HCLK after frequency change")?
            .0;
        debug!(
            "LPSYS HCLK reduced to {} Hz (was {} Hz)",
            new_hclk_hz, hclk_hz
        );
    } else {
        let hdiv = get_lpsys_hclk_div();
        debug!(
            "LPSYS HCLK within limit for LCPU loading: {} Hz (HDIV1={})",
            hclk_hz, hdiv
        );
    }

    Ok(hclk_hz)
}
