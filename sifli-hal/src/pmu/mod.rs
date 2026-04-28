pub mod dvfs;

use crate::efuse::Bank1Calibration;
use crate::pac::PMUC;
use crate::syscfg;

/// Apply factory PMU calibration values from EFUSE bank1 to the PMU registers.
///
/// Equivalent to `HAL_PMU_LoadCalData()` from the SDK
/// (drivers/hal/bf0_hal_pmu.c). Without this, the BUCK / LPSYS_LDO / VRET /
/// PERI_LDO / AON_BG run on chip-default voltages, which leaves the RF
/// front-end supplies slightly off-spec — HCI commands will succeed but
/// advertising packets may not actually reach the air.
///
/// Returns `false` if the EFUSE bank looks blank (`hpsys_ldo_vout == 0`),
/// meaning the part was never ATE-calibrated; in that case nothing is
/// written and the caller should continue on defaults.
pub fn apply_calibration(cal: &Bank1Calibration) -> bool {
    let p = &cal.primary.low;

    // Sanity: SDK refuses to write when the canonical hpsys_ldo_vout slot
    // reads 0 — it means the OTP block was never programmed.
    if p.hpsys_ldo_vout() == 0 {
        return false;
    }

    let is_letter = syscfg::read_idr().revision().is_letter_series();

    // BUCK_CR1: BG_BUF_VOS_POLAR + BG_BUF_VOS_TRIM
    PMUC.buck_cr1().modify(|w| {
        w.set_bg_buf_vos_polar(p.buck_vos_polar());
        w.set_bg_buf_vos_trim(p.buck_vos_trim());
    });

    // LPSYS_VOUT (D-mode): VOUT
    PMUC.lpsys_vout().modify(|w| {
        w.set_vout(p.lpsys_ldo_vout());
    });

    // VRET_CR: TRIM
    PMUC.vret_cr().modify(|w| {
        w.set_trim(p.vret_trim());
    });

    if !is_letter {
        // PERI_LDO: LDO18_VREF_SEL + VDD33_LDO2_SET_VOUT + VDD33_LDO3_SET_VOUT
        PMUC.peri_ldo().modify(|w| {
            w.set_ldo18_vref_sel(p.ldo18_vref_sel());
            w.set_vdd33_ldo2_set_vout(p.vdd33_ldo2_vout());
            w.set_vdd33_ldo3_set_vout(p.vdd33_ldo3_vout());
        });

        // AON_BG: BUF_VOS_POLAR + BUF_VOS_TRIM
        PMUC.aon_bg().modify(|w| {
            w.set_buf_vos_polar(p.aon_vos_polar());
            w.set_buf_vos_trim(p.aon_vos_trim());
        });
    } else {
        // Letter series: only LDO18_VREF_SEL touched (when not running on PVDD 1V8).
        PMUC.peri_ldo().modify(|w| {
            w.set_ldo18_vref_sel(p.ldo18_vref_sel());
        });
    }

    true
}
