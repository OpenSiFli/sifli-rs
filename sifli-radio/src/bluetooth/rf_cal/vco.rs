//! VCO (Voltage-Controlled Oscillator) calibration for Bluetooth RF.
//!
//! This module implements full 79-channel VCO frequency calibration for the Bluetooth RF
//! subsystem. Based on SDK `bt_rfc_lo_cal()` in `bt_rf_fulcal.c`.
//!
//! The calibration consists of:
//! - Binary search for initial capcode near high frequency boundary
//! - Linear sweep from high to low frequency (incrementing capcode)
//! - Sequential ACAL (amplitude calibration) at each sweep step
//! - Channel matching: find best sweep point for each of 79 TX / 40 BLE RX / 79 BT RX channels
//! - KCAL: frequency compensation coefficient per TX channel
//! - PACAL: PA bias calibration
//! - ROSCAL: RX DC offset calibration
//! - RCCAL: RC oscillator calibration

use crate::pac::{BT_PHY, BT_RFC};

use super::consts::*;

// ============================================================
// VCO 5GHz calibration constants
// ============================================================

/// Residual count upper threshold for 5GHz VCO binary search.
const RESIDUAL_CNT_VTH: u32 = 33864;

/// Residual count lower threshold for 5GHz VCO linear sweep termination.
const RESIDUAL_CNT_VTL: u32 = 30224;

/// FKCAL counter divider for 5GHz VCO calibration.
const FKCAL_DIVN_5G: u16 = 7680;

/// FKCAL counter divider for KCAL computation.
const FKCAL_DIVN_KCAL: u16 = 17280;

/// KCAL normalization constant (216 * 2048).
const KCAL_CONST_A: u32 = 216 * 2048;

/// KCAL frequency correction factor for channels 0-39 (pivot ch19).
const KCAL_CONST_B_LOW: f32 = 1.0 / 96800.0;

/// KCAL frequency correction factor for channels 40-78 (pivot ch59).
const KCAL_CONST_B_HIGH: f32 = 1.0 / 98400.0;

/// KCAL fallback normalization value when pmax <= pmin.
const KCAL_DEFAULT_NORM: u32 = 200;

/// KCAL frequency correction multiplier used in the per-channel factor formula.
const KCAL_FREQ_MULTIPLIER: f32 = 3.0;

/// HFP_FCW minimum value (for KCAL pmin measurement).
const HFP_FCW_MIN: u8 = 0x00;

/// HFP_FCW maximum value (for KCAL pmax measurement).
const HFP_FCW_MAX: u8 = 0x3F;

/// PA LDO voltage reference selection for PACAL.
const PA_LDO_VREF_SEL: u8 = 0x0E;

/// PA unit selection for PACAL.
const PA_UNIT_SEL: u8 = 0x01;

/// PA buffer load selection for PACAL.
const PA_BUFLOAD_SEL: u8 = 1;

/// BLE RX 1M reference residual counts (40 channels)
static REF_RESIDUAL_CNT_TBL_RX_1M: [u16; 40] = [
    30485, 30565, 30645, 30725, 30805, 30885, 30965, 31045, 31125, 31205, 31285, 31365, 31445,
    31525, 31605, 31685, 31765, 31845, 31925, 32005, 32085, 32165, 32245, 32325, 32405, 32485,
    32565, 32645, 32725, 32805, 32885, 32965, 33045, 33125, 33205, 33285, 33365, 33445, 33525,
    33605,
];

/// BLE RX 2M reference residual counts (40 channels)
static REF_RESIDUAL_CNT_TBL_RX_2M: [u16; 40] = [
    30425, 30505, 30585, 30665, 30745, 30825, 30905, 30985, 31065, 31145, 31225, 31305, 31385,
    31465, 31545, 31625, 31705, 31785, 31865, 31945, 32025, 32105, 32185, 32265, 32345, 32425,
    32505, 32585, 32665, 32745, 32825, 32905, 32985, 33065, 33145, 33225, 33305, 33385, 33465,
    33545,
];

/// BT RX reference residual counts (79 channels)
static REF_RESIDUAL_CNT_TBL_RX_BT: [u16; 79] = [
    30445, 30485, 30525, 30565, 30605, 30645, 30685, 30725, 30765, 30805, 30845, 30885, 30925,
    30965, 31005, 31045, 31085, 31125, 31165, 31205, 31245, 31285, 31325, 31365, 31405, 31445,
    31485, 31525, 31565, 31605, 31645, 31685, 31725, 31765, 31805, 31845, 31885, 31925, 31965,
    32005, 32045, 32085, 32125, 32165, 32205, 32245, 32285, 32325, 32365, 32405, 32445, 32485,
    32525, 32565, 32605, 32645, 32685, 32725, 32765, 32805, 32845, 32885, 32925, 32965, 33005,
    33045, 33085, 33125, 33165, 33205, 33245, 33285, 33325, 33365, 33405, 33445, 33485, 33525,
    33565,
];

/// TX reference residual counts (79 channels, used for both BLE TX and BT TX)
static REF_RESIDUAL_CNT_TBL_TX: [u16; 79] = [
    30545, 30585, 30625, 30665, 30705, 30745, 30785, 30825, 30865, 30905, 30945, 30985, 31025,
    31065, 31105, 31145, 31185, 31225, 31265, 31305, 31345, 31385, 31425, 31465, 31505, 31545,
    31585, 31625, 31665, 31705, 31745, 31785, 31825, 31865, 31905, 31945, 31985, 32025, 32065,
    32105, 32145, 32185, 32225, 32265, 32305, 32345, 32385, 32425, 32465, 32505, 32545, 32585,
    32625, 32665, 32705, 32745, 32785, 32825, 32865, 32905, 32945, 32985, 33025, 33065, 33105,
    33145, 33185, 33225, 33265, 33305, 33345, 33385, 33425, 33465, 33505, 33545, 33585, 33625,
    33665,
];

// ============================================================
// Result types
// ============================================================

/// Full 79-channel VCO calibration result.
pub struct VcoCalResult {
    /// TX IDAC per channel (79)
    pub idac_tx: [u8; 79],
    /// TX capcode per channel (79)
    pub capcode_tx: [u8; 79],
    /// TX KCAL per channel (79)
    pub kcal: [u16; 79],
    /// BLE RX 1M IDAC per channel (40)
    pub idac_rx_1m: [u8; 40],
    /// BLE RX 1M capcode per channel (40)
    pub capcode_rx_1m: [u8; 40],
    /// BLE RX 2M IDAC per channel (40)
    pub idac_rx_2m: [u8; 40],
    /// BLE RX 2M capcode per channel (40)
    pub capcode_rx_2m: [u8; 40],
    /// BT RX IDAC per channel (79)
    pub idac_rx_bt: [u8; 79],
    /// BT RX capcode per channel (79)
    pub capcode_rx_bt: [u8; 79],
}

// ============================================================
// Low-level helpers
// ============================================================

/// Initialize RFC hardware for VCO calibration.
/// Based on SDK `bt_rfc_init()`.
pub fn rfc_init() {
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_adc_q_en_1(true);
    });
    BT_PHY.rx_ctrl2().modify(|w| {
        w.set_adc_q_en_frc_en(true);
    });
    BT_PHY.tx_if_mod_cfg().modify(|w| {
        w.set_tx_if_phase_ble(0);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_rstb_rccal_lv(false);
    });
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_rstb_adc_lv(true);
    });
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_pkdet_en_early_off_en(false);
    });
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mmdiv_sel(true);
    });
}

/// Read FBDV frequency counter value.
/// Resets counter, enables it, waits for ready, returns result.
pub(super) fn get_fbdv_cnt() -> u32 {
    crate::cortex_m_blocking_delay_us(4);
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_en_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_rstb_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_rstb_lv(true);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_en_lv(true);
    });
    crate::cortex_m_blocking_delay_us(10);
    // Poll for ready with timeout
    for _ in 0..10_000 {
        if BT_RFC.fbdv_reg1().read().brf_fkcal_cnt_rdy_lv() {
            break;
        }
    }
    BT_RFC.fbdv_reg2().read().brf_fkcal_cnt_op_lv() as u32
}

/// ACAL binary search (used during initial binary FCAL).
/// Returns calibrated IDAC value.
fn acal_binary_search() -> u8 {
    let mut acal_cnt: u8 = IDAC_INITIAL;
    let acal_cnt_fs: u8 = IDAC_FS;

    BT_RFC.vco_reg3().modify(|w| {
        w.set_brf_vco_idac_lv(acal_cnt);
    });
    crate::cortex_m_blocking_delay_us(4);
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(true);
    });

    for j in 1..7u32 {
        if !BT_RFC.vco_reg2().read().brf_vco5g_acal_incal_lv() {
            break;
        }
        let step = ((acal_cnt_fs as u32) >> j) as u8;
        if !BT_RFC.vco_reg2().read().brf_vco5g_acal_up_lv() {
            acal_cnt = acal_cnt.saturating_sub(step);
        } else {
            acal_cnt = acal_cnt.saturating_add(step);
        }
        BT_RFC.vco_reg3().modify(|w| {
            w.set_brf_vco_idac_lv(acal_cnt);
        });
        crate::cortex_m_blocking_delay_us(1);
    }
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(false);
    });
    acal_cnt
}

/// Sequential ACAL (used during linear sweep).
/// Adjusts IDAC by +/-1 until stable. Returns updated IDAC.
fn acal_sequential(mut acal_cnt: u8) -> u8 {
    let mut seq_acal_jump_cnt: u8 = 0;
    let mut seq_acal_ful_cnt: u8 = 0;
    let mut pre_acal_up_vld: bool = false;
    let mut pre_acal_up: bool = false;

    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(true);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(true);
    });

    while seq_acal_jump_cnt < SEQ_ACAL_JUMP_LIMIT && seq_acal_ful_cnt < SEQ_ACAL_FULL_LIMIT {
        BT_RFC.vco_reg3().modify(|w| {
            w.set_brf_vco_idac_lv(acal_cnt);
        });
        crate::cortex_m_blocking_delay_us(4);

        if !BT_RFC.vco_reg2().read().brf_vco5g_acal_incal_lv() {
            break;
        }
        let curr_acal_up = BT_RFC.vco_reg2().read().brf_vco5g_acal_up_lv();

        if !curr_acal_up {
            if acal_cnt > 0 {
                acal_cnt -= 1;
                seq_acal_ful_cnt = 0;
            } else {
                seq_acal_ful_cnt += 1;
            }
        } else if acal_cnt < IDAC_MAX {
            acal_cnt += 1;
            seq_acal_ful_cnt = 0;
        } else {
            seq_acal_ful_cnt += 1;
            acal_cnt = IDAC_MAX;
        }

        if pre_acal_up_vld {
            if pre_acal_up == curr_acal_up {
                seq_acal_jump_cnt = 0;
            } else {
                seq_acal_jump_cnt += 1;
            }
        }
        pre_acal_up = curr_acal_up;
        pre_acal_up_vld = true;
    }
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(false);
    });
    acal_cnt
}

/// Search sweep results for closest match to target reference frequency.
pub(super) fn search_closest(
    ref_tbl: &[u16],
    sweep_residual: &[u16],
    sweep_idac: &[u8],
    sweep_capcode: &[u8],
    sweep_num: usize,
    out_idac: &mut [u8],
    out_capcode: &mut [u8],
) {
    for (j, &target) in ref_tbl.iter().enumerate() {
        let mut best_err: u32 = 0;
        for i in 0..sweep_num {
            let err = if target > sweep_residual[i] {
                (target - sweep_residual[i]) as u32
            } else {
                (sweep_residual[i] - target) as u32
            };
            if i == 0 || err < best_err {
                best_err = err;
                out_idac[j] = sweep_idac[i];
                out_capcode[j] = sweep_capcode[i];
            }
        }
    }
}

// ============================================================
// Main calibration function
// ============================================================

/// Perform full 79-channel VCO calibration.
///
/// This is the complete implementation matching SDK `bt_rfc_lo_cal()`.
/// Includes: binary FCAL search -> linear sweep -> channel matching ->
/// KCAL -> PACAL -> ROSCAL -> RCCAL -> post-cal cleanup.
pub fn vco_cal_full() -> VcoCalResult {
    // ---- Setup ----
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(false);
        w.set_vco3g_auto_incfcal_en(false);
    });
    BT_RFC.inccal_reg2().modify(|w| {
        w.set_vco5g_auto_incacal_en(false);
        w.set_vco5g_auto_incfcal_en(false);
    });
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_idac_force_en(true);
        w.set_pdx_force_en(true);
        w.set_en_2m_mod_frc_en(true);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(true);
        w.set_brf_en_vddpsw_lv(true);
        w.set_brf_lo_iary_en_lv(true);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_vl_sel_lv(VCO_ACAL_VL_CAL);
        w.set_brf_vco_acal_vh_sel_lv(VCO_ACAL_VH_CAL);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco5g_en_lv(true);
        w.set_brf_en_2m_mod_lv(true);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(true);
        w.set_brf_vco_fkcal_en_lv(true);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(true);
    });
    BT_PHY.tx_hfp_cfg().modify(|w| {
        w.set_hfp_fcw(HFP_FCW_CAL);
        w.set_hfp_fcw_sel(false);
    });
    BT_RFC.vco_reg3().modify(|w| {
        w.set_brf_vco_idac_lv(IDAC_INITIAL);
    });

    // ---- FCAL binary search ----
    let mut fcal_cnt: u8 = PDX_INITIAL;
    let fcal_cnt_fs: u8 = PDX_FS;

    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_en_lv(true);
        w.set_brf_sdm_clk_sel_lv(true);
        w.set_brf_fbdv_mod_stg_lv(FBDV_MOD_STG_5G);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_fkcal_en_lv(true);
    });
    BT_RFC.fbdv_reg2().modify(|w| {
        w.set_brf_fkcal_cnt_divn_lv(FKCAL_DIVN_5G);
    });
    BT_PHY.tx_lfp_cfg().modify(|w| {
        w.set_lfp_fcw(LFP_FCW_CAL);
        w.set_lfp_fcw_sel(false);
    });
    BT_RFC.vco_reg3().modify(|w| {
        w.set_brf_vco_pdx_lv(PDX_INITIAL);
    });
    BT_PHY.tx_hfp_cfg().modify(|w| {
        w.set_hfp_fcw(HFP_FCW_CAL);
        w.set_hfp_fcw_sel(false);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_en_2m_mod_lv(true);
    });
    // FBDV reset sequence
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_rstb_lv(true);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_rstb_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_rstb_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_rstb_lv(true);
    });
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_xtal_ref_en(true);
        w.set_xtal_ref_en_frc_en(true);
    });
    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_en_lv(true);
    });
    BT_RFC.vco_reg3().modify(|w| {
        w.set_brf_vco_idac_lv(IDAC_INITIAL);
    });

    // Binary search variables
    let mut idac0: u8 = 0;
    let mut idac1: u8 = 0;
    let mut capcode0: u8 = 0;
    let mut capcode1: u8 = 0;
    let mut p0: u32 = 0;
    let mut p1: u32 = 0;
    let mut error0: u32 = u32::MAX;
    let mut error1: u32 = u32::MAX;

    for i in 1..9u32 {
        let acal_cnt = acal_binary_search();
        let residual_cnt = get_fbdv_cnt();

        let step = ((fcal_cnt_fs as u32) >> i) as u8;
        if residual_cnt > RESIDUAL_CNT_VTH {
            idac1 = acal_cnt;
            p1 = residual_cnt;
            error1 = residual_cnt - RESIDUAL_CNT_VTH;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt.saturating_add(step);
        } else {
            idac0 = acal_cnt;
            p0 = residual_cnt;
            error0 = RESIDUAL_CNT_VTH - residual_cnt;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt.saturating_sub(step);
        }
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });
        BT_RFC.vco_reg3().modify(|w| {
            w.set_brf_vco_pdx_lv(fcal_cnt);
        });
    }

    // ---- Sweep tables ----
    let mut sweep_idac = [0u8; MAX_LO_CAL_STEP];
    let mut sweep_capcode = [0u8; MAX_LO_CAL_STEP];
    let mut sweep_residual = [0u16; MAX_LO_CAL_STEP];

    // Pick best starting point
    if error0 < error1 {
        sweep_idac[0] = idac0;
        sweep_capcode[0] = capcode0;
        sweep_residual[0] = p0 as u16;
    } else {
        sweep_idac[0] = idac1;
        sweep_capcode[0] = capcode1;
        sweep_residual[0] = p1 as u16;
    }

    BT_RFC.vco_reg3().modify(|w| {
        w.set_brf_vco_pdx_lv(fcal_cnt);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_en_lv(false);
    });

    fcal_cnt = sweep_capcode[0];
    let mut acal_cnt = sweep_idac[0];

    // ---- Linear sweep (capcode+1 each step until residual <= VTL) ----
    let mut sweep_num: usize = 0;
    for step in 1..MAX_LO_CAL_STEP {
        fcal_cnt = fcal_cnt.saturating_add(1);
        BT_RFC.vco_reg3().modify(|w| {
            w.set_brf_vco_pdx_lv(fcal_cnt);
        });

        // Sequential ACAL
        acal_cnt = acal_sequential(acal_cnt);
        BT_RFC.vco_reg3().modify(|w| {
            w.set_brf_vco_idac_lv(acal_cnt);
        });

        let residual_cnt = get_fbdv_cnt();
        if residual_cnt <= RESIDUAL_CNT_VTL {
            sweep_num = step;
            break;
        }

        sweep_idac[step] = acal_cnt;
        sweep_capcode[step] = fcal_cnt;
        sweep_residual[step] = residual_cnt as u16;
        sweep_num = step + 1;
    }

    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_fkcal_en_lv(false);
    });

    debug!(
        "VCO sweep: {} steps, capcode {}..{}",
        sweep_num,
        sweep_capcode[0],
        if sweep_num > 0 {
            sweep_capcode[sweep_num - 1]
        } else {
            0
        }
    );

    // ---- Channel matching ----
    let mut result = VcoCalResult {
        idac_tx: [0; 79],
        capcode_tx: [0; 79],
        kcal: [0; 79],
        idac_rx_1m: [0; 40],
        capcode_rx_1m: [0; 40],
        idac_rx_2m: [0; 40],
        capcode_rx_2m: [0; 40],
        idac_rx_bt: [0; 79],
        capcode_rx_bt: [0; 79],
    };

    search_closest(
        &REF_RESIDUAL_CNT_TBL_RX_1M,
        &sweep_residual,
        &sweep_idac,
        &sweep_capcode,
        sweep_num,
        &mut result.idac_rx_1m,
        &mut result.capcode_rx_1m,
    );
    search_closest(
        &REF_RESIDUAL_CNT_TBL_RX_2M,
        &sweep_residual,
        &sweep_idac,
        &sweep_capcode,
        sweep_num,
        &mut result.idac_rx_2m,
        &mut result.capcode_rx_2m,
    );
    search_closest(
        &REF_RESIDUAL_CNT_TBL_RX_BT,
        &sweep_residual,
        &sweep_idac,
        &sweep_capcode,
        sweep_num,
        &mut result.idac_rx_bt,
        &mut result.capcode_rx_bt,
    );
    search_closest(
        &REF_RESIDUAL_CNT_TBL_TX,
        &sweep_residual,
        &sweep_idac,
        &sweep_capcode,
        sweep_num,
        &mut result.idac_tx,
        &mut result.capcode_tx,
    );

    // ---- KCAL computation (ch 0-39, pivot ch19) ----
    {
        BT_RFC.vco_reg2().modify(|w| {
            w.set_brf_vco_fkcal_en_lv(true);
        });
        BT_RFC.fbdv_reg2().modify(|w| {
            w.set_brf_fkcal_cnt_divn_lv(FKCAL_DIVN_KCAL);
        });
        BT_PHY.tx_lfp_cfg().modify(|w| {
            w.set_lfp_fcw(LFP_FCW_CAL);
            w.set_lfp_fcw_sel(false);
        });
        // Set ch19 PDX/IDAC
        BT_RFC
            .vco_reg3()
            .write_value(crate::pac::bt_rfc::regs::VcoReg3(
                (result.capcode_tx[19] as u32) | ((result.idac_tx[19] as u32) << 8),
            ));
        BT_PHY.tx_hfp_cfg().modify(|w| {
            w.set_hfp_fcw(HFP_FCW_MIN);
            w.set_hfp_fcw_sel(false);
        });

        let pmin = get_fbdv_cnt();
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });

        BT_PHY.tx_hfp_cfg().modify(|w| {
            w.set_hfp_fcw(HFP_FCW_MAX);
        });

        let pmax = get_fbdv_cnt();
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });

        let kcal_norm = if pmax > pmin {
            KCAL_CONST_A / (pmax - pmin)
        } else {
            KCAL_DEFAULT_NORM
        };
        debug!("KCAL 0-39: norm={} pmin={} pmax={}", kcal_norm, pmin, pmax);

        for i in 0..40usize {
            let p_delta =
                (REF_RESIDUAL_CNT_TBL_TX[i] as i32) - (REF_RESIDUAL_CNT_TBL_TX[19] as i32);
            let factor = 1.0f32 - KCAL_FREQ_MULTIPLIER * (p_delta as f32) * KCAL_CONST_B_LOW;
            result.kcal[i] = ((kcal_norm as f32) * factor) as u16;
        }
    }

    // ---- KCAL computation (ch 40-78, pivot ch59) ----
    {
        BT_RFC.vco_reg2().modify(|w| {
            w.set_brf_vco_fkcal_en_lv(true);
        });
        BT_RFC.fbdv_reg2().modify(|w| {
            w.set_brf_fkcal_cnt_divn_lv(FKCAL_DIVN_KCAL);
        });
        BT_PHY.tx_lfp_cfg().modify(|w| {
            w.set_lfp_fcw(LFP_FCW_CAL);
            w.set_lfp_fcw_sel(false);
        });
        // Set ch59 PDX/IDAC
        BT_RFC
            .vco_reg3()
            .write_value(crate::pac::bt_rfc::regs::VcoReg3(
                (result.capcode_tx[59] as u32) | ((result.idac_tx[59] as u32) << 8),
            ));
        BT_PHY.tx_hfp_cfg().modify(|w| {
            w.set_hfp_fcw(HFP_FCW_MIN);
            w.set_hfp_fcw_sel(false);
        });

        let pmin = get_fbdv_cnt();
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });

        BT_PHY.tx_hfp_cfg().modify(|w| {
            w.set_hfp_fcw(HFP_FCW_MAX);
        });

        let pmax = get_fbdv_cnt();
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });
        BT_RFC.vco_reg2().modify(|w| {
            w.set_brf_vco_fkcal_en_lv(false);
        });

        let kcal_norm = if pmax > pmin {
            KCAL_CONST_A / (pmax - pmin)
        } else {
            KCAL_DEFAULT_NORM
        };
        debug!("KCAL 40-78: norm={} pmin={} pmax={}", kcal_norm, pmin, pmax);

        for i in 40..79usize {
            let p_delta =
                (REF_RESIDUAL_CNT_TBL_TX[i] as i32) - (REF_RESIDUAL_CNT_TBL_TX[59] as i32);
            let factor = 1.0f32 - KCAL_FREQ_MULTIPLIER * (p_delta as f32) * KCAL_CONST_B_HIGH;
            result.kcal[i] = ((kcal_norm as f32) * factor) as u16;
        }
    }

    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_en_lv(false);
    });

    // ---- PACAL (PA bias calibration) ----
    pacal();

    // ---- Post-VCO cleanup ----
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_xtal_ref_en_frc_en(false);
        w.set_idac_force_en(false);
        w.set_pdx_force_en(false);
        w.set_en_2m_mod_frc_en(false);
    });
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(true);
        w.set_vco3g_auto_incfcal_en(true);
    });
    BT_RFC.inccal_reg2().modify(|w| {
        w.set_vco5g_auto_incacal_en(true);
        w.set_vco5g_auto_incfcal_en(true);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(false);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_fkcal_en_lv(false);
    });
    BT_PHY.tx_lfp_cfg().modify(|w| {
        w.set_lfp_fcw_sel(true);
    });
    BT_PHY.tx_hfp_cfg().modify(|w| {
        w.set_hfp_fcw_sel(true);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_en_2m_mod_lv(false);
    });
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_cas_bp_lv(true);
    });
    BT_RFC.trf_reg2().modify(|w| {
        w.set_brf_pa_mcap_lv(true);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_rstb_lv(true);
    });
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_rstb_adc_lv(true);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco5g_en_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_en_lv(false);
    });

    // ---- ROSCAL (RX DC offset calibration) ----
    roscal();

    // ---- RCCAL (RC oscillator calibration) ----
    rccal();

    // ---- Final cleanup ----
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(false);
        w.set_brf_en_vddpsw_lv(false);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_rstb_lv(true);
    });

    result
}

// ============================================================
// Sub-calibrations (PACAL, ROSCAL, RCCAL)
// ============================================================

/// PA bias calibration.
/// Based on SDK PACAL section in `bt_rfc_lo_cal()`.
fn pacal() {
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_trf_ldo_vref_sel_lv(PA_LDO_VREF_SEL);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lodist5g_bletx_en_lv(true);
    });
    BT_RFC.trf_reg2().modify(|w| {
        w.set_brf_pa_unit_sel_lv(PA_UNIT_SEL);
        w.set_brf_pa_bufload_sel_lv(PA_BUFLOAD_SEL);
    });
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_buf_pu_lv(true);
        w.set_brf_trf_sig_en_lv(true);
        w.set_brf_pa_bcsel_lv(true);
    });
    BT_RFC.pacal_reg().modify(|w| {
        w.set_pa_rstb_frc_en(true);
        w.set_pacal_clk_en(true);
    });

    crate::cortex_m_blocking_delay_us(20);
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_rstn_lv(false);
    });
    crate::cortex_m_blocking_delay_us(2);
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_rstn_lv(true);
    });
    crate::cortex_m_blocking_delay_us(2);

    // Start PACAL
    BT_RFC.pacal_reg().modify(|w| {
        w.set_pacal_start(false);
    });
    BT_RFC.pacal_reg().modify(|w| {
        w.set_pacal_start(true);
    });

    // Wait for PACAL done
    for _ in 0..10_000 {
        if BT_RFC.pacal_reg().read().pacal_done() {
            break;
        }
    }

    // Read and apply results
    let pacal = BT_RFC.pacal_reg().read();
    let setbc_rslt = pacal.bc_cal_rslt();
    let setsgn_rslt = pacal.sgn_cal_rslt();
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_setbc_lv(setbc_rslt);
        w.set_brf_pa_setsgn_lv(setsgn_rslt);
    });

    debug!("PACAL done: bc={} sgn={}", setbc_rslt, setsgn_rslt);

    // Cleanup
    BT_RFC.pacal_reg().modify(|w| {
        w.set_pacal_start(false);
    });
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_bcsel_lv(false);
        w.set_brf_pa_buf_pu_lv(false);
    });
    BT_RFC.pacal_reg().modify(|w| {
        w.set_pacal_clk_en(false);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lodist5g_bletx_en_lv(false);
    });
}

/// RX DC offset calibration.
/// Based on SDK ROSCAL section in `bt_rfc_lo_cal()`.
fn roscal() {
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_en_ldo_rbb_lv(true);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_iarray_lv(true);
    });

    // Enable ADC
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_adc_clk_en_frc_en(true);
        w.set_adc_clk_en(true);
    });
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_en_ldo_adc_lv(true);
        w.set_brf_en_ldo_adcref_lv(true);
        w.set_brf_en_adc_i_lv(true);
        w.set_brf_en_adc_q_lv(true);
    });
    // Power up mixer/RVGA/CBPF
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_en_rvga_i_lv(true);
        w.set_brf_en_rvga_q_lv(true);
        w.set_brf_en_cbpf_lv(true);
        w.set_brf_cbpf_en_rc(false);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_osdaci_lv(true);
        w.set_brf_en_osdacq_lv(true);
    });

    // Start ROSCAL
    BT_RFC.roscal_reg1().modify(|w| {
        w.set_en_rosdac_i(true);
        w.set_en_rosdac_q(true);
        w.set_roscal_bypass(false);
    });
    BT_RFC.roscal_reg1().modify(|w| {
        w.set_roscal_start(true);
    });

    // Wait for ROSCAL done
    for _ in 0..10_000 {
        if BT_RFC.roscal_reg2().read().roscal_done() {
            break;
        }
    }

    // Read DOS_Q result and write to ROSCAL_REG2
    let dos_q = BT_RFC.rbb_reg4().read().brf_dos_q_lv();
    BT_RFC.roscal_reg2().modify(|w| {
        w.set_dos_q_sw(dos_q);
    });
    // Read DOS_I result and write to ROSCAL_REG2
    let dos_i = BT_RFC.rbb_reg4().read().brf_dos_i_lv();
    BT_RFC.roscal_reg2().modify(|w| {
        w.set_dos_i_sw(dos_i);
    });

    debug!("ROSCAL done: dos_i={} dos_q={}", dos_i, dos_q);

    // Cleanup
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_en_cbpf_lv(false);
        w.set_brf_en_rvga_i_lv(false);
        w.set_brf_en_rvga_q_lv(false);
        w.set_brf_cbpf_en_rc(true);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_osdaci_lv(false);
        w.set_brf_en_osdacq_lv(false);
    });
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_en_adc_i_lv(false);
        w.set_brf_en_adc_q_lv(false);
        w.set_brf_en_ldo_adc_lv(false);
        w.set_brf_en_ldo_adcref_lv(false);
    });
    BT_RFC.roscal_reg1().modify(|w| {
        w.set_roscal_bypass(true);
        w.set_roscal_start(false);
    });
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_adc_clk_en_frc_en(false);
    });
}

/// RC oscillator calibration.
/// Based on SDK RCCAL section in `bt_rfc_lo_cal()`.
fn rccal() {
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_en_ldo_rbb_lv(true);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_iarray_lv(true);
        w.set_brf_rccal_selxo_lv(true);
        w.set_brf_rccal_mancap_lv(false);
    });
    BT_RFC.rcroscal_reg().modify(|w| {
        w.set_rc_capcode_offset(0);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_rccal_lv(true);
        w.set_brf_rstb_rccal_lv(false);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_rstb_rccal_lv(true);
    });
    BT_RFC.rcroscal_reg().modify(|w| {
        w.set_rccal_start(true);
    });

    // Wait for RCCAL done
    for _ in 0..20 {
        if BT_RFC.rcroscal_reg().read().rccal_done() {
            break;
        }
        crate::cortex_m_blocking_delay_us(1);
    }

    // Read and apply result
    let rc_capcode = BT_RFC.rcroscal_reg().read().rc_capcode();
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_cbpf_capman_lv(rc_capcode);
        w.set_brf_rccal_mancap_lv(true);
    });

    debug!("RCCAL done: rc_capcode={}", rc_capcode);

    // Cleanup
    BT_RFC.rcroscal_reg().modify(|w| {
        w.set_rccal_start(false);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_iarray_lv(false);
        w.set_brf_en_rccal_lv(false);
    });
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_en_ldo_rbb_lv(false);
    });
}

/// Store driver version in reserved register.
#[allow(unused_variables)]
pub fn set_driver_version(_version: u32) {
    // RSVD_REG2 not available in PAC
}
