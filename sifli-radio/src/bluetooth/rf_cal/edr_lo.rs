//! EDR LO 3GHz calibration: VCO3G frequency calibration + OSLO phase calibration.
//!
//! Based on SDK `bt_rfc_edrlo_3g_cal()` in `bt_rf_fulcal.c` (lines 3026-3655).
//!
//! EDR (Enhanced Data Rate) uses a 3GHz VCO for the LO (Local Oscillator),
//! which requires separate calibration from the 5GHz VCO used by BLE.
//! The OSLO (On-chip Sub-harmonic LO) provides the sub-harmonic mixing
//! needed for EDR operation and requires per-channel FC/BM calibration.

use crate::pac::{BT_MAC, BT_PHY, BT_RFC, GPADC, HPSYS_CFG};

use super::consts::*;
use super::vco;

// ============================================================
// EDR LO 3GHz calibration constants
// ============================================================

/// Residual count upper threshold for 3GHz VCO binary search.
const RESIDUAL_CNT_VTH_3G: u32 = 33670;

/// Residual count lower threshold for 3GHz VCO linear sweep termination.
const RESIDUAL_CNT_VTL_3G: u32 = 30530;

/// FKCAL counter divider for 3GHz VCO calibration.
const FKCAL_DIVN_3G: u16 = 11520;

/// OSLO frequency control default value.
const OSLO_FC_DEFAULT: u8 = 3;

/// OSLO bias magnitude initial value for binary search.
const OSLO_BM_INITIAL: u8 = 0x10;

/// OSLO BM binary search initial step size.
const BM_SEARCH_STEP: u8 = 0x8;

/// Default TMXCAP selection value written to calibration table.
pub(super) const TMXCAP_DEFAULT: u32 = 6;

/// GPADC conversion width for OSLO calibration.
const GPADC_CONV_WIDTH: u16 = 252;

/// GPADC sampling width for OSLO calibration.
const GPADC_SAMP_WIDTH: u16 = 239;

/// GPADC data sampling delay for OSLO calibration.
const GPADC_SAMP_DLY: u8 = 7;

/// Number of GPADC slots (slot 0 enabled, slots 1-7 disabled).
const GPADC_SLOT_COUNT: usize = 8;

/// OSLO BM binary search iterations (5 = ceil(log2(0x10/0x8)) + margin).
const OSLO_BM_SEARCH_STEPS: usize = 5;

/// EDR TX 3G reference residual counts (79 channels).
/// From SDK `ref_residual_cnt_tbl_tx_3g[]` (bt_rf_fulcal.c:228-309).
static REF_RESIDUAL_CNT_TBL_TX_3G: [u16; 79] = [
    30544, 30584, 30624, 30664, 30704, 30744, 30784, 30824, 30864, 30904, 30944, 30984, 31024,
    31064, 31104, 31144, 31184, 31224, 31264, 31304, 31344, 31384, 31424, 31464, 31504, 31544,
    31584, 31624, 31664, 31704, 31744, 31784, 31824, 31864, 31904, 31944, 31984, 32024, 32064,
    32104, 32144, 32184, 32224, 32264, 32304, 32344, 32384, 32424, 32464, 32504, 32544, 32584,
    32624, 32664, 32704, 32744, 32784, 32824, 32864, 32904, 32944, 32984, 33024, 33064, 33104,
    33144, 33184, 33224, 33264, 33304, 33344, 33384, 33424, 33464, 33504, 33544, 33584, 33624,
    33664,
];

/// Initial DPSK gain values per channel (79 channels).
/// From SDK `dpsk_gain[]` (bt_rf_fulcal.c:569-650).
/// These are packed into the BT TX calibration table alongside VCO/OSLO results.
pub(super) static DPSK_GAIN_INITIAL: [u8; 79] = [
    0x3C, 0x3A, 0x3C, 0x3C, 0x3A, 0x3B, 0x3B, 0x3B, 0x3E, 0x3A, 0x3A, 0x3A, 0x39, 0x38, 0x39, 0x38,
    0x38, 0x37, 0x37, 0x37, 0x36, 0x36, 0x35, 0x35, 0x34, 0x34, 0x33, 0x33, 0x33, 0x32, 0x32, 0x32,
    0x31, 0x32, 0x32, 0x30, 0x30, 0x30, 0x2F, 0x30, 0x30, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F, 0x2F,
    0x2F, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x31, 0x32, 0x32, 0x32, 0x33,
    0x33, 0x34, 0x34, 0x35, 0x35, 0x36, 0x37, 0x37, 0x38, 0x38, 0x39, 0x39, 0x3A, 0x3A, 0x3A,
];

// ============================================================
// Result type
// ============================================================

/// EDR LO calibration result for 79 channels.
pub struct EdrLoCalResult {
    /// VCO3G IDAC per channel
    pub idac: [u8; 79],
    /// VCO3G capcode (PDX) per channel
    pub capcode: [u8; 79],
    /// OSLO FC (frequency control) per channel (0-7)
    pub oslo_fc: [u8; 79],
    /// OSLO BM (bias magnitude) per channel
    pub oslo_bm: [u8; 79],
}

// ============================================================
// VCO3G ACAL helpers (different registers from BLE VCO5G)
// ============================================================

/// ACAL binary search for VCO3G.
/// Writes IDAC to EDR_CAL_REG1, reads vco3g feedback bits from VCO_REG2.
fn acal_binary_search_3g() -> u8 {
    let mut acal_cnt: u8 = IDAC_INITIAL;
    let acal_cnt_fs: u8 = IDAC_FS;

    BT_RFC.edr_cal_reg1().modify(|w| {
        w.set_brf_edr_vco_idac_lv(acal_cnt);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(true);
    });

    for j in 1..7u32 {
        if !BT_RFC.vco_reg2().read().brf_vco3g_acal_incal_lv() {
            break;
        }
        let step = ((acal_cnt_fs as u32) >> j) as u8;
        if !BT_RFC.vco_reg2().read().brf_vco3g_acal_up_lv() {
            acal_cnt = acal_cnt.saturating_sub(step);
        } else {
            acal_cnt = acal_cnt.saturating_add(step);
        }
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_edr_vco_idac_lv(acal_cnt);
        });
    }
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(false);
    });
    acal_cnt
}

/// Sequential ACAL for VCO3G (used during linear sweep).
/// Same logic as vco.rs `acal_sequential()`, but operates on EDR_CAL_REG1
/// and reads vco3g feedback bits.
fn acal_sequential_3g(mut acal_cnt: u8) -> u8 {
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
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_edr_vco_idac_lv(acal_cnt);
        });

        if !BT_RFC.vco_reg2().read().brf_vco3g_acal_incal_lv() {
            break;
        }
        let curr_acal_up = BT_RFC.vco_reg2().read().brf_vco3g_acal_up_lv();

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

// ============================================================
// GPADC save/restore guard
// ============================================================

/// RAII guard that saves GPADC registers on creation and restores on drop.
struct GpadcCalGuard {
    saved_cfg1: u32,
    saved_ctrl: u32,
    saved_ctrl2: u32,
}

impl GpadcCalGuard {
    /// Save current GPADC state and configure for OSLO calibration.
    fn new() -> Self {
        let saved_cfg1 = GPADC.cfg_reg1().read().0;
        let saved_ctrl = GPADC.ctrl_reg().read().0;
        let saved_ctrl2 = GPADC.ctrl_reg2().read().0;

        // Enable bandgap
        HPSYS_CFG.anau_cr().modify(|w| {
            w.set_en_bg(true);
        });

        // Configure GPADC for OSLO: P_INT_EN, SE, LDOREF_EN
        GPADC.cfg_reg1().modify(|w| {
            w.set_anau_gpadc_p_int_en(true);
            w.set_anau_gpadc_se(true);
            w.set_anau_gpadc_ldoref_en(true);
        });

        GPADC.ctrl_reg().modify(|w| {
            w.set_data_samp_dly(GPADC_SAMP_DLY);
        });

        GPADC.ctrl_reg2().write(|w| {
            w.set_conv_width(GPADC_CONV_WIDTH);
            w.set_samp_width(GPADC_SAMP_WIDTH);
        });

        // Enable SLOT0, disable SLOT1-7
        GPADC.slot(0).modify(|w| {
            w.set_slot_en(true);
        });
        for i in 1..GPADC_SLOT_COUNT {
            GPADC.slot(i).modify(|w| {
                w.set_slot_en(false);
            });
        }

        Self {
            saved_cfg1,
            saved_ctrl,
            saved_ctrl2,
        }
    }

    /// Start a single GPADC conversion and read the result.
    /// Returns the 12-bit ADC value, or 0 on timeout.
    fn read_single(&self) -> u16 {
        // Start conversion
        GPADC.ctrl_reg().modify(|w| {
            w.set_adc_start(true);
        });

        // Poll for completion (timeout ~1ms at 48MHz)
        for _ in 0..50_000u32 {
            if GPADC.gpadc_irq().read().gpadc_irsr() {
                break;
            }
        }

        // Read 12-bit result from RDATA0 (slot 0 = even_slot_rdata)
        let value = GPADC.rdata(0).read().even_slot_rdata();

        // Clear interrupt
        GPADC.gpadc_irq().modify(|w| {
            w.set_gpadc_icr(true);
        });

        value
    }
}

impl Drop for GpadcCalGuard {
    fn drop(&mut self) {
        // Restore original GPADC registers
        GPADC
            .cfg_reg1()
            .write_value(crate::pac::gpadc::regs::CfgReg1(self.saved_cfg1));
        GPADC
            .ctrl_reg()
            .write_value(crate::pac::gpadc::regs::CtrlReg(self.saved_ctrl));
        GPADC
            .ctrl_reg2()
            .write_value(crate::pac::gpadc::regs::CtrlReg2(self.saved_ctrl2));
    }
}

// ============================================================
// Main calibration function
// ============================================================

/// Perform full EDR LO 3GHz calibration.
///
/// This includes:
/// - Part A: VCO3G frequency calibration (ACAL/FCAL binary search + linear sweep)
/// - Part B: OSLO phase calibration (FC sweep + BM binary search, using GPADC)
/// - Part C: Store results to RFC SRAM (BT TX table)
///
/// Returns calibration results for 79 channels.
///
/// Corresponds to SDK `bt_rfc_edrlo_3g_cal()` (bt_rf_fulcal.c:3026-3655).
pub fn edr_lo_cal_full() -> EdrLoCalResult {
    debug!("begin EDR 3G LO fulcal");

    // ================================================================
    // Part A: VCO3G Frequency Calibration
    // ================================================================

    // --- A1: Hardware initialization ---

    // Enable IQ mod TX for calibration
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mac_mod_ctrl_en(false);
        w.set_mod_method_ble(true);
        w.set_mod_method_br(true);
    });

    // Force TX on via MAC
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx(true);
        w.set_force_tx_val(true);
    });
    crate::cortex_m_blocking_delay_us(100);

    // Disable VCO3G auto incremental calibration
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(false);
        w.set_vco3g_auto_incfcal_en(false);
    });

    // Enable RFBG, VDDPSW, LO_IARY
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(true);
        w.set_brf_en_vddpsw_lv(true);
        w.set_brf_lo_iary_en_lv(true);
    });

    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_vl_sel_lv(VCO_ACAL_VL_NORMAL);
        w.set_brf_vco_acal_vh_sel_lv(VCO_ACAL_VH_NORMAL);
        w.set_brf_vco_incfcal_vl_sel_lv(VCO_INCFCAL_VL);
        w.set_brf_vco_incfcal_vh_sel_lv(VCO_INCFCAL_VH);
    });

    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco_ldo_vref_lv(VCO_LDO_VREF);
    });

    // Enable VCO3G
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco3g_en_lv(true);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(true);
        w.set_brf_vco_fkcal_en_lv(true);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(true);
    });

    // --- A2: Configure FBDV for 3G ---
    // Key differences from BLE VCO: MOD_STG=1 (not 2), SDM_CLK_SEL=0 (not 1), DIVN=11520 (not 7680)
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_mod_stg_lv(FBDV_MOD_STG_3G);
        w.set_brf_sdm_clk_sel_lv(false);
        w.set_brf_fbdv_en_lv(true);
    });

    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_fkcal_en_lv(true);
    });
    BT_RFC.fbdv_reg2().modify(|w| {
        w.set_brf_fkcal_cnt_divn_lv(FKCAL_DIVN_3G);
    });

    BT_PHY.tx_lfp_cfg().modify(|w| {
        w.set_lfp_fcw(LFP_FCW_CAL);
        w.set_lfp_fcw_sel(false);
    });

    BT_RFC.edr_cal_reg1().modify(|w| {
        w.set_brf_edr_vco_pdx_lv(PDX_INITIAL);
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

    // Enable EDR XTAL reference
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_edr_xtal_ref_en(true);
        w.set_edr_xtal_ref_en_frc_en(true);
    });
    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_en_lv(true);
    });

    BT_RFC.edr_cal_reg1().modify(|w| {
        w.set_brf_edr_vco_idac_lv(IDAC_INITIAL);
    });

    // --- A3: FCAL binary search (8 iterations) ---
    let mut fcal_cnt: u8 = PDX_INITIAL;
    let fcal_cnt_fs: u8 = PDX_FS;

    let mut idac0: u8 = 0;
    let mut idac1: u8 = 0;
    let mut capcode0: u8 = 0;
    let mut capcode1: u8 = 0;
    let mut p0: u32 = 0;
    let mut p1: u32 = 0;
    let mut error0: u32 = u32::MAX;
    let mut error1: u32 = u32::MAX;

    for i in 1..9u32 {
        let acal_cnt = acal_binary_search_3g();
        let residual_cnt = vco::get_fbdv_cnt();

        let step = ((fcal_cnt_fs as u32) >> i) as u8;
        if residual_cnt > RESIDUAL_CNT_VTH_3G {
            idac1 = acal_cnt;
            p1 = residual_cnt;
            error1 = residual_cnt - RESIDUAL_CNT_VTH_3G;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt.saturating_add(step);
        } else {
            idac0 = acal_cnt;
            p0 = residual_cnt;
            error0 = RESIDUAL_CNT_VTH_3G - residual_cnt;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt.saturating_sub(step);
        }
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fkcal_cnt_en_lv(false);
        });
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_edr_vco_pdx_lv(fcal_cnt);
        });
    }

    // --- A4: Linear sweep ---
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

    BT_RFC.edr_cal_reg1().modify(|w| {
        w.set_brf_edr_vco_pdx_lv(fcal_cnt);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_en_lv(false);
    });

    fcal_cnt = sweep_capcode[0];
    let mut acal_cnt = sweep_idac[0];

    let mut sweep_num: usize = 0;
    for step in 1..MAX_LO_CAL_STEP {
        fcal_cnt = fcal_cnt.saturating_add(1);
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_edr_vco_pdx_lv(fcal_cnt);
        });

        // Sequential ACAL for VCO3G
        acal_cnt = acal_sequential_3g(acal_cnt);
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_edr_vco_idac_lv(acal_cnt);
        });

        let residual_cnt = vco::get_fbdv_cnt();
        if residual_cnt <= RESIDUAL_CNT_VTL_3G {
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
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fkcal_cnt_en_lv(false);
    });

    debug!(
        "EDR VCO3G sweep: {} steps, capcode {}..{}",
        sweep_num,
        sweep_capcode[0],
        if sweep_num > 0 {
            sweep_capcode[sweep_num - 1]
        } else {
            0
        }
    );

    // --- A5: 79-channel matching ---
    let mut result = EdrLoCalResult {
        idac: [0; 79],
        capcode: [0; 79],
        oslo_fc: [OSLO_FC_DEFAULT; 79],
        oslo_bm: [OSLO_BM_INITIAL; 79],
    };

    vco::search_closest(
        &REF_RESIDUAL_CNT_TBL_TX_3G,
        &sweep_residual,
        &sweep_idac,
        &sweep_capcode,
        sweep_num,
        &mut result.idac,
        &mut result.capcode,
    );

    // --- A6: Clean up VCO3G hardware ---
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco3g_en_lv(false);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lo_iary_en_lv(false);
        w.set_brf_en_rfbg_lv(false);
        w.set_brf_en_vddpsw_lv(false);
    });

    // --- Write initial VCO3G results to RFC SRAM (before OSLO overwrites fc/bm) ---
    // SDK writes initial results here with fc=3, bm=0x10, then OSLO overwrites
    store_initial_edr_table(&result);

    // ================================================================
    // Part B: OSLO Calibration
    // ================================================================

    // --- B1: Disable force TX, prepare for OSLO ---
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx_val(false);
    });
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx(false);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(false);
    });
    BT_PHY.tx_lfp_cfg().modify(|w| {
        w.set_lfp_fcw_sel(true);
    });

    // --- B2: Enable OSLO hardware ---
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(true);
        w.set_brf_en_vddpsw_lv(true);
    });
    crate::cortex_m_blocking_delay_us(2);

    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco3g_en_lv(true);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lo_iary_en_lv(false);
    });
    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_en_lv(true);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_en_lv(true);
    });
    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_en_lv(true);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lodistedr_en_lv(true);
    });

    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_pkdet_en_lv(true);
        w.set_brf_oslo_fcal_en_lv(true);
    });
    BT_RFC.edr_cal_reg1().modify(|w| {
        w.set_brf_oslo_bm_lv(OSLO_BM_INITIAL);
    });

    // --- B3: Save GPADC state and configure for OSLO ---
    let gpadc_guard = GpadcCalGuard::new();

    // --- B4: Per-channel FC + BM calibration ---
    for ch in 0..79u8 {
        // Set channel and force TX
        BT_MAC.dmradiocntl1().modify(|w| {
            w.set_channel(ch);
            w.set_force_channel(true);
            w.set_force_tx(true);
            w.set_force_tx_val(true);
        });
        crate::cortex_m_blocking_delay_us(40);

        // --- FC sweep: find FC (0-7) that maximizes GPADC output ---
        let mut max_adc_value: u16 = 0;
        let mut best_fc: u8 = 0;
        for fc in 0..8u8 {
            BT_RFC.edr_cal_reg1().modify(|w| {
                w.set_brf_oslo_fc_lv(fc);
            });
            crate::cortex_m_blocking_delay_us(40);

            let adc_value = gpadc_guard.read_single();
            if adc_value > max_adc_value {
                max_adc_value = adc_value;
                best_fc = fc;
            }
        }
        result.oslo_fc[ch as usize] = best_fc;

        // --- BM binary search (5 iterations) ---
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_oslo_fc_lv(best_fc);
        });

        let mut bm: u8 = OSLO_BM_INITIAL;
        let mut bm_step: u8 = BM_SEARCH_STEP;
        BT_RFC.edr_cal_reg1().modify(|w| {
            w.set_brf_oslo_bm_lv(bm);
        });

        // Lock LO (FBDV reset toggle)
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fbdv_rstb_lv(true);
        });
        BT_RFC.fbdv_reg1().modify(|w| {
            w.set_brf_fbdv_rstb_lv(false);
        });
        crate::cortex_m_blocking_delay_us(40);

        for _ in 0..OSLO_BM_SEARCH_STEPS {
            let acal_cmp = BT_RFC.oslo_reg().read().brf_oslo_acal_cmp_lv();
            if acal_cmp {
                bm = bm.saturating_sub(bm_step);
            } else {
                bm = bm.saturating_add(bm_step);
            }
            bm_step >>= 1;
            BT_RFC.edr_cal_reg1().modify(|w| {
                w.set_brf_oslo_bm_lv(bm);
            });
            crate::cortex_m_blocking_delay_us(4);
        }
        result.oslo_bm[ch as usize] = bm;

        // Release TX for this channel
        BT_MAC.dmradiocntl1().modify(|w| {
            w.set_force_tx_val(false);
        });
    }

    // ================================================================
    // Part C: OSLO Cleanup
    // ================================================================

    // Disable misc force bits
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_edr_xtal_ref_en_frc_en(false);
        w.set_idac_force_en(false);
        w.set_pdx_force_en(false);
    });

    // Re-enable VCO3G auto calibration
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(true);
        w.set_vco3g_auto_incfcal_en(true);
    });

    // Disable OSLO and related blocks
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(false);
        w.set_brf_en_vddpsw_lv(false);
        w.set_brf_lodistedr_en_lv(false);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco3g_en_lv(false);
    });
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lo_iary_en_lv(false);
    });
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_en_lv(false);
        w.set_brf_vco_fkcal_en_lv(false);
    });
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lo_open_lv(false);
    });
    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_en_lv(false);
    });
    BT_PHY.tx_lfp_cfg().modify(|w| {
        w.set_lfp_fcw_sel(true);
    });

    // Restore FBDV to 5G mode: clear then set MOD_STG=2, SDM_CLK_SEL=1, RSTB=1
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_en_lv(false);
        w.set_brf_fbdv_mod_stg_lv(0);
    });
    BT_RFC.fbdv_reg1().modify(|w| {
        w.set_brf_fbdv_rstb_lv(true);
        w.set_brf_fbdv_mod_stg_lv(FBDV_MOD_STG_5G);
        w.set_brf_sdm_clk_sel_lv(true);
    });
    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_en_lv(false);
    });

    // Release MAC force
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx_val(false);
    });
    crate::cortex_m_blocking_delay_us(20);
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx(false);
        w.set_force_channel(false);
    });

    // Disable IQ mod TX
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mod_method_ble(false);
        w.set_mod_method_br(false);
    });

    // Disable OSLO FCAL
    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_fcal_en_lv(false);
    });

    // GPADC guard drop restores registers automatically
    drop(gpadc_guard);

    result
}

/// Write initial EDR LO VCO3G results to RFC SRAM (BT TX area).
///
/// SDK writes these immediately after VCO3G cal with fc=3, bm=0x10.
/// The OSLO cal loop later overwrites the same area with final fc/bm.
fn store_initial_edr_table(result: &EdrLoCalResult) {
    let base = super::BT_RFC_MEM_BASE;
    let bt_tx_addr = BT_RFC.cal_addr_reg2().read().bt_tx_cal_addr() as u32;

    for i in 0..79usize {
        let mut word: u32 = 0;
        word |= result.capcode[i] as u32; // [7:0] PDX
        word |= (result.idac[i] as u32) << 8; // [14:8] IDAC
        word |= (OSLO_FC_DEFAULT as u32) << 16; // [18:16] OSLO_FC
        word |= (OSLO_BM_INITIAL as u32) << 20; // [24:20] OSLO_BM
        word |= TMXCAP_DEFAULT << 28; // [31:28] TMXCAP

        unsafe {
            core::ptr::write_volatile((base + bt_tx_addr + (i as u32) * 4) as *mut u32, word);
        }
    }
}
