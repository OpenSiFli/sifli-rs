//! Post-calibration PHY/RFC optimization.
//!
//! Restores PHY registers to their correct state for normal BLE/BR operation
//! after calibration routines may have modified them. Also configures key RF
//! parameters like PA settings, modulation gains, demod parameters, and filters.
//!
//! Based on SDK `bt_rf_opt_cal()` in `bt_rf_fulcal.c`.

use crate::pac::{BT_PHY, BT_RFC};

/// Post-calibration PHY/RFC optimization.
pub(super) fn bt_rf_opt_cal() {
    debug!(
        "opt_cal BEFORE: TRF1=0x{:08X} TRF2=0x{:08X} RBB1=0x{:08X} RBB2=0x{:08X} RBB4=0x{:08X}",
        BT_RFC.trf_reg1().read().0,
        BT_RFC.trf_reg2().read().0,
        BT_RFC.rbb_reg1().read().0,
        BT_RFC.rbb_reg2().read().0,
        BT_RFC.rbb_reg4().read().0,
    );
    debug!(
        "opt_cal BEFORE: TX_CTRL=0x{:08X} DEMOD1=0x{:08X} GAUSS1=0x{:08X} GAUSS2=0x{:08X}",
        BT_PHY.tx_ctrl().read().0,
        BT_PHY.demod_cfg1().read().0,
        BT_PHY.tx_gaussflt_cfg1().read().0,
        BT_PHY.tx_gaussflt_cfg2().read().0,
    );

    // --- PA configuration ---
    BT_RFC.trf_reg1().modify(|w| {
        w.set_brf_pa_pm_lv(0x01);
        w.set_brf_pa_cas_bp_lv(true);
    });
    BT_RFC.trf_reg2().modify(|w| {
        w.set_brf_pa_unit_sel_lv(0x01);
        w.set_brf_pa_mcap_lv(false);
    });

    // --- CBPF / PKDET thresholds ---
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_pkdet_vth1i_bt(0x08);
        w.set_brf_pkdet_vth1q_bt(0x08);
        w.set_brf_pkdet_vth2i_bt(0x08);
        w.set_brf_pkdet_vth2q_bt(0x08);
    });
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_cbpf_fc_lv(0x3);
    });
    BT_RFC.rbb_reg4().modify(|w| {
        w.set_brf_pkdet_vth1i_lv(0x0A);
        w.set_brf_pkdet_vth1q_lv(0x0A);
        w.set_brf_pkdet_vth2i_lv(0x0A);
        w.set_brf_pkdet_vth2q_lv(0x0A);
    });

    // --- Re-initialize ADC/PHY (same as rfc_init, restored after cal) ---
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_adc_q_en_1(true);
    });
    BT_PHY.rx_ctrl2().modify(|w| {
        w.set_adc_q_en_frc_en(true);
    });
    BT_PHY.tx_if_mod_cfg().modify(|w| {
        w.set_tx_if_phase_ble(0);
    });
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_rstb_adc_lv(true);
    });
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_pkdet_en_early_off_en(false);
    });
    // Select 3G VCO for EDR
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mmdiv_sel(true);
    });

    // --- IQ modulation gain tables ---
    // SDK does direct register writes for these multi-field registers
    BT_PHY.tx_if_mod_cfg3().write_value(crate::pac::bt_phy::regs::TxIfModCfg3(0x8055_5555));
    BT_PHY.tx_if_mod_cfg5().write_value(crate::pac::bt_phy::regs::TxIfModCfg5(0x6855_5555));
    BT_PHY.tx_if_mod_cfg6().write_value(crate::pac::bt_phy::regs::TxIfModCfg6(0x4444_4444));
    BT_PHY.tx_if_mod_cfg7().write_value(crate::pac::bt_phy::regs::TxIfModCfg7(0x5050_5044));
    BT_PHY.tx_dpsk_cfg1().write_value(crate::pac::bt_phy::regs::TxDpskCfg1(0x4444_4444));
    BT_PHY.tx_dpsk_cfg2().write_value(crate::pac::bt_phy::regs::TxDpskCfg2(0x5050_5044));

    // --- Mixer phase ---
    BT_PHY.mixer_cfg1().modify(|w| {
        w.set_rx_mixer_phase_1(0xA6);
        w.set_rx_mixer_phase_2(0x80);
    });

    // --- BLE DEMOD ---
    BT_PHY.demod_cfg1().modify(|w| {
        w.set_ble_demod_g(0xB0);
        w.set_ble_mu_dc(0x22);
        w.set_ble_mu_err(0x168);
    });

    // --- TX GFSK modulation index ---
    BT_PHY.tx_gaussflt_cfg1().modify(|w| {
        w.set_polar_gauss_gain_2(0xF7);
        w.set_polar_gauss_gain_1(0xFD);
        w.set_polar_gauss_gain_br(0xAA);
    });
    BT_PHY.tx_gaussflt_cfg2().modify(|w| {
        w.set_iq_gauss_gain_br(0xAE);
        w.set_iq_gauss_gain_1(0xFF);
        w.set_iq_gauss_gain_2(0xFF);
    });

    // --- MMDIV OFFSET (BLE RX frequency offset correction) ---
    BT_PHY.lfp_mmdiv_cfg0().modify(|w| {
        w.set_rx_mmdiv_offset_1m(0x1AAE1);
    });
    BT_PHY.lfp_mmdiv_cfg1().modify(|w| {
        w.set_rx_mmdiv_offset_2m(0x18000);
    });

    // --- BR DEMOD ---
    BT_PHY.demod_cfg8().modify(|w| {
        w.set_br_demod_g(0x50);
        w.set_br_mu_dc(0x10);
        w.set_br_mu_err(0x120);
    });

    // --- BR HADAPT ---
    BT_PHY.demod_cfg16().modify(|w| {
        w.set_br_hadapt_en(true);
    });

    // --- TED (Timing Error Detector) ---
    BT_PHY.ted_cfg1().write_value(crate::pac::bt_phy::regs::TedCfg1(
        (0x02 << 0)   // TED_MU_F_U
        | (0x04 << 4) // TED_MU_P_U
        | (0x03 << 8) // TED_MU_F_BR
        | (0x05 << 12) // TED_MU_P_BR
    ));

    // --- PKT detect threshold (BR) ---
    BT_PHY.pktdet_cfg2().modify(|w| {
        w.set_br_pktdet_thd(0x500);
    });

    // --- TRF_EDR_REG1 voltage matching (1.2V mode) ---
    BT_RFC.trf_edr_reg1().modify(|w| {
        w.set_brf_trf_edr_tmxcas_sel_lv(false);
    });

    // --- NOTCH filter ---
    BT_PHY.notch_cfg1().modify(|w| {
        w.set_notch_b1_1(0x3000);
    });
    BT_PHY.notch_cfg7().modify(|w| {
        w.set_chnl_notch_en1_1(0x00004000);
    });
    BT_PHY.notch_cfg10().modify(|w| {
        w.set_chnl_notch_en1_2(0x00004000);
    });

    // --- Interpolator ---
    BT_PHY.interp_cfg1().modify(|w| {
        w.set_interp_method_u(true);
    });

    // --- EDR sync ---
    BT_PHY.edrsync_cfg1().modify(|w| {
        w.set_edrsync_method(true);
    });

    // --- EDR DEMOD ---
    BT_PHY.edrdemod_cfg1().modify(|w| {
        w.set_edr2_mu_dc(0x40);
        w.set_edr2_mu_err(0x100);
    });
    BT_PHY.edrdemod_cfg2().modify(|w| {
        w.set_edr3_mu_dc(0x40);
        w.set_edr3_mu_err(0x140);
    });

    // --- BR MU_H ---
    BT_PHY.demod_cfg16().modify(|w| {
        w.set_br_mu_h(0x28);
    });

    // --- EDR TED ---
    BT_PHY.edrted_cfg1().modify(|w| {
        w.set_ted_edr2_mu_f(0x8);
        w.set_ted_edr2_mu_p(0x4);
        w.set_ted_edr3_mu_f(0x8);
        w.set_ted_edr3_mu_p(0x4);
    });

    // --- BT OP mode ---
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_bt_op_mode(true);
    });

    // --- LPF coefficients ---
    BT_PHY.lpf_cfg0().modify(|w| {
        w.set_lpf_coef_0(0x8);
        w.set_lpf_coef_1(0x2);
        w.set_lpf_coef_2(0x1F1);
    });
    BT_PHY.lpf_cfg1().modify(|w| {
        w.set_lpf_coef_3(0x1DF);
        w.set_lpf_coef_4(0x1DD);
        w.set_lpf_coef_5(0x1FE);
    });
    BT_PHY.lpf_cfg2().modify(|w| {
        w.set_lpf_coef_6(0x43);
        w.set_lpf_coef_7(0x9B);
        w.set_lpf_coef_8(0xE5);
    });
    BT_PHY.lpf_cfg3().modify(|w| {
        w.set_lpf_coef_9(0xFF);
    });

    // --- NOTCH filter (set 2) ---
    BT_PHY.notch_cfg6().write_value(crate::pac::bt_phy::regs::NotchCfg6(0x400000));
    BT_PHY.notch_cfg8().modify(|w| {
        w.set_chnl_notch_en2_1(0x40);
    });
    BT_PHY.notch_cfg9().write_value(crate::pac::bt_phy::regs::NotchCfg9(0x400000));
    BT_PHY.notch_cfg11().modify(|w| {
        w.set_chnl_notch_en2_2(0x40);
    });

    // --- RBB_REG6 (BR CBPF bandwidth) ---
    BT_RFC.rbb_reg6().modify(|w| {
        w.set_brf_cbpf_bw_lv_br(true);
        w.set_brf_cbpf_w2x_stg1_lv_br(true);
        w.set_brf_cbpf_w2x_stg2_lv_br(true);
    });

    debug!(
        "opt_cal AFTER: TRF1=0x{:08X} TRF2=0x{:08X} RBB1=0x{:08X} RBB2=0x{:08X} RBB4=0x{:08X}",
        BT_RFC.trf_reg1().read().0,
        BT_RFC.trf_reg2().read().0,
        BT_RFC.rbb_reg1().read().0,
        BT_RFC.rbb_reg2().read().0,
        BT_RFC.rbb_reg4().read().0,
    );
    debug!(
        "opt_cal AFTER: TX_CTRL=0x{:08X} DEMOD1=0x{:08X} GAUSS1=0x{:08X} GAUSS2=0x{:08X}",
        BT_PHY.tx_ctrl().read().0,
        BT_PHY.demod_cfg1().read().0,
        BT_PHY.tx_gaussflt_cfg1().read().0,
        BT_PHY.tx_gaussflt_cfg2().read().0,
    );
    debug!(
        "opt_cal AFTER: IF_MOD3=0x{:08X} IF_MOD5=0x{:08X} IF_MOD6=0x{:08X} IF_MOD7=0x{:08X}",
        BT_PHY.tx_if_mod_cfg3().read().0,
        BT_PHY.tx_if_mod_cfg5().read().0,
        BT_PHY.tx_if_mod_cfg6().read().0,
        BT_PHY.tx_if_mod_cfg7().read().0,
    );
    debug!(
        "opt_cal AFTER: DPSK1=0x{:08X} DPSK2=0x{:08X} MIXER1=0x{:08X} TRF_EDR1=0x{:08X}",
        BT_PHY.tx_dpsk_cfg1().read().0,
        BT_PHY.tx_dpsk_cfg2().read().0,
        BT_PHY.mixer_cfg1().read().0,
        BT_RFC.trf_edr_reg1().read().0,
    );
}
