//! TXDC calibration hardware configuration and cleanup.
//!
//! Contains the register-level setup and teardown sequences for TX DC offset
//! calibration. These are separated from the algorithm logic in `txdc.rs`.

use crate::pac::{BT_MAC, BT_PHY, BT_RFC};
use super::txdc::TxdcCalConfig;

/// Configure hardware for TXDC calibration.
///
/// This sets up the necessary registers before TXDC calibration can be performed.
/// Based on SDK `bt_rfc_txdc_cal()`.
pub(super) fn configure_for_txdc_cal() {
    // =====================================================================
    // Replicate BT_TXON analog enables that force_tx (TXON) does NOT set.
    // force_tx triggers the TXON command sequence (BLE TX), but TXDC
    // calibration needs the BT_TXON (EDR TX) signal chain:
    //   TX modulator -> PA -> power meter -> loopback -> RVGA -> ADC -> PHY dump
    // We must manually enable every analog block in this chain.
    // =====================================================================

    // [BT_TXON step 1] VDDPSW/RFBG_EN/LO_IARY_EN (offset 0x10, bits 16,17,18)
    // These are cleared by VCO cal cleanup; SDK re-enables them in edrlo_3g_cal.
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_en_rfbg_lv(true);
        w.set_brf_en_vddpsw_lv(true);
        w.set_brf_lo_iary_en_lv(true);
        // [BT_TXON] LODISTEDR_EN (offset 0x10, bit 0)
        w.set_brf_lodistedr_en_lv(true);
    });

    // [BT_TXON step 2] LDO_RBB (offset 0x48, bit 13) - RBB LDO for receive baseband
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_en_ldo_rbb_lv(true);
    });

    // [BT_TXON] VCO3G_EN + EDR_VCO_FLT_EN (offset 0x00, bits 13,7)
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco3g_en_lv(true);
        w.set_brf_vco_flt_en_lv(true);
    });

    // [BT_TXON] EDR_EN_OSLO (offset 0x28, bit 11)
    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_en_lv(true);
    });

    // [BT_TXON] EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC (offset 0x64, bits 8-11)
    BT_RFC.tbb_reg().modify(|w| {
        w.set_brf_en_tbb_iarray_lv(true);
        w.set_brf_en_ldo_dac_dvdd_lv(true);
        w.set_brf_en_ldo_dac_avdd_lv(true);
        w.set_brf_en_dac_lv(true);
    });

    // [BT_TXON] TRF_EDR_IARRAY_EN, EDR_TMXBUF_PU, EDR_TMX_PU, EDR_PA_PU (offset 0x3C)
    BT_RFC.trf_edr_reg1().modify(|w| {
        w.set_brf_trf_edr_iarray_en_lv(true);
        w.set_brf_trf_edr_tmxbuf_pu_lv(true);
        w.set_brf_trf_edr_tmx_pu_lv(true);
        w.set_brf_trf_edr_pa_pu_lv(true);
    });

    // [BT_TXON] EDR_PACAP_EN, EDR_PA_XFMR_SG (offset 0x40, bits 11,17)
    // Also PWRMTR_EN (bit 10) - moved here to be together with other 0x40 enables
    BT_RFC.trf_edr_reg2().modify(|w| {
        w.set_brf_trf_edr_pacap_en_lv(true);
        w.set_brf_trf_edr_pa_xfmr_sg_lv(true);
    });

    // [BT_TXON] RBB_REG5: EN_IARRAY (offset 0x58, bit 5)
    // TX_LPBK_EN (bit 0) is added later to match BT_TXON ordering
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_en_iarray_lv(true);
    });

    // [BT_TXON] EN_RVGA_I (offset 0x4C, bit 7)
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_en_rvga_i_lv(true);
    });

    // [BT_TXON] ADC: LDO_ADCREF (bit 4), LDO_ADC (bit 9), ADC_I enable (bit 21)
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_en_ldo_adcref_lv(true);
        w.set_brf_en_ldo_adc_lv(true);
        w.set_brf_en_adc_i_lv(true);
    });

    // Set BR/BLE/EDR TX to IQ modulation
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mod_method_ble(true);
        w.set_mod_method_br(true);
        w.set_mod_method_edr(true);
    });

    // Force to BR mode and channel 38 (2440 MHz)
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_nbt_ble(true);
        w.set_force_channel(true);
        w.set_force_syncword(true);
        w.set_channel(38);
    });

    // Force IQ power to level 0
    BT_MAC.aescntl().modify(|w| {
        w.set_force_polar_pwr(true);
        w.set_force_polar_pwr_val(0);
    });

    // Enable TX DC calibration module
    BT_PHY.tx_dc_cal_cfg0().modify(|w| {
        w.set_tx_dc_cal_en(true);
    });

    // Disable ADC Q for calibration
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_adc_q_en_1(false);
    });

    // Configure PFD/CP current
    BT_RFC.pfdcp_reg().modify(|w| {
        w.set_brf_pfdcp_icp_set_lv(4);
    });

    // Configure LPF parameters
    BT_RFC.lpf_reg().modify(|w| {
        w.set_brf_lpf_rz_sel_lv(4);
        w.set_brf_lpf_rp4_sel_lv(5);
        w.set_brf_lpf_cz_sel_lv(2);
        w.set_brf_lpf_cp3_sel_lv(2);
    });

    // Configure TRF EDR REG2 for power meter and ENABLE the power meter.
    BT_RFC.trf_edr_reg2().modify(|w| {
        w.set_brf_trf_edr_pwrmtr_gc_lv(0);
        w.set_brf_trf_edr_pwrmtr_bm_lv(3);
        w.set_brf_trf_edr_pwrmtr_os_lv(0x0F);
        w.set_brf_trf_edr_pwrmtr_os_pn_lv(true);
        w.set_brf_trf_edr_pwrmtr_en_lv(true); // [BT_TXON] Power meter enable (0x40, bit 10)
    });

    // [BT_TXON] TX loopback: routes power meter output -> RVGA -> ADC (0x58, bit 0)
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_rvga_tx_lpbk_en_lv(true);
    });

    // Configure RBB gain
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_rvga_gc_lv(0xC);
    });

    // Enable VGA gain force
    BT_RFC.agc_reg().modify(|w| {
        w.set_vga_gain_frc_en(true);
    });

    // Configure RBB REG3
    BT_RFC.rbb_reg3().modify(|w| {
        w.set_brf_rvga_vcmref_lv(0);
        w.set_brf_rvga_vstart_lv(0);
    });

    // Enable EDR crystal reference (SDK sets this in bt_rfc_edrlo_3g_cal, line 3141).
    // Without this, the IQ modulator in EDR mode produces no signal and ADC reads 0x200.
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_edr_xtal_ref_en_frc_en(true);
        w.set_edr_xtal_ref_en(true);
        w.set_adc_clk_en_frc_en(true);
        w.set_adc_clk_en(true);
    });

    // Enable RX force on and PHY dump
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_force_rx_on(true);
        w.set_phy_rx_dump_en(true);
        w.set_rx_dump_data_sel(0);
    });

    // Force TX on to lock LO
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx(true);
        w.set_force_tx_val(true);
    });

    // Wait for LO lock (BT_TXON waits 30us for LO lock, we use 100us for margin)
    crate::cortex_m_blocking_delay_us(100);

    // [BT_TXON] DAC_START (offset 0x64, bit 12) - must be after LO lock
    BT_RFC.tbb_reg().modify(|w| {
        w.set_brf_dac_start_lv(true);
    });

    // Small delay for DAC to settle
    crate::cortex_m_blocking_delay_us(10);

    // Disable auto INCCAL to prevent force_tx from corrupting INCCAL registers (SDK line 3083)
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(false);
        w.set_vco3g_auto_incfcal_en(false);
    });
    BT_RFC.inccal_reg2().modify(|w| {
        w.set_vco5g_auto_incacal_en(false);
        w.set_vco5g_auto_incfcal_en(false);
    });
}

/// Configure power level for TXDC calibration.
///
/// Sets up PA BM, TMXBUF gain, and other power-level specific registers.
pub(super) fn configure_power_level(level: usize, config: &TxdcCalConfig) {
    let pa_bm = config.edr_pa_bm[level];
    let tmxbuf_gc = config.tmxbuf_gc[level];

    // Set EDR PA BM
    BT_RFC.iq_pwr_reg2().modify(|w| {
        w.set_brf_trf_edr_pa_bm_lv(pa_bm & 0x1F);
    });

    // Set TMXBUF gain control
    BT_RFC.iq_pwr_reg1().modify(|w| {
        w.set_edr_tmxbuf_gc_gfsk(tmxbuf_gc & 0x0F);
    });

    // Configure TX DC CAL gain based on power level
    let dc_cal_gain = match level {
        0 | 1 | 2 => 0x60,
        3 => 0x70,
        4 => 0x50,
        5 => 0x30,
        6 => 0x18,
        _ => 0x60,
    };

    BT_PHY.tx_dc_cal_cfg2().write(|w| {
        w.set_tx_dc_cal_gain0(dc_cal_gain & 0x0F);
        w.set_tx_dc_cal_gain1((dc_cal_gain >> 4) & 0x0F);
    });

    // Additional power-level specific configuration
    match level {
        2 => {
            BT_RFC.rbb_reg2().modify(|w| {
                w.set_brf_rvga_gc_lv(4);
            });
        }
        3 => {
            BT_RFC.rbb_reg2().modify(|w| {
                w.set_brf_rvga_gc_lv(0x10);
            });
            BT_RFC.trf_edr_reg2().modify(|w| {
                w.set_brf_trf_edr_pwrmtr_gc_lv(0);
            });
        }
        4 => {
            BT_RFC.trf_edr_reg1().modify(|w| {
                w.set_brf_trf_edr_tmxbuf_ibld_lv(0);
            });
            BT_RFC.trf_edr_reg2().modify(|w| {
                w.set_brf_trf_edr_pwrmtr_gc_lv(0);
            });
            BT_RFC.rbb_reg2().modify(|w| {
                w.set_brf_rvga_gc_lv(0x10);
            });
        }
        5 => {
            BT_RFC.rbb_reg2().modify(|w| {
                w.set_brf_rvga_gc_lv(0);
            });
        }
        6 => {
            BT_RFC.trf_edr_reg2().modify(|w| {
                w.set_brf_trf_edr_pwrmtr_gc_lv(0);
                w.set_brf_trf_edr_pwrmtr_bm_lv(1);
            });
            BT_RFC.rbb_reg2().modify(|w| {
                w.set_brf_rvga_gc_lv(6);
            });
        }
        _ => {}
    }
}

/// Cleanup after TXDC calibration.
///
/// Restores all MAC/PHY registers modified by `configure_for_txdc_cal()`
/// to their default state so normal BLE operation can proceed.
pub(super) fn cleanup_txdc_cal() {
    // DMA channel cleanup is handled by Transfer::Drop (HAL DMA API)

    // Disable TX DC calibration module (SDK line 4912)
    BT_PHY.tx_dc_cal_cfg0().modify(|w| {
        w.set_tx_dc_cal_en(false);
    });

    // Clear force flags except force_tx (SDK lines 4917-4918)
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_nbt_ble(false);
        w.set_force_channel(false);
        w.set_force_syncword(false);
    });

    // Release TX in two steps with delays (SDK lines 4924-4928)
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx_val(false);
    });
    crate::cortex_m_blocking_delay_us(20);
    BT_MAC.dmradiocntl1().modify(|w| {
        w.set_force_tx(false);
    });
    crate::cortex_m_blocking_delay_us(20);

    // Restore TX modulation to polar mode (SDK lines 4931-4932)
    BT_PHY.tx_ctrl().modify(|w| {
        w.set_mac_mod_ctrl_en(true);
        w.set_mod_method_ble(false);
        w.set_mod_method_br(false);
    });

    // Clear IQ power force (SDK line 4936)
    BT_MAC.aescntl().modify(|w| {
        w.set_force_polar_pwr(false);
    });

    // Restore IQ_PWR normal working values (SDK lines 4672-4679)
    BT_RFC.iq_pwr_reg2().modify(|w| {
        w.set_brf_trf_edr_pa_bm_lv(7);
    });
    BT_RFC.iq_pwr_reg1().modify(|w| {
        w.set_edr_tmxbuf_gc_gfsk(7);
    });

    // Restore PHY RX state -- do NOT clear phy_rx_dump_en (SDK preserves it)
    BT_PHY.rx_ctrl1().modify(|w| {
        w.set_force_rx_on(false);
        w.set_adc_q_en_1(true);
    });

    // Clear ADC clock force, adc_clk_en, and EDR xtal ref force
    // SDK clears only frc_en (line 3572), preserving edr_xtal_ref_en for later use
    BT_RFC.misc_ctrl_reg().modify(|w| {
        w.set_adc_clk_en_frc_en(false);
        w.set_adc_clk_en(false);
        w.set_edr_xtal_ref_en_frc_en(false);
    });

    // Disable all BT_TXON analog enables we set in configure_for_txdc_cal().
    // Power meter and TX loopback
    BT_RFC.trf_edr_reg2().modify(|w| {
        w.set_brf_trf_edr_pwrmtr_en_lv(false);
        w.set_brf_trf_edr_pacap_en_lv(false);
        w.set_brf_trf_edr_pa_xfmr_sg_lv(false);
    });
    BT_RFC.rbb_reg5().modify(|w| {
        w.set_brf_rvga_tx_lpbk_en_lv(false);
        w.set_brf_en_iarray_lv(false);
    });
    // TRF EDR REG1 enables
    BT_RFC.trf_edr_reg1().modify(|w| {
        w.set_brf_trf_edr_iarray_en_lv(false);
        w.set_brf_trf_edr_tmxbuf_pu_lv(false);
        w.set_brf_trf_edr_tmx_pu_lv(false);
        w.set_brf_trf_edr_pa_pu_lv(false);
    });
    // DAC / TBB block
    BT_RFC.tbb_reg().modify(|w| {
        w.set_brf_dac_start_lv(false);
        w.set_brf_en_dac_lv(false);
        w.set_brf_en_ldo_dac_avdd_lv(false);
        w.set_brf_en_ldo_dac_dvdd_lv(false);
        w.set_brf_en_tbb_iarray_lv(false);
    });
    // ADC block
    BT_RFC.adc_reg().modify(|w| {
        w.set_brf_en_adc_i_lv(false);
        w.set_brf_en_ldo_adc_lv(false);
        w.set_brf_en_ldo_adcref_lv(false);
    });
    // LDO_RBB
    BT_RFC.rbb_reg1().modify(|w| {
        w.set_brf_en_ldo_rbb_lv(false);
    });
    // OSLO
    BT_RFC.oslo_reg().modify(|w| {
        w.set_brf_oslo_en_lv(false);
    });
    // RVGA_I (in rbb_reg2, cleared with gc_lv below)
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_en_rvga_i_lv(false);
    });
    // LODIST EDR
    BT_RFC.rf_lodist_reg().modify(|w| {
        w.set_brf_lodistedr_en_lv(false);
    });

    // Clear VGA gain force
    BT_RFC.agc_reg().modify(|w| {
        w.set_vga_gain_frc_en(false);
    });

    // Reset RBB VGA gain (SDK line 4685)
    BT_RFC.rbb_reg2().modify(|w| {
        w.set_brf_rvga_gc_lv(0);
    });

    // Restore RBB vcmref/vstart (SDK line 4971)
    BT_RFC.rbb_reg3().modify(|w| {
        w.set_brf_rvga_vcmref_lv(2);
        w.set_brf_rvga_vstart_lv(2);
    });

    // Restore auto INCCAL (SDK lines 3576-3577)
    BT_RFC.inccal_reg1().modify(|w| {
        w.set_vco3g_auto_incacal_en(true);
        w.set_vco3g_auto_incfcal_en(true);
    });
    BT_RFC.inccal_reg2().modify(|w| {
        w.set_vco5g_auto_incacal_en(true);
        w.set_vco5g_auto_incfcal_en(true);
    });
}
