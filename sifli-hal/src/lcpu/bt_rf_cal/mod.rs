//! Bluetooth RF calibration.
//!
//! This module implements the RF calibration sequence required for BLE/BR operation:
//! - RFC hardware initialization and command sequence generation
//! - VCO frequency calibration (79 channels)
//! - TX DC offset calibration (7 power levels)
//! - Post-calibration PHY optimization
//! - Calibration table storage in RFC SRAM
//!
//! Based on SDK `bt_rf_cal()` and related functions in `bt_rf_fulcal.c`.

mod consts;
#[cfg(feature = "edr-cal")]
mod edr_lo;
mod opt;
pub mod rfc_cmd;
pub mod rfc_tables;
pub mod txdc;
mod txdc_hw;
pub mod vco;

use crate::dma::Channel;
use crate::efuse::{Bank1Calibration, Efuse};
use crate::pac::{BT_PHY, BT_RFC};
use crate::rcc::{lp_rfc_reset_asserted, set_lp_rfc_reset};
use crate::Peripheral;

/// RFC SRAM base address
const BT_RFC_MEM_BASE: u32 = super::memory_map::rf::BT_RFC_MEM_BASE;

/// RF driver version: v6.0.0.
const RF_DRIVER_VERSION: u32 = 0x0006_0000;

/// Default EDR PA BM values for each power level (0-7)
///
/// These values are adjusted based on eFUSE calibration data.
/// See SDK `bt_rfc_pwr_cal_edr()` in `bt_rf_fulcal.c`.
const DEFAULT_EDR_PA_BM: [u8; 8] = [5, 5, 0xE, 0xA, 0x1B, 0x1F, 0x1F, 0x1F];

/// Reset Bluetooth RF module.
///
/// Corresponds to `HAL_RCC_ResetBluetoothRF` in SDK.
fn reset_bluetooth_rf() {
    // Set RFC reset bit
    set_lp_rfc_reset(true);
    // Wait for bit to take effect
    while !lp_rfc_reset_asserted() {}
    // Clear RFC reset bit
    set_lp_rfc_reset(false);
}

/// Apply EDR power calibration from eFUSE Bank1 calibration data.
///
/// Applies calibration to:
/// - TBB_REG.BRF_DAC_LSB_CNT_LV field
/// - EDR PA BM table adjustments
///
/// Based on SDK `bt_rfc_pwr_cal_edr()` with `ABS_EDR_CAL` enabled.
///
/// Returns the adjusted EDR PA BM values if calibration was applied, None otherwise.
pub fn apply_edr_power_cal(cal: &Bank1Calibration) -> Option<[u8; 8]> {
    let low = &cal.primary.low;
    let high = &cal.primary.high;

    debug!(
        "eFUSE RF params: edr_cal={} pa_bm={} dac_lsb={} tmxcap_flag={} tmxcap_00={} tmxcap_78={}",
        low.edr_cal_done(),
        low.pa_bm(),
        high.dac_lsb_cnt(),
        high.tmxcap_flag(),
        high.tmxcap_ch00(),
        high.tmxcap_ch78()
    );

    if !low.edr_cal_done() {
        debug!("EDR cal flag not set, skipping EDR power cal");
        return None;
    }

    // Apply DAC LSB count calibration to TBB_REG
    BT_RFC.tbb_reg().modify(|w| {
        w.set_brf_dac_lsb_cnt_lv(high.dac_lsb_cnt());
    });

    // Adjust EDR PA BM values based on pa_bm
    let mut edr_pa_bm = DEFAULT_EDR_PA_BM;
    match low.pa_bm() {
        1 => {
            edr_pa_bm[0] = edr_pa_bm[0].saturating_add(1);
            edr_pa_bm[1] = edr_pa_bm[1].saturating_add(2);
            edr_pa_bm[2] = edr_pa_bm[2].saturating_add(2);
            edr_pa_bm[3] = edr_pa_bm[3].saturating_add(2);
            edr_pa_bm[4] = edr_pa_bm[4].saturating_add(4);
        }
        3 => {
            edr_pa_bm[1] = edr_pa_bm[1].saturating_sub(1);
            edr_pa_bm[2] = edr_pa_bm[2].saturating_sub(2);
            edr_pa_bm[3] = edr_pa_bm[3].saturating_sub(2);
            edr_pa_bm[4] = edr_pa_bm[4].saturating_sub(3);
        }
        _ => {}
    }

    Some(edr_pa_bm)
}

/// Get TMXCAP selection values from eFUSE Bank1 calibration data.
///
/// Returns (tmxcap_ch00, tmxcap_ch78) if TMXCAP calibration flag is set.
#[allow(dead_code)]
pub fn get_tmxcap_sel(cal: &Bank1Calibration) -> Option<(u8, u8)> {
    if !cal.primary.high.tmxcap_flag() {
        return None;
    }
    Some((
        cal.primary.high.tmxcap_ch00(),
        cal.primary.high.tmxcap_ch78(),
    ))
}

/// Default BT RF power parameters.
fn default_tx_power_params() -> (i8, i8, i8, u8) {
    let max_pwr: i8 = 10;
    let init_pwr: i8 = 0;
    let min_pwr: i8 = 0;
    let is_bqb: u8 = 0;
    (max_pwr, min_pwr, init_pwr, is_bqb)
}

/// Power table for calibration level index mapping (dBm).
/// SDK: `pwr_tab[] = {0, 3, 6, 10, 13, 16, 19}` in bt_rf_cal_index().
const PWR_TAB: [i8; 7] = [0, 3, 6, 10, 13, 16, 19];

/// Compute calibration enable bitmask from TX power range.
///
/// Determines which of the 7 power levels need TXDC calibration based on the
/// configured min/max/init TX power. Only levels within the active range are
/// calibrated; others use default values.
///
/// Corresponds to SDK `bt_rf_cal_index()` (bt_rf_fulcal.c:5172).
fn bt_rf_cal_index(min_pwr: i8, max_pwr: i8, init_pwr: i8) -> u8 {
    let effective_max = max_pwr.max(init_pwr);

    // Find lowest level where pwr_tab[i] <= min_pwr (search from top)
    let mut min_level: usize = 0;
    for i in (0..PWR_TAB.len()).rev() {
        if PWR_TAB[i] <= min_pwr {
            min_level = i;
            break;
        }
    }

    // Find highest level where pwr_tab[i] >= effective_max (search from bottom)
    let mut max_level: usize = PWR_TAB.len() - 1;
    for i in 0..PWR_TAB.len() {
        if PWR_TAB[i] >= effective_max {
            max_level = i;
            break;
        }
    }

    let mut cal_enable: u8 = 0;
    for i in min_level..=max_level {
        cal_enable |= 1 << i;
    }
    cal_enable
}

/// Encode power parameters into 32-bit packed format.
fn encode_tx_power(max: i8, min: i8, init: i8, is_bqb: u8) -> u32 {
    let max_u = max as u8 as u32;
    let min_u = min as u8 as u32;
    let init_u = init as u8 as u32;
    let is_bqb_u = is_bqb as u32;

    (is_bqb_u << 24) | (init_u << 16) | (min_u << 8) | max_u
}

/// Dump all key RF registers for comparison with SDK.
fn rf_dump_checkpoint(name: &str) {
    debug!("\n===== CHECKPOINT: {} =====", name);
    debug!(
        "CU_ADDR: R1=0x{:08X} R2=0x{:08X} R3=0x{:08X}",
        BT_RFC.cu_addr_reg1().read().0,
        BT_RFC.cu_addr_reg2().read().0,
        BT_RFC.cu_addr_reg3().read().0
    );
    debug!(
        "CAL_ADDR: R1=0x{:08X} R2=0x{:08X} R3=0x{:08X}",
        BT_RFC.cal_addr_reg1().read().0,
        BT_RFC.cal_addr_reg2().read().0,
        BT_RFC.cal_addr_reg3().read().0
    );
    debug!(
        "VCO: R1=0x{:08X} R2=0x{:08X} R3=0x{:08X}",
        BT_RFC.vco_reg1().read().0,
        BT_RFC.vco_reg2().read().0,
        BT_RFC.vco_reg3().read().0
    );
    debug!(
        "MISC=0x{:08X} FBDV1=0x{:08X} FBDV2=0x{:08X}",
        BT_RFC.misc_ctrl_reg().read().0,
        BT_RFC.fbdv_reg1().read().0,
        BT_RFC.fbdv_reg2().read().0
    );
    debug!(
        "ADC=0x{:08X} LPF=0x{:08X} PFDCP=0x{:08X}",
        BT_RFC.adc_reg().read().0,
        BT_RFC.lpf_reg().read().0,
        BT_RFC.pfdcp_reg().read().0
    );
    debug!(
        "TRF1=0x{:08X} TRF2=0x{:08X} TBB=0x{:08X}",
        BT_RFC.trf_reg1().read().0,
        BT_RFC.trf_reg2().read().0,
        BT_RFC.tbb_reg().read().0
    );
    debug!(
        "RBB1=0x{:08X} RBB2=0x{:08X} RBB4=0x{:08X}",
        BT_RFC.rbb_reg1().read().0,
        BT_RFC.rbb_reg2().read().0,
        BT_RFC.rbb_reg4().read().0
    );
    debug!(
        "INCCAL1=0x{:08X} INCCAL2=0x{:08X}",
        BT_RFC.inccal_reg1().read().0,
        BT_RFC.inccal_reg2().read().0
    );
    debug!(
        "IQ_PWR1=0x{:08X} IQ_PWR2=0x{:08X}",
        BT_RFC.iq_pwr_reg1().read().0,
        BT_RFC.iq_pwr_reg2().read().0
    );
    debug!(
        "PHY: RX_CTRL1=0x{:08X} TX_CTRL=0x{:08X}",
        BT_PHY.rx_ctrl1().read().0,
        BT_PHY.tx_ctrl().read().0
    );
    debug!(
        "PHY: LFP_CFG=0x{:08X} HFP_CFG=0x{:08X}",
        BT_PHY.tx_lfp_cfg().read().0,
        BT_PHY.tx_hfp_cfg().read().0
    );
    debug!(
        "PHY: IF_MOD3=0x{:08X} IF_MOD5=0x{:08X}",
        BT_PHY.tx_if_mod_cfg3().read().0,
        BT_PHY.tx_if_mod_cfg5().read().0
    );
    debug!(
        "PHY: DEMOD1=0x{:08X} MIXER1=0x{:08X}",
        BT_PHY.demod_cfg1().read().0,
        BT_PHY.mixer_cfg1().read().0
    );
    debug!(
        "PHY: GAUSS1=0x{:08X} GAUSS2=0x{:08X}",
        BT_PHY.tx_gaussflt_cfg1().read().0,
        BT_PHY.tx_gaussflt_cfg2().read().0
    );
    unsafe {
        let w0 = core::ptr::read_volatile(BT_RFC_MEM_BASE as *const u32);
        let w1 = core::ptr::read_volatile((BT_RFC_MEM_BASE + 4) as *const u32);
        debug!("SRAM[0x000]=0x{:08X} [0x004]=0x{:08X}", w0, w1);
    }
}

/// Perform Bluetooth RF calibration.
///
/// Corresponds to SDK call chain:
/// ```text
/// lcpu_ble_patch_install()           // bf0_lcpu_init.c:179
///   ├─ lcpu_patch_install()          // patch (done by caller)
///   ├─ bt_rf_cal()                   // bt_rf_fulcal.c:5451
///   │   ├─ bt_rf_cal_index()         //   compute s_cal_enable mask
///   │   ├─ HAL_RCC_ResetBluetoothRF()
///   │   ├─ bt_rfc_init()             //   RFC regs + command sequences → returns addr
///   │   ├─ bt_ful_cal(addr)
///   │   │   ├─ bt_rfc_lo_cal()       //     BLE VCO ACAL/FCAL 79ch
///   │   │   ├─ bt_rfc_edrlo_3g_cal() //     EDR LO + OSLO (uses GPADC)
///   │   │   └─ bt_rfc_txdc_cal()     //     TX DC offset (DMA-based)
///   │   ├─ bt_rf_opt_cal()           //   PHY register optimization
///   │   ├─ RSVD_REG2 = version
///   │   └─ HAL_LCPU_CONFIG_set(BT_TX_PWR)
///   ├─ adc_resume()                  // re-init GPADC after OSLO touched it
///   └─ memset(EM, 0, 0x5000)         // clear Exchange Memory
/// ```
pub fn bt_rf_cal(dma_ch: impl Peripheral<P = impl Channel>) {
    // TODO: bt_is_in_BQB_mode() check (SDK:5453) — always assumes non-BQB
    // SDK:5461 — bt_rf_cal_index(): compute s_cal_enable from power range
    let (max_pwr, min_pwr, init_pwr, _is_bqb) = default_tx_power_params();
    let cal_enable = bt_rf_cal_index(min_pwr, max_pwr, init_pwr);

    // SDK:5465 — HAL_RCC_ResetBluetoothRF()
    reset_bluetooth_rf();

    // SDK:5471 — PA voltage mode (non-1.8V): clear TMXCAS_SEL
    BT_RFC.trf_edr_reg1().modify(|w| {
        w.set_brf_trf_edr_tmxcas_sel_lv(false);
    });

    // SDK:5473 — bt_rfc_init(): RFC register init + 6 command sequences → addr
    vco::rfc_init();
    let cmd_end_addr = rfc_cmd::generate_rfc_cmd_sequences();
    rf_dump_checkpoint("AFTER_RFC_INIT");

    // SDK:5072 bt_ful_cal — step a: bt_rfc_lo_cal()
    // BLE VCO calibration: ACAL+FCAL for 79 TX / 40 RX_1M / 40 RX_2M / 79 RX_BT
    // Includes PACAL, ROSCAL (RX DC offset), RCCAL sub-steps.
    let vco_cal = vco::vco_cal_full();
    debug!(
        "VCO cal result: tx[0] idac={} capcode={} kcal={}, tx[39] idac={} capcode={} kcal={}",
        vco_cal.idac_tx[0],
        vco_cal.capcode_tx[0],
        vco_cal.kcal[0],
        vco_cal.idac_tx[39],
        vco_cal.capcode_tx[39],
        vco_cal.kcal[39],
    );
    rf_dump_checkpoint("AFTER_VCO_CAL");

    // SDK:5078 bt_ful_cal — step b: bt_rfc_edrlo_3g_cal()
    #[cfg(feature = "edr-cal")]
    let edr_lo_result = edr_lo::edr_lo_cal_full();
    #[cfg(feature = "edr-cal")]
    {
        debug!(
            "EDR LO cal: ch[0] fc={} bm={}, ch[39] fc={} bm={}",
            edr_lo_result.oslo_fc[0],
            edr_lo_result.oslo_bm[0],
            edr_lo_result.oslo_fc[39],
            edr_lo_result.oslo_bm[39]
        );
        rf_dump_checkpoint("AFTER_EDR_LO_CAL");
    }

    // SDK:5085-5086 bt_ful_cal — step c: LPSYS clock switch before TXDC
    // TODO: hwp_lpsys_rcc->CSR = (CSR & ~SEL_SYS) | (1 << SEL_SYS_Pos)
    //   May affect TXDC sampling timing accuracy.

    // SDK:5088 bt_ful_cal — step d: bt_rfc_txdc_cal(addr, s_cal_enable)
    // Note: SDK does EDR eFUSE power cal inside bt_rfc_txdc_cal (line 3753-3791);
    // we extract it here — the result is equivalent.
    // Read eFUSE calibration data for EDR power calibration
    let efuse_cal = unsafe { Efuse::new(crate::peripherals::EFUSEC::steal()) }
        .ok()
        .map(|e| *e.calibration());
    let edr_pa_bm_opt = efuse_cal.as_ref().and_then(apply_edr_power_cal);
    match &edr_pa_bm_opt {
        Some(pa_bm) => debug!(
            "EDR power cal applied: PA_BM=[{},{},{},{},{},{},{},{}]",
            pa_bm[0], pa_bm[1], pa_bm[2], pa_bm[3], pa_bm[4], pa_bm[5], pa_bm[6], pa_bm[7]
        ),
        None => debug!("EDR power cal skipped (no eFUSE data or flag not set)"),
    }

    // Store VCO cal tables first — force_tx needs CAL_ADDR to look up VCO params.
    let txdc_table_addr = rfc_tables::store_vco_cal_tables(cmd_end_addr, &vco_cal);

    // Overwrite BT TX table with EDR LO results (idac, capcode, oslo_fc, oslo_bm, dpsk_gain)
    #[cfg(feature = "edr-cal")]
    rfc_tables::store_edr_lo_cal_tables(&edr_lo_result);

    let mut txdc_config = txdc::TxdcCalConfig::default();
    txdc_config.power_level_mask = cal_enable;
    if let Some(pa_bm) = edr_pa_bm_opt {
        txdc_config.edr_pa_bm = pa_bm;
    }

    // TODO: TMXCAP eFUSE calibration (SDK:3818-3842)
    //   Reads tmxcap_sel from eFUSE, fills tmxcap_sel[79] per-channel array.

    let txdc_cal = txdc::txdc_cal_full(edr_pa_bm_opt, cal_enable, dma_ch);
    debug!(
        "TXDC cal[0]: oi={} oq={} c0={} c1={}",
        txdc_cal.points[0].offset_i,
        txdc_cal.points[0].offset_q,
        txdc_cal.points[0].coef0,
        txdc_cal.points[0].coef1
    );

    // Restore VCO thresholds to normal mode after TXDC cal (SDK:4664-4673)
    BT_RFC.vco_reg2().modify(|w| {
        w.set_brf_vco_acal_vl_sel_lv(consts::VCO_ACAL_VL_NORMAL);
        w.set_brf_vco_acal_vh_sel_lv(consts::VCO_ACAL_VH_NORMAL);
        w.set_brf_vco_incfcal_vl_sel_lv(consts::VCO_INCFCAL_VL);
        w.set_brf_vco_incfcal_vh_sel_lv(consts::VCO_INCFCAL_VH);
    });
    BT_RFC.vco_reg1().modify(|w| {
        w.set_brf_vco_ldo_vref_lv(consts::VCO_LDO_VREF);
    });
    rf_dump_checkpoint("AFTER_TXDC_CAL");

    // SDK:5477 — bt_rf_opt_cal()
    opt::bt_rf_opt_cal();
    rf_dump_checkpoint("AFTER_OPT_CAL");

    // SDK:5481 — store driver version (v6.0.0)
    vco::set_driver_version(RF_DRIVER_VERSION);

    // TODO: BQB co-channel config (SDK:5482-5486, BR_BQB_COCHANNEL_CASE)
    //   DEMOD_CFG8 BR_DEMOD_G/MU_DC/MU_ERR, DEMOD_CFG16 BR_HADAPT_EN

    // SDK:5488-5492 — save TX power params to LCPU ROM config
    let tx_pwr = encode_tx_power(max_pwr, min_pwr, init_pwr, _is_bqb);
    crate::lcpu::ram::set_bt_tx_power(tx_pwr);

    // Store TXDC cal tables into RFC SRAM.
    // SDK does this inside bt_rfc_txdc_cal; we do it after opt_cal for cleaner ordering.
    rfc_tables::store_txdc_cal_tables(
        txdc_table_addr,
        &txdc_cal,
        &txdc_config.edr_pa_bm,
        &txdc_config.tmxbuf_gc,
    );

    // TODO: adc_resume() (SDK bf0_lcpu_init.c:205)
    //   Re-initializes GPADC after OSLO cal may have modified its registers.
    //   Not needed now (OSLO not implemented), must add when EDR LO cal is done.

    // SDK bf0_lcpu_init.c:208 — clear Exchange Memory
    unsafe {
        core::ptr::write_bytes(
            super::memory_map::shared::EM_START as *mut u8,
            0,
            super::memory_map::shared::EM_SIZE,
        );
    }
}
