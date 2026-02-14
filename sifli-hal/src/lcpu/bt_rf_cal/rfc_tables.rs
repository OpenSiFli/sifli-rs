//! RFC calibration table storage in SRAM.
//!
//! Stores VCO and TXDC calibration result tables into RFC SRAM so the BLE MAC
//! can load per-channel VCO parameters (via `RD_FULCAL`) and per-power-level
//! TXDC parameters (via `RD_DCCAL1`/`RD_DCCAL2`) from tables addressed by
//! `CAL_ADDR_REG1/2/3`.

use crate::pac::BT_RFC;
#[cfg(feature = "edr-cal")]
use super::edr_lo::{EdrLoCalResult, DPSK_GAIN_INITIAL};
use super::vco::VcoCalResult;
use super::txdc::{TxdcCalResult, NUM_POWER_LEVELS};

/// Write VCO calibration tables (RX + TX) to RFC SRAM and update CAL_ADDR_REG1/REG2.
///
/// Must be called after VCO calibration and BEFORE TXDC calibration,
/// because TXDC's force_tx needs CAL_ADDR to look up the correct VCO
/// parameters for the forced channel.
///
/// Returns the next free SRAM offset (for TXDC tables).
pub fn store_vco_cal_tables(
    cmd_end_addr: u32,
    vco_cal: &VcoCalResult,
) -> u32 {
    let base = super::BT_RFC_MEM_BASE;
    let mut addr = cmd_end_addr;

    // Helper: write one u32 word to RFC SRAM and advance the offset.
    let write_word = |offset: &mut u32, val: u32| {
        unsafe {
            core::ptr::write_volatile((base + *offset) as *mut u32, val);
        }
        *offset += 4;
    };

    // === BLE RX calibration table (40 words) ===
    let ble_rx_addr = addr;
    for i in 0..40 {
        let rx_1m = (vco_cal.capcode_rx_1m[i] as u32) | ((vco_cal.idac_rx_1m[i] as u32) << 8);
        let rx_2m = (vco_cal.capcode_rx_2m[i] as u32) | ((vco_cal.idac_rx_2m[i] as u32) << 8);
        write_word(&mut addr, (rx_2m << 16) | rx_1m);
    }
    // === BT RX calibration table (40 words, packing 79 channels as pairs) ===
    let bt_rx_addr = addr;
    for i in 0..40 {
        let ch0 = 2 * i;
        let ch1 = 2 * i + 1;
        let lo = (vco_cal.capcode_rx_bt[ch0] as u32) | ((vco_cal.idac_rx_bt[ch0] as u32) << 8);
        let hi = if ch1 < 79 {
            (vco_cal.capcode_rx_bt[ch1] as u32) | ((vco_cal.idac_rx_bt[ch1] as u32) << 8)
        } else {
            lo // last odd channel: duplicate
        };
        write_word(&mut addr, (hi << 16) | lo);
    }
    // Set CAL_ADDR_REG1
    BT_RFC.cal_addr_reg1().write(|w| {
        w.set_ble_rx_cal_addr(ble_rx_addr as u16);
        w.set_bt_rx_cal_addr(bt_rx_addr as u16);
    });

    // === BLE TX calibration table (79 words) ===
    let ble_tx_addr = addr;
    for i in 0..79 {
        let word = (vco_cal.capcode_tx[i] as u32)
            | ((vco_cal.idac_tx[i] as u32) << 8)
            | ((vco_cal.kcal[i] as u32) << 16);
        write_word(&mut addr, word);
    }
    // === BT TX calibration table (79 words) -- same as BLE TX ===
    let bt_tx_addr = addr;
    for i in 0..79 {
        let word = (vco_cal.capcode_tx[i] as u32)
            | ((vco_cal.idac_tx[i] as u32) << 8)
            | ((vco_cal.kcal[i] as u32) << 16);
        write_word(&mut addr, word);
    }
    // Set CAL_ADDR_REG2
    BT_RFC.cal_addr_reg2().write(|w| {
        w.set_ble_tx_cal_addr(ble_tx_addr as u16);
        w.set_bt_tx_cal_addr(bt_tx_addr as u16);
    });

    addr
}

/// Write TXDC calibration tables to RFC SRAM and update CAL_ADDR_REG3.
///
/// Called after TXDC calibration with the calibration results.
/// `txdc_table_addr` is the SRAM offset returned by `store_vco_cal_tables`.
pub fn store_txdc_cal_tables(
    txdc_table_addr: u32,
    txdc_cal: &TxdcCalResult,
    edr_pa_bm: &[u8; 8],
    tmxbuf_gc: &[u8; 8],
) {
    let base = super::BT_RFC_MEM_BASE;
    let mut addr = txdc_table_addr;

    let write_word = |offset: &mut u32, val: u32| {
        unsafe {
            core::ptr::write_volatile((base + *offset) as *mut u32, val);
        }
        *offset += 4;
    };

    // === TXDC calibration table (8 power levels x 2 words = 16 words) ===
    let txdc_addr = addr;
    for level in 0..8usize {
        // SDK mapping: m=i; if(i>4) m=i-1  ->  [0,1,2,3,4,4,5,6]
        let m = if level > 4 { level - 1 } else { level };
        let m = m.min(NUM_POWER_LEVELS - 1);
        let pt = &txdc_cal.points[m];

        let word1 = (pt.coef0 as u32 & 0x3FFF)
            | (((pt.coef1 as u32) & 0x3FFF) << 14)
            | (((tmxbuf_gc[level] as u32) & 0xF) << 28);
        let word2 = (pt.offset_q as u32 & 0x7FF)
            | (((edr_pa_bm[level] as u32) & 0x1F) << 11)
            | (((pt.offset_i as u32) & 0x7FF) << 16)
            | (((tmxbuf_gc[level] as u32) & 0xF) << 28);

        write_word(&mut addr, word1);
        write_word(&mut addr, word2);
    }
    // Set CAL_ADDR_REG3
    BT_RFC.cal_addr_reg3().write(|w| {
        w.set_txdc_cal_addr(txdc_addr as u16);
    });

    // Replace EDR cal related commands in BT_TXON with WAIT commands
    // SDK: bt_rfc_txdc_cal lines 5001-5009
    let bt_txon_addr = BT_RFC.cu_addr_reg3().read().bt_txon_cfg_addr() as u32;
    for i in 0..10u32 {
        unsafe {
            core::ptr::write_volatile(
                (base + bt_txon_addr + 32 * 4 + i * 4) as *mut u32,
                0x5001_5001, // Two WAIT(1) commands
            );
        }
    }

    // Diagnostic log
    let r1 = BT_RFC.cal_addr_reg1().read();
    let r2 = BT_RFC.cal_addr_reg2().read();
    let r3 = BT_RFC.cal_addr_reg3().read();
    debug!(
        "CAL_ADDR: REG1=0x{:08X} REG2=0x{:08X} REG3=0x{:08X}",
        r1.0, r2.0, r3.0
    );
}

/// Overwrite BT TX calibration table with final EDR LO results (idac, capcode,
/// oslo_fc, oslo_bm, dpsk_gain, tmxcap).
///
/// Called after OSLO calibration completes. Overwrites the BT TX region
/// previously written by `store_vco_cal_tables` (or `edr_lo::store_initial_edr_table`).
/// The MAC hardware's BT_TXON `RD_FULCAL` command reads from this address
/// and loads the word into `EDR_CAL_REG1`.
///
/// Word format matches `EDR_CAL_REG1` bit layout:
/// - [7:0]   brf_edr_vco_pdx_lv (capcode)
/// - [14:8]  brf_edr_vco_idac_lv
/// - [15]    dpsk_gain bit 1
/// - [18:16] brf_oslo_fc_lv
/// - [19]    dpsk_gain bit 2
/// - [24:20] brf_oslo_bm_lv
/// - [27:25] dpsk_gain bits 4:2
/// - [31:28] brf_trf_edr_tmxcap_sel_lv (default 6)
#[cfg(feature = "edr-cal")]
pub fn store_edr_lo_cal_tables(edr_lo: &EdrLoCalResult) {
    let base = super::BT_RFC_MEM_BASE;
    let bt_tx_addr = BT_RFC.cal_addr_reg2().read().bt_tx_cal_addr() as u32;

    for i in 0..79usize {
        let mut word: u32 = 0;

        // [7:0] capcode (PDX)
        word |= edr_lo.capcode[i] as u32;
        // [14:8] IDAC
        word |= (edr_lo.idac[i] as u32) << 8;
        // [18:16] OSLO FC
        word |= (edr_lo.oslo_fc[i] as u32) << 16;
        // [24:20] OSLO BM
        word |= (edr_lo.oslo_bm[i] as u32) << 20;
        // [31:28] TMXCAP default=6
        word |= 6u32 << 28;

        // DPSK gain bits scattered across the word:
        // SDK: d0 = (dpsk_gain[i] >> 1) & 0x1  -> bit 15
        //      d1 = (dpsk_gain[i] >> 1) & 0x2  -> bit 19
        //      d2 = (dpsk_gain[i] >> 1) & 0x1c -> bits 27:25
        let dg = DPSK_GAIN_INITIAL[i] >> 1;
        let d0 = ((dg as u32) & 0x1) << 15;
        let d1 = ((dg as u32) & 0x2) << 18;
        let d2 = ((dg as u32) & 0x1c) << 23;
        word |= d0 | d1 | d2;

        unsafe {
            core::ptr::write_volatile(
                (base + bt_tx_addr + (i as u32) * 4) as *mut u32,
                word,
            );
        }
    }
}
