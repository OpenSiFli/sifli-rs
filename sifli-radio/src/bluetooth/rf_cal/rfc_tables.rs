//! RFC calibration table storage in SRAM.
//!
//! Stores VCO and TXDC calibration result tables into RFC SRAM so the BLE MAC
//! can load per-channel VCO parameters (via `RD_FULCAL`) and per-power-level
//! TXDC parameters (via `RD_DCCAL1`/`RD_DCCAL2`) from tables addressed by
//! `CAL_ADDR_REG1/2/3`.

#[cfg(feature = "edr")]
use super::edr_lo::{EdrLoCalResult, DPSK_GAIN_INITIAL};
use super::txdc::{TxdcCalResult, NUM_POWER_LEVELS};
use super::vco::VcoCalResult;
use crate::pac::BT_RFC;
#[cfg(feature = "edr")]
use rwbt::rfc::sifli::cal_table::pack_edr_cal;
use rwbt::rfc::sifli::cal_table::{pack_txdc, pack_vco_rx_half, pack_vco_rx_word, pack_vco_tx};

/// Write VCO calibration tables (RX + TX) to RFC SRAM and update CAL_ADDR_REG1/REG2.
///
/// Must be called after VCO calibration and BEFORE TXDC calibration,
/// because TXDC's force_tx needs CAL_ADDR to look up the correct VCO
/// parameters for the forced channel.
///
/// Returns the next free SRAM offset (for TXDC tables).
pub fn store_vco_cal_tables(cmd_end_addr: u32, vco_cal: &VcoCalResult) -> u32 {
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
        let rx_1m = pack_vco_rx_half(vco_cal.capcode_rx_1m[i], vco_cal.idac_rx_1m[i]);
        let rx_2m = pack_vco_rx_half(vco_cal.capcode_rx_2m[i], vco_cal.idac_rx_2m[i]);
        write_word(&mut addr, pack_vco_rx_word(rx_1m, rx_2m));
    }
    // === BT RX calibration table (40 words, packing 79 channels as pairs) ===
    let bt_rx_addr = addr;
    for i in 0..40 {
        let ch0 = 2 * i;
        let ch1 = 2 * i + 1;
        let lo = pack_vco_rx_half(vco_cal.capcode_rx_bt[ch0], vco_cal.idac_rx_bt[ch0]);
        let hi = if ch1 < 79 {
            pack_vco_rx_half(vco_cal.capcode_rx_bt[ch1], vco_cal.idac_rx_bt[ch1])
        } else {
            lo // last odd channel: duplicate
        };
        write_word(&mut addr, pack_vco_rx_word(lo, hi));
    }
    // Set CAL_ADDR_REG1
    BT_RFC.cal_addr_reg1().write(|w| {
        w.set_ble_rx_cal_addr(ble_rx_addr as u16);
        w.set_bt_rx_cal_addr(bt_rx_addr as u16);
    });

    // === BLE TX calibration table (79 words) ===
    let ble_tx_addr = addr;
    for i in 0..79 {
        write_word(
            &mut addr,
            pack_vco_tx(vco_cal.capcode_tx[i], vco_cal.idac_tx[i], vco_cal.kcal[i]),
        );
    }
    // === BT TX calibration table (79 words) -- same as BLE TX ===
    let bt_tx_addr = addr;
    for i in 0..79 {
        write_word(
            &mut addr,
            pack_vco_tx(vco_cal.capcode_tx[i], vco_cal.idac_tx[i], vco_cal.kcal[i]),
        );
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

        let entry = pack_txdc(
            pt.coef0,
            pt.coef1,
            pt.offset_i,
            pt.offset_q,
            tmxbuf_gc[level],
            edr_pa_bm[level],
        );
        write_word(&mut addr, entry.word1);
        write_word(&mut addr, entry.word2);
    }
    // Set CAL_ADDR_REG3
    BT_RFC.cal_addr_reg3().write(|w| {
        w.set_txdc_cal_addr(txdc_addr as u16);
    });

    // Replace EDR cal related commands in BT_TXON with WAIT commands
    // SDK: bt_rfc_txdc_cal lines 5001-5009

    /// Number of command words to overwrite in BT_TXON EDR calibration section.
    const BT_TXON_EDR_CAL_CMD_WORDS: u32 = 10;
    /// Byte offset of the EDR calibration commands within the BT_TXON sequence.
    const BT_TXON_EDR_CAL_OFFSET: u32 = 32 * 4;

    let wait1 = rwbt::rfc::cmd::wait(1) as u32;
    let wait1_pair = wait1 | (wait1 << 16); // Two WAIT(1) commands packed per word
    let bt_txon_addr = BT_RFC.cu_addr_reg3().read().bt_txon_cfg_addr() as u32;
    for i in 0..BT_TXON_EDR_CAL_CMD_WORDS {
        unsafe {
            core::ptr::write_volatile(
                (base + bt_txon_addr + BT_TXON_EDR_CAL_OFFSET + i * 4) as *mut u32,
                wait1_pair,
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
#[cfg(feature = "edr")]
pub fn store_edr_lo_cal_tables(edr_lo: &EdrLoCalResult) {
    let base = super::BT_RFC_MEM_BASE;
    let bt_tx_addr = BT_RFC.cal_addr_reg2().read().bt_tx_cal_addr() as u32;

    for i in 0..79usize {
        let word = pack_edr_cal(
            edr_lo.capcode[i],
            edr_lo.idac[i],
            edr_lo.oslo_fc[i],
            edr_lo.oslo_bm[i],
            super::edr_lo::TMXCAP_DEFAULT as u8,
            DPSK_GAIN_INITIAL[i],
        );
        unsafe {
            core::ptr::write_volatile((base + bt_tx_addr + (i as u32) * 4) as *mut u32, word);
        }
    }
}
