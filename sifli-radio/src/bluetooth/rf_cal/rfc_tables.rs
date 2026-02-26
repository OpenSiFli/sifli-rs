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
use super::rfc_sram::{RfcSram, RfcTable};
use sifli_hal::ram::RamSlice;

/// All calibration tables allocated in RFC SRAM.
pub struct CalTables {
    /// BLE RX calibration table (40 words, 1M+2M packed per word).
    pub ble_rx: RfcTable,
    /// BT RX calibration table (40 words, 79 channels packed as pairs).
    pub bt_rx: RfcTable,
    /// BLE TX calibration table (79 words, one per channel).
    pub ble_tx: RfcTable,
    /// BT TX calibration table (79 words). Overwritten by EDR LO results when `edr` enabled.
    pub bt_tx: RfcTable,
    /// TXDC calibration table (16 words = 8 power levels × 2 words).
    pub txdc: RfcTable,
}

/// Allocate all calibration tables sequentially from `cmd_end_addr`.
///
/// Must be called after `generate_rfc_cmd_sequences()` which determines `cmd_end_addr`.
/// Tables are allocated in fixed order: BLE RX → BT RX → BLE TX → BT TX → TXDC.
///
/// # Panics
///
/// Panics if the tables would exceed RFC SRAM bounds.
pub fn alloc_cal_tables(sram: &RfcSram, cmd_end_addr: u32) -> CalTables {
    let (ble_rx, next) = sram.alloc_table(cmd_end_addr, 40);
    let (bt_rx, next) = sram.alloc_table(next, 40);
    let (ble_tx, next) = sram.alloc_table(next, 79);
    let (bt_tx, next) = sram.alloc_table(next, 79);
    let (txdc, _) = sram.alloc_table(next, 16);
    CalTables {
        ble_rx,
        bt_rx,
        ble_tx,
        bt_tx,
        txdc,
    }
}

/// Write VCO calibration tables (RX + TX) to RFC SRAM and update CAL_ADDR_REG1/REG2.
///
/// Must be called after VCO calibration and BEFORE TXDC calibration,
/// because TXDC's force_tx needs CAL_ADDR to look up the correct VCO
/// parameters for the forced channel.
pub fn store_vco_cal_tables(tables: &CalTables, vco_cal: &VcoCalResult) {
    // === BLE RX calibration table (40 words) ===
    for i in 0..40 {
        let rx_1m = pack_vco_rx_half(vco_cal.capcode_rx_1m[i], vco_cal.idac_rx_1m[i]);
        let rx_2m = pack_vco_rx_half(vco_cal.capcode_rx_2m[i], vco_cal.idac_rx_2m[i]);
        tables.ble_rx.write(i, pack_vco_rx_word(rx_1m, rx_2m));
    }
    // === BT RX calibration table (40 words, packing 79 channels as pairs) ===
    for i in 0..40 {
        let ch0 = 2 * i;
        let ch1 = 2 * i + 1;
        let lo = pack_vco_rx_half(vco_cal.capcode_rx_bt[ch0], vco_cal.idac_rx_bt[ch0]);
        let hi = if ch1 < 79 {
            pack_vco_rx_half(vco_cal.capcode_rx_bt[ch1], vco_cal.idac_rx_bt[ch1])
        } else {
            lo // last odd channel: duplicate
        };
        tables.bt_rx.write(i, pack_vco_rx_word(lo, hi));
    }
    // Set CAL_ADDR_REG1
    BT_RFC.cal_addr_reg1().write(|w| {
        w.set_ble_rx_cal_addr(tables.ble_rx.offset());
        w.set_bt_rx_cal_addr(tables.bt_rx.offset());
    });

    // === BLE TX calibration table (79 words) ===
    for i in 0..79 {
        tables.ble_tx.write(
            i,
            pack_vco_tx(vco_cal.capcode_tx[i], vco_cal.idac_tx[i], vco_cal.kcal[i]),
        );
    }
    // === BT TX calibration table (79 words) -- same as BLE TX ===
    for i in 0..79 {
        tables.bt_tx.write(
            i,
            pack_vco_tx(vco_cal.capcode_tx[i], vco_cal.idac_tx[i], vco_cal.kcal[i]),
        );
    }
    // Set CAL_ADDR_REG2
    BT_RFC.cal_addr_reg2().write(|w| {
        w.set_ble_tx_cal_addr(tables.ble_tx.offset());
        w.set_bt_tx_cal_addr(tables.bt_tx.offset());
    });
}

/// Write TXDC calibration tables to RFC SRAM and update CAL_ADDR_REG3.
///
/// Called after TXDC calibration with the calibration results.
/// `sram_region` is the full RFC SRAM `RamSlice` (from `RfcSram::region()`),
/// used to overwrite BT_TXON EDR calibration commands.
pub fn store_txdc_cal_tables(
    sram_region: &RamSlice,
    tables: &CalTables,
    txdc_cal: &TxdcCalResult,
    edr_pa_bm: &[u8; 8],
    tmxbuf_gc: &[u8; 8],
) {
    // === TXDC calibration table (8 power levels x 2 words = 16 words) ===
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
        tables.txdc.write_pair(level, entry.word1, entry.word2);
    }
    // Set CAL_ADDR_REG3
    BT_RFC.cal_addr_reg3().write(|w| {
        w.set_txdc_cal_addr(tables.txdc.offset());
    });

    // Replace EDR cal related commands in BT_TXON with WAIT commands
    // SDK: bt_rfc_txdc_cal lines 5001-5009

    /// Number of command words to overwrite in BT_TXON EDR calibration section.
    const BT_TXON_EDR_CAL_CMD_WORDS: usize = 10;
    /// Byte offset of the EDR calibration commands within the BT_TXON sequence.
    const BT_TXON_EDR_CAL_OFFSET: usize = 32 * 4;

    let wait1 = rwbt::rfc::cmd::wait(1) as u32;
    let wait1_pair = wait1 | (wait1 << 16); // Two WAIT(1) commands packed per word
    let bt_txon_addr = BT_RFC.cu_addr_reg3().read().bt_txon_cfg_addr() as usize;
    let overwrite_region = sram_region.slice(
        bt_txon_addr + BT_TXON_EDR_CAL_OFFSET,
        BT_TXON_EDR_CAL_CMD_WORDS * 4,
    );
    for i in 0..BT_TXON_EDR_CAL_CMD_WORDS {
        overwrite_region.write::<u32>(i * 4, wait1_pair);
    }

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
pub fn store_edr_lo_cal_tables(tables: &CalTables, edr_lo: &EdrLoCalResult) {
    for i in 0..79usize {
        let word = pack_edr_cal(
            edr_lo.capcode[i],
            edr_lo.idac[i],
            edr_lo.oslo_fc[i],
            edr_lo.oslo_bm[i],
            super::edr_lo::TMXCAP_DEFAULT as u8,
            DPSK_GAIN_INITIAL[i],
        );
        tables.bt_tx.write(i, word);
    }
}
