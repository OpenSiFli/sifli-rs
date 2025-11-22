//! LCPU-side Bluetooth RF calibration helper.
//!
//! This implements the minimum necessary subset of SDK `bt_rf_cal`, used for:
//! - Resetting Bluetooth RF module via LPSYS_RCC
//! - Writing BT transmit power parameters to LCPU ROM configuration area
//!
//! The complete analog RF self-calibration algorithms (VCO / TXDC / IQ etc., see
//! `SiFli-SDK/drivers/cmsis/sf32lb52x/bt_rf_fulcal.c`) are not yet ported.

use super::ram;
use crate::lpsys_rcc::{get_rfc_reset, set_rfc_reset};
use crate::syscfg::ChipRevision;

/// Reset Bluetooth RF module.
///
/// Corresponds to `HAL_RCC_ResetBluetoothRF` in SDK.
fn reset_bluetooth_rf() {
    // Set RFC reset bit
    set_rfc_reset(true);
    // Wait for bit to take effect
    while !get_rfc_reset() {}
    // Clear RFC reset bit
    set_rfc_reset(false);
}

/// Default BT RF power parameters.
///
/// Weak default implementation corresponding to `bt_rf_get_max_tx_pwr` / `bt_rf_get_min_tx_pwr` /
/// `bt_rf_get_init_tx_pwr` and `bt_is_in_BQB_mode` in SDK.
fn default_tx_power_params() -> (i8, i8, i8, u8) {
    let max_pwr: i8 = 10;
    let init_pwr: i8 = 0;
    let min_pwr: i8 = 0;
    let is_bqb: u8 = 0;
    (max_pwr, min_pwr, init_pwr, is_bqb)
}

/// Encode power parameters into 32-bit packed format.
///
/// Equivalent to `RF_PWR_PARA` macro in SDK:
/// `(is_bqb << 24) | (init << 16) | (min << 8) | (int8_t)(max)`.
fn encode_tx_power(max: i8, min: i8, init: i8, is_bqb: u8) -> u32 {
    let max_u = max as u8 as u32;
    let min_u = min as u8 as u32;
    let init_u = init as u8 as u32;
    let is_bqb_u = is_bqb as u32;

    (is_bqb_u << 24) | (init_u << 16) | (min_u << 8) | max_u
}

/// Perform basic Bluetooth RF "calibration" steps.
///
/// Current implementation:
/// - Reset Bluetooth RF module;
/// - Calculate BT TX power encoding based on default parameters;
/// - Write to `bt_txpwr` field in LCPU ROM configuration area.
///
/// Note: This does not perform the complete analog RF self-calibration flow in SDK, only guarantees
/// configuration fields required for LCPU startup are correctly set.
pub fn bt_rf_cal(revision: ChipRevision) {
    // 1. Reset Bluetooth RF
    reset_bluetooth_rf();

    // 2. Calculate TX power encoding
    let (max_pwr, min_pwr, init_pwr, is_bqb) = default_tx_power_params();
    let tx_pwr = encode_tx_power(max_pwr, min_pwr, init_pwr, is_bqb);

    // 3. Write to LCPU ROM configuration area (BT_TXPWR)
    ram::set_bt_tx_power(revision, tx_pwr);
}
