//! TX DC offset calibration for Bluetooth RF.
//!
//! This module implements TX DC offset calibration for the Bluetooth RF subsystem.
//! Based on SDK `bt_rfc_txdc_cal()` in `bt_rf_fulcal.c`.
//!
//! The TXDC calibration corrects for DC offset in the IQ modulator transmit path,
//! which is critical for EDR (Enhanced Data Rate) transmission quality.
//!
//! ## Calibration Parameters
//!
//! For each power level (0-6), the calibration determines:
//! - `offset_i`: DC offset correction for I channel (11-bit)
//! - `offset_q`: DC offset correction for Q channel (11-bit)
//! - `coef0`: Calibration coefficient 0 (14-bit)
//! - `coef1`: Calibration coefficient 1 (14-bit)
//!
//! ## Implementation
//!
//! `txdc_cal_full()` performs full DMA-based calibration with iterative search.
//! It uses a DMAC2 channel to capture ADC samples from the PHY RX dump,
//! then calculates mixer power using IQ demodulation to find optimal DC offset values.

use crate::dma::{self, Channel, Increment, Transfer, TransferOptions};
use crate::pac::{BT_PHY, BT_RFC, PMUC};
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

/// Number of power levels for TXDC calibration
pub const NUM_POWER_LEVELS: usize = 7;

/// Number of ADC samples per DMA transfer
const DMA_SAMPLE_COUNT: usize = 512;

/// PHY RX dump source address (CPU cannot read this directly, DMA only)
const PHY_RX_DUMP_ADDR: u32 = super::super::memory_map::rf::PHY_RX_DUMP_ADDR;

/// DMA buffer address in LPSYS EM memory (SDK: BT_RFC_TXDC_DMA_ADDR).
const DMA_BUFFER_ADDR: u32 = super::super::memory_map::shared::EM_START as u32;

/// Cosine table for mixer demodulation (16-point period, Q10 format)
const COS_TABLE: [i16; 16] = [
    1024, 946, 724, 392, 0, -392, -724, -946, -1024, -946, -724, -392, 0, 392, 724, 946,
];

/// Sine table for mixer demodulation (16-point period, Q10 format)
const SIN_TABLE: [i16; 16] = [
    0, -392, -724, -946, -1024, -946, -724, -392, 0, 392, 724, 946, 1024, 946, 724, 392,
];

/// TXDC calibration result for a single power level.
#[derive(Clone, Copy, Debug, Default)]
pub struct TxdcCalPoint {
    /// DC offset correction for I channel (0-2047)
    pub offset_i: u16,
    /// DC offset correction for Q channel (0-2047)
    pub offset_q: u16,
    /// Calibration coefficient 0 (0-16383)
    pub coef0: u16,
    /// Calibration coefficient 1 (0-16383)
    pub coef1: u16,
}

/// TXDC calibration results for all power levels.
#[derive(Clone, Copy, Debug)]
pub struct TxdcCalResult {
    /// Calibration points for each power level (0-6)
    pub points: [TxdcCalPoint; NUM_POWER_LEVELS],
}

impl Default for TxdcCalResult {
    fn default() -> Self {
        Self {
            points: [TxdcCalPoint::default(); NUM_POWER_LEVELS],
        }
    }
}

/// TXDC calibration configuration.
#[derive(Clone, Copy, Debug)]
pub struct TxdcCalConfig {
    /// Enable calibration for each power level (bit mask)
    pub power_level_mask: u8,
    /// TMXBUF gain control values for each power level
    pub tmxbuf_gc: [u8; 8],
    /// EDR PA BM values for each power level
    pub edr_pa_bm: [u8; 8],
}

impl Default for TxdcCalConfig {
    fn default() -> Self {
        Self {
            // Enable all 7 power levels
            power_level_mask: 0x7F,
            // Default TMXBUF gain control values from SDK
            tmxbuf_gc: [2, 3, 3, 5, 5, 6, 8, 0xF],
            // Default EDR PA BM values from SDK
            edr_pa_bm: [5, 5, 0xE, 0xA, 0x1B, 0x1F, 0x1F, 0x1F],
        }
    }
}

/// Default TXDC calibration values from SDK.
///
/// These values are used as fallback when full calibration is not performed.
pub fn default_txdc_cal() -> TxdcCalResult {
    // Default values from SDK: offset centered at 0x7E0, coef0=0x3000, coef1=0x1000
    let default_point = TxdcCalPoint {
        offset_i: 0x7E0,
        offset_q: 0x7E0,
        coef0: 0x3000,
        coef1: 0x1000,
    };

    TxdcCalResult {
        points: [default_point; NUM_POWER_LEVELS],
    }
}

/// Apply TXDC calibration result for a specific power level.
///
/// Writes the calibration values to IQ_PWR_REG1 and IQ_PWR_REG2.
pub fn apply_txdc_cal_point(point: &TxdcCalPoint) {
    BT_RFC.iq_pwr_reg1().modify(|w| {
        w.set_tx_dc_cal_coef0(point.coef0 & 0x3FFF);
        w.set_tx_dc_cal_coef1(point.coef1 & 0x3FFF);
    });

    BT_RFC.iq_pwr_reg2().modify(|w| {
        w.set_tx_dc_cal_offset_i(point.offset_i & 0x7FF);
        w.set_tx_dc_cal_offset_q(point.offset_q & 0x7FF);
    });
}

// ============================================================================
// Full TXDC Calibration with DMA-based Power Measurement
// ============================================================================

/// Blocking DMA options: TCIE enabled for TCIF polling (no NVIC interrupt needed).
const DMA_OPTS: TransferOptions = TransferOptions {
    priority: dma::Priority::Low,
    circular: false,
    half_transfer_ir: false,
    complete_transfer_ir: true,
    interrupt_priority: crate::interrupt::Priority::P0,
};

/// Read a sample from the DMA buffer at the given index.
#[inline]
fn read_dma_sample(index: usize) -> u32 {
    unsafe { core::ptr::read_volatile((DMA_BUFFER_ADDR as *const u32).add(index)) }
}

/// Capture ADC samples via DMA: PHY RX dump (fixed) → EM buffer (incrementing).
#[inline]
fn capture_adc_samples(ch: &mut PeripheralRef<'_, dma::AnyChannel>) {
    unsafe {
        Transfer::new_transfer_raw::<u32>(
            ch.reborrow(),
            PHY_RX_DUMP_ADDR as *const u32,
            DMA_BUFFER_ADDR as *mut u32,
            DMA_SAMPLE_COUNT,
            Increment::Memory,
            DMA_OPTS,
        )
        .blocking_wait();
    }
}

/// Calculate mixer power from captured ADC samples.
fn calculate_mixer_power() -> i64 {
    let mut mixer_i_sum: i64 = 0;
    let mut mixer_q_sum: i64 = 0;
    let mut phase_index: usize = 0;

    // Process all samples
    for k in 0..DMA_SAMPLE_COUNT {
        // Read ADC sample (10-bit unsigned)
        let mem_data = read_dma_sample(k);
        let adc_data = (mem_data & 0x3FF) as i32;

        // Get mixer coefficients
        let mixer_cos = COS_TABLE[phase_index] as i32;
        let mixer_sin = SIN_TABLE[phase_index] as i32;

        // Advance phase (16-point period)
        phase_index = (phase_index + 1) & 0x0F;

        // Accumulate I/Q components
        mixer_i_sum += (adc_data * mixer_cos) as i64;
        mixer_q_sum += (adc_data * mixer_sin) as i64;
    }

    // Average
    mixer_i_sum /= DMA_SAMPLE_COUNT as i64;
    mixer_q_sum /= DMA_SAMPLE_COUNT as i64;

    // Return power (I^2 + Q^2)
    mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum
}

/// Search for optimal offset value using DMA-based power measurement.
fn search_optimal_offset<F>(set_offset: F, ch: &mut PeripheralRef<'_, dma::AnyChannel>) -> u16
where
    F: Fn(u16),
{
    let mut dc_out_min: i64 = i64::MAX;
    let mut best_offset: u16 = 0x7E0;

    for j in 0..64u16 {
        let data = (0x7E0 + j) & 0x7FF;
        set_offset(data);

        capture_adc_samples(ch);
        let mixer_pwr = calculate_mixer_power();

        if dc_out_min > mixer_pwr {
            dc_out_min = mixer_pwr;
            best_offset = data;
        }
    }

    best_offset
}

/// Perform full TXDC calibration with DMA-based power measurement.
///
/// `cal_enable` is a bitmask from [`super::bt_rf_cal_index`] indicating which
/// power levels (0-6) need calibration. Disabled levels get default values.
///
/// Requires a DMAC2 channel for PHY RX dump capture.
pub fn txdc_cal_full(
    edr_pa_bm: Option<[u8; 8]>,
    cal_enable: u8,
    dma_ch: impl Peripheral<P = impl Channel>,
) -> TxdcCalResult {
    into_ref!(dma_ch);
    let mut dma_ch: PeripheralRef<'_, dma::AnyChannel> = dma_ch.map_into();

    let mut config = TxdcCalConfig::default();
    config.power_level_mask = cal_enable;
    let mut result = TxdcCalResult::default();

    // Apply eFUSE-based EDR PA BM adjustment if available
    if let Some(pa_bm) = edr_pa_bm {
        config.edr_pa_bm = pa_bm;
    }

    // Configure hardware for TXDC calibration
    super::txdc_hw::configure_for_txdc_cal();

    // Set LPSYS clock to 24MHz for DMAC2 (SDK line 3963-3965):
    // SEL_SYS = HXT48 (48MHz), HDIV1 = 2 -> HCLK = 24MHz
    use crate::rcc::lpsys_vals::Sysclk;
    crate::rcc::select_lpsys_sysclk(Sysclk::Hxt48);
    crate::rcc::set_lpsys_hdiv(2);

    // Calibrate each power level
    for level in 0..NUM_POWER_LEVELS {
        if config.power_level_mask & (1 << level) == 0 {
            // Use default values for disabled levels
            result.points[level] = TxdcCalPoint {
                offset_i: 0x7E0,
                offset_q: 0x7E0,
                coef0: 0x3000,
                coef1: 0x1000,
            };
            continue;
        }

        // Configure power level
        super::txdc_hw::configure_power_level(level, &config);

        // Set rx mixer phase to 750KHz for DC offset calibration
        BT_PHY.mixer_cfg1().modify(|w| {
            w.set_rx_mixer_phase_1(0x40);
        });

        // Configure BUCK for calibration
        PMUC.buck_cr1().modify(|w| {
            w.set_cot_ctune(7);
        });

        // Fix coef1 for offset search
        BT_RFC.iq_pwr_reg1().modify(|w| {
            w.set_tx_dc_cal_coef1(0x1000);
            w.set_tx_dc_cal_coef0(0);
        });
        BT_RFC.iq_pwr_reg2().modify(|w| {
            w.set_tx_dc_cal_offset_i(0);
            w.set_tx_dc_cal_offset_q(0);
        });

        // First round: search offset_i
        let offset_i = search_optimal_offset(
            |val| {
                BT_RFC.iq_pwr_reg2().modify(|w| {
                    w.set_tx_dc_cal_offset_i(val);
                });
            },
            &mut dma_ch,
        );

        // Fix offset_i, search offset_q
        BT_RFC.iq_pwr_reg2().modify(|w| {
            w.set_tx_dc_cal_offset_i(offset_i);
        });

        let offset_q = search_optimal_offset(
            |val| {
                BT_RFC.iq_pwr_reg2().modify(|w| {
                    w.set_tx_dc_cal_offset_q(val);
                });
            },
            &mut dma_ch,
        );

        // Fix offset_q
        BT_RFC.iq_pwr_reg2().modify(|w| {
            w.set_tx_dc_cal_offset_q(offset_q);
        });

        // Second round: refine offset_i
        let offset_i = search_optimal_offset(
            |val| {
                BT_RFC.iq_pwr_reg2().modify(|w| {
                    w.set_tx_dc_cal_offset_i(val);
                });
            },
            &mut dma_ch,
        );

        BT_RFC.iq_pwr_reg2().modify(|w| {
            w.set_tx_dc_cal_offset_i(offset_i);
        });

        // Second round: refine offset_q
        let offset_q = search_optimal_offset(
            |val| {
                BT_RFC.iq_pwr_reg2().modify(|w| {
                    w.set_tx_dc_cal_offset_q(val);
                });
            },
            &mut dma_ch,
        );

        BT_RFC.iq_pwr_reg2().modify(|w| {
            w.set_tx_dc_cal_offset_q(offset_q);
        });

        // Restore BUCK setting
        PMUC.buck_cr1().modify(|w| {
            w.set_cot_ctune(4);
        });

        // Set rx mixer phase to 1.5MHz for coef calibration
        BT_PHY.mixer_cfg1().modify(|w| {
            w.set_rx_mixer_phase_1(0x80);
        });

        // Store calibration result
        result.points[level] = TxdcCalPoint {
            offset_i,
            offset_q,
            coef0: 0x3000, // Fixed value
            coef1: 0x1000, // Fixed value
        };
    }

    // Cleanup
    super::txdc_hw::cleanup_txdc_cal();

    result
}
