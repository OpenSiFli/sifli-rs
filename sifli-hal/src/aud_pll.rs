//! Audio PLL driver for SF32LB52x
//!
//! Manages the AUDCODEC audio PLL: HXT buffer, bandgap, analog PLL, VCO
//! calibration, and SDM frequency configuration.
//!
//! The [`AudioPll`] must be created before constructing [`AudioDac`](crate::audio::AudioDac)
//! or [`AudioAdc`](crate::audio::AudioAdc), and passed by reference to ensure the PLL
//! outlives the audio driver (compile-time lifetime guarantee).
//!
//! # Example
//!
//! ```ignore
//! use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
//!
//! let pll = AudioPll::new(AudPllFreq::Mhz49_152);
//! let dac = audio::AudioDac::new_blocking(p.AUDPRC, p.DMAC1_CH1, &pll, config);
//! ```

use core::sync::atomic::{AtomicBool, Ordering};

use crate::pac;
use crate::rcc;
use crate::time::Hertz;

/// Audio PLL output frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AudPllFreq {
    /// 49.152 MHz — for 48 kHz family (8k, 16k, 32k, 48k, 96k, 192k).
    Mhz49_152,
    /// 45.1584 MHz — for 44.1 kHz family (11.025k, 22.05k, 44.1k, 88.2k, 176.4k).
    Mhz45_1584,
}

impl AudPllFreq {
    /// PLL SDM FCW value.
    ///
    /// Formula: `freq = (FCW + 3 + SDIN / 2^20) * 6 MHz`
    pub(crate) fn fcw(&self) -> u8 {
        match self {
            // (5 + 3 + 201327/1048576) * 6 = 49.152 MHz
            Self::Mhz49_152 => 5,
            // (4 + 3 + 572662/1048576) * 6 = 45.1584 MHz
            Self::Mhz45_1584 => 4,
        }
    }

    /// PLL SDM SDIN value.
    pub(crate) fn sdin(&self) -> u32 {
        match self {
            Self::Mhz49_152 => 201327,
            Self::Mhz45_1584 => 572662,
        }
    }

    /// PLL output frequency in Hz.
    pub fn freq(&self) -> u32 {
        match self {
            Self::Mhz49_152 => 49_152_000,
            Self::Mhz45_1584 => 45_158_400,
        }
    }
}

/// Audio sample rate.
///
/// Each sample rate belongs to either the 48 kHz family (PLL = 49.152 MHz)
/// or the 44.1 kHz family (PLL = 45.1584 MHz). The PLL frequency must be
/// configured accordingly via [`AudioPll`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleRate {
    Hz8000,
    Hz11025,
    Hz16000,
    Hz22050,
    Hz32000,
    Hz44100,
    Hz48000,
    Hz88200,
    Hz96000,
    Hz176400,
    Hz192000,
}

impl SampleRate {
    /// Required PLL frequency for this sample rate.
    pub fn pll_freq(&self) -> AudPllFreq {
        match self {
            Self::Hz8000 | Self::Hz16000 | Self::Hz32000 | Self::Hz48000 | Self::Hz96000 | Self::Hz192000 => {
                AudPllFreq::Mhz49_152
            }
            Self::Hz11025 | Self::Hz22050 | Self::Hz44100 | Self::Hz88200 | Self::Hz176400 => {
                AudPllFreq::Mhz45_1584
            }
        }
    }

    /// AUDPRC STB register `dac_div` value.
    ///
    /// PLL_freq / dac_div = sample_rate.
    pub(crate) fn dac_div(&self) -> u16 {
        match self {
            Self::Hz8000 => 6144,    // 49152000 / 8000
            Self::Hz11025 => 4096,   // 45158400 / 11025
            Self::Hz16000 => 3072,   // 49152000 / 16000
            Self::Hz22050 => 2048,   // 45158400 / 22050
            Self::Hz32000 => 1536,   // 49152000 / 32000
            Self::Hz44100 => 1024,   // 45158400 / 44100
            Self::Hz48000 => 1024,   // 49152000 / 48000
            Self::Hz88200 => 512,    // 45158400 / 88200
            Self::Hz96000 => 512,    // 49152000 / 96000
            Self::Hz176400 => 256,   // 45158400 / 176400
            Self::Hz192000 => 256,   // 49152000 / 192000
        }
    }

    /// AUDPRC STB register `adc_div` value.
    ///
    /// Same divisor table as DAC.
    pub(crate) fn adc_div(&self) -> u16 {
        self.dac_div()
    }

    /// STB clock select: always PLL (true) since AudioPll configures the correct frequency.
    pub(crate) fn stb_clk_sel(&self) -> bool {
        true
    }

    /// Sample rate in Hz.
    pub fn hz(&self) -> u32 {
        match self {
            Self::Hz8000 => 8000,
            Self::Hz11025 => 11025,
            Self::Hz16000 => 16000,
            Self::Hz22050 => 22050,
            Self::Hz32000 => 32000,
            Self::Hz44100 => 44100,
            Self::Hz48000 => 48000,
            Self::Hz88200 => 88200,
            Self::Hz96000 => 96000,
            Self::Hz176400 => 176400,
            Self::Hz192000 => 192000,
        }
    }
}

/// Singleton flag to prevent multiple AudioPll instances.
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Audio PLL driver.
///
/// Manages the AUDCODEC PLL, bandgap, and reference generator.
/// Only one instance can exist at a time (enforced by a runtime check).
///
/// The PLL is initialized in [`new()`](Self::new) and shut down on [`Drop`].
pub struct AudioPll {
    freq: AudPllFreq,
}

fn delay_us(us: u32) {
    crate::cortex_m_blocking_delay_us(us);
}

fn delay_ms(ms: u32) {
    crate::cortex_m_blocking_delay_us(ms * 1000);
}

impl AudioPll {
    /// Create and initialize the Audio PLL at the given frequency.
    ///
    /// Performs the full initialization sequence:
    /// 1. Enable AUDCODEC clock and HXT audio buffer
    /// 2. Bandgap + reference generator power-up (with ADC-required config)
    /// 3. PLL analog enable + VCO calibration
    /// 4. SDM frequency programming
    /// 5. PLL lock check
    /// 6. Update RCC clocks cache
    ///
    /// # Panics
    ///
    /// Panics if an `AudioPll` instance already exists.
    pub fn new(freq: AudPllFreq) -> Self {
        if TAKEN.swap(true, Ordering::SeqCst) {
            panic!("AudioPll: already taken");
        }

        // Enable AUDCODEC clock
        rcc::enable::<crate::peripherals::AUDCODEC>();

        let codec = pac::AUDCODEC;

        // HXT audio buffer enable
        pac::PMUC.hxt_cr1().modify(|w| w.set_buf_aud_en(true));

        // ===== Bandgap + Refgen =====
        codec.bg_cfg1().write(|w| w.0 = 48000);
        codec.bg_cfg2().write(|w| w.0 = 48_000_000);
        codec.bg_cfg0().modify(|w| {
            w.set_en(true);
            w.set_en_rcflt(true); // RC filter (needed by ADC)
            w.set_vref_sel(4); // 3.3V AVDD (needed by ADC)
            w.set_mic_vref_sel(4); // mic reference (needed by ADC)
        });
        delay_us(100);
        codec.bg_cfg0().modify(|w| w.set_en_smpl(false));
        codec.refgen_cfg().modify(|w| {
            w.set_en_chop(false);
            w.set_en(true);
            w.set_lp_mode(false);
        });
        delay_ms(2);
        codec.bg_cfg0().modify(|w| w.set_en_smpl(true));

        // ===== PLL analog enable =====
        codec.pll_cfg0().modify(|w| w.set_en_iary(true));
        codec.pll_cfg0().modify(|w| w.set_en_vco(true));
        codec.pll_cfg0().modify(|w| w.set_en_ana(true));
        codec.pll_cfg0().modify(|w| w.set_icp_sel(8));
        codec.pll_cfg2().modify(|w| w.set_en_dig(true));
        codec.pll_cfg3().modify(|w| w.set_en_sdm(true));
        codec.pll_cfg4().modify(|w| w.set_en_clk_dig(true));

        // Loop filter configuration
        codec.pll_cfg1().modify(|w| {
            w.set_r3_sel(3);
            w.set_rz_sel(1);
            w.set_c2_sel(3);
            w.set_cz_sel(6);
            w.set_csd_rst(false);
            w.set_csd_en(false);
        });
        delay_us(50);

        // ===== VCO calibration (binary search) =====
        let target_cnt: u32 = 1838;
        codec.pll_cfg0().modify(|w| w.set_open(true));
        codec.pll_cfg2().modify(|w| w.set_en_lf_vcin(true));
        codec.pll_cal_cfg().write(|w| {
            w.set_en(false);
            w.set_len(2000);
        });

        let mut fc_vco: u32 = 16;
        let mut delta: u32 = 8;
        while delta != 0 {
            codec.pll_cfg0().modify(|w| w.set_fc_vco(0));
            codec
                .pll_cfg0()
                .modify(|w| w.set_fc_vco(fc_vco as u8));
            codec.pll_cal_cfg().modify(|w| w.set_en(true));
            let mut cal_timeout = 100_000u32;
            while !codec.pll_cal_cfg().read().done() && cal_timeout > 0 {
                delay_us(1);
                cal_timeout -= 1;
            }
            let cnt = codec.pll_cal_result().read().pll_cnt() as u32;
            codec.pll_cal_cfg().modify(|w| w.set_en(false));
            if cnt < target_cnt {
                fc_vco = fc_vco.saturating_add(delta);
            } else if cnt > target_cnt {
                fc_vco = fc_vco.saturating_sub(delta);
            }
            delta >>= 1;
        }

        // Neighbor refinement
        let measure_vco = |fc: u32| -> u32 {
            codec.pll_cfg0().modify(|w| w.set_fc_vco(fc as u8));
            codec.pll_cal_cfg().modify(|w| w.set_en(true));
            let mut timeout = 100_000u32;
            while !codec.pll_cal_cfg().read().done() && timeout > 0 {
                delay_us(1);
                timeout -= 1;
            }
            let cnt = codec.pll_cal_result().read().pll_cnt() as u32;
            codec.pll_cal_cfg().modify(|w| w.set_en(false));
            cnt
        };
        let best_cnt = measure_vco(fc_vco);
        let fc_min = fc_vco.saturating_sub(1);
        let fc_max = if fc_vco < 31 { fc_vco + 1 } else { fc_vco };
        let cnt_min = measure_vco(fc_min);
        let cnt_max = measure_vco(fc_max);
        let delta_mid = (best_cnt as i32 - target_cnt as i32).unsigned_abs();
        let delta_lo = (cnt_min as i32 - target_cnt as i32).unsigned_abs();
        let delta_hi = (cnt_max as i32 - target_cnt as i32).unsigned_abs();
        let best_fc = if delta_lo <= delta_mid && delta_lo <= delta_hi {
            fc_min
        } else if delta_hi <= delta_mid && delta_hi <= delta_lo {
            fc_max
        } else {
            fc_vco
        };
        codec.pll_cfg0().modify(|w| w.set_fc_vco(best_fc as u8));
        codec.pll_cfg2().modify(|w| w.set_en_lf_vcin(false));
        codec.pll_cfg0().modify(|w| w.set_open(false));
        delay_us(50);

        // ===== SDM frequency programming =====
        codec.pll_cfg2().modify(|w| w.set_rstb(true));
        delay_us(50);
        codec.pll_cfg3().write(|w| {
            w.set_sdin(freq.sdin());
            w.set_fcw(freq.fcw());
            w.set_en_sdm(true);
            w.set_sdmin_bypass(true);
        });
        codec.pll_cfg3().modify(|w| w.set_sdm_update(true));
        codec
            .pll_cfg3()
            .modify(|w| w.set_sdmin_bypass(false));
        codec.pll_cfg2().modify(|w| w.set_rstb(false));
        delay_us(50);
        codec.pll_cfg2().modify(|w| w.set_rstb(true));
        delay_us(50);

        // ===== PLL lock check =====
        codec.pll_cfg1().modify(|w| {
            w.set_csd_en(true);
            w.set_csd_rst(true);
        });
        delay_us(50);
        codec.pll_cfg1().modify(|w| w.set_csd_rst(false));
        delay_us(100);
        let locked = !codec.pll_stat().read().unlock();
        if locked {
            codec.pll_cfg1().modify(|w| w.set_csd_en(false));
        }

        // PLL_CFG5: chopping clocks for bandgap and refgen
        codec.pll_cfg5().write(|w| {
            w.set_divb_clk_chop_bg(2);
            w.set_diva_clk_chop_bg(20);
            w.set_en_clk_chop_bg(true);
            w.set_divb_clk_chop_refgen(2);
            w.set_diva_clk_chop_refgen(20);
            w.set_en_clk_chop_refgen(true);
            w.set_divb_clk_chop_dac2(2);
            w.set_diva_clk_chop_dac2(4);
            w.set_en_clk_chop_dac2(true);
            w.set_diva_clk_dac2(5);
            w.set_en_clk_dac2(true);
        });

        // Update clocks cache
        Self::update_clocks_cache(Some(freq));

        Self { freq }
    }

    /// Get the configured PLL frequency.
    pub fn freq(&self) -> AudPllFreq {
        self.freq
    }

    /// Assert that a sample rate is compatible with this PLL frequency.
    ///
    /// # Panics
    ///
    /// Panics if the sample rate requires a different PLL frequency.
    pub fn assert_compatible(&self, sample_rate: SampleRate) {
        assert_eq!(
            sample_rate.pll_freq(),
            self.freq,
            "SampleRate {:?} requires PLL {:?}, but AudioPll is configured for {:?}",
            sample_rate,
            sample_rate.pll_freq(),
            self.freq,
        );
    }

    fn update_clocks_cache(freq: Option<AudPllFreq>) {
        unsafe {
            let mut clocks = *rcc::get_freqs();
            match freq {
                Some(f) => {
                    let hz = f.freq();
                    clocks.clk_aud_pll = Some(Hertz(hz)).into();
                    clocks.clk_aud_pll_div16 = Some(Hertz(hz / 16)).into();
                }
                None => {
                    clocks.clk_aud_pll = None.into();
                    clocks.clk_aud_pll_div16 = None.into();
                }
            }
            rcc::set_freqs(clocks);
        }
    }
}

impl Drop for AudioPll {
    fn drop(&mut self) {
        let codec = pac::AUDCODEC;

        // Disable PLL
        codec.pll_cfg4().modify(|w| w.set_en_clk_dig(false));
        codec.pll_cfg3().modify(|w| w.set_en_sdm(false));
        codec.pll_cfg2().modify(|w| w.set_en_dig(false));
        codec.pll_cfg0().modify(|w| {
            w.set_en_ana(false);
            w.set_en_vco(false);
            w.set_en_iary(false);
        });

        // Disable bandgap + refgen
        codec.refgen_cfg().modify(|w| w.set_en(false));
        codec.bg_cfg0().modify(|w| w.set_en(false));

        // Disable HXT audio buffer
        pac::PMUC.hxt_cr1().modify(|w| w.set_buf_aud_en(false));

        // Update clocks cache
        Self::update_clocks_cache(None);

        // Release singleton
        TAKEN.store(false, Ordering::SeqCst);
    }
}
