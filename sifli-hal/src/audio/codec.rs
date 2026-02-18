//! AUDCODEC low-level control
//!
//! Manages the analog codec: PLL, bandgap, DAC/ADC power-up sequences.
//! These functions operate directly on the PAC singleton since AUDCODEC
//! has no independent interrupt and is always accessed alongside AUDPRC.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::pac;

fn delay_us(us: u32) {
    crate::blocking_delay_us(us);
}

fn delay_ms(ms: u32) {
    crate::blocking_delay_us(ms * 1000);
}

/// Tracks whether the shared codec common init (bandgap + PLL) has been done.
static COMMON_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialize shared AUDCODEC resources: HXT buffer, bandgap, PLL.
///
/// Safe to call multiple times — only the first call performs initialization.
/// Shared by both DAC and ADC paths.
fn init_codec_common() {
    if COMMON_INITIALIZED.swap(true, Ordering::SeqCst) {
        return;
    }

    let codec = pac::AUDCODEC;

    // HXT audio buffer enable
    pac::PMUC.hxt_cr1().modify(|w| w.set_buf_aud_en(true));

    // ===== Bandgap + Refgen =====
    codec.bg_cfg1().write(|w| w.0 = 48000);
    codec.bg_cfg2().write(|w| w.0 = 48_000_000);
    codec.bg_cfg0().modify(|w| {
        w.set_en(true);
        w.set_en_rcflt(true); // RC filter enabled (needed by ADC)
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

    // ===== PLL =====
    codec.pll_cfg0().modify(|w| w.set_en_iary(true));
    codec.pll_cfg0().modify(|w| w.set_en_vco(true));
    codec.pll_cfg0().modify(|w| w.set_en_ana(true));
    codec.pll_cfg0().modify(|w| w.set_icp_sel(8));
    codec.pll_cfg2().modify(|w| w.set_en_dig(true));
    codec.pll_cfg3().modify(|w| w.set_en_sdm(true));
    codec.pll_cfg4().modify(|w| w.set_en_clk_dig(true));

    // Loop filter configuration (must be set before VCO calibration)
    codec.pll_cfg1().modify(|w| {
        w.set_r3_sel(3);
        w.set_rz_sel(1);
        w.set_c2_sel(3);
        w.set_cz_sel(6);
        w.set_csd_rst(false);
        w.set_csd_en(false);
    });
    delay_us(50);

    // VCO calibration (binary search)
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
    // Neighbor refinement: measure fc_vco-1, fc_vco, fc_vco+1
    // and pick the one closest to target
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

    // PLL frequency: 49.152MHz for 48kHz family
    // Formula: [(FCW + 3) + SDIN / 2^20] * 6 MHz
    // (5 + 3 + 201327/1048576) * 6 = 8.192 * 6 = 49.152 MHz
    codec.pll_cfg2().modify(|w| w.set_rstb(true));
    delay_us(50);
    codec.pll_cfg3().write(|w| {
        w.set_sdin(201327);
        w.set_fcw(5);
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

    // Check PLL lock
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

    // PLL_CFG5: chopping clocks for bandgap and refgen (shared by DAC and ADC)
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
}

/// Initialize AUDCODEC for DAC output.
///
/// Performs the full power-up sequence:
/// 1. Shared init (HXT, bandgap, PLL)
/// 2. DAC channel config
/// 3. AUDCODEC DAC digital enable
///
/// After this, call `codec_unmute_dac()` once audio data is flowing.
pub(crate) fn init_codec_dac(volume: u8) {
    let codec = pac::AUDCODEC;

    init_codec_common();

    // ===== DAC channel 0 config =====
    codec.cfg().modify(|w| w.set_adc_en_dly_sel(3));

    codec.dac_cfg().write(|w| {
        w.set_osr_sel(0);
        w.set_op_mode(0); // internal bus from AUDPRC
        w.set_path_reset(false);
        w.set_clk_src_sel(false);
        w.set_clk_div(10);
    });

    codec.dac_ch0_cfg().write(|w| {
        w.set_enable(true);
        w.set_dout_mute(false);
        w.set_dem_mode(2);
        w.set_dma_en(false); // data from AUDPRC, not APB DMA
        w.set_rough_vol(volume);
        w.set_fine_vol(0);
        w.set_data_format(true); // 16-bit
        w.set_sinc_gain(0x14D);
    });
    codec.dac_ch0_cfg_ext().write(|w| {
        w.set_ramp_en(true);
        w.set_ramp_mode(true);
        w.set_zero_adjust_en(true);
        w.set_ramp_interval(6);
    });

    // DAC_CH0_DEBUG: required for data path
    codec.dac_ch0_debug().write(|w| {
        w.set_bypass(false);
        w.set_data_out(0xFF);
    });
}

/// Start DAC analog output path.
///
/// Enables AUDCODEC DAC, powers up analog, initially muted to avoid pop noise.
/// Call `codec_unmute_dac()` after a short delay.
pub(crate) fn start_dac_analog() {
    let codec = pac::AUDCODEC;

    // Enable DAC digital
    codec.cfg().modify(|w| w.set_dac_enable(true));

    // Mute during analog startup
    codec.dac_ch0_cfg().modify(|w| w.set_dout_mute(true));
    codec.dac_ch0_debug().modify(|w| w.set_bypass(true));

    // PLL clock config for DAC
    codec.pll_cfg4().write(|w| {
        w.set_divb_clk_chop_dac(2);
        w.set_diva_clk_chop_dac(4);
        w.set_en_clk_chop_dac(true);
        w.set_diva_clk_dac(5);
        w.set_en_clk_dac(true);
        w.set_sel_clk_dac_source(0);
        w.set_en_clk_dig(true);
        w.set_clk_dig_str(1);
        w.set_diva_clk_dig(2);
    });
    codec.pll_cfg2().modify(|w| w.set_rstb(false));
    delay_us(100);
    codec.pll_cfg2().modify(|w| w.set_rstb(true));

    // DAC1 analog power-up sequence
    codec.dac1_cfg().modify(|w| w.set_en_os_dac(false));
    codec.dac1_cfg().modify(|w| w.set_en_vcm(true));
    delay_us(5);
    codec.dac1_cfg().modify(|w| w.set_en_amp(true));
    delay_us(1);
    codec.dac1_cfg().modify(|w| w.set_en_os_dac(true));
    delay_us(10);
    codec.dac1_cfg().modify(|w| w.set_en_dac(true));
    delay_us(10);
    codec.dac1_cfg().modify(|w| w.set_sr(false));
}

/// Unmute DAC output after analog has stabilized.
pub(crate) fn codec_unmute_dac() {
    let codec = pac::AUDCODEC;
    codec.dac_ch0_debug().modify(|w| w.set_bypass(false));
    codec.dac_ch0_cfg().modify(|w| w.set_dout_mute(false));
}

/// Mute DAC output.
pub(crate) fn codec_mute_dac() {
    let codec = pac::AUDCODEC;
    codec.dac_ch0_cfg().modify(|w| w.set_dout_mute(true));
}

/// Set DAC coarse volume (0-15).
pub(crate) fn codec_set_volume(vol: u8) {
    let codec = pac::AUDCODEC;
    let vol = vol.min(15);
    codec.dac_ch0_cfg().modify(|w| w.set_rough_vol(vol));
}

/// Shutdown DAC analog path.
pub(crate) fn shutdown_dac() {
    let codec = pac::AUDCODEC;

    // Mute first
    codec.dac_ch0_cfg().modify(|w| w.set_dout_mute(true));
    codec.dac_ch0_debug().modify(|w| w.set_bypass(true));

    // Power down DAC1 analog (reverse order)
    codec.dac1_cfg().modify(|w| w.set_sr(true));
    codec.dac1_cfg().modify(|w| w.set_en_dac(false));
    codec.dac1_cfg().modify(|w| w.set_en_os_dac(false));
    codec.dac1_cfg().modify(|w| w.set_en_amp(false));
    codec.dac1_cfg().modify(|w| w.set_en_vcm(false));

    // Disable DAC digital
    codec.cfg().modify(|w| w.set_dac_enable(false));
}

// ============================================================================
// ADC codec functions
// ============================================================================

/// Initialize AUDCODEC for ADC input (microphone recording).
///
/// Performs:
/// 1. Shared init (HXT, bandgap, PLL) — skipped if already done
/// 2. LPSYS_RCC AUDCODEC clock enable (ADC analog domain)
/// 3. MICBIAS enable
/// 4. ADC analog clocks (PLL_CFG6)
/// 5. ADC1 analog power-up (using `modify()` to preserve bootloader bias calibration)
/// 6. AUDCODEC ADC digital path config
///
/// `volume` is the AUDCODEC ADC_CH0 rough_vol (0-15, default 6 = 0dB).
pub(crate) fn init_codec_adc(volume: u8) {
    let codec = pac::AUDCODEC;

    init_codec_common();

    // Enable LPSYS_RCC AUDCODEC clock (bit 24 of ENR1).
    // ADC analog portion uses LP clock domain. The PAC may not expose this
    // register field, so we use raw pointer access as in the validated test.
    unsafe {
        let lpsys_rcc_enr1 = 0x4000_0004 as *mut u32;
        let val = core::ptr::read_volatile(lpsys_rcc_enr1);
        core::ptr::write_volatile(lpsys_rcc_enr1, val | (1 << 24));
    }
    delay_ms(1);

    // ===== MICBIAS enable =====
    codec.bg_cfg0().modify(|w| w.set_en_smpl(false));
    codec.adc_ana_cfg().modify(|w| w.set_micbias_en(true));
    codec
        .adc_ana_cfg()
        .modify(|w| w.set_micbias_chop_en(false));
    delay_ms(2);
    codec.bg_cfg0().modify(|w| w.set_en_smpl(true));

    // ===== ADC analog clocks (PLL_CFG6) =====
    // C SDK codec_adc_clk_config[48kHz]: sel_clk_adc=1 (PLL), diva=5
    codec.pll_cfg6().write(|w| {
        w.set_sel_clk_chop_micbias(3);
        w.set_en_clk_chop_micbias(true);
        w.set_sel_clk_adc2(true); // PLL
        w.set_diva_clk_adc2(5);
        w.set_en_clk_adc2(true);
        w.set_sel_clk_adc1(true); // PLL
        w.set_diva_clk_adc1(5);
        w.set_en_clk_adc1(true);
        w.set_sel_clk_adc0(true); // PLL
        w.set_diva_clk_adc0(5);
        w.set_en_clk_adc0(true);
        w.set_sel_clk_adc_source(0); // XTAL master
    });
    // Reset PLL after clock config change
    codec.pll_cfg2().modify(|w| w.set_rstb(false));
    delay_ms(1);
    codec.pll_cfg2().modify(|w| w.set_rstb(true));
    delay_us(50);

    // ===== ADC1 analog power-up =====
    // CRITICAL: Use modify() to preserve bootloader-calibrated bias fields
    // (bm_int1, bm_int2, peri_bm in ADC1_CFG1/CFG2).
    codec.adc1_cfg1().modify(|w| w.set_fsp(0));
    codec.adc1_cfg1().modify(|w| w.set_vcmst(true));
    codec.adc1_cfg2().modify(|w| w.set_clear(true));
    codec.adc1_cfg1().modify(|w| w.set_gc(4)); // 18dB PGA gain
    codec.adc1_cfg1().modify(|w| {
        w.set_dacn_en(false);
        w.set_diff_en(false);
    });
    codec.adc1_cfg2().modify(|w| w.set_en(true));
    codec.adc1_cfg2().modify(|w| w.set_rstb(false)); // hold in reset
    codec.adc1_cfg1().modify(|w| w.set_vref_sel(2));

    // ADC2 analog power-up (same sequence)
    codec.adc2_cfg1().modify(|w| w.set_fsp(0));
    codec.adc2_cfg1().modify(|w| w.set_vcmst(true));
    codec.adc2_cfg2().modify(|w| w.set_clear(true));
    codec.adc2_cfg1().modify(|w| w.set_gc(4)); // 18dB PGA gain
    codec.adc2_cfg2().modify(|w| w.set_en(true));
    codec.adc2_cfg2().modify(|w| w.set_rstb(false));
    codec.adc2_cfg1().modify(|w| w.set_vref_sel(2));

    // Wait 20ms for analog settling
    delay_ms(20);

    // Release reset, clear VCMST and CLEAR
    codec.adc1_cfg2().modify(|w| w.set_rstb(true));
    codec.adc1_cfg1().modify(|w| w.set_vcmst(false));
    codec.adc1_cfg2().modify(|w| w.set_clear(false));
    codec.adc2_cfg2().modify(|w| w.set_rstb(true));
    codec.adc2_cfg1().modify(|w| w.set_vcmst(false));
    codec.adc2_cfg2().modify(|w| w.set_clear(false));

    // ===== AUDCODEC digital ADC path =====
    codec.cfg().modify(|w| w.set_adc_en_dly_sel(3));

    // ADC_CFG: normal mode (op_mode=0, data → AUDPRC RX)
    codec.adc_cfg().write(|w| {
        w.set_osr_sel(0); // OSR=200
        w.set_op_mode(0); // normal mode → AUDPRC
        w.set_path_reset(false);
        w.set_clk_src_sel(false); // XTAL 48MHz
        w.set_clk_div(5); // 48/(5+1)=8MHz
    });

    let vol = volume.min(15);

    // ADC_CH0_CFG: enable with HPF
    codec.adc_ch0_cfg().write(|w| {
        w.set_enable(true);
        w.set_hpf_bypass(false);
        w.set_hpf_coef(0x7);
        w.set_stb_inv(false);
        w.set_dma_en(false); // data goes via AUDPRC, not APB
        w.set_rough_vol(vol);
        w.set_fine_vol(0);
        w.set_data_format(true); // 16-bit
    });

    // ADC_CH1_CFG: also enable for stereo path
    codec.adc_ch1_cfg().write(|w| {
        w.set_enable(true);
        w.set_hpf_bypass(false);
        w.set_hpf_coef(0x7);
        w.set_stb_inv(false);
        w.set_dma_en(false);
        w.set_rough_vol(vol);
        w.set_fine_vol(0);
        w.set_data_format(true);
    });

    // Enable ADC digital
    codec.cfg().modify(|w| w.set_adc_enable(true));
}

/// Shutdown ADC codec path.
///
/// Reverse order: disable digital → reset ADC1/ADC2 → disable MICBIAS → disable clocks.
pub(crate) fn shutdown_adc() {
    let codec = pac::AUDCODEC;

    // Disable ADC digital
    codec.cfg().modify(|w| w.set_adc_enable(false));

    // Reset ADC1 analog
    codec.adc1_cfg2().modify(|w| {
        w.set_en(false);
        w.set_rstb(false);
    });
    // Reset ADC2 analog
    codec.adc2_cfg2().modify(|w| {
        w.set_en(false);
        w.set_rstb(false);
    });

    // Disable MICBIAS
    codec.adc_ana_cfg().modify(|w| w.set_micbias_en(false));

    // Disable ADC clocks (PLL_CFG6)
    codec.pll_cfg6().write(|w| {
        w.set_en_clk_chop_micbias(false);
        w.set_en_clk_adc2(false);
        w.set_en_clk_adc1(false);
        w.set_en_clk_adc0(false);
    });
}
