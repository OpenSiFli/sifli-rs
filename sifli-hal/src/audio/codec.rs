//! AUDCODEC low-level control
//!
//! Manages the analog codec: PLL, bandgap, DAC/ADC power-up sequences.
//! These functions operate directly on the PAC singleton since AUDCODEC
//! has no independent interrupt and is always accessed alongside AUDPRC.

use crate::pac;

fn delay_us(us: u32) {
    crate::blocking_delay_us(us);
}

fn delay_ms(ms: u32) {
    crate::blocking_delay_us(ms * 1000);
}

/// Initialize AUDCODEC for DAC output.
///
/// Performs the full power-up sequence:
/// 1. Enable HXT audio buffer
/// 2. Bandgap + refgen
/// 3. PLL + VCO calibration
/// 4. DAC channel config
/// 5. AUDCODEC DAC digital enable
/// 6. DAC analog power-up (muted during startup)
///
/// After this, call `codec_unmute_dac()` once audio data is flowing.
pub(crate) fn init_codec_dac(volume: u8) {
    let codec = pac::AUDCODEC;

    // HXT audio buffer enable
    pac::PMUC.hxt_cr1().modify(|w| w.set_buf_aud_en(true));

    // ===== Bandgap + Refgen =====
    codec.bg_cfg1().write(|w| w.0 = 48000);
    codec.bg_cfg2().write(|w| w.0 = 48_000_000);
    codec
        .bg_cfg0()
        .modify(|w| {
            w.set_en(true);
            w.set_en_rcflt(false);
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
    delay_ms(1);

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
            fc_vco += delta;
        } else if cnt > target_cnt {
            fc_vco -= delta;
        }
        delta >>= 1;
    }
    codec.pll_cfg0().modify(|w| w.set_fc_vco(0));
    codec
        .pll_cfg0()
        .modify(|w| w.set_fc_vco(fc_vco as u8));
    codec.pll_cfg2().modify(|w| w.set_en_lf_vcin(false));
    codec.pll_cfg0().modify(|w| w.set_open(false));
    delay_us(50);

    // PLL frequency: ~48MHz (fcw=5, sdin=0)
    codec.pll_cfg2().modify(|w| w.set_rstb(true));
    delay_us(50);
    codec.pll_cfg3().write(|w| {
        w.set_sdin(0);
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
