//! AUDCODEC low-level control
//!
//! Manages the analog codec: DAC/ADC power-up sequences.
//! PLL and bandgap initialization is handled by [`crate::aud_pll::AudioPll`].
//!
//! These functions operate directly on the PAC singleton since AUDCODEC
//! has no independent interrupt and is always accessed alongside AUDPRC.

use crate::pac;

// ---------------------------------------------------------------------------
// Named constants for AUDCODEC register field values
// (These replace magic numbers; ideally they'd be PAC-level enums in future.)
// ---------------------------------------------------------------------------

// DAC oversampling ratio: OSR = 300
const DAC_OSR_300: u8 = 0;
// ADC oversampling ratio: OSR = 200
const ADC_OSR_200: u8 = 0;
// Operation mode: internal bus (data via AUDPRC, not APB)
const OP_MODE_INTERNAL_BUS: u8 = 0;
// Dynamic Element Matching: 2nd-order
const DEM_2ND_ORDER: u8 = 2;
// SINC filter gain for OSR=300 (C SDK default: 333 = 0x14D)
const SINC_GAIN_OSR300: u16 = 0x14D;
// DAC clock divider: 48 MHz / (10+1) ≈ 4.36 MHz
const DAC_CLK_DIV: u8 = 10;
// ADC clock divider: 48 MHz / (5+1) = 8 MHz
const ADC_CLK_DIV: u8 = 5;
// PGA gain: 18 dB
const PGA_GAIN_18DB: u8 = 4;
// ADC voltage reference selection
const ADC_VREF_SEL: u8 = 2;
// HPF coefficient for ADC high-pass filter
const ADC_HPF_COEF: u8 = 0x7;

/// Initialize AUDCODEC for DAC output.
///
/// Performs the DAC-specific power-up sequence:
/// 1. DAC channel config
/// 2. AUDCODEC DAC digital enable
///
/// Requires [`AudioPll`](crate::aud_pll::AudioPll) to be initialized first.
/// After this, call `codec_unmute_dac()` once audio data is flowing.
pub(crate) fn init_codec_dac(volume: u8) {
    let codec = pac::AUDCODEC;

    // ===== DAC channel 0 config =====
    codec.cfg().modify(|w| w.set_adc_en_dly_sel(3));

    codec.dac_cfg().write(|w| {
        w.set_osr_sel(DAC_OSR_300);
        w.set_op_mode(OP_MODE_INTERNAL_BUS);
        w.set_path_reset(false);
        w.set_clk_src_sel(false); // XTAL 48MHz master clock
        w.set_clk_div(DAC_CLK_DIV);
    });

    codec.dac_ch0_cfg().write(|w| {
        w.set_enable(true);
        w.set_dout_mute(false);
        w.set_dem_mode(DEM_2ND_ORDER);
        w.set_dma_en(false); // data from AUDPRC, not APB DMA
        w.set_rough_vol(volume);
        w.set_fine_vol(0);
        w.set_data_format(true); // 16-bit
        w.set_sinc_gain(SINC_GAIN_OSR300);
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
    crate::blocking_delay_us(100);
    codec.pll_cfg2().modify(|w| w.set_rstb(true));

    // DAC1 analog power-up sequence
    codec.dac1_cfg().modify(|w| w.set_en_os_dac(false));
    codec.dac1_cfg().modify(|w| w.set_en_vcm(true));
    crate::blocking_delay_us(5);
    codec.dac1_cfg().modify(|w| w.set_en_amp(true));
    crate::blocking_delay_us(1);
    codec.dac1_cfg().modify(|w| w.set_en_os_dac(true));
    crate::blocking_delay_us(10);
    codec.dac1_cfg().modify(|w| w.set_en_dac(true));
    crate::blocking_delay_us(10);
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
/// 1. LPSYS_RCC AUDCODEC clock enable (ADC analog domain)
/// 2. MICBIAS enable
/// 3. ADC analog clocks (PLL_CFG6)
/// 4. ADC1 analog power-up (using `modify()` to preserve bootloader bias calibration)
/// 5. AUDCODEC ADC digital path config
///
/// Requires [`AudioPll`](crate::aud_pll::AudioPll) to be initialized first.
///
/// `volume` is the AUDCODEC ADC_CH0 rough_vol (0-15, default 6 = 0dB).
pub(crate) fn init_codec_adc(volume: u8) {
    let codec = pac::AUDCODEC;

    // Enable LPSYS_RCC AUDCODEC clock (bit 24 of ENR1).
    // ADC analog portion uses LP clock domain. Bit 24 is in the SVD's RSVD
    // region so there is no named field — use raw register access.
    pac::LPSYS_RCC.enr1().modify(|w| w.0 |= 1 << 24);
    crate::blocking_delay_us(1_000);

    // ===== MICBIAS enable =====
    codec.bg_cfg0().modify(|w| w.set_en_smpl(false));
    codec.adc_ana_cfg().modify(|w| w.set_micbias_en(true));
    codec
        .adc_ana_cfg()
        .modify(|w| w.set_micbias_chop_en(false));
    crate::blocking_delay_us(2_000);
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
    crate::blocking_delay_us(1_000);
    codec.pll_cfg2().modify(|w| w.set_rstb(true));
    crate::blocking_delay_us(50);

    // ===== ADC1 analog power-up =====
    // CRITICAL: Use modify() to preserve bootloader-calibrated bias fields
    // (bm_int1, bm_int2, peri_bm in ADC1_CFG1/CFG2).
    codec.adc1_cfg1().modify(|w| w.set_fsp(0));
    codec.adc1_cfg1().modify(|w| w.set_vcmst(true));
    codec.adc1_cfg2().modify(|w| w.set_clear(true));
    codec.adc1_cfg1().modify(|w| w.set_gc(PGA_GAIN_18DB));
    codec.adc1_cfg1().modify(|w| {
        w.set_dacn_en(false);
        w.set_diff_en(false);
    });
    codec.adc1_cfg2().modify(|w| w.set_en(true));
    codec.adc1_cfg2().modify(|w| w.set_rstb(false)); // hold in reset
    codec.adc1_cfg1().modify(|w| w.set_vref_sel(ADC_VREF_SEL));

    // ADC2 analog power-up (same sequence)
    codec.adc2_cfg1().modify(|w| w.set_fsp(0));
    codec.adc2_cfg1().modify(|w| w.set_vcmst(true));
    codec.adc2_cfg2().modify(|w| w.set_clear(true));
    codec.adc2_cfg1().modify(|w| w.set_gc(PGA_GAIN_18DB));
    codec.adc2_cfg2().modify(|w| w.set_en(true));
    codec.adc2_cfg2().modify(|w| w.set_rstb(false));
    codec.adc2_cfg1().modify(|w| w.set_vref_sel(ADC_VREF_SEL));

    // Wait 20ms for analog settling
    crate::blocking_delay_us(20_000);

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
        w.set_osr_sel(ADC_OSR_200);
        w.set_op_mode(OP_MODE_INTERNAL_BUS);
        w.set_path_reset(false);
        w.set_clk_src_sel(false); // XTAL 48MHz master clock
        w.set_clk_div(ADC_CLK_DIV);
    });

    let vol = volume.min(15);

    // ADC_CH0_CFG: enable with HPF
    codec.adc_ch0_cfg().write(|w| {
        w.set_enable(true);
        w.set_hpf_bypass(false);
        w.set_hpf_coef(ADC_HPF_COEF);
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
        w.set_hpf_coef(ADC_HPF_COEF);
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

    // Disable LPSYS_RCC AUDCODEC clock (bit 24 of ENR1), enabled during init.
    pac::LPSYS_RCC.enr1().modify(|w| w.0 &= !(1 << 24));
}
