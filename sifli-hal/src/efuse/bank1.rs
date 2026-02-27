//! SF32LB52x EFUSE bank1 factory calibration values.

// `bitfield-struct` generates `const fn` setters that call `panic!` on out-of-range values.
// This crate defines a `panic!` macro that maps to `defmt::panic!` when `defmt` is enabled,
// which is not usable in const-eval. Shadow it here so the generated code uses `core::panic!`.
#[allow(unused_macros)]
macro_rules! panic {
    ($($x:tt)*) => {
        ::core::panic!($($x)*)
    };
}

use bitfield_struct::bitfield;

/// Decoded EFUSE bank1 calibration values.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Bank1Calibration {
    pub primary: Bank1Primary,
    pub vol2: Bank1Vol2,
}

/// Bank1 "primary" calibration fields (positions 0..=159).
///
/// This spans multiple underlying words, so it is represented as two bitfields:
/// - [`Bank1PrimaryLow`]  (bits 0..=127)
/// - [`Bank1PrimaryHigh`] (bits 128..=159)
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Bank1Primary {
    pub low: Bank1PrimaryLow,
    pub high: Bank1PrimaryHigh,
}

impl Bank1Calibration {
    pub(crate) fn decode(words: &[u32; 8]) -> Self {
        let low_bits = u128_from_le_words(&words[0..4]);
        let high_bits = u128_from_le_words(&words[4..8]);

        let low: Bank1PrimaryLow = low_bits.into();
        let high: Bank1PrimaryHigh = (high_bits as u32).into();
        let vol2: Bank1Vol2 = (high_bits >> 32).into();

        Self {
            primary: Bank1Primary { low, high },
            vol2,
        }
    }
}

fn u128_from_le_words(words: &[u32]) -> u128 {
    debug_assert!(words.len() == 4);
    (words[0] as u128)
        | ((words[1] as u128) << 32)
        | ((words[2] as u128) << 64)
        | ((words[3] as u128) << 96)
}

#[bitfield(u128, defmt = cfg(feature = "defmt"))]
#[derive(PartialEq, Eq)]
pub struct Bank1PrimaryLow {
    /// [2:0] BUCK_VOS_TRIM
    #[bits(3)]
    pub buck_vos_trim: u8,
    /// [3] BUCK_VOS_POLAR
    pub buck_vos_polar: bool,
    /// [7:4] HPSYS_LDO_VOUT
    #[bits(4)]
    pub hpsys_ldo_vout: u8,
    /// [11:8] LPSYS_LDO_VOUT
    #[bits(4)]
    pub lpsys_ldo_vout: u8,
    /// [15:12] VRET_TRIM
    #[bits(4)]
    pub vret_trim: u8,
    /// [19:16] LDO18_VREF_SEL
    #[bits(4)]
    pub ldo18_vref_sel: u8,
    /// [23:20] VDD33_LDO2_VOUT
    #[bits(4)]
    pub vdd33_ldo2_vout: u8,
    /// [27:24] VDD33_LDO3_VOUT
    #[bits(4)]
    pub vdd33_ldo3_vout: u8,
    /// [30:28] AON_VOS_TRIM
    #[bits(3)]
    pub aon_vos_trim: u8,
    /// [31] AON_VOS_POLAR
    pub aon_vos_polar: bool,

    /// [43:32] ADC_VOL1_REG
    #[bits(12)]
    pub adc_vol1_reg: u16,
    /// [48:44] VOLT1_100MV
    #[bits(5)]
    pub volt1_100mv: u8,
    /// [60:49] ADC_VOL2_REG
    #[bits(12)]
    pub adc_vol2_reg: u16,
    /// [65:61] VOLT2_100MV
    #[bits(5)]
    pub volt2_100mv: u8,
    /// [77:66] VBAT_REG
    #[bits(12)]
    pub vbat_reg: u16,
    /// [83:78] VBAT_VOLT_100MV
    #[bits(6)]
    pub vbat_volt_100mv: u8,

    /// [87:84] PROG_V1P2
    #[bits(4)]
    pub prog_v1p2: u8,
    /// [93:88] CV_VCTRL
    #[bits(6)]
    pub cv_vctrl: u8,
    /// [98:94] CC_MN
    #[bits(5)]
    pub cc_mn: u8,
    /// [103:99] CC_MP
    #[bits(5)]
    pub cc_mp: u8,

    /// [106:104] BUCK_VOS_TRIM2
    #[bits(3)]
    pub buck_vos_trim2: u8,
    /// [107] BUCK_VOS_POLAR2
    pub buck_vos_polar2: bool,
    /// [111:108] HPSYS_LDO_VOUT2
    #[bits(4)]
    pub hpsys_ldo_vout2: u8,
    /// [115:112] LPSYS_LDO_VOUT2
    #[bits(4)]
    pub lpsys_ldo_vout2: u8,

    /// [123:116] VBAT_STEP
    #[bits(8)]
    pub vbat_step: u8,
    /// [124] IS_IO18
    pub is_io18: bool,
    /// [125] ERD_Cal_Done
    pub edr_cal_done: bool,
    /// [127:126] PA_BM
    #[bits(2)]
    pub pa_bm: u8,
}

#[bitfield(u32, defmt = cfg(feature = "defmt"))]
#[derive(PartialEq, Eq)]
pub struct Bank1PrimaryHigh {
    /// [129:128] DAC_LSB_CNT
    #[bits(2)]
    pub dac_lsb_cnt: u8,
    /// [130] tmxcap_flag
    pub tmxcap_flag: bool,
    /// [134:131] tmxcap_ch78
    #[bits(4)]
    pub tmxcap_ch78: u8,
    /// [138:135] tmxcap_ch00
    #[bits(4)]
    pub tmxcap_ch00: u8,
    /// [139] Vref_Flag
    pub vref_flag: bool,
    /// [143:140] Vref_REG
    #[bits(4)]
    pub vref_reg: u8,
    /// [159:144] Reserved
    #[bits(16)]
    __reserved0: u16,
}

/// Bank1 voltage calibration fields at positions 160..=255, shifted to bit 0.
#[bitfield(u128, defmt = cfg(feature = "defmt"))]
#[derive(PartialEq, Eq)]
pub struct Bank1Vol2 {
    /// [162:160] BUCK_VOS_TRIM_Vol2
    #[bits(3)]
    pub buck_vos_trim: u8,
    /// [163] BUCK_VOS_POLAR_Vol2
    pub buck_vos_polar: bool,
    /// [167:164] HPSYS_LDO_VOUT_Vol2
    #[bits(4)]
    pub hpsys_ldo_vout: u8,
    /// [171:168] LPSYS_LDO_VOUT_Vol2
    #[bits(4)]
    pub lpsys_ldo_vout: u8,
    /// [175:172] VRET_TRIM_Vol2
    #[bits(4)]
    pub vret_trim: u8,

    /// [187:176] ADC_VOL1_REG_Vol2
    #[bits(12)]
    pub adc_vol1_reg: u16,
    /// [192:188] VOLT1_100MV_Vol2
    #[bits(5)]
    pub volt1_100mv: u8,
    /// [204:193] ADC_VOL2_REG_Vol2
    #[bits(12)]
    pub adc_vol2_reg: u16,
    /// [209:205] VOLT2_100MV_Vol2
    #[bits(5)]
    pub volt2_100mv: u8,
    /// [221:210] VBAT_REG_Vol2
    #[bits(12)]
    pub vbat_reg: u16,
    /// [227:222] VBAT_VOLT_100MV_Vol2
    #[bits(6)]
    pub vbat_volt_100mv: u8,

    /// [231:228] HPSYS_LDO_VOUT2_Vol2
    #[bits(4)]
    pub hpsys_ldo_vout2: u8,
    /// [232] ERD_Cal_Vol2_flag
    pub erd_cal_flag: bool,
    /// [234:233] PA_BM_Vol2
    #[bits(2)]
    pub pa_bm: u8,
    /// [236:235] DAC_LSB_CNT_Vol2
    #[bits(2)]
    pub dac_lsb_cnt: u8,

    /// [237] tmxcap_Vol2_flag
    pub tmxcap_flag: bool,
    /// [241:238] tmxcap_ch78_Vol2
    #[bits(4)]
    pub tmxcap_ch78: u8,
    /// [245:242] tmxcap_ch00_Vol2
    #[bits(4)]
    pub tmxcap_ch00: u8,
    /// [255:246] Reserved
    #[bits(10)]
    __reserved0: u16,

    /// Padding to fill the `u128` representation.
    __padding: u32,
}
