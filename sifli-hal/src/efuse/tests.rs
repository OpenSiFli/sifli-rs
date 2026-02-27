use super::bank1::Bank1Calibration;
use super::Uid;

fn set_bits(words: &mut [u32; 8], pos: u16, bits: u8, value: u32) {
    for i in 0..bits {
        let bit_value = (value >> i) & 1;
        let bit_pos = pos + i as u16;

        let word_index = (bit_pos / 32) as usize;
        let bit_index = (bit_pos % 32) as u32;

        if bit_value != 0 {
            words[word_index] |= 1u32 << bit_index;
        } else {
            words[word_index] &= !(1u32 << bit_index);
        }
    }
}

#[test]
fn bank1_decode_matches_c_extraction() {
    // Cross-check the bit positions used by the CSDK helpers in:
    // `drivers/cmsis/sf32lb52x/bt_rf_fulcal.c`
    let mut words = [0u32; 8];
    set_bits(&mut words, 125, 1, 1); // EDR_Cal_Done
    set_bits(&mut words, 126, 2, 0b10); // PA_BM
    set_bits(&mut words, 128, 2, 0b01); // DAC_LSB_CNT
    set_bits(&mut words, 130, 1, 1); // tmxcap_flag
    set_bits(&mut words, 131, 4, 0b1111); // tmxcap_ch78
    set_bits(&mut words, 135, 4, 0b1010); // tmxcap_ch00

    let cal = Bank1Calibration::decode(&words);
    assert!(cal.primary.low.edr_cal_done());
    assert_eq!(cal.primary.low.pa_bm(), 0b10);
    assert_eq!(cal.primary.high.dac_lsb_cnt(), 0b01);
    assert!(cal.primary.high.tmxcap_flag());
    assert_eq!(cal.primary.high.tmxcap_ch78(), 0b1111);
    assert_eq!(cal.primary.high.tmxcap_ch00(), 0b1010);
}

#[test]
fn bank1_decode_cross_word_field() {
    let mut words = [0u32; 8];
    set_bits(&mut words, 61, 5, 0b10101); // VOLT2_100MV crosses 64-bit boundary

    let cal = Bank1Calibration::decode(&words);
    assert_eq!(cal.primary.low.volt2_100mv(), 0b10101);
}

#[test]
fn bank1_decode_aon_trim_does_not_overlap_polar() {
    let mut words = [0u32; 8];
    set_bits(&mut words, 28, 3, 0b101); // AON_VOS_TRIM [30:28]
    set_bits(&mut words, 31, 1, 1); // AON_VOS_POLAR [31]

    let cal = Bank1Calibration::decode(&words);
    assert_eq!(cal.primary.low.aon_vos_trim(), 0b101);
    assert!(cal.primary.low.aon_vos_polar());
}

#[test]
fn uid_words_le_roundtrip() {
    let bank0_words = [
        0x1122_3344,
        0x5566_7788,
        0x99aa_bbcc,
        0xddee_ff00,
        0,
        0,
        0,
        0,
    ];

    let uid = Uid::from_bank0_words(&bank0_words);
    assert_eq!(
        uid.bytes(),
        &[
            0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xcc, 0xbb, 0xaa, 0x99, 0x00, 0xff,
            0xee, 0xdd
        ]
    );
    assert_eq!(
        uid.words_le(),
        [0x1122_3344, 0x5566_7788, 0x99aa_bbcc, 0xddee_ff00]
    );
}
