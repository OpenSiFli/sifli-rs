//! EFUSE (eFuse controller)

use core::marker::PhantomData;

use embassy_hal_internal::Peripheral;

use crate::pac::{EFUSEC, PMUC};
use crate::{blocking_delay_us, peripherals, rcc};

mod bank1;
pub use bank1::{Bank1Calibration, Bank1Primary, Bank1PrimaryHigh, Bank1PrimaryLow, Bank1Vol2};

/// EFUSE error.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// PCLK frequency is unknown.
    PclkUnknown,
    /// PCLK is higher than the supported limit.
    PclkTooFast { pclk_hz: u32 },
    /// A timing value does not fit in the EFUSE timing register.
    TimingOutOfRange { field: &'static str, value: u32 },
    /// Bank index is invalid.
    InvalidBank { bank: u8 },
    /// EFUSE read operation timed out.
    Timeout { bank: u8 },
}

/// Unique ID (UID) read from EFUSE.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Uid {
    bytes: [u8; 16],
}

impl Uid {
    /// Get UID raw bytes.
    pub fn bytes(&self) -> &[u8; 16] {
        &self.bytes
    }

    /// Get UID as 4 little-endian 32-bit words.
    pub fn words_le(&self) -> [u32; 4] {
        let b = self.bytes;
        [
            u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
            u32::from_le_bytes([b[12], b[13], b[14], b[15]]),
        ]
    }

    fn from_bank0_words(bank0_words: &[u32; 8]) -> Self {
        let mut bytes = [0u8; 16];
        for (i, word) in bank0_words.iter().take(4).enumerate() {
            bytes[i * 4..(i + 1) * 4].copy_from_slice(&word.to_le_bytes());
        }
        Self { bytes }
    }
}

/// EFUSE driver.
pub struct Efuse<'d> {
    bank0_words: [u32; 8],
    bank1_words: [u32; 8],
    uid: Uid,
    bank1_calibration: Bank1Calibration,
    _phantom: PhantomData<&'d peripherals::EFUSEC>,
}

impl<'d> Efuse<'d> {
    /// Create a new EFUSE driver, initialize the controller timing register, and cache the UID
    /// and bank0/bank1 raw contents in memory.
    pub fn new(_efusec: impl Peripheral<P = peripherals::EFUSEC> + 'd) -> Result<Self, Error> {
        rcc::enable_and_reset::<peripherals::EFUSEC>();
        init_timr()?;

        let bank0_words = read_bank_words(0)?;
        let bank1_words = read_bank_words(1)?;
        let uid = Uid::from_bank0_words(&bank0_words);
        let bank1_calibration = Bank1Calibration::decode(&bank1_words);

        Ok(Self {
            bank0_words,
            bank1_words,
            uid,
            bank1_calibration,
            _phantom: PhantomData,
        })
    }

    /// Get cached UID.
    pub fn uid(&self) -> Uid {
        self.uid
    }

    /// Get cached raw bank0 words.
    pub fn bank0_words(&self) -> &[u32; 8] {
        &self.bank0_words
    }

    /// Get cached raw bank1 words.
    pub fn bank1_words(&self) -> &[u32; 8] {
        &self.bank1_words
    }

    /// Get cached bank1 factory calibration values.
    pub fn calibration(&self) -> &Bank1Calibration {
        &self.bank1_calibration
    }
}

fn init_timr() -> Result<(), Error> {
    let pclk_hz = rcc::get_pclk_freq().ok_or(Error::PclkUnknown)?.0;

    // CSDK: EFUSE_PCLK_LIMIT = 120000000
    #[cfg(not(feature = "unchecked-overclocking"))]
    if pclk_hz > 120_000_000 {
        return Err(Error::PclkTooFast { pclk_hz });
    }

    let (thrck, thpck, tckhp) = compute_timings(pclk_hz)?;
    EFUSEC.timr().write(|w| {
        w.set_thrck(thrck);
        w.set_thpck(thpck);
        w.set_tckhp(tckhp);
    });

    Ok(())
}

fn read_bank_words(bank: u8) -> Result<[u32; 8], Error> {
    if bank >= 4 {
        return Err(Error::InvalidBank { bank });
    }

    // CSDK (SF32LB52X): temporarily raise HPSYS_VOUT before reading efuse.
    let org_vout = PMUC.hpsys_vout().read();
    let mut boosted_vout = org_vout;
    let mut value = boosted_vout.vout() as u32 + 3;
    value = value.clamp(0xe, 0xf);
    boosted_vout.set_vout(value as u8);
    PMUC.hpsys_vout().write_value(boosted_vout);
    blocking_delay_us(20);

    // Select bank and start READ.
    EFUSEC.cr().write(|w| {
        w.set_banksel(bank);
        w.set_mode(false);
        w.set_ie(false);
    });
    EFUSEC.cr().modify(|w| w.set_en(true));

    // CSDK timeout heuristic: timeout = size * 8 * 48000 (size in bytes, bank=32 bytes)
    let timeout = 32u32 * 8 * 48_000;
    let mut ready = 0u32;
    while !EFUSEC.sr().read().done() && ready < timeout {
        ready += 1;
    }

    // Write 1 to clear DONE.
    EFUSEC.sr().write(|w| w.set_done(true));

    if ready >= timeout {
        PMUC.hpsys_vout().write_value(org_vout);
        return Err(Error::Timeout { bank });
    }

    let words = read_bank_data_words(bank);

    // Restore HPSYS_VOUT.
    PMUC.hpsys_vout().write_value(org_vout);

    Ok(words)
}

fn read_bank_data_words(bank: u8) -> [u32; 8] {
    match bank {
        0 => [
            EFUSEC.bank0_data0().read().data(),
            EFUSEC.bank0_data1().read().data(),
            EFUSEC.bank0_data2().read().data(),
            EFUSEC.bank0_data3().read().data(),
            EFUSEC.bank0_data4().read().data(),
            EFUSEC.bank0_data5().read().data(),
            EFUSEC.bank0_data6().read().data(),
            EFUSEC.bank0_data7().read().data(),
        ],
        1 => [
            EFUSEC.bank1_data0().read().data(),
            EFUSEC.bank1_data1().read().data(),
            EFUSEC.bank1_data2().read().data(),
            EFUSEC.bank1_data3().read().data(),
            EFUSEC.bank1_data4().read().data(),
            EFUSEC.bank1_data5().read().data(),
            EFUSEC.bank1_data6().read().data(),
            EFUSEC.bank1_data7().read().data(),
        ],
        2 => [
            EFUSEC.bank2_data0().read().data(),
            EFUSEC.bank2_data1().read().data(),
            EFUSEC.bank2_data2().read().data(),
            EFUSEC.bank2_data3().read().data(),
            EFUSEC.bank2_data4().read().data(),
            EFUSEC.bank2_data5().read().data(),
            EFUSEC.bank2_data6().read().data(),
            EFUSEC.bank2_data7().read().data(),
        ],
        3 => [
            EFUSEC.bank3_data0().read().data(),
            EFUSEC.bank3_data1().read().data(),
            EFUSEC.bank3_data2().read().data(),
            EFUSEC.bank3_data3().read().data(),
            EFUSEC.bank3_data4().read().data(),
            EFUSEC.bank3_data5().read().data(),
            EFUSEC.bank3_data6().read().data(),
            EFUSEC.bank3_data7().read().data(),
        ],
        _ => unreachable!(),
    }
}

fn compute_timings(pclk_hz: u32) -> Result<(u8, u8, u16), Error> {
    // From CSDK `HAL_EFUSE_Init` (drivers/hal/bf0_hal_efuse.c).

    // EFUSE_RD_TIM_NS = 500
    let rd_thrck = (500u64 * pclk_hz as u64) / 1_000_000_000u64 + 1;
    if rd_thrck > 0x7f {
        return Err(Error::TimingOutOfRange {
            field: "thrck",
            value: rd_thrck as u32,
        });
    }

    // EFUSE_PGM_THPCK_NS = 20
    let pgm_thpck = (20u64 * pclk_hz as u64) / 1_000_000_000u64 + 1;
    if pgm_thpck > 0x07 {
        return Err(Error::TimingOutOfRange {
            field: "thpck",
            value: pgm_thpck as u32,
        });
    }

    // EFUSE_PGM_TCKHP_US = 10
    let mut pgm_tckhp = ((10u64 * pclk_hz as u64) + 500_000) / 1_000_000u64;
    let pgm_tckhp_ns = (pgm_tckhp * 1_000_000_000u64) / (pclk_hz as u64);
    if pgm_tckhp_ns > 11_000 {
        pgm_tckhp = pgm_tckhp.saturating_sub(1);
    } else if pgm_tckhp_ns < 9_000 {
        pgm_tckhp += 1;
    }
    if pgm_tckhp > 0x07ff {
        return Err(Error::TimingOutOfRange {
            field: "tckhp",
            value: pgm_tckhp as u32,
        });
    }

    Ok((rd_thrck as u8, pgm_thpck as u8, pgm_tckhp as u16))
}

#[cfg(test)]
mod tests;
