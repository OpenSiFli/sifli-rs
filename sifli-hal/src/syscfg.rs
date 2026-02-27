//! SYSCFG / chip identification helper module.
//!
//! Lightweight access to `HPSYS_CFG->IDR` and friends for reading chip revision and IDs.
//!
//! ```no_run
//! use sifli_hal::syscfg;
//!
//! let idr = syscfg::read_idr();
//! let rev = idr.revision();
//! if rev.is_letter_series() {
//!     // Letter-series specific logic
//! }
//! ```

use crate::pac;

/// Read `HPSYS_CFG->IDR` and return a parsed [`Idr`].
///
/// ```no_run
/// use sifli_hal::syscfg;
///
/// let idr = syscfg::read_idr();
/// ```
#[inline]
pub fn read_idr() -> Idr {
    Idr::from_regs(pac::HPSYS_CFG)
}

/// Parsed view of the `IDR` (Identification Register) contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Idr {
    /// Revision ID (bit[7:0]) - hardware revision.
    pub revid: u8,

    /// Package ID (bit[15:8]) - package type.
    pub pid: u8,

    /// Company ID (bit[23:16]) - vendor/company ID.
    pub cid: u8,

    /// Series ID (bit[31:24]) - product series ID.
    pub sid: u8,
}

impl Idr {
    /// Read IDR from the HPSYS_CFG peripheral.
    #[inline]
    fn from_regs(regs: pac::hpsys_cfg::HpsysCfg) -> Self {
        let idr = regs.idr().read();
        Self {
            revid: idr.revid(),
            pid: idr.pid(),
            cid: idr.cid(),
            sid: idr.sid(),
        }
    }

    /// Parse the `revid` field into a [`ChipRevision`].
    #[inline]
    pub fn revision(&self) -> ChipRevision {
        ChipRevision::from_revid(self.revid)
    }

    /// Return the raw 32-bit IDR register value.
    #[inline]
    pub fn raw(&self) -> u32 {
        ((self.sid as u32) << 24)
            | ((self.cid as u32) << 16)
            | ((self.pid as u32) << 8)
            | (self.revid as u32)
    }
}

/// Chip revision information, matching SDK `__HAL_SYSCFG_CHECK_REVID()` logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipRevision {
    /// A3 and earlier revisions (0x00-0x03).
    A3OrEarlier(u8),

    /// A4 revision (0x07) - Letter Series.
    A4,

    /// B4 revision (0x0F) - Letter Series.
    B4,

    /// Invalid or unknown revision.
    Invalid(u8),
}

impl ChipRevision {
    /// Parse revision information from a REVID value.
    #[inline]
    pub fn from_revid(revid: u8) -> Self {
        match revid {
            0x00..=0x03 => ChipRevision::A3OrEarlier(revid),
            0x07 => ChipRevision::A4,
            0x0F => ChipRevision::B4,
            _ => ChipRevision::Invalid(revid),
        }
    }

    /// Whether this revision is considered valid by the SDK.
    #[inline]
    pub fn is_valid(&self) -> bool {
        !matches!(self, ChipRevision::Invalid(_))
    }

    /// Whether this is a Letter-Series revision (A4 or B4).
    #[inline]
    pub fn is_letter_series(&self) -> bool {
        matches!(self, ChipRevision::A4 | ChipRevision::B4)
    }

    /// Get the raw REVID value for this chip revision.
    #[inline]
    pub fn revid(&self) -> u8 {
        match self {
            ChipRevision::A3OrEarlier(id) => *id,
            ChipRevision::A4 => 0x07,
            ChipRevision::B4 => 0x0F,
            ChipRevision::Invalid(id) => *id,
        }
    }

    /// Return a human-readable revision name.
    #[inline]
    pub fn name(&self) -> &'static str {
        match self {
            ChipRevision::A3OrEarlier(0x00) => "Pre-A3 (ES v0.0)",
            ChipRevision::A3OrEarlier(0x01) => "Pre-A3 (ES v0.1)",
            ChipRevision::A3OrEarlier(0x02) => "Pre-A3 (ES v0.2)",
            ChipRevision::A3OrEarlier(0x03) => "A3",
            ChipRevision::A3OrEarlier(_) => unreachable!(),
            ChipRevision::A4 => "A4 (Letter Series)",
            ChipRevision::B4 => "B4 (Letter Series)",
            ChipRevision::Invalid(_) => "Invalid",
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ChipRevision {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ChipRevision::A3OrEarlier(id) => {
                defmt::write!(fmt, "A3OrEarlier(0x{:02x})", id)
            }
            ChipRevision::A4 => defmt::write!(fmt, "A4"),
            ChipRevision::B4 => defmt::write!(fmt, "B4"),
            ChipRevision::Invalid(id) => defmt::write!(fmt, "Invalid(0x{:02x})", id),
        }
    }
}

// ============================================================================
// Future extensions (placeholder implementations)
// ============================================================================

/// Chip boot mode (from `HPSYS_CFG->BMR`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootMode {
    /// Normal boot mode.
    Normal,
    /// Download / firmware update mode.
    Download,
}

/// Read current boot mode (not implemented yet; placeholder).
pub fn boot_mode() -> BootMode {
    todo!("boot_mode: read HPSYS_CFG->BMR register")
}
