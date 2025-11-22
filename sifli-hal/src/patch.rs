//! LCPU patch installation module.
//!
//! Chooses A3 or Letter-Series patch layout based on chip revision and writes into the proper memory region.
//!
//! ```no_run
//! use sifli_hal::{patch, syscfg};
//!
//! let idr = syscfg::read_idr();
//! patch::install(&idr, &PATCH_LIST_BYTES, &PATCH_BIN_BYTES)?;
//! ```

use crate::lcpu::ram::PatchRegion;
use crate::syscfg::ChipRevision;

//=============================================================================
// Const
//=============================================================================

/// Patch tag magic number.
///
/// Reference: `SiFli-SDK/drivers/Include/bf0_hal_patch.h:83`
#[allow(dead_code)]
const PATCH_TAG: u32 = 0x5054_4348; // "PTCH" (big-endian in memory)

//=============================================================================
// Error types
//=============================================================================

/// Patch installation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Patch record list is empty.
    EmptyRecord,

    /// Patch code array is empty.
    EmptyCode,

    /// Patch code exceeds available space.
    CodeTooLarge {
        /// Actual size (bytes).
        size_bytes: usize,
        /// Maximum allowed size (bytes).
        max_bytes: usize,
    },

    /// Invalid or unsupported chip revision.
    InvalidRevision {
        /// Revision ID (`REVID`).
        revid: u8,
    },
}

//=============================================================================
// Core API
//=============================================================================

/// High-level helper to install LCPU patches based on chip revision.
///
/// ```no_run
/// use sifli_hal::{patch, syscfg};
///
/// let idr = syscfg::read_idr();
/// patch::install(idr.revision(), &PATCH_LIST_BYTES, &PATCH_BIN_BYTES)?;
/// ```
pub fn install(revision: ChipRevision, list: &[u8], bin: &[u8]) -> Result<(), Error> {
    // Parameter validation.
    if list.is_empty() {
        return Err(Error::EmptyRecord);
    }
    if bin.is_empty() {
        return Err(Error::EmptyCode);
    }

    if !revision.is_valid() {
        return Err(Error::InvalidRevision {
            revid: revision.revid(),
        });
    }

    // Dispatch to A3 or Letter-Series patch installer based on revision.
    if revision.is_letter_series() {
        install_letter(list, bin)
    } else {
        install_a3(list, bin)
    }
}

/// Install A3 / earlier-format patches (internal).
fn install_a3(list: &[u8], bin: &[u8]) -> Result<(), Error> {
    let code_size = bin.len();
    if code_size > PatchRegion::A3_TOTAL_SIZE {
        return Err(Error::CodeTooLarge {
            size_bytes: code_size,
            max_bytes: PatchRegion::A3_TOTAL_SIZE,
        });
    }

    debug!(
        "Installing A3 patch: record={} bytes, code={} bytes",
        list.len(),
        code_size
    );

    unsafe {
        // 1. Copy patch record list (entry list).
        let record_dst = PatchRegion::A3_RECORD_ADDR as *mut u8;
        core::ptr::copy_nonoverlapping(list.as_ptr(), record_dst, list.len());

        // 2. Clear patch code area (bytes).
        let code_dst = PatchRegion::A3_CODE_START as *mut u8;
        core::ptr::write_bytes(code_dst, 0, PatchRegion::A3_TOTAL_SIZE);

        // 3. Copy patch code as raw bytes.
        let code_dst = PatchRegion::A3_CODE_START as *mut u8;
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_dst, bin.len());
    }

    debug!("A3 patch installed successfully");
    Ok(())
}

/// Install Letter-Series patches (internal).
fn install_letter(_list: &[u8], bin: &[u8]) -> Result<(), Error> {
    let code_size = bin.len();
    if code_size > PatchRegion::LETTER_CODE_SIZE {
        return Err(Error::CodeTooLarge {
            size_bytes: code_size,
            max_bytes: PatchRegion::LETTER_CODE_SIZE,
        });
    }

    debug!("Installing Letter Series patch: code={} bytes", code_size);

    unsafe {
        // 1. Write header (12 bytes). Reference: lcpu_patch_rev_b.c:60-66.
        let header = [
            PatchRegion::LETTER_MAGIC,                 // magic: "PACH"
            PatchRegion::LETTER_ENTRY_COUNT,           // entry_count (fixed)
            PatchRegion::LETTER_CODE_START as u32 + 1, // code_addr (Thumb bit)
        ];
        let header_dst = PatchRegion::LETTER_BUF_START as *mut u32;
        core::ptr::copy_nonoverlapping(header.as_ptr(), header_dst, 3);

        // 2. Clear patch code area.
        let code_dst = PatchRegion::LETTER_CODE_START as *mut u8;
        core::ptr::write_bytes(code_dst, 0, PatchRegion::LETTER_CODE_SIZE);

        // 3. Copy patch code as raw bytes.
        let code_dst = PatchRegion::LETTER_CODE_START as *mut u8;
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_dst, bin.len());
    }

    info!("Letter Series patch installed successfully");
    Ok(())
}
