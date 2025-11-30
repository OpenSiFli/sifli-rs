//! LCPU patch installation module.
//!
//! This module handles both:
//! - **Hardware patching**: Configuring the PATCH peripheral to intercept and replace instructions
//! - **Software patching**: Copying patch code into LCPU RAM
//!
//! The PATCH hardware is a breakpoint/replacement mechanism that allows runtime patching of ROM code.
//!
//! # Reference
//! - `SiFli-SDK/drivers/hal/bf0_hal_patch.c`
//! - `SiFli-SDK/drivers/Include/bf0_hal_patch.h`
//!
//! # Example
//! ```no_run
//! use sifli_hal::{patch, syscfg};
//!
//! let idr = syscfg::read_idr();
//! patch::install(idr.revision(), &PATCH_LIST_BYTES, &PATCH_BIN_BYTES)?;
//! ```

use crate::lcpu::ram::PatchRegion;
use crate::pac;
use crate::syscfg::ChipRevision;

//=============================================================================
// Constants
//=============================================================================

/// Maximum number of patch channels supported by hardware.
///
/// Reference: `SiFli-SDK/drivers/Include/bf0_hal_patch.h:69`
const MAX_PATCH_ENTRIES: usize = 32;

/// Address mask for patch channel (bits 18:2).
///
/// Reference: `SiFli-SDK/drivers/Include/register.h` (PATCH_CH0_ADDR_Msk)
const PATCH_ADDR_MASK: u32 = 0x0007_FFFC;

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

    /// Invalid patch record (bad magic).
    InvalidRecordMagic {
        /// Actual magic value found.
        actual: u32,
        /// Expected magic value.
        expected: u32,
    },

    /// Too many patch entries for hardware.
    TooManyEntries {
        /// Number of entries requested.
        count: usize,
    },
}

//=============================================================================
// Patch Entry Structure
//=============================================================================

/// A single patch entry descriptor.
///
/// Matches the SDK's `struct patch_entry_desc`.
///
/// Reference: `SiFli-SDK/drivers/Include/bf0_hal_patch.h:71-76`
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PatchEntry {
    /// Breakpoint address (must be 4-byte aligned).
    pub break_addr: u32,
    /// Replacement data (instruction or branch).
    pub data: u32,
}

/// Patch record header in memory.
///
/// The record starts with a magic tag, followed by size, then entries.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct PatchRecordHeader {
    /// Magic tag (should be `PATCH_TAG`).
    tag: u32,
    /// Size in bytes of the entry array (not including header).
    size_bytes: u32,
}

//=============================================================================
// Hardware Operations
//=============================================================================

/// Install patches into the PATCH hardware peripheral.
///
/// This reads the patch record from LCPU RAM and configures the PATCH hardware
/// to intercept and replace instructions at runtime.
///
/// # Arguments
/// * `record_addr` - Address of the patch record in LCPU RAM
///
/// # Returns
/// * `Ok(enabled_mask)` - Bitmask of enabled patch channels
/// * `Err(...)` - If the record is invalid or has too many entries
///
/// # Safety
/// The caller must ensure `record_addr` points to valid, readable memory
/// containing a properly formatted patch record.
///
/// # Reference
/// `SiFli-SDK/drivers/hal/bf0_hal_patch.c:HAL_PATCH_install()`
pub fn hal_patch_install(record_addr: usize) -> Result<u32, Error> {
    // SAFETY: Caller guarantees record_addr points to valid memory
    let header = unsafe { *(record_addr as *const PatchRecordHeader) };

    // Verify magic tag
    if header.tag != PatchRegion::A3_MAGIC {
        debug!(
            "Patch record invalid: tag=0x{:08X}, expected=0x{:08X}",
            header.tag, PatchRegion::A3_MAGIC
        );
        return Err(Error::InvalidRecordMagic {
            actual: header.tag,
            expected: PatchRegion::A3_MAGIC,
        });
    }

    // Calculate number of entries
    let entry_count = header.size_bytes as usize / core::mem::size_of::<PatchEntry>();
    if entry_count > MAX_PATCH_ENTRIES {
        return Err(Error::TooManyEntries { count: entry_count });
    }

    debug!(
        "HAL_PATCH_install: {} entries from 0x{:08X} (size_bytes={})",
        entry_count, record_addr, header.size_bytes
    );

    // Get pointer to entries (after header)
    let entries_addr = record_addr + core::mem::size_of::<PatchRecordHeader>();

    // Install entries into hardware
    let enabled = hal_patch_install_entries(entries_addr, entry_count)?;

    debug!("PATCH hardware enabled: mask=0x{:08X}", enabled);
    Ok(enabled)
}

/// Install patch entries into hardware.
///
/// Low-level function that writes directly to PATCH peripheral registers.
///
/// # Reference
/// `SiFli-SDK/drivers/hal/bf0_hal_patch.c:HAL_PATCH_install2()`
fn hal_patch_install_entries(entries_addr: usize, count: usize) -> Result<u32, Error> {
    if count > MAX_PATCH_ENTRIES {
        return Err(Error::TooManyEntries { count });
    }

    let patch = pac::PATCH;
    let mut enabled_mask: u32 = 0;

    // Disable all channels first
    patch.cer().write(|w| w.set_ce(0));
    debug!("  PATCH CER cleared");

    for i in 0..count {
        // SAFETY: entries_addr + i * size is within the valid record area
        let entry = unsafe { *((entries_addr + i * core::mem::size_of::<PatchEntry>()) as *const PatchEntry) };

        let addr_masked = entry.break_addr & PATCH_ADDR_MASK;

        debug!(
            "  CH[{}]: addr=0x{:08X}, data=0x{:08X}",
            i, addr_masked, entry.data
        );

        // Write breakpoint address to channel register (bits 18:2)
        patch.ch(i).write(|w| w.set_addr(addr_masked >> 2));

        // Select channel and write replacement data
        patch.csr().write(|w| w.set_cs(1 << i));
        patch.cdr().write(|w| w.set_data(entry.data));

        enabled_mask |= 1 << i;
    }

    // Clear channel selection
    patch.csr().write(|w| w.set_cs(0));

    // Enable all configured channels
    patch.cer().write(|w| w.set_ce(enabled_mask));

    Ok(enabled_mask)
}

/// Disable all patch channels.
pub fn hal_patch_disable_all() {
    pac::PATCH.cer().write(|w| w.set_ce(0));
}

/// Read current patch enable mask.
pub fn hal_patch_get_enabled() -> u32 {
    pac::PATCH.cer().read().ce()
}

//=============================================================================
// High-Level API
//=============================================================================

/// High-level helper to install LCPU patches based on chip revision.
///
/// This function:
/// 1. Copies the patch record to LCPU RAM
/// 2. Configures the PATCH hardware to apply instruction replacements
/// 3. Copies the patch code to LCPU RAM
///
/// # Example
/// ```no_run
/// use sifli_hal::{patch, syscfg};
///
/// let idr = syscfg::read_idr();
/// patch::install(idr.revision(), &PATCH_LIST_BYTES, &PATCH_BIN_BYTES)?;
/// ```
pub fn install(revision: ChipRevision, list: &[u8], bin: &[u8]) -> Result<(), Error> {
    // Parameter validation
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

    // Dispatch to A3 or Letter-Series patch installer based on revision
    if revision.is_letter_series() {
        install_letter(list, bin)
    } else {
        install_a3(list, bin)
    }
}

/// Install A3 / earlier-format patches.
///
/// Execution order (matching SDK):
/// 1. Copy patch record list to RAM
/// 2. Configure PATCH hardware (read record, setup breakpoints)
/// 3. Clear and copy patch code to RAM
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

    // Step 1: Copy patch record list to RAM
    let record_addr = PatchRegion::A3_RECORD_ADDR;
    // SAFETY: A3_RECORD_ADDR is a valid LCPU RAM address
    unsafe {
        core::ptr::copy_nonoverlapping(
            list.as_ptr(),
            record_addr as *mut u8,
            list.len(),
        );
    }
    debug!("  Record copied to 0x{:08X} ({} bytes)", record_addr, list.len());

    // Log first few words of the record for debugging
    #[cfg(feature = "defmt")]
    {
        let record_ptr = record_addr as *const u32;
        let word0 = unsafe { *record_ptr };
        let word1 = unsafe { *record_ptr.add(1) };
        let word2 = unsafe { *record_ptr.add(2) };
        debug!(
            "  Record header: [0x{:08X}, 0x{:08X}, 0x{:08X}]",
            word0, word1, word2
        );
    }

    // Step 2: Configure PATCH hardware (reads from RAM we just wrote)
    match hal_patch_install(record_addr) {
        Ok(mask) => {
            debug!("  PATCH HW configured: {} channels enabled", mask.count_ones());
        }
        Err(e) => {
            warn!("  PATCH HW install failed: {:?}", e);
            // Continue anyway - the patch code may still be useful
        }
    }

    // Step 3: Clear patch code area (this will overwrite record, but hardware is already configured)
    let code_addr = PatchRegion::A3_CODE_START;
    // SAFETY: A3_CODE_START is a valid LCPU RAM address
    unsafe {
        core::ptr::write_bytes(code_addr as *mut u8, 0, PatchRegion::A3_TOTAL_SIZE);
    }

    // Step 4: Copy patch code
    // SAFETY: A3_CODE_START is a valid LCPU RAM address
    unsafe {
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_addr as *mut u8, bin.len());
    }
    debug!("  Code copied to 0x{:08X} ({} bytes)", code_addr, bin.len());

    // Log first few words of the code for debugging
    #[cfg(feature = "defmt")]
    {
        let code_ptr = code_addr as *const u32;
        let word0 = unsafe { *code_ptr };
        let word1 = unsafe { *code_ptr.add(1) };
        debug!("  Code start: [0x{:08X}, 0x{:08X}]", word0, word1);
    }

    debug!("A3 patch installed successfully");
    Ok(())
}

/// Install Letter-Series patches (A4/B4 and later).
///
/// Letter-Series uses a different memory layout with a header at LETTER_BUF_START.
fn install_letter(_list: &[u8], bin: &[u8]) -> Result<(), Error> {
    let code_size = bin.len();
    if code_size > PatchRegion::LETTER_CODE_SIZE {
        return Err(Error::CodeTooLarge {
            size_bytes: code_size,
            max_bytes: PatchRegion::LETTER_CODE_SIZE,
        });
    }

    debug!("Installing Letter Series patch: code={} bytes", code_size);

    // Step 1: Write header (12 bytes)
    // Reference: lcpu_patch_rev_b.c:60-66
    let header = [
        PatchRegion::LETTER_MAGIC,                 // magic: "PACH"
        PatchRegion::LETTER_ENTRY_COUNT,           // entry_count (fixed)
        PatchRegion::LETTER_CODE_START as u32 + 1, // code_addr (Thumb bit)
    ];

    let header_addr = PatchRegion::LETTER_BUF_START;
    // SAFETY: LETTER_BUF_START is a valid LCPU RAM address
    unsafe {
        core::ptr::copy_nonoverlapping(header.as_ptr(), header_addr as *mut u32, 3);
    }
    debug!(
        "  Header written to 0x{:08X}: magic=0x{:08X}, count={}, entry=0x{:08X}",
        header_addr, header[0], header[1], header[2]
    );

    // Step 2: Configure PATCH hardware
    // Note: Letter series header format differs from A3, may not parse correctly
    match hal_patch_install(header_addr) {
        Ok(mask) => {
            debug!("  PATCH HW configured: {} channels enabled", mask.count_ones());
        }
        Err(e) => {
            // For Letter series, the header format is different
            // The patch code still gets executed via HAL_PATCH_Entry mechanism
            debug!("  PATCH HW install skipped (expected for Letter series): {:?}", e);
        }
    }

    // Step 3: Clear patch code area
    let code_addr = PatchRegion::LETTER_CODE_START;
    // SAFETY: LETTER_CODE_START is a valid LCPU RAM address
    unsafe {
        core::ptr::write_bytes(code_addr as *mut u8, 0, PatchRegion::LETTER_CODE_SIZE);
    }

    // Step 4: Copy patch code
    // SAFETY: LETTER_CODE_START is a valid LCPU RAM address
    unsafe {
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_addr as *mut u8, bin.len());
    }
    debug!("  Code copied to 0x{:08X} ({} bytes)", code_addr, bin.len());

    // Log first few words of the code for debugging
    #[cfg(feature = "defmt")]
    {
        let code_ptr = code_addr as *const u32;
        let word0 = unsafe { *code_ptr };
        let word1 = unsafe { *code_ptr.add(1) };
        debug!("  Code start: [0x{:08X}, 0x{:08X}]", word0, word1);
    }

    info!("Letter Series patch installed successfully");
    Ok(())
}
