//! LCPU memory management: firmware image loading and memory layout constants.

use crate::syscfg;
use core::ptr;

//=============================================================================
// Memory Map Constants
//=============================================================================

/// LCPU Patch memory layout (HCPU view).
///
/// Defines addresses for Patch code and buffers for different chip revisions.
#[derive(Debug, Clone, Copy)]
pub struct PatchRegion;

impl PatchRegion {
    // ===== A3 and earlier =====

    /// A3 patch record header magic value ("PTCH").
    /// Reference: `SiFli-SDK/drivers/Include/bf0_hal_patch.h:83`
    pub const A3_MAGIC: u32 = 0x5054_4348;

    /// Patch code start address for A3.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:328`
    pub const A3_CODE_START: usize = super::memory_map::a3::PATCH_CODE_START;

    /// Patch record area address for A3.
    /// Located at the last 256 bytes of the patch region.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:331` (`LCPU_PATCH_RECORD_ADDR`)
    pub const A3_RECORD_ADDR: usize = super::memory_map::a3::PATCH_RECORD_ADDR;

    /// Total patch area size for A3.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:300`
    pub const A3_TOTAL_SIZE: usize = 8 * 1024;

    // ===== Letter Series (A4/B4) =====

    /// Patch buffer start address.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:334`
    pub const LETTER_BUF_START: usize = super::memory_map::letter::PATCH_BUF_START;

    /// Patch code start address (after 12-byte header) — HCPU-visible (secure alias).
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:335`
    pub const LETTER_CODE_START: usize = super::memory_map::letter::PATCH_CODE_START;

    /// Patch code start address as seen by LCPU (non-secure alias).
    /// LCPU cannot access the 0x2040_xxxx range; it uses 0x0040_xxxx.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:336`
    pub const LETTER_CODE_START_LCPU: usize = super::memory_map::letter::PATCH_CODE_START_LCPU;

    /// Patch buffer size.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:337`
    pub const LETTER_BUF_SIZE: usize = 0x3000; // 12KB

    /// Patch code usable size.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:338`
    pub const LETTER_CODE_SIZE: usize = 0x2FF4; // 12KB - 12 bytes

    /// Letter Series patch header magic value ("PACH").
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c:60`
    pub const LETTER_MAGIC: u32 = 0x4843_4150;

    /// Fixed entry_count value in header.
    pub const LETTER_ENTRY_COUNT: u32 = 7;
}

/// LPSYS RAM layout (HCPU view, SF32LB52x).
#[derive(Debug, Clone, Copy)]
pub struct LpsysRam;

impl LpsysRam {
    /// LPSYS RAM base address (HCPU view).
    pub const BASE: usize = super::memory_map::shared::LPSYS_RAM_BASE;

    /// LPSYS RAM size for A3 and earlier revisions (24KB).
    pub const SIZE: usize = 24 * 1024;

    /// LCPU code start address.
    pub const CODE_START: usize = Self::BASE;
}

/// IPC mailbox buffer layout (HCPU view).
#[derive(Debug, Clone, Copy)]
pub struct IpcRegion;

impl IpcRegion {
    /// Mailbox buffer size for CH1 (bytes).
    pub const BUF_SIZE: usize = 512;

    /// HCPU -> LCPU (CH1) TX buffer start, HCPU view.
    pub const HCPU_TO_LCPU_CH1: usize = super::memory_map::shared::HCPU2LCPU_MB_CH1;
    /// HCPU -> LCPU (CH2) TX buffer start, HCPU view.
    pub const HCPU_TO_LCPU_CH2: usize = super::memory_map::shared::HCPU2LCPU_MB_CH2;

    /// LCPU -> HCPU (CH1) RX buffer start, HCPU view, Rev A/A3.
    pub const LCPU_TO_HCPU_CH1_A3: usize = super::memory_map::a3::LCPU2HCPU_CH1;

    /// LCPU -> HCPU (CH1) RX buffer start, HCPU view, Rev B/Letter.
    pub const LCPU_TO_HCPU_CH1_REV_B: usize = super::memory_map::letter::LCPU2HCPU_CH1;

    /// LCPU -> HCPU (CH2) RX buffer start, HCPU view, Rev A/A3.
    pub const LCPU_TO_HCPU_CH2_A3: usize = super::memory_map::a3::LCPU2HCPU_CH2;

    /// LCPU -> HCPU (CH2) RX buffer start, HCPU view, Rev B/Letter.
    pub const LCPU_TO_HCPU_CH2_REV_B: usize = super::memory_map::letter::LCPU2HCPU_CH2;

    /// HCPU SRAM -> LCPU alias offset (for sharing TX buffer).
    pub const HCPU_TO_LCPU_OFFSET: usize = super::memory_map::shared::HCPU_TO_LCPU_OFFSET;

    /// Convert HCPU SRAM address to LCPU view.
    #[inline]
    pub const fn hcpu_to_lcpu_addr(addr: usize) -> usize {
        addr + Self::HCPU_TO_LCPU_OFFSET
    }

    /// Select LCPU -> HCPU buffer start by revision.
    #[inline]
    pub fn lcpu_to_hcpu_start() -> usize {
        if syscfg::read_idr().revision().is_letter_series() {
            Self::LCPU_TO_HCPU_CH1_REV_B
        } else {
            Self::LCPU_TO_HCPU_CH1_A3
        }
    }

    /// Select LCPU -> HCPU CH2 buffer start by revision.
    #[inline]
    pub fn lcpu_to_hcpu_ch2_start() -> usize {
        if syscfg::read_idr().revision().is_letter_series() {
            Self::LCPU_TO_HCPU_CH2_REV_B
        } else {
            Self::LCPU_TO_HCPU_CH2_A3
        }
    }
}

//=============================================================================
// Errors
//=============================================================================

/// LCPU memory operation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Image is empty.
    EmptyImage,
    /// Image size exceeds LPSYS RAM capacity.
    ImageTooLarge { size_bytes: usize, max_bytes: usize },
    /// Invalid chip revision.
    InvalidRevision { revid: u8 },
}

//=============================================================================
// Public Functions
//=============================================================================

/// Install LCPU firmware image.
///
/// Replaces `lcpu_img::install`.
pub fn img_install(image: &[u8]) -> Result<(), Error> {
    if image.is_empty() {
        return Err(Error::EmptyImage);
    }

    let revision = syscfg::read_idr().revision();
    if !revision.is_valid() {
        return Err(Error::InvalidRevision {
            revid: revision.revid(),
        });
    }

    // Only A3 or Earlier is required to load LCPU image
    if !revision.is_letter_series() {
        let size_bytes = image.len();
        if size_bytes > LpsysRam::SIZE {
            error!(
                "LCPU image too large: {} bytes (max {} bytes)",
                size_bytes,
                LpsysRam::SIZE
            );
            return Err(Error::ImageTooLarge {
                size_bytes,
                max_bytes: LpsysRam::SIZE,
            });
        }

        debug!("Installing LCPU image: {} bytes", size_bytes);

        unsafe {
            let dst = LpsysRam::CODE_START as *mut u8;
            ptr::copy_nonoverlapping(image.as_ptr(), dst, size_bytes);
        }

        debug!("LCPU image installed successfully");
    } else {
        debug!("Letter Series detected, skipping image install");
    }

    Ok(())
}
