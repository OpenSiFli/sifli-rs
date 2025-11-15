//! LCPU image installation support.
//!
//! 对于早期 A3 以及之前版本的芯片，需要在启动时向 LPSYS RAM 写入 LCPU 固件；
//! Letter Series (A4/B4) 则可直接运行 ROM 中的镜像。

use core::ptr;

use crate::syscfg::Idr;

/// LPSYS RAM base address (HCPU view)
pub const LPSYS_RAM_BASE: usize = 0x2040_0000;

/// LCPU code start address for SF32LB52x
pub const LCPU_CODE_START_ADDR: usize = LPSYS_RAM_BASE;

/// LPSYS RAM size for A3 and earlier revisions (24KB)
/// Reference: SiFli-SDK `mem_map.h` `LPSYS_RAM_SIZE`
pub const LPSYS_RAM_SIZE: usize = 24 * 1024;

/// LCPU image installation error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Image is empty
    EmptyImage,
    /// Image size exceeds LPSYS RAM capacity
    ImageTooLarge { size_bytes: usize, max_bytes: usize },
    /// Unknown or invalid chip revision
    InvalidRevision { revid: u8 },
}

/// Install LCPU firmware image
///
/// 自动检测芯片版本，仅在需要时安装镜像：
/// - A3 及更早 (REVID <= 0x03)：复制镜像至 LPSYS RAM
/// - Letter Series (A4/B4)：无需安装，直接从 ROM 运行
///
/// # Arguments
///
/// - `idr`: Chip identification from `SysCfg::read_idr()`
/// - `image_words`: LCPU firmware image as u32 words
#[inline]
pub fn install(idr: &Idr, image_words: &[u32]) -> Result<(), Error> {
    if image_words.is_empty() {
        return Err(Error::EmptyImage);
    }

    let revision = idr.revision();

    if !revision.is_valid() {
        let revid = idr.revid;

        warn!("Invalid chip revision: 0x{:02x}", revid);

        return Err(Error::InvalidRevision { revid });
    }

    // 仅 A3 及更早的版本需要写入 LCPU 镜像
    if !revision.is_letter_series() {
        let size_bytes = core::mem::size_of_val(image_words);
        if size_bytes > LPSYS_RAM_SIZE {
            error!(
                "LCPU image too large: {} bytes (max {} bytes)",
                size_bytes,
                LPSYS_RAM_SIZE
            );

            return Err(Error::ImageTooLarge {
                size_bytes,
                max_bytes: LPSYS_RAM_SIZE,
            });
        }

        debug!("Installing LCPU image: {} bytes", size_bytes);

        unsafe {
            install_image_unsafe(image_words);
        }

        debug!("LCPU image installed successfully");
    } else {
        debug!("Letter Series detected, skipping image install");
    }

    Ok(())
}

/// Internal unsafe function to copy image to LPSYS RAM
///
/// # Safety
///
/// 调用方需确保：
/// - `image_words` 不为空且不超过 `LPSYS_RAM_SIZE`
/// - LPSYS RAM（地址 `LCPU_CODE_START_ADDR`）可由当前内核访问且已正确映射
/// - 在复制期间，其他代码不会并发访问 LPSYS RAM
/// - LCPU 处于停止或复位状态，不会访问该内存区域
#[inline]
unsafe fn install_image_unsafe(image_words: &[u32]) {
    let dst = LCPU_CODE_START_ADDR as *mut u32;
    let len = image_words.len();

    // 使用 copy_nonoverlapping 高效复制 LCPU 镜像到 LPSYS RAM
    ptr::copy_nonoverlapping(image_words.as_ptr(), dst, len);
}
