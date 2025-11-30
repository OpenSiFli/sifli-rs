//! LCPU memory management: ROM configuration and firmware image loading.
//!
//! This module unifies LCPU memory operations, including configuring ROM parameters
//! and loading firmware images into LPSYS RAM.

use crate::syscfg::ChipRevision;
use core::{mem, ptr};

//=============================================================================
// ROM Configuration Layout
//=============================================================================

/// LCPU ROM Configuration Block Layout.
///
/// Maps to the memory structure expected by LCPU ROM.
/// Offsets and fields align with SiFli-SDK `lcpu_config_type_int.h`.
#[repr(C)]
pub struct RomControlBlock {
    /// Magic number (0x45457878).
    pub magic: u32, // 0x00
    pub _pad0: [u8; 8], // 0x04..0x0C

    /// Watchdog timeout configuration.
    pub wdt_time: u32, // 0x0C (12)
    pub wdt_status: u32, // 0x10 (16)
    pub _pad1: [u8; 4],  // 0x14..0x18
    pub wdt_clk: u16,    // 0x18 (24)

    /// Clock configuration.
    pub is_xtal_enable: u8, // 0x1A (26)
    pub is_rccal_in_l: u8, // 0x1B (27)

    // Padding to reach 0xAC (172).
    // 28 (0x1C) to 172 (0xAC) = 144 bytes.
    pub _pad2: [u8; 144], // 0x1C..0xAC (28..172)

    /// Letter Series (A4/B4) extended configuration (BT/BLE).
    pub bt_config: BtRomConfig, // 0xAC (172)

    // Padding to reach 0xC8 (200).
    // BtRomConfig size is 21 bytes.
    // 172 + 21 = 193.
    // 200 - 193 = 7 bytes.
    pub _pad3: [u8; 7],

    /// HCPU to LCPU IPC address (Letter Series only).
    pub hcpu_ipc_addr: u32, // 0xC8 (200)
}

/// BT/BLE specific configuration (A4+).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BtRomConfig {
    pub bit_valid: u32,
    pub max_sleep_time: u32,
    pub controller_enable_bit: u8,
    pub lld_prog_delay: u8,
    pub lld_prog_delay_min: u8,
    pub default_sleep_mode: u8,
    pub default_sleep_enabled: u8,
    pub default_xtal_enabled: u8,
    pub default_rc_cycle: u8,
    pub default_swprofiling_cfg: u8,
    pub boot_mode: u8,
    pub is_fpga: u8,
    pub en_inq_filter: u8,
    pub support_3m: u8,
    pub sco_cfg: u8,
}

impl Default for BtRomConfig {
    fn default() -> Self {
        // Zeroed by default
        unsafe { mem::zeroed() }
    }
}

//=============================================================================
// User Configuration
//=============================================================================

/// User-configurable ROM parameters.
#[derive(Debug, Clone, Copy)]
pub struct RomConfig {
    /// Watchdog timeout (in seconds, default 10).
    pub wdt_time: u32,
    /// Watchdog clock frequency (Hz, default 32768).
    pub wdt_clk: u16,
    /// Enable external low-speed crystal (default true).
    pub enable_lxt: bool,
}

impl Default for RomConfig {
    fn default() -> Self {
        Self {
            wdt_time: 10,
            wdt_clk: 32_768,
            enable_lxt: true,
        }
    }
}

//=============================================================================
// Memory Map Constants
//=============================================================================

impl RomControlBlock {
    /// Base address for A3 and earlier (fixed region).
    pub const ADDR_A3: usize = 0x2040_FDC0;

    /// Base address for Letter Series (A4/B4) (Mailbox CH2).
    pub const ADDR_LETTER: usize = 0x2040_2A00;

    /// HCPU->LCPU Mailbox CH1 buffer start (TX queue).
    pub const HCPU2LCPU_MB_CH1_BUF_START_ADDR: usize = 0x2007_FE00;

    /// Magic number expected by ROM.
    pub const MAGIC: u32 = 0x4545_7878;

    /// Get the configuration base address for the given chip revision.
    pub fn address(revision: ChipRevision) -> usize {
        if revision.is_letter_series() {
            Self::ADDR_LETTER
        } else {
            Self::ADDR_A3
        }
    }
}

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
    pub const A3_CODE_START: usize = 0x2040_6000;

    /// Patch record area address for A3.
    /// Located at the last 256 bytes of the patch region.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:331` (`LCPU_PATCH_RECORD_ADDR`)
    pub const A3_RECORD_ADDR: usize = 0x2040_7F00;

    /// Total patch area size for A3.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:300`
    pub const A3_TOTAL_SIZE: usize = 8 * 1024;

    // ===== Letter Series (A4/B4) =====

    /// Patch buffer start address.
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:334`
    pub const LETTER_BUF_START: usize = 0x2040_5000;

    /// Patch code start address (after 12-byte header).
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:335`
    pub const LETTER_CODE_START: usize = 0x2040_500C;

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
    pub const BASE: usize = 0x2040_0000;

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
    pub const HCPU_TO_LCPU_CH1: usize = RomControlBlock::HCPU2LCPU_MB_CH1_BUF_START_ADDR;
    /// HCPU -> LCPU (CH2) TX buffer start, HCPU view.
    pub const HCPU_TO_LCPU_CH2: usize = 0x2007_FC00;

    /// LCPU -> HCPU (CH1) RX buffer start, HCPU view, Rev A/A3.
    pub const LCPU_TO_HCPU_CH1_A3: usize = 0x2040_5C00;

    /// LCPU -> HCPU (CH1) RX buffer start, HCPU view, Rev B/Letter.
    pub const LCPU_TO_HCPU_CH1_REV_B: usize = 0x2040_2800;

    /// LCPU -> HCPU (CH2) RX buffer start, HCPU view, Rev A/A3.
    pub const LCPU_TO_HCPU_CH2_A3: usize = 0x2040_5E00;

    /// LCPU -> HCPU (CH2) RX buffer start, HCPU view, Rev B/Letter.
    pub const LCPU_TO_HCPU_CH2_REV_B: usize = 0x2040_2A00;

    /// HCPU SRAM -> LCPU alias offset (for sharing TX buffer).
    pub const HCPU_TO_LCPU_OFFSET: usize = 0x0A00_0000;

    /// Convert HCPU SRAM address to LCPU view.
    #[inline]
    pub const fn hcpu_to_lcpu_addr(addr: usize) -> usize {
        addr + Self::HCPU_TO_LCPU_OFFSET
    }

    /// Select LCPU -> HCPU buffer start by revision.
    #[inline]
    pub fn lcpu_to_hcpu_start(rev: ChipRevision) -> usize {
        if rev.is_letter_series() {
            Self::LCPU_TO_HCPU_CH1_REV_B
        } else {
            Self::LCPU_TO_HCPU_CH1_A3
        }
    }

    /// Select LCPU -> HCPU CH2 buffer start by revision.
    #[inline]
    pub fn lcpu_to_hcpu_ch2_start(rev: ChipRevision) -> usize {
        if rev.is_letter_series() {
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

/// Configure LCPU ROM parameters.
///
/// Replaces `lcpu_rom_config`.
    pub fn rom_config(revision: ChipRevision, config: &RomConfig) -> Result<(), Error> {
        let base = RomControlBlock::address(revision);
        let is_letter = revision.is_letter_series();

        // Calculate size to clear/write.
        // A3: 0x40 (64 bytes)
        // Letter: 0xCC (204 bytes) -> sizeof(RomControlBlock) is 204
        let size = if is_letter {
        mem::size_of::<RomControlBlock>()
    } else {
        0x40 // LCPU_CONFIG_ROM_SIZE
    };

    debug!(
        "Initializing LCPU ROM config: base=0x{:08X}, size={} (Letter Series: {})",
        base, size, is_letter
    );

    unsafe {
        // 1. Clear config area.
        ptr::write_bytes(base as *mut u8, 0, size);

        // 2. Map structure to memory.
        let block = &mut *(base as *mut RomControlBlock);

        // 3. Write common fields.
        ptr::write_volatile(&mut block.magic, RomControlBlock::MAGIC);
        ptr::write_volatile(&mut block.is_xtal_enable, config.enable_lxt as u8);
        ptr::write_volatile(&mut block.is_rccal_in_l, (!config.enable_lxt) as u8);
        ptr::write_volatile(&mut block.wdt_status, 0xFF); // Enable WDT
        ptr::write_volatile(&mut block.wdt_time, config.wdt_time);
        ptr::write_volatile(&mut block.wdt_clk, config.wdt_clk);

        // 4. Write Letter Series specific fields.
        if is_letter {
            // HCPU IPC Address
            ptr::write_volatile(
                &mut block.hcpu_ipc_addr,
                RomControlBlock::HCPU2LCPU_MB_CH1_BUF_START_ADDR as u32,
            );

            // BT Config
            let bt_cfg = BtRomConfig {
                bit_valid: (1 << 10) | (1 << 6), // From SDK
                is_fpga: 0,
                default_xtal_enabled: config.enable_lxt as u8,
                ..Default::default()
            };

            // Write BT config struct
            ptr::write_volatile(&mut block.bt_config, bt_cfg);
        }
    }

    Ok(())
}

/// Install LCPU firmware image.
///
/// Replaces `lcpu_img::install`.
pub fn img_install(revision: ChipRevision, image: &[u8]) -> Result<(), Error> {
    if image.is_empty() {
        return Err(Error::EmptyImage);
    }

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

/// Write BT TX power parameters to LCPU ROM configuration area.
///
/// Corresponds to `HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_TX_PWR, ...)` in SDK,
/// writes to `bt_txpwr` field at offset 20.
/// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_config_type_int.h`.
pub fn set_bt_tx_power(revision: ChipRevision, tx_pwr: u32) {
    // LCPU_CONFIG_BT_TXPWR_ROM_OFFSET = 20
    const BT_TXPWR_OFFSET: usize = 20;

    let base = RomControlBlock::address(revision);
    let addr = base + BT_TXPWR_OFFSET;

    unsafe {
        ptr::write_volatile(addr as *mut u32, tx_pwr);
    }
}
