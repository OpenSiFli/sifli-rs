//! LCPU memory management: ROM configuration write and firmware image loading.

use super::config::{ActConfig, EmConfig, RomConfig};
use crate::syscfg;
use core::ptr;

//=============================================================================
// ROM Configuration Layout
//=============================================================================

/// LCPU ROM Configuration Block Layout (common header only).
///
/// Covers the fields up to offset 28. Letter-series extensions
/// (ke_mem_config at 128, bt_rom_config at 172, hcpu_ipc_addr at 200) are
/// written at absolute offsets in `rom_config()` rather than via struct
/// fields — `repr(C)` alignment rules make them hard to lay out exactly
/// against `lcpu_config_type_int.h`.
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
}

/// BT/BLE specific configuration (A4+).
#[repr(C)]
#[derive(Default, Debug, Clone, Copy)]
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

//=============================================================================
// Memory Map Constants
//=============================================================================

impl RomControlBlock {
    /// Base address for A3 and earlier (fixed region).
    pub const ADDR_A3: usize = super::memory_map::a3::ROM_CONFIG_BASE;

    /// Base address for Letter Series (A4/B4) (Mailbox CH2).
    pub const ADDR_LETTER: usize = super::memory_map::letter::ROM_CONFIG_BASE;

    /// HCPU->LCPU Mailbox CH1 buffer start (TX queue).
    pub const HCPU2LCPU_MB_CH1_BUF_START_ADDR: usize = super::memory_map::shared::HCPU2LCPU_MB_CH1;

    /// Magic number expected by ROM.
    pub const MAGIC: u32 = 0x4545_7878;

    /// Get the configuration base address for the given chip revision.
    pub fn address() -> usize {
        if syscfg::read_idr().revision().is_letter_series() {
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
    pub const HCPU_TO_LCPU_CH1: usize = RomControlBlock::HCPU2LCPU_MB_CH1_BUF_START_ADDR;
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

/// Configure LCPU ROM parameters.
///
/// Replaces `lcpu_rom_config`. Uses absolute offsets for Letter-series fields
/// to match `lcpu_config_type_int.h` exactly — the `RomControlBlock` struct
/// layout under `repr(C)` does not match the SDK because `BtRomConfig` is
/// 24 bytes (u32 alignment) rather than 21.
pub fn rom_config(config: &RomConfig, ctrl: &super::config::ControllerConfig) -> Result<(), Error> {
    // SDK offsets from `lcpu_config_type_int.h`.
    const OFFSET_KE_MEM_CONFIG: usize = 128;
    const OFFSET_BT_ROM_CONFIG: usize = 172;
    const OFFSET_HCPU_IPC_ADDR: usize = 200;
    // Letter-series total config size (LCPU_CONFIG_ROM_A4_SIZE).
    const LCPU_CONFIG_ROM_A4_SIZE: usize = 0xCC;
    // Offset of `max_nb_of_hci_completed` within `hal_lcpu_ble_mem_config_t`:
    // bit_valid(4) + 6 pointers(24) + 6 u16 sizes(12) = 40.
    const OFFSET_MAX_NB_HCI_COMPLETED_IN_KE: usize = 40;

    let base = RomControlBlock::address();
    let is_letter = syscfg::read_idr().revision().is_letter_series();

    let size = if is_letter {
        LCPU_CONFIG_ROM_A4_SIZE
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

        // 2. Map structure to memory for the common header.
        let block = &mut *(base as *mut RomControlBlock);

        // 3. Write common fields.
        ptr::write_volatile(&mut block.magic, RomControlBlock::MAGIC);
        ptr::write_volatile(&mut block.is_xtal_enable, config.enable_lxt as u8);
        ptr::write_volatile(&mut block.is_rccal_in_l, (!config.enable_lxt) as u8);
        ptr::write_volatile(&mut block.wdt_status, 0xFF); // Enable WDT
        ptr::write_volatile(&mut block.wdt_time, config.wdt_time);
        ptr::write_volatile(&mut block.wdt_clk, config.wdt_clk);

        // 4. Write Letter Series specific fields at absolute offsets.
        if is_letter {
            // KE_BUF (ke_mem_config) — the SDK sets bit_valid = 1 << 6 and
            // max_nb_of_hci_completed = 6 via HAL_LCPU_CONFIG_BT_KE_BUF.
            // Without this the LCPU ROM may use an uninitialized or zero value
            // which can prevent HCI command-complete events from being sent.
            let ke_base = base + OFFSET_KE_MEM_CONFIG;
            ptr::write_volatile(ke_base as *mut u32, 1u32 << 6); // bit_valid
            ptr::write_volatile(
                (ke_base + OFFSET_MAX_NB_HCI_COMPLETED_IN_KE) as *mut i8,
                6,
            ); // max_nb_of_hci_completed

            // BT Config (bt_rom_config, 24 bytes at offset 172).
            // Must include sleep bits so ROM disables sleep — without
            // SLEEP_MODE/SLEEP_ENABLED in bit_valid, ROM uses internal defaults
            // that may enable sleep, causing 0x3E connection timeouts since we
            // lack ble_standby_sleep_after_handler.
            let bt_cfg = BtRomConfig {
                bit_valid: (1 << 10)  // is_fpga
                    | (1 << 7)        // rc_cycle
                    | (1 << 6)        // xtal_enabled
                    | (1 << 5)        // sleep_enabled
                    | (1 << 4)        // sleep_mode
                    | (1 << 2)        // lld_prog_delay
                    | (1 << 1), // controller_enable
                controller_enable_bit: 0x03, // BLE(1) | BT(2)
                lld_prog_delay: ctrl.lld_prog_delay,
                default_sleep_mode: 0,    // No sleep
                default_sleep_enabled: 0, // Disable sleep
                default_xtal_enabled: ctrl.xtal_enabled as u8,
                default_rc_cycle: ctrl.rc_cycle,
                is_fpga: 0,
                ..Default::default()
            };
            ptr::write_volatile((base + OFFSET_BT_ROM_CONFIG) as *mut BtRomConfig, bt_cfg);

            // HCPU IPC address at offset 200 (NOT via the struct field, which
            // sits at offset 204 due to u32 alignment of BtRomConfig).
            ptr::write_volatile(
                (base + OFFSET_HCPU_IPC_ADDR) as *mut u32,
                RomControlBlock::HCPU2LCPU_MB_CH1_BUF_START_ADDR as u32,
            );

            // EM buffer configuration
            if let Some(ref em) = config.em_config {
                let dst = (base + EmConfig::ROM_OFFSET) as *mut EmConfig;
                ptr::write_volatile(dst, *em);
            }

            // Activity configuration
            if let Some(ref act) = config.act_config {
                let dst = (base + ActConfig::ROM_OFFSET) as *mut ActConfig;
                ptr::write_volatile(dst, *act);
            }
        }
    }

    Ok(())
}

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

/// Write BT TX power parameters to LCPU ROM configuration area.
///
/// Corresponds to `HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_BT_TX_PWR, ...)` in SDK,
/// writes to `bt_txpwr` field at offset 20.
/// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_config_type_int.h`.
pub fn set_bt_tx_power(tx_pwr: u32) {
    // LCPU_CONFIG_BT_TXPWR_ROM_OFFSET = 20
    const BT_TXPWR_OFFSET: usize = 20;

    let base = RomControlBlock::address();
    let addr = base + BT_TXPWR_OFFSET;

    unsafe {
        ptr::write_volatile(addr as *mut u32, tx_pwr);
    }
}
