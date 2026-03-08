//! Type-safe accessor for BT_RFC internal SRAM.
//!
//! The BLE MAC controller reads per-channel VCO parameters and per-power-level
//! TXDC parameters from calibration tables stored in RFC SRAM. This module
//! provides typed accessors with bounds checking, replacing raw pointer
//! arithmetic scattered across calibration code.
//!
//! All addresses and sizes originate from `sram_layout.toml` via code generation.

use sifli_hal::ram::memory_map::rf;
use sifli_hal::ram::RamSlice;

/// RFC SRAM accessor.
///
/// Wraps the hardware base address and provides typed table allocation
/// with bounds checking against the known SRAM size.
///
/// # Usage
///
/// ```ignore
/// let sram = RfcSram::new();
/// let (ble_rx, next) = sram.alloc_table(cmd_end_addr, 40);
/// ble_rx.write(0, some_value);
/// ```
///
/// # Safety contract
///
/// - Caller must ensure exclusive access to RFC SRAM (no concurrent DMA/MAC
///   access to the same region).
/// - BT_RFC clock must be enabled before construction.
pub struct RfcSram {
    region: RamSlice,
}

impl RfcSram {
    /// Create a new RFC SRAM accessor.
    #[inline]
    pub fn new() -> Self {
        Self {
            region: RamSlice::new(rf::BT_RFC_MEM_BASE as usize, rf::BT_RFC_SRAM_SIZE as usize),
        }
    }

    /// Raw base address (for backward compatibility).
    #[inline]
    pub const fn base(&self) -> u32 {
        rf::BT_RFC_MEM_BASE
    }

    /// Access the underlying `RamSlice`.
    #[inline]
    pub fn region(&self) -> &RamSlice {
        &self.region
    }

    /// Allocate a table region starting at `offset` words-aligned with
    /// `word_count` u32 entries.
    ///
    /// Returns the table handle and the next free byte offset.
    ///
    /// # Panics
    ///
    /// Panics if the table would exceed RFC SRAM bounds.
    pub fn alloc_table(&self, offset: u32, word_count: usize) -> (RfcTable, u32) {
        let byte_len = (word_count as u32) * 4;
        let sub = self.region.slice(offset as usize, byte_len as usize);
        (RfcTable { region: sub }, offset + byte_len)
    }
}

/// A typed table region within RFC SRAM.
///
/// Provides indexed write access with bounds checking. Each entry is one `u32`
/// word. The table's absolute address can be programmed into `CAL_ADDR_REG`
/// registers via [`offset()`](RfcTable::offset).
pub struct RfcTable {
    region: RamSlice,
}

impl RfcTable {
    /// SRAM offset of this table relative to RFC SRAM base.
    ///
    /// Use this for programming `CAL_ADDR_REG1/2/3` and `CU_ADDR_REG` fields,
    /// which store offsets (not absolute addresses).
    #[inline]
    pub fn offset(&self) -> u16 {
        (self.region.addr() - rf::BT_RFC_MEM_BASE as usize) as u16
    }

    /// Absolute address of this table in the peripheral bus address space.
    #[inline]
    pub fn addr(&self) -> u32 {
        self.region.addr() as u32
    }

    /// Number of u32 entries in this table.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.region.len() / 4
    }

    /// Write a single u32 word at the given index.
    ///
    /// # Panics
    ///
    /// Panics if `index >= capacity`.
    #[inline]
    pub fn write(&self, index: usize, value: u32) {
        self.region.write(index * 4, value);
    }

    /// Read a single u32 word at the given index.
    ///
    /// # Panics
    ///
    /// Panics if `index >= capacity`.
    #[inline]
    pub fn read(&self, index: usize) -> u32 {
        self.region.read(index * 4)
    }

    /// Write a pair of u32 words at positions `index * 2` and `index * 2 + 1`.
    ///
    /// Used for TXDC tables where each power level occupies 2 words.
    ///
    /// # Panics
    ///
    /// Panics if `index * 2 + 1 >= capacity`.
    #[inline]
    pub fn write_pair(&self, index: usize, word1: u32, word2: u32) {
        self.region.write(index * 8, word1);
        self.region.write(index * 8 + 4, word2);
    }
}
