//! Chip-wide RAM accessors and memory map constants.
//!
//! - [`RamSlice`]: bounds-checked volatile memory accessor
//! - [`memory_map`]: address constants generated from `sram_layout.toml`
pub mod memory_map;
pub mod ram_slice;
pub use ram_slice::RamSlice;
