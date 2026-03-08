//! Memory map constants.
//!
//! Re-exports sifli-hal SRAM layout + radio-specific ROM config offsets.

pub use sifli_hal::ram::memory_map::a3;
pub use sifli_hal::ram::memory_map::a3_rom;
pub use sifli_hal::ram::memory_map::letter;
pub use sifli_hal::ram::memory_map::letter_rom;
pub use sifli_hal::ram::memory_map::rf;
pub use sifli_hal::ram::memory_map::shared;

// ROM config field offsets (generated from rom_config_layout.toml)
include!(concat!(env!("OUT_DIR"), "/rom_config_generated.rs"));
