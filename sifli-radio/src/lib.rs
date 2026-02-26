#![cfg_attr(not(test), no_std)]
#![doc = "Bluetooth radio stack for SiFli MCUs."]
#![doc = ""]
#![doc = "Provides BLE controller initialization, HCI transport, and RF calibration."]
#![doc = "Built on top of `sifli-hal` for hardware access."]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

pub mod bluetooth;

// Internal convenience re-exports so moved code can keep `crate::` paths.
pub(crate) use sifli_hal::dma;
pub(crate) use sifli_hal::efuse;
pub(crate) use sifli_hal::rcc;
pub(crate) use sifli_hal::Peripheral;
pub(crate) use sifli_hal::cortex_m_blocking_delay_us;
pub(crate) use sifli_hal::interrupt;
pub(crate) use sifli_hal::peripherals;

// PAC re-export (using sifli-pac directly since sifli-hal may not expose it)
pub(crate) use sifli_pac as pac;

// Memory map: re-exports sifli-hal SRAM layout + radio-specific ROM config offsets.
pub(crate) mod memory_map;
