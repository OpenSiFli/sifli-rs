#![cfg_attr(not(test), no_std)]
#![doc = include_str!("../README.md")]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

#[cfg(feature = "set-msplim")]
use core::arch::global_asm;

mod macros;

mod utils;

pub mod adc;
pub mod aud_pll;
pub mod audio;
#[allow(clippy::all)] // modified from embassy-stm32
pub mod dma;
pub mod efuse;
pub mod gpio;
pub mod i2c;
#[cfg(feature = "sf32lb52x")]
pub mod ipc;
pub mod lcdc;
pub mod lcpu;
pub(crate) mod lpaon;
pub mod ram;
pub mod mailbox;
pub mod patch;
pub mod pmu;
pub mod rcc;
pub mod rng;
pub mod syscfg;
pub mod time;
#[cfg(feature = "_time-driver")]
pub mod time_driver;
pub mod timer;
#[allow(clippy::all)] // modified from embassy-stm32
pub mod usart;
pub mod usb;

// Reexports
pub use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
#[cfg(feature = "unstable-pac")]
pub use sifli_pac as pac;
#[cfg(not(feature = "unstable-pac"))]
pub(crate) use sifli_pac as pac;

/// Operating modes for peripherals.
pub mod mode {
    trait SealedMode {}

    /// Operating mode for a peripheral.
    #[allow(private_bounds)]
    pub trait Mode: SealedMode {}

    macro_rules! impl_mode {
        ($name:ident) => {
            impl SealedMode for $name {}
            impl Mode for $name {}
        };
    }

    /// Blocking mode.
    pub struct Blocking;
    /// Async mode.
    pub struct Async;

    impl_mode!(Blocking);
    impl_mode!(Async);
}

/// HAL configuration for SiFli
pub mod config {
    use crate::interrupt;
    use crate::rcc;

    /// HAL configuration passed when initializing.
    #[non_exhaustive]
    pub struct Config {
        pub rcc: rcc::Config,
        pub gpio1_it_priority: interrupt::Priority,
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                rcc: const { rcc::ConfigBuilder::new().checked() },
                gpio1_it_priority: interrupt::Priority::P3,
            }
        }
    }

    impl Config {
        pub fn with_rcc(mut self, rcc: rcc::Config) -> Self {
            self.rcc = rcc;
            self
        }

        pub fn with_gpio1_it_priority(mut self, p: interrupt::Priority) -> Self {
            self.gpio1_it_priority = p;
            self
        }
    }
}
pub use config::Config;

#[cfg(target_arch = "arm")]
mod mpu;

/// Initialize the `sifli-hal` with the provided configuration.
///
/// Returns peripheral singletons. Clock frequencies can be queried
/// via [`rcc::clocks()`] after initialization.
///
/// This should only be called once at startup, otherwise it panics.
pub fn init(config: Config) -> Peripherals {
    system_init();

    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let p = Peripherals::take();

    unsafe {
        rcc::init(config.rcc);

        #[cfg(feature = "_time-driver")]
        time_driver::init();

        gpio::init(config.gpio1_it_priority);
        critical_section::with(|cs| {
            dma::init(cs);
        });
    }
    p
}

fn system_init() {
    unsafe {
        #[allow(unused_mut)] // mut needed on ARM for SCB cache ops
        let mut cp = cortex_m::Peripherals::steal();

        // enable CP0/CP1/CP2 Full Access
        cp.SCB.cpacr.modify(|r| r | (0b111111));

        // Consistent with SDK `mpu_config()`: invalidate stale I-cache before MPU/Cache configuration.
        #[cfg(target_arch = "arm")]
        cp.SCB.invalidate_icache();

        // Configure MPU to make cross-core shared SRAM non-cacheable (matching SDK behavior).
        #[cfg(target_arch = "arm")]
        mpu::init();

        // Enable Cache
        #[cfg(target_arch = "arm")]
        cp.SCB.enable_icache();
        #[cfg(target_arch = "arm")]
        cp.SCB.enable_dcache(&mut cp.CPUID);
    }
}

pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]
    #![allow(clippy::all)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

pub use _generated::interrupt;
pub use _generated::{peripherals, Peripherals};

/// Performs a busy-wait delay for a specified number of microseconds, using the `cortex-m::asm::delay` function.
pub fn cortex_m_blocking_delay_us(us: u32) {
    let freq = rcc::get_hclk_freq().unwrap().0 as u64;
    let cycles = freq * us as u64 / 1_000_000;
    cortex_m::asm::delay(cycles as u32);
}

/// Performs a busy-wait delay for a specified number of microseconds.
pub fn blocking_delay_us(us: u32) {
    #[cfg(feature = "time")]
    embassy_time::block_for(embassy_time::Duration::from_micros(us as u64));
    #[cfg(not(feature = "time"))]
    cortex_m_blocking_delay_us(us);
}

/// Converts a address to a System Bus address.
/// `HCPU_MPI_SBUS_ADDR``
pub fn to_system_bus_addr(addr: usize) -> usize {
    match addr {
        0x1000_0000..0x2000_0000 => addr + 0x5000_0000,
        _ => addr,
    }
}

/// Converts a address to a Code Bus address.
/// `HCPU_MPI_CBUS_ADDR``
pub fn to_code_bus_addr(addr: usize) -> usize {
    match addr {
        0x6000_0000..0x7000_0000 => addr - 0x5000_0000,
        _ => addr,
    }
}

/// Macro to bind interrupts to handlers.
///
/// This defines the right interrupt handlers, and creates a unit struct (like `struct Irqs;`)
/// and implements the right [`Binding`]s for it. You can pass this struct to drivers to
/// prove at compile-time that the right interrupts have been bound.
///
/// Example of how to bind one interrupt:
///
/// ```rust,ignore
/// use sifli_hal::{bind_interrupts, usb, peripherals};
///
/// bind_interrupts!(struct Irqs {
///     USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
/// });
/// ```
///
// developer note: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            $(#[cfg($cond_irq)])?
            unsafe extern "C" fn $irq() {
                $(
                    $(#[cfg($cond_handler)])?
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();

                )*
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
                )*
            );
        )*
    };
    (@inner $($t:tt)*) => {
        $($t)*
    }
}

// Check README.md for details
#[cfg(feature = "set-msplim")]
global_asm!(
    ".section .text._pre_init",
    ".global __pre_init",
    ".type __pre_init, %function",
    ".thumb_func",
    "__pre_init:",
    "    ldr r0, =_stack_end",
    "    msr MSPLIM, r0",
    "    bx lr",
    ".size __pre_init, . - __pre_init"
);
