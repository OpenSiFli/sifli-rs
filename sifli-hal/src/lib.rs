#![no_std]
#![doc = include_str!("../README.md")]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

#[cfg(feature = "set-msplim")]
use core::arch::global_asm;

mod macros;

mod utils;

pub mod rcc;
pub mod gpio;
pub mod timer;
pub mod time;
pub mod pmu;
#[allow(clippy::all)] // modified from embassy-stm32
pub mod usart;
pub mod adc;
pub mod lcdc;
#[allow(clippy::all)] // modified from embassy-stm32
pub mod dma;
pub mod mailbox;
#[cfg(feature = "usb")]
pub mod usb;
#[cfg(feature = "_time-driver")]
pub mod time_driver;

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
    use crate::rcc;
    use crate::interrupt;

    /// HAL configuration passed when initializing.
    #[non_exhaustive]
    pub struct Config {
        pub rcc: rcc::Config,
        pub gpio1_it_priority: interrupt::Priority,
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                rcc: rcc::Config::new_keep(),
                gpio1_it_priority: interrupt::Priority::P3,
            }
        }
    }
}
pub use config::Config;

/// Initialize the `sifli-hal` with the provided configuration.
///
/// This returns the peripheral singletons that can be used for creating drivers.
///
/// This should only be called once at startup, otherwise it panics.
pub fn init(config: Config) -> Peripherals {
    system_init();

    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let p = Peripherals::take();

    unsafe {
        config.rcc.apply();

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
        let mut cp = cortex_m::Peripherals::steal();

        // enable CP0/CP1/CP2 Full Access
        cp.SCB.cpacr.modify(|r| {
            r | (0b111111)
        });

        // Enable Cache
        cp.SCB.enable_icache();
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