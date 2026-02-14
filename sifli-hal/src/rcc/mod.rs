use critical_section::CriticalSection;

mod clock;
pub use clock::*;

mod clock_read;
pub(crate) use clock_read::*;

mod clock_config;
pub use clock_config::*;

mod clock_lpsys;
pub use clock_lpsys::*;

mod token;
pub use token::*;

// Re-export PAC RCC enums
pub use crate::pac::hpsys_rcc::vals;
pub use crate::pac::lpsys_rcc::vals as lpsys_vals;

use crate::time::Hertz;


// TODO: should we split this into `RccEnable` and `RccReset` ?
pub(crate) trait SealedRccEnableReset {
    fn rcc_enable() {}

    fn rcc_disable() {}

    fn rcc_reset() {}
}
#[allow(private_bounds)]
pub trait RccEnableReset: SealedRccEnableReset + 'static {}

pub(crate) trait SealedRccGetFreq {
    /// Get peripheral frequency
    /// Returns `None` if clock is disabled
    fn get_freq() -> Option<Hertz>;
}

#[allow(private_bounds)]
pub trait RccGetFreq: SealedRccGetFreq + 'static {
    /// The clock token type this peripheral depends on.
    type Clock;
    /// Get peripheral frequency
    /// Returns `None` if clock is disabled
    fn frequency() -> Option<Hertz> {
        Self::get_freq()
    }
}

/// Enables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable_with_cs<T: RccEnableReset>(_cs: CriticalSection) {
    T::rcc_enable();
}

/// Enables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable<T: RccEnableReset>() {
    critical_section::with(|cs| enable_with_cs::<T>(cs));
}

/// Enables and resets peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable_and_reset_with_cs<T: RccEnableReset>(_cs: CriticalSection) {
    T::rcc_enable();
    T::rcc_reset();
}


/// Enables and resets peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn enable_and_reset<T: RccEnableReset>() {
    critical_section::with(|cs| enable_and_reset_with_cs::<T>(cs));
}

/// Disables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn disable_with_cs<T: RccEnableReset>(_cs: CriticalSection) {
    T::rcc_disable();
}

/// Disables peripheral `T`.
///
/// # Safety
///
/// Peripheral must not be in use.
// TODO: should this be `unsafe`?
pub fn disable<T: RccEnableReset>() {
    critical_section::with(|cs| disable_with_cs::<T>(cs));
}


pub fn test_print_clocks() {
    let clocks = clocks();
    info!("Clock frequencies: {:#?}", clocks);
}