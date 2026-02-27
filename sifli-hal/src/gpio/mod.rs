//! GPIO driver.
#![macro_use]
use core::convert::Infallible;
use core::future::Future;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};

use embassy_hal_internal::{impl_peripheral, into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use hpsys::HpsysPin;

use crate::interrupt::InterruptExt;
use crate::utils::BitIter64;
use crate::{interrupt, peripherals, Peripheral};

// TODO: move this const to _generated.rs
#[cfg(feature = "sf32lb52x")]
pub(crate) const PA_PIN_COUNT: usize = 44;

pub mod hpsys;

static PA_WAKERS: [AtomicWaker; PA_PIN_COUNT] = [const { AtomicWaker::new() }; PA_PIN_COUNT];

/// Represents a digital input or output level.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Level {
    /// Logical low.
    Low,
    /// Logical high.
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl core::ops::Not for Level {
    type Output = Self;

    fn not(self) -> Self::Output {
        (!bool::from(self)).into()
    }
}

/// Represents a pull setting for an input.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum Pull {
    /// No pull.
    None,
    /// Internal pull-up resistor.
    Up,
    /// Internal pull-down resistor.
    Down,
}

/// Drive strength of an output
#[derive(Debug, Eq, PartialEq)]
pub enum Drive {
    /// min drive, {ds1, ds0} = 0b00
    Drive0,
    /// {ds1, ds0} = 0b01
    Drive1,
    /// {ds1, ds0} = 0b10
    Drive2,
    /// {ds1, ds0} = 0b11
    Drive3,
}
/// Slew rate of an output
#[derive(Debug, Eq, PartialEq)]
pub enum SlewRate {
    /// Fast slew rate.
    Fast,
    /// Slow slew rate.
    Slow,
}

/// GPIO input driver.
pub struct Input<'d> {
    pin: Flex<'d>,
}

impl<'d> Input<'d> {
    /// Create GPIO input driver for a [Pin] with the provided [Pull] configuration.
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_pull(pull);
        pin.set_as_input();

        Self { pin }
    }

    /// Set the pin's Schmitt trigger.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        self.pin.set_schmitt(enable)
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

/// Interrupt trigger levels.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptTrigger {
    /// Trigger on pin low.
    LevelLow,
    /// Trigger on pin high.
    LevelHigh,
    /// Trigger on high to low transition.
    EdgeLow,
    /// Trigger on low to high transition.
    EdgeHigh,
    /// Trigger on any transition.
    AnyEdge,
}

pub(crate) unsafe fn init(gpio1_it_priority: interrupt::Priority) {
    unsafe {
        interrupt::GPIO1.disable();
        interrupt::GPIO1.set_priority(gpio1_it_priority);
        interrupt::GPIO1.enable();
    }
    crate::rcc::enable::<peripherals::HPSYS_GPIO>();

    // We should not reset PINMUX here, otherwise the pins used for PSRAM
    // or FLASH will be invalid.
    // PINMUX is already turned on in the bootloader.
}

#[cfg(feature = "rt")]
fn irq_handler<const N: usize>(wakers: &[AtomicWaker; N]) {
    let gpio = crate::pac::HPSYS_GPIO;

    // Read both interrupt status registers
    let isr0 = gpio.isr0().read().0;
    let isr1 = gpio.isr1().read().0;
    // trace!("gpio irq: isr0={:x} isr1={:x}", isr0, isr1);

    // Combine into a single 64-bit status for easier iteration
    let status = (isr1 as u64) << 32 | isr0 as u64;

    if status == 0 {
        return;
    }

    for pin_idx in BitIter64(status) {
        let mut pin = HpsysPin::new(pin_idx);

        // Disable the interrupt for this pin to prevent re-firing.
        // The future will re-enable it on the next await.
        pin.disable_interrupt();

        // Clear the interrupt flag.
        pin.clear_interrupt();

        // TODO: The C HAL implementation (`HAL_GPIO_EXTI_IRQHandler`) includes logic
        // to clear the corresponding AON (Always-On) Wakeup Source Register (WSR)
        // if a GPIO pin is also configured as a wakeup pin. This is important for
        // low-power sleep/wakeup functionality.
        // This implementation omits that step as it requires a mapping from GPIO
        // pins to AON wakeup pins and access to AON registers, which is beyond
        // the scope of a plain GPIO interrupt handler. If you use GPIO interrupts
        // to wake the system from sleep, you may need to add this logic manually
        // by calling `HAL_HPAON_CLEAR_WSR` or its Rust equivalent.

        wakers[pin_idx as usize].wake();
    }
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO1() {
    irq_handler(&PA_WAKERS);
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct InputFuture<'d> {
    pin: PeripheralRef<'d, AnyPin>,
}

impl<'d> InputFuture<'d> {
    fn new(pin: PeripheralRef<'d, AnyPin>, trigger: InterruptTrigger) -> Self {
        let mut hpsys_pin = HpsysPin::new(pin.pin_bank());
        // Configure the hardware for the desired interrupt.
        hpsys_pin.set_interrupt_trigger(trigger);
        // Clear any pending interrupt flags before we start waiting.
        hpsys_pin.clear_interrupt();
        // Enable the interrupt.
        hpsys_pin.enable_interrupt();

        //
        hpsys_pin.set_high();
        Self { pin }
    }
}

impl<'d> Drop for InputFuture<'d> {
    fn drop(&mut self) {
        // When the future is dropped, disable the interrupt for the pin
        // to prevent stray interrupts.
        // let mut pin = HpsysPin::new(self.pin.pin_bank());
        // pin.disable_interrupt();
        // pin.clear_interrupt();
    }
}

impl<'d> Future for InputFuture<'d> {
    type Output = ();

    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let pin_idx = self.pin.pin() as usize;
        let hpsys_pin = HpsysPin::new(self.pin.pin_bank());

        // Register the waker.
        PA_WAKERS[pin_idx].register(cx.waker());

        // Check if the interrupt has already fired since we registered the waker.
        // The ISR will clear the IER bit, so if it's clear, we know it fired.
        if hpsys_pin.is_interrupt_disabled() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// GPIO output driver.
pub struct Output<'d> {
    pin: Flex<'d>,
}

impl<'d> Output<'d> {
    /// Create GPIO output driver for a [Pin] with the provided [Level].
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_level(initial_output);

        pin.set_as_output();
        Self { pin }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        self.pin.set_drive_strength(strength)
    }

    /// Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

/// GPIO output open-drain.
pub struct OutputOpenDrain<'d> {
    pin: Flex<'d>,
}

impl<'d> OutputOpenDrain<'d> {
    /// Create GPIO output driver for a [Pin] in open drain mode with the provided [Level].
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        // SET_OPEN_DRAIN_FLAG
        pin.set_low();

        pin.set_as_output_od();

        pin.set_level(initial_output);
        Self { pin }
    }

    /// Set the pin's pull-up.
    #[inline]
    pub fn set_pullup(&mut self, enable: bool) {
        if enable {
            self.pin.set_pull(Pull::Up);
        } else {
            self.pin.set_pull(Pull::None);
        }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        self.pin.set_drive_strength(strength)
    }

    /// Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        // For Open Drain High, disable the output pin.
        self.pin.set_as_input()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        // For Open Drain Low, enable the output pin.
        self.pin.set_as_output()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the output level low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_as_output()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    /// Get whether the pin input level is high.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Get whether the pin input level is low.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

pub struct Analog<'d> {
    _pin: Flex<'d>,
}

impl<'d> Analog<'d> {
    /// Create GPIO analog driver for a [Pin].
    #[inline]
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_analog();
        Self { _pin: pin }
    }
}

/// GPIO flexible pin.
///
/// This pin can be either an input or output pin. The output level register bit will remain
/// set while not in output mode, so the pin's level will be 'remembered' when it is not in output
/// mode.
pub struct Flex<'d> {
    pub(crate) pin: PeripheralRef<'d, AnyPin>,
    inner: HpsysPin,
}

impl<'d> Flex<'d> {
    /// Create a new Flex pin
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        into_ref!(pin);
        let mut flex = Self {
            inner: HpsysPin::new(pin.pin_bank()),
            pin: pin.map_into(),
        };
        flex.disable_interrupt();
        unsafe { flex.inner.set_fsel_unchecked(0) };
        flex
    }

    pub fn new_without_init(pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        into_ref!(pin);
        Self {
            inner: HpsysPin::new(pin.pin_bank()),
            pin: pin.map_into(),
        }
    }

    pub fn disable_interrupt(&mut self) {
        self.inner.disable_interrupt();
        self.inner.clear_interrupt();
    }

    /// Configure pin as output
    pub fn set_as_output(&mut self) {
        self.inner.set_as_output();
    }

    /// Configure pin as input
    pub fn set_as_input(&mut self) {
        self.inner.set_as_input();
    }

    /// Configure pin as open drain output
    pub fn set_as_output_od(&mut self) {
        self.inner.set_as_output_od();
    }

    /// Configure pin as analog mode
    pub fn set_as_analog(&mut self) {
        self.inner.set_as_analog();
    }

    /// Returns whether pin is configured as output
    pub fn is_set_as_output(&self) -> bool {
        self.inner.is_set_as_output()
    }

    /// Set pin level high
    pub fn set_high(&mut self) {
        self.inner.set_high();
    }

    /// Set pin level low
    pub fn set_low(&mut self) {
        self.inner.set_low();
    }

    /// Toggle pin level
    pub fn toggle(&mut self) {
        self.inner.toggle();
    }

    /// Get current pin output level
    pub fn get_output_level(&self) -> Level {
        self.inner.get_output_level()
    }

    /// Get current pin level (either input or output depending on mode)
    pub fn get_level(&self) -> Level {
        self.inner.get_level()
    }

    /// Set pin level
    pub fn set_level(&mut self, level: Level) {
        self.inner.set_level(level)
    }

    /// Returns whether pin level is high
    pub fn is_high(&self) -> bool {
        self.inner.is_high()
    }

    /// Returns whether pin level is low
    pub fn is_low(&self) -> bool {
        self.inner.is_low()
    }

    /// Returns whether pin output level is high
    pub fn is_set_high(&self) -> bool {
        self.inner.is_set_high()
    }

    /// Returns whether pin output level is low
    pub fn is_set_low(&self) -> bool {
        self.inner.is_set_low()
    }

    /// Set pin pull resistor
    pub fn set_pull(&mut self, pull: Pull) {
        self.inner.set_pull(pull);
    }

    /// Set pin drive strength
    pub fn set_drive_strength(&mut self, drive: Drive) {
        self.inner.set_drive_strength(drive);
    }

    /// Set pin slew rate
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.inner.set_slew_rate(slew_rate);
    }

    /// Enable schmitt input
    pub fn set_schmitt(&mut self, enable: bool) {
        self.inner.set_schmitt(enable);
    }

    /// Set pin function select and attributes
    pub fn set_function(&mut self, fsel: u8, af_type: crate::gpio::AfType) {
        self.inner.set_function(fsel, af_type);
    }

    /// Wait until the pin is high. If it is already high, return immediately.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        if self.is_high() {
            return;
        }
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelHigh).await;
    }

    /// Wait until the pin is low. If it is already low, return immediately.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        if self.is_low() {
            return;
        }
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelLow).await;
    }

    /// Wait for the pin to undergo a transition from low to high.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeHigh).await;
    }

    /// Wait for the pin to undergo a transition from high to low.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeLow).await;
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high to low.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::AnyEdge).await;
    }
}

impl<'d> Drop for Flex<'d> {
    #[inline]
    fn drop(&mut self) {
        // TODO
        self.pin.set_as_disconnected();
    }
}

pub struct AfType {
    pub(crate) pull: Pull,
}

impl AfType {
    pub(crate) fn new(pull: Pull) -> Self {
        Self { pull }
    }
}

pub(crate) trait SealedPin: Sized {
    fn pin_bank(&self) -> u8;

    #[inline]
    fn _pin(&self) -> u8 {
        self.pin_bank() & 0x7f
    }

    #[inline]
    fn _bank(&self) -> u8 {
        self.pin_bank() >> 7
    }

    // fn gpio(&self) -> pac::hpsys_gpio::HpsysGpio {
    //     pac::HPSYS_GPIO
    // }

    // fn pinmux(&self) -> pac::hpsys_pinmux::HpsysPinmux {
    //     pac::HPSYS_PINMUX
    // }

    /// Set the pin as "disconnected", ie doing nothing and consuming the lowest
    /// amount of power possible.
    #[inline]
    fn set_as_disconnected(&self) {
        let mut pin = HpsysPin::new(self.pin_bank());
        pin.disable_interrupt();
        pin.clear_interrupt();
        pin.set_ipr(false, false);
        unsafe { pin.set_fsel_unchecked(0) };
        // pin.set_ie(false);
        pin.set_drive_strength(Drive::Drive0);
        pin.set_pull(Pull::None);
    }

    #[inline]
    fn set_function(&self, fsel: u8, af_type: AfType) {
        let mut pin = HpsysPin::new(self.pin_bank());
        pin.set_function(fsel, af_type);
    }
}

/// Interface for a Pin that can be configured by an [Input] or [Output] driver, or converted to an [AnyPin].
#[allow(private_bounds)]
pub trait Pin: Peripheral<P = Self> + Into<AnyPin> + SealedPin + Sized + 'static {
    /// Degrade to a generic pin struct
    fn degrade(self) -> AnyPin {
        AnyPin {
            pin_bank: self.pin_bank(),
        }
    }

    /// Returns the pin number within a bank
    #[inline]
    fn pin(&self) -> u8 {
        self._pin()
    }

    /// Returns the bank of this pin (PA=0, PB=1)
    #[inline]
    fn bank(&self) -> u8 {
        self._bank()
    }
}

/// Type-erased GPIO pin
pub struct AnyPin {
    pin_bank: u8,
}

impl AnyPin {
    /// Unsafely create a new type-erased pin.
    ///
    /// # Safety
    ///
    /// You must ensure that you’re only using one instance of this type at a time.
    pub unsafe fn steal(pin_bank: u8) -> Self {
        Self { pin_bank }
    }
}

impl_peripheral!(AnyPin);

impl Pin for AnyPin {}
impl SealedPin for AnyPin {
    fn pin_bank(&self) -> u8 {
        self.pin_bank
    }
}

// ==========================

macro_rules! impl_pin {
    ($name:ident, $bank:expr, $pin_num:expr) => {
        impl Pin for peripherals::$name {}
        impl SealedPin for peripherals::$name {
            #[inline]
            fn pin_bank(&self) -> u8 {
                ($bank as u8) * 128 + $pin_num
            }
        }

        impl From<peripherals::$name> for crate::gpio::AnyPin {
            fn from(val: peripherals::$name) -> Self {
                crate::gpio::Pin::degrade(val)
            }
        }
    };
}

// ====================

mod eh02 {
    use super::*;

    impl<'d> embedded_hal_02::digital::v2::InputPin for Input<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for Output<'d> {
        type Error = Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for Output<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for Output<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::InputPin for OutputOpenDrain<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for OutputOpenDrain<'d> {
        type Error = Infallible;

        #[inline]
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }

        #[inline]
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for OutputOpenDrain<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for OutputOpenDrain<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::InputPin for Flex<'d> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_high())
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::OutputPin for Flex<'d> {
        type Error = Infallible;

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.set_high();
            Ok(())
        }

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.set_low();
            Ok(())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::StatefulOutputPin for Flex<'d> {
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_high())
        }

        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(self.is_set_low())
        }
    }

    impl<'d> embedded_hal_02::digital::v2::ToggleableOutputPin for Flex<'d> {
        type Error = Infallible;
        #[inline]
        fn toggle(&mut self) -> Result<(), Self::Error> {
            self.toggle();
            Ok(())
        }
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Input<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Input<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::OutputPin for Output<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Output<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for OutputOpenDrain<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::OutputPin for OutputOpenDrain<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for OutputOpenDrain<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_1::digital::InputPin for OutputOpenDrain<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::ErrorType for Flex<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal_1::digital::InputPin for Flex<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_low())
    }
}

impl<'d> embedded_hal_1::digital::OutputPin for Flex<'d> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<'d> embedded_hal_1::digital::StatefulOutputPin for Flex<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok((*self).is_set_low())
    }
}

impl<'d> embedded_hal_async::digital::Wait for Flex<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal_async::digital::Wait for Input<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

impl<'d> embedded_hal_async::digital::Wait for OutputOpenDrain<'d> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}
