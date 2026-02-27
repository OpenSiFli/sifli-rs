use embassy_hal_internal::Peripheral;
// use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt;
use crate::rcc::SealedRccEnableReset;

/// Timer channel.
#[derive(Clone, Copy)]
pub enum Channel {
    /// Channel 1.
    Ch1,
    /// Channel 2.
    Ch2,
    /// Channel 3.
    Ch3,
    /// Channel 4.
    Ch4,
    /// Channel 5, only available on ATIM.
    Ch5,
    /// Channel 6, only available on ATIM.
    Ch6,
}

impl Channel {
    /// Get the channel index (0..3)
    pub fn index(&self) -> usize {
        match self {
            Channel::Ch1 => 0,
            Channel::Ch2 => 1,
            Channel::Ch3 => 2,
            Channel::Ch4 => 3,
            Channel::Ch5 => 4,
            Channel::Ch6 => 5,
        }
    }
}

/// Channel 1 marker type.
pub enum Ch1 {}
/// Channel 2 marker type.
pub enum Ch2 {}
/// Channel 3 marker type.
pub enum Ch3 {}
/// Channel 4 marker type.
pub enum Ch4 {}
/// Channel 1 complementary marker type (for ATIM).
pub enum Ch1n {}
/// Channel 2 complementary marker type (for ATIM).
pub enum Ch2n {}
/// Channel 3 complementary marker type (for ATIM).
pub enum Ch3n {}

// ATIM advanced features (not all implemented yet)
/// External trigger marker type.
pub enum Etr {}
/// Break input marker type.
pub enum Bk {}
/// Break input 2 marker type.
pub enum Bk2 {}
/// Output marker type.
pub enum Out {}
/// Input marker type.
pub enum In {}

/// Timer channel marker trait.
#[allow(private_bounds)]
pub trait TimerChannel: sealed::TimerChannel {
    /// The runtime channel.
    const CHANNEL: Channel;
}

mod sealed {
    pub trait TimerChannel {}
}

impl TimerChannel for Ch1 {
    const CHANNEL: Channel = Channel::Ch1;
}
impl TimerChannel for Ch2 {
    const CHANNEL: Channel = Channel::Ch2;
}
impl TimerChannel for Ch3 {
    const CHANNEL: Channel = Channel::Ch3;
}
impl TimerChannel for Ch4 {
    const CHANNEL: Channel = Channel::Ch4;
}

impl sealed::TimerChannel for Ch1 {}
impl sealed::TimerChannel for Ch2 {}
impl sealed::TimerChannel for Ch3 {}
impl sealed::TimerChannel for Ch4 {}
impl sealed::TimerChannel for Ch1n {}
impl sealed::TimerChannel for Ch2n {}
impl sealed::TimerChannel for Ch3n {}

/// Amount of bits of a timer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimerBits {
    /// 16 bits.
    Bits16,
    /// 32 bits.
    Bits32,
}

// struct State {
//     up_waker: AtomicWaker,
//     cc_waker: [AtomicWaker; 4],
// }

// impl State {
//     const fn new() -> Self {
//         Self {
//             up_waker: AtomicWaker::new(),
//             cc_waker: [const { AtomicWaker::new() }; 4],
//         }
//     }
// }

trait SealedInstance: SealedRccEnableReset + Peripheral<P = Self> {
    // /// Async state for this timer
    // fn state() -> &'static State;
}

/// timer instance.
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + crate::rcc::RccGetFreq + crate::rcc::RccEnableReset + 'static
{
    /// Interrupt for this timer.
    type Interrupt: interrupt::typelevel::Interrupt;

    /// Amount of bits this timer has.
    const BITS: TimerBits;

    /// Registers for this timer.
    ///
    /// This is a raw pointer to the register block. The actual register block layout varies depending on the timer type.
    fn regs() -> *mut ();
}

pub trait AtimInstance: Instance + 'static {}
pub trait GptimInstance: Instance + 'static {}
pub trait BimInstance: Instance + 'static {}

// TODO: move to _generated.rs
use crate::peripherals;

impl SealedInstance for peripherals::ATIM1 {}
impl Instance for peripherals::ATIM1 {
    type Interrupt = interrupt::typelevel::ATIM1;
    const BITS: TimerBits = TimerBits::Bits32;
    fn regs() -> *mut () {
        crate::pac::ATIM1.as_ptr()
    }
}
impl AtimInstance for peripherals::ATIM1 {}

impl SealedInstance for peripherals::GPTIM1 {}
impl Instance for peripherals::GPTIM1 {
    type Interrupt = interrupt::typelevel::GPTIM1;
    const BITS: TimerBits = TimerBits::Bits32;
    fn regs() -> *mut () {
        crate::pac::GPTIM1.as_ptr()
    }
}
impl GptimInstance for peripherals::GPTIM1 {}

impl SealedInstance for peripherals::GPTIM2 {}
impl Instance for peripherals::GPTIM2 {
    type Interrupt = interrupt::typelevel::GPTIM2;
    const BITS: TimerBits = TimerBits::Bits32;
    fn regs() -> *mut () {
        crate::pac::GPTIM2.as_ptr()
    }
}
impl GptimInstance for peripherals::GPTIM2 {}

impl SealedInstance for peripherals::BTIM1 {}
impl Instance for peripherals::BTIM1 {
    type Interrupt = interrupt::typelevel::BTIM1;
    const BITS: TimerBits = TimerBits::Bits32;
    fn regs() -> *mut () {
        crate::pac::BTIM1.as_ptr()
    }
}
impl BimInstance for peripherals::BTIM1 {}

impl SealedInstance for peripherals::BTIM2 {}
impl Instance for peripherals::BTIM2 {
    type Interrupt = interrupt::typelevel::BTIM2;
    const BITS: TimerBits = TimerBits::Bits32;
    fn regs() -> *mut () {
        crate::pac::BTIM2.as_ptr()
    }
}
impl BimInstance for peripherals::BTIM2 {}

// DMA trait system (auto-generated request() method via dma_trait! macro)
dma_trait!(UpdateDma, Instance);
dma_trait!(TriggerDma, Instance);
dma_trait!(Cc1Dma, Instance);
dma_trait!(Cc2Dma, Instance);
dma_trait!(Cc3Dma, Instance);
dma_trait!(Cc4Dma, Instance);

/// Convenience alias for Update DMA
pub trait UpDma<T: Instance>: UpdateDma<T> {}

// Blanket implementation: any UpdateDma is also an UpDma
impl<T: Instance, D: UpdateDma<T>> UpDma<T> for D {}

// DMA mapping will be auto-generated by build.rs from dma.yaml
// Example generated code:
// impl UpdateDma<peripherals::GPTIM2> for peripherals::DMAC1_CH1 {}
// impl Cc1Dma<peripherals::GPTIM2> for peripherals::DMAC1_CH2 {}
// impl UpDma<peripherals::GPTIM2> for T where T: UpdateDma<peripherals::GPTIM2> {}

// Module declarations
pub mod low_level;
pub mod simple_pwm;

// Re-export commonly used types
pub use low_level::{CountingMode, OutputCompareMode, Timer};
pub use simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel, TimerPin};

// Timer pin implementations are auto-generated by build.rs
// from pinmux.yaml and pinmux_signals.yaml
//
// Generated code will be like:
// impl TimerPin<peripherals::GPTIM2, Ch1> for peripherals::PA32 {
//     fn fsel(&self) -> u8 { 5 }
//     fn set_cfg_pin(&self) {
//         crate::pac::HPSYS_CFG.gptim2_pinr().modify(|w| w.set_ch1_pin(32));
//     }
// }
