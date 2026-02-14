//! Mailbox driver for inter-processor communication
//!
//! ## Hardware Overview
//!
//! - **MAILBOX1** (4 channels): HCPU writes ITR → triggers LCPU interrupt
//! - **MAILBOX2** (2 channels): LCPU writes ITR → triggers HCPU interrupt
//!
//! Each channel has 16 interrupt bits that can be triggered independently.
//!
//! ## Usage
//!
//! ```ignore
//! use sifli_hal::{bind_interrupts, mailbox};
//!
//! bind_interrupts!(struct Irqs {
//!     MAILBOX2_CH1 => mailbox::InterruptHandler<mailbox::Mailbox2Ch1>;
//! });
//!
//! async fn receive(mut ch: mailbox::Mailbox2Ch1<'static>) {
//!     ch.enable_interrupt(0xFF);
//!     loop {
//!         let bits = ch.wait().await;
//!         // Handle triggered bits...
//!     }
//! }
//! ```

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::Poll;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt;
use crate::peripherals;

pub use crate::pac::mailbox::vals::LockCore;

// ============================================================================
// Async state for MAILBOX2 channels
// ============================================================================

struct ChannelState {
    waker: AtomicWaker,
    pending_bits: AtomicU16,
}

impl ChannelState {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            pending_bits: AtomicU16::new(0),
        }
    }
}

static MAILBOX2_CH1_STATE: ChannelState = ChannelState::new();
static MAILBOX2_CH2_STATE: ChannelState = ChannelState::new();

// ============================================================================
// Channel implementations via macro
// ============================================================================

macro_rules! impl_mailbox_channel {
    // TX channel (MAILBOX1)
    (tx: $name:ident, $peri:ident, $ch:expr) => {
        /// MAILBOX1 TX channel (HCPU → LCPU)
        pub struct $name<'d> {
            _peri: PeripheralRef<'d, peripherals::$peri>,
        }

        impl<'d> $name<'d> {
            /// Create new channel instance
            pub fn new(peri: impl Peripheral<P = peripherals::$peri> + 'd) -> Self {
                into_ref!(peri);
                Self { _peri: peri }
            }

            /// Trigger interrupt on LCPU
            #[inline]
            pub fn trigger(&mut self, mask: u16) {
                crate::pac::MAILBOX1
                    .itr($ch - 1)
                    .write_value(crate::pac::mailbox::regs::Ixr(mask as u32));
            }

            /// Enable interrupt bits
            #[inline]
            pub fn enable_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX1
                    .ier($ch - 1)
                    .modify(|w| w.0 |= mask as u32);
            }

            /// Disable interrupt bits
            #[inline]
            pub fn disable_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX1
                    .ier($ch - 1)
                    .modify(|w| w.0 &= !(mask as u32));
            }

            /// Clear interrupt flags
            #[inline]
            pub fn clear_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX1
                    .icr($ch - 1)
                    .write_value(crate::pac::mailbox::regs::Ixr(mask as u32));
            }

            /// Read raw interrupt status
            #[inline]
            pub fn status(&self) -> u16 {
                crate::pac::MAILBOX1.isr($ch - 1).read().0 as u16
            }

            /// Read masked interrupt status (ISR & IER)
            #[inline]
            pub fn masked_status(&self) -> u16 {
                crate::pac::MAILBOX1.misr($ch - 1).read().0 as u16
            }

            /// Try to acquire hardware mutex
            #[inline]
            pub fn try_lock(&mut self) -> Result<(), LockCore> {
                let exr = crate::pac::MAILBOX1.exr($ch - 1).read();
                if exr.ex() {
                    Ok(())
                } else {
                    Err(exr.id())
                }
            }

            /// Release hardware mutex
            #[inline]
            pub fn unlock(&mut self) {
                crate::pac::MAILBOX1.exr($ch - 1).write(|w| w.set_ex(true));
            }
        }

        impl crate::mailbox::sealed::SealedTxChannel for peripherals::$peri {}
        impl crate::mailbox::TxChannel for peripherals::$peri {}
    };

    // RX channel (MAILBOX2) with async support
    (rx: $name:ident, $peri:ident, $ch:expr, $state:ident, $irq:ident) => {
        /// MAILBOX2 RX channel (LCPU → HCPU)
        pub struct $name<'d> {
            _peri: PeripheralRef<'d, peripherals::$peri>,
        }

        impl<'d> $name<'d> {
            /// Create new channel instance
            pub fn new(peri: impl Peripheral<P = peripherals::$peri> + 'd) -> Self {
                into_ref!(peri);
                Self { _peri: peri }
            }

            /// Trigger interrupt (for testing, normally LCPU triggers this)
            #[inline]
            pub fn trigger(&mut self, mask: u16) {
                crate::pac::MAILBOX2
                    .itr($ch - 1)
                    .write_value(crate::pac::mailbox::regs::Ixr(mask as u32));
            }

            /// Enable interrupt bits
            #[inline]
            pub fn enable_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX2
                    .ier($ch - 1)
                    .modify(|w| w.0 |= mask as u32);
            }

            /// Disable interrupt bits
            #[inline]
            pub fn disable_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX2
                    .ier($ch - 1)
                    .modify(|w| w.0 &= !(mask as u32));
            }

            /// Clear interrupt flags
            #[inline]
            pub fn clear_interrupt(&mut self, mask: u16) {
                crate::pac::MAILBOX2
                    .icr($ch - 1)
                    .write_value(crate::pac::mailbox::regs::Ixr(mask as u32));
            }

            /// Read raw interrupt status
            #[inline]
            pub fn status(&self) -> u16 {
                crate::pac::MAILBOX2.isr($ch - 1).read().0 as u16
            }

            /// Read masked interrupt status (ISR & IER)
            #[inline]
            pub fn masked_status(&self) -> u16 {
                crate::pac::MAILBOX2.misr($ch - 1).read().0 as u16
            }

            /// Try to acquire hardware mutex
            #[inline]
            pub fn try_lock(&mut self) -> Result<(), LockCore> {
                let exr = crate::pac::MAILBOX2.exr($ch - 1).read();
                if exr.ex() {
                    Ok(())
                } else {
                    Err(exr.id())
                }
            }

            /// Release hardware mutex
            #[inline]
            pub fn unlock(&mut self) {
                crate::pac::MAILBOX2.exr($ch - 1).write(|w| w.set_ex(true));
            }

            /// Wait for interrupt asynchronously
            pub async fn wait(&mut self) -> u16 {
                poll_fn(|cx| {
                    $state.waker.register(cx.waker());
                    let bits = $state.pending_bits.swap(0, Ordering::SeqCst);
                    if bits != 0 {
                        Poll::Ready(bits)
                    } else {
                        Poll::Pending
                    }
                })
                .await
            }
        }

        impl crate::mailbox::sealed::SealedRxChannel for $name<'_> {
            type Interrupt = interrupt::typelevel::$irq;

            fn on_interrupt() {
                let regs = crate::pac::MAILBOX2;
                let misr = regs.misr($ch - 1).read().0 as u16;
                if misr == 0 {
                    return;
                }
                regs.icr($ch - 1).write_value(crate::pac::mailbox::regs::Ixr(misr as u32));
                $state.pending_bits.fetch_or(misr, Ordering::SeqCst);
                $state.waker.wake();
            }
        }
        impl crate::mailbox::RxChannel for $name<'_> {}
    };
}

// MAILBOX1 TX channels
impl_mailbox_channel!(tx: Mailbox1Ch1, MAILBOX1_CH1, 1);
impl_mailbox_channel!(tx: Mailbox1Ch2, MAILBOX1_CH2, 2);
impl_mailbox_channel!(tx: Mailbox1Ch3, MAILBOX1_CH3, 3);
impl_mailbox_channel!(tx: Mailbox1Ch4, MAILBOX1_CH4, 4);

// MAILBOX2 RX channels
impl_mailbox_channel!(rx: Mailbox2Ch1, MAILBOX2_CH1, 1, MAILBOX2_CH1_STATE, MAILBOX2_CH1);
impl_mailbox_channel!(rx: Mailbox2Ch2, MAILBOX2_CH2, 2, MAILBOX2_CH2_STATE, MAILBOX2_CH2);

// ============================================================================
// Traits
// ============================================================================

mod sealed {
    pub trait SealedTxChannel {}

    pub trait SealedRxChannel {
        type Interrupt: crate::interrupt::typelevel::Interrupt;
        fn on_interrupt();
    }
}

/// TX channel trait (MAILBOX1: HCPU → LCPU)
#[allow(private_bounds)]
pub trait TxChannel: sealed::SealedTxChannel {}

/// RX channel trait (MAILBOX2: LCPU → HCPU)
#[allow(private_bounds)]
pub trait RxChannel: sealed::SealedRxChannel {}

// ============================================================================
// Interrupt handler
// ============================================================================

/// Interrupt handler for MAILBOX2 RX channels
///
/// # Example
///
/// ```no_run
/// use sifli_hal::{bind_interrupts, mailbox};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => mailbox::InterruptHandler<mailbox::Mailbox2Ch1<'static>>;
/// });
/// ```
pub struct InterruptHandler<C: RxChannel> {
    _phantom: PhantomData<C>,
}

impl<C: RxChannel> interrupt::typelevel::Handler<C::Interrupt> for InterruptHandler<C> {
    unsafe fn on_interrupt() {
        C::on_interrupt();
    }
}
