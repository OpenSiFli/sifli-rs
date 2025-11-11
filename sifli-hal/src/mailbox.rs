//! Mailbox HAL driver
//!
//! The Mailbox HAL driver provides high-level APIs for using the hardware mailbox module.
//! Each subsystem has a hardware mailbox module that can be used to:
//! - Trigger interrupts to notify other subsystems (e.g., HPSYS mailbox group H2L_MAILBOX
//!   triggers LPSYS interrupts)
//! - Protect shared hardware resources across multiple subsystems using mutex channels
//!
//! ## Features
//!
//! - Each mailbox group has 16 channels, allowing simultaneous triggering of all interrupts
//! - Mailbox interrupts can automatically wake up subsystems in LIGHT/DEEP/STANDBY low-power modes
//! - Mutex channels to protect shared resources, accessible from all subsystems
//!
//! ## Available Resources
//!
//! ### HPSYS (High-Power Subsystem)
//! - `MAILBOX1` (H2L_MAILBOX) - Mailbox for HCPU to LCPU communication
//! - Mutex channels (HMUTEX_CH1-4)
//!
//! ### LPSYS (Low-Power Subsystem)
//! - `MAILBOX2` (L2H_MAILBOX) - Mailbox for LCPU to HCPU communication
//! - Mutex channels (LMUTEX_CH1-2)
//!
//! ## Hardware Register Structure
//!
//! Each mailbox channel has the following registers:
//! - CxIER: Interrupt Enable Register
//! - CxITR: Interrupt Trigger Register
//! - CxICR: Interrupt Clear Register
//! - CxISR: Interrupt Status Register
//! - CxMISR: Masked Interrupt Status Register
//! - CxEXR: Exclusive (Mutex) Register
//!
//! ## Note
//!
//! This is a hardware abstraction layer that provides safe access to mailbox functionality.
//! The actual peripheral instances and interrupts must be properly configured in the PAC.
//! Currently, this module provides the types and interfaces, with actual peripheral
//! implementations to be added when PAC support is available.

use crate::_generated::interrupt::typelevel::Interrupt;

/// Mailbox channel identifier (0-15)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel(u8);

impl Channel {
    /// Maximum number of channels
    pub const MAX_CHANNELS: u8 = 16;

    /// Create a new channel (0-15)
    ///
    /// Returns `None` if the channel number is >= 16
    #[inline]
    pub const fn new(channel: u8) -> Option<Self> {
        if channel < Self::MAX_CHANNELS {
            Some(Channel(channel))
        } else {
            None
        }
    }

    /// Create a new channel without bounds checking
    ///
    /// # Safety
    /// The caller must ensure that `channel` is in range 0-15
    #[inline]
    pub const unsafe fn new_unchecked(channel: u8) -> Self {
        Channel(channel)
    }

    /// Convert channel to bit mask
    #[inline]
    pub const fn mask(self) -> u32 {
        1u32 << self.0
    }

    /// Get channel index
    #[inline]
    pub const fn index(self) -> u8 {
        self.0
    }
}

// Predefined channel constants for convenience
impl Channel {
    pub const CH0: Self = Channel(0);
    pub const CH1: Self = Channel(1);
    pub const CH2: Self = Channel(2);
    pub const CH3: Self = Channel(3);
    pub const CH4: Self = Channel(4);
    pub const CH5: Self = Channel(5);
    pub const CH6: Self = Channel(6);
    pub const CH7: Self = Channel(7);
    pub const CH8: Self = Channel(8);
    pub const CH9: Self = Channel(9);
    pub const CH10: Self = Channel(10);
    pub const CH11: Self = Channel(11);
    pub const CH12: Self = Channel(12);
    pub const CH13: Self = Channel(13);
    pub const CH14: Self = Channel(14);
    pub const CH15: Self = Channel(15);
}

/// Mailbox driver state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Mailbox not yet initialized or disabled
    Reset,
    /// Mailbox initialized and ready for use
    Ready,
    /// Mailbox internal processing is ongoing
    Busy,
}

/// Mutex lock core identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LockCore {
    /// Mutex is not locked
    Unlocked = 0,
    /// Mutex is locked by HCPU
    Hcpu = 1,
    /// Mutex is locked by LCPU
    Lcpu = 2,
    /// Mutex is locked by BCPU
    Bcpu = 3,
}

impl LockCore {
    fn from_u8(value: u8) -> Self {
        match value {
            0 => LockCore::Unlocked,
            1 => LockCore::Hcpu,
            2 => LockCore::Lcpu,
            3 => LockCore::Bcpu,
            _ => LockCore::Unlocked,
        }
    }

    /// Check if the mutex is locked
    #[inline]
    pub const fn is_locked(self) -> bool {
        !matches!(self, LockCore::Unlocked)
    }
}

/// Mailbox register block structure (matching C HAL)
///
/// This represents the hardware register layout for ONE channel within a mailbox peripheral.
/// The full mailbox has 4 channels (C1-C4), each with this register layout.
/// Each register group controls 16 interrupt lines (INT0-INT15).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MailboxChannelRegs {
    /// Interrupt Enable Register - enables specific interrupt channels
    pub ier: u32,
    /// Interrupt Trigger Register - triggers interrupts to the other core
    pub itr: u32,
    /// Interrupt Clear Register - clears pending interrupts
    pub icr: u32,
    /// Interrupt Status Register - raw interrupt status
    pub isr: u32,
    /// Masked Interrupt Status Register - masked interrupt status (ISR & IER)
    pub misr: u32,
    /// Exclusive (Mutex) Register - for mutex lock/unlock operations
    pub exr: u32,
}

/// Complete mailbox peripheral with 4 channels
///
/// MAILBOX1 (H2L_MAILBOX @ 0x50082000): Used by HCPU to send to LCPU
/// MAILBOX2 (L2H_MAILBOX @ 0x40002000): Used by LCPU to send to HCPU
#[repr(C)]
pub struct MailboxRegs {
    pub c1: MailboxChannelRegs,
    pub c2: MailboxChannelRegs,
    pub c3: MailboxChannelRegs,
    pub c4: MailboxChannelRegs,
}

/// Helper functions for mailbox operations
impl MailboxChannelRegs {
    /// Mask (disable) interrupt for the specified channel
    #[inline]
    pub fn mask_channel(&mut self, channel: Channel) {
        unsafe {
            core::ptr::write_volatile(&mut self.ier as *mut u32,
                core::ptr::read_volatile(&self.ier as *const u32) & !(channel.mask()));
        }
    }

    /// Unmask (enable) interrupt for the specified channel
    #[inline]
    pub fn unmask_channel(&mut self, channel: Channel) {
        unsafe {
            core::ptr::write_volatile(&mut self.ier as *mut u32,
                core::ptr::read_volatile(&self.ier as *const u32) | channel.mask());
        }
    }

    /// Trigger interrupt on the specified channel
    #[inline]
    pub fn trigger_channel(&mut self, channel: Channel) {
        unsafe {
            core::ptr::write_volatile(&mut self.itr as *mut u32, channel.mask());
        }
    }

    /// Check if interrupt is pending on the specified channel
    #[inline]
    pub fn is_channel_pending(&self, channel: Channel) -> bool {
        unsafe {
            (core::ptr::read_volatile(&self.isr as *const u32) & channel.mask()) != 0
        }
    }

    /// Clear interrupt on the specified channel
    #[inline]
    pub fn clear_channel(&mut self, channel: Channel) {
        unsafe {
            core::ptr::write_volatile(&mut self.icr as *mut u32, channel.mask());
        }
    }

    /// Get the masked interrupt status (all channels)
    #[inline]
    pub fn get_status(&self) -> u32 {
        unsafe {
            core::ptr::read_volatile(&self.misr as *const u32)
        }
    }

    /// Clear all interrupts with the specified status mask
    #[inline]
    pub fn clear_status(&mut self, status: u32) {
        unsafe {
            core::ptr::write_volatile(&mut self.icr as *mut u32, status);
        }
    }

    /// Try to lock the mutex
    ///
    /// Returns the core that holds the lock, or Unlocked if successfully acquired
    pub fn try_lock(&self) -> LockCore {
        unsafe {
            let exr = core::ptr::read_volatile(&self.exr as *const u32);
            // Bit 0 (EX): 1 = unlocked, 0 = locked
            // Bits [2:1] (ID): Core ID that holds the lock
            if (exr & 0x1) != 0 {
                LockCore::Unlocked
            } else {
                let core_id = ((exr >> 1) & 0x3) as u8;
                LockCore::from_u8(core_id)
            }
        }
    }

    /// Unlock the mutex
    ///
    /// # Safety
    /// This should only be called by the core that currently holds the lock
    pub unsafe fn unlock(&mut self) {
        core::ptr::write_volatile(&mut self.exr as *mut u32,
            core::ptr::read_volatile(&self.exr as *const u32) | 0x1);
    }
}

/// RAII guard for mutex lock
///
/// The mutex is automatically unlocked when this guard is dropped.
pub struct MutexGuard<'a> {
    regs: &'a mut MailboxChannelRegs,
}

impl<'a> Drop for MutexGuard<'a> {
    fn drop(&mut self) {
        unsafe {
            self.regs.unlock();
        }
    }
}

/// Mailbox base addresses (matching C SDK register.h)
pub mod addrs {
    /// MAILBOX1 base address (HPSYS mailbox for H2L communication)
    pub const MAILBOX1_BASE: usize = 0x50082000;

    /// MAILBOX2 base address (LPSYS mailbox for L2H communication)
    pub const MAILBOX2_BASE: usize = 0x40002000;
}

// /// Get a pointer to MAILBOX1 channel registers
// ///
// /// # Safety
// /// Caller must ensure exclusive access and proper synchronization
// pub unsafe fn mailbox1_regs() -> &'static mut MailboxChannelRegs {
//     &mut *(addrs::MAILBOX1_BASE as *mut MailboxChannelRegs)
// }

// /// Get a pointer to MAILBOX2 channel registers
// ///
// /// # Safety
// /// Caller must ensure exclusive access and proper synchronization
// pub unsafe fn mailbox2_regs() -> &'static mut MailboxChannelRegs {
//     &mut *(addrs::MAILBOX2_BASE as *mut MailboxChannelRegs)
// }


// TODO: When PAC support is available, implement the following:
// - Mailbox driver struct with Peripheral trait
// - Interrupt handlers
// - Integration with embassy_hal patterns
// - Proper RCC enable/reset support

use core::marker::PhantomData;

// Example of future API once PAC is available:
//
// ```rust,ignore
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::{interrupt, rcc::RccEnableReset};
//
pub struct Mailbox<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Mailbox<'d, T> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        into_ref!(peri);
        // T::enable_and_reset();
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        Self { _peri: peri }
    }

    pub fn mask_channel(&mut self, channel: Channel) {
        T::regs().mask_channel(channel);
    }
    // ... other methods
}

pub trait Instance: RccEnableReset + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
    fn regs() -> &'static mut MailboxChannelRegs;
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        todo!()
    }
}

impl Instance for crate::peripherals::MAILBOX1 {
    type Interrupt = crate::interrupt::typelevel::MAILBOX2_CH1;
    
    fn regs() -> &'static mut MailboxChannelRegs {
        unsafe {
            &mut *(addrs::MAILBOX2_BASE as *mut MailboxChannelRegs)
        }
    }
}
// ```
