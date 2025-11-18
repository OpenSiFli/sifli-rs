//! Mailbox HAL driver
//!
//! Provides hardware mailbox for inter-processor communication on SF32LB52x chips.
//!
//! ## Hardware Architecture
//!
//! Physical MAILBOX peripherals on chip:
//! - **MAILBOX1** @ 0x40002000 (HPSYS address space)
//! - **MAILBOX2** @ 0x40042000 (LPSYS address space)
//!
//! Each CPU uses one for TX, listens to the other's IRQ for RX:
//!
//! - **HCPU usage**:
//!   - TX: Write MAILBOX1.ITR → triggers LCPU interrupt
//!   - RX: Handle MAILBOX2_CH1_IRQn → read LCPU shared memory
//!     (HCPU doesn't need to access MAILBOX2 registers)
//!
//! - **LCPU usage**:
//!   - TX: Write MAILBOX2.ITR → triggers HCPU interrupt
//!   - RX: Handle MAILBOX1 interrupts → read HCPU shared memory
//!
//! ## Current Support (HCPU side)
//!
//! **TX only via MAILBOX1** (PAC provides MAILBOX1 registers):
//! - Trigger remote LCPU interrupts
//! - Hardware mutex for cross-core locking
//!
//! **RX support pending**: Requires MAILBOX2_CH1 interrupt handler (interrupt exists in PAC,
//! but HCPU doesn't need MAILBOX2 register access - just handle the IRQ)
//!
//! ## Usage
//!
//! ```no_run
//! use sifli_hal::{mailbox, peripherals};
//!
//! let p = /* get peripherals */;
//! # unsafe { peripherals::Peripherals::steal() };
//!
//! let mut mb = mailbox::Mailbox::new(p.MAILBOX1);
//!
//! // Trigger LCPU interrupt on channel 0, bit 0
//! mb.trigger(mailbox::Channel::CH0, 0);
//!
//! // Use as mutex (all cores can access)
//! match mb.try_lock(mailbox::Channel::CH0) {
//!     mailbox::LockCore::Unlocked => {
//!         // Got the lock
//!         // ... critical section ...
//!         unsafe { mb.unlock(mailbox::Channel::CH0) };
//!     }
//!     core => println!("Locked by {:?}", core),
//! }
//! ```

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::pac::mailbox::Mailbox1;
use crate::rcc::RccEnableReset;

/// Mailbox channel (0-3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel(u8);

impl Channel {
    /// Create channel (0-3)
    #[inline]
    pub const fn new(channel: u8) -> Option<Self> {
        if channel < 4 {
            Some(Channel(channel))
        } else {
            None
        }
    }

    /// Create without bounds check
    ///
    /// # Safety
    /// Caller must ensure channel is 0-3
    #[inline]
    pub const unsafe fn new_unchecked(channel: u8) -> Self {
        Channel(channel)
    }

    /// Get index
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }

    /// Channel 0
    pub const CH0: Self = Channel(0);
    /// Channel 1
    pub const CH1: Self = Channel(1);
    /// Channel 2
    pub const CH2: Self = Channel(2);
    /// Channel 3
    pub const CH3: Self = Channel(3);
}

/// Mutex lock owner
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LockCore {
    /// Unlocked (lock acquired if this is returned from try_lock)
    Unlocked = 0,
    /// Locked by HCPU
    Hcpu = 1,
    /// Locked by LCPU
    Lcpu = 2,
    /// Locked by BCPU
    Bcpu = 3,
}

impl LockCore {
    fn from_bits(value: u8) -> Self {
        match value & 0x3 {
            0 => LockCore::Unlocked,
            1 => LockCore::Hcpu,
            2 => LockCore::Lcpu,
            3 => LockCore::Bcpu,
            _ => unreachable!(),
        }
    }

    /// Check if locked
    #[inline]
    pub const fn is_locked(self) -> bool {
        !matches!(self, LockCore::Unlocked)
    }
}

/// Mailbox driver
pub struct Mailbox<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Mailbox<'d, T> {
    /// Create new mailbox
    pub fn new(peri: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(peri);
        crate::rcc::enable_and_reset::<T>();
        Self { _peri: peri }
    }

    /// Trigger interrupt on remote core
    ///
    /// # Arguments
    /// - `channel`: Channel 0-3
    /// - `bit`: Interrupt bit 0-15
    #[inline]
    pub fn trigger(&mut self, channel: Channel, bit: u8) {
        assert!(bit < 16, "bit must be 0-15");
        let regs = T::regs();
        regs.itr(channel.index()).write(|w| w.set_int(bit as usize, true));
    }

    /// Trigger multiple bits
    #[inline]
    pub fn trigger_mask(&mut self, channel: Channel, mask: u16) {
        let regs = T::regs();
        regs.itr(channel.index()).write(|w| w.0 = mask as u32);
    }

    /// Try to acquire mutex lock
    ///
    /// Returns `Unlocked` if lock was acquired, otherwise returns current owner
    #[inline]
    pub fn try_lock(&mut self, channel: Channel) -> LockCore {
        let regs = T::regs();
        let exr = regs.exr(channel.index()).read();

        if exr.ex() {
            LockCore::Unlocked
        } else {
            LockCore::from_bits(exr.id())
        }
    }

    /// Unlock mutex
    ///
    /// # Safety
    /// Caller must own the lock
    #[inline]
    pub unsafe fn unlock(&mut self, channel: Channel) {
        let regs = T::regs();
        regs.exr(channel.index()).write(|w| w.set_ex(true));
    }

    /// Check lock status without acquiring
    #[inline]
    pub fn lock_status(&self, channel: Channel) -> LockCore {
        let regs = T::regs();
        let exr = regs.exr(channel.index()).read();

        if exr.ex() {
            LockCore::Unlocked
        } else {
            LockCore::from_bits(exr.id())
        }
    }
}

trait SealedInstance: RccEnableReset {
    fn regs() -> Mailbox1;
}

/// Mailbox instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + 'static {}

// Peripheral implementations

use crate::peripherals;

impl SealedInstance for peripherals::MAILBOX1 {
    fn regs() -> Mailbox1 {
        crate::pac::MAILBOX1
    }
}

impl Instance for peripherals::MAILBOX1 {}

// TODO: Add RX support via MAILBOX2_CH1 interrupt when needed
// Note: HCPU has MAILBOX2_CH1/CH2 interrupts in PAC, but doesn't need
// MAILBOX2 register access - LCPU writes MAILBOX2.ITR, HCPU just handles IRQ

