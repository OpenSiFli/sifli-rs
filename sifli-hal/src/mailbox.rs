//! Mailbox HAL driver
//!
//! Provides hardware mailbox for inter-processor communication on SF32LB52x chips.
//!
//! ## Hardware Architecture
//!
//! Physical MAILBOX peripherals on chip:
//! - **MAILBOX1** @ 0x50082000 (HPSYS address space) - 4 channels
//! - **MAILBOX2** @ 0x40002000 (LPSYS address space) - 2 channels
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
//! ## Design: Channels as Fields
//!
//! Each mailbox exposes channels as struct fields for compile-time safety:
//! - `Mailbox<MAILBOX1>` has 4 channels: `ch1`, `ch2`, `ch3`, `ch4`
//! - `Mailbox<MAILBOX2>` has 2 channels: `ch1`, `ch2`
//!
//! This design provides:
//! - **Compile-time safety**: Cannot access non-existent channels
//! - **Zero-cost abstractions**: Channel ownership can be split across tasks
//! - **Type-driven API**: Different mailbox instances have different field counts
//!
//! ## Usage
//!
//! ```no_run
//! use sifli_hal::{mailbox, peripherals};
//!
//! let p = /* get peripherals */;
//! # unsafe { peripherals::Peripherals::steal() };
//!
//! let mut mb = mailbox::Mailbox1::new(p.MAILBOX1);
//!
//! // Access channels as fields - compile-time safe!
//! mb.ch1.trigger(0);  // Trigger interrupt bit 0 on channel 1
//! mb.ch4.trigger(15); // Trigger interrupt bit 15 on channel 4
//!
//! // Use as mutex (all cores can access)
//! match mb.ch1.try_lock() {
//!     mailbox::LockCore::Unlocked => {
//!         // Got the lock
//!         // ... critical section ...
//!         unsafe { mb.ch1.unlock() };
//!     }
//!     core => defmt::info!("Locked by {:?}", core),
//! }
//!
//! // Split ownership across tasks
//! let (ch1, ch2, ch3, ch4) = mb.split();
//! spawner.spawn(task1(ch1)).unwrap();
//! spawner.spawn(task2(ch2)).unwrap();
//! ```

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use crate::peripherals;

/// Re-export LockCore enum from PAC
pub use crate::pac::mailbox::vals::LockCore;

/// Sealed trait to constrain mailbox peripheral types
mod sealed {
    pub trait SealedMailboxInstance {}
}

/// Unified mailbox register block type (uses MAILBOX1 layout as common type)
type Regs = crate::pac::mailbox::Mailbox1;

/// Trait for mailbox peripheral instances
///
/// This trait is sealed and cannot be implemented outside this module.
/// It provides a unified interface to access MAILBOX1 and MAILBOX2 registers
/// via unsafe pointer conversion (embassy-stm32 style).
#[allow(private_bounds)]
pub trait MailboxInstance: sealed::SealedMailboxInstance + 'static {
    /// Get the unified register block via unsafe pointer conversion
    fn regs() -> Regs;
}

impl sealed::SealedMailboxInstance for peripherals::MAILBOX1 {}
impl MailboxInstance for peripherals::MAILBOX1 {
    #[inline]
    fn regs() -> Regs {
        unsafe { Regs::from_ptr(crate::pac::MAILBOX1.as_ptr()) }
    }
}

impl sealed::SealedMailboxInstance for peripherals::MAILBOX2 {}
impl MailboxInstance for peripherals::MAILBOX2 {
    #[inline]
    fn regs() -> Regs {
        unsafe { Regs::from_ptr(crate::pac::MAILBOX2.as_ptr()) }
    }
}

/// Mailbox channel with generic peripheral type
///
/// Generic over:
/// - `T`: Mailbox peripheral type (MAILBOX1 or MAILBOX2)
/// - `CH`: Channel index (compile-time constant)
pub struct MailboxChannel<'d, T: MailboxInstance, const CH: usize> {
    _phantom: core::marker::PhantomData<&'d mut T>,
}

impl<'d, T: MailboxInstance, const CH: usize> MailboxChannel<'d, T, CH> {
    /// Create new channel (internal use only)
    #[inline]
    const fn new() -> Self {
        Self {
            _phantom: core::marker::PhantomData,
        }
    }

    /// Enable interrupt reception (unmask)
    ///
    /// Allows receiving interrupts from remote core for this bit.
    /// Must be called on the **receiving side** before remote can send interrupts.
    ///
    /// # Arguments
    /// - `bit`: Interrupt bit 0-15
    #[inline]
    pub fn enable_interrupt(&mut self, bit: u8) {
        assert!(bit < 16, "bit must be 0-15");
        T::regs().ier(CH).modify(|w| w.set_int(bit as usize, true));
    }

    /// Disable interrupt reception (mask)
    ///
    /// Prevents receiving interrupts from remote core for this bit.
    ///
    /// # Arguments
    /// - `bit`: Interrupt bit 0-15
    #[inline]
    pub fn disable_interrupt(&mut self, bit: u8) {
        assert!(bit < 16, "bit must be 0-15");
        T::regs().ier(CH).modify(|w| w.set_int(bit as usize, false));
    }

    /// Enable interrupt reception for multiple bits at once
    ///
    /// Unmasks all interrupt bits specified in the mask.
    ///
    /// # Arguments
    /// - `mask`: Bitmask of interrupts to enable (bits 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// // Enable interrupts for bits 0, 1, and 2
    /// mb.ch1.enable_interrupt_mask(0b0111);
    /// ```
    #[inline]
    pub fn enable_interrupt_mask(&mut self, mask: u16) {
        T::regs().ier(CH).modify(|w| w.0 |= mask as u32);
    }

    /// Disable interrupt reception for multiple bits at once
    ///
    /// Masks all interrupt bits specified in the mask.
    ///
    /// # Arguments
    /// - `mask`: Bitmask of interrupts to disable (bits 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// // Disable interrupts for bits 0, 1, and 2
    /// mb.ch1.disable_interrupt_mask(0b0111);
    /// ```
    #[inline]
    pub fn disable_interrupt_mask(&mut self, mask: u16) {
        T::regs().ier(CH).modify(|w| w.0 &= !(mask as u32));
    }

    /// Trigger interrupt on remote core
    ///
    /// # Arguments
    /// - `bit`: Interrupt bit 0-15
    #[inline]
    pub fn trigger(&mut self, bit: u8) {
        assert!(bit < 16, "bit must be 0-15");
        T::regs().itr(CH).write(|w| w.set_int(bit as usize, true));
    }

    /// Trigger multiple bits at once
    ///
    /// # Arguments
    /// - `mask`: Bitmask of interrupts to trigger (bits 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// // Trigger bits 0, 3, and 7 simultaneously
    /// mb.ch1.trigger_mask(0b1000_1001);
    /// ```
    #[inline]
    pub fn trigger_mask(&mut self, mask: u16) {
        T::regs().itr(CH).write(|w| w.0 = mask as u32);
    }

    /// Clear interrupt flag
    ///
    /// Clears the specified interrupt bit in the ISR register.
    /// This should be called in the interrupt handler after processing the interrupt.
    ///
    /// # Arguments
    /// - `bit`: Interrupt bit 0-15 to clear
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// // In interrupt handler
    /// if mb.ch1.check_interrupt(5) {
    ///     // Process interrupt bit 5
    ///     // ...
    ///     // Clear the flag
    ///     mb.ch1.clear_interrupt(5);
    /// }
    /// ```
    #[inline]
    pub fn clear_interrupt(&mut self, bit: u8) {
        assert!(bit < 16, "bit must be 0-15");
        T::regs().icr(CH).write(|w| w.set_int(bit as usize, true));
    }

    /// Clear multiple interrupt flags at once
    ///
    /// Clears all interrupt bits specified in the mask.
    ///
    /// # Arguments
    /// - `mask`: Bitmask of interrupts to clear (bits 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// // Clear bits 0, 3, and 7
    /// mb.ch1.clear_interrupt_mask(0b1000_1001);
    /// ```
    #[inline]
    pub fn clear_interrupt_mask(&mut self, mask: u16) {
        T::regs().icr(CH).write(|w| w.0 = mask as u32);
    }

    /// Check if specific interrupt bit is triggered
    ///
    /// This is a convenience method that checks a single bit in the ISR register.
    /// Equivalent to `(read_interrupt_status() & (1 << bit)) != 0`.
    ///
    /// # Arguments
    /// - `bit`: Interrupt bit 0-15 to check
    ///
    /// # Returns
    /// `true` if the interrupt bit is set, `false` otherwise
    ///
    /// # Example
    /// ```no_run
    /// # let mb = todo!();
    /// if mb.ch1.check_interrupt(3) {
    ///     // Interrupt bit 3 is triggered
    /// }
    /// ```
    #[inline]
    pub fn check_interrupt(&self, bit: u8) -> bool {
        assert!(bit < 16, "bit must be 0-15");
        T::regs().isr(CH).read().int(bit as usize)
    }

    /// Read raw interrupt status register (ISR)
    ///
    /// Returns the current state of all interrupt bits (0-15) regardless of masking.
    /// Each set bit indicates that the corresponding interrupt was triggered.
    ///
    /// # Returns
    /// Bitmask of triggered interrupts (bit 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mb = todo!();
    /// let status = mb.ch1.read_interrupt_status();
    /// if status & 0x01 != 0 {
    ///     // Interrupt bit 0 is triggered
    /// }
    /// ```
    #[inline]
    pub fn read_interrupt_status(&self) -> u16 {
        T::regs().isr(CH).read().0 as u16
    }

    /// Read masked interrupt status register (MISR)
    ///
    /// Returns only the interrupts that are both triggered AND enabled.
    /// This is equivalent to `ISR & IER`.
    ///
    /// # Returns
    /// Bitmask of enabled and triggered interrupts (bit 0-15)
    ///
    /// # Example
    /// ```no_run
    /// # let mb = todo!();
    /// // Only shows interrupts that are both triggered and unmasked
    /// let masked_status = mb.ch1.read_masked_interrupt_status();
    /// ```
    #[inline]
    pub fn read_masked_interrupt_status(&self) -> u16 {
        T::regs().misr(CH).read().0 as u16
    }

    /// Try to acquire mutex lock
    ///
    /// Returns `Unlocked` if lock was acquired, otherwise returns current owner.
    ///
    /// # Hardware behavior
    /// Reading the EXR register is an atomic operation:
    /// - If unlocked: Sets the lock and returns `Unlocked`
    /// - If locked: Returns the core ID that owns the lock
    /// - Read-to-clear: When `EX = 1` is read, hardware clears it to `EX = 0` to claim the mutex
    ///
    /// # Example
    /// ```no_run
    /// # let mut mb = todo!();
    /// match mb.ch1.try_lock() {
    ///     LockCore::Unlocked => {
    ///         // Lock acquired, enter critical section
    ///         critical_work();
    ///         unsafe { mb.ch1.unlock() };
    ///     }
    ///     LockCore::Hcpu => {
    ///         // Locked by HCPU, retry later
    ///     }
    ///     _ => {}
    /// }
    /// ```
    #[inline]
    pub fn try_lock(&mut self) -> LockCore {
        let exr = T::regs().exr(CH).read();

        if exr.ex() {
            LockCore::Unlocked
        } else {
            exr.id()
        }
    }

    /// Unlock mutex
    ///
    /// # Safety
    /// Caller must own the lock (i.e., `try_lock()` returned `Unlocked`)
    #[inline]
    pub unsafe fn unlock(&mut self) {
        T::regs().exr(CH).write(|w| w.set_ex(true));
    }
}

/// MAILBOX1 driver (4 channels)
pub struct Mailbox1<'d> {
    _peri: PeripheralRef<'d, peripherals::MAILBOX1>,
    /// Channel 1 (hardware channel 0)
    pub ch1: MailboxChannel<'d, peripherals::MAILBOX1, 0>,
    pub ch2: MailboxChannel<'d, peripherals::MAILBOX1, 1>,
    pub ch3: MailboxChannel<'d, peripherals::MAILBOX1, 2>,
    pub ch4: MailboxChannel<'d, peripherals::MAILBOX1, 3>,
}

impl<'d> Mailbox1<'d> {
    /// Create new MAILBOX1 instance
    ///
    /// Enables the mailbox peripheral clock via RCC.
    pub fn new(peri: impl Peripheral<P = peripherals::MAILBOX1> + 'd) -> Self {
        into_ref!(peri);
        crate::rcc::enable_and_reset::<peripherals::MAILBOX1>();
        Self {
            _peri: peri,
            ch1: MailboxChannel::new(),
            ch2: MailboxChannel::new(),
            ch3: MailboxChannel::new(),
            ch4: MailboxChannel::new(),
        }
    }

    /// Split into individual channels for separate ownership
    ///
    /// This allows passing channels to different tasks or modules.
    ///
    /// # Example
    /// ```no_run
    /// # use embassy_executor::Spawner;
    /// # async fn example(spawner: Spawner) {
    /// let p = sifli_hal::init(Default::default());
    /// let mb = sifli_hal::mailbox::Mailbox1::new(p.MAILBOX1);
    ///
    /// let (ch1, ch2, ch3, ch4) = mb.split();
    ///
    /// spawner.spawn(control_task(ch1)).unwrap();
    /// spawner.spawn(data_task(ch2)).unwrap();
    /// # }
    /// ```
    pub fn split(
        self,
    ) -> (
        MailboxChannel<'d, peripherals::MAILBOX1, 0>,
        MailboxChannel<'d, peripherals::MAILBOX1, 1>,
        MailboxChannel<'d, peripherals::MAILBOX1, 2>,
        MailboxChannel<'d, peripherals::MAILBOX1, 3>,
    ) {
        (self.ch1, self.ch2, self.ch3, self.ch4)
    }
}

/// MAILBOX2 driver (2 channels)
pub struct Mailbox2<'d> {
    _peri: PeripheralRef<'d, peripherals::MAILBOX2>,
    pub ch1: MailboxChannel<'d, peripherals::MAILBOX2, 0>,
    pub ch2: MailboxChannel<'d, peripherals::MAILBOX2, 1>,
}

impl<'d> Mailbox2<'d> {
    /// Create new MAILBOX2 instance
    ///
    /// Note: MAILBOX2 is in LPSYS and clock is managed by LPSYS_RCC, not HPSYS_RCC.
    /// The clock should already be enabled by bootloader or LCPU firmware.
    pub fn new(peri: impl Peripheral<P = peripherals::MAILBOX2> + 'd) -> Self {
        into_ref!(peri);
        // MAILBOX2 is in LPSYS, no HPSYS RCC control
        Self {
            _peri: peri,
            ch1: MailboxChannel::new(),
            ch2: MailboxChannel::new(),
        }
    }

    /// Split into individual channels for separate ownership
    ///
    /// This allows passing channels to different tasks or modules.
    ///
    /// # Example
    /// ```no_run
    /// # use embassy_executor::Spawner;
    /// # async fn example(spawner: Spawner) {
    /// let p = sifli_hal::init(Default::default());
    /// let mb = sifli_hal::mailbox::Mailbox2::new(p.MAILBOX2);
    ///
    /// let (ch1, ch2) = mb.split();
    ///
    /// spawner.spawn(control_task(ch1)).unwrap();
    /// spawner.spawn(data_task(ch2)).unwrap();
    /// # }
    /// ```
    pub fn split(
        self,
    ) -> (
        MailboxChannel<'d, peripherals::MAILBOX2, 0>,
        MailboxChannel<'d, peripherals::MAILBOX2, 1>,
    ) {
        (self.ch1, self.ch2)
    }
}
