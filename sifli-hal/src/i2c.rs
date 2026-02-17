//! Inter-Integrated Circuit (I2C) bus driver
//!
//! SiFli I2C controller implementation based on DesignWare I2C IP.
//!
//! Supports both blocking and async (interrupt-driven) modes.
//!
//! # Example (blocking)
//! ```ignore
//! let i2c = i2c::I2c::new_blocking(
//!     p.I2C3, p.PA40, p.PA39,
//!     i2c::Config::default(),
//! );
//! let mut buf = [0u8; 1];
//! i2c.blocking_write_read(0x30, &[0x39], &mut buf).unwrap();
//! ```
//!
//! # Example (async)
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     I2C3 => i2c::InterruptHandler<peripherals::I2C3>;
//! });
//! let i2c = i2c::I2c::new(
//!     p.I2C3, p.PA40, p.PA39,
//!     Irqs, i2c::Config::default(),
//! );
//! let mut buf = [0u8; 1];
//! i2c.write_read(0x30, &[0x39], &mut buf).await.unwrap();
//! ```
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant};

use crate::gpio::{AfType, Pull, SealedPin};
use crate::interrupt::typelevel::Interrupt as _;
use crate::mode::{Async, Blocking, Mode};
use crate::pac::i2c::I2c as Regs;
use crate::time::Hertz;
use crate::{interrupt, rcc, Peripheral};

/// I2C error
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK received
    Nack,
    /// Timeout
    Timeout,
    /// Overrun/underrun
    Overrun,
    /// Zero-length transfer
    ZeroLength,
}

/// I2C configuration
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// I2C bus frequency (default: 100 kHz standard mode)
    pub frequency: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Hertz(100_000),
        }
    }
}

/// I2C bus mode
#[repr(u8)]
#[derive(Debug, Copy, Clone)]
enum BusMode {
    Standard = 0b00,
    Fast = 0b01,
}

const TIMEOUT_MS: u64 = 1000;

/// Interrupt handler for async I2C operations.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let sr = regs.sr().read();
        let state = T::state();

        let has_event = sr.te() || sr.rf() || sr.bed() || sr.ald() || sr.dmadone();

        if has_event {
            // Disable interrupts that fired to prevent re-entry
            regs.ier().modify(|w| {
                if sr.te() {
                    w.set_teie(false);
                }
                if sr.rf() {
                    w.set_rfie(false);
                }
                if sr.bed() {
                    w.set_bedie(false);
                }
                if sr.ald() {
                    w.set_aldie(false);
                }
                if sr.dmadone() {
                    w.set_dmadoneie(false);
                }
            });

            compiler_fence(Ordering::SeqCst);
            state.waker.wake();
        }
    }
}

/// I2C driver
pub struct I2c<'d, T: Instance, M: Mode> {
    _peri: PeripheralRef<'d, T>,
    _phantom: PhantomData<M>,
}

impl<'d, T: Instance> I2c<'d, T, Blocking> {
    /// Create a new blocking I2C driver
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);

        // Enable I2C clock and reset
        rcc::enable_and_reset::<T>();

        // Configure pins: set FSEL (AF function) and PINR routing
        let scl_ref = new_pin!(scl, AfType::new(Pull::Up));
        let sda_ref = new_pin!(sda, AfType::new(Pull::Up));

        // I2C requires input enabled on both SCL and SDA (bidirectional)
        {
            use crate::gpio::hpsys::HpsysPin;
            let scl_pin_ref = scl_ref.as_ref().unwrap();
            let sda_pin_ref = sda_ref.as_ref().unwrap();
            let mut scl_hw = HpsysPin::new(scl_pin_ref.pin_bank());
            let mut sda_hw = HpsysPin::new(sda_pin_ref.pin_bank());
            scl_hw.set_ie(true);
            sda_hw.set_ie(true);
        }

        // Initialize hardware
        Self::init(config);

        Self {
            _peri: peri,
            _phantom: PhantomData,
        }
    }
}

impl<'d, T: Instance> I2c<'d, T, Async> {
    /// Create a new async I2C driver
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);

        // Enable I2C clock and reset
        rcc::enable_and_reset::<T>();

        // Configure pins
        let scl_ref = new_pin!(scl, AfType::new(Pull::Up));
        let sda_ref = new_pin!(sda, AfType::new(Pull::Up));

        {
            use crate::gpio::hpsys::HpsysPin;
            let scl_pin_ref = scl_ref.as_ref().unwrap();
            let sda_pin_ref = sda_ref.as_ref().unwrap();
            let mut scl_hw = HpsysPin::new(scl_pin_ref.pin_bank());
            let mut sda_hw = HpsysPin::new(sda_pin_ref.pin_bank());
            scl_hw.set_ie(true);
            sda_hw.set_ie(true);
        }

        // Initialize hardware
        Self::init(config);

        // Enable interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            _peri: peri,
            _phantom: PhantomData,
        }
    }

    /// Async write
    pub async fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        self.async_wait_bus_idle(timeout).await?;
        clear_all_flags(regs);

        // Send START + address (write)
        regs.dbr().write(|w| w.set_data(addr << 1));

        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        self.async_wait_te(timeout).await?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        if bytes.is_empty() {
            // Probe only: address ACKed, send STOP
            stop_and_cleanup(regs);
            return Ok(());
        }

        for (i, byte) in bytes.iter().enumerate() {
            regs.sr().write(|w| w.set_te(true));
            regs.dbr().write(|w| w.set_data(*byte));
            if i == bytes.len() - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }
            self.async_wait_te(timeout).await?;
            if let Err(e) = check_nack(regs) {
                stop_and_cleanup(regs);
                return Err(e);
            }
        }

        self.async_wait_bus_idle(timeout).await?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }

    /// Async read
    pub async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        self.async_wait_bus_idle(timeout).await?;
        clear_all_flags(regs);

        // Send START + address (read)
        regs.dbr().write(|w| w.set_data((addr << 1) | 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        self.async_wait_te(timeout).await?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        let len = buffer.len();
        for (i, byte) in buffer.iter_mut().enumerate() {
            regs.sr().write(|w| w.set_rf(true));
            if i == len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }
            self.async_wait_rf(timeout).await?;
            *byte = regs.dbr().read().data();
        }

        self.async_wait_bus_idle(timeout).await?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }

    /// Async write then read (repeated START)
    pub async fn write_read(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        if write.is_empty() || read.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        self.async_wait_bus_idle(timeout).await?;
        clear_all_flags(regs);

        // Write phase
        regs.dbr().write(|w| w.set_data(addr << 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });
        self.async_wait_te(timeout).await?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        for byte in write.iter() {
            regs.sr().write(|w| w.set_te(true));
            regs.dbr().write(|w| w.set_data(*byte));
            regs.tcr().write(|w| w.set_tb(true));
            self.async_wait_te(timeout).await?;
            if let Err(e) = check_nack(regs) {
                stop_and_cleanup(regs);
                return Err(e);
            }
        }

        // Read phase (repeated START)
        regs.sr().write(|w| w.set_te(true));
        regs.dbr().write(|w| w.set_data((addr << 1) | 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });
        self.async_wait_te(timeout).await?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        let read_len = read.len();
        for (i, byte) in read.iter_mut().enumerate() {
            regs.sr().write(|w| w.set_rf(true));
            if i == read_len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }
            self.async_wait_rf(timeout).await?;
            *byte = regs.dbr().read().data();
        }

        self.async_wait_bus_idle(timeout).await?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }

    async fn async_wait_te(&self, timeout: Duration) -> Result<(), Error> {
        let regs = T::regs();
        let state = T::state();
        let start = Instant::now();

        // Check if already set
        if regs.sr().read().te() {
            return Ok(());
        }

        // Enable TE interrupt
        regs.ier().modify(|w| w.set_teie(true));

        poll_fn(|cx| {
            state.waker.register(cx.waker());
            if regs.sr().read().te() {
                return Poll::Ready(Ok(()));
            }
            if start.elapsed() > timeout {
                return Poll::Ready(Err(Error::Timeout));
            }
            regs.ier().modify(|w| w.set_teie(true));
            Poll::Pending
        })
        .await
    }

    async fn async_wait_rf(&self, timeout: Duration) -> Result<(), Error> {
        let regs = T::regs();
        let state = T::state();
        let start = Instant::now();

        if regs.sr().read().rf() {
            return Ok(());
        }

        regs.ier().modify(|w| w.set_rfie(true));

        poll_fn(|cx| {
            state.waker.register(cx.waker());
            if regs.sr().read().rf() {
                return Poll::Ready(Ok(()));
            }
            if start.elapsed() > timeout {
                return Poll::Ready(Err(Error::Timeout));
            }
            regs.ier().modify(|w| w.set_rfie(true));
            Poll::Pending
        })
        .await
    }

    async fn async_wait_bus_idle(&self, timeout: Duration) -> Result<(), Error> {
        let regs = T::regs();
        let start = Instant::now();

        loop {
            if !regs.sr().read().ub() {
                return Ok(());
            }
            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }
            embassy_futures::yield_now().await;
        }
    }
}

// Common initialization and blocking implementations
impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    /// Initialize I2C hardware registers.
    /// Matches rcc-v2 (known working) init sequence.
    fn init(config: Config) {
        let i2c_clk = T::frequency().expect("I2C clock not configured");
        let (lcr_val, wcr_val) = compute_timing(i2c_clk, config.frequency);

        let regs = T::regs();

        // Module reset using modify to preserve register structure
        regs.cr().modify(|w| w.set_ur(true));
        crate::cortex_m_blocking_delay_us(100);
        regs.cr().modify(|w| w.set_ur(false));

        // Check bus state and attempt recovery if needed
        let bmr = regs.bmr().read();
        if bmr.scl() && bmr.sda() {
            // Both lines high - send clock cycles for bus recovery (slave might be stuck)
            regs.cr().modify(|w| w.set_rstreq(true));
            let start = Instant::now();
            while regs.cr().read().rstreq() {
                if start.elapsed() > Duration::from_millis(10) {
                    break;
                }
            }
        }

        let bus_mode = if config.frequency.0 <= 100_000 {
            BusMode::Standard
        } else {
            BusMode::Fast
        };

        // Set CR: mode + SCLE + IUE (matching rcc-v2 working init)
        regs.cr().write(|w| {
            w.set_mode(bus_mode as u8);
            w.set_scle(true);
            w.set_iue(true);
        });

        // Disable all interrupts
        regs.ier().write(|_| {});

        // Set reset cycle count
        regs.rccr().write(|w| w.set_rstcyc(9));

        // Set timing
        regs.lcr().write(|w| {
            w.set_flv(lcr_val);
            w.set_slv(0x1FF);
        });
        regs.wcr().write(|w| w.set_cnt(wcr_val));
    }

    /// Blocking write to an I2C device
    pub fn blocking_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        wait_bus_idle(regs, timeout)?;
        clear_all_flags(regs);

        // Send START + address (write)
        regs.dbr().write(|w| w.set_data(addr << 1));

        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        wait_te(regs, timeout)?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        if bytes.is_empty() {
            // Probe only: address ACKed, send STOP
            stop_and_cleanup(regs);
            return Ok(());
        }

        // Send data bytes
        for (i, byte) in bytes.iter().enumerate() {
            // Clear TE flag before next byte
            regs.sr().write(|w| w.set_te(true));

            regs.dbr().write(|w| w.set_data(*byte));
            if i == bytes.len() - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }

            wait_te(regs, timeout)?;
            if let Err(e) = check_nack(regs) {
                stop_and_cleanup(regs);
                return Err(e);
            }
        }

        wait_bus_idle(regs, timeout)?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }

    /// Blocking read from an I2C device
    pub fn blocking_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        wait_bus_idle(regs, timeout)?;
        clear_all_flags(regs);

        // Send START + address (read)
        regs.dbr().write(|w| w.set_data((addr << 1) | 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        wait_te(regs, timeout)?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        let len = buffer.len();
        for (i, byte) in buffer.iter_mut().enumerate() {
            // Clear RF flag
            regs.sr().write(|w| w.set_rf(true));

            if i == len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }

            wait_rf(regs, timeout)?;
            *byte = regs.dbr().read().data();
        }

        wait_bus_idle(regs, timeout)?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }

    /// Blocking write then read (combined with repeated START)
    pub fn blocking_write_read(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        if write.is_empty() || read.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        wait_bus_idle(regs, timeout)?;
        clear_all_flags(regs);

        // Write phase
        regs.dbr().write(|w| w.set_data(addr << 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });
        wait_te(regs, timeout)?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        for byte in write.iter() {
            regs.sr().write(|w| w.set_te(true));
            regs.dbr().write(|w| w.set_data(*byte));
            regs.tcr().write(|w| w.set_tb(true));
            wait_te(regs, timeout)?;
            if let Err(e) = check_nack(regs) {
                stop_and_cleanup(regs);
                return Err(e);
            }
        }

        // Read phase (repeated START)
        regs.sr().write(|w| w.set_te(true));
        regs.dbr().write(|w| w.set_data((addr << 1) | 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });
        wait_te(regs, timeout)?;
        if let Err(e) = check_nack(regs) {
            stop_and_cleanup(regs);
            return Err(e);
        }

        let read_len = read.len();
        for (i, byte) in read.iter_mut().enumerate() {
            regs.sr().write(|w| w.set_rf(true));
            if i == read_len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }
            wait_rf(regs, timeout)?;
            *byte = regs.dbr().read().data();
        }

        wait_bus_idle(regs, timeout)?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }
}

// Helper functions - matching rcc-v2 working logic

fn wait_bus_idle(regs: Regs, timeout: Duration) -> Result<(), Error> {
    let start = Instant::now();
    while regs.sr().read().ub() {
        if start.elapsed() > timeout {
            return Err(Error::Timeout);
        }
    }
    Ok(())
}

/// Wait for TE (Transmit Empty) flag. Simple poll, no error checking in loop.
fn wait_te(regs: Regs, timeout: Duration) -> Result<(), Error> {
    let start = Instant::now();
    while !regs.sr().read().te() {
        if start.elapsed() > timeout {
            return Err(Error::Timeout);
        }
    }
    Ok(())
}

/// Wait for RF (Receive Full) flag.
fn wait_rf(regs: Regs, timeout: Duration) -> Result<(), Error> {
    let start = Instant::now();
    while !regs.sr().read().rf() {
        if start.elapsed() > timeout {
            return Err(Error::Timeout);
        }
    }
    Ok(())
}

/// Check NACK flag after transfer. Uses SR.NACK (matching rcc-v2).
fn check_nack(regs: Regs) -> Result<(), Error> {
    if regs.sr().read().nack() {
        Err(Error::Nack)
    } else {
        Ok(())
    }
}

/// Release the bus by sending STOP, waiting briefly, then forcing a module reset
/// if the bus doesn't go idle. This handles cases where STOP alone doesn't work
/// (e.g., after address-only probe where device ACKed but no data follows).
///
/// Note: module reset (UR) preserves LCR/WCR/RCCR register values, but clears
/// CR (MODE, SCLE, IUE). The CR is restored after reset.
fn stop_and_cleanup(regs: Regs) {
    regs.tcr().write(|w| w.set_stop(true));
    // Try to wait for bus idle
    if wait_bus_idle(regs, Duration::from_millis(5)).is_err() {
        // STOP didn't complete - force module reset and restore CR
        let cr_saved = regs.cr().read();
        regs.cr().modify(|w| w.set_ur(true));
        crate::cortex_m_blocking_delay_us(10);
        regs.cr().modify(|w| w.set_ur(false));
        // Restore CR (MODE, SCLE) but with IUE=false
        regs.cr().write(|w| {
            w.set_mode(cr_saved.mode());
            w.set_scle(cr_saved.scle());
            // IUE intentionally left false
        });
    } else {
        regs.cr().modify(|w| w.set_iue(false));
    }
    clear_all_flags(regs);
}

/// Clear all W1C status flags.
fn clear_all_flags(regs: Regs) {
    regs.sr().write(|w| {
        w.set_te(true);
        w.set_nack(true);
        w.set_rf(true);
        w.set_bed(true);
        w.set_ald(true);
        w.set_ssd(true);
        w.set_sad(true);
        w.set_msd(true);
        w.set_dmadone(true);
    });
}

/// Compute LCR and WCR timing values from clock and target frequency.
fn compute_timing(i2c_clk: Hertz, target: Hertz) -> (u16, u8) {
    let dnf = 0u32;
    let flv = ((i2c_clk.0 + (target.0 / 2)) / target.0 - dnf - 7 + 1) / 2;
    let flv = flv.min(0x1FF);

    let cnt = flv / 2;
    let wcr = if cnt < 3 { 0 } else { (cnt - 3).min(0xFF) };

    (flv as u16, wcr as u8)
}

// --- embedded-hal v1 traits ---

impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        match *self {
            Error::Bus => embedded_hal_1::i2c::ErrorKind::Bus,
            Error::Arbitration => embedded_hal_1::i2c::ErrorKind::ArbitrationLoss,
            Error::Nack => {
                embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Unknown)
            }
            Error::Overrun => embedded_hal_1::i2c::ErrorKind::Overrun,
            _ => embedded_hal_1::i2c::ErrorKind::Other,
        }
    }
}

impl<T: Instance, M: Mode> embedded_hal_1::i2c::ErrorType for I2c<'_, T, M> {
    type Error = Error;
}

impl<T: Instance, M: Mode> embedded_hal_1::i2c::I2c for I2c<'_, T, M> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));
        wait_bus_idle(regs, timeout)?;
        clear_all_flags(regs);

        let last_op_idx = operations.len() - 1;

        for (op_idx, op) in operations.iter_mut().enumerate() {
            let is_last_op = op_idx == last_op_idx;

            match op {
                embedded_hal_1::i2c::Operation::Write(bytes) => {
                    // Clear TE before sending address (first op already cleared by clear_all_flags)
                    if op_idx > 0 {
                        regs.sr().write(|w| w.set_te(true));
                    }
                    regs.dbr().write(|w| w.set_data(address << 1));
                    regs.tcr().write(|w| {
                        w.set_tb(true);
                        w.set_start(true);
                    });
                    wait_te(regs, timeout)?;
                    if let Err(e) = check_nack(regs) {
                        stop_and_cleanup(regs);
                        return Err(e);
                    }

                    if bytes.is_empty() {
                        if is_last_op {
                            stop_and_cleanup(regs);
                        }
                        continue;
                    }

                    let last_byte_idx = bytes.len() - 1;
                    for (i, byte) in bytes.iter().enumerate() {
                        regs.sr().write(|w| w.set_te(true));
                        regs.dbr().write(|w| w.set_data(*byte));
                        if is_last_op && i == last_byte_idx {
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_stop(true);
                            });
                        } else {
                            regs.tcr().write(|w| w.set_tb(true));
                        }
                        wait_te(regs, timeout)?;
                        if let Err(e) = check_nack(regs) {
                            stop_and_cleanup(regs);
                            return Err(e);
                        }
                    }
                }
                embedded_hal_1::i2c::Operation::Read(buffer) => {
                    if buffer.is_empty() {
                        continue;
                    }
                    // Clear TE before sending read address
                    regs.sr().write(|w| w.set_te(true));
                    regs.dbr().write(|w| w.set_data((address << 1) | 1));
                    regs.tcr().write(|w| {
                        w.set_tb(true);
                        w.set_start(true);
                    });
                    wait_te(regs, timeout)?;
                    if let Err(e) = check_nack(regs) {
                        stop_and_cleanup(regs);
                        return Err(e);
                    }

                    let last_byte_idx = buffer.len() - 1;
                    for (i, byte) in buffer.iter_mut().enumerate() {
                        regs.sr().write(|w| w.set_rf(true));
                        let is_last_byte = i == last_byte_idx;
                        if is_last_op && is_last_byte {
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_stop(true);
                                w.set_nack(true);
                            });
                        } else if is_last_byte {
                            // Last byte of non-last read: NACK to end read segment
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_nack(true);
                            });
                        } else {
                            regs.tcr().write(|w| w.set_tb(true));
                        }
                        wait_rf(regs, timeout)?;
                        *byte = regs.dbr().read().data();
                    }
                }
            }
        }

        wait_bus_idle(regs, timeout)?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }
}

impl<T: Instance> embedded_hal_async::i2c::I2c for I2c<'_, T, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let regs = T::regs();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));
        self.async_wait_bus_idle(timeout).await?;
        clear_all_flags(regs);

        let last_op_idx = operations.len() - 1;

        for (op_idx, op) in operations.iter_mut().enumerate() {
            let is_last_op = op_idx == last_op_idx;

            match op {
                embedded_hal_1::i2c::Operation::Write(bytes) => {
                    if op_idx > 0 {
                        regs.sr().write(|w| w.set_te(true));
                    }
                    regs.dbr().write(|w| w.set_data(address << 1));
                    regs.tcr().write(|w| {
                        w.set_tb(true);
                        w.set_start(true);
                    });
                    self.async_wait_te(timeout).await?;
                    if let Err(e) = check_nack(regs) {
                        stop_and_cleanup(regs);
                        return Err(e);
                    }

                    if bytes.is_empty() {
                        if is_last_op {
                            stop_and_cleanup(regs);
                        }
                        continue;
                    }

                    let last_byte_idx = bytes.len() - 1;
                    for (i, byte) in bytes.iter().enumerate() {
                        regs.sr().write(|w| w.set_te(true));
                        regs.dbr().write(|w| w.set_data(*byte));
                        if is_last_op && i == last_byte_idx {
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_stop(true);
                            });
                        } else {
                            regs.tcr().write(|w| w.set_tb(true));
                        }
                        self.async_wait_te(timeout).await?;
                        if let Err(e) = check_nack(regs) {
                            stop_and_cleanup(regs);
                            return Err(e);
                        }
                    }
                }
                embedded_hal_1::i2c::Operation::Read(buffer) => {
                    if buffer.is_empty() {
                        continue;
                    }
                    regs.sr().write(|w| w.set_te(true));
                    regs.dbr().write(|w| w.set_data((address << 1) | 1));
                    regs.tcr().write(|w| {
                        w.set_tb(true);
                        w.set_start(true);
                    });
                    self.async_wait_te(timeout).await?;
                    if let Err(e) = check_nack(regs) {
                        stop_and_cleanup(regs);
                        return Err(e);
                    }

                    let last_byte_idx = buffer.len() - 1;
                    for (i, byte) in buffer.iter_mut().enumerate() {
                        regs.sr().write(|w| w.set_rf(true));
                        let is_last_byte = i == last_byte_idx;
                        if is_last_op && is_last_byte {
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_stop(true);
                                w.set_nack(true);
                            });
                        } else if is_last_byte {
                            regs.tcr().write(|w| {
                                w.set_tb(true);
                                w.set_nack(true);
                            });
                        } else {
                            regs.tcr().write(|w| w.set_tb(true));
                        }
                        self.async_wait_rf(timeout).await?;
                        *byte = regs.dbr().read().data();
                    }
                }
            }
        }

        self.async_wait_bus_idle(timeout).await?;
        regs.cr().modify(|w| w.set_iue(false));
        Ok(())
    }
}

// --- Instance infrastructure ---

struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

#[allow(private_interfaces)]
pub(crate) trait SealedInstance: crate::rcc::RccEnableReset + crate::rcc::RccGetFreq {
    fn regs() -> Regs;
    fn state() -> &'static State;
}

/// I2C peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
dma_trait!(Dma, Instance);

macro_rules! impl_i2c {
    ($inst:ident, $irq:ident) => {
        #[allow(private_interfaces)]
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> Regs {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_i2c!(I2C1, I2C1);
impl_i2c!(I2C2, I2C2);
impl_i2c!(I2C3, I2C3);
impl_i2c!(I2C4, I2C4);
