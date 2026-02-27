//! Universal Synchronous/Asynchronous Receiver Transmitter (USART, UART, LPUART)
#![macro_use]
#![warn(missing_docs)]

// The following code is modified from embassy-stm32 under MIT license
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU8, Ordering};
use core::task::Poll;

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::PeripheralRef;
use embassy_sync::waitqueue::AtomicWaker;
use futures_util::future::{select, Either};

use crate::dma::ChannelAndRequest;
use crate::gpio::{AfType, AnyPin, Pull, SealedPin};
use crate::interrupt::{self, typelevel::Interrupt as _, Interrupt};
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{rcc, Peripheral};

use crate::pac::usart::Usart as Regs;
use crate::pac::usart::{regs, vals};

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt(T::regs(), T::state())
    }
}

unsafe fn on_interrupt(r: Regs, s: &'static State) {
    let (sr, cr1, cr3) = (r.isr().read(), r.cr1().read(), r.cr3().read());

    let has_errors = (sr.pe() && cr1.peie()) || ((sr.fe() || sr.ore()) && cr3.eie());
    if has_errors {
        // clear all interrupts and DMA Rx Request
        r.cr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // disable parity interrupt
            w.set_peie(false);
            // disable idle line interrupt
            w.set_idleie(false);
        });
        r.cr3().modify(|w| {
            // disable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(false);
            // disable DMA Rx Request
            w.set_dmar(false);
        });
    } else if cr1.idleie() && sr.idle() {
        // IDLE detected: no more data will come
        r.cr1().modify(|w| {
            // disable idle line detection
            w.set_idleie(false);
        });
    } else if cr1.tcie() && sr.tc() {
        // Transmission complete detected
        r.cr1().modify(|w| {
            // disable Transmission complete interrupt
            w.set_tcie(false);
        });
    } else if cr1.rxneie() {
        // We cannot check the RXNE flag as it is auto-cleared by the DMA controller

        // It is up to the listener to determine if this in fact was a RX event and disable the RXNE detection
    } else {
        return;
    }

    compiler_fence(Ordering::SeqCst);
    s.rx_waker.wake();
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Number of data bits
pub enum DataBits {
    /// 6 Data Bits
    DataBits6,
    /// 7 Data Bits
    DataBits7,
    /// 8 Data Bits
    DataBits8,
    /// 9 Data Bits
    DataBits9,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Parity
pub enum Parity {
    /// No parity
    ParityNone,
    /// Even Parity
    ParityEven,
    /// Odd Parity
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Number of stop bits
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    // #[doc = "0.5 stop bits"]
    // STOP0P5,
    #[doc = "2 stop bits"]
    STOP2,
    // #[doc = "1.5 stop bits"]
    // STOP1P5,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Enables or disables receiver so written data are read back in half-duplex mode
pub enum HalfDuplexReadback {
    /// Disables receiver so written data are not read back
    NoReadback,
    /// Enables receiver so written data are read back
    Readback,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Duplex mode
pub enum Duplex {
    /// Full duplex
    Full,
    /// Half duplex with possibility to read back written data
    Half(HalfDuplexReadback),
}

impl Duplex {
    /// Returns true if half-duplex
    fn is_half(&self) -> bool {
        matches!(self, Duplex::Half(_))
    }
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Config Error
pub enum ConfigError {
    /// Baudrate too low
    BaudrateTooLow,
    /// Baudrate too high
    BaudrateTooHigh,
    /// Rx or Tx not enabled
    RxOrTxNotEnabled,
    /// Data bits and parity combination not supported
    DataParityNotSupported,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Config
pub struct Config {
    /// Baud rate
    pub baudrate: u32,
    /// Number of data bits
    pub data_bits: DataBits,
    /// Number of stop bits
    pub stop_bits: StopBits,
    /// Parity type
    pub parity: Parity,

    /// If true: on a read-like method, if there is a latent error pending,
    /// the read will abort and the error will be reported and cleared
    ///
    /// If false: the error is ignored and cleared
    pub _detect_previous_overrun: bool,

    /// Set this to true if the line is considered noise free.
    /// This will increase the receiver’s tolerance to clock deviations,
    /// but will effectively disable noise detection.
    pub assume_noise_free: bool,

    /// Set the pull configuration for the RX pin.
    pub rx_pull: Pull,

    // private: set by new_half_duplex, not by the user.
    duplex: Duplex,
}

impl Config {
    fn tx_af(&self) -> AfType {
        AfType::new(Pull::Up)
    }

    fn rx_af(&self) -> AfType {
        AfType::new(self.rx_pull)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
            // historical behavior
            _detect_previous_overrun: false,
            assume_noise_free: false,
            rx_pull: Pull::None,
            duplex: Duplex::Full,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
/// Half duplex IO mode
pub struct HalfDuplexConfig {
    // TODO
    pull: Pull,
}

impl HalfDuplexConfig {
    fn af_type(self) -> AfType {
        AfType::new(self.pull)
    }
}

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Framing => write!(f, "framing error"),
            Error::Noise => write!(f, "noise error"),
            Error::Overrun => write!(f, "RX buffer overrun"),
            Error::Parity => write!(f, "parity check error"),
            Error::BufferTooLong => write!(f, "buffer too large for DMA"),
        }
    }
}

enum ReadCompletionEvent {
    // DMA Read transfer completed first
    DmaCompleted,
    // Idle line detected first
    Idle(usize),
}

/// Bidirectional UART Driver, which acts as a combination of [`UartTx`] and [`UartRx`].
///
/// ### Notes on [`embedded_io::Read`]
///
/// `embedded_io::Read` requires guarantees that the base [`UartRx`] cannot provide.
///
/// See [`UartRx`] for more details, and see [`BufferedUart`] and [`RingBufferedUartRx`]
/// as alternatives that do provide the necessary guarantees for `embedded_io::Read`.
pub struct Uart<'d, T: Instance, M: Mode> {
    tx: UartTx<'d, T, M>,
    rx: UartRx<'d, T, M>,
}

impl<'d, T: Instance, M: Mode> SetConfig for Uart<'d, T, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.tx.set_config(config)?;
        self.rx.set_config(config)
    }
}

/// Tx-only UART Driver.
///
/// Can be obtained from [`Uart::split`], or can be constructed independently,
/// if you do not need the receiving half of the driver.
pub struct UartTx<'d, T: Instance, M: Mode> {
    state: &'static State,
    kernel_clock: Hertz,
    tx: Option<PeripheralRef<'d, AnyPin>>,
    cts: Option<PeripheralRef<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    duplex: Duplex,
    _phantom: PhantomData<(T, M)>,
}

impl<'d, T: Instance, M: Mode> SetConfig for UartTx<'d, T, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

/// Rx-only UART Driver.
///
/// Can be obtained from [`Uart::split`], or can be constructed independently,
/// if you do not need the transmitting half of the driver.
///
/// ### Notes on [`embedded_io::Read`]
///
/// `embedded_io::Read` requires guarantees that this struct cannot provide:
///
/// - Any data received between calls to [`UartRx::read`] or [`UartRx::blocking_read`]
/// will be thrown away, as `UartRx` is unbuffered.
/// Users of `embedded_io::Read` are likely to not expect this behavior
/// (for instance if they read multiple small chunks in a row).
/// - [`UartRx::read`] and [`UartRx::blocking_read`] only return once the entire buffer has been
/// filled, whereas `embedded_io::Read` requires us to fill the buffer with what we already
/// received, and only block/wait until the first byte arrived.
/// <br />
/// While [`UartRx::read_until_idle`] does return early, it will still eagerly wait for data until
/// the buffer is full or no data has been transmitted in a while,
/// which may not be what users of `embedded_io::Read` expect.
///
/// [`UartRx::into_ring_buffered`] can be called to equip `UartRx` with a buffer,
/// that it can then use to store data received between calls to `read`,
/// provided you are using DMA already.
///
/// Alternatively, you can use [`BufferedUartRx`], which is interrupt-based and which can also
/// store data received between calls.
///
/// Also see [this github comment](https://github.com/embassy-rs/embassy/pull/2185#issuecomment-1810047043).
pub struct UartRx<'d, T: Instance, M: Mode> {
    state: &'static State,
    kernel_clock: Hertz,
    rx: Option<PeripheralRef<'d, AnyPin>>,
    rts: Option<PeripheralRef<'d, AnyPin>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _detect_previous_overrun: bool,
    _phantom: PhantomData<(T, M)>,
}

impl<'d, T: Instance, M: Mode> SetConfig for UartRx<'d, T, M> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.set_config(config)
    }
}

impl<'d, T: Instance> UartTx<'d, T, Async> {
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, AfType::new(Pull::Up)),
            None,
            new_dma!(tx_dma),
            config,
        )
    }

    /// Create a new tx-only UART with a clear-to-send pin
    pub fn new_with_cts(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, AfType::new(Pull::Up)),
            new_pin!(cts, AfType::new(Pull::None)),
            new_dma!(tx_dma),
            config,
        )
    }

    /// Initiate an asynchronous UART write
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let r = T::regs();

        half_duplex_set_rx_tx_before_write(
            &r,
            self.duplex == Duplex::Half(HalfDuplexReadback::Readback),
        );

        let ch = self.tx_dma.as_mut().unwrap();
        r.cr3().modify(|reg| {
            reg.set_dmat(true);
        });
        // If we don't assign future to a variable, the data register pointer
        // is held across an await and makes the future non-Send.
        let transfer = unsafe { ch.write(buffer, r.tdr().as_ptr() as _, Default::default()) };
        transfer.await;
        Ok(())
    }

    /// Wait until transmission complete
    pub async fn flush(&mut self) -> Result<(), Error> {
        flush::<T>(&self.state).await
    }
}

impl<'d, T: Instance> UartTx<'d, T, Blocking> {
    /// Create a new blocking tx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, AfType::new(Pull::Up)),
            None,
            None,
            config,
        )
    }

    /// Create a new blocking tx-only UART with a clear-to-send pin
    pub fn new_blocking_with_cts(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(tx, AfType::new(Pull::Up)),
            new_pin!(cts, AfType::new(config.rx_pull)),
            None,
            config,
        )
    }
}

impl<'d, T: Instance, M: Mode> UartTx<'d, T, M> {
    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        tx: Option<PeripheralRef<'d, AnyPin>>,
        cts: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            state: T::state(),
            kernel_clock: T::frequency().unwrap(),
            tx,
            cts,
            tx_dma,
            duplex: config.duplex,
            _phantom: PhantomData,
        };
        this.enable_and_configure(&config)?;
        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let state = self.state;
        state.tx_rx_refcount.store(1, Ordering::Relaxed);

        rcc::enable_and_reset::<T>();

        T::regs().cr3().modify(|w| {
            w.set_ctse(self.cts.is_some());
        });
        configure::<T>(self.kernel_clock, config, false, true)?;

        Ok(())
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(self.kernel_clock, config)
    }

    /// Perform a blocking UART write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let r = T::regs();

        half_duplex_set_rx_tx_before_write(
            &r,
            self.duplex == Duplex::Half(HalfDuplexReadback::Readback),
        );

        for &b in buffer {
            while !r.isr().read().txe() {}
            r.tdr().write_value(regs::Tdr(b as _));
        }
        Ok(())
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        blocking_flush::<T>()
    }

    /// Send break character
    pub fn send_break(&self) {
        send_break(&T::regs());
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate::<T>(self.kernel_clock, baudrate)
    }
}

/// Wait until transmission complete
async fn flush<T: Instance>(state: &State) -> Result<(), Error> {
    let r = T::regs();
    if r.cr1().read().te() && !r.isr().read().tc() {
        r.cr1().modify(|w| {
            // enable Transmission Complete interrupt
            w.set_tcie(true);
        });

        compiler_fence(Ordering::SeqCst);

        // future which completes when Transmission complete is detected
        let abort = poll_fn(move |cx| {
            state.rx_waker.register(cx.waker());

            let sr = r.isr().read();
            if sr.tc() {
                // Transmission complete detected
                return Poll::Ready(());
            }

            Poll::Pending
        });

        abort.await;
    }

    Ok(())
}

fn blocking_flush<T: Instance>() -> Result<(), Error> {
    let r = T::regs();
    if r.cr1().read().te() {
        while !r.isr().read().tc() {}
    }

    Ok(())
}

/// Send break character
pub fn send_break(regs: &Regs) {
    // Busy wait until previous break has been sent
    while regs.isr().read().sbkf() {}

    // Send break right after completing the current character transmission
    regs.rqr().write(|w| w.set_sbkrq(true));
}

/// Enable Transmitter and disable Receiver for Half-Duplex mode
/// In case of readback, keep Receiver enabled
fn half_duplex_set_rx_tx_before_write(r: &Regs, enable_readback: bool) {
    let mut cr1 = r.cr1().read();
    if r.cr3().read().hdsel() && !cr1.te() {
        cr1.set_te(true);
        cr1.set_re(enable_readback);
        r.cr1().write_value(cr1);
    }
}

impl<'d, T: Instance> UartRx<'d, T, Async> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::new(config.rx_pull)),
            None,
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_with_rts(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::new(config.rx_pull)),
            new_pin!(rts, AfType::new(Pull::Up)),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Initiate an asynchronous UART read
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.inner_read(buffer, false).await?;

        Ok(())
    }

    /// Initiate an asynchronous read with idle line detection enabled
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.inner_read(buffer, true).await
    }

    async fn inner_read_run(
        &mut self,
        buffer: &mut [u8],
        enable_idle_line_detection: bool,
    ) -> Result<ReadCompletionEvent, Error> {
        let r = T::regs();

        // Call flush for Half-Duplex mode if some bytes were written and flush was not called.
        // It prevents reading of bytes which have just been written.
        if r.cr3().read().hdsel() && r.cr1().read().te() {
            flush::<T>(&self.state).await?;

            // Disable Transmitter and enable Receiver after flush
            r.cr1().modify(|reg| {
                reg.set_re(true);
                reg.set_te(false);
            });
        }

        // make sure USART state is restored to neutral state when this future is dropped
        let on_drop = OnDrop::new(move || {
            // defmt::trace!("Clear all USART interrupts and DMA Read Request");
            // clear all interrupts and DMA Rx Request
            r.cr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // disable parity interrupt
                w.set_peie(false);
                // disable idle line interrupt
                w.set_idleie(false);
            });
            r.cr3().modify(|w| {
                // disable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(false);
                // disable DMA Rx Request
                w.set_dmar(false);
            });
        });

        let ch = self.rx_dma.as_mut().unwrap();

        let buffer_len = buffer.len();

        // Start USART DMA
        // will not do anything yet because DMAR is not yet set
        // future which will complete when DMA Read request completes
        let transfer = unsafe { ch.read(r.rdr().as_ptr() as _, buffer, Default::default()) };

        // clear ORE flag just before enabling DMA Rx Request: can be mandatory for the second transfer
        if !self._detect_previous_overrun {
            let sr = r.isr().read();
            // This read also clears the error and idle interrupt flags on v1.
            unsafe { r.rdr().as_ptr().read_volatile() };
            clear_interrupt_flags(r, sr);
        }

        r.cr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // enable parity interrupt if not ParityNone
            w.set_peie(w.pce());
        });

        r.cr3().modify(|w| {
            // enable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(true);
            // enable DMA Rx Request
            w.set_dmar(true);
        });

        compiler_fence(Ordering::SeqCst);

        // In case of errors already pending when reception started, interrupts may have already been raised
        // and lead to reception abortion (Overrun error for instance). In such a case, all interrupts
        // have been disabled in interrupt handler and DMA Rx Request has been disabled.

        let cr3 = r.cr3().read();

        if !cr3.dmar() {
            // something went wrong
            // because the only way to get this flag cleared is to have an interrupt

            // DMA will be stopped when transfer is dropped

            let sr = r.isr().read();
            // This read also clears the error and idle interrupt flags on v1.
            unsafe { r.rdr().as_ptr().read_volatile() };
            clear_interrupt_flags(r, sr);

            if sr.pe() {
                return Err(Error::Parity);
            }
            if sr.fe() {
                return Err(Error::Framing);
            }
            if sr.nf() {
                return Err(Error::Noise);
            }
            if sr.ore() {
                return Err(Error::Overrun);
            }

            unreachable!();
        }

        if enable_idle_line_detection {
            // clear idle flag
            let sr = r.isr().read();
            // This read also clears the error and idle interrupt flags on v1.
            unsafe { r.rdr().as_ptr().read_volatile() };
            clear_interrupt_flags(r, sr);

            // enable idle interrupt
            r.cr1().modify(|w| {
                w.set_idleie(true);
            });
        }

        compiler_fence(Ordering::SeqCst);

        // future which completes when idle line or error is detected
        let s = self.state;
        let abort = poll_fn(move |cx| {
            s.rx_waker.register(cx.waker());

            let sr = r.isr().read();

            // This read also clears the error and idle interrupt flags on v1.
            unsafe { r.rdr().as_ptr().read_volatile() };
            clear_interrupt_flags(r, sr);

            if enable_idle_line_detection {
                // enable idle interrupt
                r.cr1().modify(|w| {
                    w.set_idleie(true);
                });
            }

            compiler_fence(Ordering::SeqCst);

            let has_errors = sr.pe() || sr.fe() || sr.nf() || sr.ore();

            if has_errors {
                // all Rx interrupts and Rx DMA Request have already been cleared in interrupt handler

                if sr.pe() {
                    return Poll::Ready(Err(Error::Parity));
                }
                if sr.fe() {
                    return Poll::Ready(Err(Error::Framing));
                }
                if sr.nf() {
                    return Poll::Ready(Err(Error::Noise));
                }
                if sr.ore() {
                    return Poll::Ready(Err(Error::Overrun));
                }
            }

            if enable_idle_line_detection && sr.idle() {
                // Idle line detected
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        });

        // wait for the first of DMA request or idle line detected to completes
        // select consumes its arguments
        // when transfer is dropped, it will stop the DMA request
        let r = match select(transfer, abort).await {
            // DMA transfer completed first
            Either::Left(((), _)) => Ok(ReadCompletionEvent::DmaCompleted),

            // Idle line detected first
            Either::Right((Ok(()), transfer)) => Ok(ReadCompletionEvent::Idle(
                buffer_len - transfer.get_remaining_transfers() as usize,
            )),

            // error occurred
            Either::Right((Err(e), _)) => Err(e),
        };

        drop(on_drop);

        r
    }

    async fn inner_read(
        &mut self,
        buffer: &mut [u8],
        enable_idle_line_detection: bool,
    ) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Ok(0);
        } else if buffer.len() > 0xFFFF {
            return Err(Error::BufferTooLong);
        }

        let buffer_len = buffer.len();

        // wait for DMA to complete or IDLE line detection if requested
        let res = self
            .inner_read_run(buffer, enable_idle_line_detection)
            .await;

        match res {
            Ok(ReadCompletionEvent::DmaCompleted) => Ok(buffer_len),
            Ok(ReadCompletionEvent::Idle(n)) => Ok(n),
            Err(e) => Err(e),
        }
    }
}

impl<'d, T: Instance> UartRx<'d, T, Blocking> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::new(config.rx_pull)),
            None,
            None,
            config,
        )
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_blocking_with_rts(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::new(config.rx_pull)),
            new_pin!(rts, AfType::new(Pull::Up)),
            None,
            config,
        )
    }
}

impl<'d, T: Instance, M: Mode> UartRx<'d, T, M> {
    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        rx: Option<PeripheralRef<'d, AnyPin>>,
        rts: Option<PeripheralRef<'d, AnyPin>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let mut this = Self {
            _phantom: PhantomData,
            state: T::state(),
            kernel_clock: T::frequency().unwrap(),
            rx,
            rts,
            rx_dma,
            _detect_previous_overrun: config._detect_previous_overrun,
        };
        this.enable_and_configure(&config)?;
        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let state = self.state;
        state.tx_rx_refcount.store(1, Ordering::Relaxed);

        rcc::enable_and_reset::<T>();

        T::regs().cr3().write(|w| {
            w.set_rtse(self.rts.is_some());
        });
        configure::<T>(self.kernel_clock, &config, true, false)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(())
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(self.kernel_clock, config)
    }

    fn check_rx_flags(&mut self) -> Result<bool, Error> {
        let r = T::regs();
        let sr = r.isr().read();
        if sr.pe() {
            r.icr().write(|w| w.set_pecf(true));
            return Err(Error::Parity);
        } else if sr.fe() {
            r.icr().write(|w| w.set_fecf(true));
            return Err(Error::Framing);
        } else if sr.ore() {
            r.icr().write(|w| w.set_orecf(true));
            return Err(Error::Overrun);
        }
        Ok(sr.rxne())
    }

    /// Read a single u8 if there is one available, otherwise return WouldBlock
    pub(crate) fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let r = T::regs();
        if self.check_rx_flags()? {
            Ok(unsafe { r.rdr().as_ptr().read_volatile().0 as u8 })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();

        // Call flush for Half-Duplex mode if some bytes were written and flush was not called.
        // It prevents reading of bytes which have just been written.
        if r.cr3().read().hdsel() && r.cr1().read().te() {
            blocking_flush::<T>()?;

            // Disable Transmitter and enable Receiver after flush
            r.cr1().modify(|reg| {
                reg.set_re(true);
                reg.set_te(false);
            });
        }

        for b in buffer {
            while !self.check_rx_flags()? {}
            unsafe { *b = r.rdr().as_ptr().read_volatile().0 as u8 }
        }
        Ok(())
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        set_baudrate::<T>(self.kernel_clock, baudrate)
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartTx<'d, T, M> {
    fn drop(&mut self) {
        self.tx.as_ref().map(|x| x.set_as_disconnected());
        self.cts.as_ref().map(|x| x.set_as_disconnected());
        drop_tx_rx::<T>(self.state);
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartRx<'d, T, M> {
    fn drop(&mut self) {
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
        drop_tx_rx::<T>(self.state);
    }
}

fn drop_tx_rx<T: Instance>(state: &State) {
    // We cannot use atomic subtraction here, because it's not supported for all targets
    let is_last_drop = critical_section::with(|_| {
        let refcount = state.tx_rx_refcount.load(Ordering::Relaxed);
        assert!(refcount >= 1);
        state.tx_rx_refcount.store(refcount - 1, Ordering::Relaxed);
        refcount == 1
    });
    if is_last_drop {
        rcc::disable::<T>();
    }
}

impl<'d, T: Instance> Uart<'d, T, Async> {
    /// Create a new bidirectional UART
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_af()),
            new_pin!(tx, config.tx_af()),
            None,
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_with_rtscts(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_af()),
            new_pin!(tx, config.tx_af()),
            new_pin!(rts, AfType::new(Pull::Up)),
            new_pin!(cts, AfType::new(Pull::None)),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a single-wire half-duplex Uart transceiver on a single Tx pin.
    ///
    /// See [`new_half_duplex_on_rx`][`Self::new_half_duplex_on_rx`] if you would prefer to use an Rx pin
    /// (when it is available for your chip). There is no functional difference between these methods, as both
    /// allow bidirectional communication.
    ///
    /// The TX pin is always released when no data is transmitted. Thus, it acts as a standard
    /// I/O in idle or in reception. It means that the I/O must be configured so that TX is
    /// configured as alternate function open-drain with an external pull-up
    /// Apart from this, the communication protocol is similar to normal USART mode. Any conflict
    /// on the line must be managed by software (for instance by using a centralized arbiter).
    #[doc(alias("HDSEL"))]
    pub fn new_half_duplex(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        mut config: Config,
        readback: HalfDuplexReadback,
        half_duplex: HalfDuplexConfig,
    ) -> Result<Self, ConfigError> {
        config.duplex = Duplex::Half(readback);

        Self::new_inner(
            peri,
            None,
            new_pin!(tx, half_duplex.af_type()),
            None,
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    // TODO: Do we have this?
    // embassy_stm32: #[cfg(not(any(usart_v1, usart_v2)))]
    //
    /// Create a single-wire half-duplex Uart transceiver on a single Rx pin.
    ///
    /// See [`new_half_duplex`][`Self::new_half_duplex`] if you would prefer to use an Tx pin.
    /// There is no functional difference between these methods, as both allow bidirectional communication.
    ///
    /// The pin is always released when no data is transmitted. Thus, it acts as a standard
    /// I/O in idle or in reception.
    /// Apart from this, the communication protocol is similar to normal USART mode. Any conflict
    /// on the line must be managed by software (for instance by using a centralized arbiter).
    #[doc(alias("HDSEL"))]
    pub fn new_half_duplex_on_rx(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        mut config: Config,
        readback: HalfDuplexReadback,
        half_duplex: HalfDuplexConfig,
    ) -> Result<Self, ConfigError> {
        config.duplex = Duplex::Half(readback);

        Self::new_inner(
            peri,
            None,
            None,
            new_pin!(rx, half_duplex.af_type()),
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Perform an asynchronous write
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.write(buffer).await
    }

    /// Wait until transmission complete
    pub async fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush().await
    }

    /// Perform an asynchronous read into `buffer`
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.read(buffer).await
    }

    /// Perform an an asynchronous read with idle line detection enabled
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_until_idle(buffer).await
    }
}

impl<'d, T: Instance> Uart<'d, T, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_af()),
            new_pin!(tx, config.tx_af()),
            None,
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_blocking_with_rtscts(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        Self::new_inner(
            peri,
            new_pin!(rx, config.rx_af()),
            new_pin!(tx, config.tx_af()),
            new_pin!(rts, AfType::new(Pull::Up)),
            new_pin!(cts, AfType::new(Pull::None)),
            None,
            None,
            config,
        )
    }

    /// Create a single-wire half-duplex Uart transceiver on a single Tx pin.
    ///
    /// See [`new_half_duplex_on_rx`][`Self::new_half_duplex_on_rx`] if you would prefer to use an Rx pin
    /// (when it is available for your chip). There is no functional difference between these methods, as both
    /// allow bidirectional communication.
    ///
    /// The pin is always released when no data is transmitted. Thus, it acts as a standard
    /// I/O in idle or in reception.
    /// Apart from this, the communication protocol is similar to normal USART mode. Any conflict
    /// on the line must be managed by software (for instance by using a centralized arbiter).
    #[doc(alias("HDSEL"))]
    pub fn new_blocking_half_duplex(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxdPin<T>> + 'd,
        mut config: Config,
        readback: HalfDuplexReadback,
        half_duplex: HalfDuplexConfig,
    ) -> Result<Self, ConfigError> {
        config.duplex = Duplex::Half(readback);

        Self::new_inner(
            peri,
            None,
            new_pin!(tx, half_duplex.af_type()),
            None,
            None,
            None,
            None,
            config,
        )
    }

    /// Create a single-wire half-duplex Uart transceiver on a single Rx pin.
    ///
    /// See [`new_half_duplex`][`Self::new_half_duplex`] if you would prefer to use an Tx pin.
    /// There is no functional difference between these methods, as both allow bidirectional communication.
    ///
    /// The pin is always released when no data is transmitted. Thus, it acts as a standard
    /// I/O in idle or in reception.
    /// Apart from this, the communication protocol is similar to normal USART mode. Any conflict
    /// on the line must be managed by software (for instance by using a centralized arbiter).
    #[doc(alias("HDSEL"))]
    pub fn new_blocking_half_duplex_on_rx(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxdPin<T>> + 'd,
        mut config: Config,
        readback: HalfDuplexReadback,
        half_duplex: HalfDuplexConfig,
    ) -> Result<Self, ConfigError> {
        config.duplex = Duplex::Half(readback);

        Self::new_inner(
            peri,
            None,
            None,
            new_pin!(rx, half_duplex.af_type()),
            None,
            None,
            None,
            config,
        )
    }
}

impl<'d, T: Instance, M: Mode> Uart<'d, T, M> {
    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        rx: Option<PeripheralRef<'d, AnyPin>>,
        tx: Option<PeripheralRef<'d, AnyPin>>,
        rts: Option<PeripheralRef<'d, AnyPin>>,
        cts: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let state = T::state();
        let kernel_clock = T::frequency().unwrap();

        let mut this = Self {
            tx: UartTx {
                _phantom: PhantomData,
                state,
                kernel_clock,
                tx,
                cts,
                tx_dma,
                duplex: config.duplex,
            },
            rx: UartRx {
                _phantom: PhantomData,
                state,
                kernel_clock,
                rx,
                rts,
                rx_dma,
                _detect_previous_overrun: config._detect_previous_overrun,
            },
        };
        this.enable_and_configure(&config)?;
        Ok(this)
    }

    fn enable_and_configure(&mut self, config: &Config) -> Result<(), ConfigError> {
        let state = self.rx.state;
        state.tx_rx_refcount.store(2, Ordering::Relaxed);

        rcc::enable_and_reset::<T>();

        T::regs().cr3().write(|w| {
            w.set_rtse(self.rx.rts.is_some());
            w.set_ctse(self.tx.cts.is_some());
        });
        configure::<T>(self.rx.kernel_clock, config, true, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(())
    }

    /// Perform a blocking write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Read a single `u8` or return `WouldBlock`
    pub(crate) fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T, M>, UartRx<'d, T, M>) {
        (self.tx, self.rx)
    }

    /// Split the Uart into a transmitter and receiver by mutable reference,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split_ref(&mut self) -> (&mut UartTx<'d, T, M>, &mut UartRx<'d, T, M>) {
        (&mut self.tx, &mut self.rx)
    }

    /// Send break character
    pub fn send_break(&self) {
        self.tx.send_break();
    }

    /// Set baudrate
    pub fn set_baudrate(&self, baudrate: u32) -> Result<(), ConfigError> {
        self.tx.set_baudrate(baudrate)?;
        self.rx.set_baudrate(baudrate)?;
        Ok(())
    }
}

fn reconfigure<T: Instance>(kernel_clock: Hertz, config: &Config) -> Result<(), ConfigError> {
    T::Interrupt::disable();
    let r = T::regs();

    let cr = r.cr1().read();
    configure::<T>(kernel_clock, config, cr.re(), cr.te())?;

    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };

    Ok(())
}

fn calculate_brr(baud: u32, pclk: u32, presc: u32, mul: u32) -> u32 {
    // The calculation to be done to get the BRR is `mul * pclk / presc / baud`
    // To do this in 32-bit only we can't multiply `mul` and `pclk`
    let clock = pclk / presc;

    // The mul is applied as the last operation to prevent overflow
    let brr = clock / baud * mul;

    // The BRR calculation will be a bit off because of integer rounding.
    // Because we multiplied our inaccuracy with mul, our rounding now needs to be in proportion to mul.
    let rounding = ((clock % baud) * mul + (baud / 2)) / baud;

    brr + rounding
}

fn set_baudrate<T: Instance>(kernel_clock: Hertz, baudrate: u32) -> Result<(), ConfigError> {
    T::Interrupt::disable();

    set_usart_baudrate::<T>(kernel_clock, baudrate)?;

    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };

    Ok(())
}

fn find_and_set_brr(
    r: Regs,
    kind: Kind,
    kernel_clock: Hertz,
    baudrate: u32,
) -> Result<bool, ConfigError> {
    static DIVS: [(u16, ()); 1] = [(1, ())];

    let (mul, brr_min, brr_max) = match kind {
        // Kind::Lpuart => {
        //     trace!("USART: Kind::Lpuart");
        //     (256, 0x300, 0x10_0000)
        // }
        Kind::Uart => {
            trace!("USART: Kind::Uart");
            (1, 0x10, 0x1_0000)
        }
    };

    let mut found_brr = None;
    let mut over8 = false;

    for &(presc, _presc_val) in &DIVS {
        let brr = calculate_brr(baudrate, kernel_clock.0, presc as u32, mul);
        trace!(
            "USART: presc={}, div=0x{:08x} (mantissa = {}, fraction = {})",
            presc,
            brr,
            brr >> 4,
            brr & 0x0F
        );

        if brr < brr_min {
            if brr * 2 >= brr_min && kind == Kind::Uart {
                over8 = true;
                r.brr()
                    .write_value(regs::Brr(((brr << 1) & !0xF) | (brr & 0x07)));
                found_brr = Some(brr);
                break;
            }
            return Err(ConfigError::BaudrateTooHigh);
        }

        if brr < brr_max {
            r.brr().write_value(regs::Brr(brr));
            found_brr = Some(brr);
            break;
        }
    }

    match found_brr {
        Some(brr) => {
            let oversampling = if over8 { "8 bit" } else { "16 bit" };
            trace!(
                "Using {} oversampling, desired baudrate: {}, actual baudrate: {}",
                oversampling,
                baudrate,
                kernel_clock.0 / brr * mul
            );
            Ok(over8)
        }
        None => Err(ConfigError::BaudrateTooLow),
    }
}

fn set_usart_baudrate<T: Instance>(kernel_clock: Hertz, baudrate: u32) -> Result<(), ConfigError> {
    let r = T::regs();
    r.cr1().modify(|w| {
        // disable uart
        w.set_ue(false);
    });

    let over8 = find_and_set_brr(r, T::kind(), kernel_clock, baudrate)?;

    r.cr1().modify(|w| {
        // enable uart
        w.set_ue(true);

        w.set_over8(vals::OVER8::from_bits(over8 as _));
    });

    Ok(())
}

fn configure<T: Instance>(
    kernel_clock: Hertz,
    config: &Config,
    enable_rx: bool,
    enable_tx: bool,
) -> Result<(), ConfigError> {
    let r = T::regs();
    let kind = T::kind();

    if !enable_rx && !enable_tx {
        return Err(ConfigError::RxOrTxNotEnabled);
    }

    // UART must be disabled during configuration.
    r.cr1().modify(|w| {
        w.set_ue(false);
    });

    let over8 = find_and_set_brr(r, kind, kernel_clock, config.baudrate)?;

    r.cr2().write(|w| {
        w.set_stop(match config.stop_bits {
            // StopBits::STOP0P5 => vals::STOP::STOP0P5,
            StopBits::STOP1 => vals::STOP::Stop1,
            // StopBits::STOP1P5 => vals::STOP::STOP1P5,
            StopBits::STOP2 => vals::STOP::Stop2,
        });
    });

    r.cr3().modify(|w| {
        w.set_onebit(config.assume_noise_free);
        w.set_hdsel(config.duplex.is_half());
    });

    let (m_val, pce_val, ps_val) = match (config.parity, config.data_bits) {
        (Parity::ParityNone, DataBits::DataBits8) => {
            trace!("USART: m: 8 data bits, no parity");
            (vals::M::Bit8, false, None)
        }
        (Parity::ParityNone, DataBits::DataBits9) => {
            trace!("USART: m: 9 data bits, no parity");
            (vals::M::Bit9, false, None)
        }
        (Parity::ParityNone, DataBits::DataBits7) => {
            trace!("USART: m: 7 data bits, no parity");
            (vals::M::Bit8, false, None)
        }
        (Parity::ParityEven, DataBits::DataBits8) => {
            trace!("USART: m: 8 data bits, even parity");
            (vals::M::Bit9, true, Some(vals::PS::Even))
        }
        (Parity::ParityEven, DataBits::DataBits7) => {
            trace!("USART: m: 7 data bits, even parity");
            (vals::M::Bit8, true, Some(vals::PS::Even))
        }
        (Parity::ParityOdd, DataBits::DataBits8) => {
            trace!("USART: m: 8 data bits, odd parity");
            (vals::M::Bit9, true, Some(vals::PS::Odd))
        }
        (Parity::ParityOdd, DataBits::DataBits7) => {
            trace!("USART: m 7 data bits, odd parity");
            (vals::M::Bit8, true, Some(vals::PS::Odd))
        }
        (_, DataBits::DataBits6) => {
            todo!()
        }
        _ => {
            return Err(ConfigError::DataParityNotSupported);
        }
    };

    r.cr1().write(|w| {
        // enable uart
        w.set_ue(true);

        if config.duplex.is_half() {
            // The te and re bits will be set by write, read and flush methods.
            // Receiver should be enabled by default for Half-Duplex.
            w.set_te(false);
            w.set_re(true);
        } else {
            // enable transceiver
            w.set_te(enable_tx);
            // enable receiver
            w.set_re(enable_rx);
        }

        w.set_m(m_val);
        w.set_pce(pce_val);
        if let Some(ps) = ps_val {
            w.set_ps(ps);
        }

        w.set_over8(vals::OVER8::from_bits(over8 as _));
    });

    Ok(())
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::serial::Read<u8> for UartRx<'d, T, M> {
    type Error = Error;
    fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
        self.nb_read()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::serial::Write<u8> for UartTx<'d, T, M> {
    type Error = Error;
    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(buffer)
    }
    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::serial::Read<u8> for Uart<'d, T, M> {
    type Error = Error;
    fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
        self.nb_read()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::serial::Write<u8> for Uart<'d, T, M> {
    type Error = Error;
    fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(buffer)
    }
    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match *self {
            Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Self::Noise => embedded_hal_nb::serial::ErrorKind::Noise,
            Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Self::BufferTooLong => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::ErrorType for Uart<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::ErrorType for UartTx<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::ErrorType for UartRx<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Read for UartRx<'d, T, M> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.nb_read()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Write for UartTx<'d, T, M> {
    fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
        self.blocking_write(&[char]).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.blocking_flush().map_err(nb::Error::Other)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Read for Uart<'d, T, M> {
    fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
        self.nb_read()
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_nb::serial::Write for Uart<'d, T, M> {
    fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
        self.blocking_write(&[char]).map_err(nb::Error::Other)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.blocking_flush().map_err(nb::Error::Other)
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<T: Instance, M: Mode> embedded_io::ErrorType for Uart<'_, T, M> {
    type Error = Error;
}

impl<T: Instance, M: Mode> embedded_io::ErrorType for UartTx<'_, T, M> {
    type Error = Error;
}

impl<T: Instance, M: Mode> embedded_io::Write for Uart<'_, T, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<T: Instance, M: Mode> embedded_io::Write for UartTx<'_, T, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<T: Instance> embedded_io_async::Write for Uart<'_, T, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().await
    }
}

impl<T: Instance> embedded_io_async::Write for UartTx<'_, T, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf).await?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().await
    }
}

pub use buffered::*;

pub use crate::usart::buffered::InterruptHandler as BufferedInterruptHandler;
mod buffered;

mod ringbuffered;
pub use ringbuffered::RingBufferedUartRx;

fn clear_interrupt_flags(r: Regs, sr: regs::Isr) {
    r.icr().write(|w| *w = regs::Icr(sr.0));
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Uart,
    // #[allow(unused)]
    // Lpuart,
}

struct State {
    rx_waker: AtomicWaker,
    tx_rx_refcount: AtomicU8,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_rx_refcount: AtomicU8::new(0),
        }
    }
}

#[allow(private_interfaces)]
pub(crate) trait SealedInstance:
    crate::rcc::RccEnableReset + crate::rcc::RccGetFreq
{
    fn regs() -> Regs;
    fn interrupt() -> Interrupt;
    fn kind() -> Kind;
    fn state() -> &'static State;
    fn buffered_state() -> &'static buffered::State;
}

/// USART peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(RxdPin, Instance);
pin_trait!(TxdPin, Instance);
pin_trait!(CtsPin, Instance);
pin_trait!(RtsPin, Instance);
pin_trait!(CkPin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);

macro_rules! impl_usart {
    ($inst:ident, $irq:ident, $kind:expr) => {
        #[allow(private_interfaces)]
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> Regs {
                crate::pac::$inst
            }

            fn interrupt() -> Interrupt {
                crate::interrupt::$irq
            }

            fn kind() -> Kind {
                $kind
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }

            fn buffered_state() -> &'static buffered::State {
                static BUFFERED_STATE: buffered::State = buffered::State::new();
                &BUFFERED_STATE
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

// TODO: move to _generated.rs
impl_usart!(USART1, USART1, Kind::Uart);
impl_usart!(USART2, USART2, Kind::Uart);
impl_usart!(USART3, USART3, Kind::Uart);
