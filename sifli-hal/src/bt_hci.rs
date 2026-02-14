//! bt-hci Transport implementation, communicating with LCPU BT controller via IPC.
//!
//! This module provides a bt-hci compatible Transport wrapper for [`IpcQueue`],
//! enabling communication with the LCPU Bluetooth controller using libraries
//! like `bt-hci` and `trouble`.
//!
//! # Example
//!
//! ```no_run
//! use sifli_hal::{bind_interrupts, ipc, bt_hci::IpcHciTransport};
//! use bt_hci::controller::ExternalController;
//!
//! bind_interrupts!(struct Irqs {
//!     MAILBOX2_CH1 => ipc::InterruptHandler;
//! });
//!
//! async fn example() {
//!     let p = sifli_hal::init(Default::default());
//!     let mut ipc_driver = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
//!     let queue = ipc_driver.open_queue(ipc::QueueConfig::qid0_hci()).unwrap();
//!
//!     let transport = IpcHciTransport::new(queue);
//!     let controller: ExternalController<_, 4> = ExternalController::new(transport);
//!
//!     // Now you can use bt-hci's type-safe API
//!     // use bt_hci::cmd::controller_baseband::Reset;
//!     // controller.exec(Reset).await.unwrap();
//! }
//! ```

use bt_hci::transport::{Transport, WithIndicator};
use bt_hci::{ControllerToHostPacket, HostToControllerPacket, ReadHci, ReadHciError, WriteHci};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io::ReadExactError;

use crate::dma::Channel;
use crate::ipc::{self, Error as IpcError, IpcQueue, IpcQueueRx, IpcQueueTx};
use crate::lcpu::{Lcpu, LcpuConfig, LcpuError};
use crate::{interrupt, peripherals, Peripheral};

#[cfg(any(feature = "defmt", feature = "log"))]
/// Temporary buffer for logging HCI writes.
struct LogBuf {
    buf: [u8; 264],
    pos: usize,
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl LogBuf {
    fn new() -> Self {
        Self {
            buf: [0; 264],
            pos: 0,
        }
    }
    fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl embedded_io::ErrorType for LogBuf {
    type Error = embedded_io::ErrorKind;
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl embedded_io::Write for LogBuf {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let remaining = self.buf.len() - self.pos;
        let n = buf.len().min(remaining);
        self.buf[self.pos..self.pos + n].copy_from_slice(&buf[..n]);
        self.pos += n;
        Ok(n)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[cfg(any(feature = "defmt", feature = "log"))]
/// Wraps IPC RX and logs all bytes read through it.
struct LoggingReader<'a> {
    inner: &'a mut IpcQueueRx,
    log: [u8; 64],
    pos: usize,
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl<'a> LoggingReader<'a> {
    fn new(inner: &'a mut IpcQueueRx) -> Self {
        Self {
            inner,
            log: [0; 64],
            pos: 0,
        }
    }

    fn dump(&self, prefix: &str) {
        let end = self.pos.min(64);
        debug!("[hci] {} ({} bytes): {:02X}", prefix, self.pos, &self.log[..end]);
    }
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl embedded_io::ErrorType for LoggingReader<'_> {
    type Error = IpcError;
}

#[cfg(any(feature = "defmt", feature = "log"))]
impl embedded_io_async::Read for LoggingReader<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = embedded_io_async::Read::read(self.inner, buf).await?;
        // Record read bytes to the log buffer
        let copy_end = (self.pos + n).min(64);
        if self.pos < 64 {
            let copy_n = copy_end - self.pos;
            self.log[self.pos..copy_end].copy_from_slice(&buf[..copy_n]);
        }
        self.pos += n;
        Ok(n)
    }
}

/// Error returned by [`IpcHciTransport::ble_init`].
#[derive(Debug)]
pub enum BleInitError {
    /// IPC queue open error.
    Ipc(IpcError),
    /// LCPU power-on error.
    Lcpu(LcpuError),
}

#[cfg(feature = "defmt")]
impl defmt::Format for BleInitError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            BleInitError::Ipc(e) => defmt::write!(f, "IPC error: {}", e),
            BleInitError::Lcpu(e) => defmt::write!(f, "LCPU error: {:?}", defmt::Debug2Format(e)),
        }
    }
}

impl From<IpcError> for BleInitError {
    fn from(e: IpcError) -> Self {
        Self::Ipc(e)
    }
}

impl From<LcpuError> for BleInitError {
    fn from(e: LcpuError) -> Self {
        Self::Lcpu(e)
    }
}

/// bt-hci Transport error type.
#[derive(Debug)]
pub enum Error {
    /// IPC read error.
    Read(ReadHciError<IpcError>),
    /// IPC write error.
    Write(IpcError),
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Read(e) => defmt::write!(f, "HCI read error: {:?}", defmt::Debug2Format(e)),
            Error::Write(e) => defmt::write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Read(e) => write!(f, "HCI read error: {:?}", e),
            Error::Write(e) => write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Read(e) => e.kind(),
            Self::Write(e) => e.kind(),
        }
    }
}

impl From<IpcError> for Error {
    fn from(e: IpcError) -> Self {
        Self::Write(e)
    }
}

impl From<ReadHciError<IpcError>> for Error {
    fn from(e: ReadHciError<IpcError>) -> Self {
        Self::Read(e)
    }
}

impl From<ReadExactError<IpcError>> for Error {
    fn from(e: ReadExactError<IpcError>) -> Self {
        Self::Read(e.into())
    }
}

impl From<bt_hci::FromHciBytesError> for Error {
    fn from(e: bt_hci::FromHciBytesError) -> Self {
        Self::Read(e.into())
    }
}

/// IPC HCI Transport, wrapping [`IpcQueue`] as a bt-hci Transport.
///
/// Communicates using H4 format (with packet indicator byte).
/// Internally splits RX and TX with independent Mutex guards for concurrent read/write.
pub struct IpcHciTransport {
    rx: Mutex<CriticalSectionRawMutex, IpcQueueRx>,
    tx: Mutex<CriticalSectionRawMutex, IpcQueueTx>,
}

impl IpcHciTransport {
    /// Initialize BLE and create an HCI transport in one step.
    ///
    /// This combines IPC queue creation, LCPU BLE power-on, and transport
    /// construction into a single call.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use sifli_hal::{bind_interrupts, ipc, bt_hci::IpcHciTransport};
    /// use sifli_hal::lcpu::LcpuConfig;
    ///
    /// bind_interrupts!(struct Irqs {
    ///     MAILBOX2_CH1 => ipc::InterruptHandler;
    /// });
    ///
    /// async fn example() {
    ///     let p = sifli_hal::init(Default::default());
    ///     let transport = IpcHciTransport::ble_init(
    ///         p.MAILBOX1_CH1, Irqs, p.DMAC2_CH8, &LcpuConfig::default(),
    ///     ).await.unwrap();
    /// }
    /// ```
    pub async fn ble_init(
        mailbox: impl Peripheral<P = peripherals::MAILBOX1_CH1>,
        irq: impl interrupt::typelevel::Binding<
            interrupt::typelevel::MAILBOX2_CH1,
            ipc::InterruptHandler,
        >,
        dma_ch: impl Peripheral<P = impl Channel>,
        config: &LcpuConfig,
    ) -> Result<Self, BleInitError> {
        let mut ipc_driver = ipc::Ipc::new(mailbox, irq, ipc::Config::default());
        let queue = ipc_driver.open_queue(ipc::QueueConfig::qid0_hci())?;
        let (mut rx, tx) = queue.split();
        let lcpu = Lcpu::new();
        lcpu.ble_power_on(config, dma_ch, &mut rx).await?;
        Ok(Self::from_parts(rx, tx))
    }

    /// Create a new IPC HCI Transport.
    pub fn new(queue: IpcQueue) -> Self {
        let (rx, tx) = queue.split();
        Self::from_parts(rx, tx)
    }

    /// Create Transport from pre-split RX/TX halves.
    ///
    /// Use this when you need to access the RX half before creating the Transport
    /// (e.g., consuming the warmup event during `ble_power_on`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn example(queue: sifli_hal::ipc::IpcQueue) {
    /// use sifli_hal::bt_hci::IpcHciTransport;
    ///
    /// let (mut rx, tx) = queue.split();
    /// // Can use rx first (e.g., ble_power_on consumes warmup event)
    /// let transport = IpcHciTransport::from_parts(rx, tx);
    /// # }
    /// ```
    pub fn from_parts(rx: IpcQueueRx, tx: IpcQueueTx) -> Self {
        Self {
            rx: Mutex::new(rx),
            tx: Mutex::new(tx),
        }
    }
}

impl embedded_io::ErrorType for IpcHciTransport {
    type Error = Error;
}

impl Transport for IpcHciTransport {
    async fn read<'a>(&self, rx: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        let mut q = self.rx.lock().await;

        #[cfg(any(feature = "defmt", feature = "log"))]
        {
            let mut logging_rx = LoggingReader::new(&mut *q);
            let pkt = ControllerToHostPacket::read_hci_async(&mut logging_rx, rx)
                .await
                .map_err(Error::Read)?;
            logging_rx.dump("rx");
            Ok(pkt)
        }

        #[cfg(not(any(feature = "defmt", feature = "log")))]
        {
            ControllerToHostPacket::read_hci_async(&mut *q, rx)
                .await
                .map_err(Error::Read)
        }
    }

    async fn write<T: HostToControllerPacket>(&self, val: &T) -> Result<(), Self::Error> {
        #[cfg(any(feature = "defmt", feature = "log"))]
        {
            let mut log_buf = LogBuf::new();
            let _ = WithIndicator::new(val).write_hci(&mut log_buf);
            let s = log_buf.as_slice();
            let end = s.len().min(64);
            if s.len() >= 4 && s[0] == 0x01 {
                let opcode = (s[2] as u16) << 8 | s[1] as u16;
                debug!(
                    "[hci] tx cmd(0x{:04X}) {} bytes: {:02X}",
                    opcode,
                    s.len(),
                    &s[..end]
                );
            } else {
                debug!("[hci] tx {} bytes: {:02X}", s.len(), &s[..end]);
            }
        }

        let mut q = self.tx.lock().await;
        WithIndicator::new(val)
            .write_hci_async(&mut *q)
            .await
            .map_err(Error::Write)?;
        q.flush()?;
        Ok(())
    }
}
