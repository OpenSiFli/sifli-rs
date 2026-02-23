//! IPC HCI Transport and high-level BLE Controller.

use bt_hci::transport::{Transport, WithIndicator};
use bt_hci::{ControllerToHostPacket, HostToControllerPacket, ReadHci, ReadHciError, WriteHci};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io::ReadExactError;

use sifli_hal::ipc::{Error as IpcError, IpcQueue, IpcQueueRx, IpcQueueTx};

#[cfg(any(feature = "defmt", feature = "log"))]
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
        debug!(
            "[hci] {} ({} bytes): {:02X}",
            prefix,
            self.pos,
            &self.log[..end]
        );
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
        let copy_end = (self.pos + n).min(64);
        if self.pos < 64 {
            let copy_n = copy_end - self.pos;
            self.log[self.pos..copy_end].copy_from_slice(&buf[..copy_n]);
        }
        self.pos += n;
        Ok(n)
    }
}

/// bt-hci Transport error type.
#[derive(Debug)]
pub enum HciError {
    /// IPC read error.
    Read(ReadHciError<IpcError>),
    /// IPC write error.
    Write(IpcError),
}

#[cfg(feature = "defmt")]
impl defmt::Format for HciError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            HciError::Read(e) => defmt::write!(f, "HCI read error: {:?}", defmt::Debug2Format(e)),
            HciError::Write(e) => defmt::write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::fmt::Display for HciError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HciError::Read(e) => write!(f, "HCI read error: {:?}", e),
            HciError::Write(e) => write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::error::Error for HciError {}

impl embedded_io::Error for HciError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Read(e) => e.kind(),
            Self::Write(e) => e.kind(),
        }
    }
}

impl From<IpcError> for HciError {
    fn from(e: IpcError) -> Self {
        Self::Write(e)
    }
}

impl From<ReadHciError<IpcError>> for HciError {
    fn from(e: ReadHciError<IpcError>) -> Self {
        Self::Read(e)
    }
}

impl From<ReadExactError<IpcError>> for HciError {
    fn from(e: ReadExactError<IpcError>) -> Self {
        Self::Read(e.into())
    }
}

impl From<bt_hci::FromHciBytesError> for HciError {
    fn from(e: bt_hci::FromHciBytesError) -> Self {
        Self::Read(e.into())
    }
}

/// IPC HCI Transport, wrapping [`IpcQueue`] as a bt-hci Transport.
pub struct IpcHciTransport {
    rx: Mutex<CriticalSectionRawMutex, IpcQueueRx>,
    tx: Mutex<CriticalSectionRawMutex, IpcQueueTx>,
}

impl IpcHciTransport {
    /// Create a new IPC HCI Transport.
    pub fn new(queue: IpcQueue) -> Self {
        let (rx, tx) = queue.split();
        Self::from_parts(rx, tx)
    }

    /// Create Transport from pre-split RX/TX halves.
    pub fn from_parts(rx: IpcQueueRx, tx: IpcQueueTx) -> Self {
        Self {
            rx: Mutex::new(rx),
            tx: Mutex::new(tx),
        }
    }
}

impl embedded_io::ErrorType for IpcHciTransport {
    type Error = HciError;
}

impl Transport for IpcHciTransport {
    async fn read<'a>(&self, rx: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        let mut q = self.rx.lock().await;

        #[cfg(any(feature = "defmt", feature = "log"))]
        {
            let mut logging_rx = LoggingReader::new(&mut *q);
            let pkt = ControllerToHostPacket::read_hci_async(&mut logging_rx, rx)
                .await
                .map_err(HciError::Read)?;
            logging_rx.dump("rx");
            Ok(pkt)
        }

        #[cfg(not(any(feature = "defmt", feature = "log")))]
        {
            ControllerToHostPacket::read_hci_async(&mut *q, rx)
                .await
                .map_err(HciError::Read)
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
            .map_err(HciError::Write)?;
        q.flush()?;
        Ok(())
    }
}

/// Consume the BT warmup event from IPC HCI RX.
pub(crate) async fn consume_warmup_event<R>(rx: &mut R) -> Result<(), sifli_hal::lcpu::LcpuError>
where
    R: embedded_io_async::Read,
{
    let mut header = [0u8; 3];
    rx.read_exact(&mut header)
        .await
        .map_err(|_| sifli_hal::lcpu::LcpuError::WarmupReadError)?;

    debug!(
        "[hci] warmup header: {:02X} {:02X} {:02X}",
        header[0], header[1], header[2]
    );

    if header[0] != 0x04 {
        warn!(
            "[hci] unexpected H4 indicator in warmup: 0x{:02X}, expected 0x04",
            header[0]
        );
    }

    let param_len = header[2] as usize;
    if param_len > 0 {
        let mut params = [0u8; 255];
        rx.read_exact(&mut params[..param_len])
            .await
            .map_err(|_| sifli_hal::lcpu::LcpuError::WarmupReadError)?;
        debug!(
            "[hci] warmup params ({} bytes): {:02X}",
            param_len,
            &params[..param_len]
        );
    }

    debug!("[hci] warmup event consumed");
    Ok(())
}

