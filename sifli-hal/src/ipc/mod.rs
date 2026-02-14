//! Inter-core IPC Queue (MAILBOX doorbell + shared memory ring buffer).
//!
//! Replicates the SiFli SDK `ipc_queue` wire protocol on the HCPU Rust side,
//! maintaining compatibility with LCPU ROM/firmware.
//!
//! # Architecture
//!
//! The IPC mechanism consists of two parts: **interrupt notification** (doorbell) and
//! **shared memory** (ring buffer).
//!
//! ## 1. Interrupt Notification (MAILBOX hardware)
//!
//! SF32LB52x has two MAILBOX peripherals:
//! - **MAILBOX1** (4 channels C1-C4): HCPU writes -> triggers LCPU interrupt
//! - **MAILBOX2** (2 channels C1-C2): LCPU writes -> triggers HCPU interrupt
//!
//! **IPC queue only uses MAILBOX1_C1 and MAILBOX2_C1**. Each channel's interrupt register
//! has 16 independent bits; IPC uses bits 0-7 corresponding to qid 0-7. To notify the
//! peer, write the corresponding bit in the ITR register to trigger an interrupt.
//!
//! ```text
//! MAILBOX1_C1.IER/ITR/ISR:  [bit15 ... bit8 | bit7(qid7) ... bit1(qid1) bit0(qid0)]
//! MAILBOX2_C1.IER/ITR/ISR:  [bit15 ... bit8 | bit7(qid7) ... bit1(qid1) bit0(qid0)]
//! ```
//!
//! **MAILBOX1_C2/C3/C4 and MAILBOX2_C2 are not used by the IPC queue mechanism.**
//!
//! ## 2. Shared Memory (SRAM buffers)
//!
//! Data is transferred through ring buffers in SRAM. The SDK pre-allocates two buffer
//! pairs (512 bytes each):
//!
//! | Buffer Name | HCPU Address | Purpose |
//! |-------------|-------------|---------|
//! | HCPU2LCPU_BUF1 | 0x2007FE00 | TX for qid0 (HCI) |
//! | HCPU2LCPU_BUF2 | 0x2007FC00 | TX for qid1-7 |
//! | LCPU2HCPU_BUF1 | 0x20405C00 | RX for qid0 (HCI) |
//! | LCPU2HCPU_BUF2 | 0x20405E00 | RX for qid1-7 |
//!
//! Note: "BUF1/BUF2" in buffer names have **no correspondence** to physical MAILBOX
//! channels C1/C2.
//!
//! ## 3. SDK qid allocation (sf32lb52x)
//!
//! | qid | Buffer | Purpose |
//! |-----|--------|---------|
//! | 0 | BUF1 | HCI (Bluetooth controller communication) |
//! | 1 | BUF2 | System IPC (data service, etc.) |
//! | 6 | BUF2 | Bluetooth audio |
//! | 7 | BUF2 | Debug/logging |
//!
//! # Usage Example
//!
//! ```ignore
//! use sifli_hal::{bind_interrupts, ipc};
//!
//! bind_interrupts!(struct Irqs {
//!     MAILBOX2_CH1 => ipc::InterruptHandler;
//! });
//!
//! let p = sifli_hal::init(Default::default());
//! let rev = sifli_hal::syscfg::read_idr().revision();
//! let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
//! let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci(rev)).unwrap();
//! ```

mod circular_buf;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem;
use core::sync::atomic::{fence, AtomicBool, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{self, InterruptExt};
use crate::lcpu::ram::IpcRegion;
use crate::{peripherals, rcc};

use circular_buf::{CircularBuf, CircularBufMutPtrExt, CircularBufPtrExt};

/// SF32LB52x `ipc_hw_port.h`: `IPC_HW_QUEUE_NUM=8`
pub const HW_QUEUE_NUM: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    InvalidQid,
    AlreadyOpen,
    BufferTooSmall,
    NotOpen,
    TxUnavailable,
    RxUnavailable,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::InvalidQid => write!(f, "invalid queue ID"),
            Error::AlreadyOpen => write!(f, "queue already open"),
            Error::BufferTooSmall => write!(f, "buffer too small"),
            Error::NotOpen => write!(f, "queue not open"),
            Error::TxUnavailable => write!(f, "TX unavailable"),
            Error::RxUnavailable => write!(f, "RX unavailable"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::NotOpen | Error::AlreadyOpen => embedded_io::ErrorKind::NotConnected,
            Error::InvalidQid | Error::BufferTooSmall => embedded_io::ErrorKind::InvalidInput,
            Error::TxUnavailable | Error::RxUnavailable => embedded_io::ErrorKind::Other,
        }
    }
}

/// IPC initialization configuration.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub irq_priority: interrupt::Priority,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // SDK `ipc_hw_enable_interrupt`: HAL_NVIC_SetPriority(..., 3, 0)
            irq_priority: interrupt::Priority::P3,
        }
    }
}

/// Single queue configuration (essential fields from SDK `ipc_queue_cfg_t`).
#[derive(Debug, Clone, Copy)]
pub struct QueueConfig {
    pub qid: u8,
    pub rx_buf_addr: usize,
    pub tx_buf_addr: usize,
    pub tx_buf_addr_alias: usize,
    pub tx_buf_size: usize,
}

impl QueueConfig {
    /// qid0: SDK Bluetooth HCI channel, using BUF1 buffer.
    pub fn qid0_hci() -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH1;
        Self {
            qid: 0,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_start(),
            tx_buf_addr: tx,
            tx_buf_addr_alias: IpcRegion::hcpu_to_lcpu_addr(tx),
            tx_buf_size: IpcRegion::BUF_SIZE,
        }
    }

    /// qid1: SDK system IPC channel (data-service, etc.), using BUF2 buffer.
    pub fn qid1_system() -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH2;
        Self {
            qid: 1,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_ch2_start(),
            tx_buf_addr: tx,
            tx_buf_addr_alias: IpcRegion::hcpu_to_lcpu_addr(tx),
            tx_buf_size: IpcRegion::BUF_SIZE,
        }
    }
}

struct QueueState {
    active: AtomicBool,
    rx_len: AtomicUsize,
    rx_waker: AtomicWaker,
    rx_buf: AtomicUsize,
    tx_buf: AtomicUsize,
}

impl QueueState {
    const fn new() -> Self {
        Self {
            active: AtomicBool::new(false),
            rx_len: AtomicUsize::new(0),
            rx_waker: AtomicWaker::new(),
            rx_buf: AtomicUsize::new(0),
            tx_buf: AtomicUsize::new(0),
        }
    }
}

static QUEUES: [QueueState; HW_QUEUE_NUM] = [const { QueueState::new() }; HW_QUEUE_NUM];

// ============================================================================
// Interrupt handler
// ============================================================================

/// IPC interrupt handler.
///
/// Handles MAILBOX2_CH1 interrupt directly and wakes the corresponding IPC queue,
/// without spawning a separate task.
///
/// # Example
///
/// ```no_run
/// use sifli_hal::{bind_interrupts, ipc};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => ipc::InterruptHandler;
/// });
/// ```
pub struct InterruptHandler {
    _phantom: PhantomData<()>,
}

impl interrupt::typelevel::Handler<interrupt::typelevel::MAILBOX2_CH1> for InterruptHandler {
    unsafe fn on_interrupt() {
        let regs = crate::pac::MAILBOX2;

        // Read masked interrupt status
        let misr = regs.misr(0).read().0 as u16;
        if misr == 0 {
            return;
        }

        // Clear interrupt
        regs.icr(0).write_value(crate::pac::mailbox::regs::Ixr(misr as u32));

        fence(Ordering::SeqCst);

        // Iterate triggered bits and wake corresponding queues
        for qid in 0..HW_QUEUE_NUM {
            if misr & (1 << qid) != 0 {
                handle_rx_irq(qid as u8);
            }
        }
    }
}

#[inline]
fn handle_rx_irq(qid: u8) {
    let st = &QUEUES[qid as usize];
    if !st.active.load(Ordering::Acquire) {
        return;
    }

    let rx_ptr = st.rx_buf.load(Ordering::Acquire);
    let len = if rx_ptr == 0 {
        0
    } else {
        // LCPU triggers mailbox interrupt after writing to the circular buffer.
        // Acquire load only guarantees rx_ptr pointer visibility, not that LCPU's
        // writes to buffer contents (write_idx_mirror, etc.) are flushed to shared SRAM.
        // Explicit SeqCst fence ensures subsequent volatile reads see the latest data.
        fence(Ordering::SeqCst);
        unsafe { (rx_ptr as *const CircularBuf).data_len() }
    };

    st.rx_len.store(len, Ordering::Release);
    st.rx_waker.wake();
}

// ============================================================================
// IPC driver
// ============================================================================

/// IPC (HCPU side) initialization and queue management.
///
/// # Example
///
/// ```ignore
/// use sifli_hal::{bind_interrupts, ipc};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => ipc::InterruptHandler;
/// });
///
/// let p = sifli_hal::init(Default::default());
/// let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
/// let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci()).unwrap();
/// ```
pub struct Ipc<'d> {
    _tx_ch: PeripheralRef<'d, peripherals::MAILBOX1_CH1>,
}

impl<'d> Ipc<'d> {
    /// Create a new IPC instance
    pub fn new(
        tx_ch: impl Peripheral<P = peripherals::MAILBOX1_CH1> + 'd,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::MAILBOX2_CH1, InterruptHandler>,
        config: Config,
    ) -> Self {
        into_ref!(tx_ch);

        // MAILBOX1 is in HPSYS; enable RCC (no reset, to avoid disturbing other channels).
        rcc::enable::<peripherals::MAILBOX1>();

        // Configure MAILBOX2_CH1 interrupt priority
        let irq = crate::interrupt::MAILBOX2_CH1;
        unsafe {
            irq.set_priority(config.irq_priority);
            irq.unpend();
            irq.enable();
        }

        Self { _tx_ch: tx_ch }
    }

    /// Open an IPC Queue (minimal subset of SDK init+open).
    pub fn open_queue(&mut self, cfg: QueueConfig) -> Result<IpcQueue, Error> {
        if cfg.qid as usize >= HW_QUEUE_NUM {
            return Err(Error::InvalidQid);
        }
        if cfg.tx_buf_size <= mem::size_of::<CircularBuf>() {
            return Err(Error::BufferTooSmall);
        }

        let st = &QUEUES[cfg.qid as usize];

        critical_section::with(|_| {
            if st.active.swap(true, Ordering::AcqRel) {
                return Err(Error::AlreadyOpen);
            }

            st.rx_buf.store(cfg.rx_buf_addr, Ordering::Release);
            st.tx_buf.store(cfg.tx_buf_addr, Ordering::Release);

            // Sender initializes TX ring buffer and remaps rd_buffer_ptr to peer-visible alias.
            if cfg.tx_buf_addr != 0 {
                unsafe {
                    let cb = cfg.tx_buf_addr as *mut CircularBuf;
                    let pool_wr = (cfg.tx_buf_addr as *mut u8).add(mem::size_of::<CircularBuf>());
                    cb.wr_init(
                        pool_wr,
                        (cfg.tx_buf_size - mem::size_of::<CircularBuf>()) as i16,
                    );

                    let pool_rd =
                        (cfg.tx_buf_addr_alias as *mut u8).add(mem::size_of::<CircularBuf>());
                    cb.rd_init(pool_rd);
                }
            }

            // SDK behavior: don't read RX ring buffer on open (peer may not be initialized yet).
            st.rx_len.store(0, Ordering::Release);

            // Unmask: per SDK semantics, unmask qid on TX mailbox; also unmask RX side to reduce missed interrupts.
            let qid_mask = 1u16 << cfg.qid;
            // Directly operate MAILBOX1 registers
            crate::pac::MAILBOX1
                .ier(0)
                .modify(|w| w.0 |= qid_mask as u32);
            {
                let mb2 = crate::pac::MAILBOX2;
                mb2.ier(0).modify(|w| w.0 |= qid_mask as u32);
                mb2.icr(0).write_value(crate::pac::mailbox::regs::Ixr(qid_mask as u32));
            }

            Ok(IpcQueue {
                rx: IpcQueueRx { qid: cfg.qid },
                tx: IpcQueueTx { qid: cfg.qid },
            })
        })
    }
}

/// Opened IPC Queue handle.
pub struct IpcQueue {
    rx: IpcQueueRx,
    tx: IpcQueueTx,
}

/// IPC Queue receive half (RX).
///
/// Obtained via [`IpcQueue::split()`], can perform read operations independently of the TX half.
pub struct IpcQueueRx {
    qid: u8,
}

/// IPC Queue transmit half (TX).
///
/// Obtained via [`IpcQueue::split()`], can perform write operations independently of the RX half.
pub struct IpcQueueTx {
    qid: u8,
}

impl IpcQueue {
    /// Returns the queue ID.
    #[inline]
    pub fn qid(&self) -> u8 {
        self.rx.qid
    }

    /// Split the IpcQueue into independent RX and TX halves.
    ///
    /// After splitting, RX and TX can be used concurrently in different tasks without
    /// blocking each other. Useful for scenarios requiring simultaneous read/write (e.g., HCI).
    ///
    /// # Example
    ///
    /// ```ignore
    /// let queue = ipc.open_queue(cfg)?;
    /// let (rx, tx) = queue.split();
    /// // rx and tx can be used in different tasks
    /// ```
    pub fn split(self) -> (IpcQueueRx, IpcQueueTx) {
        (self.rx, self.tx)
    }

    /// Readable bytes in RX buffer (updated by interrupt; can re-read shared ring buffer for self-calibration).
    #[inline]
    pub fn rx_available(&self) -> Result<usize, Error> {
        self.rx.rx_available()
    }

    /// Read data (non-blocking). Returns `Ok(0)` when no data available.
    #[inline]
    pub fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(out)
    }

    /// Wait until RX has data available.
    #[inline]
    pub async fn wait_readable(&mut self) -> Result<(), Error> {
        self.rx.wait_readable().await
    }

    /// Async read (reads at least 1 byte unless an error occurs).
    #[inline]
    pub async fn read_async(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_async(out).await
    }

    /// Write data to TX ring buffer.
    ///
    /// Consistent with SDK: this interface is not thread-safe (avoid concurrent writes to the same queue).
    #[inline]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write(data)
    }

    /// Trigger doorbell to notify the peer that data is ready.
    ///
    /// Must call `flush` after `write` to notify LCPU.
    /// This design supports batching multiple writes with a single doorbell, preventing LCPU from receiving incomplete packets.
    #[inline]
    pub fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush()
    }
}

// ============================================================================
// IpcQueueRx implementation
// ============================================================================

impl IpcQueueRx {
    #[inline]
    pub fn qid(&self) -> u8 {
        self.qid
    }

    /// Readable bytes in RX buffer.
    pub fn rx_available(&self) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }
        Ok(st.rx_len.load(Ordering::Acquire))
    }

    /// Read data (non-blocking). Returns `Ok(0)` when no data available.
    pub fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        let rx_ptr = st.rx_buf.load(Ordering::Acquire);
        if rx_ptr == 0 {
            return Err(Error::RxUnavailable);
        }

        if st.rx_len.load(Ordering::Acquire) == 0 {
            return Ok(0);
        }

        let cb = rx_ptr as *mut CircularBuf;
        let n = critical_section::with(|_| {
            fence(Ordering::SeqCst);
            let n = unsafe { cb.get(out) };
            st.rx_len.store(
                unsafe { (cb as *const CircularBuf).data_len() },
                Ordering::Release,
            );
            n
        });
        Ok(n)
    }

    /// Wait until RX has data available.
    pub async fn wait_readable(&mut self) -> Result<(), Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        poll_fn(|cx| {
            if st.rx_len.load(Ordering::Acquire) > 0 {
                return Poll::Ready(());
            }
            st.rx_waker.register(cx.waker());
            fence(Ordering::SeqCst);
            if st.rx_len.load(Ordering::Acquire) > 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    /// Async read (reads at least 1 byte unless an error occurs).
    pub async fn read_async(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.wait_readable().await?;
        self.read(out)
    }
}

impl embedded_io_async::ErrorType for IpcQueueRx {
    type Error = Error;
}

impl embedded_io_async::Read for IpcQueueRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.wait_readable().await?;
        IpcQueueRx::read(self, buf)
    }
}

// ============================================================================
// IpcQueueTx implementation
// ============================================================================

impl IpcQueueTx {
    #[inline]
    pub fn qid(&self) -> u8 {
        self.qid
    }

    /// Write data to TX ring buffer.
    ///
    /// Consistent with SDK: this interface is not thread-safe (avoid concurrent writes to the same queue).
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        let tx_ptr = st.tx_buf.load(Ordering::Acquire);
        if tx_ptr == 0 {
            return Err(Error::TxUnavailable);
        }

        let cb = tx_ptr as *mut CircularBuf;
        let n = unsafe { cb.put(data) };
        Ok(n)
    }

    /// Trigger doorbell to notify the peer that data is ready.
    ///
    /// Must call `flush` after `write` to notify LCPU.
    pub fn flush(&mut self) -> Result<(), Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        fence(Ordering::SeqCst);
        crate::pac::MAILBOX1
            .itr(0)
            .write_value(crate::pac::mailbox::regs::Ixr(1u32 << self.qid));
        Ok(())
    }
}

impl embedded_io_async::ErrorType for IpcQueueTx {
    type Error = Error;
}

impl embedded_io_async::Write for IpcQueueTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        IpcQueueTx::write(self, buf)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        IpcQueueTx::flush(self)
    }
}

// ============================================================================
// embedded_io_async trait implementations for IpcQueue
// ============================================================================

impl embedded_io_async::ErrorType for IpcQueue {
    type Error = Error;
}

impl embedded_io_async::Read for IpcQueue {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // embedded_io::Read semantics: must read at least 1 byte (unless EOF or error)
        // Use wait_readable to wait for data, then read
        self.wait_readable().await?;
        IpcQueue::read(self, buf)
    }
}

impl embedded_io_async::Write for IpcQueue {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // IpcQueue::write is non-blocking, call directly
        IpcQueue::write(self, buf)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // Trigger doorbell to notify LCPU
        IpcQueue::flush(self)
    }
}
