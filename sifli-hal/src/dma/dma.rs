// The following code is modified from embassy-stm32 under MIT license
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use core::future::{poll_fn, Future};
use core::pin::Pin;
use core::sync::atomic::{fence, AtomicUsize, Ordering};
use core::task::{Context, Poll, Waker};

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use super::ringbuffer::{DmaCtrl, Error, ReadableDmaRingBuffer, WritableDmaRingBuffer};
use super::word::{Word, WordSize};
use super::{AnyChannel, Channel, Request, STATE};
use crate::{interrupt, pac, peripherals};

pub use pac::dmac::vals::Dir;
pub use pac::dmac::vals::Pl as Priority;

/// DMA address increment mode.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Increment {
    /// DMA will not increment either of the addresses.
    None,
    /// DMA will increment the peripheral address.
    Peripheral,
    /// DMA will increment the memory address.
    Memory,
    /// DMA will increment both peripheral and memory addresses simultaneously.
    Both,
}

pub(crate) struct ChannelInfo {
    pub(crate) dma: pac::dmac::Dmac,
    pub(crate) num: usize,
}

/// DMA transfer options.
///
/// # Example
///
/// ```rust,ignore
/// // Use defaults (VeryHigh priority, transfer-complete interrupt enabled)
/// let opts = TransferOptions::default();
///
/// // Custom options
/// let opts = TransferOptions {
///     priority: Priority::Medium,
///     circular: true,
///     half_transfer_ir: true,
///     ..Default::default()
/// };
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// Request priority level
    pub priority: Priority,
    /// Channel Interrupt priority level
    pub interrupt_priority: interrupt::Priority,
    /// Enable circular DMA
    ///
    /// Note:
    /// If you enable circular mode manually, you may want to build and `.await` the `Transfer` in a separate task.
    /// Since DMA in circular mode need manually stop, `.await` in current task would block the task forever.
    pub circular: bool,
    /// Enable half transfer interrupt
    pub half_transfer_ir: bool,
    /// Enable transfer complete interrupt
    pub complete_transfer_ir: bool,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            priority: Priority::VeryHigh,
            interrupt_priority: interrupt::Priority::P1,
            circular: false,
            half_transfer_ir: false,
            complete_transfer_ir: true,
        }
    }
}

impl From<WordSize> for pac::dmac::vals::Size {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::Bits8,
            WordSize::TwoBytes => Self::Bits16,
            WordSize::FourBytes => Self::Bits32,
        }
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: AtomicUsize,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: AtomicUsize::new(0),
    };
}

/// safety: must be called only once
pub(crate) unsafe fn init(cs: critical_section::CriticalSection) {
    crate::rcc::enable_and_reset_with_cs::<peripherals::DMAC1>(cs);

    // Initialize DMAC2 if LCPU feature is enabled
    #[cfg(feature = "sf32lb52x-lcpu")]
    {
        crate::rcc::enable_and_reset_with_cs::<peripherals::DMAC2>(cs);
    }
}

impl AnyChannel {
    /// Safety: Must be called with a matching set of parameters for a valid dma channel
    pub(crate) unsafe fn on_irq(&self) {
        let info = self.info();
        let state = &STATE[self.state_index()];
        let r = info.dma;
        let cr = r.ccr(info.num);
        let isr = r.isr().read();

        if isr.teif(info.num) {
            panic!(
                "DMA: error on DMA@{:08x} channel {}",
                r.as_ptr() as u32,
                info.num
            );
        }

        if isr.htif(info.num) && cr.read().htie() {
            // Acknowledge half transfer complete interrupt
            r.ifcr().write(|w| w.set_chtif(info.num, true));
        } else if isr.tcif(info.num) && cr.read().tcie() {
            // Acknowledge transfer complete interrupt
            r.ifcr().write(|w| w.set_ctcif(info.num, true));

            // stop the channel.
            // we should set EN manually on sf32. (?)
            if !r.ccr(info.num).read().circ() {
                r.ccr(info.num).modify(|w| {
                    w.set_en(false);
                });
            }
            state.complete_count.fetch_add(1, Ordering::Release);
        } else {
            return;
        }
        state.waker.wake();
    }

    unsafe fn configure(
        &self,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr: Increment,
        mem2mem: bool,
        mem_size: WordSize,
        peri_size: WordSize,
        options: TransferOptions,
    ) {
        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        let info = self.info();
        let r = info.dma;
        let state: &ChannelState = &STATE[self.state_index()];
        let channel_num = info.num;

        state.complete_count.store(0, Ordering::Release);
        self.clear_irqs();

        // NDTR is the number of transfers in the *peripheral* word size.
        // ex: if mem_size=1, peri_size=4 and ndtr=3 it'll do 12 mem transfers, 3 peri transfers.
        let ndtr = match (mem_size, peri_size) {
            (WordSize::FourBytes, WordSize::OneByte) => mem_len * 4,
            (WordSize::FourBytes, WordSize::TwoBytes) | (WordSize::TwoBytes, WordSize::OneByte) => {
                mem_len * 2
            }
            (WordSize::FourBytes, WordSize::FourBytes)
            | (WordSize::TwoBytes, WordSize::TwoBytes)
            | (WordSize::OneByte, WordSize::OneByte) => mem_len,
            (WordSize::TwoBytes, WordSize::FourBytes) | (WordSize::OneByte, WordSize::TwoBytes) => {
                assert!(mem_len % 2 == 0);
                mem_len / 2
            }
            (WordSize::OneByte, WordSize::FourBytes) => {
                assert!(mem_len % 4 == 0);
                mem_len / 4
            }
        };

        assert!(ndtr > 0 && ndtr <= 0xFFFF);

        // In M2M mode CPAR is also a memory address, apply remap for flash addresses
        let peri_addr = if mem2mem {
            crate::to_system_bus_addr(peri_addr as _) as _
        } else {
            peri_addr as u32
        };
        r.cpar(channel_num)
            .write_value(pac::dmac::regs::Cpar(peri_addr));

        r.cm0ar(channel_num)
            .write_value(pac::dmac::regs::Cm0ar(crate::to_system_bus_addr(mem_addr as _) as _));
        r.cndtr(channel_num)
            .write_value(pac::dmac::regs::Cndtr(ndtr as _));
        r.cselr(channel_num / 4)
            .modify(|w| w.set_cs(channel_num % 4, request as u8));
        r.ccr(channel_num).write(|w| {
            w.set_dir(dir.into());
            w.set_msize(mem_size.into());
            w.set_psize(peri_size.into());
            w.set_pl(options.priority.into());
            match incr {
                Increment::None => {
                    w.set_minc(false);
                    w.set_pinc(false);
                }
                Increment::Peripheral => {
                    w.set_minc(false);
                    w.set_pinc(true);
                }
                Increment::Memory => {
                    w.set_minc(true);
                    w.set_pinc(false);
                }
                Increment::Both => {
                    w.set_minc(true);
                    w.set_pinc(true);
                }
            }
            w.set_teie(true);
            w.set_htie(options.half_transfer_ir);
            w.set_tcie(options.complete_transfer_ir);
            w.set_circ(options.circular);
            w.set_mem2mem(mem2mem);
            w.set_en(false);
        });

        crate::_generated::enable_dma_channel_interrupt_priority(
            self.id,
            options.interrupt_priority,
        );
    }

    fn start(&self) {
        let info = self.info();
        let r = info.dma;
        r.ccr(info.num).modify(|w| w.set_en(true))
    }

    fn clear_irqs(&self) {
        let info = self.info();
        let r = info.dma;

        r.ifcr().write(|w| {
            w.set_chtif(info.num, true);
            w.set_ctcif(info.num, true);
            w.set_cteif(info.num, true);
        });
    }

    fn request_stop(&self) {
        let info = self.info();
        let r = info.dma;
        // Disable the channel. Keep the IEs enabled so the irqs still fire.
        r.ccr(info.num).write(|w| {
            w.set_teie(true);
            w.set_tcie(true);
        });
    }

    fn request_pause(&self) {
        let info = self.info();
        let r = info.dma;
        // Disable the channel without overwriting the existing configuration
        r.ccr(info.num).modify(|w| {
            w.set_en(false);
        });
    }

    fn is_running(&self) -> bool {
        let info = self.info();
        let r = info.dma;
        r.ccr(info.num).read().en()
    }

    fn get_remaining_transfers(&self) -> u16 {
        let info = self.info();
        let r = info.dma;
        r.cndtr(info.num).read().ndt()
    }

    fn disable_circular_mode(&self) {
        let info = self.info();
        let r = info.dma;
        r.ccr(info.num).modify(|w| {
            w.set_circ(false);
        })
    }

    fn poll_stop(&self) -> Poll<()> {
        use core::sync::atomic::compiler_fence;
        compiler_fence(Ordering::SeqCst);

        if !self.is_running() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: PeripheralRef<'a, AnyChannel>,
}

impl<'a> Transfer<'a> {
    /// Create a new read DMA transfer (peripheral to memory).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let mut buf = [0u32; 64];
    /// let transfer = unsafe {
    ///     Transfer::new_read(
    ///         p.DMAC1_CH1,
    ///         Request::USART1_RX,
    ///         pac::USART1.dr().as_ptr() as *mut u32,
    ///         &mut buf,
    ///         TransferOptions::default(),
    ///     )
    /// };
    /// transfer.await;
    /// // buf now contains data read from USART1
    /// ```
    pub unsafe fn new_read<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    /// Create a new read DMA transfer (peripheral to memory), using raw pointers.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let mut buf = [0u8; 256];
    /// let transfer = unsafe {
    ///     Transfer::new_read_raw::<u8, u32>(
    ///         p.DMAC1_CH1,
    ///         Request::USART1_RX,
    ///         pac::USART1.dr().as_ptr() as *mut u32,
    ///         &mut buf as *mut [u8],
    ///         TransferOptions::default(),
    ///     )
    /// };
    /// transfer.await;
    /// ```
    pub unsafe fn new_read_raw<MW: Word, PW: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut PW,
        buf: *mut [MW],
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut MW as *mut u32,
            buf.len(),
            Increment::Memory,
            MW::size(),
            PW::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let data = [0x48u8, 0x65, 0x6C, 0x6C, 0x6F]; // "Hello"
    /// let transfer = unsafe {
    ///     Transfer::new_write(
    ///         p.DMAC1_CH2,
    ///         Request::USART1_TX,
    ///         &data,
    ///         pac::USART1.dr().as_ptr() as *mut u8,
    ///         TransferOptions::default(),
    ///     )
    /// };
    /// transfer.await;
    /// ```
    pub unsafe fn new_write<MW: Word, PW: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: &'a [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    /// Create a new write DMA transfer (memory to peripheral), using raw pointers.
    ///
    /// Allows different memory and peripheral word sizes (e.g. writing `u8` buffer
    /// to a `u32` peripheral register).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let data = [0u8; 128];
    /// let transfer = unsafe {
    ///     Transfer::new_write_raw::<u8, u32>(
    ///         p.DMAC1_CH2,
    ///         Request::USART1_TX,
    ///         &data as *const [u8],
    ///         pac::USART1.dr().as_ptr() as *mut u32,
    ///         TransferOptions::default(),
    ///     )
    /// };
    /// transfer.await;
    /// ```
    pub unsafe fn new_write_raw<MW: Word, PW: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: *const [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const MW as *mut u32,
            buf.len(),
            Increment::Memory,
            MW::size(),
            PW::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral), writing the same value repeatedly.
    ///
    /// Useful for clearing a peripheral FIFO or sending a constant pattern.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let zero: u32 = 0;
    /// // Write 0 to the peripheral register 1024 times
    /// let transfer = unsafe {
    ///     Transfer::new_write_repeated(
    ///         p.DMAC1_CH3,
    ///         Request::SPI1_TX,
    ///         &zero,
    ///         1024,
    ///         pac::SPI1.dr().as_ptr() as *mut u32,
    ///         TransferOptions::default(),
    ///     )
    /// };
    /// transfer.await;
    /// ```
    pub unsafe fn new_write_repeated<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            Increment::None,
            W::size(),
            W::size(),
            options,
        )
    }

    /// Create a new memory-to-memory DMA transfer.
    ///
    /// Copies data from `src` to `dst`. Both slices must have the same length.
    /// The transfer starts immediately without waiting for a peripheral request.
    ///
    /// # Safety
    /// - `src` and `dst` must not overlap.
    /// - Both buffers must remain valid for the lifetime of the transfer.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let src = [1u32, 2, 3, 4];
    /// let mut dst = [0u32; 4];
    /// unsafe {
    ///     Transfer::new_transfer(
    ///         p.DMAC1_CH1,
    ///         &src,
    ///         &mut dst,
    ///         TransferOptions::default(),
    ///     )
    /// }.await;
    /// assert_eq!(dst, [1, 2, 3, 4]);
    /// ```
    pub unsafe fn new_transfer<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        src: &'a [W],
        dst: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        assert!(src.len() == dst.len());
        Self::new_transfer_raw(
            channel,
            src.as_ptr(),
            dst.as_mut_ptr(),
            src.len(),
            Increment::Both,
            options,
        )
    }

    /// Create a memory-to-memory DMA transfer with raw pointers.
    ///
    /// This is the low-level M2M API. The caller controls source/destination
    /// addresses, transfer count, and which addresses increment.
    ///
    /// In M2M mode, CPAR holds the source address and CM0AR holds the
    /// destination address. The `incr` parameter controls which of them
    /// auto-increment after each beat:
    ///
    /// | `incr` | Source (CPAR) | Dest (CM0AR) | Use case |
    /// |--------|-------------|-------------|----------|
    /// | `Both` | increments | increments | memcpy |
    /// | `Memory` | **fixed** | increments | HW register dump, memset |
    /// | `Peripheral` | increments | **fixed** | scatter read |
    /// | `None` | **fixed** | **fixed** | single-word relay |
    ///
    /// # Safety
    /// - `src` and `dst` must be valid, aligned addresses for type `W`.
    /// - The accessed region must remain valid for the lifetime of the transfer.
    /// - Source and destination regions must not overlap.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // memset: fill dst with a fixed value
    /// let pattern: u32 = 0xDEAD_BEEF;
    /// let mut dst = [0u32; 256];
    /// unsafe {
    ///     Transfer::new_transfer_raw(
    ///         p.DMAC1_CH1,
    ///         &pattern as *const u32,       // fixed source
    ///         dst.as_mut_ptr(),              // incrementing destination
    ///         dst.len(),
    ///         Increment::Memory,             // only dest increments
    ///         TransferOptions::default(),
    ///     )
    /// }.await;
    /// assert!(dst.iter().all(|&v| v == 0xDEAD_BEEF));
    /// ```
    pub unsafe fn new_transfer_raw<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        src: *const W,
        dst: *mut W,
        count: usize,
        incr: Increment,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);
        assert!(count > 0 && count <= 0xFFFF);

        let channel: PeripheralRef<'a, AnyChannel> = channel.map_into();
        // M2M mode ignores the request field; use Mpi1 as a dummy value.
        channel.configure(
            Request::Mpi1,
            Dir::PeripheralToMemory,
            src as *const u32,
            dst as *mut u32,
            count,
            incr,
            true,
            W::size(),
            W::size(),
            options,
        );
        channel.start();
        Self { channel }
    }

    unsafe fn new_inner(
        channel: PeripheralRef<'a, AnyChannel>,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr: Increment,
        mem_size: WordSize,
        peri_size: WordSize,
        options: TransferOptions,
    ) -> Self {
        assert!(mem_len > 0 && mem_len <= 0xFFFF);

        channel.configure(
            request, dir, peri_addr, mem_addr, mem_len, incr, false, mem_size, peri_size, options,
        );
        channel.start();
        Self { channel }
    }

    /// Request the transfer to stop.
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let mut transfer = unsafe { Transfer::new_read(ch, req, peri, &mut buf, opts) };
    /// // ... some condition ...
    /// transfer.request_stop();
    /// while transfer.is_running() {}
    /// ```
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let mut transfer = unsafe { Transfer::new_read(ch, req, peri, &mut buf, opts) };
    /// transfer.request_pause();
    /// while transfer.is_running() {}
    /// // ... later, resume ...
    /// // transfer.start();
    /// ```
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// if !transfer.is_running() {
    ///     defmt::info!("DMA transfer complete");
    /// }
    /// ```
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Gets the total remaining transfers for the channel.
    /// Note: this will be zero for transfers that completed without cancellation.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let remaining = transfer.get_remaining_transfers();
    /// defmt::info!("remaining: {} words", remaining);
    /// ```
    pub fn get_remaining_transfers(&self) -> u16 {
        self.channel.get_remaining_transfers()
    }

    /// Blocking wait until the transfer finishes.
    ///
    /// Polls the TCIF (Transfer Complete Interrupt Flag) in ISR, which is set
    /// by hardware when CNDTR reaches 0. Unlike polling the EN bit (which
    /// STM32 auto-clears but SiFli does not), this works on all controllers
    /// regardless of interrupt configuration.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let transfer = unsafe {
    ///     Transfer::new_transfer(p.DMAC1_CH1, &src, &mut dst, TransferOptions::default())
    /// };
    /// transfer.blocking_wait(); // spins until done, no async needed
    /// ```
    pub fn blocking_wait(self) {
        let info = self.channel.info();
        let r = info.dma;
        let ch = info.num;

        // Poll TCIF — hardware sets this when CNDTR reaches 0
        while !r.isr().read().tcif(ch) {}

        // Clear TCIF and disable channel
        r.ifcr().write(|w| w.set_ctcif(ch, true));
        r.ccr(ch).modify(|w| w.set_en(false));

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state: &ChannelState = &STATE[self.channel.state_index()];

        state.waker.register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}
// ==============================

struct DmaCtrlImpl<'a>(PeripheralRef<'a, AnyChannel>);

impl<'a> DmaCtrl for DmaCtrlImpl<'a> {
    fn get_remaining_transfers(&self) -> usize {
        self.0.get_remaining_transfers() as _
    }

    fn reset_complete_count(&mut self) -> usize {
        let state = &STATE[self.0.state_index()];
        return state.complete_count.swap(0, Ordering::AcqRel);
    }

    fn set_waker(&mut self, waker: &Waker) {
        STATE[self.0.state_index()].waker.register(waker);
    }
}

/// Ringbuffer for receiving data using DMA circular mode.
///
/// # Example
///
/// ```rust,ignore
/// let mut dma_buf = [0u8; 256];
/// let mut ring = unsafe {
///     ReadableRingBuffer::new(
///         p.DMAC1_CH1,
///         Request::USART1_RX,
///         pac::USART1.dr().as_ptr() as *mut u8,
///         &mut dma_buf,
///         TransferOptions::default(),
///     )
/// };
/// ring.start();
///
/// let mut tmp = [0u8; 64];
/// loop {
///     let (count, _remaining) = ring.read(&mut tmp).unwrap();
///     if count > 0 {
///         defmt::info!("received {} bytes", count);
///     }
///     // or use async:
///     // ring.read_exact(&mut tmp).await.unwrap();
/// }
/// ```
pub struct ReadableRingBuffer<'a, W: Word> {
    channel: PeripheralRef<'a, AnyChannel>,
    ringbuf: ReadableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> ReadableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        into_ref!(channel);
        let channel: PeripheralRef<'a, AnyChannel> = channel.map_into();

        let buffer_ptr = buffer.as_mut_ptr();
        let len = buffer.len();
        let dir = Dir::PeripheralToMemory;
        let data_size = W::size();

        options.half_transfer_ir = true;
        options.complete_transfer_ir = true;
        options.circular = true;

        channel.configure(
            request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            Increment::Memory,
            false,
            data_size,
            data_size,
            options,
        );

        Self {
            channel,
            ringbuf: ReadableDmaRingBuffer::new(buffer),
        }
    }

    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start();
    }

    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf
            .reset(&mut DmaCtrlImpl(self.channel.reborrow()));
    }

    /// Read elements from the ring buffer
    /// Return a tuple of the length read and the length remaining in the buffer
    /// If not all of the elements were read, then there will be some elements in the buffer remaining
    /// The length remaining is the capacity, ring_buf.len(), less the elements remaining after the read
    /// Error is returned if the portion to be read was overwritten by the DMA controller.
    pub fn read(&mut self, buf: &mut [W]) -> Result<(usize, usize), Error> {
        self.ringbuf
            .read(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }

    /// Read an exact number of elements from the ringbuffer.
    ///
    /// Returns the remaining number of elements available for immediate reading.
    /// Error is returned if the portion to be read was overwritten by the DMA controller.
    ///
    /// Async/Wake Behavior:
    /// The underlying DMA peripheral only can wake us when its buffer pointer has reached the halfway point,
    /// and when it wraps around. This means that when called with a buffer of length 'M', when this
    /// ring buffer was created with a buffer of size 'N':
    /// - If M equals N/2 or N/2 divides evenly into M, this function will return every N/2 elements read on the DMA source.
    /// - Otherwise, this function may need up to N/2 extra elements to arrive before returning.
    pub async fn read_exact(&mut self, buffer: &mut [W]) -> Result<usize, Error> {
        self.ringbuf
            .read_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }

    /// The current length of the ringbuffer
    pub fn len(&mut self) -> Result<usize, Error> {
        Ok(self
            .ringbuf
            .len(&mut DmaCtrlImpl(self.channel.reborrow()))?)
    }

    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }

    /// Request the DMA to stop.
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }

    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Stop the DMA transfer and await until the buffer is full.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when the buffer is full.
    ///
    /// This is designed to be used with streaming input data such as the
    /// I2S/SAI or ADC.
    ///
    /// When using the UART, you probably want `request_stop()`.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        //wait until cr.susp reads as true
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}

impl<'a, W: Word> Drop for ReadableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

/// Ringbuffer for writing data using DMA circular mode.
///
/// # Example
///
/// ```rust,ignore
/// let mut dma_buf = [0u8; 256];
/// let mut ring = unsafe {
///     WritableRingBuffer::new(
///         p.DMAC1_CH2,
///         Request::USART1_TX,
///         pac::USART1.dr().as_ptr() as *mut u8,
///         &mut dma_buf,
///         TransferOptions::default(),
///     )
/// };
/// ring.start();
///
/// let data = [0x48u8, 0x65, 0x6C, 0x6C, 0x6F];
/// ring.write_exact(&data).await.unwrap();
/// ```
pub struct WritableRingBuffer<'a, W: Word> {
    channel: PeripheralRef<'a, AnyChannel>,
    ringbuf: WritableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> WritableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        into_ref!(channel);
        let channel: PeripheralRef<'a, AnyChannel> = channel.map_into();

        let len = buffer.len();
        let dir = Dir::MemoryToPeripheral;
        let data_size = W::size();
        let buffer_ptr = buffer.as_mut_ptr();

        options.half_transfer_ir = true;
        options.complete_transfer_ir = true;
        options.circular = true;

        channel.configure(
            request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            Increment::Memory,
            false,
            data_size,
            data_size,
            options,
        );

        Self {
            channel,
            ringbuf: WritableDmaRingBuffer::new(buffer),
        }
    }

    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start();
    }

    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf
            .reset(&mut DmaCtrlImpl(self.channel.reborrow()));
    }

    /// Write elements directly to the raw buffer.
    /// This can be used to fill the buffer before starting the DMA transfer.
    pub fn write_immediate(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf.write_immediate(buf)
    }

    /// Write elements from the ring buffer
    /// Return a tuple of the length written and the length remaining in the buffer
    pub fn write(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf
            .write(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }

    /// Write an exact number of elements to the ringbuffer.
    pub async fn write_exact(&mut self, buffer: &[W]) -> Result<usize, Error> {
        self.ringbuf
            .write_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }

    /// Wait for any ring buffer write error.
    pub async fn wait_write_error(&mut self) -> Result<usize, Error> {
        self.ringbuf
            .wait_write_error(&mut DmaCtrlImpl(self.channel.reborrow()))
            .await
    }

    /// The current length of the ringbuffer
    pub fn len(&mut self) -> Result<usize, Error> {
        Ok(self
            .ringbuf
            .len(&mut DmaCtrlImpl(self.channel.reborrow()))?)
    }

    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }

    /// Request the DMA to stop.
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }

    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Stop the DMA transfer and await until the buffer is empty.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when all available data has been written.
    ///
    /// This is designed to be used with streaming output data such as the
    /// I2S/SAI or DAC.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        //wait until cr.susp reads as true
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}

impl<'a, W: Word> Drop for WritableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}
