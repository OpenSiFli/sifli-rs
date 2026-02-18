//! Audio DAC driver
//!
//! Drives audio output through AUDPRC TX_CH0 → AUDCODEC DAC → Class-D PA.
//!
//! Supports three playback patterns:
//! - **One-shot**: [`write`](AudioDac::write) / [`write_blocking`](AudioDac::write_blocking)
//! - **Circular**: [`start_circular`](AudioDac::start_circular) — loops a fixed buffer
//! - **Streaming**: [`start_stream`](AudioDac::start_stream) — continuous feed via ring buffer

use core::marker::PhantomData;
use embassy_hal_internal::{into_ref, PeripheralRef};

use super::codec;
use super::{ChannelMode, DacConfig, Error, TxCh0Dma};
use crate::dma::{ChannelAndRequest, Transfer, TransferOptions, WritableRingBuffer};
use crate::mode::{Async, Blocking, Mode};
use crate::pac;
use crate::{rcc, Peripheral};

fn audprc() -> pac::audprc::Audprc {
    pac::AUDPRC
}

fn tx_entry_addr() -> *mut u32 {
    audprc().tx_ch0_entry().as_ptr() as *mut u32
}

/// Unmask DMA request and enable TX_CH0.
fn tx_ch0_enable() {
    audprc().tx_ch0_cfg().modify(|w| {
        w.set_dma_msk(false);
        w.set_enable(true);
    });
}

/// Mask DMA request and disable TX_CH0.
fn tx_ch0_disable() {
    audprc().tx_ch0_cfg().modify(|w| {
        w.set_enable(false);
        w.set_dma_msk(true);
    });
}

/// Clean D-Cache for a memory region so DMA reads actual data.
///
/// # Safety
/// The caller must ensure the address and size are valid.
unsafe fn clean_dcache(addr: usize, size: usize) {
    let mut scb = cortex_m::Peripherals::steal().SCB;
    scb.clean_dcache_by_address(addr, size);
}

/// Audio DAC driver.
///
/// Outputs audio through AUDPRC TX_CH0 → AUDCODEC DAC → Class-D PA.
///
/// On [`Drop`], the DAC is muted, DMA stopped, and the analog path is
/// powered down to avoid leaving the speaker in an undefined state.
pub struct AudioDac<'d, M: Mode> {
    _peri: PeripheralRef<'d, crate::peripherals::AUDPRC>,
    tx_dma: ChannelAndRequest<'d>,
    config: DacConfig,
    _phantom: PhantomData<M>,
}

// ============================================================================
// Blocking mode
// ============================================================================

impl<'d> AudioDac<'d, Blocking> {
    /// Create a blocking-mode AudioDac.
    ///
    /// Initializes AUDCODEC (PLL, bandgap, DAC analog) and AUDPRC (mixer,
    /// strobe divider, TX channel). The DAC is unmuted and ready to play
    /// after this returns.
    pub fn new_blocking(
        peri: impl Peripheral<P = crate::peripherals::AUDPRC> + 'd,
        tx_dma: impl Peripheral<P = impl TxCh0Dma<crate::peripherals::AUDPRC>> + 'd,
        config: DacConfig,
    ) -> Self {
        into_ref!(peri);
        let tx_dma = new_dma!(tx_dma);

        Self::init_hardware(&config);

        Self {
            _peri: peri,
            tx_dma: tx_dma.unwrap(),
            config,
            _phantom: PhantomData,
        }
    }

    /// Write audio samples via DMA, blocking until the transfer completes.
    ///
    /// D-Cache is automatically cleaned before the DMA transfer.
    ///
    /// `samples` must reside in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    pub fn write_blocking(&mut self, samples: &[u32]) {
        unsafe {
            clean_dcache(
                samples.as_ptr() as usize,
                samples.len() * core::mem::size_of::<u32>(),
            );
        }

        audprc()
            .tx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let transfer = unsafe {
            self.tx_dma
                .write(samples, tx_entry_addr(), TransferOptions::default())
        };

        tx_ch0_enable();
        transfer.blocking_wait();
        tx_ch0_disable();
    }

    /// Start circular DMA playback of a fixed buffer.
    ///
    /// The DMA hardware loops the buffer indefinitely. Returns a
    /// [`CircularPlayback`] guard — playback stops when the guard is dropped.
    ///
    /// D-Cache is automatically cleaned before the DMA transfer.
    /// `samples` must reside in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    pub fn start_circular<'buf>(
        &'buf mut self,
        samples: &'buf [u32],
    ) -> CircularPlayback<'buf> {
        unsafe {
            clean_dcache(
                samples.as_ptr() as usize,
                samples.len() * core::mem::size_of::<u32>(),
            );
        }

        audprc()
            .tx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let mut opts = TransferOptions::default();
        opts.circular = true;
        opts.complete_transfer_ir = false;

        let transfer = unsafe {
            Transfer::new_write(
                self.tx_dma.channel.reborrow(),
                self.tx_dma.request,
                samples,
                tx_entry_addr(),
                opts,
            )
        };

        tx_ch0_enable();

        CircularPlayback {
            _transfer: transfer,
        }
    }
}

// ============================================================================
// Async mode
// ============================================================================

impl<'d> AudioDac<'d, Async> {
    /// Create an async-mode AudioDac.
    ///
    /// Requires binding the AUDPRC interrupt (used for future error detection;
    /// DMA completion is handled by the DMA interrupt layer).
    pub fn new(
        peri: impl Peripheral<P = crate::peripherals::AUDPRC> + 'd,
        tx_dma: impl Peripheral<P = impl TxCh0Dma<crate::peripherals::AUDPRC>> + 'd,
        _irq: impl crate::interrupt::typelevel::Binding<
                crate::interrupt::typelevel::AUDPRC,
                InterruptHandler,
            > + 'd,
        config: DacConfig,
    ) -> Self {
        into_ref!(peri);
        let tx_dma = new_dma!(tx_dma);

        Self::init_hardware(&config);

        use crate::interrupt::typelevel::Interrupt as _;
        crate::interrupt::typelevel::AUDPRC::unpend();
        unsafe { crate::interrupt::typelevel::AUDPRC::enable() };

        Self {
            _peri: peri,
            tx_dma: tx_dma.unwrap(),
            config,
            _phantom: PhantomData,
        }
    }

    /// Write audio samples via DMA, awaiting completion.
    ///
    /// D-Cache is automatically cleaned before the DMA transfer.
    ///
    /// Samples format depends on [`ChannelMode`]:
    /// - Stereo: each `u32` = `[R:hi16 | L:lo16]`
    /// - Mono: each `u32` low 16 bits = sample
    ///
    /// `samples` must reside in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    pub async fn write(&mut self, samples: &[u32]) -> Result<(), Error> {
        unsafe {
            clean_dcache(
                samples.as_ptr() as usize,
                samples.len() * core::mem::size_of::<u32>(),
            );
        }

        audprc()
            .tx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let transfer = unsafe {
            self.tx_dma
                .write(samples, tx_entry_addr(), TransferOptions::default())
        };

        tx_ch0_enable();
        transfer.await;
        tx_ch0_disable();

        Ok(())
    }

    /// Start continuous streaming playback using a ring buffer.
    ///
    /// Returns an [`AudioStream`] that accepts audio data via [`write`](AudioStream::write).
    /// The DMA runs in circular mode with half-transfer interrupts for
    /// gapless playback.
    ///
    /// `dma_buf` **must** be in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    /// Recommended size: `2 * (sample_rate / 10)` for ~100ms per half-buffer.
    ///
    /// The stream is automatically started; begin writing data immediately.
    pub fn start_stream<'buf>(
        &'buf mut self,
        dma_buf: &'buf mut [u32],
    ) -> AudioStream<'buf> {
        // Zero the DMA buffer to avoid playing garbage before first write().
        dma_buf.fill(0);

        audprc()
            .tx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let mut ring = unsafe {
            WritableRingBuffer::new(
                self.tx_dma.channel.reborrow(),
                self.tx_dma.request,
                tx_entry_addr(),
                dma_buf,
                TransferOptions::default(),
            )
        };

        ring.start();
        tx_ch0_enable();

        AudioStream { ring }
    }
}

// ============================================================================
// Shared methods
// ============================================================================

impl<'d, M: Mode> AudioDac<'d, M> {
    /// Initialize all hardware: RCC, AUDCODEC, AUDPRC.
    fn init_hardware(config: &DacConfig) {
        rcc::enable_and_reset::<crate::peripherals::AUDPRC>();
        rcc::enable_and_reset::<crate::peripherals::AUDCODEC>();

        codec::init_codec_dac(config.volume.min(15));

        let audprc = audprc();

        audprc.cfg().modify(|w| w.set_enable(false));

        // Flush FIFOs
        audprc.cfg().modify(|w| {
            w.set_dac_path_flush(true);
            w.set_adc_path_flush(true);
        });
        crate::blocking_delay_us(10);
        audprc.cfg().modify(|w| {
            w.set_dac_path_flush(false);
            w.set_adc_path_flush(false);
        });

        // Clock source
        audprc.cfg().modify(|w| {
            w.set_audclk_div(0);
            w.set_stb_clk_sel(config.sample_rate.stb_clk_sel());
            w.set_auto_gate_en(true);
        });
        audprc
            .cfg()
            .modify(|w| w.set_audclk_div_update(true));

        // Strobe divider
        let dac_div = config.sample_rate.dac_div();
        audprc.stb().write(|w| {
            w.set_dac_div(dac_div);
            w.set_adc_div(dac_div);
        });

        // DAC path mixer: TX_CH0_L → left, TX_CH0_R → right
        let vol = config.volume.min(15);
        audprc.dac_path_cfg0().write(|w| {
            w.set_rough_vol_l(vol);
            w.set_fine_vol_l(0);
            w.set_rough_vol_r(vol);
            w.set_fine_vol_r(0);
            w.set_mixlsrc0(0); // L: TX_CH0_L
            w.set_mixlsrc1(5); // mute
            w.set_mixrsrc0(1); // R: TX_CH0_R
            w.set_mixrsrc1(5); // mute
            w.set_dst_sel(0);
        });
        audprc.dac_path_cfg1().write(|w| {
            w.set_muxlsrc0(0);
            w.set_muxrsrc0(1);
            w.set_muxlsrc1(5);
            w.set_muxrsrc1(5);
            w.set_eq_ch_en(0);
            w.set_src_ch_en(0);
            w.set_src_hbf1_en(false);
            w.set_src_hbf2_en(false);
            w.set_src_hbf3_en(false);
            w.set_src_ch_clr(3);
        });

        // Wait for SRC clear done
        let mut timeout = 10_000u32;
        while audprc.dac_path_cfg1().read().src_ch_clr_done() == 0 && timeout > 0 {
            crate::blocking_delay_us(1);
            timeout -= 1;
        }
        audprc
            .dac_path_cfg1()
            .modify(|w| w.set_src_ch_clr(0));

        // TX_CH0: 16-bit, stereo/mono, DMA masked until playback starts
        let stereo = matches!(config.channel_mode, ChannelMode::Stereo);
        audprc.tx_ch0_cfg().write(|w| {
            w.set_enable(false);
            w.set_format(false); // 16-bit
            w.set_mode(stereo);
            w.set_dma_msk(true);
        });

        // Enable DAC path + AUDPRC global
        audprc.cfg().modify(|w| w.set_dac_path_en(true));
        audprc.cfg().modify(|w| w.set_enable(true));

        // Start DAC analog (muted), wait for stabilization, then unmute
        codec::start_dac_analog();
        crate::blocking_delay_us(10_000);
        codec::codec_unmute_dac();
    }

    /// Set DAC volume (0–15).
    pub fn set_volume(&mut self, vol: u8) {
        let vol = vol.min(15);
        self.config.volume = vol;

        audprc().dac_path_cfg0().modify(|w| {
            w.set_rough_vol_l(vol);
            w.set_rough_vol_r(vol);
        });
        codec::codec_set_volume(vol);
    }

    /// Mute or unmute DAC output.
    pub fn set_mute(&mut self, mute: bool) {
        if mute {
            codec::codec_mute_dac();
        } else {
            codec::codec_unmute_dac();
        }
    }

    /// Get current configuration.
    pub fn config(&self) -> &DacConfig {
        &self.config
    }
}

impl<'d, M: Mode> Drop for AudioDac<'d, M> {
    fn drop(&mut self) {
        tx_ch0_disable();

        audprc().cfg().modify(|w| {
            w.set_enable(false);
            w.set_dac_path_en(false);
        });

        codec::shutdown_dac();
    }
}

// ============================================================================
// CircularPlayback
// ============================================================================

/// Guard for circular DMA playback.
///
/// Created by [`AudioDac::start_circular`]. The DMA hardware loops the
/// pre-filled buffer indefinitely. Dropping this guard stops playback.
pub struct CircularPlayback<'a> {
    _transfer: Transfer<'a>,
}

impl<'a> CircularPlayback<'a> {
    /// Check if DMA is still running.
    pub fn is_running(&mut self) -> bool {
        self._transfer.is_running()
    }
}

impl<'a> Drop for CircularPlayback<'a> {
    fn drop(&mut self) {
        // Disable TX channel before Transfer::drop stops the DMA channel.
        tx_ch0_disable();
    }
}

// ============================================================================
// AudioStream
// ============================================================================

/// Continuous streaming playback via DMA ring buffer.
///
/// Created by [`AudioDac::start_stream`]. Write audio data with [`write`](Self::write);
/// the ring buffer handles DMA double-buffering automatically.
///
/// Dropping the stream stops DMA and disables the TX channel.
pub struct AudioStream<'a> {
    ring: WritableRingBuffer<'a, u32>,
}

impl<'a> AudioStream<'a> {
    /// Write samples to the ring buffer, waiting until all data is accepted.
    ///
    /// Returns the remaining free space in the ring buffer.
    pub async fn write(&mut self, samples: &[u32]) -> Result<usize, Error> {
        self.ring
            .write_exact(samples)
            .await
            .map_err(|_| Error::Dma)
    }

    /// Stop streaming gracefully, draining remaining buffered data.
    pub async fn stop(mut self) {
        self.ring.stop().await;
        tx_ch0_disable();
    }

    /// Check if the ring buffer DMA is still running.
    pub fn is_running(&mut self) -> bool {
        self.ring.is_running()
    }

    /// Get remaining free space in the ring buffer.
    pub fn free(&mut self) -> Result<usize, Error> {
        let len = self.ring.len().map_err(|_| Error::Dma)?;
        Ok(self.ring.capacity() - len)
    }
}

impl<'a> Drop for AudioStream<'a> {
    fn drop(&mut self) {
        // Disable TX and mask DMA request BEFORE WritableRingBuffer drops.
        // Rust drops fields in declaration order AFTER the explicit Drop body runs,
        // so `ring` drops after this. WritableRingBuffer::drop calls request_stop()
        // and spins on is_running(); disabling TX first ensures the DMA channel can
        // actually stop (otherwise the AUDPRC keeps requesting transfers).
        tx_ch0_disable();
    }
}

// ============================================================================
// Interrupt handler
// ============================================================================

/// Interrupt handler for AUDPRC.
///
/// DMA-based transfers use DMA channel interrupts managed by the DMA layer.
/// This handler clears AUDPRC-specific interrupt flags (reserved for future
/// error detection such as FIFO overrun).
pub struct InterruptHandler {
    _phantom: PhantomData<()>,
}

impl crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::AUDPRC>
    for InterruptHandler
{
    unsafe fn on_interrupt() {
        let audprc = audprc();
        let irq = audprc.irq().read();
        audprc.irq().write(|w| w.0 = irq.0);
    }
}
