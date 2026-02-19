//! Audio ADC driver
//!
//! Records audio through AUDCODEC ADC → AUDPRC RX_CH0 → DMA → SRAM.
//!
//! Supports three recording patterns:
//! - **One-shot**: [`read`](AudioAdc::read) / [`read_blocking`](AudioAdc::read_blocking)
//! - **Streaming**: [`start_stream`](AudioAdc::start_stream) — continuous recording via ring buffer

use core::marker::PhantomData;
use embassy_hal_internal::{into_ref, PeripheralRef};

use super::codec;
use super::{AdcConfig, ChannelMode, Error, RxCh0Dma};
use crate::dma::{ChannelAndRequest, ReadableRingBuffer, TransferOptions};
use crate::mode::{Async, Blocking, Mode};
use crate::pac;
use crate::{rcc, Peripheral};

fn audprc() -> pac::audprc::Audprc {
    pac::AUDPRC
}

fn rx_entry_addr() -> *mut u32 {
    audprc().rx_ch0_entry().as_ptr() as *mut u32
}

/// Unmask DMA request and enable RX_CH0.
fn rx_ch0_enable() {
    audprc().rx_ch0_cfg().modify(|w| {
        w.set_dma_msk(false);
        w.set_enable(true);
    });
}

/// Mask DMA request and disable RX_CH0.
fn rx_ch0_disable() {
    audprc().rx_ch0_cfg().modify(|w| {
        w.set_enable(false);
        w.set_dma_msk(true);
    });
}

/// Invalidate D-Cache for a memory region so CPU reads DMA-written data.
///
/// # Safety
/// The caller must ensure the address and size are valid.
unsafe fn invalidate_dcache(addr: usize, size: usize) {
    let mut scb = cortex_m::Peripherals::steal().SCB;
    scb.invalidate_dcache_by_address(addr, size);
}

/// Audio ADC driver.
///
/// Records audio through AUDCODEC ADC → AUDPRC RX_CH0 → DMA.
///
/// On [`Drop`], the ADC path is disabled and the analog codec is shut down.
pub struct AudioAdc<'d, M: Mode> {
    _peri: PeripheralRef<'d, crate::peripherals::AUDPRC>,
    rx_dma: ChannelAndRequest<'d>,
    config: AdcConfig,
    _phantom: PhantomData<M>,
}

// ============================================================================
// Blocking mode
// ============================================================================

impl<'d> AudioAdc<'d, Blocking> {
    /// Create a blocking-mode AudioAdc.
    ///
    /// Initializes AUDCODEC (PLL, bandgap, ADC analog) and AUDPRC (ADC path,
    /// RX channel). Ready to record after this returns.
    pub fn new_blocking(
        peri: impl Peripheral<P = crate::peripherals::AUDPRC> + 'd,
        rx_dma: impl Peripheral<P = impl RxCh0Dma<crate::peripherals::AUDPRC>> + 'd,
        config: AdcConfig,
    ) -> Self {
        into_ref!(peri);
        let rx_dma = new_dma!(rx_dma);

        Self::init_hardware(&config);

        Self {
            _peri: peri,
            rx_dma: rx_dma.unwrap(),
            config,
            _phantom: PhantomData,
        }
    }

    /// Read audio samples via DMA, blocking until the buffer is filled.
    ///
    /// D-Cache is automatically invalidated after the DMA transfer.
    ///
    /// `buf` must reside in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    pub fn read_blocking(&mut self, buf: &mut [u32]) {
        audprc()
            .rx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let transfer = unsafe {
            self.rx_dma
                .read(rx_entry_addr(), buf, TransferOptions::default())
        };

        rx_ch0_enable();
        transfer.blocking_wait();
        rx_ch0_disable();

        unsafe {
            invalidate_dcache(
                buf.as_ptr() as usize,
                buf.len() * core::mem::size_of::<u32>(),
            );
        }
    }
}

// ============================================================================
// Async mode
// ============================================================================

impl<'d> AudioAdc<'d, Async> {
    /// Create an async-mode AudioAdc.
    ///
    /// Requires binding the AUDPRC interrupt.
    pub fn new(
        peri: impl Peripheral<P = crate::peripherals::AUDPRC> + 'd,
        rx_dma: impl Peripheral<P = impl RxCh0Dma<crate::peripherals::AUDPRC>> + 'd,
        _irq: impl crate::interrupt::typelevel::Binding<
                crate::interrupt::typelevel::AUDPRC,
                super::InterruptHandler,
            > + 'd,
        config: AdcConfig,
    ) -> Self {
        into_ref!(peri);
        let rx_dma = new_dma!(rx_dma);

        Self::init_hardware(&config);

        use crate::interrupt::typelevel::Interrupt as _;
        crate::interrupt::typelevel::AUDPRC::unpend();
        unsafe { crate::interrupt::typelevel::AUDPRC::enable() };

        Self {
            _peri: peri,
            rx_dma: rx_dma.unwrap(),
            config,
            _phantom: PhantomData,
        }
    }

    /// Read audio samples via DMA, awaiting completion.
    ///
    /// D-Cache is automatically invalidated after the DMA transfer.
    ///
    /// `buf` must reside in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    pub async fn read(&mut self, buf: &mut [u32]) -> Result<(), Error> {
        audprc()
            .rx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let transfer = unsafe {
            self.rx_dma
                .read(rx_entry_addr(), buf, TransferOptions::default())
        };

        rx_ch0_enable();
        transfer.await;
        rx_ch0_disable();

        unsafe {
            invalidate_dcache(
                buf.as_ptr() as usize,
                buf.len() * core::mem::size_of::<u32>(),
            );
        }

        Ok(())
    }

    /// Start continuous streaming recording using a ring buffer.
    ///
    /// Returns an [`AudioInputStream`] that produces audio data via
    /// [`read`](AudioInputStream::read).
    /// The DMA runs in circular mode with half-transfer interrupts.
    ///
    /// `dma_buf` **must** be in SRAM — DMAC1 cannot access PSRAM (0x60000000).
    /// Recommended size: `2 * (sample_rate / 10)` for ~100ms per half-buffer.
    pub fn start_stream<'buf>(
        &'buf mut self,
        dma_buf: &'buf mut [u32],
    ) -> AudioInputStream<'buf> {
        audprc()
            .rx_ch0_cfg()
            .modify(|w| w.set_dma_msk(true));

        let mut ring = unsafe {
            ReadableRingBuffer::new(
                self.rx_dma.channel.reborrow(),
                self.rx_dma.request,
                rx_entry_addr(),
                dma_buf,
                TransferOptions::default(),
            )
        };

        ring.start();
        rx_ch0_enable();

        AudioInputStream { ring }
    }
}

// ============================================================================
// Shared methods
// ============================================================================

impl<'d, M: Mode> AudioAdc<'d, M> {
    /// Initialize all hardware: RCC, AUDCODEC, AUDPRC ADC path.
    fn init_hardware(config: &AdcConfig) {
        rcc::enable_and_reset::<crate::peripherals::AUDPRC>();
        rcc::enable::<crate::peripherals::AUDCODEC>();

        codec::init_codec_adc(config.volume.min(15));

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
        audprc.stb().modify(|w| {
            w.set_adc_div(config.sample_rate.adc_div());
        });

        // ADC path config: source = AUDCODEC
        let vol = config.volume.min(15);
        audprc.adc_path_cfg0().write(|w| {
            w.set_rough_vol_l(vol);
            w.set_fine_vol_l(0);
            w.set_rough_vol_r(vol);
            w.set_fine_vol_r(0);
            w.set_src_sel(false); // source = AUDCODEC
            w.set_data_swap(false);
            w.set_rx2tx_loopback(false);
        });

        // RX_CH0: 16-bit, stereo/mono, DMA masked until recording starts
        let stereo = matches!(config.channel_mode, ChannelMode::Stereo);
        audprc.rx_ch0_cfg().write(|w| {
            w.set_enable(false);
            w.set_format(false); // 16-bit
            w.set_mode(stereo);
            w.set_dma_msk(true);
        });

        // Enable ADC path + AUDPRC global
        audprc.cfg().modify(|w| w.set_adc_path_en(true));
        audprc.cfg().modify(|w| w.set_enable(true));
    }

    /// Set ADC volume (0-15).
    ///
    /// Updates both the AUDPRC digital path and AUDCODEC ADC channel gains
    /// to keep them in sync.
    pub fn set_volume(&mut self, vol: u8) {
        let vol = vol.min(15);
        self.config.volume = vol;

        audprc().adc_path_cfg0().modify(|w| {
            w.set_rough_vol_l(vol);
            w.set_rough_vol_r(vol);
        });

        let codec = pac::AUDCODEC;
        codec.adc_ch0_cfg().modify(|w| w.set_rough_vol(vol));
        codec.adc_ch1_cfg().modify(|w| w.set_rough_vol(vol));
    }

    /// Get current configuration.
    pub fn config(&self) -> &AdcConfig {
        &self.config
    }
}

impl<'d, M: Mode> Drop for AudioAdc<'d, M> {
    fn drop(&mut self) {
        rx_ch0_disable();

        audprc().cfg().modify(|w| {
            w.set_adc_path_en(false);
        });

        codec::shutdown_adc();
    }
}

// ============================================================================
// AudioInputStream
// ============================================================================

/// Continuous streaming recording via DMA ring buffer.
///
/// Created by [`AudioAdc::start_stream`]. Read audio data with
/// [`read`](Self::read); the ring buffer handles DMA double-buffering
/// automatically.
///
/// Dropping the stream stops DMA and disables the RX channel.
pub struct AudioInputStream<'a> {
    ring: ReadableRingBuffer<'a, u32>,
}

impl<'a> AudioInputStream<'a> {
    /// Read samples from the ring buffer, waiting until the buffer is filled.
    ///
    /// Returns the remaining number of elements available for immediate reading.
    /// D-Cache is automatically invalidated.
    pub async fn read(&mut self, buf: &mut [u32]) -> Result<usize, Error> {
        let remaining = self
            .ring
            .read_exact(buf)
            .await
            .map_err(|_| Error::Overrun)?;

        // DMA wrote to memory → invalidate D-Cache so CPU sees fresh data
        unsafe {
            invalidate_dcache(
                buf.as_ptr() as usize,
                buf.len() * core::mem::size_of::<u32>(),
            );
        }

        Ok(remaining)
    }

    /// Stop recording gracefully.
    pub async fn stop(mut self) {
        self.ring.stop().await;
        rx_ch0_disable();
    }

    /// Check if the ring buffer DMA is still running.
    pub fn is_running(&mut self) -> bool {
        self.ring.is_running()
    }
}

impl<'a> Drop for AudioInputStream<'a> {
    fn drop(&mut self) {
        // Disable RX and mask DMA request BEFORE ReadableRingBuffer drops.
        rx_ch0_disable();
    }
}
