//! ADC (Analog-to-Digital Converter)

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicBool, Ordering};
use core::task::Poll;

use embassy_hal_internal::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;
use sifli_pac::HPSYS_CFG;

use crate::_generated::{FIRST_CHANNEL_PIN, VBAT_CHANNEL_ID, VOL_OFFSET, VOL_RATIO};
use crate::gpio::{self, Analog};
use crate::interrupt::typelevel::Binding;
use crate::interrupt::InterruptExt;
use crate::mode::{Async, Blocking, Mode};
use crate::pac::gpadc::vals as AdcVals;
use crate::pac::GPADC;
use crate::peripherals;
use crate::{blocking_delay_us, interrupt, rcc};

static WAKER: AtomicWaker = AtomicWaker::new();
static IRQ_DONE: AtomicBool = AtomicBool::new(false);

/// ADC configuration.
/// f_ADCCLK = f_PCLK / (DATA_SAMP_DLY + CONV_WIDTH + SAMP_WIDTH + 2)
#[non_exhaustive]
pub struct Config {
    /// Sample width in ADCCLK cycles. Affects sample rate.
    pub sample_width: u32,
    /// Conversion width in ADCCLK cycles. Affects sample rate.
    pub conv_width: u8,
    /// Data sample delay in PCLK cycles. Affects sample rate.
    pub data_samp_dly: u8,
}

impl Default for Config {
    fn default() -> Self {
        // example\hal\adc\multichannel\src\main.c
        Self {
            sample_width: 0x71,
            conv_width: 75,
            data_samp_dly: 0x4,
        }
    }
}

/// ADC error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Conversion failed.
    ConversionFailed,
}

/// ADC sample.
/// The ADC returns a 12-bit result for single reads and a 13-bit result for DMA reads.
/// Both are stored in a u16.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct Sample(u16);

impl Sample {
    /// Get the raw sample value.
    pub fn value(&self) -> u16 {
        self.0
    }

    /// Convert the sample to millivolts.
    pub fn to_mv(&self) -> u16 {
        if self.0 <= VOL_OFFSET {
            0 // Below the offset, return 0 mV
        } else {
            // Convert to millivolts using the formula:
            // (sample - VOL_OFFSET) * VOL_RATIO / 1000(ADC_RATIO_ACCURATE)
            ((self.0 - VOL_OFFSET) as u32 * VOL_RATIO as u32 / 1000) as u16
        }
    }

    /// Convert the sample to volts (float).
    pub fn to_v_float(&self) -> f32 {
        if self.0 <= VOL_OFFSET {
            0.0 // Below the offset, return 0 V
        } else {
            // Convert to volts using the formula:
            // (sample - VOL_OFFSET) * VOL_RATIO / 1000000(ADC_RATIO_ACCURATE)
            (self.0 - VOL_OFFSET) as f32 * VOL_RATIO as f32 / 1_000_000.0
        }
    }
}

/// An ADC channel, which can be a pin or an internal source.
pub struct Channel<'p> {
    pub id: u8,
    phantom: PhantomData<&'p ()>,
}

/// A GPIO pin that can be used as an ADC channel.
///
/// The user must configure the pin's MUX to the ADC function.
pub trait AdcPin: gpio::Pin {
    fn adc_channel_id(&self) -> u8 {
        self.pin() - FIRST_CHANNEL_PIN
    }
}

impl<'p> Channel<'p> {
    /// Create a new ADC channel from a GPIO pin.
    pub fn new_pin(pin: impl AdcPin + 'p) -> Self {
        let id = pin.adc_channel_id();
        Analog::new(pin);
        Self {
            id,
            phantom: PhantomData,
        }
    }

    /// Create a new ADC channel for the internal battery voltage monitor.
    /// This corresponds to ADC channel 7.
    /// An ownership token for `ADC_VBAT` is required to ensure exclusive access.
    pub fn new_vbat(_vbat: impl Peripheral<P = peripherals::ADC_VBAT> + 'p) -> Self {
        Self {
            id: VBAT_CHANNEL_ID,
            phantom: PhantomData,
        }
    }
}

/// ADC driver.
pub struct Adc<'d, M: Mode> {
    _phantom: PhantomData<(&'d peripherals::GPADC, M)>,
}

impl<'d, M: Mode> Adc<'d, M> {
    /// Common initialization logic for both blocking and async modes.
    fn new_inner(_inner: impl Peripheral<P = peripherals::GPADC> + 'd, config: Config) -> Self {
        rcc::enable_and_reset::<peripherals::GPADC>();
        let regs = GPADC;

        // This initialization sequence is based on `HAL_ADC_Init` for
        // GPADC_CALIB_FLOW_VERSION == 3 (targeting SF32LB52x) and the user manual.

        // 1. Enable shared bandgap from HPSYS_CFG.
        // The manual suggests this is shared with the temperature sensor and recommends leaving it on.
        // This driver enables it but does not disable it on Drop, leaving that to the application owner.
        HPSYS_CFG.anau_cr().modify(|r| r.set_en_bg(true));

        // 2. Set ADC to single-ended mode by default.
        regs.cfg_reg1().modify(|r| r.set_anau_gpadc_se(true));

        // 3. Configure timing/width parameters from the Config struct.
        regs.ctrl_reg2().write(|w| {
            w.set_samp_width(config.sample_width);
            w.set_conv_width(config.conv_width);
        });
        regs.ctrl_reg().modify(|r| {
            r.set_data_samp_dly(config.data_samp_dly);
            // Set init time. The C HAL uses a value of 8 for SF32LB52x.
            r.set_init_time(8);
            // Disable hardware triggers by default.
            r.set_timer_trig_en(false);
        });

        // 4. Set default analog tuning parameters from the C HAL for SF32LB52x.
        regs.cfg_reg1().modify(|r| {
            r.set_anau_gpadc_vsp(AdcVals::Vsp::V0_642); // Value '2'
            r.set_anau_gpadc_cmm(0x10);
            r.set_anau_gpadc_en_v18(false); // SF32LB52x is 3.3V AVDD
        });

        // 5. Disable all conversion slots initially.
        for i in 0..8 {
            regs.slot(i).modify(|r| r.set_slot_en(false));
        }

        Self {
            _phantom: PhantomData,
        }
    }

    /// Prepares the ADC for a conversion by powering it up and waiting for stabilization.
    fn prepare(&mut self, channel: &Channel) {
        // From manual and `HAL_ADC_Prepare`.

        if channel.id == VBAT_CHANNEL_ID {
            // Enable battery monitoring path when using channel 7.
            HPSYS_CFG.anau_cr().modify(|r| r.set_en_vbat_mon(true));
        }

        // Necessary! Otherwise the data is incorrect (but why?)
        GPADC.slot(channel.id as _).modify(|r| r.set_slot_en(true));

        // 1. Enable the LDO that provides the reference voltage to the ADC.
        GPADC
            .cfg_reg1()
            .modify(|r| r.set_anau_gpadc_ldoref_en(true));
        // Manual: Wait 200us for LDO to stabilize.
        blocking_delay_us(200);

        // 2. Unmute ADC inputs to connect them to the external pins.
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_mute(false));

        // 3. Enable the main GPADC core logic.
        GPADC.ctrl_reg().modify(|r| r.set_frc_en_adc(true));
        // Manual: Wait 200us for GPADC core to stabilize.
        blocking_delay_us(200);
    }

    /// Powers down ADC components after a conversion to save power.
    fn finish(&mut self, channel: &Channel) {
        // Reverse of the `prepare` sequence.

        if channel.id == VBAT_CHANNEL_ID {
            // Disable battery monitoring path to save power.
            HPSYS_CFG.anau_cr().modify(|r| r.set_en_vbat_mon(false));
        }

        GPADC.ctrl_reg().modify(|r| r.set_frc_en_adc(false));
        GPADC.cfg_reg1().modify(|r| {
            r.set_anau_gpadc_ldoref_en(false);
            // Mute inputs to disconnect them.
            r.set_anau_gpadc_mute(true);
        });
    }

    /// Perform a single conversion on a channel in blocking mode.
    pub fn blocking_read(&mut self, ch: &mut Channel) -> Result<Sample, Error> {
        self.prepare(ch);

        // Use forced channel selection for single-shot conversions.
        GPADC.ctrl_reg().modify(|r| {
            r.set_adc_op_mode(false); // Single conversion mode
            r.set_chnl_sel_frc_en(true); // Enable forced channel selection
        });
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_sel_pch(ch.id));

        // Start the conversion.
        GPADC.ctrl_reg().modify(|r| r.set_adc_start(true));

        // Poll for completion flag (GPADC_IRSR).
        while !GPADC.gpadc_irq().read().gpadc_irsr() {}

        // Clear the interrupt flag by writing 1 to ICR.
        GPADC.gpadc_irq().write(|w| w.set_gpadc_icr(true));

        // In single conversion mode, the result is always in the even part of the first data register.
        let result = GPADC.rdata(0).read().even_slot_rdata();

        self.finish(ch);

        Ok(Sample(result & 0xfff))
    }
}

impl<'d, M: Mode> Drop for Adc<'d, M> {
    fn drop(&mut self) {
        // Ensure ADC is powered down when the driver is dropped.
        GPADC.ctrl_reg().modify(|r| r.set_frc_en_adc(false));
        GPADC
            .cfg_reg1()
            .modify(|r| r.set_anau_gpadc_ldoref_en(false));
        // The shared HPSYS bandgap (`EN_BG`) is not disabled here.
        // The application is responsible for managing it if it's no longer needed by any peripheral.
    }
}

impl<'d> Adc<'d, Blocking> {
    /// Create a new ADC driver in blocking mode.
    ///
    /// - `inner`: The ADC peripheral singleton.
    /// - `hpsys`: The HPSYS_CFG peripheral singleton, required for managing shared analog resources.
    /// - `config`: ADC timing and operational configuration.
    pub fn new_blocking(
        inner: impl Peripheral<P = peripherals::GPADC> + 'd,
        config: Config,
    ) -> Self {
        Self::new_inner(inner, config)
    }
}

/// ADC interrupt handler.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::GPADC> for InterruptHandler {
    unsafe fn on_interrupt() {
        if GPADC.gpadc_irq().read().gpadc_irsr() {
            IRQ_DONE.store(true, Ordering::SeqCst);
        }
        GPADC.gpadc_irq().modify(|w| w.set_gpadc_icr(true));
        WAKER.wake();
    }
}

impl<'d> Adc<'d, Async> {
    /// Create a new ADC driver in asynchronous mode.
    pub fn new(
        inner: impl Peripheral<P = peripherals::GPADC> + 'd,
        _irq: impl Binding<interrupt::typelevel::GPADC, InterruptHandler>,
        config: Config,
    ) -> Self {
        let s = Self::new_inner(inner, config);

        let irq = crate::interrupt::GPADC;
        irq.unpend();
        unsafe { irq.enable() };

        s
    }

    /// Waits asynchronously for the current ADC operation to complete.
    async fn wait_for_completion(&mut self) {
        let regs = GPADC;
        poll_fn(move |cx| {
            WAKER.register(cx.waker());
            // Re-enable interrupt mask before pending.
            regs.gpadc_irq().modify(|r| r.set_gpadc_imr(false));
            compiler_fence(Ordering::SeqCst);

            if IRQ_DONE.load(Ordering::SeqCst) {
                IRQ_DONE.store(false, Ordering::SeqCst);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Perform a single conversion on a channel asynchronously.
    pub async fn read(&mut self, ch: &mut Channel<'_>) -> Result<Sample, Error> {
        self.prepare(ch);

        // Configure for single-shot forced channel conversion.
        GPADC.ctrl_reg().modify(|r| {
            r.set_adc_op_mode(false);
            r.set_chnl_sel_frc_en(true);
        });
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_sel_pch(ch.id));

        // Enable interrupt and start conversion.
        GPADC.gpadc_irq().modify(|r| r.set_gpadc_imr(false));

        // Clear any previous IRQ done state before starting a new conversion.
        IRQ_DONE.store(false, Ordering::SeqCst);
        compiler_fence(Ordering::SeqCst);

        GPADC.ctrl_reg().modify(|r| r.set_adc_start(true));
        self.wait_for_completion().await;

        let result = GPADC.rdata(0).read().even_slot_rdata();

        self.finish(ch);

        Ok(Sample(result & 0xfff))
    }
}

#[allow(private_interfaces)]
pub(crate) trait SealedInstance:
    crate::rcc::RccEnableReset + crate::rcc::RccGetFreq
{
}

#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}
impl SealedInstance for peripherals::GPADC {}
impl Instance for peripherals::GPADC {
    type Interrupt = crate::interrupt::typelevel::GPADC;
}

dma_trait!(Dma, Instance);
