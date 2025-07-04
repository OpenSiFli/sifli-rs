//! ADC (Analog-to-Digital Converter)

#[cfg(feature = "sf32lb52x")]
const VBAT_CHANNEL_ID: u8 = 7; // The internal battery voltage monitor channel ID.
#[cfg(feature = "sf32lb52x")]
const FIRST_CHANNEL_PIN: u8 = 28;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use sifli_pac::HPSYS_CFG;

use crate::{blocking_delay_us, interrupt};
use crate::mode::{Async, Blocking, Mode};
use crate::gpio::{self, AnyPin, Pull};
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::interrupt::InterruptExt;
use crate::pac::gpadc::vals as AdcVals;
use crate::pac::gpadc::Gpadc;
use crate::pac::GPADC;
use crate::{pac, peripherals, PeripheralRef};

static WAKER: AtomicWaker = AtomicWaker::new();

/// ADC configuration.
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
        // Default values are from the reference manual's reset values for ADC_CTRL_REG2 and ADC_CTRL_REG.
        // f_ADCCLK = f_PCLK / (DATA_SAMP_DLY + CONV_WIDTH + SAMP_WIDTH + 2)
        Self {
            sample_width: 0x8000,
            conv_width: 0x80,
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
    ///
    /// The user is responsible for setting the appropriate pin function (MUX) for the ADC.
    /// The `pull` parameter is for API compatibility with other HALs but is not
    /// used, as pad settings are not specified for ADC operation in the manual.
    pub fn new_pin(pin: PeripheralRef<'p, impl AdcPin + 'p>, _pull: Pull) -> Self {
        Self {
            id: pin.adc_channel_id(),
            phantom: PhantomData,
        }
    }

    /// Create a new ADC channel for the internal battery voltage monitor.
    /// This corresponds to ADC channel 7.
    /// An ownership token for `ADC_VBAT` is required to ensure exclusive access.
    pub fn new_vbat(_vbat: PeripheralRef<'p, peripherals::ADC_VBAT>) -> Self {
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
    fn new_inner(
        _inner: PeripheralRef<'d, peripherals::GPADC>,
        config: Config,
    ) -> Self {
        // NOTE: Peripheral clock enablement is assumed to be handled by the Embassy runtime.

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
        // From manual section 8.1.3.10 and `HAL_ADC_Prepare`.

        // Handle channel-specific dependencies.
        if channel.id == VBAT_CHANNEL_ID {
            // Enable battery monitoring path when using channel 7.
            HPSYS_CFG.anau_cr().modify(|r| r.set_en_vbat_mon(true));
        }

        // 1. Enable the LDO that provides the reference voltage to the ADC.
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_ldoref_en(true));
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
}

impl<'d, M: Mode> Drop for Adc<'d, M> {
    fn drop(&mut self) {
        // Ensure ADC is powered down when the driver is dropped.
        GPADC.ctrl_reg().modify(|r| r.set_frc_en_adc(false));
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_ldoref_en(false));
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
        inner: PeripheralRef<'d, peripherals::GPADC>,
        config: Config,
    ) -> Self {
        Self::new_inner(inner, config)
    }

    /// Perform a single conversion on a channel in blocking mode.
    pub fn read(&mut self, ch: &mut Channel) -> Result<u16, Error> {
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

        Ok(result)
    }
}

/// ADC interrupt handler.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::GPADC> for InterruptHandler {
    unsafe fn on_interrupt() {
        let regs = Gpadc::from_ptr(pac::GPADC.as_ptr());
        // Disable interrupt mask to prevent spurious wakeups.
        regs.gpadc_irq().modify(|r| r.set_gpadc_imr(false));
        // Clear the raw interrupt status.
        regs.gpadc_irq().write(|w| w.set_gpadc_icr(true));
        WAKER.wake();
    }
}

impl<'d> Adc<'d, Async> {
    /// Create a new ADC driver in asynchronous mode.
    pub fn new(
        inner: PeripheralRef<'d, peripherals::GPADC>,
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
            regs.gpadc_irq().modify(|r| r.set_gpadc_imr(true));
            compiler_fence(Ordering::SeqCst);

            // Check if already completed to handle race condition.
            if regs.gpadc_irq().read().gpadc_irsr() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Perform a single conversion on a channel asynchronously.
    pub async fn read(&mut self, ch: &mut Channel<'_>) -> Result<u16, Error> {
        self.prepare(ch);

        // Configure for single-shot forced channel conversion.
        GPADC.ctrl_reg().modify(|r| {
            r.set_adc_op_mode(false);
            r.set_chnl_sel_frc_en(true);
        });
        GPADC.cfg_reg1().modify(|r| r.set_anau_gpadc_sel_pch(ch.id));

        // Enable interrupt and start conversion.
        GPADC.gpadc_irq().modify(|r| r.set_gpadc_imr(true));
        GPADC.ctrl_reg().modify(|r| r.set_adc_start(true));

        self.wait_for_completion().await;

        let result = GPADC.rdata(0).read().even_slot_rdata();

        self.finish(ch);

        Ok(result)
    }
}
