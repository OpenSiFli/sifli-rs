//! Simple PWM driver

use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use super::low_level::{CountingMode, OutputCompareMode, Timer};
use super::{Ch1, Ch2, Ch3, Ch4, Channel, GptimInstance, TimerChannel};
use crate::dma::ChannelAndRequest;
use crate::gpio::{AnyPin, Pin as GpioPin};
use crate::time::Hertz;

/// Timer pin trait
pub trait TimerPin<T, C>: GpioPin
where
    T: super::Instance,
{
    /// Get the FSEL value for this pin as timer function
    fn fsel(&self) -> u8;

    /// Configure HPSYS_CFG pin routing if needed
    fn set_cfg_pin(&self) {}
}

/// PWM pin wrapper
pub struct PwmPin<'d, T, C>
where
    T: super::GptimInstance, // Only GPTIM supports SimplePwm for now
    C: TimerChannel,
{
    _pin: PeripheralRef<'d, AnyPin>,
    _phantom: PhantomData<(T, C)>,
}

impl<'d, T, C> PwmPin<'d, T, C>
where
    T: super::GptimInstance,
    C: TimerChannel,
{
    /// Create a new PWM pin
    ///
    /// This will automatically configure:
    /// 1. GPIO FSEL (AF function)
    /// 2. GPIO output enable (DOER)
    /// 3. Timer PINR (pin routing) - SiFli specific!
    pub fn new(pin: impl Peripheral<P = impl TimerPin<T, C>> + 'd) -> Self {
        into_ref!(pin);

        let pin_num = pin.pin();
        let fsel_val = pin.fsel();

        critical_section::with(|_| {
            // 1. Configure GPIO FSEL (AF function select)
            if pin_num <= 38 {
                crate::pac::HPSYS_PINMUX
                    .pad_pa0_38(pin_num as usize)
                    .modify(|w| {
                        w.set_fsel(fsel_val);
                        w.set_pe(false); // No pull
                    });
            } else if pin_num <= 42 {
                crate::pac::HPSYS_PINMUX
                    .pad_pa39_42((pin_num - 39) as usize)
                    .modify(|w| {
                        w.set_fsel(fsel_val);
                        w.set_pe(false);
                    });
            } else if pin_num <= 44 {
                crate::pac::HPSYS_PINMUX
                    .pad_pa43_44((pin_num - 43) as usize)
                    .modify(|w| {
                        w.set_fsel(fsel_val);
                        w.set_pe(false);
                    });
            }

            // 1b. Configure HPSYS_CFG PINR (required by some peripherals)
            pin.set_cfg_pin();

            // 2. Enable GPIO output
            // PA0-PA31 in Bank 0, PA32-PA44 in Bank 1
            if pin_num < 32 {
                crate::pac::HPSYS_GPIO
                    .doesr0()
                    .write_value(crate::pac::hpsys_gpio::regs::Doesr0(1 << pin_num));
            } else {
                crate::pac::HPSYS_GPIO
                    .doesr1()
                    .write_value(crate::pac::hpsys_gpio::regs::Doesr1(1 << (pin_num - 32)));
            }

            // 3. Configure Timer PINR (Pin Routing) - SiFli specific!
            // Done by pin.set_cfg_pin()
        });

        Self {
            _pin: pin.map_into(),
            _phantom: PhantomData,
        }
    }
}

/// Simple PWM driver
pub struct SimplePwm<'d, T: GptimInstance> {
    inner: Timer<'d, T>,
    update_dma: Option<ChannelAndRequest<'d>>,
}

impl<'d, T: GptimInstance> SimplePwm<'d, T> {
    /// Create a new SimplePwm without DMA support
    ///
    /// Use `new_with_dma()` if you need waveform generation (e.g., for WS2812).
    pub fn new(
        tim: impl Peripheral<P = T> + 'd,
        _ch1: Option<PwmPin<'d, T, Ch1>>,
        _ch2: Option<PwmPin<'d, T, Ch2>>,
        _ch3: Option<PwmPin<'d, T, Ch3>>,
        _ch4: Option<PwmPin<'d, T, Ch4>>,
        freq: Hertz,
        counting_mode: CountingMode,
    ) -> Self {
        Self::new_inner(tim, None, freq, counting_mode)
    }

    /// Create a new SimplePwm with Update DMA support
    ///
    /// This enables `waveform_up_blocking()` for WS2812 and other waveform generation.
    ///
    /// # Arguments
    /// * `tim` - Timer peripheral
    /// * `ch1` - Optional PWM pin for channel 1 (auto-configures GPIO, AF, PINR)
    /// * `ch2` - Optional PWM pin for channel 2
    /// * `ch3` - Optional PWM pin for channel 3
    /// * `ch4` - Optional PWM pin for channel 4
    /// * `update_dma` - DMA channel for Update events
    /// * `freq` - PWM frequency
    /// * `counting_mode` - Counting mode (up or down)
    pub fn new_with_dma(
        tim: impl Peripheral<P = T> + 'd,
        _ch1: Option<PwmPin<'d, T, Ch1>>,
        _ch2: Option<PwmPin<'d, T, Ch2>>,
        _ch3: Option<PwmPin<'d, T, Ch3>>,
        _ch4: Option<PwmPin<'d, T, Ch4>>,
        update_dma: impl Peripheral<P = impl super::UpDma<T>> + 'd,
        freq: Hertz,
        counting_mode: CountingMode,
    ) -> Self {
        into_ref!(update_dma);
        let dma_req = update_dma.request();
        let dma_and_req = Some(ChannelAndRequest {
            channel: update_dma.map_into(),
            request: dma_req,
        });

        Self::new_inner(tim, dma_and_req, freq, counting_mode)
    }

    /// Internal constructor
    fn new_inner(
        tim: impl Peripheral<P = T> + 'd,
        update_dma: Option<ChannelAndRequest<'d>>,
        freq: Hertz,
        counting_mode: CountingMode,
    ) -> Self {
        let mut inner = Timer::new(tim);

        // Configure counting mode
        inner.set_counting_mode(counting_mode);

        // Set frequency
        inner.set_frequency(freq);

        // Configure all channels to PWM Mode 1
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            inner.set_output_compare_mode(ch, OutputCompareMode::PwmMode1);
            inner.set_output_compare_preload(ch, true);
        }

        // Enable auto-reload preload
        inner.set_autoreload_preload(true);

        // Generate update event to load preload registers
        inner.generate_update_event();

        // Start the timer
        inner.start();

        Self { inner, update_dma }
    }

    /// Get max duty cycle value
    ///
    /// This value depends on the configured frequency and timer clock.
    /// The duty cycle ranges from 0 (0%) to max_duty_cycle() (100%).
    pub fn max_duty_cycle(&self) -> u16 {
        self.inner.get_autoreload() + 1
    }

    /// Set PWM frequency
    ///
    /// Note: This will temporarily stop the timer to reconfigure it.
    pub fn set_frequency(&mut self, freq: Hertz) {
        self.inner.set_frequency(freq);
    }

    /// Get a PWM channel
    ///
    /// # Example
    /// ```ignore
    /// let mut ch1 = pwm.channel(Channel::Ch1);
    /// ch1.enable();
    /// ch1.set_duty_cycle(pwm.max_duty_cycle() / 2); // 50% duty
    /// ```
    pub fn channel(&mut self, channel: Channel) -> SimplePwmChannel<'_, T> {
        SimplePwmChannel {
            timer: unsafe { self.inner.clone_unchecked() },
            channel,
        }
    }

    /// Get channel 1
    pub fn ch1(&mut self) -> SimplePwmChannel<'_, T> {
        self.channel(Channel::Ch1)
    }

    /// Get channel 2
    pub fn ch2(&mut self) -> SimplePwmChannel<'_, T> {
        self.channel(Channel::Ch2)
    }

    /// Get channel 3
    pub fn ch3(&mut self) -> SimplePwmChannel<'_, T> {
        self.channel(Channel::Ch3)
    }

    /// Get channel 4
    pub fn ch4(&mut self) -> SimplePwmChannel<'_, T> {
        self.channel(Channel::Ch4)
    }

    /// Generate PWM waveform using DMA (blocking)
    ///
    /// This is useful for WS2812 LEDs where each bit requires a different duty cycle.
    /// The DMA will automatically update the compare value on each timer update event.
    ///
    /// # Panics
    /// Panics if SimplePwm was not created with `new_with_dma()`.
    ///
    /// # Example (WS2812)
    /// ```ignore
    /// let mut pwm = SimplePwm::new_with_dma(
    ///     p.GPTIM2,
    ///     Some(PwmPin::new(p.PA32)),
    ///     None, None, None,
    ///     p.DMAC1_CH1,  // DMA channel
    ///     Hertz::khz(800),
    ///     CountingMode::EdgeAlignedUp,
    /// );
    ///
    /// let max = pwm.max_duty_cycle();
    /// let ws2812_data = [...]; // GRB color data
    ///
    /// loop {
    ///     pwm.waveform_up_blocking(Channel::Ch1, &ws2812_data);  // Can be called repeatedly!
    ///     Timer::after_millis(100).await;
    /// }
    /// ```
    pub fn waveform_up_blocking(&mut self, channel: Channel, duty: &[u16]) {
        use crate::dma::{Transfer, TransferOptions};

        let dma = self
            .update_dma
            .as_mut()
            .expect("waveform_up_blocking requires DMA. Use new_with_dma() instead of new()");

        // Debug: Check DMA state before starting
        #[cfg(feature = "defmt")]
        {
            let regs = self.inner.regs();
            defmt::debug!(
                "waveform_up_blocking: channel={}, len={}",
                channel.index(),
                duty.len()
            );
            defmt::debug!("  DIER.UDE before: {}", regs.dier().read().ude());
            defmt::debug!("  CCR before: {}", regs.ccr(channel.index()).read().ccr());
        }

        let original_ude = self.inner.get_update_dma_state();

        // Get CCRx register address
        let regs = self.inner.regs();
        let ccr_addr = regs.ccr(channel.index()).as_ptr();

        // Reference: C SDK HAL_GPT_PWM_Update_Start_DMA
        // Critical sequence:
        // 1. Start DMA first
        // 2. Then enable Update DMA request (DIER.UDE)
        // 3. Ensure channel and Timer are enabled

        // Start DMA transfer using reborrow
        unsafe {
            // 1. Start DMA (configures and initiates DMA transfer)
            let transfer = Transfer::new_write(
                dma.channel.reborrow(),
                dma.request,
                duty,
                ccr_addr as *mut u16,
                TransferOptions::default(),
            );

            // 2. Enable Update DMA request (DMA is already waiting for Update events)
            if !original_ude {
                self.inner.enable_update_dma(true);
            }

            // 3. Ensure channel is enabled (should already be enabled in new())
            regs.ccer().modify(|w| w.set_cce(channel.index(), true));

            // 4. Ensure Timer is running (should already be started in new())
            regs.cr1().modify(|w| w.set_cen(true));

            // Wait for DMA transfer to complete
            transfer.blocking_wait();

            // 5. Disable Update DMA
            if !original_ude {
                self.inner.enable_update_dma(false);
            }
        }
    }
}

/// A single PWM channel
pub struct SimplePwmChannel<'d, T: GptimInstance> {
    timer: ManuallyDrop<Timer<'d, T>>,
    channel: Channel,
}

impl<'d, T: GptimInstance> SimplePwmChannel<'d, T> {
    /// Enable the channel output
    pub fn enable(&mut self) {
        self.timer.enable_channel(self.channel, true);
    }

    /// Disable the channel output
    pub fn disable(&mut self) {
        self.timer.enable_channel(self.channel, false);
    }

    /// Get max duty cycle value
    pub fn max_duty_cycle(&self) -> u16 {
        self.timer.get_autoreload() + 1
    }

    /// Set duty cycle
    ///
    /// # Arguments
    /// * `duty` - Duty cycle value from 0 (0%) to max_duty_cycle() (100%)
    ///
    /// # Panics
    /// Panics if duty > max_duty_cycle()
    pub fn set_duty_cycle(&mut self, duty: u16) {
        assert!(
            duty <= self.max_duty_cycle(),
            "duty must be <= max_duty_cycle()"
        );
        self.timer.set_compare_value(self.channel, duty);
    }

    /// Get current duty cycle
    pub fn get_duty_cycle(&self) -> u16 {
        self.timer.get_compare_value(self.channel)
    }

    /// Set duty cycle as percentage (0-100)
    ///
    /// # Arguments
    /// * `percent` - Duty cycle percentage (0-100)
    ///
    /// # Panics
    /// Panics if percent > 100
    pub fn set_duty_percent(&mut self, percent: u8) {
        assert!(percent <= 100, "percent must be <= 100");
        let duty = (self.max_duty_cycle() as u32 * percent as u32) / 100;
        self.set_duty_cycle(duty as u16);
    }
}
