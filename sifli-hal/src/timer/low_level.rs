//! Low-level Timer driver

use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};

use super::{Channel, GptimInstance, Instance};
use crate::pac::gptim::Gptim;
use crate::time::Hertz;

/// Output compare mode
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputCompareMode {
    /// Frozen - comparison has no effect on outputs
    Frozen,
    /// PWM Mode 1 - active when counter < compare value
    PwmMode1,
    /// PWM Mode 2 - inactive when counter < compare value
    PwmMode2,
}

/// Counting mode
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CountingMode {
    /// Edge-aligned up-counting
    EdgeAlignedUp,
    /// Edge-aligned down-counting
    EdgeAlignedDown,
}

/// Low-level Timer driver
pub struct Timer<'d, T: Instance> {
    _tim: PeripheralRef<'d, T>,
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: GptimInstance> Timer<'d, T> {
    /// Create a new Timer instance
    pub fn new(tim: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(tim);

        // Enable and reset the timer clock
        crate::rcc::enable_and_reset::<T>();

        Self {
            _tim: tim,
            _phantom: PhantomData,
        }
    }

    /// Get the register block
    pub(crate) fn regs(&self) -> Gptim {
        unsafe { Gptim::from_ptr(T::regs()) }
    }

    /// Clone the timer (unsafe)
    pub(crate) unsafe fn clone_unchecked(&self) -> ManuallyDrop<Self> {
        let tim = unsafe { self._tim.clone_unchecked() };
        ManuallyDrop::new(Self {
            _tim: tim,
            _phantom: PhantomData,
        })
    }

    /// Start the timer
    pub fn start(&mut self) {
        let regs = self.regs();
        regs.cr1().modify(|w| w.set_cen(true));
    }

    /// Stop the timer
    pub fn stop(&mut self) {
        let regs = self.regs();
        regs.cr1().modify(|w| w.set_cen(false));
    }

    /// Set prescaler
    pub fn set_prescaler(&mut self, psc: u16) {
        let regs = self.regs();
        regs.psc().write(|w| w.set_psc(psc));
    }

    /// Set auto-reload value
    pub fn set_autoreload(&mut self, arr: u16) {
        let regs = self.regs();
        regs.arr().write(|w| w.set_arr(arr));
    }

    /// Get auto-reload value
    pub fn get_autoreload(&self) -> u16 {
        let regs = self.regs();
        regs.arr().read().arr()
    }

    /// Generate update event
    pub fn generate_update_event(&mut self) {
        let regs = self.regs();
        regs.egr().write(|w| w.set_ug(true));
    }

    /// Set timer frequency
    pub fn set_frequency(&mut self, freq: Hertz) {
        // Use RccGetFreq trait to get timer clock frequency
        let timer_clk = T::frequency().expect("Timer clock not configured");
        let (psc, arr) = calculate_frequency_16bit(timer_clk, freq);

        self.set_prescaler(psc);
        self.set_autoreload(arr);
        self.generate_update_event();
    }

    /// Set counting mode
    pub fn set_counting_mode(&mut self, mode: CountingMode) {
        use crate::pac::tim_common::vals::DIR;
        let regs = self.regs();
        regs.cr1().modify(|w| {
            w.set_dir(match mode {
                CountingMode::EdgeAlignedUp => DIR::Up,
                CountingMode::EdgeAlignedDown => DIR::Down,
            });
        });
    }

    /// Set output compare mode for a channel
    pub fn set_output_compare_mode(&mut self, channel: Channel, mode: OutputCompareMode) {
        let regs = self.regs();
        let index = channel.index();

        let oc_mode = match mode {
            OutputCompareMode::Frozen => 0,
            OutputCompareMode::PwmMode1 => 6,
            OutputCompareMode::PwmMode2 => 7,
        };

        // Configure CCMRx register
        // CCMR1 controls CH1 and CH2 (index 0, 1)
        // CCMR2 controls CH3 and CH4 (index 2, 3)
        if index < 2 {
            regs.ccmr1().modify(|w| w.set_ocm(index, oc_mode));
        } else {
            regs.ccmr2().modify(|w| w.set_ocm(index - 2, oc_mode));
        }
    }

    /// Set compare value for a channel (duty cycle)
    pub fn set_compare_value(&mut self, channel: Channel, value: u16) {
        let regs = self.regs();
        regs.ccr(channel.index()).write(|w| w.set_ccr(value));
    }

    /// Get compare value for a channel
    pub fn get_compare_value(&self, channel: Channel) -> u16 {
        let regs = self.regs();
        regs.ccr(channel.index()).read().ccr()
    }

    /// Enable/disable a channel
    pub fn enable_channel(&mut self, channel: Channel, enable: bool) {
        let regs = self.regs();
        let index = channel.index();
        regs.ccer().modify(|w| w.set_cce(index, enable));
    }

    /// Set output compare preload enable
    pub fn set_output_compare_preload(&mut self, channel: Channel, enable: bool) {
        let regs = self.regs();
        let index = channel.index();

        // CCMR1 controls CH1 and CH2 (index 0, 1)
        // CCMR2 controls CH3 and CH4 (index 2, 3)
        if index < 2 {
            regs.ccmr1().modify(|w| w.set_ocpe(index, enable));
        } else {
            regs.ccmr2().modify(|w| w.set_ocpe(index - 2, enable));
        }
    }

    /// Set auto-reload preload enable
    pub fn set_autoreload_preload(&mut self, enable: bool) {
        let regs = self.regs();
        regs.cr1().modify(|w| w.set_arpe(enable));
    }

    /// Enable/disable Update DMA
    pub fn enable_update_dma(&mut self, enable: bool) {
        let regs = self.regs();
        regs.dier().modify(|w| w.set_ude(enable));
    }

    /// Get Update DMA state
    pub(crate) fn get_update_dma_state(&self) -> bool {
        let regs = self.regs();
        regs.dier().read().ude()
    }

    /// Generate PWM waveform using Update DMA (blocking)
    ///
    /// This is useful for WS2812 LEDs where each bit requires a different duty cycle.
    /// The DMA will automatically update the compare value on each timer update event.
    ///
    /// # Example (WS2812)
    /// ```ignore
    /// let max = pwm.max_duty_cycle();
    /// let bit0 = max * 32 / 100;  // ~0.4us high
    /// let bit1 = max * 64 / 100;  // ~0.8us high
    ///
    /// // GRB format: 24 bits + 1 reset
    /// let ws2812_data = [bit1, bit0, ...];
    /// timer.waveform_up_blocking(dma, Channel::Ch1, &ws2812_data);
    /// ```
    pub fn waveform_up_blocking(
        &mut self,
        dma: impl Peripheral<P = impl super::UpDma<T>>,
        channel: Channel,
        duty: &[u16],
    ) {
        use crate::dma::{Transfer, TransferOptions};

        let original_ude = self.get_update_dma_state();

        // Enable Update DMA if not already enabled
        if !original_ude {
            self.enable_update_dma(true);
        }

        // Get CCRx register address
        let regs = self.regs();
        let ccr_addr = regs.ccr(channel.index()).as_ptr();

        // Get DMA request from trait
        into_ref!(dma);
        let dma_req = dma.request();

        // Start DMA transfer
        unsafe {
            Transfer::new_write(
                dma,
                dma_req,
                duty,
                ccr_addr as *mut u16,
                TransferOptions::default(),
            )
            .blocking_wait(); // Block until complete
        }

        // Restore Update DMA state
        if !original_ude {
            self.enable_update_dma(false);
        }
    }
}

/// Calculate PSC and ARR for target frequency (16-bit ARR)
///
/// Based on C SDK implementation (drv_pwm.c line 926-931)
pub(crate) fn calculate_frequency_16bit(timer_clk: Hertz, target_freq: Hertz) -> (u16, u16) {
    const MAX_ARR: u32 = 0xFFFF;
    const MIN_PERIOD: u32 = 3;

    // Calculate period in timer ticks
    let period = (timer_clk.0 as u64) / (target_freq.0 as u64);

    // Calculate prescaler
    let psc = ((period / MAX_ARR as u64) + 1) as u32;

    // Calculate auto-reload value
    let mut arr = (period / psc as u64) as u32;

    if arr < MIN_PERIOD {
        arr = MIN_PERIOD;
    }

    ((psc - 1) as u16, (arr - 1) as u16)
}
