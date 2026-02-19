//! Audio DAC — blocking circular playback of a 1kHz sine wave.
//!
//! Demonstrates `AudioDac::new_blocking()` + `start_circular()` for looping
//! a pre-filled DMA buffer indefinitely.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;

use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
use sifli_hal::audio::{AudioDac, DacConfig};
use sifli_hal::usart::{Config as UartConfig, Uart};

use libm::sinf;

const PI: f32 = core::f32::consts::PI;
const SAMPLE_RATE: u32 = 48000;
const SINE_FREQ: f32 = 1000.0;
const SAMPLES_PER_PERIOD: usize = (SAMPLE_RATE as f32 / SINE_FREQ) as usize; // 48
const BUF_SIZE: usize = SAMPLES_PER_PERIOD * 4; // 4 periods = 192

// DMA buffer must be in SRAM (not PSRAM)
static mut DMA_BUF: [u32; BUF_SIZE] = [0u32; BUF_SIZE];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    // UART debug
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();
    let _ = writeln!(usart, "\r\n=== Audio DAC Driver Test ===");

    // Fill buffer with stereo packed 1kHz sine
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };
    for i in 0..buf.len() {
        let t = (i % SAMPLES_PER_PERIOD) as f32 / SAMPLE_RATE as f32;
        let val = sinf(2.0 * PI * SINE_FREQ * t);
        let sample = (val * 20000.0) as i16;
        let s16 = sample as u16 as u32;
        buf[i] = s16 | (s16 << 16); // packed stereo: L=lo16, R=hi16
    }
    let _ = writeln!(usart, "Buffer: {} samples, stereo packed", BUF_SIZE);

    // PA26: speaker amplifier enable
    {
        let pa = sifli_hal::gpio::Output::new(&mut p.PA26, sifli_hal::gpio::Level::High);
        core::mem::forget(pa);
    }

    // Create DAC (blocking mode, 48kHz stereo)
    let pll = AudioPll::new(AudPllFreq::Mhz49_152);
    let mut dac = AudioDac::new_blocking(p.AUDPRC, p.DMAC1_CH1, &pll, DacConfig::default());
    let _ = writeln!(usart, "AudioDac initialized");

    // Start circular playback — DMA loops the buffer forever
    let buf = unsafe { &*core::ptr::addr_of!(DMA_BUF) };
    let _playback = dac.start_circular(buf);
    let _ = writeln!(usart, "Playing 1kHz sine (circular DMA)");

    // Keep running — dropping _playback would stop the DMA
    loop {
        Timer::after_millis(1000).await;
    }
}
