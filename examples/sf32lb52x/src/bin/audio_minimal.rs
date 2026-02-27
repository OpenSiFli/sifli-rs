//! Audio ADC recording example using HAL driver
//!
//! Records audio from ADCIN1 microphone via the AudioAdc HAL driver,
//! and prints peak-to-peak amplitude statistics over UART.
//!
//! Path: ADCIN1 (analog mic) → AUDCODEC ADC1 → AUDPRC RX → DMA → SRAM

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write as _;

use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
use sifli_hal::audio::{self, AudioAdc};
use sifli_hal::bind_interrupts;
use sifli_hal::usart::{Config as UartConfig, Uart};

bind_interrupts!(struct Irqs {
    AUDPRC => audio::InterruptHandler;
});

// DMA ring buffer: 960 u32 = 10ms * 2 half-buffers @ 48kHz stereo
// Must be in SRAM (DMAC1 cannot access PSRAM).
static mut DMA_BUF: [u32; 960] = [0u32; 960];

// Read buffer for processing
static mut SAMPLES: [u32; 480] = [0u32; 480];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    // UART debug
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();
    let _ = writeln!(usart, "\r\n=== Audio ADC HAL Recording ===");

    // Create async AudioAdc with default config (48kHz stereo)
    let pll = AudioPll::new(AudPllFreq::Mhz49_152);
    let mut adc = AudioAdc::new(
        p.AUDPRC,
        p.DMAC1_CH2,
        &pll,
        Irqs,
        audio::AdcConfig::default(),
    );
    let _ = writeln!(usart, "AudioAdc initialized");

    // Start continuous streaming
    let dma_buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };
    let mut stream = adc.start_stream(dma_buf);
    let _ = writeln!(usart, "Recording started - speak or clap near the mic!");

    let samples = unsafe { &mut *core::ptr::addr_of_mut!(SAMPLES) };
    let mut round = 0u32;

    loop {
        match stream.read(samples).await {
            Ok(_remaining) => {
                // Compute L and R channel statistics
                let mut min_l: i16 = i16::MAX;
                let mut max_l: i16 = i16::MIN;
                let mut min_r: i16 = i16::MAX;
                let mut max_r: i16 = i16::MIN;

                for &w in samples.iter() {
                    let l = w as i16;
                    let r = (w >> 16) as i16;
                    if l < min_l { min_l = l; }
                    if l > max_l { max_l = l; }
                    if r < min_r { min_r = r; }
                    if r > max_r { max_r = r; }
                }

                let pp_l = max_l as i32 - min_l as i32;
                let pp_r = max_r as i32 - min_r as i32;

                // Visual bar for L channel
                let bar_len = ((pp_l as u32).min(32768) * 40 / 32768) as usize;
                let mut bar = [b' '; 40];
                for b in bar.iter_mut().take(bar_len) {
                    *b = b'#';
                }
                let bar_str = core::str::from_utf8(&bar).unwrap_or("");

                let _ = writeln!(
                    usart,
                    "[{:4}] L: pp={:5} [{:6}..{:6}] R: pp={:5} |{}|",
                    round, pp_l, min_l, max_l, pp_r, bar_str
                );

                // Print raw hex every 20th round
                if round % 20 == 0 {
                    let _ = writeln!(usart, "  raw (first 4):");
                    for i in 0..4 {
                        let w = samples[i];
                        let _ = writeln!(
                            usart,
                            "    [{:2}] 0x{:08x}  L={:6} R={:6}",
                            i, w, w as i16, (w >> 16) as i16
                        );
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(usart, "Read error: {:?}", e);
                Timer::after_millis(100).await;
            }
        }

        round += 1;
    }
}
