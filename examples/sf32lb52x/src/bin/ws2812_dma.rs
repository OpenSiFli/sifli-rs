#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::time::Hertz;
use sifli_hal::timer::{Channel, CountingMode, PwmPin, SimplePwm};
use sifli_hal::usart::{Config, Uart};
use {defmt_rtt as _, panic_probe as _};

// WS2812 LED driver - HSV rainbow cycle effect
//
// Hardware connection:
//   PA32 (GPTIM2_CH1) -> WS2812 DIN
//
// Effect:
//   Rainbow hue cycle (0-360 degrees)
//   50ms refresh (20Hz)
//
// Features:
//   DMA auto-updates CCR for perfect timing
//   Minimal CPU usage
//   Smooth HSV color space transitions

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    // UART debug output
    let mut config = Config::default();
    config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, config).unwrap();

    // Enable RGB LED power supply
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo3(true);
        w.set_vdd33_ldo3_pd(false);
    });

    let _ = writeln!(usart, "\r\n=== WS2812 with DMA (Perfect Solution!) ===");

    // Create PWM with DMA
    // NOTE: GPTIM2 is used by embassy time-driver, so we use GPTIM1 here
    let mut pwm = SimplePwm::new_with_dma(
        p.GPTIM1,
        Some(PwmPin::new(p.PA32)), // Auto-configures GPIO
        None,
        None,
        None,
        p.DMAC1_CH1, // DMA channel for Update events
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );

    let max = pwm.max_duty_cycle();
    let bit0 = max * 32 / 100;
    let bit1 = max * 64 / 100;

    let _ = writeln!(usart, "PWM configured with DMA:");
    let _ = writeln!(usart, "  Frequency: 800 kHz");
    let _ = writeln!(usart, "  Max duty: {}", max);
    let _ = writeln!(usart, "  Bit0: {} (32%), Bit1: {} (64%)", bit0, bit1);
    let _ = writeln!(usart, "  DMA: DMAC1_CH1 for GPTIM1_UPDATE\n");

    let _ = writeln!(usart, "🌈 Starting HSV Rainbow Cycle...");
    let _ = writeln!(usart, "   Hue: 0-360° @ 50ms/step (20Hz refresh)\n");

    let mut ws2812_data = [0u16; 25];
    let mut hue: u16 = 0; // 0-359 degrees

    loop {
        // HSV to RGB (H: 0-359, S: 100%, V: 50% for eye safety)
        let (r, g, b) = hsv_to_rgb(hue, 100, 50);

        // Encode color (GRB format, MSB first)
        for i in 0..8 {
            ws2812_data[i] = if (g & (0x80 >> i)) != 0 { bit1 } else { bit0 };
            ws2812_data[8 + i] = if (r & (0x80 >> i)) != 0 { bit1 } else { bit0 };
            ws2812_data[16 + i] = if (b & (0x80 >> i)) != 0 { bit1 } else { bit0 };
        }
        ws2812_data[24] = 0; // Reset

        // Send via DMA (can be called repeatedly)
        pwm.waveform_up_blocking(Channel::Ch1, &ws2812_data);

        // Reset timing
        Timer::after_micros(80).await;

        // Print every 30 degrees (reduce output)
        if hue.is_multiple_of(30) {
            let _ = writeln!(usart, "Hue: {:3}° -> RGB({:3}, {:3}, {:3})", hue, r, g, b);
        }

        // Increment hue, wrapping 0-359
        hue = (hue + 1) % 360;

        // 50ms refresh (20Hz, full cycle takes 360 x 50ms = 18s)
        Timer::after_millis(10).await;
    }
}

/// HSV to RGB color space conversion.
///
/// # Arguments
/// * `h` - Hue: 0-359 degrees
/// * `s` - Saturation: 0-100%
/// * `v` - Value (brightness): 0-100%
///
/// # Returns
/// (r, g, b) where each component is 0-255
fn hsv_to_rgb(h: u16, s: u8, v: u8) -> (u8, u8, u8) {
    let s = s as u16;
    let v = v as u16;

    let c = (v * s) / 100; // Chroma
    let h_prime = (h % 360) / 60; // 0-5
    let x = (c * (60 - ((h % 120) as i16 - 60).unsigned_abs())) / 60;

    let (r1, g1, b1) = match h_prime {
        0 => (c, x, 0),
        1 => (x, c, 0),
        2 => (0, c, x),
        3 => (0, x, c),
        4 => (x, 0, c),
        _ => (c, 0, x), // 5
    };

    let m = v - c;

    let r = ((r1 + m) * 255 / 100) as u8;
    let g = ((g1 + m) * 255 / 100) as u8;
    let b = ((b1 + m) * 255 / 100) as u8;

    (r, g, b)
}
