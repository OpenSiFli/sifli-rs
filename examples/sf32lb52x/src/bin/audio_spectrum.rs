//! Audio Spectrum Analyzer — Mic input visualized on LCDC
//!
//! Records audio from ADCIN1 microphone via AudioAdc HAL driver,
//! performs 256-point FFT, and displays a real-time frequency spectrum
//! bar chart on the CO5300 AMOLED display.
//!
//! Path: ADCIN1 mic → AUDCODEC ADC → AUDPRC RX → DMA → FFT → LCDC

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_time::Delay;

use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
use sifli_hal::audio::{self, AudioAdc};
use sifli_hal::bind_interrupts;
use sifli_hal::lcdc::{self, SpiConfig};
use sifli_hal::rcc::{ConfigBuilder, Dll, DllStage, Sysclk};
use sifli_hal::time::mhz;
use sifli_hal::gpio;

use display_driver::bus::QspiFlashBus;
use display_driver::panel::reset::LCDResetOption;
use display_driver::{ColorFormat, DisplayDriver};
use display_driver_co5300::{
    spec::{Co5300Spec, PanelSpec},
    Co5300,
};

use libm::{sinf, cosf, sqrtf, log10f};

// ============================================================================
// Display config
// ============================================================================

const WIDTH: usize = 390;
const HEIGHT: usize = 450;
const WIDTH_I: i32 = WIDTH as i32;
const HEIGHT_I: i32 = HEIGHT as i32;
const FB_SIZE: usize = WIDTH * HEIGHT * 2;

pub struct MyCo5300;
impl PanelSpec for MyCo5300 {
    const PHYSICAL_WIDTH: u16 = WIDTH as u16;
    const PHYSICAL_HEIGHT: u16 = HEIGHT as u16;
    const PHYSICAL_X_OFFSET: u16 = 0;
    const PHYSICAL_Y_OFFSET: u16 = 0;
    const BGR: bool = false;
}
impl Co5300Spec for MyCo5300 {
    const INIT_PAGE_PARAM: u8 = 0x20;
    const IGNORE_ID_CHECK: bool = true;
}

#[link_section = ".psram_bss"]
static mut FB_BUF: MaybeUninit<[u8; FB_SIZE]> = MaybeUninit::uninit();

bind_interrupts!(struct Irqs {
    AUDPRC => audio::InterruptHandler;
    LCDC1 => lcdc::InterruptHandler<sifli_hal::peripherals::LCDC1>;
});

// ============================================================================
// Audio config
// ============================================================================

// DMA ring buffer must be in SRAM (DMAC1 cannot access PSRAM)
// Larger buffer to tolerate write_frame latency (~10ms for 351KB)
static mut DMA_BUF: [u32; 4800] = [0u32; 4800]; // 50ms @ 48kHz stereo
static mut AUDIO_BUF: [u32; 256] = [0u32; 256];

// ============================================================================
// FFT (256-point radix-2 DIT)
// ============================================================================

const FFT_N: usize = 256;
const FFT_LOG2: usize = 8;
const PI: f32 = core::f32::consts::PI;

fn bit_reverse(mut x: usize, bits: usize) -> usize {
    let mut result = 0;
    for _ in 0..bits {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

/// 256-point in-place radix-2 DIT FFT.
/// Input/output: arrays of (real, imag) pairs.
fn fft256(re: &mut [f32; FFT_N], im: &mut [f32; FFT_N]) {
    // Bit-reversal permutation
    for i in 0..FFT_N {
        let j = bit_reverse(i, FFT_LOG2);
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages
    let mut half_size = 1usize;
    for _ in 0..FFT_LOG2 {
        let size = half_size << 1;
        let angle_step = -PI / half_size as f32;
        let mut k = 0;
        while k < FFT_N {
            for j in 0..half_size {
                let angle = angle_step * j as f32;
                let wr = cosf(angle);
                let wi = sinf(angle);

                let even = k + j;
                let odd = k + j + half_size;

                let tr = wr * re[odd] - wi * im[odd];
                let ti = wr * im[odd] + wi * re[odd];

                re[odd] = re[even] - tr;
                im[odd] = im[even] - ti;
                re[even] += tr;
                im[even] += ti;
            }
            k += size;
        }
        half_size = size;
    }
}

// ============================================================================
// Spectrum bar config
// ============================================================================

const NUM_BARS: usize = 32;
const BAR_WIDTH: i32 = 8;
const BAR_GAP: i32 = 3;
const SPECTRUM_TOP: i32 = 50;
const SPECTRUM_HEIGHT: i32 = 330;
const SPECTRUM_BOTTOM: i32 = SPECTRUM_TOP + SPECTRUM_HEIGHT;
const VU_TOP: i32 = SPECTRUM_BOTTOM + 15;
const VU_HEIGHT: i32 = 30;

/// Compute logarithmic frequency bin boundaries for bars.
/// Returns [NUM_BARS + 1] boundaries in bin indices (0..FFT_N/2).
fn compute_bar_boundaries() -> [usize; NUM_BARS + 1] {
    let mut bounds = [0usize; NUM_BARS + 1];
    let half = (FFT_N / 2) as f32;
    for i in 0..=NUM_BARS {
        // Logarithmic mapping: emphasize low frequencies
        let frac = i as f32 / NUM_BARS as f32;
        let bin = libm::powf(half, frac) as usize;
        bounds[i] = if i == 0 { 1 } else { bin.max(bounds[i - 1] + 1).min(FFT_N / 2) };
    }
    bounds
}

/// Bar color: green (low) → yellow (mid) → red (high).
fn bar_color(bar_idx: usize) -> u16 {
    let frac = bar_idx as f32 / (NUM_BARS - 1) as f32;
    let (r, g, b);
    if frac < 0.5 {
        // Green → Yellow
        let t = frac * 2.0;
        r = (t * 255.0) as u8;
        g = 255u8;
        b = 0u8;
    } else {
        // Yellow → Red
        let t = (frac - 0.5) * 2.0;
        r = 255u8;
        g = ((1.0 - t) * 255.0) as u8;
        b = 0u8;
    }
    rgb565(r, g, b)
}

#[inline(always)]
fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

// ============================================================================
// Pixel helpers
// ============================================================================

#[inline(always)]
fn set_pixel(buf: &mut [u8], x: i32, y: i32, color: u16) {
    if x >= 0 && x < WIDTH_I && y >= 0 && y < HEIGHT_I {
        let offset = (y as usize * WIDTH + x as usize) * 2;
        let bytes = color.to_le_bytes();
        buf[offset] = bytes[0];
        buf[offset + 1] = bytes[1];
    }
}

fn fill_rect(buf: &mut [u8], x0: i32, y0: i32, w: i32, h: i32, color: u16) {
    let x_start = x0.max(0);
    let x_end = (x0 + w).min(WIDTH_I);
    let y_start = y0.max(0);
    let y_end = (y0 + h).min(HEIGHT_I);
    let bytes = color.to_le_bytes();
    for y in y_start..y_end {
        let row_base = y as usize * WIDTH * 2;
        for x in x_start..x_end {
            let offset = row_base + x as usize * 2;
            buf[offset] = bytes[0];
            buf[offset + 1] = bytes[1];
        }
    }
}

/// Draw a horizontal line.
fn hline(buf: &mut [u8], x0: i32, x1: i32, y: i32, color: u16) {
    fill_rect(buf, x0, y, x1 - x0, 1, color);
}

// ============================================================================
// Simple 5x7 font for title/labels
// ============================================================================

fn font_glyph(ch: u8) -> [u8; 5] {
    match ch {
        b'A' => [0x7E, 0x09, 0x09, 0x09, 0x7E],
        b'B' => [0x7F, 0x49, 0x49, 0x49, 0x36],
        b'C' => [0x3E, 0x41, 0x41, 0x41, 0x22],
        b'D' => [0x7F, 0x41, 0x41, 0x41, 0x3E],
        b'E' => [0x7F, 0x49, 0x49, 0x49, 0x41],
        b'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
        b'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
        b'I' => [0x00, 0x41, 0x7F, 0x41, 0x00],
        b'L' => [0x7F, 0x40, 0x40, 0x40, 0x40],
        b'M' => [0x7F, 0x02, 0x04, 0x02, 0x7F],
        b'N' => [0x7F, 0x04, 0x08, 0x10, 0x7F],
        b'O' => [0x3E, 0x41, 0x41, 0x41, 0x3E],
        b'P' => [0x7F, 0x09, 0x09, 0x09, 0x06],
        b'R' => [0x7F, 0x09, 0x19, 0x29, 0x46],
        b'S' => [0x26, 0x49, 0x49, 0x49, 0x32],
        b'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
        b'U' => [0x3F, 0x40, 0x40, 0x40, 0x3F],
        b'V' => [0x1F, 0x20, 0x40, 0x20, 0x1F],
        b'W' => [0x3F, 0x40, 0x30, 0x40, 0x3F],
        b'X' => [0x63, 0x14, 0x08, 0x14, 0x63],
        b'Y' => [0x07, 0x08, 0x70, 0x08, 0x07],
        b'Z' => [0x61, 0x51, 0x49, 0x45, 0x43],
        b'a' => [0x20, 0x54, 0x54, 0x54, 0x78],
        b'c' => [0x38, 0x44, 0x44, 0x44, 0x28],
        b'd' => [0x38, 0x44, 0x44, 0x48, 0x7F],
        b'e' => [0x38, 0x54, 0x54, 0x54, 0x18],
        b'i' => [0x00, 0x44, 0x7D, 0x40, 0x00],
        b'l' => [0x00, 0x41, 0x7F, 0x40, 0x00],
        b'm' => [0x7C, 0x04, 0x18, 0x04, 0x78],
        b'n' => [0x7C, 0x08, 0x04, 0x04, 0x78],
        b'o' => [0x38, 0x44, 0x44, 0x44, 0x38],
        b'p' => [0x7C, 0x14, 0x14, 0x14, 0x08],
        b'r' => [0x7C, 0x08, 0x04, 0x04, 0x08],
        b's' => [0x48, 0x54, 0x54, 0x54, 0x24],
        b't' => [0x04, 0x3F, 0x44, 0x40, 0x20],
        b'u' => [0x3C, 0x40, 0x40, 0x20, 0x7C],
        b' ' => [0x00, 0x00, 0x00, 0x00, 0x00],
        b'0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
        b'1' => [0x00, 0x42, 0x7F, 0x40, 0x00],
        b'2' => [0x72, 0x49, 0x49, 0x49, 0x46],
        b'3' => [0x21, 0x41, 0x49, 0x4D, 0x33],
        b'4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
        b'5' => [0x27, 0x45, 0x45, 0x45, 0x39],
        b'6' => [0x3C, 0x4A, 0x49, 0x49, 0x31],
        b'7' => [0x41, 0x21, 0x11, 0x09, 0x07],
        b'8' => [0x36, 0x49, 0x49, 0x49, 0x36],
        b'9' => [0x46, 0x49, 0x49, 0x29, 0x1E],
        b'k' => [0x7F, 0x10, 0x28, 0x44, 0x00],
        _ => [0x00, 0x00, 0x00, 0x00, 0x00],
    }
}

fn draw_text(buf: &mut [u8], text: &[u8], x: i32, y: i32, color: u16, scale: i32) {
    let mut cx = x;
    for &ch in text {
        let glyph = font_glyph(ch);
        for col in 0..5i32 {
            let bits = glyph[col as usize];
            for row in 0..7i32 {
                if bits & (1 << row) != 0 {
                    for sy in 0..scale {
                        for sx in 0..scale {
                            set_pixel(buf, cx + col * scale + sx, y + row * scale + sy, color);
                        }
                    }
                }
            }
        }
        cx += 6 * scale;
    }
}

// ============================================================================
// Hann window
// ============================================================================

fn apply_hann_window(data: &mut [f32; FFT_N]) {
    for i in 0..FFT_N {
        let w = 0.5 * (1.0 - cosf(2.0 * PI * i as f32 / (FFT_N - 1) as f32));
        data[i] *= w;
    }
}

// ============================================================================
// Main
// ============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 240MHz for fast LCDC rendering
    let config = sifli_hal::Config::default().with_rcc(
        const {
            ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul10))
                .checked()
        },
    );
    let p = sifli_hal::init(config);

    // ===== Display init =====
    let _pa26_pwr = gpio::Output::new(p.PA26, gpio::Level::High);
    embassy_time::Timer::after_millis(10).await;
    let _pa38_pwr = gpio::Output::new(p.PA38, gpio::Level::High);
    embassy_time::Timer::after_millis(10).await;

    let lcdc_config = sifli_hal::lcdc::Config {
        width: WIDTH as u16,
        height: HEIGHT as u16,
        interface_config: SpiConfig {
            line_mode: lcdc::SpiLineMode::FourLine4Data,
            write_frequency: lcdc::FrequencyConfig::Freq(mhz(120)),
            ..Default::default()
        },
        ..Default::default()
    };

    let lcdc_inst = lcdc::Lcdc::new_qspi(
        p.LCDC1, Irqs, p.PA2, p.PA3, p.PA4, p.PA5, p.PA6, p.PA7, p.PA8,
        lcdc_config,
    );
    let disp_bus = QspiFlashBus::new(lcdc_inst);
    let rst = gpio::Output::new(p.PA0, gpio::Level::Low);
    let mut bl = gpio::Output::new(p.PA1, gpio::Level::Low);
    let panel = Co5300::<MyCo5300, _, _>::new(LCDResetOption::new_pin(rst));

    let mut display = DisplayDriver::builder(disp_bus, panel)
        .with_color_format(ColorFormat::RGB565)
        .init(&mut Delay)
        .await
        .unwrap();
    display.set_brightness(255).await.unwrap();
    bl.set_high();

    // Init framebuffer (PSRAM, clear to black)
    let fb: &mut [u8; FB_SIZE] = unsafe {
        let ptr = core::ptr::addr_of_mut!(FB_BUF) as *mut [u8; FB_SIZE];
        core::ptr::write_bytes(ptr as *mut u8, 0, FB_SIZE);
        &mut *ptr
    };

    // ===== Audio init =====
    let pll = AudioPll::new(AudPllFreq::Mhz49_152);
    let mut adc = AudioAdc::new(
        p.AUDPRC,
        p.DMAC1_CH2,
        &pll,
        Irqs,
        audio::AdcConfig::default(),
    );
    let dma_buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };
    let mut stream = adc.start_stream(dma_buf);

    // ===== FFT workspace =====
    let mut fft_re = [0.0f32; FFT_N];
    let mut fft_im = [0.0f32; FFT_N];
    let mut magnitudes = [0.0f32; FFT_N / 2];
    let mut display_bars = [0.0f32; NUM_BARS];
    let mut peak_bars = [0.0f32; NUM_BARS];
    let mut peak_hold = [0u8; NUM_BARS]; // frames since peak was set

    let bar_bounds = compute_bar_boundaries();

    // Pre-compute bar colors
    let mut colors = [0u16; NUM_BARS];
    for i in 0..NUM_BARS {
        colors[i] = bar_color(i);
    }

    // Bar layout: center 20 bars in 390px
    let total_bar_width = NUM_BARS as i32 * BAR_WIDTH + (NUM_BARS as i32 - 1) * BAR_GAP;
    let bar_x_start = (WIDTH_I - total_bar_width) / 2;

    // Draw static elements once
    let dim_white = rgb565(180, 180, 180);
    let title_color = rgb565(0, 200, 255);
    draw_text(fb, b"Audio Spectrum", 100, 10, title_color, 3);

    // Frequency labels below bars
    let labels: [&[u8]; 5] = [b"100", b"500", b"2k", b"8k", b"20k"];
    let label_positions: [usize; 5] = [1, 5, 10, 15, 19];
    for (li, &pos) in label_positions.iter().enumerate() {
        let bx = bar_x_start + pos as i32 * (BAR_WIDTH + BAR_GAP);
        draw_text(fb, labels[li], bx - 5, SPECTRUM_BOTTOM + 2, dim_white, 1);
    }

    // VU meter label
    draw_text(fb, b"VU", 10, VU_TOP + 8, dim_white, 2);

    let audio_buf = unsafe { &mut *core::ptr::addr_of_mut!(AUDIO_BUF) };

    // ===== Main loop =====
    loop {
        // 1. Read audio samples
        if stream.read(audio_buf).await.is_err() {
            continue;
        }

        // 2. Extract L channel → float, window, FFT
        for i in 0..FFT_N {
            fft_re[i] = (audio_buf[i] as i16) as f32;
            fft_im[i] = 0.0;
        }
        apply_hann_window(&mut fft_re);
        fft256(&mut fft_re, &mut fft_im);

        // 3. Compute magnitudes (dB scale)
        // gc=4, vol=6: noise floor max_db ~38-42
        const DB_FLOOR: f32 = 35.0;
        const DB_RANGE: f32 = 25.0;
        magnitudes[0] = 0.0; // skip DC bin
        for i in 1..FFT_N / 2 {
            let mag = sqrtf(fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i]);
            let db = if mag > 0.5 { 20.0 * log10f(mag) } else { 0.0 };
            magnitudes[i] = ((db - DB_FLOOR) / DB_RANGE).max(0.0).min(1.0);
        }

        // 4. Group into bars (average magnitude per group)
        let mut raw_bars = [0.0f32; NUM_BARS];
        for i in 0..NUM_BARS {
            let lo = bar_bounds[i];
            let hi = bar_bounds[i + 1];
            let count = (hi - lo).max(1);
            let mut sum = 0.0f32;
            for j in lo..hi {
                if j < FFT_N / 2 {
                    sum += magnitudes[j];
                }
            }
            raw_bars[i] = sum / count as f32;
        }

        // 5. Smooth: fast attack, slow decay
        for i in 0..NUM_BARS {
            if raw_bars[i] > display_bars[i] {
                display_bars[i] = raw_bars[i]; // instant attack
            } else {
                display_bars[i] = display_bars[i] * 0.7 + raw_bars[i] * 0.3; // faster decay
            }

            // Peak hold
            if raw_bars[i] >= peak_bars[i] {
                peak_bars[i] = raw_bars[i];
                peak_hold[i] = 0;
            } else {
                peak_hold[i] = peak_hold[i].saturating_add(1);
                if peak_hold[i] > 8 {
                    peak_bars[i] *= 0.85; // faster peak drop
                }
            }
        }

        // 6. Compute VU level (RMS of L channel)
        let mut rms_sum = 0.0f32;
        for i in 0..FFT_N {
            let s = (audio_buf[i] as i16) as f32;
            rms_sum += s * s;
        }
        let rms = sqrtf(rms_sum / FFT_N as f32);
        let vu_db = if rms > 0.5 { 20.0 * log10f(rms) } else { 0.0 };
        let vu_level = ((vu_db - DB_FLOOR) / DB_RANGE).max(0.0).min(1.0);

        // 7. Render spectrum bars (clear + redraw area)
        // Clear spectrum area
        fill_rect(fb, 0, SPECTRUM_TOP, WIDTH_I, SPECTRUM_HEIGHT, 0);

        // Draw grid lines
        let grid_color = rgb565(25, 25, 25);
        for i in 0..=6 {
            let y = SPECTRUM_BOTTOM - (i * SPECTRUM_HEIGHT / 6);
            hline(fb, bar_x_start - 5, bar_x_start + total_bar_width + 5, y, grid_color);
        }

        // Draw bars
        for i in 0..NUM_BARS {
            let bar_h = (display_bars[i] * SPECTRUM_HEIGHT as f32) as i32;
            let bar_h = bar_h.max(1).min(SPECTRUM_HEIGHT);
            let bx = bar_x_start + i as i32 * (BAR_WIDTH + BAR_GAP);
            let by = SPECTRUM_BOTTOM - bar_h;

            // Main bar: gradient from dark at bottom to bright at top
            let color = colors[i];
            // Dimmer version for bottom half
            let dim_color = dim_rgb565(color, 128);
            let mid_y = by + bar_h / 2;
            fill_rect(fb, bx, mid_y, BAR_WIDTH, SPECTRUM_BOTTOM - mid_y, dim_color);
            fill_rect(fb, bx, by, BAR_WIDTH, mid_y - by, color);

            // Bright cap at top
            fill_rect(fb, bx, by, BAR_WIDTH, 2.min(bar_h), brighten_rgb565(color));

            // Peak marker (thin white line)
            let peak_h = (peak_bars[i] * SPECTRUM_HEIGHT as f32) as i32;
            let peak_y = SPECTRUM_BOTTOM - peak_h.max(1).min(SPECTRUM_HEIGHT);
            if peak_h > 2 {
                let peak_color = rgb565(255, 255, 255);
                fill_rect(fb, bx, peak_y, BAR_WIDTH, 2, peak_color);
            }
        }

        // 8. Render VU meter
        let vu_x_start = 50;
        let vu_width = WIDTH_I - 70;
        fill_rect(fb, vu_x_start, VU_TOP, vu_width, VU_HEIGHT, rgb565(15, 15, 15));

        let vu_fill = (vu_level * vu_width as f32) as i32;
        // VU gradient: green → yellow → red
        for x in 0..vu_fill {
            let frac = x as f32 / vu_width as f32;
            let color = if frac < 0.6 {
                rgb565(0, 200, 0)
            } else if frac < 0.8 {
                rgb565(220, 200, 0)
            } else {
                rgb565(255, 50, 0)
            };
            for y in (VU_TOP + 2)..(VU_TOP + VU_HEIGHT - 2) {
                set_pixel(fb, vu_x_start + x, y, color);
            }
        }

        // VU border
        let border_color = rgb565(80, 80, 80);
        hline(fb, vu_x_start, vu_x_start + vu_width, VU_TOP, border_color);
        hline(fb, vu_x_start, vu_x_start + vu_width, VU_TOP + VU_HEIGHT - 1, border_color);

        // 9. Send frame to display
        display.write_frame(fb).await.unwrap();
    }
}

/// Dim an RGB565 color by a factor (0-255 where 255 = full brightness).
fn dim_rgb565(color: u16, factor: u8) -> u16 {
    let r = ((color >> 11) & 0x1F) as u32 * factor as u32 / 255;
    let g = ((color >> 5) & 0x3F) as u32 * factor as u32 / 255;
    let b = (color & 0x1F) as u32 * factor as u32 / 255;
    ((r as u16) << 11) | ((g as u16) << 5) | (b as u16)
}

/// Brighten an RGB565 color (push towards white).
fn brighten_rgb565(color: u16) -> u16 {
    let r = (((color >> 11) & 0x1F) + 8).min(31);
    let g = (((color >> 5) & 0x3F) + 16).min(63);
    let b = ((color & 0x1F) + 8).min(31);
    (r << 11) | (g << 5) | b
}
