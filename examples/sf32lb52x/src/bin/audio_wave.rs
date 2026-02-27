//! Audio Waveform + FFT Line Chart
//!
//! Top half:  real-time audio waveform (time domain)
//! Bottom half: FFT magnitude line chart (frequency domain)
//!
//! Path: ADCIN1 mic → AUDCODEC ADC → AUDPRC RX → DMA → display

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_io::Write as _;

use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
use sifli_hal::audio::{self, AudioAdc};
use sifli_hal::bind_interrupts;
use sifli_hal::gpio;
use sifli_hal::lcdc::{self, SpiConfig};
use sifli_hal::rcc::{ConfigBuilder, Dll, DllStage, Sysclk};
use sifli_hal::time::mhz;
use sifli_hal::usart::{Config as UartConfig, Uart};

use display_driver::bus::QspiFlashBus;
use display_driver::panel::reset::LCDResetOption;
use display_driver::{ColorFormat, DisplayDriver};
use display_driver_co5300::{
    spec::{Co5300Spec, PanelSpec},
    Co5300,
};

use libm::{cosf, sinf, sqrtf, log10f};

// ============================================================================
// Display
// ============================================================================

const WIDTH: usize = 390;
const HEIGHT: usize = 450;
const W: i32 = WIDTH as i32;
const H: i32 = HEIGHT as i32;
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
// Audio
// ============================================================================

static mut DMA_BUF: [u32; 4800] = [0u32; 4800];
static mut AUDIO_BUF: [u32; 390] = [0u32; 390]; // exactly WIDTH samples

// ============================================================================
// FFT (256-point)
// ============================================================================

const FFT_N: usize = 256;
const FFT_LOG2: usize = 8;
const PI: f32 = core::f32::consts::PI;

fn bit_reverse(mut x: usize, bits: usize) -> usize {
    let mut r = 0;
    for _ in 0..bits {
        r = (r << 1) | (x & 1);
        x >>= 1;
    }
    r
}

fn fft256(re: &mut [f32; FFT_N], im: &mut [f32; FFT_N]) {
    for i in 0..FFT_N {
        let j = bit_reverse(i, FFT_LOG2);
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut half = 1usize;
    for _ in 0..FFT_LOG2 {
        let size = half << 1;
        let step = -PI / half as f32;
        let mut k = 0;
        while k < FFT_N {
            for j in 0..half {
                let a = step * j as f32;
                let (wr, wi) = (cosf(a), sinf(a));
                let e = k + j;
                let o = e + half;
                let tr = wr * re[o] - wi * im[o];
                let ti = wr * im[o] + wi * re[o];
                re[o] = re[e] - tr;
                im[o] = im[e] - ti;
                re[e] += tr;
                im[e] += ti;
            }
            k += size;
        }
        half = size;
    }
}

// ============================================================================
// Drawing helpers
// ============================================================================

#[inline(always)]
fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3)
}

#[inline(always)]
fn put(fb: &mut [u8], x: i32, y: i32, c: u16) {
    if x >= 0 && x < W && y >= 0 && y < H {
        let off = (y as usize * WIDTH + x as usize) * 2;
        let b = c.to_le_bytes();
        fb[off] = b[0];
        fb[off + 1] = b[1];
    }
}

fn fill(fb: &mut [u8], x0: i32, y0: i32, w: i32, h: i32, c: u16) {
    let xs = x0.max(0);
    let xe = (x0 + w).min(W);
    let ys = y0.max(0);
    let ye = (y0 + h).min(H);
    let b = c.to_le_bytes();
    for y in ys..ye {
        let base = y as usize * WIDTH * 2;
        for x in xs..xe {
            let off = base + x as usize * 2;
            fb[off] = b[0];
            fb[off + 1] = b[1];
        }
    }
}

/// Draw a line between two points (Bresenham).
fn line(fb: &mut [u8], x0: i32, y0: i32, x1: i32, y1: i32, c: u16) {
    let mut dx = (x1 - x0).abs();
    let mut dy = -(y1 - y0).abs();
    let sx: i32 = if x0 < x1 { 1 } else { -1 };
    let sy: i32 = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    let mut x = x0;
    let mut y = y0;
    loop {
        put(fb, x, y, c);
        if x == x1 && y == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

/// Draw a thick line (draw line + neighbors for width).
fn thick_line(fb: &mut [u8], x0: i32, y0: i32, x1: i32, y1: i32, c: u16) {
    line(fb, x0, y0, x1, y1, c);
    line(fb, x0, y0 - 1, x1, y1 - 1, c);
}

// ============================================================================
// Simple 5x7 font
// ============================================================================

fn glyph(ch: u8) -> [u8; 5] {
    match ch {
        b'A' => [0x7E, 0x09, 0x09, 0x09, 0x7E],
        b'D' => [0x7F, 0x41, 0x41, 0x41, 0x3E],
        b'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
        b'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
        b'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
        b'W' => [0x3F, 0x40, 0x30, 0x40, 0x3F],
        b'a' => [0x20, 0x54, 0x54, 0x54, 0x78],
        b'e' => [0x38, 0x54, 0x54, 0x54, 0x18],
        b'f' => [0x08, 0x7E, 0x09, 0x01, 0x02],
        b'i' => [0x00, 0x44, 0x7D, 0x40, 0x00],
        b'k' => [0x7F, 0x10, 0x28, 0x44, 0x00],
        b'l' => [0x00, 0x41, 0x7F, 0x40, 0x00],
        b'm' => [0x7C, 0x04, 0x18, 0x04, 0x78],
        b'o' => [0x38, 0x44, 0x44, 0x44, 0x38],
        b'p' => [0x7C, 0x14, 0x14, 0x14, 0x08],
        b'r' => [0x7C, 0x08, 0x04, 0x04, 0x08],
        b'v' => [0x1C, 0x20, 0x40, 0x20, 0x1C],
        b'w' => [0x3C, 0x40, 0x30, 0x40, 0x3C],
        b' ' => [0x00; 5],
        b'0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
        b'2' => [0x72, 0x49, 0x49, 0x49, 0x46],
        b'4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
        b'8' => [0x36, 0x49, 0x49, 0x49, 0x36],
        _ => [0x00; 5],
    }
}

fn text(fb: &mut [u8], s: &[u8], x: i32, y: i32, c: u16, sc: i32) {
    let mut cx = x;
    for &ch in s {
        let g = glyph(ch);
        for col in 0..5i32 {
            let bits = g[col as usize];
            for row in 0..7i32 {
                if bits & (1 << row) != 0 {
                    for sy in 0..sc {
                        for sx in 0..sc {
                            put(fb, cx + col * sc + sx, y + row * sc + sy, c);
                        }
                    }
                }
            }
        }
        cx += 6 * sc;
    }
}

// ============================================================================
// Layout constants
// ============================================================================

const WAVE_TOP: i32 = 30;
const WAVE_H: i32 = 180;
const WAVE_MID: i32 = WAVE_TOP + WAVE_H / 2;
const WAVE_BOT: i32 = WAVE_TOP + WAVE_H;

const FFT_TOP: i32 = WAVE_BOT + 30;
const FFT_H: i32 = 180;
const FFT_BOT: i32 = FFT_TOP + FFT_H;

const MARGIN: i32 = 5;
const PLOT_W: i32 = W - 2 * MARGIN; // 380 pixels

// ============================================================================
// Main
// ============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = sifli_hal::Config::default().with_rcc(
        const {
            ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul10))
                .checked()
        },
    );
    let p = sifli_hal::init(config);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();
    let _ = writeln!(usart, "\r\n=== Audio Wave ===");

    // ===== Display init =====
    let _pa26 = gpio::Output::new(p.PA26, gpio::Level::High);
    embassy_time::Timer::after_millis(10).await;
    let _pa38 = gpio::Output::new(p.PA38, gpio::Level::High);
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
    let lcdc = lcdc::Lcdc::new_qspi(
        p.LCDC1, Irqs, p.PA2, p.PA3, p.PA4, p.PA5, p.PA6, p.PA7, p.PA8,
        lcdc_config,
    );
    let bus = QspiFlashBus::new(lcdc);
    let rst = gpio::Output::new(p.PA0, gpio::Level::Low);
    let mut bl = gpio::Output::new(p.PA1, gpio::Level::Low);
    let panel = Co5300::<MyCo5300, _, _>::new(LCDResetOption::new_pin(rst));

    let mut display = DisplayDriver::builder(bus, panel)
        .with_color_format(ColorFormat::RGB565)
        .init(&mut Delay)
        .await
        .unwrap();
    display.set_brightness(255).await.unwrap();
    bl.set_high();

    let fb: &mut [u8; FB_SIZE] = unsafe {
        let ptr = core::ptr::addr_of_mut!(FB_BUF) as *mut [u8; FB_SIZE];
        core::ptr::write_bytes(ptr as *mut u8, 0, FB_SIZE);
        &mut *ptr
    };

    // ===== Audio init =====
    let pll = AudioPll::new(AudPllFreq::Mhz49_152);
    let mut adc = AudioAdc::new(p.AUDPRC, p.DMAC1_CH2, &pll, Irqs, audio::AdcConfig::default());
    let dma_buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };
    let mut stream = adc.start_stream(dma_buf);
    let _ = writeln!(usart, "Ready");

    let audio_buf = unsafe { &mut *core::ptr::addr_of_mut!(AUDIO_BUF) };

    let mut fft_re = [0.0f32; FFT_N];
    let mut fft_im = [0.0f32; FFT_N];

    // Colors
    let bg = rgb565(0, 0, 0);
    let wave_color = rgb565(0, 255, 100);
    let fft_color = rgb565(0, 180, 255);
    let grid_color = rgb565(30, 30, 30);
    let label_color = rgb565(140, 140, 140);
    let title_color = rgb565(255, 200, 50);

    // Draw static labels once
    text(fb, b"Waveform", MARGIN, WAVE_TOP - 20, title_color, 2);
    text(fb, b"FFT", MARGIN, FFT_TOP - 20, title_color, 2);

    let mut frame = 0u32;

    loop {
        if stream.read(audio_buf).await.is_err() {
            continue;
        }

        // Periodic debug
        if frame % 200 == 0 {
            let mut min_l: i16 = i16::MAX;
            let mut max_l: i16 = i16::MIN;
            for &w in audio_buf.iter() {
                let l = w as i16;
                if l < min_l { min_l = l; }
                if l > max_l { max_l = l; }
            }
            let pp = max_l as i32 - min_l as i32;
            let _ = writeln!(usart, "[{:4}] pp={}", frame, pp);
        }
        frame += 1;

        // ========== Waveform plot ==========
        // Clear waveform area
        fill(fb, 0, WAVE_TOP, W, WAVE_H, bg);

        // Grid: center line + quarter lines
        for y in [WAVE_TOP, WAVE_MID, WAVE_BOT - 1] {
            for x in (MARGIN..MARGIN + PLOT_W).step_by(4) {
                put(fb, x, y, grid_color);
            }
        }

        // Find min/max for auto-scaling
        let mut s_min: i16 = i16::MAX;
        let mut s_max: i16 = i16::MIN;
        for i in 0..PLOT_W as usize {
            let s = audio_buf[i] as i16;
            if s < s_min { s_min = s; }
            if s > s_max { s_max = s; }
        }
        let range = (s_max as i32 - s_min as i32).max(1);
        // Scale with some headroom
        let scale_range = (range as f32 * 1.1).max(10.0);
        let center = (s_max as f32 + s_min as f32) / 2.0;

        // Draw waveform as connected line segments
        let sample_to_y = |s: i16| -> i32 {
            let normalized = (s as f32 - center) / scale_range; // -0.5..0.5
            WAVE_MID - (normalized * WAVE_H as f32) as i32
        };

        let mut prev_y = sample_to_y(audio_buf[0] as i16);
        for i in 1..PLOT_W as usize {
            let cur_y = sample_to_y(audio_buf[i] as i16);
            let x0 = MARGIN + (i - 1) as i32;
            let x1 = MARGIN + i as i32;
            thick_line(fb, x0, prev_y, x1, cur_y, wave_color);
            prev_y = cur_y;
        }

        // ========== FFT plot ==========
        fill(fb, 0, FFT_TOP, W, FFT_H, bg);

        // Grid lines
        for x in (MARGIN..MARGIN + PLOT_W).step_by(4) {
            put(fb, x, FFT_BOT - 1, grid_color);
            put(fb, x, FFT_TOP, grid_color);
        }
        // Horizontal grid at 25%, 50%, 75%
        for frac in [0.25f32, 0.5, 0.75] {
            let gy = FFT_BOT - (frac * FFT_H as f32) as i32;
            for x in (MARGIN..MARGIN + PLOT_W).step_by(6) {
                put(fb, x, gy, grid_color);
            }
        }

        // Compute FFT on first 256 samples
        for i in 0..FFT_N {
            fft_re[i] = (audio_buf[i] as i16) as f32;
            fft_im[i] = 0.0;
        }
        // Hann window
        for i in 0..FFT_N {
            let w = 0.5 * (1.0 - cosf(2.0 * PI * i as f32 / (FFT_N - 1) as f32));
            fft_re[i] *= w;
        }
        fft256(&mut fft_re, &mut fft_im);

        // Magnitude in dB, map to FFT_H
        // Only show 0-8kHz (bins 1..43): voice range
        // Each bin = 48000/256 = 187.5 Hz, bin 43 ≈ 8kHz
        const DB_FLOOR: f32 = 20.0;
        const DB_CEIL: f32 = 70.0;
        const MAX_BIN: usize = 43; // 8kHz

        let bin_to_x = |bin: usize| -> i32 {
            MARGIN + ((bin - 1) as i32 * PLOT_W / (MAX_BIN - 1) as i32)
        };
        let db_to_y = |db: f32| -> i32 {
            let norm = ((db - DB_FLOOR) / (DB_CEIL - DB_FLOOR)).max(0.0).min(1.0);
            FFT_BOT - (norm * FFT_H as f32) as i32
        };

        // Draw FFT as filled area + line
        let mut prev_x = bin_to_x(1);
        let mag0 = sqrtf(fft_re[1] * fft_re[1] + fft_im[1] * fft_im[1]);
        let db0 = if mag0 > 0.5 { 20.0 * log10f(mag0) } else { 0.0 };
        let mut prev_fy = db_to_y(db0);

        let fill_color = rgb565(0, 50, 80);
        for bin in 2..MAX_BIN {
            let mag = sqrtf(fft_re[bin] * fft_re[bin] + fft_im[bin] * fft_im[bin]);
            let db = if mag > 0.5 { 20.0 * log10f(mag) } else { 0.0 };
            let cur_x = bin_to_x(bin);
            let cur_fy = db_to_y(db);

            // Fill below the line
            for x in prev_x..cur_x {
                let t = if cur_x > prev_x {
                    (x - prev_x) as f32 / (cur_x - prev_x) as f32
                } else {
                    0.0
                };
                let y_interp = prev_fy as f32 + t * (cur_fy - prev_fy) as f32;
                for y in y_interp as i32..FFT_BOT {
                    put(fb, x, y, fill_color);
                }
            }

            thick_line(fb, prev_x, prev_fy, cur_x, cur_fy, fft_color);
            prev_x = cur_x;
            prev_fy = cur_fy;
        }

        // Frequency labels: 0, 2k, 4k, 6k, 8k
        // bin = freq / 187.5 → 2kHz=bin10.7, 4kHz=bin21.3, 6kHz=bin32, 8kHz=bin42.7
        // Each bin = 187.5Hz: bin11=2k, bin21=4k, bin32=6k, bin42=8k
        let freq_labels: [(usize, &[u8]); 4] = [
            (1, b"0"), (11, b"2k"), (22, b"4k"), (42, b"8k"),
        ];
        for (bin, lt) in freq_labels {
            text(fb, lt, bin_to_x(bin), FFT_BOT + 3, label_color, 1);
        }

        // pp display at bottom
        if frame % 5 == 0 {
            let mut min_l: i16 = i16::MAX;
            let mut max_l: i16 = i16::MIN;
            for i in 0..PLOT_W as usize {
                let s = audio_buf[i] as i16;
                if s < min_l { min_l = s; }
                if s > max_l { max_l = s; }
            }
            let pp = max_l as i32 - min_l as i32;
            // Clear and redraw pp area
            fill(fb, 0, H - 20, W, 20, bg);
            // Simple numeric display via text
            let mut pp_str = [b' '; 12];
            pp_str[0] = b'p';
            pp_str[1] = b'p';
            pp_str[2] = b'=';
            // Convert pp to string (simple decimal)
            let mut val = pp as u32;
            let mut pos = 8;
            if val == 0 {
                pp_str[pos] = b'0';
            } else {
                while val > 0 && pos > 3 {
                    pp_str[pos] = b'0' + (val % 10) as u8;
                    val /= 10;
                    pos -= 1;
                }
            }
            text(fb, &pp_str, MARGIN, H - 18, label_color, 2);
        }

        display.write_frame(fb).await.unwrap();
    }
}
