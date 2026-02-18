//! Audio DAC — async streaming playback (Beethoven's "Ode to Joy").
//!
//! Demonstrates `AudioDac::new()` (async) + `start_stream()` for continuous
//! ring-buffer playback. Melody is synthesized into PSRAM via wavetable,
//! then streamed in chunks through the DMA ring buffer.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embedded_io::Write as _;

use sifli_hal::audio::{self, AudioDac, DacConfig};
use sifli_hal::usart::{Config as UartConfig, Uart};
use sifli_hal::bind_interrupts;

use libm::sinf;

const PI: f32 = core::f32::consts::PI;
const SAMPLE_RATE: u32 = 48000;
const SAMPLES_PER_BEAT: usize = 24000; // 120 BPM = 0.5s per beat

// Note frequencies (Hz)
const C4: f32 = 261.63;
const D4: f32 = 293.66;
const E4: f32 = 329.63;
const F4: f32 = 349.23;
const G4: f32 = 392.00;
const G3: f32 = 196.00;

// Durations (in beats)
const Q: f32 = 1.0;
const H: f32 = 2.0;
const DQ: f32 = 1.5;
const EN: f32 = 0.5;

// "Ode to Joy" — AABA form (64 beats)
const MELODY: [(f32, f32); 62] = [
    // A1
    (E4, Q), (E4, Q), (F4, Q), (G4, Q),
    (G4, Q), (F4, Q), (E4, Q), (D4, Q),
    (C4, Q), (C4, Q), (D4, Q), (E4, Q),
    (E4, DQ), (D4, EN), (D4, H),
    // A2
    (E4, Q), (E4, Q), (F4, Q), (G4, Q),
    (G4, Q), (F4, Q), (E4, Q), (D4, Q),
    (C4, Q), (C4, Q), (D4, Q), (E4, Q),
    (D4, DQ), (C4, EN), (C4, H),
    // B (bridge)
    (D4, Q), (D4, Q), (E4, Q), (C4, Q),
    (D4, Q), (E4, EN), (F4, EN), (E4, Q), (C4, Q),
    (D4, Q), (E4, EN), (F4, EN), (E4, Q), (D4, Q),
    (C4, Q), (D4, Q), (G3, H),
    // A2 (return)
    (E4, Q), (E4, Q), (F4, Q), (G4, Q),
    (G4, Q), (F4, Q), (E4, Q), (D4, Q),
    (C4, Q), (C4, Q), (D4, Q), (E4, Q),
    (D4, DQ), (C4, EN), (C4, H),
];

const TOTAL_BEATS: usize = 64;
const TOTAL_SAMPLES: usize = TOTAL_BEATS * SAMPLES_PER_BEAT; // 1,536,000

// Melody data in PSRAM (CPU-accessible, but DMAC1 cannot read it directly)
#[link_section = ".psram_bss"]
static mut MELODY_DATA: [u32; TOTAL_SAMPLES] = [0u32; TOTAL_SAMPLES];

// DMA ring buffer in SRAM — WritableRingBuffer needs mutable access
const STREAM_BUF_SIZE: usize = 9600; // ~200ms at 48kHz stereo
static mut STREAM_BUF: [u32; STREAM_BUF_SIZE] = [0u32; STREAM_BUF_SIZE];

// Wavetable: 1024-point sine
const WAVE_TABLE_BITS: u32 = 10;
const WAVE_TABLE_SIZE: usize = 1 << WAVE_TABLE_BITS;
static mut WAVE_TABLE: [i16; WAVE_TABLE_SIZE] = [0; WAVE_TABLE_SIZE];

fn init_wavetable() {
    let table = unsafe { &mut *core::ptr::addr_of_mut!(WAVE_TABLE) };
    for i in 0..WAVE_TABLE_SIZE {
        table[i] = (sinf(2.0 * PI * i as f32 / WAVE_TABLE_SIZE as f32) * 32767.0) as i16;
    }
}

fn phase_inc(freq: f32) -> u32 {
    (freq * (4294967296.0 / SAMPLE_RATE as f32)) as u32
}

#[inline(always)]
fn wave_at(phase: u32) -> i32 {
    let idx = (phase >> (32 - WAVE_TABLE_BITS)) as usize;
    unsafe { (*core::ptr::addr_of!(WAVE_TABLE))[idx] as i32 }
}

fn fill_note(buf: &mut [u32], pos: usize, freq: f32, dur: usize) {
    let inc = phase_inc(freq);
    let mut phase: u32 = 0;
    let attack = 96usize.min(dur / 4);
    let release = 480usize.min(dur / 4);
    let gap = 240usize.min(dur / 8);

    for i in 0..dur {
        if pos + i >= buf.len() { break; }

        let w1 = wave_at(phase);
        let w2 = wave_at(phase.wrapping_mul(2));
        let w3 = wave_at(phase.wrapping_mul(3));
        let wave = (w1 + w2 * 3 / 10 + w3 / 10) / 2;

        let env = if i < attack {
            (i * 256 / attack) as i32
        } else if i >= dur - gap {
            0
        } else if i >= dur - gap - release {
            ((dur - gap - i) * 256 / release) as i32
        } else {
            256
        };

        let sample = (wave * env / 256) as i16;
        let s = sample as u16 as u32;
        buf[pos + i] = s | (s << 16); // packed stereo

        phase = phase.wrapping_add(inc);
    }
}

bind_interrupts!(struct Irqs {
    AUDPRC => audio::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();
    let _ = writeln!(usart, "\r\n=== Ode to Joy ===");

    // Build wavetable
    init_wavetable();

    // Generate melody into PSRAM
    let _ = writeln!(usart, "Generating melody...");
    {
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(MELODY_DATA) };
        let mut pos = 0usize;
        for &(freq, dur_beats) in MELODY.iter() {
            let dur_samples = (dur_beats * SAMPLES_PER_BEAT as f32) as usize;
            fill_note(buf, pos, freq, dur_samples);
            pos += dur_samples;
        }
    }
    let _ = writeln!(usart, "Done ({} samples in PSRAM)", TOTAL_SAMPLES);

    // PA26: speaker enable
    {
        let pa = sifli_hal::gpio::Output::new(&mut p.PA26, sifli_hal::gpio::Level::High);
        core::mem::forget(pa);
    }

    // Create async AudioDac
    let mut dac = AudioDac::new(p.AUDPRC, p.DMAC1_CH1, Irqs, DacConfig::default());
    let _ = writeln!(usart, "AudioDac initialized");

    // Start streaming with ring buffer
    let stream_buf = unsafe { &mut *core::ptr::addr_of_mut!(STREAM_BUF) };
    let mut stream = dac.start_stream(stream_buf);
    let _ = writeln!(usart, "Streaming started (ring buffer {})", STREAM_BUF_SIZE);

    // Feed melody data to the stream in chunks
    // WritableRingBuffer handles D-Cache and DMA double-buffering internally
    let melody = unsafe { &*core::ptr::addr_of!(MELODY_DATA) };
    let chunk_size = 4800; // 100ms chunks
    let mut pos = 0usize;

    let _ = writeln!(usart, "Playing! (loop forever)");

    loop {
        let end = (pos + chunk_size).min(TOTAL_SAMPLES);
        let chunk = &melody[pos..end];

        // write() awaits until the ring buffer has space
        if stream.write(chunk).await.is_err() {
            let _ = writeln!(usart, "Stream error!");
            break;
        }

        pos = end;
        if pos >= TOTAL_SAMPLES {
            pos = 0; // loop the melody
        }
    }
}
