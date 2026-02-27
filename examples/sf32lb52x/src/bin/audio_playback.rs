//! Audio Playback — Loop an embedded PCM clip with 1-second gaps
//!
//! Plays a pre-converted 48kHz 16-bit stereo PCM file through the
//! AUDCODEC DAC → Class-D PA, repeating every 1 second.
//!
//! The PCM data lives in Flash (include_bytes!) and is streamed to
//! an SRAM ring buffer for DMA playback.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;

use sifli_hal::aud_pll::{AudioPll, AudPllFreq};
use sifli_hal::audio::{self, AudioDac};
use sifli_hal::bind_interrupts;

bind_interrupts!(struct Irqs {
    AUDPRC => audio::InterruptHandler;
});

// PCM data: 48kHz, 16-bit signed LE, stereo (interleaved L16 R16)
// Each 4 bytes = one u32 FIFO entry = [R:hi16 | L:lo16]
static PCM_DATA: &[u8] = include_bytes!("../../assets/yanpai.raw");

// DMA ring buffer in SRAM (DMAC1 cannot access PSRAM)
// 9600 u32 = 100ms @ 48kHz stereo
static mut DMA_BUF: [u32; 9600] = [0u32; 9600];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(sifli_hal::Config::default());

    let pll = AudioPll::new(AudPllFreq::Mhz49_152);
    let mut dac = AudioDac::new(
        p.AUDPRC,
        p.DMAC1_CH1,
        &pll,
        Irqs,
        audio::DacConfig::default(),
    );

    // Interpret PCM bytes as u32 slice (already aligned: L16+R16 = u32)
    let pcm_samples: &[u32] = unsafe {
        core::slice::from_raw_parts(
            PCM_DATA.as_ptr() as *const u32,
            PCM_DATA.len() / 4,
        )
    };

    info!("Audio playback: {} samples ({} ms), looping with 1s gap",
        pcm_samples.len(),
        pcm_samples.len() as u64 * 1000 / 48000
    );

    let dma_buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };

    loop {
        // Start streaming playback
        let mut stream = dac.start_stream(dma_buf);

        // Feed PCM data in chunks
        const CHUNK: usize = 2400; // 50ms chunks
        for chunk in pcm_samples.chunks(CHUNK) {
            if stream.write(chunk).await.is_err() {
                break;
            }
        }

        // Drain ring buffer by writing silence until all real data is played
        let silence = [0u32; CHUNK];
        // Write enough silence to flush the ring buffer (2x capacity)
        for _ in 0..8 {
            let _ = stream.write(&silence).await;
        }

        stream.stop().await;

        // 1-second gap
        embassy_time::Timer::after_millis(1000).await;
    }
}
