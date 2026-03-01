#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{self, AsyncNorFlash, Error, NorConfig};
use sifli_hal::{bind_interrupts, peripherals};

const TEST_OFFSET: u32 = 0x0024_0000;
const ERASE_SIZE: u32 = 4096;
const VERIFY_LEN: usize = 256;

bind_interrupts!(struct Irqs {
    MPI2 => mpi::InterruptHandler<peripherals::MPI2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("MPI NOR async auto-detect test");
    let p = sifli_hal::init(Default::default());

    let mut flash =
        match AsyncNorFlash::new_async_auto(p.MPI2, p.DMAC1_CH1, Irqs, NorConfig::default()) {
            Ok(flash) => flash,
            Err(Error::AsyncForbiddenInXip) => {
                info!("Async auto XIP-guard PASSED: same-instance XIP is active");
                loop {
                    Timer::after_secs(5).await;
                }
            }
            Err(err) => defmt::panic!(
                "Async auto constructor failed: {:?}",
                defmt::Debug2Format(&err)
            ),
        };

    if let Some(detected) = flash.detected_info() {
        let jedec_raw = ((detected.jedec.manufacturer as u32) << 16)
            | ((detected.jedec.memory_type as u32) << 8)
            | detected.jedec.density as u32;
        info!(
            "Detected JEDEC=0x{=u32:06X}, profile={}",
            jedec_raw, detected.profile_name
        );
    } else {
        warn!("No detected_info returned from async auto constructor");
    }

    let mut payload = [0u8; VERIFY_LEN];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = (i as u8) ^ 0x3c;
    }

    flash
        .erase(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE)
        .await
        .unwrap();
    flash.write(TEST_OFFSET, &payload).await.unwrap();

    let mut verify = [0u8; VERIFY_LEN];
    flash.read(TEST_OFFSET, &mut verify).await.unwrap();

    if verify == payload {
        info!(
            "Async auto verification PASSED - all {} bytes match",
            VERIFY_LEN
        );
    } else {
        error!("Async auto verification FAILED");
    }

    loop {
        Timer::after_secs(5).await;
    }
}
