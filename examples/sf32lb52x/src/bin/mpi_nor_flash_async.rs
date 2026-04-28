#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{self, AsyncNorFlash, BuiltInProfile, Error, NorConfig, ProfileSource};
use sifli_hal::{bind_interrupts, peripherals};

const TEST_OFFSET: u32 = 0x0022_0000;
const ERASE_SIZE: u32 = 4096;
const VERIFY_LEN: usize = 256;

bind_interrupts!(struct Irqs {
    MPI2 => mpi::InterruptHandler<peripherals::MPI2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("MPI NOR async flash test");
    let p = sifli_hal::init(Default::default());

    let mut flash = match AsyncNorFlash::new_async(
        p.MPI2,
        p.DMAC1_CH1,
        Irqs,
        ProfileSource::BuiltIn(BuiltInProfile::CommonSpiNor16MiB),
        NorConfig::default(),
    ) {
        Ok(flash) => flash,
        Err(Error::AsyncForbiddenInXip) => {
            warn!("Async MPI2 is forbidden while running from the same XIP flash");
            loop {
                Timer::after_secs(5).await;
            }
        }
        Err(err) => defmt::panic!("Async NOR init failed: {:?}", defmt::Debug2Format(&err)),
    };

    let mut payload = [0u8; VERIFY_LEN];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = (i as u8) ^ 0xa5;
    }

    flash
        .erase(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE)
        .await
        .unwrap();
    flash.write(TEST_OFFSET, &payload).await.unwrap();

    let mut verify = [0u8; VERIFY_LEN];
    flash.read(TEST_OFFSET, &mut verify).await.unwrap();

    if verify == payload {
        info!("Async verification PASSED - all {} bytes match", VERIFY_LEN);
    } else {
        error!("Async verification FAILED");
    }

    loop {
        Timer::after_secs(5).await;
    }
}
