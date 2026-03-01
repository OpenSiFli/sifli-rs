#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{self, AsyncNorFlash, BuiltInProfile, Error, NorConfig, ProfileSource};
use sifli_hal::{bind_interrupts, peripherals};

const FLASH_CODE_BUS_BASE: usize = 0x1200_0000;
const FLASH_CODE_BUS_END: usize = 0x1300_0000;

#[inline(never)]
fn code_marker() {}

fn running_from_mpi2_xip() -> bool {
    let code_addr = code_marker as *const () as usize;
    (FLASH_CODE_BUS_BASE..FLASH_CODE_BUS_END).contains(&code_addr)
}

bind_interrupts!(struct Irqs {
    MPI2 => mpi::InterruptHandler<peripherals::MPI2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("MPI NOR async XIP guard test");
    let p = sifli_hal::init(Default::default());

    let from_xip = running_from_mpi2_xip();
    info!(
        "Running from MPI2 XIP: {}",
        if from_xip { "yes" } else { "no" }
    );

    let result = AsyncNorFlash::new_async(
        p.MPI2,
        p.DMAC1_CH1,
        Irqs,
        ProfileSource::BuiltIn(BuiltInProfile::CommonSpiNor16MiB),
        NorConfig::default(),
    );

    match (from_xip, result) {
        (true, Err(Error::AsyncForbiddenInXip)) => {
            info!("PASS: constructor rejected async in same-instance XIP");
        }
        (false, Ok(_flash)) => {
            info!("PASS: constructor allowed async when not in same-instance XIP");
        }
        (_, Err(err)) => {
            error!(
                "Unexpected constructor error: {:?}",
                defmt::Debug2Format(&err)
            );
        }
        (_, Ok(_flash)) => {
            error!("Unexpected constructor success for current execution mode");
        }
    }

    loop {
        Timer::after_secs(5).await;
    }
}
