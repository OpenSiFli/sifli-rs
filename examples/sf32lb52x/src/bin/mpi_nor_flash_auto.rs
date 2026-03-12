#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{BlockingNorFlash, NorConfig};

const FLASH_CODE_BUS_BASE: usize = 0x1200_0000;
const FLASH_CODE_BUS_END: usize = 0x1300_0000;
const TEST_OFFSET: u32 = 0x0022_0000;
const ERASE_SIZE: u32 = 4096;
const VERIFY_LEN: usize = 256;

#[inline(never)]
fn code_marker() {}

fn running_from_mpi2_xip() -> bool {
    let code_addr = code_marker as *const () as usize;
    (FLASH_CODE_BUS_BASE..FLASH_CODE_BUS_END).contains(&code_addr)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("MPI NOR Flash auto-detect test");
    let p = sifli_hal::init(Default::default());

    let code_addr = code_marker as *const () as usize;
    info!("Code marker address: 0x{=u32:08X}", code_addr as u32);
    info!(
        "Running from MPI2 XIP: {}",
        if running_from_mpi2_xip() { "yes" } else { "no" }
    );

    let mut flash = BlockingNorFlash::new_blocking_auto(p.MPI2, NorConfig::default()).unwrap();

    let detected = flash.detected_info().unwrap();
    let jedec_raw = ((detected.jedec.manufacturer as u32) << 16)
        | ((detected.jedec.memory_type as u32) << 8)
        | detected.jedec.density as u32;

    info!(
        "Auto-detected JEDEC: 0x{=u32:06X}, profile: {}",
        jedec_raw, detected.profile_name
    );

    info!("Checkpoint A: before erase");
    flash.erase(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE).unwrap();
    info!("Checkpoint B: erase complete");

    let mut payload = [0u8; VERIFY_LEN];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = (i as u8) ^ 0x5a;
    }

    info!("Checkpoint C: before write");
    flash.write(TEST_OFFSET, &payload).unwrap();
    info!("Checkpoint D: write complete");

    let mut verify = [0u8; VERIFY_LEN];
    info!("Checkpoint E: before read");
    flash.read(TEST_OFFSET, &mut verify).unwrap();
    info!("Checkpoint F: read complete");

    if verify == payload {
        info!(
            "Auto-detect verification PASSED - all {} bytes match",
            VERIFY_LEN
        );
    } else {
        error!("Auto-detect verification FAILED");
    }

    loop {
        Timer::after_secs(5).await;
    }
}
