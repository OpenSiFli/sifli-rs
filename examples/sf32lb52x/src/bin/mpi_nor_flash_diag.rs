#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{BlockingNorFlash, BuiltInProfile, Error, NorConfig, ProfileSource};

const FLASH_CODE_BUS_BASE: usize = 0x1200_0000;
const FLASH_CODE_BUS_END: usize = 0x1300_0000;

#[inline(never)]
fn code_marker() {}

fn running_from_mpi2_xip() -> bool {
    let code_addr = code_marker as *const () as usize;
    (FLASH_CODE_BUS_BASE..FLASH_CODE_BUS_END).contains(&code_addr)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("MPI NOR blocking diagnostics test");
    let p = sifli_hal::init(Default::default());
    let from_xip = running_from_mpi2_xip();
    info!(
        "Running from MPI2 XIP: {}",
        if from_xip { "yes" } else { "no" }
    );

    let mut flash = BlockingNorFlash::new_blocking_without_reset(
        p.MPI2,
        ProfileSource::BuiltIn(BuiltInProfile::CommonSpiNor16MiB),
        NorConfig::default(),
    )
    .unwrap();

    let jedec = flash.read_jedec_id().unwrap();
    let jedec_raw = ((jedec.manufacturer as u32) << 16)
        | ((jedec.memory_type as u32) << 8)
        | jedec.density as u32;
    info!("JEDEC: 0x{=u32:06X}", jedec_raw);
    info!("Capacity bytes: {=u32}", flash.capacity() as u32);

    let status = flash.read_status().unwrap();
    info!("Status register: 0x{=u8:02X}", status);

    let mut all_checks_ok = true;

    let mut sfdp = [0u8; 16];
    match flash.read_sfdp(0, &mut sfdp) {
        Ok(()) => info!("SFDP[0..16]: {=[u8]:02X}", sfdp),
        Err(e) => {
            error!("read_sfdp unexpected error: {:?}", defmt::Debug2Format(&e));
            all_checks_ok = false;
        }
    }

    let mut uid = [0u8; 16];
    match flash.read_unique_id(&mut uid) {
        Ok(()) => info!("UID[0..16]: {=[u8]:02X}", uid),
        Err(e) => {
            error!(
                "read_unique_id unexpected error: {:?}",
                defmt::Debug2Format(&e)
            );
            all_checks_ok = false;
        }
    }

    match flash.enter_deep_power_down() {
        Ok(()) => {
            info!("Entered deep power down");
            Timer::after_millis(1).await;

            match flash.release_deep_power_down() {
                Ok(()) => {
                    info!("Released deep power down");
                    Timer::after_millis(1).await;
                }
                Err(e) => {
                    error!(
                        "release_deep_power_down unexpected error: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    all_checks_ok = false;
                }
            }
        }
        Err(Error::CommandForbiddenInXip) if from_xip => {
            info!("Deep power down blocked in same-instance XIP as expected");
            match flash.release_deep_power_down() {
                Err(Error::CommandForbiddenInXip) => {
                    info!("Release deep power down also blocked as expected");
                }
                Err(e) => {
                    error!(
                        "release_deep_power_down unexpected error in XIP mode: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    all_checks_ok = false;
                }
                Ok(()) => {
                    warn!("release_deep_power_down unexpectedly succeeded in XIP mode");
                }
            }
        }
        Err(e) => {
            error!(
                "enter_deep_power_down unexpected error: {:?}",
                defmt::Debug2Format(&e)
            );
            all_checks_ok = false;
        }
    }

    if all_checks_ok {
        info!("MPI NOR diagnostics PASSED");
    } else {
        error!("MPI NOR diagnostics FAILED");
    }

    loop {
        Timer::after_secs(5).await;
    }
}
