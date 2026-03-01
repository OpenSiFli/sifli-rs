#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::mpi::{BlockingNorFlash, BuiltInProfile, NorConfig, ProfileSource};

const FLASH_CODE_BUS_BASE: usize = 0x1200_0000;
const FLASH_CODE_BUS_END: usize = 0x1300_0000;
// Test offset should be in a safe area that doesn't overlap with running code.
// Code starts at 0x12020000, so 0x00220000 (flash offset 2MB+) should be safe.
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
    info!("MPI NOR Flash XIP-safe test [mm-read-v1]");
    let p = sifli_hal::init(Default::default());

    let code_addr = code_marker as *const () as usize;
    info!("Code marker address: 0x{=u32:08X}", code_addr as u32);
    info!(
        "Running from MPI2 XIP: {}",
        if running_from_mpi2_xip() { "yes" } else { "no" }
    );

    // Explicit XIP-safe constructor: preserve preconfigured timing while running from this flash.
    let mut flash = BlockingNorFlash::new_blocking_without_reset(
        p.MPI2,
        ProfileSource::BuiltIn(BuiltInProfile::CommonSpiNor16MiB),
        NorConfig::default(),
    )
    .unwrap();

    // Debug: Print actual MPI2 base address from PAC
    let mpi2_addr = sifli_hal::pac::MPI2.as_ptr() as usize;
    info!("MPI2 PAC base address: 0x{=u32:08X}", mpi2_addr as u32);

    // Read registers via PAC
    let mpi2 = sifli_hal::pac::MPI2;
    info!("MPI2 CR: 0x{=u32:08X}", mpi2.cr().read().0);
    info!("MPI2 SR: 0x{=u32:08X}", mpi2.sr().read().0);
    info!("MPI2 CCR1: 0x{=u32:08X}", mpi2.ccr1().read().0);
    info!("MPI2 HCCR: 0x{=u32:08X}", mpi2.hrccr().read().0);

    // Read JEDEC ID - uses hardware CMD2 status polling
    info!("Reading JEDEC ID...");
    let jedec_id = flash.read_jedec_id().unwrap();
    let jedec_raw = ((jedec_id.manufacturer as u32) << 16)
        | ((jedec_id.memory_type as u32) << 8)
        | jedec_id.density as u32;
    info!(
        "JEDEC ID: 0x{=u32:06X} (manufacturer: 0x{=u8:02X})",
        jedec_raw, jedec_id.manufacturer
    );

    info!("Checkpoint F: before read_bytes(TEST_OFFSET, 16)");
    let mut head = [0u8; 16];
    flash.read(TEST_OFFSET, &mut head).unwrap();
    info!(
        "Checkpoint G: read_bytes ok, data[0..16]={=[u8]:02X}",
        &head[..]
    );

    // Note: Reading from flash offset 0 would conflict with XIP code execution
    // (code runs from 0x12020000 = flash offset 0x20000), so we skip reading
    // the flash header and go directly to the test area.

    // Test erase/write/read cycle at a safe offset
    warn!(
        "Testing erase/write/read at flash offset 0x{=u32:08X}",
        TEST_OFFSET
    );

    info!("Checkpoint H: before erase_range");
    flash.erase(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE).unwrap();
    info!("Checkpoint I: erase complete");

    // Read after erase to verify it's all 0xFF
    let mut after_erase = [0u8; 16];
    flash.read(TEST_OFFSET, &mut after_erase).unwrap();
    info!("After erase[0..16]: {=[u8]:02X}", &after_erase[..]);

    // Prepare test pattern
    let mut payload = [0u8; VERIFY_LEN];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = (i as u8) ^ 0x5a;
    }
    info!("Test pattern[0..16]: {=[u8]:02X}", &payload[..16]);

    info!("Checkpoint J: before write_bytes, len={}", VERIFY_LEN);
    flash.write(TEST_OFFSET, &payload).unwrap();
    info!("Checkpoint K: write complete");

    // Verify
    let mut verify = [0u8; VERIFY_LEN];
    info!("Checkpoint L: before verify read");
    flash.read(TEST_OFFSET, &mut verify).unwrap();
    info!("Checkpoint M: verify read complete");
    info!("Read back[0..16]:    {=[u8]:02X}", &verify[..16]);

    if verify == payload {
        info!("Verification PASSED - all {} bytes match", VERIFY_LEN);
    } else {
        error!("Verification FAILED!");
        error!("Expected[0..16]: {=[u8]:02X}", &payload[..16]);
        error!("Got[0..16]:      {=[u8]:02X}", &verify[..16]);
    }

    info!("Test complete. Entering idle loop.");
    loop {
        Timer::after_secs(5).await;
    }
}
