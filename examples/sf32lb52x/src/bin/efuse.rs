#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::efuse::Efuse;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let efuse = match Efuse::new(p.EFUSEC) {
        Ok(v) => v,
        Err(e) => {
            error!("EFUSE init/read failed: {:?}", e);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    let uid = efuse.uid();
    let cal = *efuse.calibration();

    loop {
        info!("UID: {:x}", uid.bytes());
        info!("Bank1 is_io18: {}", cal.primary.low.is_io18());
        info!("Bank1 primary low: {:?}", cal.primary.low);
        info!("Bank1 primary high: {:?}", cal.primary.high);
        info!("Bank1 vol2: {:?}", cal.vol2);
        Timer::after_secs(10).await;
    }
}
