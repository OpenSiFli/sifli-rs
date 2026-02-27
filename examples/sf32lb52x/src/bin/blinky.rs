#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
// use panic_halt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;

use sifli_hal::gpio;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    // SF32LB52-DevKit-LCD LED pin
    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);
    info!("Hello World!");
    sifli_hal::rcc::test_print_clocks();

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_secs(1).await;

        info!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}
