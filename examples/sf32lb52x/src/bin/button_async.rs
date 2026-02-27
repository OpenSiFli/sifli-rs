//! an asynchronous button using GPIO interrupt (EXTI).
//!
//! Note: This example is simplified.
//! When using EXTI as a button input in an application, debouncing is mandatory!

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

use sifli_hal::gpio::{Input, Level, Output, Pull};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    // SF32LB52-DevKit-LCD KEY2 pin, with a pull down resister on the board
    let mut button = Input::new(p.PA11, Pull::None);
    // let mut button = Input::new(p.PA34, Pull::None);

    // SF32LB52-DevKit-LCD LED pin
    let mut led = Output::new(p.PA26, Level::High);

    info!("Press the USER button...");

    loop {
        button.wait_for_rising_edge().await;
        info!("Pressed!");
        led.set_low();

        button.wait_for_falling_edge().await;
        info!("Released!");
        led.set_high();
    }
}
