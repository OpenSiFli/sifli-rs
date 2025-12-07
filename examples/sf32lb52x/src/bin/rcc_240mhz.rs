#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal;
use sifli_hal::gpio;
use sifli_hal::rcc::{self, Dll, DllStage, Sysclk};

// **WARN**:
// The RCC clock configuration module is still under construction,
// and there is no guarantee that other clock configurations will
// run correctly.
// https://github.com/OpenSiFli/sifli-rs/issues/7

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");
    let mut config = sifli_hal::Config::default();

    // Configure 240MHz system clock using DLL1
    // DLL1 Freq = (stg + 1) * 24MHz = (15 + 1) * 24MHz = 384MHz
    // DLL2 Freq = (stg + 1) * 24MHz = (9 + 1) * 24MHz = 240MHz
    // Hclk Freq = Dll1 / hdiv = 192MHz
    // Usbclk Freq = Dll2 / 4(usb_div_internal) = 60MHz(IMMUTABLE)
    config.rcc.sys = Sysclk::Dll1;
    config.rcc.dll1 = Some(Dll {
        out_div2: false,
        stg: DllStage::Mul16, // Mul16 = enum value 15
    });
    config.rcc.dll2 = Some(Dll {
        out_div2: false,
        stg: DllStage::Mul10, // Mul10 = enum value 9
    });
    config.rcc.hdiv = rcc::HclkPrescaler(2);
    config.rcc.mux.usbsel = rcc::Usbsel::Dll2;

    let p = sifli_hal::init(config);

    info!("Clock configuration complete");
    rcc::test_print_clocks();

    // SF32LB52-DevKit-LCD LED pin
    let mut led = gpio::Output::new(p.PA1, gpio::Level::Low);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_secs(1).await;

        info!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}
