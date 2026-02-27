#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

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

    // Configure 240MHz system clock using DLL1
    // DLL1 Freq = (stg + 1) * 24MHz = (15 + 1) * 24MHz = 384MHz
    // DLL2 Freq = (stg + 1) * 24MHz = (9 + 1) * 24MHz = 240MHz
    // Hclk Freq = Dll1 / hdiv = 192MHz
    // Usbclk Freq = Dll2 / 4(usb_div_internal) = 60MHz(IMMUTABLE)
    let config = sifli_hal::Config::default().with_rcc(
        const {
            rcc::ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul16))
                .with_dll2(Dll::new().with_stg(DllStage::Mul10))
                .with_hdiv(rcc::HclkPrescaler::new(2))
                .with_mux(rcc::ClockMux::new().with_usbsel(rcc::Usbsel::Dll2))
                .checked()
        },
    );

    let p = sifli_hal::init(config);

    info!("Clock configuration complete");
    rcc::test_print_clocks();

    // LED pin: PA26 for SF32LB52-DevKit-LCD, PA31/PA32 for SF32LB52-Nano
    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_secs(1).await;

        info!("led off!");
        led.set_low();
        Timer::after_secs(1).await;
    }
}
