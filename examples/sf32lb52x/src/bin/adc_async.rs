#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use sifli_hal::adc::{Adc, Channel};
use sifli_hal::bind_interrupts;

bind_interrupts!(
    struct Irqs {
        GPADC => sifli_hal::adc::InterruptHandler;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    info!("Hello World!");

    // f_ADCCLK = f_PCLK / (DATA_SAMP_DLY + CONV_WIDTH + SAMP_WIDTH + 2)
    let mut adc = Adc::new(p.GPADC, Irqs, Default::default());
    let pin = p.PA28;
    let mut ch0 = Channel::new_pin(pin);
    let mut ch7 = Channel::new_vbat(p.ADC_VBAT);

    loop {
        let ch0_res = adc.read(&mut ch0).await.unwrap();
        let ch7_res = adc.read(&mut ch7).await.unwrap();

        info!("ch0 value: {}, mv = {}", ch0_res.value(), ch0_res.to_mv());
        info!(
            "vbat(ch7)_sample value: {}, mv = {}",
            ch7_res.value(),
            ch7_res.to_mv()
        );
        Timer::after_millis(500).await;
    }
}
