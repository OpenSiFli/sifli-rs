#![no_std]
#![no_main]

use core::fmt::Write;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write as _;
use heapless::String;
use panic_probe as _;
use sifli_hal::usart::{Config, Uart};
use sifli_hal::{bind_interrupts, peripherals, usart};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// This example defaults to using the debug UART at a baud rate of 115200.
// This is because `Buffered Usart`, which relies only on interrupts
// (without DMA), cannot handle higher baud rates. You can view debug logs
// and output via SiFli Trace, but this may interfere with debugging
// and flashing. Before the next download, you can manually press
// the reset button or change the UART pins to any other GPIO in the
// following code.

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 1000000;
    let mut usart = Uart::new(
        p.USART1,
        p.PA18,
        p.PA19,
        Irqs,
        p.DMAC1_CH1,
        p.DMAC1_CH2,
        config,
    )
    .unwrap();

    info!("SiFli USART DMA example started");

    unwrap!(usart.write_all(b"Hello SiFli!\n").await);
    unwrap!(usart.write_all(b"Hello Embassy!\n").await);
    unwrap!(usart.write_all(b"Hello Embedded Rust!\n").await);
    unwrap!(
        usart
            .write_all(b"DMA USART. wrote Hello, starting echo\n")
            .await
    );

    info!("wrote Hello, starting echo");

    for n in 0u32.. {
        Timer::after(Duration::from_millis(500)).await;
        let mut s: String<128> = String::new();
        core::write!(
            &mut s,
            "DMA USART transmit is testing! {}!\r\n, Try write someting at 5 bytes\r\n",
            n
        )
        .unwrap();

        unwrap!(usart.write(s.as_bytes()).await);
        info!("wrote DMA");

        let mut buffer = [0u8; 5];
        usart.read(&mut buffer).await.unwrap();

        unwrap!(usart.write_all(b"Read from your writeing: ").await);
        unwrap!(usart.write_all(&buffer).await);

        info!(
            "read from you: {}",
            core::str::from_utf8(&buffer).unwrap_or("Invalid UTF-8")
        );
    }
}
