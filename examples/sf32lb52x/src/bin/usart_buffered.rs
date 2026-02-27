#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embedded_io_async::{Read, Write};
use panic_probe as _;
use sifli_hal::usart::{BufferedUart, Config};
use sifli_hal::{bind_interrupts, peripherals, usart};

bind_interrupts!(struct Irqs {
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
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

    let mut tx_buf = [0u8; 256];
    let mut rx_buf = [0u8; 256];

    let mut config = Config::default();
    // `BufferedUsart` using only interrupts without DMA is barely usable at 1M baud rate.
    // config.baudrate = 1000000;
    config.baudrate = 115200;
    let mut usart = BufferedUart::new(
        p.USART1,
        Irqs,
        p.PA18,
        p.PA19,
        &mut tx_buf,
        &mut rx_buf,
        config,
    )
    .unwrap();

    unwrap!(usart.write_all(b"Hello SiFli!\n").await);
    unwrap!(usart.write_all(b"Hello Embassy!\n").await);
    unwrap!(usart.write_all(b"Hello Embedded Rust!\n").await);
    unwrap!(usart.write_all(b"wrote Hello, starting echo\n").await);

    info!("wrote Hello, starting echo");

    let mut buf = [0; 5];
    loop {
        // When using defmt, be cautious with the info and other logging macros
        // If you're using a single channel (as is usually the case), defmt requires global_logger to
        // acquire interrupts to be disabled.
        // For example, defmt-rtt uses critical_section, which temporarily disables global interrupts.
        // This can lead to USART Overrun error(SR.ORE), causing some data to be lost.
        usart.read_exact(&mut buf[..]).await.unwrap();
        // info!("Received:{} {}", buf, buf.len());
        usart.write_all(&buf[..]).await.unwrap();

        // use embedded_io_async::BufRead;
        // let buf = usart.fill_buf().await.unwrap();
        // let n = buf.len();
        // usart.consume(n);
    }
}
