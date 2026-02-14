#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::Delay;
use sifli_hal::usart::{Config, Uart};
use defmt_rtt as _;
use panic_probe as _;
use embedded_hal_async::delay::DelayNs;

// This example uses the debug UART by default. You can view debug logs  
// and output via SiFli Trace, but this may interfere with debugging  
// and flashing. Before the next download, you may need manually press 
// the reset button or change the UART pins to any other GPIO in the 
// following code.  

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, config).unwrap();

    unwrap!(usart.blocking_write(b"Hello SiFli!\n"));
    unwrap!(usart.blocking_write(b"Hello Embassy!\n"));
    unwrap!(usart.blocking_write(b"Hello Embedded Rust!\n"));
    unwrap!(usart.blocking_write(b"wrote Hello, starting echo\n"));
    
    info!("wrote Hello, starting echo(Try write someting at 5 bytes)");

    loop {
        unwrap!(usart.blocking_write(b"Hello SiFli!\n"));
        unwrap!(usart.blocking_write(b"Hello Embassy!\n"));

    //    let mut buf = [0u8; 5];
      //  unwrap!(usart.blocking_read(&mut buf));
       // unwrap!(usart.blocking_write(&buf));
       Delay.delay_ms(1000).await;
    }
}
