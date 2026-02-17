#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::i2c::{self, I2c};
use sifli_hal::usart::{Config as UartConfig, Uart};
use sifli_hal::{bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C3 => i2c::InterruptHandler<peripherals::I2C3>;
});

// Async I2C example - read sensors on I2C3 bus
//
// Hardware:
//   PA40 (SCL) -> I2C bus SCL (with pull-up)
//   PA39 (SDA) -> I2C bus SDA (with pull-up)
//   PA38       -> Sensor board power enable 1
//   PA30       -> Sensor board power enable 2
//   PA18/PA19  -> UART1 TX/RX (debug)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    // UART debug
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();

    let _ = writeln!(usart, "\r\n=== I2C Async Example ===");

    // Enable sensor board power
    // LDO3 for general peripheral power
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo3(true);
        w.set_vdd33_ldo3_pd(false);
    });
    // PA38 and PA30: sensor board power enables
    {
        let en1 = sifli_hal::gpio::Output::new(&mut p.PA38, sifli_hal::gpio::Level::High);
        let en2 = sifli_hal::gpio::Output::new(&mut p.PA30, sifli_hal::gpio::Level::High);
        core::mem::forget(en1);
        core::mem::forget(en2);
    }
    Timer::after_millis(100).await;

    let _ = writeln!(usart, "Sensor power enabled");

    // Create async I2C
    let mut i2c = I2c::new(p.I2C3, p.PA40, p.PA39, Irqs, i2c::Config::default());

    let _ = writeln!(usart, "I2C3 async initialized");

    // Quick scan of common sensor addresses
    let _ = writeln!(usart, "Scanning common addresses...");
    let probe_addrs: &[u8] = &[0x0E, 0x19, 0x23, 0x29, 0x30, 0x6A, 0x6B];
    for &addr in probe_addrs {
        match i2c.write(addr, &[]).await {
            Ok(_) => {
                let _ = writeln!(usart, "  0x{:02X}: ACK", addr);
            }
            Err(_) => {
                let _ = writeln!(usart, "  0x{:02X}: NACK", addr);
            }
        }
    }

    // Try reading WHO_AM_I from known sensors
    let mut buf = [0u8; 1];

    // MMC5603NJ magnetometer (0x30, WHO_AM_I register 0x39, expect 0x10)
    match i2c.write_read(0x30, &[0x39], &mut buf).await {
        Ok(_) => {
            let _ = writeln!(usart, "MMC5603 (0x30) WHO_AM_I: 0x{:02X} (expect 0x10)", buf[0]);
        }
        Err(e) => {
            let _ = writeln!(usart, "MMC5603 (0x30) error: {:?}", defmt::Debug2Format(&e));
        }
    }

    // LSM6DSx IMU (0x6A or 0x6B, WHO_AM_I register 0x0F)
    for addr in [0x6Au8, 0x6B] {
        match i2c.write_read(addr, &[0x0F], &mut buf).await {
            Ok(_) => {
                let _ = writeln!(usart, "LSM6DSx (0x{:02X}) WHO_AM_I: 0x{:02X}", addr, buf[0]);
            }
            Err(_) => {}
        }
    }

    // QMC5883L / QMI8658 etc at 0x29
    match i2c.write_read(0x29, &[0x00], &mut buf).await {
        Ok(_) => {
            let _ = writeln!(usart, "Device (0x29) reg0x00: 0x{:02X}", buf[0]);
        }
        Err(_) => {}
    }

    let _ = writeln!(usart, "\nEntering periodic read loop...");

    loop {
        // Try reading from any device that responded
        match i2c.write_read(0x30, &[0x39], &mut buf).await {
            Ok(_) => {
                let _ = writeln!(usart, "WHO_AM_I: 0x{:02X}", buf[0]);
            }
            Err(_) => {
                let _ = writeln!(usart, "read failed");
            }
        }
        Timer::after_secs(2).await;
    }
}
