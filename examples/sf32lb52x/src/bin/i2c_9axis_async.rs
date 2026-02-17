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

// Async 9-axis sensor example: LSM6DS3TR-C + MMC5603NJ
// Verifies both write_read() and transaction() API with repeated START.
//
// Hardware:
//   I2C3: PA40 (SCL), PA39 (SDA)
//   PA38, PA30: Sensor board power enables
//   PA18/PA19: UART1 TX/RX (debug, 1Mbps)

const LSM6_ADDR: u8 = 0x6A;
const LSM6_WHO_AM_I: u8 = 0x0F;
const LSM6_CTRL1_XL: u8 = 0x10;
const LSM6_CTRL2_G: u8 = 0x11;
const LSM6_CTRL3_C: u8 = 0x12;
const LSM6_STATUS: u8 = 0x1E;
const LSM6_OUT_GYRO: u8 = 0x22;
const LSM6_OUT_ACCEL: u8 = 0x28;

const MMC_ADDR: u8 = 0x30;
const MMC_PRODUCT_ID: u8 = 0x39;
const MMC_XOUT0: u8 = 0x00;
const MMC_STATUS: u8 = 0x18;
const MMC_CTRL0: u8 = 0x1B;

/// Parse 16-bit signed value from two bytes (little-endian)
fn i16_from_le(buf: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([buf[offset], buf[offset + 1]])
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();

    let _ = writeln!(usart, "\r\n=== Async 9-Axis Sensor Example ===");
    let _ = writeln!(usart, "Tests write_read() and transaction() API\r\n");

    // Enable sensor board power
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo3(true);
        w.set_vdd33_ldo3_pd(false);
    });
    {
        let en1 = sifli_hal::gpio::Output::new(&mut p.PA38, sifli_hal::gpio::Level::High);
        let en2 = sifli_hal::gpio::Output::new(&mut p.PA30, sifli_hal::gpio::Level::High);
        core::mem::forget(en1);
        core::mem::forget(en2);
    }
    Timer::after_millis(100).await;

    let mut i2c = I2c::new(p.I2C3, p.PA40, p.PA39, Irqs, i2c::Config::default());

    // --- Verify transaction() API with WHO_AM_I reads ---
    let _ = writeln!(usart, "--- transaction() API test ---");

    // Test 1: transaction([Write, Read]) should use repeated START
    {
        use embedded_hal_async::i2c::I2c as _;
        let mut buf = [0u8; 1];
        match i2c
            .transaction(
                MMC_ADDR,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[MMC_PRODUCT_ID]),
                    embedded_hal_async::i2c::Operation::Read(&mut buf),
                ],
            )
            .await
        {
            Ok(_) => {
                let _ = writeln!(
                    usart,
                    "transaction() MMC5603 WHO_AM_I: 0x{:02X} (expect 0x10) {}",
                    buf[0],
                    if buf[0] == 0x10 { "OK" } else { "MISMATCH" }
                );
            }
            Err(e) => {
                let _ = writeln!(usart, "transaction() MMC5603 error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    // Test 2: Same read using write_read() for comparison
    {
        let mut buf = [0u8; 1];
        match i2c.write_read(MMC_ADDR, &[MMC_PRODUCT_ID], &mut buf).await {
            Ok(_) => {
                let _ = writeln!(
                    usart,
                    "write_read()  MMC5603 WHO_AM_I: 0x{:02X} (expect 0x10) {}",
                    buf[0],
                    if buf[0] == 0x10 { "OK" } else { "MISMATCH" }
                );
            }
            Err(e) => {
                let _ = writeln!(usart, "write_read()  MMC5603 error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    // Test 3: LSM6DS3TR-C WHO_AM_I via transaction()
    {
        use embedded_hal_async::i2c::I2c as _;
        let mut buf = [0u8; 1];
        match i2c
            .transaction(
                LSM6_ADDR,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[LSM6_WHO_AM_I]),
                    embedded_hal_async::i2c::Operation::Read(&mut buf),
                ],
            )
            .await
        {
            Ok(_) => {
                let _ = writeln!(
                    usart,
                    "transaction() LSM6DS3 WHO_AM_I: 0x{:02X} (expect 0x6A) {}",
                    buf[0],
                    if buf[0] == 0x6A { "OK" } else { "MISMATCH" }
                );
            }
            Err(e) => {
                let _ = writeln!(usart, "transaction() LSM6DS3 error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    // --- Initialize sensors ---
    let _ = writeln!(usart, "\n--- Sensor init ---");

    // LSM6DS3TR-C init
    let has_imu = match init_sensor_async(&mut i2c, &mut usart).await {
        Ok(v) => v,
        Err(e) => {
            let _ = writeln!(usart, "IMU init error: {:?}", defmt::Debug2Format(&e));
            false
        }
    };

    // MMC5603NJ SET operation
    if i2c.write(MMC_ADDR, &[MMC_CTRL0, 0x08]).await.is_ok() {
        let _ = writeln!(usart, "MMC5603NJ SET done");
    }

    if !has_imu {
        let _ = writeln!(usart, "No IMU, entering mag-only loop");
    }

    Timer::after_millis(100).await;
    let _ = writeln!(usart, "\n--- Sensor data loop (async) ---\r\n");

    loop {
        if has_imu {
            // Read status + accel + gyro data
            let mut status = [0u8; 1];
            if i2c.write_read(LSM6_ADDR, &[LSM6_STATUS], &mut status).await.is_ok()
                && (status[0] & 0x03) != 0
            {
                let mut gyro_buf = [0u8; 6];
                let mut accel_buf = [0u8; 6];
                // Read gyro and accel separately using async
                if i2c
                    .write_read(LSM6_ADDR, &[LSM6_OUT_GYRO], &mut gyro_buf)
                    .await
                    .is_ok()
                    && i2c
                        .write_read(LSM6_ADDR, &[LSM6_OUT_ACCEL], &mut accel_buf)
                        .await
                        .is_ok()
                {
                    let ax = i16_from_le(&accel_buf, 0);
                    let ay = i16_from_le(&accel_buf, 2);
                    let az = i16_from_le(&accel_buf, 4);
                    let gx = i16_from_le(&gyro_buf, 0);
                    let gy = i16_from_le(&gyro_buf, 2);
                    let gz = i16_from_le(&gyro_buf, 4);
                    let _ = writeln!(
                        usart,
                        "A:{:6},{:6},{:6} G:{:6},{:6},{:6}",
                        ax, ay, az, gx, gy, gz,
                    );
                }
            }
        }

        // MMC5603NJ single-shot read
        if i2c.write(MMC_ADDR, &[MMC_CTRL0, 0x01]).await.is_ok() {
            Timer::after_millis(10).await;
            let mut status = [0u8; 1];
            if i2c.write_read(MMC_ADDR, &[MMC_STATUS], &mut status).await.is_ok()
                && (status[0] & 0x40) != 0
            {
                let mut buf = [0u8; 6];
                if i2c.write_read(MMC_ADDR, &[MMC_XOUT0], &mut buf).await.is_ok() {
                    let mx = ((buf[0] as u16) << 8 | buf[1] as u16) as i32 - 32768;
                    let my = ((buf[2] as u16) << 8 | buf[3] as u16) as i32 - 32768;
                    let mz = ((buf[4] as u16) << 8 | buf[5] as u16) as i32 - 32768;
                    let _ = writeln!(usart, "M:{:6},{:6},{:6}", mx, my, mz);
                }
            }
        }

        Timer::after_millis(100).await;
    }
}

async fn init_sensor_async(
    i2c: &mut I2c<'_, impl i2c::Instance, sifli_hal::mode::Async>,
    usart: &mut impl Write,
) -> Result<bool, i2c::Error> {
    let mut buf = [0u8; 1];
    i2c.write_read(LSM6_ADDR, &[LSM6_WHO_AM_I], &mut buf).await?;
    let _ = writeln!(usart, "LSM6DS3TR-C WHO_AM_I: 0x{:02X}", buf[0]);
    if buf[0] != 0x6A {
        let _ = writeln!(usart, "  WARNING: expected 0x6A");
    }
    // CTRL3_C: BDU=1, IF_INC=1
    i2c.write(LSM6_ADDR, &[LSM6_CTRL3_C, 0x44]).await?;
    // CTRL1_XL: 104Hz, ±4g
    i2c.write(LSM6_ADDR, &[LSM6_CTRL1_XL, 0x48]).await?;
    // CTRL2_G: 104Hz, ±500dps
    i2c.write(LSM6_ADDR, &[LSM6_CTRL2_G, 0x44]).await?;
    let _ = writeln!(usart, "LSM6DS3TR-C initialized (104Hz, +/-4g, +/-500dps)");
    Ok(true)
}
