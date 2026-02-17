#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::i2c::{self, I2c};
use sifli_hal::usart::{Config as UartConfig, Uart};
use {defmt_rtt as _, panic_probe as _};

// 9-axis sensor example: LSM6DS3TR-C (6-axis IMU) + MMC5603NJ (3-axis magnetometer)
//
// Hardware:
//   I2C3: PA40 (SCL), PA39 (SDA)
//   PA38, PA30: Sensor board power enables
//   PA18/PA19: UART1 TX/RX (debug, 1Mbps)
//
// Devices:
//   LSM6DS3TR-C @ 0x6A  - Accelerometer + Gyroscope
//   MMC5603NJ   @ 0x30  - Magnetometer

// --- LSM6DS3TR-C registers ---
const LSM6_ADDR: u8 = 0x6A;
const LSM6_WHO_AM_I: u8 = 0x0F;
const LSM6_CTRL1_XL: u8 = 0x10; // Accelerometer control
const LSM6_CTRL2_G: u8 = 0x11; // Gyroscope control
const LSM6_CTRL3_C: u8 = 0x12; // Common control (BDU, IF_INC)
const LSM6_STATUS: u8 = 0x1E;
const LSM6_OUT_GYRO: u8 = 0x22; // OUTX_L_G, 6 bytes (gyro XYZ)
const LSM6_OUT_ACCEL: u8 = 0x28; // OUTX_L_XL, 6 bytes (accel XYZ)

// --- MMC5603NJ registers ---
const MMC_ADDR: u8 = 0x30;
const MMC_PRODUCT_ID: u8 = 0x39;
const MMC_XOUT0: u8 = 0x00; // X[19:12]
const MMC_STATUS: u8 = 0x18; // Bit 6: Meas_M_Done
const MMC_CTRL0: u8 = 0x1B;
#[allow(dead_code)]
const MMC_CTRL2: u8 = 0x1D;

/// Read a single register
fn read_reg(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, addr: u8, reg: u8) -> Result<u8, i2c::Error> {
    let mut buf = [0u8; 1];
    i2c.blocking_write_read(addr, &[reg], &mut buf)?;
    Ok(buf[0])
}

/// Write a single register
fn write_reg(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, addr: u8, reg: u8, val: u8) -> Result<(), i2c::Error> {
    i2c.blocking_write(addr, &[reg, val])
}

/// Parse 16-bit signed value from two bytes (little-endian)
fn i16_from_le(buf: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([buf[offset], buf[offset + 1]])
}

/// Initialize LSM6DS3TR-C: 104Hz ODR, ±4g accel, ±500dps gyro
fn init_lsm6(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, usart: &mut impl Write) -> bool {
    let who = read_reg(i2c, LSM6_ADDR, LSM6_WHO_AM_I);
    match who {
        Ok(id) => {
            let _ = writeln!(usart, "LSM6DS3TR-C WHO_AM_I: 0x{:02X}", id);
            if id != 0x6A {
                let _ = writeln!(usart, "  WARNING: expected 0x6A");
            }
        }
        Err(e) => {
            let _ = writeln!(usart, "LSM6DS3TR-C not found: {:?}", defmt::Debug2Format(&e));
            return false;
        }
    }

    // CTRL3_C: BDU=1 (block data update), IF_INC=1 (auto-increment address)
    let _ = write_reg(i2c, LSM6_ADDR, LSM6_CTRL3_C, 0x44);

    // CTRL1_XL: ODR=104Hz (0100), FS=±4g (10), BW=400Hz (00)
    // 0100_10_00 = 0x48
    let _ = write_reg(i2c, LSM6_ADDR, LSM6_CTRL1_XL, 0x48);

    // CTRL2_G: ODR=104Hz (0100), FS=±500dps (01), FS_125=0
    // 0100_01_0_0 = 0x44
    let _ = write_reg(i2c, LSM6_ADDR, LSM6_CTRL2_G, 0x44);

    let _ = writeln!(usart, "LSM6DS3TR-C initialized (104Hz, ±4g, ±500dps)");
    true
}

/// Read LSM6DS3TR-C accelerometer and gyroscope data
fn read_lsm6(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>) -> Option<([i16; 3], [i16; 3])> {
    // Check data ready
    let status = read_reg(i2c, LSM6_ADDR, LSM6_STATUS).ok()?;
    if status & 0x03 == 0 {
        return None; // No new data
    }

    // Read gyroscope (6 bytes from 0x22, auto-increment)
    let mut gyro_buf = [0u8; 6];
    i2c.blocking_write_read(LSM6_ADDR, &[LSM6_OUT_GYRO], &mut gyro_buf).ok()?;
    let gx = i16_from_le(&gyro_buf, 0);
    let gy = i16_from_le(&gyro_buf, 2);
    let gz = i16_from_le(&gyro_buf, 4);

    // Read accelerometer (6 bytes from 0x28, auto-increment)
    let mut accel_buf = [0u8; 6];
    i2c.blocking_write_read(LSM6_ADDR, &[LSM6_OUT_ACCEL], &mut accel_buf).ok()?;
    let ax = i16_from_le(&accel_buf, 0);
    let ay = i16_from_le(&accel_buf, 2);
    let az = i16_from_le(&accel_buf, 4);

    Some(([ax, ay, az], [gx, gy, gz]))
}

/// Initialize MMC5603NJ
fn init_mmc5603(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, usart: &mut impl Write) -> bool {
    let who = read_reg(i2c, MMC_ADDR, MMC_PRODUCT_ID);
    match who {
        Ok(id) => {
            let _ = writeln!(usart, "MMC5603NJ Product ID: 0x{:02X}", id);
            if id != 0x10 {
                let _ = writeln!(usart, "  WARNING: expected 0x10");
            }
        }
        Err(e) => {
            let _ = writeln!(usart, "MMC5603NJ not found: {:?}", defmt::Debug2Format(&e));
            return false;
        }
    }

    // Do a SET operation first for proper sensor offset calibration (CTRL0 bit 3)
    let _ = write_reg(i2c, MMC_ADDR, MMC_CTRL0, 0x08);

    let _ = writeln!(usart, "MMC5603NJ initialized (single-shot mode)");
    true
}

/// Read MMC5603NJ magnetometer data using single-shot measurement.
/// Triggers a measurement, waits, then reads result.
fn read_mmc5603(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>) -> Option<[i32; 3]> {
    // Trigger single measurement: CTRL0 TM_M bit (bit 0)
    write_reg(i2c, MMC_ADDR, MMC_CTRL0, 0x01).ok()?;

    // Wait for measurement (~8ms typical for default BW)
    embassy_time::block_for(embassy_time::Duration::from_millis(10));

    // Check status (bit 6 = Meas_M_Done)
    let status = read_reg(i2c, MMC_ADDR, MMC_STATUS).ok()?;
    if status & 0x40 == 0 {
        return None;
    }

    // Read 6 bytes from 0x00 (XOUT0..ZOUT1), 16-bit mode
    let mut buf = [0u8; 6];
    i2c.blocking_write_read(MMC_ADDR, &[MMC_XOUT0], &mut buf).ok()?;

    // Data is unsigned 16-bit, offset binary (midpoint = 32768 = 0 Gauss)
    let mx = ((buf[0] as u16) << 8 | buf[1] as u16) as i32 - 32768;
    let my = ((buf[2] as u16) << 8 | buf[3] as u16) as i32 - 32768;
    let mz = ((buf[4] as u16) << 8 | buf[5] as u16) as i32 - 32768;

    Some([mx, my, mz])
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    // UART debug
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();

    let _ = writeln!(usart, "\r\n=== 9-Axis Sensor Example ===");
    let _ = writeln!(usart, "LSM6DS3TR-C (Accel+Gyro) + MMC5603NJ (Mag)\r\n");

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

    // Create blocking I2C
    let mut i2c = I2c::new_blocking(p.I2C3, p.PA40, p.PA39, i2c::Config::default());

    // Initialize sensors
    let has_imu = init_lsm6(&mut i2c, &mut usart);
    let has_mag = init_mmc5603(&mut i2c, &mut usart);

    if !has_imu && !has_mag {
        let _ = writeln!(usart, "\nNo sensors found!");
        loop { Timer::after_secs(1).await; }
    }

    Timer::after_millis(100).await; // Wait for first measurements

    let _ = writeln!(usart, "\nReading sensor data...\r\n");

    loop {
        if has_imu {
            if let Some((accel, gyro)) = read_lsm6(&mut i2c) {
                // ±4g: sensitivity = 0.122 mg/LSB → mg = raw * 122 / 1000
                // ±500dps: sensitivity = 17.50 mdps/LSB
                let _ = writeln!(usart,
                    "Accel: X={:6} Y={:6} Z={:6}  Gyro: X={:6} Y={:6} Z={:6}",
                    accel[0], accel[1], accel[2],
                    gyro[0], gyro[1], gyro[2],
                );
            }
        }

        if has_mag {
            if let Some(mag) = read_mmc5603(&mut i2c) {
                // 16-bit mode: 1 LSB ≈ 0.0625 mGauss (range ±30 Gauss)
                let _ = writeln!(usart,
                    "Mag:   X={:6} Y={:6} Z={:6}",
                    mag[0], mag[1], mag[2],
                );
            }
        }

        Timer::after_millis(100).await;
    }
}
