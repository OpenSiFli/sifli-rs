#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::i2c::{self, I2c};
use sifli_hal::usart::{Config as UartConfig, Uart};
use {defmt_rtt as _, panic_probe as _};

// Ambient light sensor example: LTR-303ALS-01
//
// Hardware:
//   I2C3: PA40 (SCL), PA39 (SDA)
//   PA38, PA30: Sensor board power enables
//   PA18/PA19: UART1 TX/RX (debug, 1Mbps)
//
// Device:
//   LTR-303ALS-01 @ 0x29 - Ambient Light Sensor (CH0 visible+IR, CH1 IR only)

// --- LTR-303ALS-01 registers ---
const LTR_ADDR: u8 = 0x29;
const LTR_ALS_CONTR: u8 = 0x80; // Control register (gain, active mode)
const LTR_ALS_MEAS_RATE: u8 = 0x85; // Measurement rate + integration time
const LTR_PART_ID: u8 = 0x86; // Part ID (expect 0xA0)
const LTR_MANUFAC_ID: u8 = 0x87; // Manufacturer ID (expect 0x05)
const LTR_ALS_STATUS: u8 = 0x8C; // ALS status
const LTR_ALS_DATA_CH1_0: u8 = 0x88; // CH1 low byte (read CH1 first, then CH0)
// CH1 high = 0x89, CH0 low = 0x8A, CH0 high = 0x8B

// ALS_CONTR register bits
const ALS_MODE_ACTIVE: u8 = 0x01; // Bit 0: active mode
const ALS_GAIN_1X: u8 = 0x00; // Bits [4:2]: gain = 1x (1-64k lux)
const ALS_GAIN_4X: u8 = 0x08; // gain = 4x
const ALS_GAIN_8X: u8 = 0x0C; // gain = 8x
const ALS_GAIN_48X: u8 = 0x18; // gain = 48x
const ALS_GAIN_96X: u8 = 0x1C; // gain = 96x (0.01-600 lux)

/// Read a single register
fn read_reg(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, reg: u8) -> Result<u8, i2c::Error> {
    let mut buf = [0u8; 1];
    i2c.blocking_write_read(LTR_ADDR, &[reg], &mut buf)?;
    Ok(buf[0])
}

/// Write a single register
fn write_reg(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, reg: u8, val: u8) -> Result<(), i2c::Error> {
    i2c.blocking_write(LTR_ADDR, &[reg, val])
}

/// Initialize LTR-303: 1x gain, 100ms integration, 500ms measurement rate
fn init_ltr303(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>, usart: &mut impl Write) -> bool {
    // Check Part ID
    match read_reg(i2c, LTR_PART_ID) {
        Ok(id) => {
            let _ = writeln!(usart, "LTR-303 Part ID: 0x{:02X} (expect 0xA0)", id);
        }
        Err(e) => {
            let _ = writeln!(usart, "LTR-303 not found: {:?}", defmt::Debug2Format(&e));
            return false;
        }
    }

    // Check Manufacturer ID
    if let Ok(id) = read_reg(i2c, LTR_MANUFAC_ID) {
        let _ = writeln!(usart, "LTR-303 Manufacturer ID: 0x{:02X} (expect 0x05)", id);
    }

    // ALS_MEAS_RATE: integration time = 100ms (000), measurement rate = 500ms (011)
    // Bits [5:3] = integration time, Bits [2:0] = measurement rate
    // 000_011 = 0x03
    let _ = write_reg(i2c, LTR_ALS_MEAS_RATE, 0x03);

    // ALS_CONTR: Gain 1x, Active mode
    let _ = write_reg(i2c, LTR_ALS_CONTR, ALS_GAIN_1X | ALS_MODE_ACTIVE);

    let _ = writeln!(usart, "LTR-303 initialized (1x gain, 100ms integration, 500ms rate)");
    true
}

/// Read ALS data (CH0 = visible+IR, CH1 = IR only)
/// Returns (ch0, ch1) raw values, or None if no new data
fn read_ltr303(i2c: &mut I2c<'_, impl i2c::Instance, impl sifli_hal::mode::Mode>) -> Option<(u16, u16)> {
    // Check status
    let status = read_reg(i2c, LTR_ALS_STATUS).ok()?;
    if status & 0x04 == 0 {
        return None; // No new data
    }

    // Must read CH1 first, then CH0 (per datasheet)
    // Read 4 bytes starting from CH1_0 (0x88): CH1_L, CH1_H, CH0_L, CH0_H
    let mut buf = [0u8; 4];
    i2c.blocking_write_read(LTR_ADDR, &[LTR_ALS_DATA_CH1_0], &mut buf).ok()?;

    let ch1 = (buf[1] as u16) << 8 | buf[0] as u16;
    let ch0 = (buf[3] as u16) << 8 | buf[2] as u16;

    Some((ch0, ch1))
}

/// Calculate approximate lux from CH0 and CH1 raw values
/// Simplified formula from LTR-303 application note (gain=1x, integration=100ms)
fn calculate_lux(ch0: u16, ch1: u16) -> u32 {
    if ch0 == 0 {
        return 0;
    }

    let c0 = ch0 as u32;
    let c1 = ch1 as u32;
    let ratio = c1 * 100 / c0;

    // Lux formula depends on CH1/CH0 ratio
    // Coefficients from LTR-303 application note (gain=1x, integration=100ms)
    if ratio < 45 {
        (17743 * c0 + 11059 * c1) / 10000
    } else if ratio < 64 {
        (42785 * c0 - 19548 * c1) / 10000
    } else if ratio < 85 {
        (5926 * c0 + 1185 * c1) / 10000
    } else if ratio <= 100 {
        // Mostly IR, very little visible
        (5926 * c0 + 1185 * c1) / 10000
    } else {
        // CH1 > CH0: dominant IR source (e.g., incandescent, dark with IR noise)
        // Return a rough estimate based on visible portion
        if c0 > c1 { 0 } else { c0 / 10 }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    // UART debug
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();

    let _ = writeln!(usart, "\r\n=== Ambient Light Sensor Example ===");
    let _ = writeln!(usart, "LTR-303ALS-01\r\n");

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

    // Initialize sensor
    if !init_ltr303(&mut i2c, &mut usart) {
        let _ = writeln!(usart, "Failed to initialize LTR-303!");
        loop { Timer::after_secs(1).await; }
    }

    // Wait for first measurement
    Timer::after_millis(200).await;

    let _ = writeln!(usart, "\nReading light data...\r\n");

    let mut gain = 0u8; // Current gain setting index

    loop {
        if let Some((ch0, ch1)) = read_ltr303(&mut i2c) {
            let lux = calculate_lux(ch0, ch1);
            let _ = writeln!(usart,
                "CH0={:5} CH1={:5} ratio={:3}% lux~{:5}",
                ch0, ch1,
                if ch0 > 0 { (ch1 as u32 * 100 / ch0 as u32) as u16 } else { 0 },
                lux,
            );

            // Auto-gain: switch gain if reading is too low or saturated
            if ch0 < 100 && gain < 4 {
                gain += 1;
                let gain_val = match gain {
                    1 => ALS_GAIN_4X,
                    2 => ALS_GAIN_8X,
                    3 => ALS_GAIN_48X,
                    _ => ALS_GAIN_96X,
                };
                let _ = write_reg(&mut i2c, LTR_ALS_CONTR, gain_val | ALS_MODE_ACTIVE);
                let _ = writeln!(usart, "  -> Gain increased to {}x",
                    match gain { 1 => 4, 2 => 8, 3 => 48, _ => 96 });
            } else if ch0 > 50000 && gain > 0 {
                gain -= 1;
                let gain_val = match gain {
                    0 => ALS_GAIN_1X,
                    1 => ALS_GAIN_4X,
                    2 => ALS_GAIN_8X,
                    _ => ALS_GAIN_48X,
                };
                let _ = write_reg(&mut i2c, LTR_ALS_CONTR, gain_val | ALS_MODE_ACTIVE);
                let _ = writeln!(usart, "  -> Gain decreased to {}x",
                    match gain { 0 => 1, 1 => 4, 2 => 8, _ => 48 });
            }
        }

        Timer::after_millis(500).await;
    }
}
