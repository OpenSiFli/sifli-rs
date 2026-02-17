#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::i2c::{self, I2c};
use sifli_hal::usart::{Config as UartConfig, Uart};
use {defmt_rtt as _, panic_probe as _};

// Blocking I2C example - scan and read sensors
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

    let _ = writeln!(usart, "\r\n=== I2C Blocking Example ===");

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
    let _ = writeln!(usart, "Sensor power enabled");

    // Create blocking I2C
    let mut i2c = I2c::new_blocking(p.I2C3, p.PA40, p.PA39, i2c::Config::default());
    let _ = writeln!(usart, "I2C3 blocking initialized");

    // Dump registers
    {
        let i2c3 = sifli_hal::pac::I2C3;
        let cr = i2c3.cr().read();
        let bmr = i2c3.bmr().read();
        let lcr = i2c3.lcr().read();
        let wcr = i2c3.wcr().read();
        let _ = writeln!(usart, "CR: 0x{:08X} MODE={}, SCLE={}, IUE={}", cr.0, cr.mode(), cr.scle(), cr.iue());
        let _ = writeln!(usart, "BMR: SCL={}, SDA={}", bmr.scl(), bmr.sda());
        let _ = writeln!(usart, "LCR: FLV={}, SLV={}", lcr.flv(), lcr.slv());
        let _ = writeln!(usart, "WCR: CNT={}", wcr.cnt());
    }

    // Test single address first - probe 0x08 (likely NACK) to see timing
    {
        let _ = write!(usart, "Probe 0x08: ");
        let t0 = embassy_time::Instant::now();
        let result = i2c.blocking_write(0x08, &[]);
        let dt = t0.elapsed().as_millis();
        let _ = writeln!(usart, "{:?} ({}ms)", defmt::Debug2Format(&result), dt);

        // Check SR after probe
        let sr = sifli_hal::pac::I2C3.sr().read();
        let _ = writeln!(usart, "SR after: 0x{:08X} TE={} NACK={} BED={} UB={}",
            sr.0, sr.te(), sr.nack(), sr.bed(), sr.ub());
    }

    // Test a known device address (0x29 was found in previous scan)
    {
        let _ = write!(usart, "Probe 0x29: ");
        let t0 = embassy_time::Instant::now();
        let result = i2c.blocking_write(0x29, &[]);
        let dt = t0.elapsed().as_millis();
        let _ = writeln!(usart, "{:?} ({}ms)", defmt::Debug2Format(&result), dt);

        let sr = sifli_hal::pac::I2C3.sr().read();
        let _ = writeln!(usart, "SR after: 0x{:08X} TE={} NACK={} BED={} UB={}",
            sr.0, sr.te(), sr.nack(), sr.bed(), sr.ub());
    }

    // Test 0x30 (MMC5603)
    {
        let _ = write!(usart, "Probe 0x30: ");
        let t0 = embassy_time::Instant::now();
        let result = i2c.blocking_write(0x30, &[]);
        let dt = t0.elapsed().as_millis();
        let _ = writeln!(usart, "{:?} ({}ms)", defmt::Debug2Format(&result), dt);

        let sr = sifli_hal::pac::I2C3.sr().read();
        let _ = writeln!(usart, "SR after: 0x{:08X} TE={} NACK={} BED={} UB={}",
            sr.0, sr.te(), sr.nack(), sr.bed(), sr.ub());
    }

    // If any device responded, try reading registers
    let _ = writeln!(usart, "\nTrying write_read on 0x29...");
    {
        let mut buf = [0u8; 1];
        match i2c.blocking_write_read(0x29, &[0x00], &mut buf) {
            Ok(_) => {
                let _ = writeln!(usart, "  0x29 reg[0x00] = 0x{:02X}", buf[0]);
            }
            Err(e) => {
                let _ = writeln!(usart, "  0x29 reg[0x00] err: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    let _ = writeln!(usart, "Trying write_read on 0x30...");
    {
        let mut buf = [0u8; 1];
        match i2c.blocking_write_read(0x30, &[0x39], &mut buf) {
            Ok(_) => {
                let _ = writeln!(usart, "  0x30 reg[0x39] = 0x{:02X}", buf[0]);
            }
            Err(e) => {
                let _ = writeln!(usart, "  0x30 reg[0x39] err: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    let _ = writeln!(usart, "\nDone.");
    loop {
        Timer::after_secs(5).await;
    }
}
