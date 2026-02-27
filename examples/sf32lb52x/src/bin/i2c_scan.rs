#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_io::Write;
use sifli_hal::i2c::{self, I2c};
use sifli_hal::usart::{Config as UartConfig, Uart};
use {defmt_rtt as _, panic_probe as _};

// I2C bus scanner
//
// Hardware connection:
//   PA40 (SCL) -> I2C bus SCL (with pull-up)
//   PA39 (SDA) -> I2C bus SDA (with pull-up)
//   PA18/PA19  -> UART1 TX/RX (debug)

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = sifli_hal::init(Default::default());

    // UART debug output
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, uart_config).unwrap();

    let _ = writeln!(usart, "\r\n=== I2C Bus Scanner ===");

    // Enable sensor power (board-specific)
    // LDO3 for general peripheral power
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo3(true);
        w.set_vdd33_ldo3_pd(false);
    });
    // PA38 and PA30: sensor board power enables (from raw debug example)
    {
        let mut en1 = sifli_hal::gpio::Output::new(&mut p.PA38, sifli_hal::gpio::Level::High);
        let mut en2 = sifli_hal::gpio::Output::new(&mut p.PA30, sifli_hal::gpio::Level::High);
        en1.set_high();
        en2.set_high();
        // Keep them alive by leaking (they must stay high)
        core::mem::forget(en1);
        core::mem::forget(en2);
    }
    Timer::after_millis(100).await;

    // Diagnostic: Read raw GPIO state of PA39/PA40 after power enable
    {
        let pa39_in = sifli_hal::gpio::Input::new(&mut p.PA39, sifli_hal::gpio::Pull::Up);
        let pa40_in = sifli_hal::gpio::Input::new(&mut p.PA40, sifli_hal::gpio::Pull::Up);
        sifli_hal::cortex_m_blocking_delay_us(100);
        let _ = writeln!(usart, "GPIO test (pull-up, after pwr): PA39(SDA)={}, PA40(SCL)={}",
            if pa39_in.is_high() { "HIGH" } else { "LOW" },
            if pa40_in.is_high() { "HIGH" } else { "LOW" });
        drop(pa39_in);
        drop(pa40_in);
    }

    // Create I2C3 on PA40 (SCL) / PA39 (SDA)
    let mut i2c = I2c::new_blocking(p.I2C3, p.PA40, p.PA39, i2c::Config::default());

    // Dump I2C3 registers for debugging
    {
        let i2c3 = sifli_hal::pac::I2C3;
        let cr = i2c3.cr().read();
        let sr = i2c3.sr().read();
        let lcr = i2c3.lcr().read();
        let wcr = i2c3.wcr().read();
        let bmr = i2c3.bmr().read();
        let _ = writeln!(usart, "I2C3 registers after init:");
        let _ = writeln!(usart, "  CR:  0x{:08X} (MODE={}, SCLE={}, IUE={})",
            cr.0, cr.mode(), cr.scle(), cr.iue());
        let _ = writeln!(usart, "  SR:  0x{:08X} (UB={}, TE={}, BED={}, ALD={})",
            sr.0, sr.ub(), sr.te(), sr.bed(), sr.ald());
        let _ = writeln!(usart, "  LCR: 0x{:08X} (FLV={}, SLV={})",
            lcr.0, lcr.flv(), lcr.slv());
        let _ = writeln!(usart, "  WCR: 0x{:08X} (CNT={})", wcr.0, wcr.cnt());
        let _ = writeln!(usart, "  BMR: 0x{:08X} (SCL={}, SDA={})",
            bmr.0, bmr.scl(), bmr.sda());

        // Dump PINR
        let pinr = sifli_hal::pac::HPSYS_CFG.i2c3_pinr().read();
        let _ = writeln!(usart, "  PINR: 0x{:08X} (SCL_PIN={}, SDA_PIN={})",
            pinr.0, pinr.scl_pin(), pinr.sda_pin());

        // Dump PINMUX for PA39 (SDA) and PA40 (SCL)
        let pa39 = sifli_hal::pac::HPSYS_PINMUX.pad_pa39_42(0).read();
        let pa40 = sifli_hal::pac::HPSYS_PINMUX.pad_pa39_42(1).read();
        let _ = writeln!(usart, "  PA39 PINMUX: 0x{:08X} (FSEL={}, IE={}, PE={}, PS={:?})",
            pa39.0, pa39.fsel(), pa39.ie(), pa39.pe(), pa39.ps());
        let _ = writeln!(usart, "  PA40 PINMUX: 0x{:08X} (FSEL={}, IE={}, PE={}, PS={:?})",
            pa40.0, pa40.fsel(), pa40.ie(), pa40.pe(), pa40.ps());

        // Check BMR again after a small delay
        sifli_hal::cortex_m_blocking_delay_us(100);
        let bmr2 = i2c3.bmr().read();
        let _ = writeln!(usart, "  BMR (after delay): SCL={}, SDA={}", bmr2.scl(), bmr2.sda());
    }

    let _ = writeln!(usart, "I2C3 initialized (100kHz, PA40=SCL, PA39=SDA)");
    let _ = writeln!(usart, "Scanning addresses 0x08-0x77...\n");

    // Print header
    let _ = write!(usart, "     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");

    let mut found = 0u32;

    for addr in 0x00..0x80u8 {
        if addr % 16 == 0 {
            let _ = write!(usart, "\r\n{:02X}:", addr);
        }

        if addr < 0x08 || addr > 0x77 {
            let _ = write!(usart, "   ");
            continue;
        }

        // Probe: try writing zero bytes (address-only)
        let t0 = embassy_time::Instant::now();
        match i2c.blocking_write(addr, &[]) {
            Ok(_) => {
                let _ = write!(usart, " {:02X}", addr);
                found += 1;
            }
            Err(_) => {
                let _ = write!(usart, " --");
            }
        }
    }

    let _ = writeln!(usart, "\n\n{} device(s) found.\n", found);

    // If devices found, try reading WHO_AM_I from known sensors
    if found > 0 {
        let _ = writeln!(usart, "Probing known sensors:");

        // MMC5603NJ magnetometer (0x30, WHO_AM_I register 0x39)
        let mut buf = [0u8; 1];
        if i2c.blocking_write_read(0x30, &[0x39], &mut buf).is_ok() {
            let _ = writeln!(usart, "  0x30: WHO_AM_I = 0x{:02X} (MMC5603: expect 0x10)", buf[0]);
        }

        // LSM6DSx IMU (0x6A or 0x6B, WHO_AM_I register 0x0F)
        for addr in [0x6Au8, 0x6B] {
            if i2c.blocking_write_read(addr, &[0x0F], &mut buf).is_ok() {
                let _ = writeln!(usart, "  0x{:02X}: WHO_AM_I = 0x{:02X} (LSM6DSx)", addr, buf[0]);
            }
        }
    }

    let _ = writeln!(usart, "\nDone. Entering idle loop.");

    loop {
        Timer::after_secs(1).await;
    }
}
