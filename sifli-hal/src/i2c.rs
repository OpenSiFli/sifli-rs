//! Inter-Integrated Circuit (I2C) bus
//!
//! SiFli I2C controller implementation based on DesignWare I2C IP

use core::marker::PhantomData;
use embassy_time::{Duration, Instant};

use crate::gpio::{AfType, AnyPin, Pin as GpioPin, Pull};
use crate::rcc::{self, RccEnableReset, RccGetFreq};
use crate::time::Hertz;
use crate::{Peripheral, PeripheralRef, interrupt, into_ref};

mod sealed {
    pub trait Sealed {}
}
use sealed::Sealed;

/// I2C error
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK received
    Nack,
    /// Timeout
    Timeout,
    /// Overrun/underrun
    Overrun,
    /// Zero-length transfer
    ZeroLength,
    /// Invalid parameter
    InvalidParam,
}

/// I2C configuration
pub struct Config {
    /// I2C bus frequency
    pub frequency: Hertz,
    /// Addressing mode
    pub addr_mode: AddressMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Hertz(100_000), // 100 kHz standard mode
            addr_mode: AddressMode::SevenBit,
        }
    }
}

/// I2C addressing mode
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum AddressMode {
    /// 7-bit addressing
    SevenBit,
    /// 10-bit addressing
    TenBit,
}

/// I2C bus mode
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum BusMode {
    StandardMode = 0b00,
    FastMode = 0b01,
    // HighSpeedStandardFallback = 0b10,
    // HighSpeedFastFallback = 0b11,
}

/// I2C driver
pub struct I2c<'d, T: Instance, M: Mode> {
    _peri: PeripheralRef<'d, T>,
    _mode: PhantomData<M>,
}

/// I2C mode marker
pub trait Mode: Sealed {}

/// Blocking mode
pub struct Blocking;
impl Sealed for Blocking {}
impl Mode for Blocking {}

/// Async mode (not implemented yet)
pub struct Async;
impl Sealed for Async {}
impl Mode for Async {}

const TIMEOUT_DEFAULT_MS: u64 = 1000;

impl<'d, T: Instance> I2c<'d, T, Blocking> {
    /// Create a new blocking I2C driver
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);

        // Convert to AnyPin to simplify
        let scl = scl.map_into();
        let sda = sda.map_into();

        // Enable I2C clock and reset
        rcc::enable_and_reset::<T>();

        // Configure GPIO pins for I2C
        // SCL and SDA must be configured as:
        // - Alternate function (I2C)
        // - Open-drain output
        // - Pull-up enabled (I2C requires pull-up)

        // Configure pinmux to route I2C SCL/SDA to selected pins
        T::configure_pins(&scl, &sda);

        // Configure GPIO pins as I2C function using SealedPin trait
        // Set FSEL to I2C mode, enable input, enable pull-up
        use crate::gpio::SealedPin;
        scl.set_function(T::scl_fsel(), AfType::new(Pull::Up));
        sda.set_function(T::sda_fsel(), AfType::new(Pull::Up));

        // Explicitly ensure IE (Input Enable) is set - critical for I2C!
        // I2C is bidirectional, so input must be enabled even though it also drives output
        use crate::gpio::hpsys::HpsysPin;
        let mut scl_pin = HpsysPin::new(scl.pin_bank());
        let mut sda_pin = HpsysPin::new(sda.pin_bank());
        scl_pin.set_ie(true);
        sda_pin.set_ie(true);

        // Get I2C clock frequency
        let i2c_clk = T::frequency().expect("I2C clock not configured");

        // Calculate timing parameters
        let (lcr, wcr) = Self::compute_timing(i2c_clk, config.frequency);

        let regs = T::regs();

        // Perform I2C module reset and bus recovery
        // This helps recover from previous abnormal states
        regs.cr().modify(|w| w.set_ur(true));
        crate::cortex_m_blocking_delay_us(100);
        regs.cr().modify(|w| w.set_ur(false));

        // Check bus state and attempt recovery if needed
        let bmr = regs.bmr().read();
        if bmr.scl() && bmr.sda() {
            // Both lines high - send clock cycles for bus recovery
            regs.cr().modify(|w| w.set_rstreq(true));

            // Wait for recovery complete (RSTREQ auto-clears)
            let start = embassy_time::Instant::now();
            while regs.cr().read().rstreq() {
                if start.elapsed() > Duration::from_millis(10) {
                    break;
                }
            }
        }
        // Note: If SCL or SDA is stuck low, it's a hardware issue
        // The driver will still initialize, but transfers will fail

        // Set bus mode and configure CR register
        let bus_mode = if config.frequency.0 <= 100_000 {
            BusMode::StandardMode
        } else {
            BusMode::FastMode
        };

        // Write CR register (like C SDK: CR = mode | SCLE | IUE)
        regs.cr().write(|w| {
            w.set_mode(bus_mode as u8);
            w.set_scle(true); // Enable SCL for master mode
            w.set_iue(true); // Enable I2C unit
        });

        // Disable all interrupts (polling mode)
        regs.ier().write(|_| {}); // IER = 0

        // Set reset cycle count for bus recovery (default 9 cycles)
        regs.rccr().write(|w| w.set_rstcyc(9));

        // Set timing
        regs.lcr().write(|w| {
            w.set_flv(lcr as u16);
            w.set_slv(0x1FF); // Standard mode lowest frequency
        });
        regs.wcr().write(|w| w.set_cnt(wcr as _));

        Self {
            _peri: peri,
            _mode: PhantomData,
        }
    }

    /// Compute Load Count Register (LCR) and Wait Count Register (WCR)
    /// Based on I2C clock frequency and target bus frequency
    /// Following C SDK HAL_I2C_Init() calculation
    fn compute_timing(i2c_clk: Hertz, target_freq: Hertz) -> (u32, u32) {
        // C SDK formula:
        // flv = ((i2c_clk + (target_freq / 2)) / target_freq - dnf - 7 + 1) / 2
        // where dnf (digital noise filter) = 0 for now
        let dnf = 0u32;
        let flv = ((i2c_clk.0 + (target_freq.0 / 2)) / target_freq.0 - dnf - 7 + 1) / 2;

        // LCR.FLV = flv (fast load value)
        // LCR.SLV = 0x1FF (standard load value, maximum for standard mode)
        let lcr = flv; // Only FLV field, SLV will be set separately

        // WCR.CNT calculation
        let cnt = flv / 2;
        let wcr = if cnt < 3 { 0 } else { cnt - 3 };

        (lcr, wcr)
    }
}

// Blocking transfer implementations
impl<'d, T: Instance> I2c<'d, T, Blocking> {
    /// Reset I2C bus to recover from stuck state
    ///
    /// This performs the bus recovery sequence:
    /// 1. Module reset (CR.UR)
    /// 2. If both SCL and SDA are high, send clock cycles (CR.RSTREQ)
    /// 3. Clear status flags
    ///
    /// Returns `Ok(())` if bus is recovered, `Err(Error::Bus)` if stuck
    pub fn reset_bus(&mut self) -> Result<(), Error> {
        let regs = T::regs();

        // Step 1: Module reset
        regs.cr().modify(|w| w.set_ur(true));
        crate::cortex_m_blocking_delay_us(100);
        regs.cr().modify(|w| w.set_ur(false));

        // Step 2: Check bus monitor
        let bmr = regs.bmr().read();

        if !bmr.scl() || !bmr.sda() {
            // One or both lines stuck low - hardware issue
            return Err(Error::Bus);
        }

        // Both lines high - send clock cycles to recover slaves
        regs.cr().modify(|w| w.set_rstreq(true));

        // Wait for RSTREQ to clear (bus recovery complete)
        let start = Instant::now();
        while regs.cr().read().rstreq() {
            if start.elapsed() > Duration::from_millis(10) {
                return Err(Error::Timeout);
            }
        }

        // Step 3: Clear all status flags
        regs.sr().write_value(crate::pac::i2c::regs::Sr(0xFFFFFFFF));

        Ok(())
    }

    /// Write data to I2C device (blocking)
    pub fn blocking_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.blocking_write_timeout(addr, bytes, Duration::from_millis(TIMEOUT_DEFAULT_MS))
    }

    /// Write data to I2C device with timeout (blocking)
    pub fn blocking_write_timeout(
        &mut self,
        addr: u8,
        bytes: &[u8],
        timeout: Duration,
    ) -> Result<(), Error> {
        let regs = T::regs();

        // Disable and re-enable I2C for clean state (following C SDK pattern)
        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        // Wait for bus idle before starting
        self.wait_bus_idle(timeout)?;

        // Clear status flags
        self.clear_flags();

        // Send START + Address (write)
        regs.dbr().write(|w| w.set_data(addr << 1));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        // Wait for TE=1 and check NACK
        self.wait_te_set(timeout)?;
        self.check_nack()?;

        // If no data to send (probe only), send STOP separately
        if bytes.is_empty() {
            // Send STOP condition (no TB bit, just STOP)
            regs.tcr().write(|w| {
                w.set_stop(true);
            });

            // Wait for STOP to complete (UB returns to 0)
            // Using a longer timeout as probe-only transfers seem slower
            let stop_timeout = Duration::from_millis(100);
            self.wait_bus_idle(stop_timeout)?;

            regs.cr().modify(|w| w.set_iue(false));
            return Ok(());
        }

        // Send data bytes
        for (i, byte) in bytes.iter().enumerate() {
            T::regs().sr().modify(|w| w.set_te(true));

            regs.dbr().write(|w| w.set_data(*byte));

            if i == bytes.len() - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }

            self.wait_te_set(timeout)?;
            self.check_nack()?;
        }

        self.wait_bus_idle(timeout)?;
        regs.cr().modify(|w| w.set_iue(false));

        Ok(())
    }

    /// Read data from I2C device (blocking)
    pub fn blocking_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, buffer, Duration::from_millis(TIMEOUT_DEFAULT_MS))
    }

    /// Read data from I2C device with timeout (blocking)
    pub fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        timeout: Duration,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();

        // Disable and re-enable I2C for clean state
        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        // Wait for bus idle
        self.wait_bus_idle(timeout)?;

        // Clear status flags
        self.clear_flags();

        // Send START + Address (read)
        // C SDK: DBR = (addr << 1) | 1; TCR = START | TB;
        regs.dbr().write(|w| w.set_data(((addr << 1) | 1) as u8));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        // Wait for transmit empty and check for NACK
        self.wait_te_set(timeout)?;
        self.check_nack()?;

        // Read data bytes
        let len = buffer.len();
        for (i, byte) in buffer.iter_mut().enumerate() {
            // Clear RF flag (write 1 to clear)
            T::regs().sr().modify(|w| w.set_rf(true));

            // C SDK: TCR = TB (or TB|STOP|NACK for last byte)
            // Last byte: send STOP + NACK
            if i == len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }

            self.wait_rf_set(timeout)?;
            *byte = regs.dbr().read().data();
        }

        // Wait for bus idle (STOP sent)
        self.wait_bus_idle(timeout)?;

        // Disable I2C after transfer
        regs.cr().modify(|w| w.set_iue(false));

        Ok(())
    }

    /// Write then read from I2C device (combined transaction)
    pub fn blocking_write_read(
        &mut self,
        addr: u8,
        write_bytes: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.blocking_write_read_timeout(
            addr,
            write_bytes,
            read_buffer,
            Duration::from_millis(TIMEOUT_DEFAULT_MS),
        )
    }

    /// Write then read from I2C device with timeout
    pub fn blocking_write_read_timeout(
        &mut self,
        addr: u8,
        write_bytes: &[u8],
        read_buffer: &mut [u8],
        timeout: Duration,
    ) -> Result<(), Error> {
        if write_bytes.is_empty() || read_buffer.is_empty() {
            return Err(Error::ZeroLength);
        }

        let regs = T::regs();

        // Disable and re-enable I2C for clean state
        regs.cr().modify(|w| w.set_iue(false));
        regs.cr().modify(|w| w.set_iue(true));

        // Wait for bus idle
        self.wait_bus_idle(timeout)?;
        self.clear_flags();

        // Write phase (with repeated START, no STOP)
        // C SDK: DBR = addr << 1; TCR = START | TB;
        regs.dbr().write(|w| w.set_data((addr << 1) as u8));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        self.wait_te_set(timeout)?;
        self.check_nack()?;

        // Send write data
        for byte in write_bytes.iter() {
            T::regs().sr().modify(|w| w.set_te(true));
            regs.dbr().write(|w| w.set_data(*byte));
            regs.tcr().write(|w| w.set_tb(true));
            self.wait_te_set(timeout)?;
            self.check_nack()?;
        }

        // Read phase (with repeated START)
        T::regs().sr().modify(|w| w.set_te(true));
        regs.dbr().write(|w| w.set_data(((addr << 1) | 1) as u8));
        regs.tcr().write(|w| {
            w.set_tb(true);
            w.set_start(true);
        });

        self.wait_te_set(timeout)?;
        self.check_nack()?;

        // Read data bytes
        let read_len = read_buffer.len();
        for (i, byte) in read_buffer.iter_mut().enumerate() {
            T::regs().sr().modify(|w| w.set_rf(true));

            if i == read_len - 1 {
                regs.tcr().write(|w| {
                    w.set_tb(true);
                    w.set_stop(true);
                    w.set_nack(true);
                });
            } else {
                regs.tcr().write(|w| w.set_tb(true));
            }

            self.wait_rf_set(timeout)?;
            *byte = regs.dbr().read().data();
        }

        // Wait for bus idle
        self.wait_bus_idle(timeout)?;

        // Disable I2C after transfer
        regs.cr().modify(|w| w.set_iue(false));

        Ok(())
    }
}

// Helper methods
impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    fn wait_bus_idle(&self, timeout: Duration) -> Result<(), Error> {
        // Wait for UB=0 (bus idle/not busy)
        let start = Instant::now();
        while T::regs().sr().read().ub() {
            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    fn wait_te_set(&self, timeout: Duration) -> Result<(), Error> {
        // Wait for TE=1 (transmit complete)
        // C SDK waits for TE to become RESET (0), but from debug we see TE becomes 1
        let start = Instant::now();
        while !T::regs().sr().read().te() {
            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    fn wait_rf_set(&self, timeout: Duration) -> Result<(), Error> {
        // Wait for RF=1 (receive data available)
        let start = Instant::now();
        while !T::regs().sr().read().rf() {
            if start.elapsed() > timeout {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    fn check_nack(&self) -> Result<(), Error> {
        if T::regs().sr().read().nack() {
            Err(Error::Nack)
        } else {
            Ok(())
        }
    }

    fn clear_flags(&self) {
        // Clear status flags by writing 1
        T::regs().sr().modify(|w| {
            w.set_te(true);
            w.set_nack(true);
            w.set_rf(true);
        });
    }
}

// Instance trait
pub(crate) trait SealedInstance {
    fn regs() -> crate::pac::i2c::I2c;
    fn configure_pins(scl: &AnyPin, sda: &AnyPin);
    fn scl_fsel() -> u8;
    fn sda_fsel() -> u8;
}

/// I2C instance trait
#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccEnableReset + RccGetFreq + 'static {
    /// Interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
}

// Pin traits - any GPIO pin can be used as I2C SCL/SDA due to flexible pinmux
pub(crate) trait SealedSclPin<T: Instance> {}
pub(crate) trait SealedSdaPin<T: Instance> {}

/// SCL pin trait - implemented for all GPIO pins
#[allow(private_bounds)]
pub trait SclPin<T: Instance>: GpioPin + SealedSclPin<T> {}

/// SDA pin trait - implemented for all GPIO pins
#[allow(private_bounds)]
pub trait SdaPin<T: Instance>: GpioPin + SealedSdaPin<T> {}

// Blanket implementation: any GPIO pin can be SCL/SDA for any I2C instance
impl<T: Instance, P: GpioPin> SealedSclPin<T> for P {}
impl<T: Instance, P: GpioPin> SclPin<T> for P {}

impl<T: Instance, P: GpioPin> SealedSdaPin<T> for P {}
impl<T: Instance, P: GpioPin> SdaPin<T> for P {}

// Instance implementation macro
macro_rules! impl_i2c {
    ($inst:ident, $irq:ident, $scl_pin_field:ident, $sda_pin_field:ident, $pinr_reg:ident, $scl_fsel:expr, $sda_fsel:expr) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> crate::pac::i2c::I2c {
                crate::pac::$inst
            }

            fn configure_pins(scl: &AnyPin, sda: &AnyPin) {
                use crate::gpio::SealedPin;
                // Configure pinmux: route I2C SCL/SDA to specified pins
                crate::pac::HPSYS_CFG.$pinr_reg().modify(|w| {
                    w.$scl_pin_field(scl.pin_bank() as u8);
                    w.$sda_pin_field(sda.pin_bank() as u8);
                });
            }

            fn scl_fsel() -> u8 {
                $scl_fsel
            }

            fn sda_fsel() -> u8 {
                $sda_fsel
            }
        }

        impl Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

// TODO: Move to _generated.rs and get FSEL values from pinmux_signals.yaml
// For now, using hardcoded FSEL values from pinmux.yaml
// Note: FSEL value depends on the specific pin!
// Most pins use FSEL=4 for I2C (PA39_I2C_UART, PA40_I2C_UART, etc.)
impl_i2c!(I2C1, I2C1, set_scl_pin, set_sda_pin, i2c1_pinr, 4, 4);
impl_i2c!(I2C2, I2C2, set_scl_pin, set_sda_pin, i2c2_pinr, 4, 4);
impl_i2c!(I2C3, I2C3, set_scl_pin, set_sda_pin, i2c3_pinr, 4, 4);
impl_i2c!(I2C4, I2C4, set_scl_pin, set_sda_pin, i2c4_pinr, 4, 4);

// Implement embedded-hal 0.2 traits
impl<'d, T: Instance> embedded_hal_02::blocking::i2c::Write for I2c<'d, T, Blocking> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(addr, bytes)
    }
}

impl<'d, T: Instance> embedded_hal_02::blocking::i2c::Read for I2c<'d, T, Blocking> {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(addr, buffer)
    }
}

impl<'d, T: Instance> embedded_hal_02::blocking::i2c::WriteRead for I2c<'d, T, Blocking> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(addr, bytes, buffer)
    }
}
