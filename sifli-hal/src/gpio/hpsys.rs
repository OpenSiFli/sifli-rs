use crate::gpio::{Drive, Level, Pull, SlewRate};
use crate::pac::hpsys_gpio::{regs, HpsysGpio};
use crate::pac::hpsys_pinmux::{vals, HpsysPinmux};

use super::AfType;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) struct HpsysPin {
    pub(crate) pin: u8,
}

impl HpsysPin {
    pub(crate) fn new(pin: u8) -> Self {
        Self { pin }
    }

    fn gpio(&self) -> HpsysGpio {
        crate::pac::HPSYS_GPIO
    }

    fn pinmux(&self) -> HpsysPinmux {
        crate::pac::HPSYS_PINMUX  
    }

    #[inline]
    fn bit(&self) -> u32 {
        1 << (self.pin % 32)
    }

    // Clear ISR and open drain flags
    pub fn clear_flags(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().isr0().write_value(regs::Isr0(self.bit()));
            self.gpio().iphcr0().write_value(regs::Iphcr0(self.bit()));
        } else {  
            self.gpio().isr1().write_value(regs::Isr1(self.bit()));
            self.gpio().iphcr1().write_value(regs::Iphcr1(self.bit()));
        }
    }

    pub fn disable_interrupt(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().iecr0().write_value(regs::Iecr0(self.bit()));
        } else {
            self.gpio().iecr1().write_value(regs::Iecr1(self.bit())); 
        }

        // WAIT_ISR_DISABLED
        crate::cortex_m_blocking_delay_us(1);
    }

    pub fn enable_interrupt(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().iesr0().write_value(regs::Iesr0(self.bit()));
        } else {
            self.gpio().iesr1().write_value(regs::Iesr1(self.bit()));
        }
    }

    pub fn set_pull(&mut self, pull: Pull) {
        let (pe, ps) = match pull {
            Pull::None => (false, vals::Ps::Down),
            Pull::Up => (true, vals::Ps::Up), 
            Pull::Down => (true, vals::Ps::Down),
        };

        match self.pin {
            0..=38 => {
                self.pinmux().pad_pa0_38(self.pin as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            39..=42 => {
                self.pinmux().pad_pa39_42((self.pin - 39) as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            43..=44 => {
                self.pinmux().pad_pa43_44((self.pin - 43) as _).modify(|w| {
                    w.set_pe(pe);
                    w.set_ps(ps);
                });
            },
            _ => unreachable!(),
        }
    }

    pub fn set_drive_strength(&mut self, strength: Drive) {
        let (ds1, ds0) = match strength {
            Drive::Drive0 => (false, false),
            Drive::Drive1 => (false, true),
            Drive::Drive2 => (true, false),
            Drive::Drive3 => (true, true),
        };
        match self.pin {
            0..=38 => {
                self.pinmux().pad_pa0_38(self.pin as _).modify(|w| {
                    w.set_ds0(ds0);
                    w.set_ds1(ds1);
                });
            },
            39..=42 => {
                let ds = match strength {
                    Drive::Drive0 => false,
                    Drive::Drive1 => true,
                    _ => {
                        warn!("PA39-42 can only be set to Drive0 or Drive1");
                        true
                    },
                };
                self.pinmux().pad_pa39_42((self.pin - 39) as _).modify(|w| {
                    w.set_ds(ds);
                });
            },
            43..=44 => {
                self.pinmux().pad_pa43_44((self.pin - 43) as _).modify(|w| {
                    w.set_ds0(ds0);
                    w.set_ds1(ds1);
                });
            },
            _ => unreachable!(),
        }
    }

    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        let sr = match slew_rate {
            SlewRate::Fast => vals::Sr::Fast,
            SlewRate::Slow => vals::Sr::Slow,  
        };
        match self.pin {
            0..=38 => {
                self.pinmux().pad_pa0_38(self.pin as _).modify(|w| {
                    w.set_sr(sr);
                });
            },
            39..=42 => {
                // TODO: should this be a panic?
                warn!("Cannot set slew rate on pad 39-42");
            }, 
            43..=44 => {
                self.pinmux().pad_pa43_44((self.pin - 43) as _).modify(|w| {
                    w.set_sr(sr);
                });
            },
            _ => unreachable!(),
        }
    }

    pub fn set_schmitt(&mut self, enable: bool) {
        let is = if enable {
            vals::Is::Schmitt
        } else {
            vals::Is::Cmos
        };
        match self.pin {
            0..=38 => {
                self.pinmux().pad_pa0_38(self.pin as _).modify(|w| {
                    w.set_is(is);
                });
            },
            39..=42 => {
                self.pinmux().pad_pa39_42((self.pin - 39) as _).modify(|w| {
                    w.set_is(is);
                });
            },
            43..=44 => {
                self.pinmux().pad_pa43_44((self.pin - 43) as _).modify(|w| {
                    w.set_is(is);
                });
            },
            _ => unreachable!(),
        }
    }

    pub fn set_as_input(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().doecr0().write_value(regs::Doecr0(self.bit()));
        } else {
            self.gpio().doecr1().write_value(regs::Doecr1(self.bit()));
        }
    }

    pub fn set_as_output(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().doesr0().write_value(regs::Doesr0(self.bit()));
        } else {
            self.gpio().doesr1().write_value(regs::Doesr1(self.bit()));
        }
    }

    pub fn set_as_output_od(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().iphsr0().write_value(regs::Iphsr0(self.bit())); 
            self.gpio().iplsr0().write_value(regs::Iplsr0(self.bit()));
        } else {
            self.gpio().iphsr1().write_value(regs::Iphsr1(self.bit()));
            self.gpio().iplsr1().write_value(regs::Iplsr1(self.bit()));
        }
    }

    pub fn set_as_analog(&mut self) {
        assert!(self.pin <= 38, "Pin {} is not an analog pin!", self.pin);
        self.pinmux().pad_pa0_38(self.pin as _).modify(|w| {
            w.set_pe(false);
            w.set_ie(false);
            w.set_fsel(0b1111);
        });

        // TODO: Set AonPE
    }

    pub fn is_set_as_output(&self) -> bool {
        let bit = if self.pin / 32 == 0 {
            self.gpio().doesr0().read().0
        } else {
            self.gpio().doesr1().read().0  
        };
        (bit & self.bit()) != 0
    }

    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    pub fn is_low(&self) -> bool {
        let bit = if self.pin / 32 == 0 {
            self.gpio().dir0().read().0
        } else {
            self.gpio().dir1().read().0
        };
        (bit & self.bit()) == 0
    }

    pub fn get_level(&self) -> Level {
        self.is_high().into()  
    }

    pub fn set_high(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().dosr0().write_value(regs::Dosr0(self.bit()));
        } else {
            self.gpio().dosr1().write_value(regs::Dosr1(self.bit()));
        }
    }

    pub fn set_low(&mut self) {
        if self.pin / 32 == 0 {
            self.gpio().docr0().write_value(regs::Docr0(self.bit()));
        } else {
            self.gpio().docr1().write_value(regs::Docr1(self.bit()));
        }
    }

    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    pub fn is_set_low(&self) -> bool {
        let bit = if self.pin / 32 == 0 {
            self.gpio().dor0().read().0
        } else {
            self.gpio().dor1().read().0
        };
        (bit & self.bit()) == 0
    }

    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    pub fn toggle(&mut self) {
        self.set_level(!self.get_output_level())
    }

    pub fn set_function(&mut self, fsel: u8, af_type: AfType) {
        self.set_pull(af_type.pull);
        unsafe {
            self.set_fsel_unchecked(fsel);
        }
        self.enable_interrupt();
    }

    pub unsafe fn set_fsel_unchecked(&mut self, fsel: u8) {
        match self.pin {
            0..=38 => {
                self.pinmux().pad_pa0_38(self.pin as _).modify(|w| w.set_fsel(fsel));
            },
            39..=42 => { 
                self.pinmux().pad_pa39_42((self.pin - 39) as _).modify(|w| w.set_fsel(fsel));
            },
            43..=44 => {
                self.pinmux().pad_pa43_44((self.pin - 43) as _).modify(|w| w.set_fsel(fsel)); 
            },
            _ => unreachable!(),
        }
    }
}
