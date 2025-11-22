//! LPAON (Low-Power Always-On) module.
//!
//! Provides helpers to configure LCPU start address and related wake control.

use crate::pac::lpsys_aon::LpsysAon as Regs;
use crate::{lcpu, Peripheral};
use core::marker::PhantomData;

#[allow(private_interfaces)]
pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

/// LPAON peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {}

#[allow(private_interfaces)]
impl SealedInstance for crate::peripherals::LPSYS_AON {
    fn regs() -> Regs {
        crate::pac::LPSYS_AON
    }
}

impl Instance for crate::peripherals::LPSYS_AON {}

/// LPAON driver.
///
/// Receives the LPSYS_AON peripheral as a parameter.
/// Always-on peripheral, no clock gating or init required.
pub struct LpAon<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> LpAon<'d, T> {
    /// Create a new LPAON driver instance.
    pub fn new(_peri: impl Peripheral<P = T> + 'd) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Read the first two entries `(SP, PC)` from the LCPU vector table in memory.
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - LCPU image has been loaded to `lcpu::LpsysRam::CODE_START`.
    /// - The first two entries form a valid stack pointer and reset handler.
    fn read_start_vector_from_mem() -> (u32, u32) {
        let vector_addr = lcpu::LpsysRam::CODE_START as *const u32;

        unsafe {
            // vector[0] (0x20400000): initial stack pointer (SP)
            // vector[1] (0x20400004): Reset_Handler address (PC)
            let sp = core::ptr::read_volatile(vector_addr);
            let pc = core::ptr::read_volatile(vector_addr.add(1));
            (sp, pc)
        }
    }

    /// Write the given `(sp, pc)` into LPSYS_AON SPR/PCR registers.
    fn write_start_vector_to_regs(sp: u32, pc: u32) {
        let r = T::regs();
        // On LCPU start/wake, hardware loads SP and PC from these registers.
        r.spr().write(|w| w.set_sp(sp));
        r.pcr().write(|w| w.set_pc(pc));
    }

    /// Configure LCPU start address: read `(SP, PC)` from the LCPU vector table and write into LPAON registers.
    pub fn configure_lcpu_start(&self) {
        let (sp, pc) = Self::read_start_vector_from_mem();
        Self::write_start_vector_to_regs(sp, pc);
    }

    /// Read the currently configured LCPU start vector `(SP, PC)` from LPAON registers.
    ///
    /// ```no_run
    /// use sifli_hal::lpaon::LpAon;
    /// use sifli_hal::peripherals;
    ///
    /// let p = sifli_hal::init(Default::default());
    /// let lpaon = LpAon::new(p.LPSYS_AON);
    /// let (sp, pc) = lpaon.read_start_vector();
    /// ```
    pub fn read_start_vector(&self) -> (u32, u32) {
        let r = T::regs();
        let spr = r.spr().read();
        let pcr = r.pcr().read();
        (spr.sp(), pcr.pc())
    }

    /// Read the CPUWAIT flag from `LPSYS_AON.PMR`.
    pub fn cpuwait(&self) -> bool {
        T::regs().pmr().read().cpuwait()
    }

    /// Set/clear the CPUWAIT flag in `LPSYS_AON.PMR`.
    pub fn set_cpuwait(&self, enable: bool) {
        T::regs().pmr().modify(|w| w.set_cpuwait(enable));
    }

    /// Read the SLEEP_STATUS flag from `LPSYS_AON.SLP_CTRL`.
    pub fn sleep_status(&self) -> bool {
        T::regs().slp_ctrl().read().sleep_status()
    }

    /// Set/clear the WKUP_REQ flag in `LPSYS_AON.SLP_CTRL` to request a wakeup.
    pub fn set_wkup_req(&self, enable: bool) {
        T::regs().slp_ctrl().modify(|w| w.set_wkup_req(enable));
    }
}
