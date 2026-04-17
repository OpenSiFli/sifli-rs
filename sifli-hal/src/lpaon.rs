//! LPAON (Low-Power Always-On) helpers.
//!
//! System-level always-on registers for LCPU start address and wake control.
//! Accessed directly via PAC (no peripheral singleton needed).

use crate::pac;

fn regs() -> pac::lpsys_aon::LpsysAon {
    pac::LPSYS_AON
}

/// Configure LCPU start address: write `(SP, PC)` into LPAON SPR/PCR registers.
///
/// On LCPU start/wake, hardware loads SP and PC from these registers.
pub(crate) fn configure_lcpu_start(sp: u32, pc: u32) {
    let r = regs();
    r.spr().write(|w| w.set_sp(sp));
    r.pcr().write(|w| w.set_pc(pc));
}

/// Read the currently configured LCPU start vector `(SP, PC)` from LPAON registers.
#[allow(dead_code)]
pub(crate) fn read_start_vector() -> (u32, u32) {
    let r = regs();
    let spr = r.spr().read();
    let pcr = r.pcr().read();
    (spr.sp(), pcr.pc())
}

/// Read the CPUWAIT flag from `LPSYS_AON.PMR`.
pub(crate) fn cpuwait() -> bool {
    regs().pmr().read().cpuwait()
}

/// Set/clear the CPUWAIT flag in `LPSYS_AON.PMR`.
pub(crate) fn set_cpuwait(enable: bool) {
    regs().pmr().modify(|w| w.set_cpuwait(enable));
}

/// Read the SLEEP_STATUS flag from `LPSYS_AON.SLP_CTRL`.
pub(crate) fn sleep_status() -> bool {
    regs().slp_ctrl().read().sleep_status()
}

/// Set/clear the WKUP_REQ flag in `LPSYS_AON.SLP_CTRL` to request a wakeup.
pub(crate) fn set_wkup_req(enable: bool) {
    regs().slp_ctrl().modify(|w| w.set_wkup_req(enable));
}

/// Read the BLE sleep target duration (LP clock cycles).
///
/// Non-zero value indicates `_rwip_sleep()` has scheduled a sleep period.
#[allow(dead_code)]
pub fn sleep_target() -> u32 {
    regs().target().read().sleep_target()
}

/// Read the actual elapsed sleep duration (LP clock cycles).
#[allow(dead_code)]
pub fn sleep_actual() -> u32 {
    regs().actual().read().sleep_cnt()
}
