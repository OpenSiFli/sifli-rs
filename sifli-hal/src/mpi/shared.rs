use core::arch::asm;
use core::cmp::min;
use core::hint::spin_loop;

use super::{Regs, SealedInstance};

pub(super) const FIFO_SIZE_BYTES: usize = 64;
pub(super) const MAX_DLEN_BYTES: usize = 0x000f_ffff + 1;
pub(super) const MAX_DUMMY_CYCLES: u8 = 31;
pub(super) const STATUS_MATCH_TIMEOUT_POLLS: u32 = 2_000_000;
pub(super) const NOR_FLASH_MAX_3B_CAPACITY_BYTES: usize = 16 * 1024 * 1024;
pub(super) const DEFAULT_DMA_THRESHOLD_BYTES: usize = 256;
pub(super) const CODE_BUS_FLASH_START: usize = 0x1000_0000;
pub(super) const CODE_BUS_FLASH_END: usize = 0x2000_0000;
pub(super) const MPI1_CODE_BUS_BASE: usize = CODE_BUS_FLASH_START;
pub(super) const MPI1_CODE_BUS_END: usize = 0x1200_0000;
pub(super) const MPI2_CODE_BUS_BASE: usize = MPI1_CODE_BUS_END;
pub(super) const MPI2_CODE_BUS_END: usize = CODE_BUS_FLASH_END;
pub(super) const ERASE_BLOCK_32K_BYTES: u32 = 32 * 1024;
pub(super) const ERASE_BLOCK_64K_BYTES: u32 = 64 * 1024;

pub(super) const NOR_FLASH_WIP_MASK: u32 = 0x01;

#[inline(always)]
pub(super) fn plan_next_erase_step(
    addr: u32,
    to: u32,
    erase_gran: u32,
    base_opcode: u8,
    block32_opcode: Option<u8>,
    block64_opcode: Option<u8>,
) -> (u8, u32) {
    let remaining = to - addr;

    if remaining >= ERASE_BLOCK_64K_BYTES
        && (addr & (ERASE_BLOCK_64K_BYTES - 1)) == 0
        && (ERASE_BLOCK_64K_BYTES % erase_gran) == 0
    {
        if let Some(cmd) = block64_opcode {
            return (cmd, ERASE_BLOCK_64K_BYTES);
        }
    }
    if remaining >= ERASE_BLOCK_32K_BYTES
        && (addr & (ERASE_BLOCK_32K_BYTES - 1)) == 0
        && (ERASE_BLOCK_32K_BYTES % erase_gran) == 0
    {
        if let Some(cmd) = block32_opcode {
            return (cmd, ERASE_BLOCK_32K_BYTES);
        }
    }

    (base_opcode, erase_gran)
}

#[inline(always)]
pub(super) fn running_from_instance_code_bus_flash<T: SealedInstance>() -> bool {
    let pc = running_from_instance_code_bus_flash::<T> as *const () as usize;
    (T::code_bus_base()..T::code_bus_end()).contains(&pc)
}

#[inline(always)]
pub(super) fn instance_code_bus_window_size<T: SealedInstance>() -> usize {
    T::code_bus_end() - T::code_bus_base()
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_irq_save_disable() -> u32 {
    let primask: u32;
    // SAFETY: This only reads/writes CPU PRIMASK, no memory access side effects.
    unsafe {
        asm!(
            "mrs {primask}, PRIMASK",
            primask = out(reg) primask,
            options(nomem, nostack, preserves_flags)
        );
        asm!("cpsid i", options(nomem, nostack, preserves_flags));
    }
    primask
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_irq_restore(saved_primask: u32) {
    // Ensure all MPI register writes are complete before returning to XIP
    // SAFETY: These are standard ARM barrier instructions with no side effects.
    unsafe {
        asm!("dsb sy", options(nomem, nostack, preserves_flags));
        asm!("isb sy", options(nomem, nostack, preserves_flags));
    }
    // Restore PRIMASK to previous state
    unsafe {
        asm!(
            "msr PRIMASK, {primask}",
            primask = in(reg) saved_primask,
            options(nomem, nostack, preserves_flags)
        );
    }
}

// ============================================================================
// RAM-resident functions for XIP-safe MPI operations
// ============================================================================
//
// These functions are placed in RAM (.data section) so they can execute while
// the flash is being accessed via manual MPI commands. This is critical for
// XIP scenarios where code runs from the same flash being programmed.
//
// The functions are standalone (not methods) to ensure the entire call doesn't
// involve any flash code fetches.

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_wait_not_busy(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if !regs.sr().read().busy() {
            return true;
        }
        spin_loop();
    }
    false
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_wait_tcf(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if regs.sr().read().tcf() {
            regs.scr().write(|w| w.set_tcfc(true));
            return true;
        }
        spin_loop();
    }
    false
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_wait_smf(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if regs.sr().read().smf() {
            regs.scr().write(|w| {
                w.set_smfc(true);
                w.set_tcfc(true);
            });
            return true;
        }
        spin_loop();
    }
    false
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_configure_ccr1(regs: Regs, fmode: bool, dmode: u8, address_mode: u8, address_size: u8) {
    regs.ccr1().modify(|w| {
        w.set_fmode(fmode);
        w.set_dmode(dmode);
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(address_size);
        w.set_admode(address_mode);
        w.set_imode(1);
    });
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_clear_status_flags(regs: Regs) {
    regs.scr().write(|w| {
        w.set_smfc(true);
        w.set_tcfc(true);
    });
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_issue_cmd1(regs: Regs, addr: u32, cmd: u8) {
    regs.ar1().write(|w| w.set_addr(addr));
    regs.cmdr1().write(|w| w.set_cmd(cmd));
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_configure_wip_status_match(regs: Regs) {
    regs.smr().write(|w| w.set_status(0));
    regs.smkr().write(|w| w.set_mask(NOR_FLASH_WIP_MASK));
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_configure_simple_cmd(regs: Regs) {
    ram_configure_ccr1(regs, false, 0, 0, 0);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_configure_status_read(regs: Regs) {
    ram_configure_ccr1(regs, false, 1, 0, 0);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_configure_write_with_addr(regs: Regs, addr_size: u8) {
    ram_configure_ccr1(regs, true, 1, 1, addr_size);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_configure_addr_only(regs: Regs, addr_size: u8) {
    ram_configure_ccr1(regs, false, 0, 1, addr_size);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_configure_cmd2_status_poll(regs: Regs, read_status_cmd: u8) {
    regs.ccr2().modify(|w| {
        w.set_fmode(false);
        w.set_dmode(1);
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(0);
        w.set_admode(0);
        w.set_imode(1);
    });
    regs.dlr2().write(|w| w.set_dlen(0));
    regs.cmdr2().write(|w| w.set_cmd(read_status_cmd));
    ram_configure_wip_status_match(regs);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_begin_cmd2_status_poll(regs: Regs) {
    regs.cr().modify(|w| {
        w.set_cmd2e(true);
        w.set_sme2(true);
    });
    ram_clear_status_flags(regs);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_end_cmd2_status_poll(regs: Regs) {
    regs.cr().modify(|w| {
        w.set_cmd2e(false);
        w.set_sme2(false);
    });
    ram_clear_status_flags(regs);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_run_cmd2_status_poll(
    regs: Regs,
    read_status_cmd: u8,
    cmd_addr: u32,
    cmd: u8,
    max_polls: u32,
) -> bool {
    ram_configure_cmd2_status_poll(regs, read_status_cmd);
    ram_begin_cmd2_status_poll(regs);
    ram_issue_cmd1(regs, cmd_addr, cmd);
    let ok = ram_wait_smf(regs, max_polls);
    ram_end_cmd2_status_poll(regs);
    ok
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_pack_le_word(chunk: &[u8]) -> u32 {
    let mut word = [0u8; 4];
    word[..chunk.len()].copy_from_slice(chunk);
    u32::from_le_bytes(word)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_read_data(regs: Regs, cmd: u8, dlen: u32, max_polls: u32) -> Result<u32, ()> {
    regs.fifocr().modify(|w| w.set_rxclr(true));
    ram_configure_status_read(regs);
    regs.dlr1().write(|w| w.set_dlen(dlen));

    regs.scr().write(|w| w.set_tcfc(true));
    ram_issue_cmd1(regs, 0, cmd);

    if !ram_wait_tcf(regs, max_polls) {
        return Err(());
    }

    Ok(regs.dr().read().data())
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_issue_simple_cmd(regs: Regs, cmd: u8, max_polls: u32) -> bool {
    ram_configure_simple_cmd(regs);
    regs.scr().write(|w| w.set_tcfc(true));
    ram_issue_cmd1(regs, 0, cmd);
    ram_wait_tcf(regs, max_polls)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_read_jedec_id(regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()> {
    // Wait for MPI to be idle before issuing manual command (critical for XIP)
    if !ram_wait_not_busy(regs, max_polls) {
        return Err(());
    }

    // 3 bytes (dlen = len - 1)
    ram_read_data(regs, cmd, 2, max_polls).map(|data| data & 0x00ff_ffff)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_read_status(regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()> {
    // 1 byte.
    ram_read_data(regs, cmd, 0, max_polls).map(|data| (data & 0xff) as u8)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
#[allow(clippy::too_many_arguments)]
pub(super) fn ram_program_chunk(
    regs: Regs,
    wren_cmd: u8,
    program_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    data: &[u8],
    max_polls: u32,
) -> bool {
    if data.is_empty() || data.len() > FIFO_SIZE_BYTES {
        return false;
    }

    if !ram_issue_simple_cmd(regs, wren_cmd, max_polls) {
        return false;
    }

    regs.fifocr().modify(|w| w.set_txclr(true));

    let mut idx = 0;
    while idx < data.len() {
        let take = min(4, data.len() - idx);
        regs.dr()
            .write(|w| w.set_data(ram_pack_le_word(&data[idx..idx + take])));
        idx += take;
    }

    ram_configure_write_with_addr(regs, addr_size);
    regs.dlr1().write(|w| w.set_dlen((data.len() - 1) as u32));
    ram_run_cmd2_status_poll(regs, read_status_cmd, addr, program_cmd, max_polls)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_erase_sector(
    regs: Regs,
    wren_cmd: u8,
    erase_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    max_polls: u32,
) -> bool {
    if !ram_issue_simple_cmd(regs, wren_cmd, max_polls) {
        return false;
    }

    ram_configure_addr_only(regs, addr_size);
    ram_run_cmd2_status_poll(regs, read_status_cmd, addr, erase_cmd, max_polls)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_erase_chip(
    regs: Regs,
    wren_cmd: u8,
    chip_erase_cmd: u8,
    read_status_cmd: u8,
    max_polls: u32,
) -> bool {
    if !ram_wait_not_busy(regs, max_polls) {
        return false;
    }

    if !ram_issue_simple_cmd(regs, wren_cmd, max_polls) {
        return false;
    }

    ram_configure_simple_cmd(regs);
    ram_run_cmd2_status_poll(regs, read_status_cmd, 0, chip_erase_cmd, max_polls)
}

macro_rules! define_ram_irq_wrappers {
    (
        $(
            $(#[$attr:meta])*
            fn $name:ident($($arg:ident: $ty:ty),* $(,)?) -> $ret:ty
                = $inner:ident($($call:expr),* $(,)?);
        )+
    ) => {
        $(
            $(#[$attr])*
            #[inline(never)]
            #[link_section = ".data.ramfunc"]
            pub(super) fn $name($($arg: $ty),*) -> $ret {
                let primask = ram_irq_save_disable();
                let result = $inner($($call),*);
                ram_irq_restore(primask);
                result
            }
        )+
    };
}

define_ram_irq_wrappers! {
    fn ram_wrapper_read_jedec_id(regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()>
        = ram_read_jedec_id(regs, cmd, max_polls);
    fn ram_wrapper_read_status(regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()>
        = ram_read_status(regs, cmd, max_polls);
    #[allow(clippy::too_many_arguments)]
    fn ram_wrapper_program_chunk(
        regs: Regs,
        wren_cmd: u8,
        program_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
        data: &[u8],
        max_polls: u32,
    ) -> bool = ram_program_chunk(
        regs,
        wren_cmd,
        program_cmd,
        read_status_cmd,
        addr,
        addr_size,
        data,
        max_polls,
    );
    fn ram_wrapper_erase_sector(
        regs: Regs,
        wren_cmd: u8,
        erase_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
        max_polls: u32,
    ) -> bool = ram_erase_sector(
        regs,
        wren_cmd,
        erase_cmd,
        read_status_cmd,
        addr,
        addr_size,
        max_polls,
    );
    fn ram_wrapper_erase_chip(
        regs: Regs,
        wren_cmd: u8,
        chip_erase_cmd: u8,
        read_status_cmd: u8,
        max_polls: u32,
    ) -> bool = ram_erase_chip(regs, wren_cmd, chip_erase_cmd, read_status_cmd, max_polls);
    fn ram_wrapper_issue_simple_cmd(regs: Regs, cmd: u8, max_polls: u32) -> bool
        = ram_issue_simple_cmd(regs, cmd, max_polls);
    fn ram_wrapper_wait_ready(regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool
        = ram_wait_ready_sme1(regs, read_status_cmd, max_polls);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_wait_ready_sme1(regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool {
    ram_configure_status_read(regs);
    regs.dlr1().write(|w| w.set_dlen(0));

    ram_configure_wip_status_match(regs);

    regs.cr().modify(|w| {
        w.set_cmd2e(false);
        w.set_sme2(false);
        w.set_sme1(true);
    });

    ram_clear_status_flags(regs);

    ram_issue_cmd1(regs, 0, read_status_cmd);

    let ok = ram_wait_smf(regs, max_polls);

    regs.cr().modify(|w| w.set_sme1(false));
    ram_clear_status_flags(regs);

    ok
}
