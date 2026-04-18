use core::arch::asm;
use core::cmp::min;
use core::hint::spin_loop;

use super::{Regs, SealedInstance};
use crate::pac::mpi::regs::{Ar1, Ccr1, Ccr2, Cmdr1, Cmdr2, Cr, Dlr1, Dlr2, Dr, Fifocr, Scr, Smkr, Smr};

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
        // Drain any in-flight transactions and flush the instruction prefetch
        // buffer before we start touching MPI registers. Without this, a
        // speculative fetch from MPI XIP that started before CPSID can land
        // mid-way through our manual command sequence and wedge the controller.
        asm!("dsb sy", options(nomem, nostack, preserves_flags));
        asm!("isb sy", options(nomem, nostack, preserves_flags));
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

// All `.data.ramfunc` helpers below avoid `Reg::modify(|w| ...)` and
// `Reg::write(|w| ...)`. The compiler is free to keep those closures in
// `.text` (flash) — when it does, calling them from a RAM-resident MPI
// code path requires an XIP fetch, defeating the whole point and risking
// fetching stale/garbage cache lines while a manual command is in flight.
// Instead, every register update is written as a `read()`/`Default::default()`
// + setter chain + `write_value()`, all of which inline cleanly into the
// caller and stay in `.data.ramfunc`.

#[inline(always)]
fn write_scr_clear(regs: Regs, smfc: bool, tcfc: bool) {
    let mut v = Scr::default();
    v.set_smfc(smfc);
    v.set_tcfc(tcfc);
    regs.scr().write_value(v);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_wait_tcf(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if regs.sr().read().tcf() {
            write_scr_clear(regs, false, true);
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
            write_scr_clear(regs, true, true);
            return true;
        }
        spin_loop();
    }
    false
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_configure_ccr1(regs: Regs, fmode: bool, dmode: u8, address_mode: u8, address_size: u8) {
    let mut v: Ccr1 = regs.ccr1().read();
    v.set_fmode(fmode);
    v.set_dmode(dmode);
    v.set_dcyc(0);
    v.set_absize(0);
    v.set_abmode(0);
    v.set_adsize(address_size);
    v.set_admode(address_mode);
    v.set_imode(1);
    regs.ccr1().write_value(v);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_configure_cmd1_read_stream(
    regs: Regs,
    address_mode: u8,
    address_size: u8,
    dummy_cycles: u8,
) {
    let mut v: Ccr1 = regs.ccr1().read();
    v.set_fmode(false);
    v.set_dmode(1);
    v.set_dcyc(dummy_cycles);
    v.set_absize(0);
    v.set_abmode(0);
    v.set_adsize(address_size);
    v.set_admode(address_mode);
    v.set_imode(1);
    regs.ccr1().write_value(v);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_clear_status_flags(regs: Regs) {
    write_scr_clear(regs, true, true);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_issue_cmd1(regs: Regs, addr: u32, cmd: u8) {
    let mut a = Ar1::default();
    a.set_addr(addr);
    regs.ar1().write_value(a);
    let mut c = Cmdr1::default();
    c.set_cmd(cmd);
    regs.cmdr1().write_value(c);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_configure_wip_status_match(regs: Regs) {
    let mut s = Smr::default();
    s.set_status(0);
    regs.smr().write_value(s);
    let mut m = Smkr::default();
    m.set_mask(NOR_FLASH_WIP_MASK);
    regs.smkr().write_value(m);
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
    let mut v: Ccr2 = regs.ccr2().read();
    v.set_fmode(false);
    v.set_dmode(1);
    v.set_dcyc(0);
    v.set_absize(0);
    v.set_abmode(0);
    v.set_adsize(0);
    v.set_admode(0);
    v.set_imode(1);
    regs.ccr2().write_value(v);

    let mut d = Dlr2::default();
    d.set_dlen(0);
    regs.dlr2().write_value(d);

    let mut c = Cmdr2::default();
    c.set_cmd(read_status_cmd);
    regs.cmdr2().write_value(c);

    ram_configure_wip_status_match(regs);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_begin_cmd2_status_poll(regs: Regs) {
    let mut v: Cr = regs.cr().read();
    v.set_cmd2e(true);
    v.set_sme2(true);
    regs.cr().write_value(v);
    ram_clear_status_flags(regs);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_end_cmd2_status_poll(regs: Regs) {
    let mut v: Cr = regs.cr().read();
    v.set_cmd2e(false);
    v.set_sme2(false);
    regs.cr().write_value(v);
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

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_fifo_clear(regs: Regs, rx: bool, tx: bool) {
    let mut v: Fifocr = regs.fifocr().read();
    if rx {
        v.set_rxclr(true);
    }
    if tx {
        v.set_txclr(true);
    }
    regs.fifocr().write_value(v);
}

#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_set_dlr1(regs: Regs, dlen: u32) {
    let mut d = Dlr1::default();
    d.set_dlen(dlen);
    regs.dlr1().write_value(d);
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_read_data(regs: Regs, cmd: u8, dlen: u32, max_polls: u32) -> Result<u32, ()> {
    ram_fifo_clear(regs, true, false);
    ram_configure_status_read(regs);
    ram_set_dlr1(regs, dlen);

    write_scr_clear(regs, false, true);
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
    write_scr_clear(regs, false, true);
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
    if !ram_wait_not_busy(regs, max_polls) {
        return Err(());
    }

    // 1 byte.
    ram_read_data(regs, cmd, 0, max_polls).map(|data| (data & 0xff) as u8)
}

#[inline(never)]
#[link_section = ".data.ramfunc"]
pub(super) fn ram_read_command_stream(
    regs: Regs,
    cmd: u8,
    mut addr: Option<u32>,
    addr_size: u8,
    dummy_cycles: u8,
    out: &mut [u8],
    max_polls: u32,
) -> bool {
    if out.is_empty() {
        return true;
    }

    let address_mode = if addr.is_some() { 1 } else { 0 };
    let address_size = if addr.is_some() { addr_size } else { 0 };
    let mut done = 0usize;

    while done < out.len() {
        let step = min(FIFO_SIZE_BYTES, out.len() - done);

        if !ram_wait_not_busy(regs, max_polls) {
            return false;
        }

        ram_fifo_clear(regs, true, false);
        ram_configure_cmd1_read_stream(regs, address_mode, address_size, dummy_cycles);
        ram_set_dlr1(regs, (step - 1) as u32);

        write_scr_clear(regs, false, true);
        ram_issue_cmd1(regs, addr.unwrap_or(0), cmd);

        if !ram_wait_tcf(regs, max_polls) {
            return false;
        }

        let mut idx = 0usize;
        while idx < step {
            let word = regs.dr().read().data().to_le_bytes();
            let take = min(4, step - idx);
            out[done + idx..done + idx + take].copy_from_slice(&word[..take]);
            idx += take;
        }

        done += step;
        if let Some(a) = &mut addr {
            *a = a.saturating_add(step as u32);
        }
    }

    true
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

    ram_fifo_clear(regs, false, true);

    // The MPI controller's TX FIFO is word-oriented and DLR1 must match the
    // number of bytes pushed, otherwise the engine never finishes draining the
    // FIFO and the WIP poll hangs forever. Round the transmission up to a
    // 4-byte boundary, padding with 0xFF (a no-op for NOR flash because every
    // unprogrammed bit is already 1, and AND-ing 1 with the current bit keeps
    // it unchanged). Do not extend past a 256-byte page boundary — the SPI
    // page-program command wraps at page boundaries, which would corrupt
    // earlier bytes in the same page.
    const PAGE: usize = 256;
    let page_remaining = PAGE - ((addr as usize) & (PAGE - 1));
    let aligned_len = ((data.len() + 3) & !3).min(page_remaining);
    let send_len = if aligned_len >= data.len() {
        aligned_len
    } else {
        data.len()
    };

    // Build each word manually — `copy_from_slice` may be emitted as an
    // out-of-line function in `.text` (flash), which would cause a code-bus
    // fetch from MPI2 mid-way through our XIP-safe section and deadlock.
    let mut idx = 0;
    while idx < send_len {
        let b0 = if idx     < data.len() { data[idx]     } else { 0xFF };
        let b1 = if idx + 1 < data.len() { data[idx + 1] } else { 0xFF };
        let b2 = if idx + 2 < data.len() { data[idx + 2] } else { 0xFF };
        let b3 = if idx + 3 < data.len() { data[idx + 3] } else { 0xFF };
        let word = (b0 as u32)
            | ((b1 as u32) << 8)
            | ((b2 as u32) << 16)
            | ((b3 as u32) << 24);
        let mut d = Dr::default();
        d.set_data(word);
        regs.dr().write_value(d);
        idx += 4;
    }

    ram_configure_write_with_addr(regs, addr_size);
    ram_set_dlr1(regs, (send_len - 1) as u32);
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
    fn ram_wrapper_read_command_stream(
        regs: Regs,
        cmd: u8,
        addr: Option<u32>,
        addr_size: u8,
        dummy_cycles: u8,
        out: &mut [u8],
        max_polls: u32,
    ) -> bool = ram_read_command_stream(regs, cmd, addr, addr_size, dummy_cycles, out, max_polls);
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
    ram_set_dlr1(regs, 0);

    ram_configure_wip_status_match(regs);

    let mut v: Cr = regs.cr().read();
    v.set_cmd2e(false);
    v.set_sme2(false);
    v.set_sme1(true);
    regs.cr().write_value(v);

    ram_clear_status_flags(regs);

    ram_issue_cmd1(regs, 0, read_status_cmd);

    let ok = ram_wait_smf(regs, max_polls);

    let mut v: Cr = regs.cr().read();
    v.set_sme1(false);
    regs.cr().write_value(v);
    ram_clear_status_flags(regs);

    ok
}
