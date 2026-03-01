#![allow(dead_code)]

use super::{Regs, SealedInstance};

pub(crate) const CODE_BUS_FLASH_START: usize = super::shared::CODE_BUS_FLASH_START;
pub(crate) const CODE_BUS_FLASH_END: usize = super::shared::CODE_BUS_FLASH_END;

#[inline(always)]
pub(crate) fn running_from_same_instance_xip<T: SealedInstance>() -> bool {
    super::shared::running_from_instance_code_bus_flash::<T>()
}

#[inline(always)]
pub(crate) fn instance_window_size<T: SealedInstance>() -> usize {
    super::shared::instance_code_bus_window_size::<T>()
}

#[inline(always)]
pub(crate) fn irq_save_disable() -> u32 {
    super::shared::ram_irq_save_disable()
}

#[inline(always)]
pub(crate) fn irq_restore(saved_primask: u32) {
    super::shared::ram_irq_restore(saved_primask)
}

#[inline(always)]
pub(crate) fn wait_not_busy(regs: Regs, max_polls: u32) -> bool {
    super::shared::ram_wait_not_busy(regs, max_polls)
}

#[inline(always)]
pub(crate) fn wait_transfer_complete(regs: Regs, max_polls: u32) -> bool {
    super::shared::ram_wait_tcf(regs, max_polls)
}

#[inline(always)]
pub(crate) fn issue_simple_cmd(regs: Regs, cmd: u8, max_polls: u32, xip_safe: bool) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_issue_simple_cmd(regs, cmd, max_polls)
    } else {
        super::shared::ram_issue_simple_cmd(regs, cmd, max_polls)
    }
}

#[inline(always)]
pub(crate) fn wait_ready(regs: Regs, read_status_cmd: u8, max_polls: u32, xip_safe: bool) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_wait_ready(regs, read_status_cmd, max_polls)
    } else {
        super::shared::ram_wait_ready_sme1(regs, read_status_cmd, max_polls)
    }
}

#[inline(always)]
pub(crate) fn read_status(regs: Regs, cmd: u8, max_polls: u32, xip_safe: bool) -> Result<u8, ()> {
    if xip_safe {
        super::shared::ram_wrapper_read_status(regs, cmd, max_polls)
    } else {
        super::shared::ram_read_status(regs, cmd, max_polls)
    }
}

#[inline(always)]
pub(crate) fn read_jedec_id(
    regs: Regs,
    cmd: u8,
    max_polls: u32,
    xip_safe: bool,
) -> Result<u32, ()> {
    if xip_safe {
        super::shared::ram_wrapper_read_jedec_id(regs, cmd, max_polls)
    } else {
        super::shared::ram_read_jedec_id(regs, cmd, max_polls)
    }
}

#[inline(always)]
pub(crate) fn read_command_stream(
    regs: Regs,
    cmd: u8,
    addr: Option<u32>,
    addr_size: u8,
    dummy_cycles: u8,
    out: &mut [u8],
    max_polls: u32,
    xip_safe: bool,
) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_read_command_stream(
            regs,
            cmd,
            addr,
            addr_size,
            dummy_cycles,
            out,
            max_polls,
        )
    } else {
        super::shared::ram_read_command_stream(
            regs,
            cmd,
            addr,
            addr_size,
            dummy_cycles,
            out,
            max_polls,
        )
    }
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) fn program_chunk(
    regs: Regs,
    wren_cmd: u8,
    program_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    data: &[u8],
    max_polls: u32,
    xip_safe: bool,
) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_program_chunk(
            regs,
            wren_cmd,
            program_cmd,
            read_status_cmd,
            addr,
            addr_size,
            data,
            max_polls,
        )
    } else {
        super::shared::ram_program_chunk(
            regs,
            wren_cmd,
            program_cmd,
            read_status_cmd,
            addr,
            addr_size,
            data,
            max_polls,
        )
    }
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) fn erase_sector(
    regs: Regs,
    wren_cmd: u8,
    erase_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    max_polls: u32,
    xip_safe: bool,
) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_erase_sector(
            regs,
            wren_cmd,
            erase_cmd,
            read_status_cmd,
            addr,
            addr_size,
            max_polls,
        )
    } else {
        super::shared::ram_erase_sector(
            regs,
            wren_cmd,
            erase_cmd,
            read_status_cmd,
            addr,
            addr_size,
            max_polls,
        )
    }
}

#[inline(always)]
pub(crate) fn erase_chip(
    regs: Regs,
    wren_cmd: u8,
    chip_erase_cmd: u8,
    read_status_cmd: u8,
    max_polls: u32,
    xip_safe: bool,
) -> bool {
    if xip_safe {
        super::shared::ram_wrapper_erase_chip(
            regs,
            wren_cmd,
            chip_erase_cmd,
            read_status_cmd,
            max_polls,
        )
    } else {
        super::shared::ram_erase_chip(regs, wren_cmd, chip_erase_cmd, read_status_cmd, max_polls)
    }
}
