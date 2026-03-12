#![allow(dead_code)]

use super::{Regs, SealedInstance};

pub(crate) const CODE_BUS_FLASH_START: usize = super::shared::CODE_BUS_FLASH_START;
pub(crate) const CODE_BUS_FLASH_END: usize = super::shared::CODE_BUS_FLASH_END;

#[derive(Clone, Copy)]
pub(crate) struct Exec {
    regs: Regs,
    max_polls: u32,
    xip_safe: bool,
}

impl Exec {
    #[inline(always)]
    pub(crate) const fn new(regs: Regs, max_polls: u32, xip_safe: bool) -> Self {
        Self {
            regs,
            max_polls,
            xip_safe,
        }
    }

    #[inline(always)]
    pub(crate) fn issue_simple_cmd(self, cmd: u8) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_issue_simple_cmd(self.regs, cmd, self.max_polls)
        } else {
            super::shared::ram_issue_simple_cmd(self.regs, cmd, self.max_polls)
        }
    }

    #[inline(always)]
    pub(crate) fn wait_ready(self, read_status_cmd: u8) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_wait_ready(self.regs, read_status_cmd, self.max_polls)
        } else {
            super::shared::ram_wait_ready_sme1(self.regs, read_status_cmd, self.max_polls)
        }
    }

    #[inline(always)]
    pub(crate) fn read_status(self, cmd: u8) -> Result<u8, ()> {
        if self.xip_safe {
            super::shared::ram_wrapper_read_status(self.regs, cmd, self.max_polls)
        } else {
            super::shared::ram_read_status(self.regs, cmd, self.max_polls)
        }
    }

    #[inline(always)]
    pub(crate) fn read_jedec_id(self, cmd: u8) -> Result<u32, ()> {
        if self.xip_safe {
            super::shared::ram_wrapper_read_jedec_id(self.regs, cmd, self.max_polls)
        } else {
            super::shared::ram_read_jedec_id(self.regs, cmd, self.max_polls)
        }
    }

    #[inline(always)]
    pub(crate) fn read_command_stream(
        self,
        cmd: u8,
        addr: Option<u32>,
        addr_size: u8,
        dummy_cycles: u8,
        out: &mut [u8],
    ) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_read_command_stream(
                self.regs,
                cmd,
                addr,
                addr_size,
                dummy_cycles,
                out,
                self.max_polls,
            )
        } else {
            super::shared::ram_read_command_stream(
                self.regs,
                cmd,
                addr,
                addr_size,
                dummy_cycles,
                out,
                self.max_polls,
            )
        }
    }

    #[inline(always)]
    pub(crate) fn program_chunk(
        self,
        wren_cmd: u8,
        program_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
        data: &[u8],
    ) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_program_chunk(
                self.regs,
                wren_cmd,
                program_cmd,
                read_status_cmd,
                addr,
                addr_size,
                data,
                self.max_polls,
            )
        } else {
            super::shared::ram_program_chunk(
                self.regs,
                wren_cmd,
                program_cmd,
                read_status_cmd,
                addr,
                addr_size,
                data,
                self.max_polls,
            )
        }
    }

    #[inline(always)]
    pub(crate) fn erase_sector(
        self,
        wren_cmd: u8,
        erase_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
    ) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_erase_sector(
                self.regs,
                wren_cmd,
                erase_cmd,
                read_status_cmd,
                addr,
                addr_size,
                self.max_polls,
            )
        } else {
            super::shared::ram_erase_sector(
                self.regs,
                wren_cmd,
                erase_cmd,
                read_status_cmd,
                addr,
                addr_size,
                self.max_polls,
            )
        }
    }

    #[inline(always)]
    pub(crate) fn erase_chip(self, wren_cmd: u8, chip_erase_cmd: u8, read_status_cmd: u8) -> bool {
        if self.xip_safe {
            super::shared::ram_wrapper_erase_chip(
                self.regs,
                wren_cmd,
                chip_erase_cmd,
                read_status_cmd,
                self.max_polls,
            )
        } else {
            super::shared::ram_erase_chip(
                self.regs,
                wren_cmd,
                chip_erase_cmd,
                read_status_cmd,
                self.max_polls,
            )
        }
    }
}

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
    Exec::new(regs, max_polls, xip_safe).issue_simple_cmd(cmd)
}

#[inline(always)]
pub(crate) fn wait_ready(regs: Regs, read_status_cmd: u8, max_polls: u32, xip_safe: bool) -> bool {
    Exec::new(regs, max_polls, xip_safe).wait_ready(read_status_cmd)
}

#[inline(always)]
pub(crate) fn read_status(regs: Regs, cmd: u8, max_polls: u32, xip_safe: bool) -> Result<u8, ()> {
    Exec::new(regs, max_polls, xip_safe).read_status(cmd)
}

#[inline(always)]
pub(crate) fn read_jedec_id(
    regs: Regs,
    cmd: u8,
    max_polls: u32,
    xip_safe: bool,
) -> Result<u32, ()> {
    Exec::new(regs, max_polls, xip_safe).read_jedec_id(cmd)
}
