use core::cmp::min;

use super::super::controller::Mpi;
use super::super::shared::{
    plan_next_erase_step, FIFO_SIZE_BYTES, MAX_DUMMY_CYCLES, NOR_FLASH_MAX_3B_CAPACITY_BYTES,
};
use super::super::types::{
    AddressSize, AhbCommandConfig, CommandConfig, CommandSlot, Error, FifoClear, FunctionMode,
    MpiInitConfig, PhaseMode,
};
use super::super::xip;
use super::super::{Instance, Regs};
use super::detect::JedecId;
use super::profile::{AddressingPolicy, NorConfig, ResolvedProfile};
use crate::Peripheral;

#[derive(Clone, Copy)]
enum NorExecBackend {
    Direct,
    XipSafe,
}

impl NorExecBackend {
    const fn from_running_from_xip(running_from_xip: bool) -> Self {
        if running_from_xip {
            Self::XipSafe
        } else {
            Self::Direct
        }
    }

    #[inline(always)]
    const fn xip_safe(self) -> bool {
        matches!(self, Self::XipSafe)
    }

    #[inline(always)]
    fn issue_simple_cmd(self, regs: Regs, cmd: u8, max_polls: u32) -> bool {
        xip::issue_simple_cmd(regs, cmd, max_polls, self.xip_safe())
    }

    #[inline(always)]
    fn wait_ready(self, regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool {
        xip::wait_ready(regs, read_status_cmd, max_polls, self.xip_safe())
    }

    #[inline(always)]
    fn read_status(self, regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()> {
        xip::read_status(regs, cmd, max_polls, self.xip_safe())
    }

    #[inline(always)]
    fn read_jedec_id(self, regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()> {
        xip::read_jedec_id(regs, cmd, max_polls, self.xip_safe())
    }

    #[inline(always)]
    fn read_command_stream(
        self,
        regs: Regs,
        cmd: u8,
        addr: Option<u32>,
        addr_size: u8,
        dummy_cycles: u8,
        out: &mut [u8],
        max_polls: u32,
    ) -> bool {
        xip::read_command_stream(
            regs,
            cmd,
            addr,
            addr_size,
            dummy_cycles,
            out,
            max_polls,
            self.xip_safe(),
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    fn program_chunk(
        self,
        regs: Regs,
        wren_cmd: u8,
        program_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
        data: &[u8],
        max_polls: u32,
    ) -> bool {
        xip::program_chunk(
            regs,
            wren_cmd,
            program_cmd,
            read_status_cmd,
            addr,
            addr_size,
            data,
            max_polls,
            self.xip_safe(),
        )
    }

    #[inline(always)]
    fn erase_sector(
        self,
        regs: Regs,
        wren_cmd: u8,
        erase_cmd: u8,
        read_status_cmd: u8,
        addr: u32,
        addr_size: u8,
        max_polls: u32,
    ) -> bool {
        xip::erase_sector(
            regs,
            wren_cmd,
            erase_cmd,
            read_status_cmd,
            addr,
            addr_size,
            max_polls,
            self.xip_safe(),
        )
    }

    #[inline(always)]
    fn erase_chip(
        self,
        regs: Regs,
        wren_cmd: u8,
        chip_erase_cmd: u8,
        read_status_cmd: u8,
        max_polls: u32,
    ) -> bool {
        xip::erase_chip(
            regs,
            wren_cmd,
            chip_erase_cmd,
            read_status_cmd,
            max_polls,
            self.xip_safe(),
        )
    }
}

pub(crate) struct NorBlockingCore<'d, T: Instance> {
    mpi: Mpi<'d, T>,
    profile: ResolvedProfile,
    config: NorConfig,
    exec: NorExecBackend,
}

impl<'d, T: Instance> NorBlockingCore<'d, T> {
    pub(crate) fn new(
        peri: impl Peripheral<P = T> + 'd,
        profile: ResolvedProfile,
        config: NorConfig,
    ) -> Result<Self, Error> {
        let running_from_xip = xip::running_from_same_instance_xip::<T>();
        let init = if running_from_xip {
            MpiInitConfig::xip_without_reset()
        } else {
            MpiInitConfig::default()
        };
        let mpi = Mpi::with_config(peri, init)?;
        Self::from_mpi_with_mode(mpi, profile, config, running_from_xip)
    }

    pub(crate) fn new_without_reset(
        peri: impl Peripheral<P = T> + 'd,
        profile: ResolvedProfile,
        config: NorConfig,
    ) -> Result<Self, Error> {
        let mpi = Mpi::new_without_reset(peri);
        Self::from_mpi_with_mode(mpi, profile, config, true)
    }

    pub(crate) fn from_mpi(
        mpi: Mpi<'d, T>,
        profile: ResolvedProfile,
        config: NorConfig,
    ) -> Result<Self, Error> {
        let running_from_xip = xip::running_from_same_instance_xip::<T>();
        Self::from_mpi_with_mode(mpi, profile, config, running_from_xip)
    }

    fn from_mpi_with_mode(
        mut mpi: Mpi<'d, T>,
        profile: ResolvedProfile,
        config: NorConfig,
        running_from_xip: bool,
    ) -> Result<Self, Error> {
        let window_size = xip::instance_window_size::<T>();
        if profile.capacity_bytes > window_size {
            return Err(Error::CapacityExceedsWindow);
        }

        let needs_4byte_address = profile.capacity_bytes > NOR_FLASH_MAX_3B_CAPACITY_BYTES;

        if needs_4byte_address {
            if running_from_xip && !config.allow_preconfigured_4byte_in_xip {
                return Err(Error::RequiresPreconfigured4ByteMode);
            }

            if let Some(cmd) = profile.commands.enter_4byte_address {
                if !running_from_xip
                    && !xip::issue_simple_cmd(T::regs(), cmd, config.max_ready_polls, false)
                {
                    return Err(Error::Timeout);
                }
            }
        }

        if !running_from_xip {
            let ahb_read_cmd = if profile.address_size == AddressSize::FourBytes {
                profile
                    .commands
                    .ahb_read_4byte
                    .unwrap_or(profile.commands.ahb_read)
            } else {
                profile.commands.ahb_read
            };
            mpi.configure_ahb_read_command(
                ahb_read_cmd,
                AhbCommandConfig::single_io(profile.address_size),
            )?;
        }

        if profile.capacity_bytes > NOR_FLASH_MAX_3B_CAPACITY_BYTES
            && profile.profile.addressing_policy() == AddressingPolicy::ThreeByteOnly
        {
            return Err(Error::UnsupportedProfileFeature);
        }

        Ok(Self {
            mpi,
            profile,
            config,
            exec: NorExecBackend::from_running_from_xip(running_from_xip),
        })
    }

    pub(crate) fn profile(&self) -> &'static dyn super::profile::NorProfile {
        self.profile.profile
    }

    pub(crate) const fn capacity(&self) -> usize {
        self.profile.capacity_bytes
    }

    pub(crate) fn read_jedec_id(&mut self) -> Result<JedecId, Error> {
        let raw = self.map_timeout(self.exec.read_jedec_id(
            T::regs(),
            self.profile.commands.read_jedec_id,
            self.max_ready_polls(),
        ))?;
        Ok(JedecId::from_raw(raw))
    }

    pub(crate) fn read_status(&mut self) -> Result<u8, Error> {
        self.map_timeout(self.exec.read_status(
            T::regs(),
            self.profile.commands.read_status,
            self.max_ready_polls(),
        ))
    }

    pub(crate) fn read_sfdp(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        let cmd = self.require_command(self.profile.commands.read_sfdp)?;
        self.read_command_stream(cmd, Some(offset), AddressSize::ThreeBytes, 8, out)
    }

    pub(crate) fn read_unique_id(&mut self, out: &mut [u8]) -> Result<(), Error> {
        let cmd = self.require_command(self.profile.commands.read_unique_id)?;
        self.read_command_stream(cmd, Some(0), AddressSize::FourBytes, 0, out)
    }

    pub(crate) fn enter_deep_power_down(&mut self) -> Result<(), Error> {
        self.ensure_direct_command_safe()?;
        self.wait_ready()?;
        let cmd = self.require_command(self.profile.commands.deep_power_down)?;
        self.issue_simple_command(cmd)
    }

    pub(crate) fn release_deep_power_down(&mut self) -> Result<(), Error> {
        self.ensure_direct_command_safe()?;
        let cmd = self.require_command(self.profile.commands.release_power_down)?;
        self.issue_simple_command(cmd)
    }

    pub(crate) fn read_bytes(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.ensure_bounds(offset, bytes.len())?;

        let base = T::code_bus_base() + offset as usize;
        let len = bytes.len();
        let mut i = 0usize;

        while i < len && !(base + i).is_multiple_of(4) {
            bytes[i] = unsafe { core::ptr::read_volatile((base + i) as *const u8) };
            i += 1;
        }

        while i + 4 <= len {
            let word = unsafe { core::ptr::read_volatile((base + i) as *const u32) };
            bytes[i..i + 4].copy_from_slice(&word.to_le_bytes());
            i += 4;
        }

        while i < len {
            bytes[i] = unsafe { core::ptr::read_volatile((base + i) as *const u8) };
            i += 1;
        }

        Ok(())
    }

    pub(crate) fn write_bytes(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        self.ensure_bounds(offset, bytes.len())?;
        self.wait_ready()?;

        let source_conflict = self.is_source_in_code_bus_flash(bytes);
        let mut bounce = [0u8; FIFO_SIZE_BYTES];

        let mut done = 0usize;
        while done < bytes.len() {
            let addr = offset as usize + done;
            let step = self.next_program_chunk_len(addr, bytes.len() - done)?;

            if source_conflict {
                bounce[..step].copy_from_slice(&bytes[done..done + step]);
                self.program_chunk(addr as u32, &bounce[..step])?;
            } else {
                self.program_chunk(addr as u32, &bytes[done..done + step])?;
            }
            done += step;
        }

        self.invalidate_cache_for_range(offset, bytes.len());
        Ok(())
    }

    pub(crate) fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if from == to {
            return Ok(());
        }
        self.ensure_erase_bounds(from, to)?;
        self.wait_ready()?;

        if from == 0 && to as usize == self.profile.capacity_bytes {
            if let Some(chip_erase_cmd) = self.profile.commands.chip_erase {
                self.erase_chip_internal(chip_erase_cmd)?;
                self.invalidate_cache_for_range(from, (to - from) as usize);
                return Ok(());
            }
        }

        let base_opcode = self.base_erase_opcode()?;
        let mut addr = from;
        while addr < to {
            let (opcode, advanced) = plan_next_erase_step(
                addr,
                to,
                4096,
                base_opcode,
                self.block_erase_32k_cmd(),
                self.block_erase_64k_cmd(),
            );
            self.erase_with_opcode(addr, opcode)?;
            addr += advanced;
        }
        self.invalidate_cache_for_range(from, (to - from) as usize);
        Ok(())
    }

    #[cfg(feature = "unstable-mpi-controller")]
    pub(crate) fn free(self) -> Mpi<'d, T> {
        self.mpi
    }

    fn ensure_bounds(&self, offset: u32, len: usize) -> Result<(), Error> {
        let start = offset as usize;
        let end = start.checked_add(len).ok_or(Error::OutOfBounds)?;
        if end > self.profile.capacity_bytes {
            return Err(Error::OutOfBounds);
        }
        Ok(())
    }

    fn ensure_erase_bounds(&self, from: u32, to: u32) -> Result<(), Error> {
        if to < from {
            return Err(Error::OutOfBounds);
        }
        if !from.is_multiple_of(4096) || !to.is_multiple_of(4096) {
            return Err(Error::NotAligned);
        }
        self.ensure_bounds(from, (to - from) as usize)
    }

    #[inline(always)]
    fn max_ready_polls(&self) -> u32 {
        self.config.max_ready_polls
    }

    #[inline(always)]
    pub(crate) const fn dma_threshold_bytes(&self) -> usize {
        self.config.dma_threshold_bytes
    }

    #[inline(always)]
    fn map_timeout<U>(&self, result: Result<U, ()>) -> Result<U, Error> {
        result.map_err(|_| Error::Timeout)
    }

    #[inline(always)]
    fn ensure_direct_command_safe(&self) -> Result<(), Error> {
        if self.exec.xip_safe() {
            return Err(Error::CommandForbiddenInXip);
        }
        Ok(())
    }

    #[inline(always)]
    fn require_command(&self, cmd: Option<u8>) -> Result<u8, Error> {
        cmd.ok_or(Error::InvalidConfiguration)
    }

    fn is_source_in_code_bus_flash(&self, bytes: &[u8]) -> bool {
        if bytes.is_empty() {
            return false;
        }
        let start = bytes.as_ptr() as usize;
        let end = start.saturating_add(bytes.len());
        start < xip::CODE_BUS_FLASH_END && end > xip::CODE_BUS_FLASH_START
    }

    fn invalidate_cache_for_range(&self, offset: u32, len: usize) {
        if len == 0 {
            return;
        }

        const DCACHE_LINE_SIZE: usize = 32;
        let start = T::code_bus_base() + offset as usize;
        let aligned_start = start & !(DCACHE_LINE_SIZE - 1);
        let end = start + len;
        let aligned_end = end.saturating_add(DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);
        let aligned_len = aligned_end - aligned_start;

        let mut cp = unsafe { cortex_m::Peripherals::steal() };
        cp.SCB
            .clean_invalidate_dcache_by_address(aligned_start, aligned_len);
        cp.SCB.invalidate_icache();
    }

    #[inline(always)]
    fn using_4byte_address(&self) -> bool {
        self.profile.address_size == AddressSize::FourBytes
    }

    #[inline(always)]
    fn addr_size_bits(&self) -> u8 {
        self.profile.address_size.bits()
    }

    #[inline(always)]
    fn selected_cmd(&self, cmd_3byte: u8, cmd_4byte: Option<u8>) -> u8 {
        if self.using_4byte_address() {
            cmd_4byte.unwrap_or(cmd_3byte)
        } else {
            cmd_3byte
        }
    }

    #[inline(always)]
    fn selected_optional_cmd(&self, cmd_3byte: Option<u8>, cmd_4byte: Option<u8>) -> Option<u8> {
        if self.using_4byte_address() {
            cmd_4byte.or(cmd_3byte)
        } else {
            cmd_3byte
        }
    }

    #[inline(always)]
    fn page_program_cmd(&self) -> u8 {
        self.selected_cmd(
            self.profile.commands.page_program,
            self.profile.commands.page_program_4byte,
        )
    }

    #[inline(always)]
    fn sector_erase_cmd(&self) -> u8 {
        self.selected_cmd(
            self.profile.commands.sector_erase,
            self.profile.commands.sector_erase_4byte,
        )
    }

    #[inline(always)]
    fn block_erase_32k_cmd(&self) -> Option<u8> {
        self.selected_optional_cmd(
            self.profile.commands.block_erase_32k,
            self.profile.commands.block_erase_32k_4byte,
        )
    }

    #[inline(always)]
    fn block_erase_64k_cmd(&self) -> Option<u8> {
        self.selected_optional_cmd(
            self.profile.commands.block_erase_64k,
            self.profile.commands.block_erase_64k_4byte,
        )
    }

    fn base_erase_opcode(&self) -> Result<u8, Error> {
        Ok(self.sector_erase_cmd())
    }

    #[inline(always)]
    fn timeout_if_false(ok: bool) -> Result<(), Error> {
        if ok {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn issue_simple_command(&mut self, cmd: u8) -> Result<(), Error> {
        let ok = self
            .exec
            .issue_simple_cmd(T::regs(), cmd, self.max_ready_polls());
        Self::timeout_if_false(ok)
    }

    fn wait_ready(&mut self) -> Result<(), Error> {
        let ok = self.exec.wait_ready(
            T::regs(),
            self.profile.commands.read_status,
            self.max_ready_polls(),
        );
        Self::timeout_if_false(ok)
    }

    fn erase_with_opcode(&mut self, addr: u32, erase_cmd: u8) -> Result<(), Error> {
        let ok = self.exec.erase_sector(
            T::regs(),
            self.profile.commands.write_enable,
            erase_cmd,
            self.profile.commands.read_status,
            addr,
            self.addr_size_bits(),
            self.max_ready_polls(),
        );
        Self::timeout_if_false(ok)
    }

    fn erase_chip_internal(&mut self, erase_cmd: u8) -> Result<(), Error> {
        let ok = self.exec.erase_chip(
            T::regs(),
            self.profile.commands.write_enable,
            erase_cmd,
            self.profile.commands.read_status,
            self.max_ready_polls(),
        );
        Self::timeout_if_false(ok)
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        let ok = self.exec.program_chunk(
            T::regs(),
            self.profile.commands.write_enable,
            self.page_program_cmd(),
            self.profile.commands.read_status,
            addr,
            self.addr_size_bits(),
            data,
            self.max_ready_polls(),
        );
        Self::timeout_if_false(ok)
    }

    fn next_program_chunk_len(&self, addr: usize, remaining: usize) -> Result<usize, Error> {
        let page_remaining = self.profile.page_size - (addr % self.profile.page_size);
        let step = min(remaining, min(page_remaining, FIFO_SIZE_BYTES));
        if step == 0 {
            return Err(Error::InvalidConfiguration);
        }
        Ok(step)
    }

    fn read_command_stream(
        &mut self,
        cmd: u8,
        mut addr: Option<u32>,
        addr_size: AddressSize,
        dummy_cycles: u8,
        out: &mut [u8],
    ) -> Result<(), Error> {
        if out.is_empty() {
            return Ok(());
        }
        if dummy_cycles > MAX_DUMMY_CYCLES {
            return Err(Error::InvalidConfiguration);
        }

        if self.exec.xip_safe() {
            let ok = self.exec.read_command_stream(
                T::regs(),
                cmd,
                addr,
                addr_size.bits(),
                dummy_cycles,
                out,
                self.max_ready_polls(),
            );
            return Self::timeout_if_false(ok);
        }

        let command_config = CommandConfig {
            function_mode: FunctionMode::Read,
            instruction_mode: PhaseMode::Single,
            address_mode: if addr.is_some() {
                PhaseMode::Single
            } else {
                PhaseMode::None
            },
            address_size: addr_size,
            alternate_mode: PhaseMode::None,
            alternate_size: AddressSize::OneByte,
            data_mode: PhaseMode::Single,
            dummy_cycles,
        };

        let mut done = 0usize;
        while done < out.len() {
            let step = min(FIFO_SIZE_BYTES, out.len() - done);
            self.mpi.clear_fifo(FifoClear::Rx);
            self.mpi
                .configure_command_for(CommandSlot::Cmd1, command_config)?;
            self.mpi.set_data_len_for(CommandSlot::Cmd1, step)?;
            self.mpi.set_command(cmd, addr.unwrap_or(0))?;

            let mut idx = 0usize;
            while idx < step {
                let word = self.mpi.read_word().to_le_bytes();
                let take = min(4, step - idx);
                out[done + idx..done + idx + take].copy_from_slice(&word[..take]);
                idx += take;
            }

            done += step;
            if let Some(a) = &mut addr {
                *a = a.saturating_add(step as u32);
            }
        }
        Ok(())
    }
}
