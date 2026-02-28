use core::cmp::min;

use embedded_storage::nor_flash::{check_erase, check_write, ErrorType, NorFlash, ReadNorFlash};

use super::driver::Mpi;
use super::shared::*;
use super::types::*;
use super::{Instance, Regs};
use crate::Peripheral;

#[derive(Clone, Copy)]
pub(super) enum NorExecBackend {
    Direct,
    XipSafe,
}

impl NorExecBackend {
    const fn direct() -> Self {
        Self::Direct
    }

    const fn xip_safe() -> Self {
        Self::XipSafe
    }

    pub(super) const fn from_running_from_xip(running_from_xip: bool) -> Self {
        if running_from_xip {
            Self::xip_safe()
        } else {
            Self::direct()
        }
    }

    #[inline(always)]
    fn issue_simple_cmd(self, regs: Regs, cmd: u8, max_polls: u32) -> bool {
        match self {
            Self::Direct => ram_issue_simple_cmd(regs, cmd, max_polls),
            Self::XipSafe => ram_wrapper_issue_simple_cmd(regs, cmd, max_polls),
        }
    }

    #[inline(always)]
    fn wait_ready(self, regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool {
        match self {
            Self::Direct => ram_wait_ready_sme1(regs, read_status_cmd, max_polls),
            Self::XipSafe => ram_wrapper_wait_ready(regs, read_status_cmd, max_polls),
        }
    }

    #[inline(always)]
    fn read_status(self, regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()> {
        match self {
            Self::Direct => ram_read_status(regs, cmd, max_polls),
            Self::XipSafe => ram_wrapper_read_status(regs, cmd, max_polls),
        }
    }

    #[inline(always)]
    fn read_jedec_id(self, regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()> {
        match self {
            Self::Direct => ram_read_jedec_id(regs, cmd, max_polls),
            Self::XipSafe => ram_wrapper_read_jedec_id(regs, cmd, max_polls),
        }
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
        match self {
            Self::Direct => ram_program_chunk(
                regs,
                wren_cmd,
                program_cmd,
                read_status_cmd,
                addr,
                addr_size,
                data,
                max_polls,
            ),
            Self::XipSafe => ram_wrapper_program_chunk(
                regs,
                wren_cmd,
                program_cmd,
                read_status_cmd,
                addr,
                addr_size,
                data,
                max_polls,
            ),
        }
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
        match self {
            Self::Direct => ram_erase_sector(
                regs,
                wren_cmd,
                erase_cmd,
                read_status_cmd,
                addr,
                addr_size,
                max_polls,
            ),
            Self::XipSafe => ram_wrapper_erase_sector(
                regs,
                wren_cmd,
                erase_cmd,
                read_status_cmd,
                addr,
                addr_size,
                max_polls,
            ),
        }
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
        match self {
            Self::Direct => {
                ram_erase_chip(regs, wren_cmd, chip_erase_cmd, read_status_cmd, max_polls)
            }
            Self::XipSafe => {
                ram_wrapper_erase_chip(regs, wren_cmd, chip_erase_cmd, read_status_cmd, max_polls)
            }
        }
    }
}

pub struct MpiNorFlash<'d, T: Instance, const WRITE_GRAN: usize = 1, const ERASE_GRAN: usize = 4096>
{
    pub(super) mpi: Mpi<'d, T>,
    pub(super) config: NorFlashConfig,
    pub(super) exec: NorExecBackend,
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize>
    MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    /// Create a blocking NOR flash instance with common defaults.
    pub fn new_blocking(
        inner: impl Peripheral<P = T> + 'd,
        capacity: usize,
    ) -> Result<Self, Error> {
        Mpi::try_new(inner)?.into_nor_flash_default::<WRITE_GRAN, ERASE_GRAN>(capacity)
    }

    /// Create a blocking NOR flash instance with explicit MPI and NOR configs.
    pub fn new_blocking_with_config(
        inner: impl Peripheral<P = T> + 'd,
        mpi_config: MpiInitConfig,
        flash_config: NorFlashConfig,
    ) -> Result<Self, Error> {
        Mpi::with_config(inner, mpi_config)?.into_nor_flash(flash_config)
    }

    /// Create a blocking NOR flash instance without resetting MPI.
    ///
    /// Useful when code may execute from this instance's XIP window.
    pub fn new_blocking_without_reset(
        inner: impl Peripheral<P = T> + 'd,
        flash_config: NorFlashConfig,
    ) -> Result<Self, Error> {
        Mpi::new_without_reset(inner).into_nor_flash(flash_config)
    }

    pub const fn config(&self) -> NorFlashConfig {
        self.config
    }

    fn is_source_in_code_bus_flash(bytes: &[u8]) -> bool {
        if bytes.is_empty() {
            return false;
        }
        let start = bytes.as_ptr() as usize;
        let end = start.saturating_add(bytes.len());
        start < CODE_BUS_FLASH_END && end > CODE_BUS_FLASH_START
    }

    fn invalidate_cache_for_range(&self, offset: u32, len: usize) {
        if len == 0 {
            return;
        }

        // Refresh cache view for memory-mapped flash area after program/erase.
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

    fn regs(&self) -> Regs {
        T::regs()
    }

    #[inline(always)]
    fn selected_cmd(&self, cmd_3byte: u8, cmd_4byte: Option<u8>) -> u8 {
        if self.config.address_size == AddressSize::FourBytes {
            cmd_4byte.unwrap_or(cmd_3byte)
        } else {
            cmd_3byte
        }
    }

    #[inline(always)]
    fn selected_optional_cmd(&self, cmd_3byte: Option<u8>, cmd_4byte: Option<u8>) -> Option<u8> {
        if self.config.address_size == AddressSize::FourBytes {
            cmd_4byte.or(cmd_3byte)
        } else {
            cmd_3byte
        }
    }

    #[inline(always)]
    fn addr_size_bits(&self) -> u8 {
        self.config.address_size.bits()
    }

    #[inline(always)]
    fn write_enable_cmd(&self) -> u8 {
        self.config.commands.write_enable
    }

    #[inline(always)]
    fn read_status_cmd(&self) -> u8 {
        self.config.commands.read_status
    }

    #[inline(always)]
    fn read_jedec_id_cmd(&self) -> u8 {
        self.config.commands.read_jedec_id
    }

    #[inline(always)]
    fn page_program_cmd(&self) -> u8 {
        self.selected_cmd(
            self.config.commands.page_program,
            self.config.commands.page_program_4byte,
        )
    }

    #[inline(always)]
    fn sector_erase_cmd(&self) -> u8 {
        self.selected_cmd(
            self.config.commands.sector_erase,
            self.config.commands.sector_erase_4byte,
        )
    }

    #[inline(always)]
    fn block_erase_32k_cmd(&self) -> Option<u8> {
        self.selected_optional_cmd(
            self.config.commands.block_erase_32k,
            self.config.commands.block_erase_32k_4byte,
        )
    }

    #[inline(always)]
    fn block_erase_64k_cmd(&self) -> Option<u8> {
        self.selected_optional_cmd(
            self.config.commands.block_erase_64k,
            self.config.commands.block_erase_64k_4byte,
        )
    }

    #[inline(always)]
    fn chip_erase_cmd(&self) -> Option<u8> {
        self.config.commands.chip_erase
    }

    #[inline(always)]
    fn base_erase_opcode(&self) -> Option<u8> {
        match ERASE_GRAN {
            4096 => Some(self.sector_erase_cmd()),
            32768 => self.block_erase_32k_cmd(),
            65536 => self.block_erase_64k_cmd(),
            _ => None,
        }
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
            .issue_simple_cmd(self.regs(), cmd, self.config.max_ready_polls);
        Self::timeout_if_false(ok)
    }

    fn erase_with_opcode(&mut self, addr: u32, erase_cmd: u8) -> Result<(), Error> {
        let ok = self.exec.erase_sector(
            self.regs(),
            self.write_enable_cmd(),
            erase_cmd,
            self.read_status_cmd(),
            addr,
            self.addr_size_bits(),
            self.config.max_ready_polls,
        );
        Self::timeout_if_false(ok)
    }

    fn erase_chip_internal(&mut self, erase_cmd: u8) -> Result<(), Error> {
        let ok = self.exec.erase_chip(
            self.regs(),
            self.write_enable_cmd(),
            erase_cmd,
            self.read_status_cmd(),
            self.config.max_ready_polls,
        );
        Self::timeout_if_false(ok)
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
        if dummy_cycles > 31 {
            return Err(Error::InvalidConfiguration);
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

    pub fn read_status(&mut self) -> Result<u8, Error> {
        self.exec
            .read_status(
                self.regs(),
                self.read_status_cmd(),
                self.config.max_ready_polls,
            )
            .map_err(|_| Error::Timeout)
    }

    pub fn wait_ready(&mut self) -> Result<(), Error> {
        let ok = self.exec.wait_ready(
            self.regs(),
            self.read_status_cmd(),
            self.config.max_ready_polls,
        );
        Self::timeout_if_false(ok)
    }

    pub fn read_jedec_id(&mut self) -> Result<u32, Error> {
        self.exec
            .read_jedec_id(
                self.regs(),
                self.read_jedec_id_cmd(),
                self.config.max_ready_polls,
            )
            .map_err(|_| Error::Timeout)
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        let ok = self.exec.program_chunk(
            self.regs(),
            self.write_enable_cmd(),
            self.page_program_cmd(),
            self.read_status_cmd(),
            addr,
            self.addr_size_bits(),
            data,
            self.config.max_ready_polls,
        );
        Self::timeout_if_false(ok)
    }

    fn next_program_chunk_len(&self, addr: usize, remaining: usize) -> Result<usize, Error> {
        let page_remaining = self.config.page_size - (addr % self.config.page_size);
        let mut step = min(remaining, min(page_remaining, FIFO_SIZE_BYTES));
        if WRITE_GRAN > 1 {
            step -= step % WRITE_GRAN;
        }
        if step == 0 {
            return Err(Error::InvalidConfiguration);
        }
        Ok(step)
    }

    pub fn set_dma_threshold_bytes(&mut self, threshold: usize) -> Result<(), Error> {
        if threshold == 0 {
            return Err(Error::InvalidConfiguration);
        }
        self.config.dma_threshold_bytes = threshold;
        Ok(())
    }

    pub fn enable_dma(&mut self, enable: bool) {
        self.mpi.enable_dma(enable);
    }

    pub fn read_bytes(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        let start = offset as usize;
        let end = start.checked_add(bytes.len()).ok_or(Error::OutOfBounds)?;
        if end > self.config.capacity {
            return Err(Error::OutOfBounds);
        }

        // Use memory-mapped read path for robustness in XIP scenarios.
        let base = T::code_bus_base() + start;
        let len = bytes.len();
        let mut i = 0usize;

        // Handle unaligned head (read bytes until 4-byte aligned).
        while i < len && !(base + i).is_multiple_of(4) {
            // SAFETY: Bounds are checked against `capacity` above.
            bytes[i] = unsafe { core::ptr::read_volatile((base + i) as *const u8) };
            i += 1;
        }

        // Bulk word-aligned reads.
        while i + 4 <= len {
            // SAFETY: `base + i` is 4-byte aligned and within bounds.
            let word = unsafe { core::ptr::read_volatile((base + i) as *const u32) };
            let wb = word.to_le_bytes();
            bytes[i] = wb[0];
            bytes[i + 1] = wb[1];
            bytes[i + 2] = wb[2];
            bytes[i + 3] = wb[3];
            i += 4;
        }

        // Handle unaligned tail.
        while i < len {
            bytes[i] = unsafe { core::ptr::read_volatile((base + i) as *const u8) };
            i += 1;
        }

        Ok(())
    }

    pub fn write_bytes(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        check_write(self, offset, bytes.len()).map_err(Error::from)?;
        self.wait_ready()?;

        // If source data is in flash code bus region, copy to bounce buffer first
        // to avoid reading from flash while programming it.
        let source_conflict = Self::is_source_in_code_bus_flash(bytes);
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

    pub fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if from == to {
            return Ok(());
        }
        check_erase(self, from, to).map_err(Error::from)?;
        self.wait_ready()?;

        // Full-chip erase path when possible.
        if from == 0 && (to as usize) == self.config.capacity {
            if let Some(chip_erase_cmd) = self.chip_erase_cmd() {
                self.erase_chip_internal(chip_erase_cmd)?;
                self.invalidate_cache_for_range(from, (to - from) as usize);
                return Ok(());
            }
        }

        let base_opcode = self
            .base_erase_opcode()
            .ok_or(Error::InvalidConfiguration)?;
        let mut addr = from;
        while addr < to {
            let (opcode, advanced) = plan_next_erase_step(
                addr,
                to,
                ERASE_GRAN as u32,
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

    pub fn enter_deep_power_down(&mut self) -> Result<(), Error> {
        self.wait_ready()?;
        let cmd = self
            .config
            .commands
            .deep_power_down
            .ok_or(Error::InvalidConfiguration)?;
        self.issue_simple_command(cmd)
    }

    pub fn release_deep_power_down(&mut self) -> Result<(), Error> {
        let cmd = self
            .config
            .commands
            .release_power_down
            .ok_or(Error::InvalidConfiguration)?;
        self.issue_simple_command(cmd)
    }

    pub fn reset_device(&mut self) -> Result<(), Error> {
        if let Some(cmd) = self.config.commands.reset_enable {
            self.issue_simple_command(cmd)?;
        }
        let reset_cmd = self
            .config
            .commands
            .reset
            .ok_or(Error::InvalidConfiguration)?;
        self.issue_simple_command(reset_cmd)?;
        self.wait_ready()
    }

    pub fn read_sfdp(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        let cmd = self
            .config
            .commands
            .read_sfdp
            .ok_or(Error::InvalidConfiguration)?;
        // SFDP command uses 3-byte address + 8 dummy cycles.
        self.read_command_stream(cmd, Some(offset), AddressSize::ThreeBytes, 8, out)
    }

    pub fn read_unique_id(&mut self, out: &mut [u8]) -> Result<(), Error> {
        let cmd = self
            .config
            .commands
            .read_unique_id
            .ok_or(Error::InvalidConfiguration)?;
        // Many devices encode this as 4-byte zero address after command.
        self.read_command_stream(cmd, Some(0), AddressSize::FourBytes, 0, out)
    }

    pub fn into_partition(
        self,
        partition: NorFlashPartition,
    ) -> Result<MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
        if !(partition.offset as usize).is_multiple_of(WRITE_GRAN)
            || !(partition.offset as usize).is_multiple_of(ERASE_GRAN)
            || !(partition.size as usize).is_multiple_of(ERASE_GRAN)
        {
            return Err(Error::NotAligned);
        }
        let end = partition
            .offset
            .checked_add(partition.size)
            .ok_or(Error::OutOfBounds)?;
        if end as usize > self.config.capacity {
            return Err(Error::OutOfBounds);
        }

        Ok(MpiNorPartition {
            flash: self,
            partition,
        })
    }

    /// Convenience helper for creating a partition from `offset` + `size`.
    pub fn into_partition_offset_size(
        self,
        offset: u32,
        size: u32,
    ) -> Result<MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
        self.into_partition(NorFlashPartition::new(offset, size))
    }

    pub fn free(self) -> Mpi<'d, T> {
        self.mpi
    }
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> ErrorType
    for MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    type Error = Error;
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> ReadNorFlash
    for MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.config.capacity
    }
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> NorFlash
    for MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    const WRITE_SIZE: usize = WRITE_GRAN;
    const ERASE_SIZE: usize = ERASE_GRAN;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_range(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(offset, bytes)
    }
}

pub struct MpiNorPartition<
    'd,
    T: Instance,
    const WRITE_GRAN: usize = 1,
    const ERASE_GRAN: usize = 4096,
> {
    flash: MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>,
    partition: NorFlashPartition,
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize>
    MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    fn map_offset(&self, offset: u32) -> Result<u32, Error> {
        if offset > self.partition.size {
            return Err(Error::OutOfBounds);
        }
        self.partition
            .offset
            .checked_add(offset)
            .ok_or(Error::OutOfBounds)
    }

    fn ensure_region(&self, offset: u32, len: usize) -> Result<(), Error> {
        let end = (offset as usize)
            .checked_add(len)
            .ok_or(Error::OutOfBounds)?;
        if end > self.partition.size as usize {
            return Err(Error::OutOfBounds);
        }
        Ok(())
    }

    pub const fn partition(&self) -> NorFlashPartition {
        self.partition
    }

    pub fn read_bytes(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.ensure_region(offset, bytes.len())?;
        let abs = self.map_offset(offset)?;
        self.flash.read_bytes(abs, bytes)
    }

    pub fn write_bytes(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.ensure_region(offset, bytes.len())?;
        let abs = self.map_offset(offset)?;
        self.flash.write_bytes(abs, bytes)
    }

    pub fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if to < from {
            return Err(Error::OutOfBounds);
        }
        self.ensure_region(from, (to - from) as usize)?;
        let abs_from = self.map_offset(from)?;
        let abs_to = self.map_offset(to)?;
        self.flash.erase_range(abs_from, abs_to)
    }

    pub fn free(self) -> MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN> {
        self.flash
    }
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> ErrorType
    for MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    type Error = Error;
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> ReadNorFlash
    for MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.partition.size as usize
    }
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize> NorFlash
    for MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>
{
    const WRITE_SIZE: usize = WRITE_GRAN;
    const ERASE_SIZE: usize = ERASE_GRAN;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_range(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(offset, bytes)
    }
}
