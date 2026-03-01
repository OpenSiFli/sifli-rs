//! Low-level MPI controller backend.
//!
//! Public re-exports are gated in [`crate::mpi`] behind
//! `unstable-mpi-controller`.

use core::cmp::min;

use embassy_hal_internal::{into_ref, PeripheralRef};

use super::shared::{
    FIFO_SIZE_BYTES, MAX_DLEN_BYTES, MAX_DUMMY_CYCLES, STATUS_MATCH_TIMEOUT_POLLS,
};
pub use super::types::{
    AddressSize, AhbCommandConfig, CommandConfig, CommandSlot, Error, FifoClear, FunctionMode,
    MpiInitConfig, PhaseMode,
};
use super::xip;
use super::{Instance, Regs};
use crate::{rcc, Peripheral};

macro_rules! apply_ahb_command_config {
    ($reg:expr, $cfg:expr) => {
        $reg.modify(|w| {
            w.set_dmode($cfg.data_mode.bits());
            w.set_dcyc($cfg.dummy_cycles);
            w.set_absize($cfg.alternate_size.bits());
            w.set_abmode($cfg.alternate_mode.bits());
            w.set_adsize($cfg.address_size.bits());
            w.set_admode($cfg.address_mode.bits());
            w.set_imode($cfg.instruction_mode.bits());
        })
    };
}

macro_rules! apply_command_config {
    ($reg:expr, $cfg:expr) => {
        $reg.modify(|w| {
            w.set_fmode($cfg.function_mode.is_write());
            w.set_dmode($cfg.data_mode.bits());
            w.set_dcyc($cfg.dummy_cycles);
            w.set_absize($cfg.alternate_size.bits());
            w.set_abmode($cfg.alternate_mode.bits());
            w.set_adsize($cfg.address_size.bits());
            w.set_admode($cfg.address_mode.bits());
            w.set_imode($cfg.instruction_mode.bits());
        })
    };
}

pub struct Mpi<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Mpi<'d, T> {
    #[inline(always)]
    fn validate_dummy_cycles(dummy_cycles: u8) -> Result<(), Error> {
        if dummy_cycles > MAX_DUMMY_CYCLES {
            return Err(Error::InvalidConfiguration);
        }
        Ok(())
    }

    #[inline(always)]
    fn wait_not_busy(regs: Regs) -> Result<(), Error> {
        if !xip::wait_not_busy(regs, STATUS_MATCH_TIMEOUT_POLLS) {
            return Err(Error::Timeout);
        }
        Ok(())
    }

    #[inline(always)]
    fn write_slot_command(regs: Regs, slot: CommandSlot, cmd: u8, addr: u32) {
        match slot {
            CommandSlot::Cmd1 => {
                regs.ar1().write(|w| w.set_addr(addr));
                regs.cmdr1().write(|w| w.set_cmd(cmd));
            }
            CommandSlot::Cmd2 => {
                regs.ar2().write(|w| w.set_addr(addr));
                regs.cmdr2().write(|w| w.set_cmd(cmd));
            }
        }
    }

    pub fn with_config(
        inner: impl Peripheral<P = T> + 'd,
        init: MpiInitConfig,
    ) -> Result<Self, Error> {
        if init.reset && !init.allow_reset_from_xip && xip::running_from_same_instance_xip::<T>() {
            return Err(Error::UnsafeResetInXip);
        }

        into_ref!(inner);

        if init.reset {
            rcc::enable_and_reset::<T>();
        } else {
            rcc::enable::<T>();
        }

        let mut this = Self { _inner: inner };
        if init.apply_default_config {
            this.apply_default_config();
        }
        Ok(this)
    }

    pub fn try_new(inner: impl Peripheral<P = T> + 'd) -> Result<Self, Error> {
        Self::with_config(inner, MpiInitConfig::default())
    }

    pub fn new_without_reset(inner: impl Peripheral<P = T> + 'd) -> Self {
        Self::with_config(inner, MpiInitConfig::xip_without_reset())
            .unwrap_or_else(|_| unreachable!())
    }

    pub fn apply_default_config(&mut self) {
        let regs = self.regs();
        regs.timr().write(|w| w.set_timeout(0x00ff));
        regs.cir().write(|w| {
            w.set_interval1(0x5000);
            w.set_interval2(0x5000);
        });
        regs.abr1().write(|w| w.set_abyte(0xff));
        regs.hrabr().write(|w| w.set_abyte(0xff));
        self.configure_ahb_read(0x03, AddressSize::ThreeBytes);
        regs.cr().modify(|w| w.set_en(true));
    }

    fn configure_ahb_read_raw(&mut self, read_cmd: u8, cfg: AhbCommandConfig) {
        let regs = self.regs();
        apply_ahb_command_config!(regs.hrccr(), cfg);
        regs.hcmdr().modify(|w| w.set_rcmd(read_cmd));
    }

    fn configure_ahb_write_raw(&mut self, write_cmd: u8, cfg: AhbCommandConfig) {
        let regs = self.regs();
        apply_ahb_command_config!(regs.hwccr(), cfg);
        regs.hcmdr().modify(|w| w.set_wcmd(write_cmd));
    }

    pub fn configure_ahb_read(&mut self, read_cmd: u8, address_size: AddressSize) {
        self.configure_ahb_read_raw(read_cmd, AhbCommandConfig::single_io(address_size));
    }

    pub fn configure_ahb_read_command(
        &mut self,
        read_cmd: u8,
        cfg: AhbCommandConfig,
    ) -> Result<(), Error> {
        Self::validate_dummy_cycles(cfg.dummy_cycles)?;
        self.configure_ahb_read_raw(read_cmd, cfg);
        Ok(())
    }

    pub fn configure_ahb_write_command(
        &mut self,
        write_cmd: u8,
        cfg: AhbCommandConfig,
    ) -> Result<(), Error> {
        Self::validate_dummy_cycles(cfg.dummy_cycles)?;
        self.configure_ahb_write_raw(write_cmd, cfg);
        Ok(())
    }

    #[inline(always)]
    fn regs(&self) -> Regs {
        T::regs()
    }

    pub fn enable(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_en(enable));
    }

    pub fn enable_dma(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_dmae(enable));
    }

    pub fn enable_prefetch(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_prefe(enable));
    }

    pub fn configure_prefetch_range(&mut self, start: u32, end: u32) -> Result<(), Error> {
        if end < start {
            return Err(Error::OutOfBounds);
        }
        self.regs().prsar().write(|w| w.set_sa(start));
        self.regs().prear().write(|w| w.set_ea(end));
        Ok(())
    }

    pub fn set_prescaler_div(&mut self, div: u8) {
        self.regs().psclr().write(|w| w.set_div(div));
    }

    pub fn set_command_interval(&mut self, interval1: u16, interval2: u16) {
        self.regs().cir().write(|w| {
            w.set_interval1(interval1);
            w.set_interval2(interval2);
        });
    }

    pub fn set_loop_count(&mut self, count: u8) {
        self.regs().cr2().write(|w| w.set_loop_(count));
    }

    pub fn clear_fifo(&mut self, mode: FifoClear) {
        let regs = self.regs();
        match mode {
            FifoClear::Rx => regs.fifocr().modify(|w| w.set_rxclr(true)),
            FifoClear::Tx => regs.fifocr().modify(|w| w.set_txclr(true)),
            FifoClear::RxTx => regs.fifocr().modify(|w| {
                w.set_rxclr(true);
                w.set_txclr(true);
            }),
        }
    }

    pub fn configure_command_for(
        &mut self,
        slot: CommandSlot,
        cfg: CommandConfig,
    ) -> Result<(), Error> {
        Self::validate_dummy_cycles(cfg.dummy_cycles)?;
        let regs = self.regs();
        match slot {
            CommandSlot::Cmd1 => apply_command_config!(regs.ccr1(), cfg),
            CommandSlot::Cmd2 => apply_command_config!(regs.ccr2(), cfg),
        }
        Ok(())
    }

    pub fn set_data_len_for(&mut self, slot: CommandSlot, len: usize) -> Result<(), Error> {
        if len == 0 || len > MAX_DLEN_BYTES {
            return Err(Error::InvalidLength);
        }
        let dlen = (len - 1) as u32;
        let regs = self.regs();
        match slot {
            CommandSlot::Cmd1 => regs.dlr1().write(|w| w.set_dlen(dlen)),
            CommandSlot::Cmd2 => regs.dlr2().write(|w| w.set_dlen(dlen)),
        }
        Ok(())
    }

    pub fn write_word(&mut self, value: u32) {
        self.regs().dr().write(|w| w.set_data(value));
    }

    pub fn read_word(&self) -> u32 {
        self.regs().dr().read().data()
    }

    pub fn enable_command2(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_cmd2e(enable));
    }

    pub fn enable_status_match_cmd2(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_sme2(enable));
    }

    pub fn set_status_mask(&mut self, mask: u32) {
        self.regs().smkr().write(|w| w.set_mask(mask));
    }

    pub fn set_status_match(&mut self, status: u32) {
        self.regs().smr().write(|w| w.set_status(status));
    }

    pub fn set_command(&mut self, cmd: u8, addr: u32) -> Result<(), Error> {
        let regs = self.regs();
        Self::wait_not_busy(regs)?;
        regs.scr().write(|w| w.set_tcfc(true));
        Self::write_slot_command(regs, CommandSlot::Cmd1, cmd, addr);
        if xip::wait_transfer_complete(regs, STATUS_MATCH_TIMEOUT_POLLS) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    pub fn try_set_command_no_wait(
        &mut self,
        slot: CommandSlot,
        cmd: u8,
        addr: u32,
    ) -> Result<(), Error> {
        let regs = self.regs();
        Self::wait_not_busy(regs)?;
        Self::write_slot_command(regs, slot, cmd, addr);
        Ok(())
    }
}
pub enum TransferData<'a> {
    None,
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

pub struct Transfer<'a> {
    pub slot: CommandSlot,
    pub command: u8,
    pub address: u32,
    pub config: CommandConfig,
    pub auto_increment_address: bool,
    pub data: TransferData<'a>,
}

impl<'a> Transfer<'a> {
    pub const fn command_only(
        slot: CommandSlot,
        command: u8,
        address: u32,
        config: CommandConfig,
    ) -> Self {
        Self {
            slot,
            command,
            address,
            config,
            auto_increment_address: false,
            data: TransferData::None,
        }
    }

    pub fn read(
        slot: CommandSlot,
        command: u8,
        address: u32,
        config: CommandConfig,
        out: &'a mut [u8],
    ) -> Self {
        Self {
            slot,
            command,
            address,
            config,
            auto_increment_address: true,
            data: TransferData::Read(out),
        }
    }

    pub const fn write(
        slot: CommandSlot,
        command: u8,
        address: u32,
        config: CommandConfig,
        data: &'a [u8],
    ) -> Self {
        Self {
            slot,
            command,
            address,
            config,
            auto_increment_address: true,
            data: TransferData::Write(data),
        }
    }

    pub fn data_len(&self) -> usize {
        match &self.data {
            TransferData::None => 0,
            TransferData::Read(out) => out.len(),
            TransferData::Write(data) => data.len(),
        }
    }
}

fn pack_le_word(data: &[u8]) -> u32 {
    let mut word = [0u8; 4];
    word[..data.len()].copy_from_slice(data);
    u32::from_le_bytes(word)
}

impl<'d, T: Instance> Mpi<'d, T> {
    pub fn transfer(&mut self, transfer: &mut Transfer<'_>) -> Result<(), Error> {
        let slot = transfer.slot;
        let command = transfer.command;
        let address = transfer.address;
        let config = transfer.config;
        let auto_increment_address = transfer.auto_increment_address;

        match &mut transfer.data {
            TransferData::None => self.transfer_command_only(slot, command, address, config),
            TransferData::Read(out) => {
                self.transfer_read(slot, command, address, config, auto_increment_address, out)
            }
            TransferData::Write(data) => {
                self.transfer_write(slot, command, address, config, auto_increment_address, data)
            }
        }
    }

    fn transfer_command_only(
        &mut self,
        slot: CommandSlot,
        command: u8,
        address: u32,
        config: CommandConfig,
    ) -> Result<(), Error> {
        if slot != CommandSlot::Cmd1 {
            return Err(Error::InvalidConfiguration);
        }
        self.configure_command_for(slot, config)?;
        self.set_command(command, address)
    }

    fn transfer_read(
        &mut self,
        slot: CommandSlot,
        command: u8,
        base_address: u32,
        config: CommandConfig,
        auto_increment_address: bool,
        out: &mut [u8],
    ) -> Result<(), Error> {
        if out.is_empty() {
            return Ok(());
        }
        if slot != CommandSlot::Cmd1 {
            return Err(Error::InvalidConfiguration);
        }

        let mut done = 0usize;
        while done < out.len() {
            let step = min(FIFO_SIZE_BYTES, out.len() - done);
            let address = if auto_increment_address {
                base_address.saturating_add(done as u32)
            } else {
                base_address
            };

            self.clear_fifo(FifoClear::Rx);
            self.configure_command_for(slot, config)?;
            self.set_data_len_for(slot, step)?;
            self.set_command(command, address)?;

            let mut idx = 0usize;
            while idx < step {
                let word = self.read_word().to_le_bytes();
                let take = min(4, step - idx);
                out[done + idx..done + idx + take].copy_from_slice(&word[..take]);
                idx += take;
            }
            done += step;
        }
        Ok(())
    }

    fn transfer_write(
        &mut self,
        slot: CommandSlot,
        command: u8,
        base_address: u32,
        config: CommandConfig,
        auto_increment_address: bool,
        data: &[u8],
    ) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        if slot != CommandSlot::Cmd1 {
            return Err(Error::InvalidConfiguration);
        }

        let mut done = 0usize;
        while done < data.len() {
            let step = min(FIFO_SIZE_BYTES, data.len() - done);
            let address = if auto_increment_address {
                base_address.saturating_add(done as u32)
            } else {
                base_address
            };

            self.clear_fifo(FifoClear::Tx);
            let mut idx = 0usize;
            while idx < step {
                let take = min(4, step - idx);
                self.write_word(pack_le_word(&data[done + idx..done + idx + take]));
                idx += take;
            }

            self.configure_command_for(slot, config)?;
            self.set_data_len_for(slot, step)?;
            self.set_command(command, address)?;
            done += step;
        }

        Ok(())
    }
}
