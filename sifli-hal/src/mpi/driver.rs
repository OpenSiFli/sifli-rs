use core::hint::spin_loop;

use embassy_hal_internal::{into_ref, PeripheralRef};

use super::nor::{MpiNorFlash, NorExecBackend};
use super::shared::*;
use super::types::*;
use super::{Instance, Regs};
use crate::{rcc, Peripheral};

pub struct Mpi<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Mpi<'d, T> {
    pub fn with_config(
        inner: impl Peripheral<P = T> + 'd,
        init: MpiInitConfig,
    ) -> Result<Self, Error> {
        if init.reset && !init.allow_reset_from_xip && running_from_instance_code_bus_flash::<T>() {
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
        regs.hrccr().modify(|w| {
            w.set_dmode(cfg.data_mode.bits());
            w.set_dcyc(cfg.dummy_cycles);
            w.set_absize(cfg.alternate_size.bits());
            w.set_abmode(cfg.alternate_mode.bits());
            w.set_adsize(cfg.address_size.bits());
            w.set_admode(cfg.address_mode.bits());
            w.set_imode(cfg.instruction_mode.bits());
        });
        regs.hcmdr().modify(|w| w.set_rcmd(read_cmd));
    }

    fn configure_ahb_write_raw(&mut self, write_cmd: u8, cfg: AhbCommandConfig) {
        let regs = self.regs();
        regs.hwccr().modify(|w| {
            w.set_dmode(cfg.data_mode.bits());
            w.set_dcyc(cfg.dummy_cycles);
            w.set_absize(cfg.alternate_size.bits());
            w.set_abmode(cfg.alternate_mode.bits());
            w.set_adsize(cfg.address_size.bits());
            w.set_admode(cfg.address_mode.bits());
            w.set_imode(cfg.instruction_mode.bits());
        });
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
        if cfg.dummy_cycles > 31 {
            return Err(Error::InvalidConfiguration);
        }
        self.configure_ahb_read_raw(read_cmd, cfg);
        Ok(())
    }

    pub fn configure_ahb_write_command(
        &mut self,
        write_cmd: u8,
        cfg: AhbCommandConfig,
    ) -> Result<(), Error> {
        if cfg.dummy_cycles > 31 {
            return Err(Error::InvalidConfiguration);
        }
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
        match mode {
            FifoClear::Rx => self.regs().fifocr().modify(|w| w.set_rxclr(true)),
            FifoClear::Tx => self.regs().fifocr().modify(|w| w.set_txclr(true)),
            FifoClear::RxTx => self.regs().fifocr().modify(|w| {
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
        if cfg.dummy_cycles > 31 {
            return Err(Error::InvalidConfiguration);
        }
        match slot {
            CommandSlot::Cmd1 => self.regs().ccr1().modify(|w| {
                w.set_fmode(cfg.function_mode.is_write());
                w.set_dmode(cfg.data_mode.bits());
                w.set_dcyc(cfg.dummy_cycles);
                w.set_absize(cfg.alternate_size.bits());
                w.set_abmode(cfg.alternate_mode.bits());
                w.set_adsize(cfg.address_size.bits());
                w.set_admode(cfg.address_mode.bits());
                w.set_imode(cfg.instruction_mode.bits());
            }),
            CommandSlot::Cmd2 => self.regs().ccr2().modify(|w| {
                w.set_fmode(cfg.function_mode.is_write());
                w.set_dmode(cfg.data_mode.bits());
                w.set_dcyc(cfg.dummy_cycles);
                w.set_absize(cfg.alternate_size.bits());
                w.set_abmode(cfg.alternate_mode.bits());
                w.set_adsize(cfg.address_size.bits());
                w.set_admode(cfg.address_mode.bits());
                w.set_imode(cfg.instruction_mode.bits());
            }),
        }
        Ok(())
    }

    pub fn set_data_len_for(&mut self, slot: CommandSlot, len: usize) -> Result<(), Error> {
        if len == 0 || len > MAX_DLEN_BYTES {
            return Err(Error::InvalidLength);
        }
        match slot {
            CommandSlot::Cmd1 => self.regs().dlr1().write(|w| w.set_dlen((len - 1) as u32)),
            CommandSlot::Cmd2 => self.regs().dlr2().write(|w| w.set_dlen((len - 1) as u32)),
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
        if !ram_wait_not_busy(regs, STATUS_MATCH_TIMEOUT_POLLS) {
            return Err(Error::Timeout);
        }
        regs.scr().write(|w| w.set_tcfc(true));
        regs.ar1().write(|w| w.set_addr(addr));
        regs.cmdr1().write(|w| w.set_cmd(cmd));
        // Wait for transfer complete
        for _ in 0..STATUS_MATCH_TIMEOUT_POLLS {
            if regs.sr().read().tcf() {
                regs.scr().write(|w| w.set_tcfc(true));
                return Ok(());
            }
            spin_loop();
        }
        Err(Error::Timeout)
    }

    pub fn try_set_command_no_wait(
        &mut self,
        slot: CommandSlot,
        cmd: u8,
        addr: u32,
    ) -> Result<(), Error> {
        let regs = self.regs();
        if !ram_wait_not_busy(regs, STATUS_MATCH_TIMEOUT_POLLS) {
            return Err(Error::Timeout);
        }
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
        Ok(())
    }

    /// Convert to NOR flash with common defaults.
    ///
    /// This is the simplest entry point for standard SPI NOR parts.
    pub fn into_nor_flash_default<const WRITE_GRAN: usize, const ERASE_GRAN: usize>(
        self,
        capacity: usize,
    ) -> Result<MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
        self.into_nor_flash(NorFlashConfig::new(capacity))
    }

    fn validate_nor_flash_config<const WRITE_GRAN: usize, const ERASE_GRAN: usize>(
        config: &NorFlashConfig,
        window_size: usize,
    ) -> Result<(), Error> {
        if config.capacity > window_size {
            return Err(Error::CapacityExceedsWindow);
        }

        if WRITE_GRAN == 0
            || ERASE_GRAN == 0
            || WRITE_GRAN > FIFO_SIZE_BYTES
            || config.capacity == 0
            || config.page_size == 0
            || config.page_size < WRITE_GRAN
            || !config.page_size.is_multiple_of(WRITE_GRAN)
            || config.max_ready_polls == 0
            || config.dma_threshold_bytes == 0
        {
            return Err(Error::InvalidConfiguration);
        }

        let needs_4byte_address = config.capacity > NOR_FLASH_MAX_3B_CAPACITY_BYTES;
        if needs_4byte_address
            && (config.address_size != AddressSize::FourBytes
                || config.commands.ahb_read_4byte.is_none())
        {
            return Err(Error::InvalidConfiguration);
        }

        Ok(())
    }

    pub fn into_nor_flash<const WRITE_GRAN: usize, const ERASE_GRAN: usize>(
        self,
        config: NorFlashConfig,
    ) -> Result<MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
        let window_size = instance_code_bus_window_size::<T>();
        Self::validate_nor_flash_config::<WRITE_GRAN, ERASE_GRAN>(&config, window_size)?;

        let mut this = self;
        let address_size = config.address_size;
        let ahb_read_cmd = if address_size == AddressSize::FourBytes {
            config
                .commands
                .ahb_read_4byte
                .unwrap_or(config.commands.ahb_read)
        } else {
            config.commands.ahb_read
        };
        let running_from_xip = running_from_instance_code_bus_flash::<T>();
        let needs_4byte_address = config.capacity > NOR_FLASH_MAX_3B_CAPACITY_BYTES;

        if needs_4byte_address {
            if running_from_xip && !config.allow_preconfigured_4byte_in_xip {
                return Err(Error::RequiresPreconfigured4ByteMode);
            }

            if let Some(cmd) = config.commands.enter_4byte_address {
                // When executing from this instance's XIP window, changing the flash's
                // global address mode can desynchronize instruction fetch and fault.
                // In that case, keep the current mode and assume boot/runtime already
                // configured a compatible 4-byte read path.
                if !running_from_xip
                    && !ram_wrapper_issue_simple_cmd(this.regs(), cmd, config.max_ready_polls)
                {
                    return Err(Error::Timeout);
                }
            }
        }

        if !running_from_xip {
            this.configure_ahb_read_command(
                ahb_read_cmd,
                AhbCommandConfig::single_io(address_size),
            )?;
        }

        Ok(MpiNorFlash {
            mpi: this,
            config,
            exec: NorExecBackend::from_running_from_xip(running_from_xip),
        })
    }
}
