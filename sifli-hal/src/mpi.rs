//! MPI (Multi-port memory interface) HAL.
//!
//! This module provides:
//! - A low-level register-oriented MPI driver (`Mpi`), similar to SiFli C HAL `hal_mpi`.
//! - A minimal NOR flash wrapper (`MpiNorFlash`) built on top of manual MPI commands.
//! - `embedded-storage` traits for NOR flash read/write/erase workflows.
//!
//! # XIP Safety
//!
//! When executing code from the same flash that is being programmed/erased (XIP scenario),
//! all MPI register operations must be executed from RAM, not from flash. This module
//! provides RAM-resident functions (marked with `#[link_section = ".data.ramfunc"]`) that
//! handle the complete MPI command sequences without any flash code fetches during the
//! critical sections.

use core::arch::asm;
use core::cmp::min;
use core::hint::spin_loop;

use embassy_hal_internal::{into_ref, PeripheralRef};
use embedded_storage::nor_flash::{
    check_erase, check_write, ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::pac::mpi::Mpi as Regs;
use crate::{rcc, Peripheral};

const FIFO_SIZE_BYTES: usize = 64;
const MAX_DLEN_BYTES: usize = 0x000f_ffff + 1;
const STATUS_MATCH_TIMEOUT_POLLS: u32 = 2_000_000;
const NOR_FLASH_MAX_3B_CAPACITY_BYTES: usize = 16 * 1024 * 1024;
const CODE_BUS_FLASH_START: usize = 0x1000_0000;
const CODE_BUS_FLASH_END: usize = 0x2000_0000;
const MPI1_CODE_BUS_BASE: usize = CODE_BUS_FLASH_START;
const MPI1_CODE_BUS_END: usize = 0x1200_0000;
const MPI2_CODE_BUS_BASE: usize = MPI1_CODE_BUS_END;
const MPI2_CODE_BUS_END: usize = CODE_BUS_FLASH_END;

/// NOR flash WIP (Write In Progress) bit mask in status register.
const NOR_FLASH_WIP_MASK: u32 = 0x01;

#[inline(always)]
fn running_from_instance_code_bus_flash<T: SealedInstance>() -> bool {
    let pc = running_from_instance_code_bus_flash::<T> as *const () as usize;
    (T::code_bus_base()..T::code_bus_end()).contains(&pc)
}

/// RAM function: save PRIMASK and disable interrupts.
#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_irq_save_disable() -> u32 {
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

/// RAM function: restore interrupt state from saved PRIMASK.
///
/// This function also ensures proper synchronization before returning to XIP execution:
/// - DSB (Data Synchronization Barrier): Ensures all memory accesses complete
/// - ISB (Instruction Synchronization Barrier): Flushes the pipeline
#[inline(always)]
#[link_section = ".data.ramfunc"]
fn ram_irq_restore(saved_primask: u32) {
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

/// RAM function: Wait for MPI controller to be idle (not busy).
/// This must be called before issuing manual commands when running from XIP.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wait_not_busy(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if !regs.sr().read().busy() {
            return true;
        }
        spin_loop();
    }
    false
}

/// RAM function: Wait for transfer complete flag (TCF) and clear it.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wait_tcf(regs: Regs, max_polls: u32) -> bool {
    for _ in 0..max_polls {
        if regs.sr().read().tcf() {
            regs.scr().write(|w| w.set_tcfc(true));
            return true;
        }
        spin_loop();
    }
    false
}

/// RAM function: Wait for status match flag (SMF) and clear it.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wait_smf(regs: Regs, max_polls: u32) -> bool {
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

/// RAM function: Configure CCR1 for a simple command (instruction only, no address, no data).
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_configure_simple_cmd(regs: Regs) {
    regs.ccr1().modify(|w| {
        w.set_fmode(false); // read mode
        w.set_dmode(0); // no data
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(0);
        w.set_admode(0); // no address
        w.set_imode(1); // single line instruction
    });
}

/// RAM function: Configure CCR1 for status read (instruction + 1 byte data).
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_configure_status_read(regs: Regs) {
    regs.ccr1().modify(|w| {
        w.set_fmode(false); // read mode
        w.set_dmode(1); // single line data
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(0);
        w.set_admode(0); // no address
        w.set_imode(1); // single line instruction
    });
}

/// RAM function: Configure CCR1 for data write with address (page program).
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_configure_write_with_addr(regs: Regs, addr_size: u8) {
    regs.ccr1().modify(|w| {
        w.set_fmode(true); // write mode
        w.set_dmode(1); // single line data
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(addr_size);
        w.set_admode(1); // single line address
        w.set_imode(1); // single line instruction
    });
}

/// RAM function: Configure CCR1 for command with address only (erase).
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_configure_addr_only(regs: Regs, addr_size: u8) {
    regs.ccr1().modify(|w| {
        w.set_fmode(false); // read mode (doesn't matter for erase)
        w.set_dmode(0); // no data
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(addr_size);
        w.set_admode(1); // single line address
        w.set_imode(1); // single line instruction
    });
}

/// RAM function: Configure CCR2 for status polling.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_configure_cmd2_status_poll(regs: Regs, read_status_cmd: u8) {
    regs.ccr2().modify(|w| {
        w.set_fmode(false); // read mode
        w.set_dmode(1); // single line data
        w.set_dcyc(0);
        w.set_absize(0);
        w.set_abmode(0);
        w.set_adsize(0);
        w.set_admode(0); // no address
        w.set_imode(1); // single line instruction
    });
    regs.dlr2().write(|w| w.set_dlen(0)); // 1 byte
    regs.cmdr2().write(|w| w.set_cmd(read_status_cmd));
    regs.smr().write(|w| w.set_status(0)); // expect WIP=0
    regs.smkr().write(|w| w.set_mask(NOR_FLASH_WIP_MASK));
}

/// RAM function: Issue a simple command and wait for completion.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_issue_simple_cmd(regs: Regs, cmd: u8, max_polls: u32) -> bool {
    ram_configure_simple_cmd(regs);
    regs.scr().write(|w| w.set_tcfc(true));
    regs.ar1().write(|w| w.set_addr(0));
    regs.cmdr1().write(|w| w.set_cmd(cmd));
    ram_wait_tcf(regs, max_polls)
}

/// RAM function: Read JEDEC ID - complete operation in RAM.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_read_jedec_id(regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()> {
    // Wait for MPI to be idle before issuing manual command (critical for XIP)
    if !ram_wait_not_busy(regs, max_polls) {
        return Err(());
    }

    // Clear RX FIFO
    regs.fifocr().modify(|w| w.set_rxclr(true));

    // Configure for status-like read (no address, 3 bytes data)
    ram_configure_status_read(regs);
    regs.dlr1().write(|w| w.set_dlen(2)); // 3 bytes (dlen = len - 1)

    // Issue command
    regs.scr().write(|w| w.set_tcfc(true));
    regs.ar1().write(|w| w.set_addr(0));
    regs.cmdr1().write(|w| w.set_cmd(cmd));

    // Wait for completion
    if !ram_wait_tcf(regs, max_polls) {
        return Err(());
    }

    // Read result
    Ok(regs.dr().read().data() & 0x00ff_ffff)
}

/// RAM function: Read status register - complete operation in RAM.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_read_status(regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()> {
    // Clear RX FIFO
    regs.fifocr().modify(|w| w.set_rxclr(true));

    // Configure for status read
    ram_configure_status_read(regs);
    regs.dlr1().write(|w| w.set_dlen(0)); // 1 byte

    // Issue command
    regs.scr().write(|w| w.set_tcfc(true));
    regs.ar1().write(|w| w.set_addr(0));
    regs.cmdr1().write(|w| w.set_cmd(cmd));

    // Wait for completion
    if !ram_wait_tcf(regs, max_polls) {
        return Err(());
    }

    // Read result
    Ok((regs.dr().read().data() & 0xff) as u8)
}

/// RAM function: Program a chunk with hardware status polling.
///
/// This function performs the complete page program sequence:
/// 1. Write Enable (WREN)
/// 2. Page Program with data
/// 3. Hardware status polling via CMD2 until WIP=0
#[inline(never)]
#[link_section = ".data.ramfunc"]
#[allow(clippy::too_many_arguments)]
fn ram_program_chunk(
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

    // Step 1: Write Enable
    if !ram_issue_simple_cmd(regs, wren_cmd, max_polls) {
        return false;
    }

    // Step 2: Prepare TX FIFO with data
    regs.fifocr().modify(|w| w.set_txclr(true));

    // Write data to FIFO
    let mut idx = 0;
    while idx < data.len() {
        let take = min(4, data.len() - idx);
        let mut word = data[idx] as u32;
        if take > 1 {
            word |= (data[idx + 1] as u32) << 8;
        }
        if take > 2 {
            word |= (data[idx + 2] as u32) << 16;
        }
        if take > 3 {
            word |= (data[idx + 3] as u32) << 24;
        }
        regs.dr().write(|w| w.set_data(word));
        idx += take;
    }

    // Configure CMD1 for page program
    ram_configure_write_with_addr(regs, addr_size);
    regs.dlr1().write(|w| w.set_dlen((data.len() - 1) as u32));

    // Configure CMD2 for status polling
    ram_configure_cmd2_status_poll(regs, read_status_cmd);

    // Enable CMD2 and status match
    regs.cr().modify(|w| {
        w.set_cmd2e(true);
        w.set_sme2(true);
    });

    // Clear flags
    regs.scr().write(|w| {
        w.set_tcfc(true);
        w.set_smfc(true);
    });

    // Issue page program command
    regs.ar1().write(|w| w.set_addr(addr));
    regs.cmdr1().write(|w| w.set_cmd(program_cmd));

    // Wait for status match (WIP=0)
    let ok = ram_wait_smf(regs, max_polls);

    // Disable CMD2 and status match
    regs.cr().modify(|w| {
        w.set_cmd2e(false);
        w.set_sme2(false);
    });
    regs.scr().write(|w| {
        w.set_smfc(true);
        w.set_tcfc(true);
    });

    ok
}

/// RAM function: Erase a sector with hardware status polling.
///
/// This function performs the complete sector erase sequence:
/// 1. Write Enable (WREN)
/// 2. Sector Erase
/// 3. Hardware status polling via CMD2 until WIP=0
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_erase_sector(
    regs: Regs,
    wren_cmd: u8,
    erase_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    max_polls: u32,
) -> bool {
    // Step 1: Write Enable
    if !ram_issue_simple_cmd(regs, wren_cmd, max_polls) {
        return false;
    }

    // Configure CMD1 for sector erase (address only, no data)
    ram_configure_addr_only(regs, addr_size);

    // Configure CMD2 for status polling
    ram_configure_cmd2_status_poll(regs, read_status_cmd);

    // Enable CMD2 and status match
    regs.cr().modify(|w| {
        w.set_cmd2e(true);
        w.set_sme2(true);
    });

    // Clear flags
    regs.scr().write(|w| {
        w.set_tcfc(true);
        w.set_smfc(true);
    });

    // Issue sector erase command
    regs.ar1().write(|w| w.set_addr(addr));
    regs.cmdr1().write(|w| w.set_cmd(erase_cmd));

    // Wait for status match (WIP=0)
    let ok = ram_wait_smf(regs, max_polls);

    // Disable CMD2 and status match
    regs.cr().modify(|w| {
        w.set_cmd2e(false);
        w.set_sme2(false);
    });
    regs.scr().write(|w| {
        w.set_smfc(true);
        w.set_tcfc(true);
    });

    ok
}

// ============================================================================
// RAM-resident wrapper functions that include interrupt disable/restore
// ============================================================================
//
// These functions wrap the core RAM operations and handle interrupt management
// entirely within RAM, avoiding any Flash code execution during critical sections.

/// RAM wrapper: Read JEDEC ID with interrupt protection.
///
/// This function disables interrupts, performs the operation, and restores interrupts,
/// all within RAM to ensure XIP safety.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wrapper_read_jedec_id(regs: Regs, cmd: u8, max_polls: u32) -> Result<u32, ()> {
    let primask = ram_irq_save_disable();

    // Perform the operation
    let result = ram_read_jedec_id(regs, cmd, max_polls);

    // Restore interrupts
    ram_irq_restore(primask);

    result
}

/// RAM wrapper: Read status register with interrupt protection.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wrapper_read_status(regs: Regs, cmd: u8, max_polls: u32) -> Result<u8, ()> {
    let primask = ram_irq_save_disable();

    let result = ram_read_status(regs, cmd, max_polls);

    ram_irq_restore(primask);

    result
}

/// RAM wrapper: Program chunk with interrupt protection.
#[inline(never)]
#[link_section = ".data.ramfunc"]
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
) -> bool {
    let primask = ram_irq_save_disable();

    let result = ram_program_chunk(
        regs,
        wren_cmd,
        program_cmd,
        read_status_cmd,
        addr,
        addr_size,
        data,
        max_polls,
    );

    ram_irq_restore(primask);

    result
}

/// RAM wrapper: Erase sector with interrupt protection.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wrapper_erase_sector(
    regs: Regs,
    wren_cmd: u8,
    erase_cmd: u8,
    read_status_cmd: u8,
    addr: u32,
    addr_size: u8,
    max_polls: u32,
) -> bool {
    let primask = ram_irq_save_disable();

    let result = ram_erase_sector(
        regs,
        wren_cmd,
        erase_cmd,
        read_status_cmd,
        addr,
        addr_size,
        max_polls,
    );

    ram_irq_restore(primask);

    result
}

/// RAM wrapper: Issue a simple command with interrupt protection.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wrapper_issue_simple_cmd(regs: Regs, cmd: u8, max_polls: u32) -> bool {
    let primask = ram_irq_save_disable();
    let result = ram_issue_simple_cmd(regs, cmd, max_polls);
    ram_irq_restore(primask);
    result
}

/// RAM wrapper: Wait until flash is ready (WIP=0) with interrupt protection.
///
/// This performs the entire polling loop in RAM to avoid XIP conflicts.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wrapper_wait_ready(regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool {
    let primask = ram_irq_save_disable();
    let ok = ram_wait_ready_sme1(regs, read_status_cmd, max_polls);
    ram_irq_restore(primask);
    ok
}

/// RAM function: wait ready using CMD1 status match loop (SME1).
///
/// This avoids software-side status parsing loops and lets hardware match WIP=0.
#[inline(never)]
#[link_section = ".data.ramfunc"]
fn ram_wait_ready_sme1(regs: Regs, read_status_cmd: u8, max_polls: u32) -> bool {
    // Configure CMD1 as status read (RDSR)
    ram_configure_status_read(regs);
    regs.dlr1().write(|w| w.set_dlen(0)); // 1 byte

    // Match condition: (status & WIP_MASK) == 0
    regs.smr().write(|w| w.set_status(0));
    regs.smkr().write(|w| w.set_mask(NOR_FLASH_WIP_MASK));

    // Ensure only CMD1 status match is active for this flow.
    regs.cr().modify(|w| {
        w.set_cmd2e(false);
        w.set_sme2(false);
        w.set_sme1(true);
    });

    // Clear stale flags before issuing command.
    regs.scr().write(|w| {
        w.set_smfc(true);
        w.set_tcfc(true);
    });

    regs.ar1().write(|w| w.set_addr(0));
    regs.cmdr1().write(|w| w.set_cmd(read_status_cmd));

    let ok = ram_wait_smf(regs, max_polls);

    // Restore state and clear flags no matter success/timeout.
    regs.cr().modify(|w| w.set_sme1(false));
    regs.scr().write(|w| {
        w.set_smfc(true);
        w.set_tcfc(true);
    });

    ok
}

/// MPI/flash error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Operation timed out.
    Timeout,
    /// Input configuration is invalid.
    InvalidConfiguration,
    /// Input data length is invalid for the requested operation.
    InvalidLength,
    /// Input address or length is not properly aligned.
    NotAligned,
    /// Input address or length is out of bounds.
    OutOfBounds,
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::Timeout | Self::InvalidConfiguration | Self::InvalidLength => {
                NorFlashErrorKind::Other
            }
        }
    }
}

impl From<NorFlashErrorKind> for Error {
    fn from(value: NorFlashErrorKind) -> Self {
        match value {
            NorFlashErrorKind::NotAligned => Self::NotAligned,
            NorFlashErrorKind::OutOfBounds => Self::OutOfBounds,
            NorFlashErrorKind::Other => Self::InvalidConfiguration,
            _ => Self::InvalidConfiguration,
        }
    }
}

/// SPI/Quad phase mode used by a command phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PhaseMode {
    /// No phase.
    None,
    /// 1-bit mode.
    Single,
    /// 2-bit mode.
    Dual,
    /// 4-bit mode.
    Quad,
    /// 4-bit DDR mode.
    QuadDdr,
}

impl PhaseMode {
    const fn bits(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Single => 1,
            Self::Dual => 2,
            Self::Quad => 3,
            Self::QuadDdr => 7,
        }
    }
}

/// Address byte count for command address phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddressSize {
    /// 1-byte address.
    OneByte,
    /// 2-byte address.
    TwoBytes,
    /// 3-byte address.
    ThreeBytes,
    /// 4-byte address.
    FourBytes,
}

impl AddressSize {
    const fn bits(self) -> u8 {
        match self {
            Self::OneByte => 0,
            Self::TwoBytes => 1,
            Self::ThreeBytes => 2,
            Self::FourBytes => 3,
        }
    }
}

/// MPI command function mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FunctionMode {
    /// Read phase.
    Read,
    /// Write phase.
    Write,
}

impl FunctionMode {
    const fn is_write(self) -> bool {
        matches!(self, Self::Write)
    }
}

/// FIFO clear mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoClear {
    /// Clear RX FIFO.
    Rx,
    /// Clear TX FIFO.
    Tx,
    /// Clear both RX and TX FIFOs.
    RxTx,
}

/// Select which manual command slot to configure/use.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandSlot {
    /// Command slot 1.
    Cmd1,
    /// Command slot 2.
    Cmd2,
}

/// Configuration of one manual command sequence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandConfig {
    /// Function mode.
    pub function_mode: FunctionMode,
    /// Instruction mode.
    pub instruction_mode: PhaseMode,
    /// Address mode.
    pub address_mode: PhaseMode,
    /// Address byte width.
    pub address_size: AddressSize,
    /// Alternate-byte mode.
    pub alternate_mode: PhaseMode,
    /// Alternate-byte size.
    pub alternate_size: AddressSize,
    /// Data mode.
    pub data_mode: PhaseMode,
    /// Dummy cycles (0..31).
    pub dummy_cycles: u8,
}

impl CommandConfig {
    /// Command config with no data and no address phase.
    pub const fn simple_cmd() -> Self {
        Self {
            function_mode: FunctionMode::Read,
            instruction_mode: PhaseMode::Single,
            address_mode: PhaseMode::None,
            address_size: AddressSize::OneByte,
            alternate_mode: PhaseMode::None,
            alternate_size: AddressSize::OneByte,
            data_mode: PhaseMode::None,
            dummy_cycles: 0,
        }
    }
}

/// Low-level MPI HAL driver.
pub struct Mpi<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Mpi<'d, T> {
    /// Create and initialize an MPI peripheral.
    ///
    /// This method enables and resets the MPI peripheral, then applies default timing registers.
    ///
    /// # Warning
    ///
    /// Do not call this for the same MPI instance currently used as XIP code source
    /// (for example executing from `0x1200_0000` while initializing `MPI2` on SF32LB52),
    /// otherwise a fault is expected.
    pub fn new(inner: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(inner);
        rcc::enable_and_reset::<T>();

        let mut this = Self { _inner: inner };
        this.apply_default_config();
        this
    }

    /// Take ownership of MPI without reset and without changing existing command timing.
    ///
    /// This is intended for advanced/XIP scenarios where resetting or reconfiguring MPI
    /// may break ongoing code fetches.
    pub fn new_without_reset(inner: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(inner);
        rcc::enable::<T>();
        Self { _inner: inner }
    }

    /// Apply common default timing and interval registers.
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

    /// Configure default AHB memory-mapped read timing/format.
    ///
    /// This aligns memory-mapped reads (`code_bus_base`) with the selected address width.
    pub fn configure_ahb_read(&mut self, read_cmd: u8, address_size: AddressSize) {
        let regs = self.regs();
        regs.hrccr().modify(|w| {
            w.set_dmode(1); // single-line data
            w.set_dcyc(0); // normal-read opcode, no dummy cycle
            w.set_absize(0);
            w.set_abmode(0);
            w.set_adsize(address_size.bits());
            w.set_admode(1); // single-line address
            w.set_imode(1); // single-line instruction
        });
        regs.hcmdr().modify(|w| w.set_rcmd(read_cmd));
    }

    #[inline(always)]
    fn regs(&self) -> Regs {
        T::regs()
    }

    /// Enable or disable MPI.
    pub fn enable(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_en(enable));
    }

    /// Configure prescaler divider.
    pub fn set_prescaler_div(&mut self, div: u8) {
        self.regs().psclr().write(|w| w.set_div(div));
    }

    /// Configure command interval.
    pub fn set_command_interval(&mut self, interval1: u16, interval2: u16) {
        self.regs().cir().write(|w| {
            w.set_interval1(interval1);
            w.set_interval2(interval2);
        });
    }

    /// Configure command auto-loop count.
    pub fn set_loop_count(&mut self, count: u8) {
        self.regs().cr2().write(|w| w.set_loop_(count));
    }

    /// Clear command/status FIFO flags.
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

    /// Configure command format for a selected command slot.
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

    /// Set command data length in bytes for a selected command slot.
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

    /// Write one word to FIFO.
    pub fn write_word(&mut self, value: u32) {
        self.regs().dr().write(|w| w.set_data(value));
    }

    /// Read one word from FIFO.
    pub fn read_word(&self) -> u32 {
        self.regs().dr().read().data()
    }

    /// Enable or disable command 2 sequence.
    pub fn enable_command2(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_cmd2e(enable));
    }

    /// Enable or disable status-match on command 2.
    pub fn enable_status_match_cmd2(&mut self, enable: bool) {
        self.regs().cr().modify(|w| w.set_sme2(enable));
    }

    /// Set status-match mask register.
    pub fn set_status_mask(&mut self, mask: u32) {
        self.regs().smkr().write(|w| w.set_mask(mask));
    }

    /// Set status-match expected value.
    pub fn set_status_match(&mut self, status: u32) {
        self.regs().smr().write(|w| w.set_status(status));
    }

    /// Configure command/address for command 1, then wait for transfer complete.
    ///
    /// This is a simple blocking command that waits for the transfer complete flag (TCF).
    /// For XIP-safe flash busy polling, use [`MpiNorFlash::wait_ready`].
    pub fn set_command(&mut self, cmd: u8, addr: u32) -> Result<(), Error> {
        let regs = self.regs();
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

    /// Set command/address only, without waiting for transfer completion.
    pub fn set_command_no_wait(&mut self, slot: CommandSlot, cmd: u8, addr: u32) {
        let regs = self.regs();
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

    /// Turn this low-level driver into a NOR flash implementation.
    pub fn into_nor_flash<const WRITE_GRAN: usize, const ERASE_GRAN: usize>(
        self,
        config: NorFlashConfig,
    ) -> Result<MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
        if WRITE_GRAN == 0
            || ERASE_GRAN == 0
            || WRITE_GRAN > FIFO_SIZE_BYTES
            || config.capacity == 0
            || config.page_size == 0
            || config.page_size < WRITE_GRAN
            || !config.page_size.is_multiple_of(WRITE_GRAN)
            || config.max_ready_polls == 0
            || (config.capacity > NOR_FLASH_MAX_3B_CAPACITY_BYTES
                && config.address_size != AddressSize::FourBytes)
        {
            return Err(Error::InvalidConfiguration);
        }

        let mut this = self;
        let running_from_xip = running_from_instance_code_bus_flash::<T>();

        if config.capacity > NOR_FLASH_MAX_3B_CAPACITY_BYTES {
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

        let ahb_read_cmd = match config.address_size {
            AddressSize::FourBytes => config
                .commands
                .ahb_read_4byte
                .unwrap_or(config.commands.ahb_read),
            _ => config.commands.ahb_read,
        };
        if !running_from_xip {
            this.configure_ahb_read(ahb_read_cmd, config.address_size);
        }

        Ok(MpiNorFlash { mpi: this, config })
    }
}

/// NOR flash command set used by [`MpiNorFlash`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NorFlashCommandSet {
    /// Write enable command.
    pub write_enable: u8,
    /// Enter 4-byte address mode command. `None` means skip.
    pub enter_4byte_address: Option<u8>,
    /// Read status register command.
    pub read_status: u8,
    /// AHB memory-mapped read command.
    pub ahb_read: u8,
    /// Optional 4-byte AHB memory-mapped read command.
    pub ahb_read_4byte: Option<u8>,
    /// Page program command.
    pub page_program: u8,
    /// Optional 4-byte page program command.
    pub page_program_4byte: Option<u8>,
    /// Sector erase command.
    pub sector_erase: u8,
    /// Optional 4-byte sector erase command.
    pub sector_erase_4byte: Option<u8>,
    /// Read JEDEC ID command.
    pub read_jedec_id: u8,
}

impl NorFlashCommandSet {
    /// Common SPI NOR command set.
    pub const fn common_spi_nor() -> Self {
        Self {
            write_enable: 0x06,
            enter_4byte_address: Some(0xB7),
            read_status: 0x05,
            ahb_read: 0x03,
            ahb_read_4byte: Some(0x13),
            page_program: 0x02,
            page_program_4byte: Some(0x12),
            sector_erase: 0x20,
            sector_erase_4byte: Some(0x21),
            read_jedec_id: 0x9f,
        }
    }
}

impl Default for NorFlashCommandSet {
    fn default() -> Self {
        Self::common_spi_nor()
    }
}

/// Configuration of [`MpiNorFlash`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NorFlashConfig {
    /// Total flash capacity in bytes.
    pub capacity: usize,
    /// Page size in bytes (used to split page-program operations).
    pub page_size: usize,
    /// Address byte width used by commands.
    pub address_size: AddressSize,
    /// Max number of polls for waiting WIP=0.
    pub max_ready_polls: u32,
    /// Command opcodes.
    pub commands: NorFlashCommandSet,
}

impl NorFlashConfig {
    /// Create a config with common NOR defaults.
    pub const fn new(capacity: usize) -> Self {
        Self {
            capacity,
            page_size: 256,
            address_size: AddressSize::ThreeBytes,
            max_ready_polls: 2_000_000,
            commands: NorFlashCommandSet::common_spi_nor(),
        }
    }
}

/// Logical storage partition in a NOR flash.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NorFlashPartition {
    /// Start offset within the flash, in bytes.
    pub offset: u32,
    /// Partition size, in bytes.
    pub size: u32,
}

impl NorFlashPartition {
    /// Create a partition descriptor.
    pub const fn new(offset: u32, size: u32) -> Self {
        Self { offset, size }
    }
}

/// NOR flash wrapper built on top of manual MPI commands.
pub struct MpiNorFlash<'d, T: Instance, const WRITE_GRAN: usize = 1, const ERASE_GRAN: usize = 4096>
{
    mpi: Mpi<'d, T>,
    config: NorFlashConfig,
}

impl<'d, T: Instance, const WRITE_GRAN: usize, const ERASE_GRAN: usize>
    MpiNorFlash<'d, T, WRITE_GRAN, ERASE_GRAN>
{
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
        let aligned_end =
            end.saturating_add(DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);
        let aligned_len = aligned_end - aligned_start;

        let mut cp = unsafe { cortex_m::Peripherals::steal() };
        cp.SCB
            .clean_invalidate_dcache_by_address(aligned_start, aligned_len);
        cp.SCB.invalidate_icache();
    }

    fn regs(&self) -> Regs {
        T::regs()
    }

    fn addr_size_bits(&self) -> u8 {
        self.config.address_size.bits()
    }

    fn page_program_cmd(&self) -> u8 {
        if self.config.address_size == AddressSize::FourBytes {
            self.config
                .commands
                .page_program_4byte
                .unwrap_or(self.config.commands.page_program)
        } else {
            self.config.commands.page_program
        }
    }

    fn sector_erase_cmd(&self) -> u8 {
        if self.config.address_size == AddressSize::FourBytes {
            self.config
                .commands
                .sector_erase_4byte
                .unwrap_or(self.config.commands.sector_erase)
        } else {
            self.config.commands.sector_erase
        }
    }

    /// Read flash status register low byte.
    ///
    /// This method is XIP-safe: it uses RAM-resident code for the entire operation,
    /// including interrupt disable/restore.
    pub fn read_status(&mut self) -> Result<u8, Error> {
        ram_wrapper_read_status(
            self.regs(),
            self.config.commands.read_status,
            self.config.max_ready_polls,
        )
        .map_err(|_| Error::Timeout)
    }

    /// Poll WIP bit until flash is ready.
    ///
    /// This method is XIP-safe: it uses RAM-resident code for the entire polling loop,
    /// including interrupt disable/restore.
    pub fn wait_ready(&mut self) -> Result<(), Error> {
        if ram_wrapper_wait_ready(
            self.regs(),
            self.config.commands.read_status,
            self.config.max_ready_polls,
        ) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    /// Read JEDEC ID (3 bytes, little-endian in returned `u32` low 24 bits).
    ///
    /// This method is XIP-safe: it uses RAM-resident code for the entire operation,
    /// including interrupt disable/restore.
    pub fn read_jedec_id(&mut self) -> Result<u32, Error> {
        ram_wrapper_read_jedec_id(
            self.regs(),
            self.config.commands.read_jedec_id,
            self.config.max_ready_polls,
        )
        .map_err(|_| Error::Timeout)
    }

    fn program_chunk(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        if ram_wrapper_program_chunk(
            self.regs(),
            self.config.commands.write_enable,
            self.page_program_cmd(),
            self.config.commands.read_status,
            addr,
            self.addr_size_bits(),
            data,
            self.config.max_ready_polls,
        ) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), Error> {
        if ram_wrapper_erase_sector(
            self.regs(),
            self.config.commands.write_enable,
            self.sector_erase_cmd(),
            self.config.commands.read_status,
            addr,
            self.addr_size_bits(),
            self.config.max_ready_polls,
        ) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    /// Read bytes from flash at `offset`.
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

    /// Program bytes to flash at `offset`.
    ///
    /// This method is XIP-safe: it uses hardware status polling (CMD2 + Status Match)
    /// to wait for flash operations to complete, avoiding CPU-based polling loops
    /// that could conflict with XIP code fetches.
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
            let page_remaining = self.config.page_size - (addr % self.config.page_size);
            let mut step = min(
                bytes.len() - done,
                min(page_remaining, min(FIFO_SIZE_BYTES, MAX_DLEN_BYTES)),
            );
            if WRITE_GRAN > 1 {
                step -= step % WRITE_GRAN;
            }
            if step == 0 {
                return Err(Error::InvalidConfiguration);
            }

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

    /// Erase flash range `[from, to)`.
    ///
    /// This method is XIP-safe: it uses hardware status polling (CMD2 + Status Match)
    /// to wait for flash operations to complete, avoiding CPU-based polling loops
    /// that could conflict with XIP code fetches.
    pub fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if from == to {
            return Ok(());
        }
        check_erase(self, from, to).map_err(Error::from)?;
        self.wait_ready()?;
        let mut addr = from;
        while addr < to {
            self.erase_sector(addr)?;
            addr += ERASE_GRAN as u32;
        }
        self.invalidate_cache_for_range(from, (to - from) as usize);
        Ok(())
    }

    /// Convert into a partition-limited flash view.
    pub fn into_partition(
        self,
        partition: NorFlashPartition,
    ) -> Result<MpiNorPartition<'d, T, WRITE_GRAN, ERASE_GRAN>, Error> {
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

    /// Consume and return the underlying low-level MPI driver.
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

/// Partition-limited NOR flash wrapper for runtime storage use.
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
    /// Map a partition-local offset to an absolute flash offset.
    ///
    /// Uses `>` (not `>=`) intentionally: `offset == size` is valid for
    /// end-of-range arguments (e.g. `to` in `erase_range`). Start offsets
    /// with nonzero length are guarded by `ensure_region`.
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

    /// Get partition descriptor.
    pub const fn partition(&self) -> NorFlashPartition {
        self.partition
    }

    /// Read bytes from partition-local offset.
    pub fn read_bytes(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.ensure_region(offset, bytes.len())?;
        let abs = self.map_offset(offset)?;
        self.flash.read_bytes(abs, bytes)
    }

    /// Program bytes at partition-local offset.
    pub fn write_bytes(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.ensure_region(offset, bytes.len())?;
        let abs = self.map_offset(offset)?;
        self.flash.write_bytes(abs, bytes)
    }

    /// Erase partition-local range `[from, to)`.
    pub fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if to < from {
            return Err(Error::OutOfBounds);
        }
        self.ensure_region(from, (to - from) as usize)?;
        let abs_from = self.map_offset(from)?;
        let abs_to = self.map_offset(to)?;
        self.flash.erase_range(abs_from, abs_to)
    }

    /// Consume and return the underlying full-flash wrapper.
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

pub(crate) trait SealedInstance:
    crate::rcc::RccEnableReset + crate::rcc::RccGetFreq
{
    fn regs() -> Regs;
    fn code_bus_base() -> usize;
    fn code_bus_end() -> usize;
}

/// MPI peripheral instance.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {}

impl SealedInstance for crate::peripherals::MPI1 {
    #[inline(always)]
    fn regs() -> Regs {
        crate::pac::MPI1
    }

    #[inline(always)]
    fn code_bus_base() -> usize {
        MPI1_CODE_BUS_BASE
    }

    #[inline(always)]
    fn code_bus_end() -> usize {
        MPI1_CODE_BUS_END
    }
}

impl Instance for crate::peripherals::MPI1 {}

impl SealedInstance for crate::peripherals::MPI2 {
    #[inline(always)]
    fn regs() -> Regs {
        crate::pac::MPI2
    }

    #[inline(always)]
    fn code_bus_base() -> usize {
        MPI2_CODE_BUS_BASE
    }

    #[inline(always)]
    fn code_bus_end() -> usize {
        MPI2_CODE_BUS_END
    }
}

impl Instance for crate::peripherals::MPI2 {}
