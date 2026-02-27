//! LCPU power-on and management (blocking mode).
//!
//! ```ignore
//! use sifli_hal::lcpu::{Lcpu, LcpuConfig};
//!
//! # fn example() -> Result<(), sifli_hal::lcpu::LcpuError> {
//! let p = sifli_hal::init(Default::default());
//! let cfg = LcpuConfig::default();
//! let lcpu = Lcpu::new();
//! lcpu.power_on(&cfg, p.DMAC2_CH8)?;
//! # Ok(()) }
//! ```

// Re-export RAM management module
pub mod memory_map;
pub mod ram;
pub use ram::LpsysRam;

mod config;
pub use config::{ActConfig, BleConfig, BootConfig, ControllerConfig, EmConfig, RomConfig};

pub(crate) mod controller;
mod nvds;

pub mod bt_rf_cal;

use core::fmt;

use crate::dma::Channel;
use crate::syscfg;
use crate::Peripheral;
use crate::{lpaon, patch, rcc};

#[cfg(feature = "sf32lb52x-lcpu")]
mod sf32lb52x_lcpu_data {
    pub const FIRMWARE: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/lcpu_firmware.bin");
    pub const PATCH_A3_LIST: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/patch_a3_list.bin");
    pub const PATCH_A3_BIN: &[u8] = include_bytes!("../../data/sf32lb52x/lcpu/patch_a3_bin.bin");
    pub const PATCH_LETTER_LIST: &[u8] =
        include_bytes!("../../data/sf32lb52x/lcpu/patch_letter_list.bin");
    pub const PATCH_LETTER_BIN: &[u8] =
        include_bytes!("../../data/sf32lb52x/lcpu/patch_letter_bin.bin");
}

//=============================================================================
// Configuration
//=============================================================================

/// LCPU power-on configuration.
#[derive(Debug, Clone, Copy)]
pub struct LcpuConfig {
    /// Boot-time configuration (firmware, patches, ROM parameters).
    pub boot: BootConfig,

    /// BLE-specific configuration (post-boot controller params + BD address).
    pub ble: BleConfig,
}

impl LcpuConfig {
    /// Create a new config with all options unset.
    pub const fn new() -> Self {
        Self {
            boot: BootConfig::new(),
            ble: BleConfig::new(),
        }
    }

    /// Skip LPSYS HCLK frequency check during image loading.
    pub const fn skip_frequency_check(mut self, skip: bool) -> Self {
        self.boot.skip_frequency_check = skip;
        self
    }

    /// Disable RF calibration (normally runs after patch installation).
    pub const fn disable_rf_cal(mut self, disable: bool) -> Self {
        self.boot.disable_rf_cal = disable;
        self
    }

    /// Set BLE Exchange Memory buffer configuration (Letter Series only).
    pub const fn em_config(mut self, config: EmConfig) -> Self {
        self.boot.rom.em_config = Some(config);
        self
    }

    /// Set BLE/BT activity configuration (Letter Series only).
    pub const fn act_config(mut self, config: ActConfig) -> Self {
        self.boot.rom.act_config = Some(config);
        self
    }

    /// Set public BD address.
    pub const fn bd_addr(mut self, addr: [u8; 6]) -> Self {
        self.ble.bd_addr = addr;
        self
    }
}

impl Default for LcpuConfig {
    fn default() -> Self {
        #[allow(unused_mut)]
        let mut cfg = Self::new();

        #[cfg(feature = "sf32lb52x-lcpu")]
        {
            cfg.boot.firmware = Some(sf32lb52x_lcpu_data::FIRMWARE);
            cfg.boot.patch_a3 = Some(PatchData {
                list: sf32lb52x_lcpu_data::PATCH_A3_LIST,
                bin: sf32lb52x_lcpu_data::PATCH_A3_BIN,
            });
            cfg.boot.patch_letter = Some(PatchData {
                list: sf32lb52x_lcpu_data::PATCH_LETTER_LIST,
                bin: sf32lb52x_lcpu_data::PATCH_LETTER_BIN,
            });
            cfg.boot.disable_rf_cal = false;
        }

        cfg
    }
}

/// Patch data (entry list + code binary), matching `g_lcpu_patch_list` / `g_lcpu_patch_bin` in SDK.
#[derive(Debug, Clone, Copy)]
pub struct PatchData {
    /// Patch entry list array.
    pub list: &'static [u8],
    /// Patch code bytes.
    pub bin: &'static [u8],
}

//=============================================================================
// Error types
//=============================================================================

/// LCPU power-on error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LcpuError {
    /// Image installation error.
    ImageInstall(ram::Error),

    /// Patch installation error.
    PatchInstall(patch::Error),

    /// Missing firmware image for A3 and earlier revisions.
    FirmwareMissing,

    /// Frequency check failed (LPSYS HCLK exceeded 24MHz during loading).
    FrequencyTooHigh {
        /// Actual frequency (Hz).
        actual_hz: u32,
        /// Maximum allowed frequency (Hz).
        max_hz: u32,
    },

    /// HPAON operation error.
    HpaonError,

    /// RCC operation error.
    RccError,

    /// ROM configuration error.
    RomConfigError,

    /// Wake reference count overflow (limit is 20).
    RefCountOverflow,

    /// Timeout waiting for LP_ACTIVE.
    WakeCoreTimeout,

    /// Error reading BT warmup event from IPC.
    WarmupReadError,
}

impl From<ram::Error> for LcpuError {
    fn from(err: ram::Error) -> Self {
        Self::ImageInstall(err)
    }
}

impl From<patch::Error> for LcpuError {
    fn from(err: patch::Error) -> Self {
        Self::PatchInstall(err)
    }
}

//=============================================================================
// Core IDs
//=============================================================================

/// Core ID (for multi-core wake operations).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CoreId {
    Default = 0,
    /// HCPU (High-performance CPU)
    Hcpu = 1,
    /// LCPU (Low-power CPU)
    Lcpu = 2,
    /// ACPU (58x)
    Acpu = 3,
}

//=============================================================================
// LCPU driver type
//=============================================================================

/// RAII guard for LCPU wakeup reference count.
/// Calls `cancel_lcpu_active_request()` on drop.
pub(crate) struct WakeGuard(());

impl WakeGuard {
    /// Acquire: calls `wake_lcpu()`, incrementing the reference count.
    pub(crate) unsafe fn acquire() -> Self {
        rcc::wake_lcpu();
        Self(())
    }
}

impl Drop for WakeGuard {
    fn drop(&mut self) {
        unsafe { rcc::cancel_lcpu_active_request() };
    }
}

/// LCPU driver (blocking).
pub struct Lcpu {
    _lcpu: crate::PeripheralRef<'static, crate::peripherals::LCPU>,
}

impl fmt::Debug for Lcpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lcpu").finish_non_exhaustive()
    }
}

impl Lcpu {
    /// Create a new blocking LCPU driver.
    pub fn new(lcpu: impl Peripheral<P = crate::peripherals::LCPU> + 'static) -> Self {
        Self {
            _lcpu: lcpu.into_ref(),
        }
    }
}

//=============================================================================
// Power-on flow
//=============================================================================

impl Lcpu {
    /// BLE boot sequence (async).
    ///
    /// Compared to [`power_on`](Self::power_on), this method additionally:
    /// 1. Waits for and consumes the BT stack warmup event
    /// 2. Runs controller initialization (sleep timing, MAC clock, etc.)
    ///
    /// ## Warmup Event
    ///
    /// After LCPU BT stack boots, it sends a VSC 0xFC11 Command Complete event (7 bytes):
    /// ```text
    /// 04 0E 04 06 11 FC 00
    /// │  │  │  │  └──┴── Opcode: 0xFC11 (Vendor Specific)
    /// │  │  │  └─ Num_HCI_Command_Packets: 6
    /// │  │  └─ Parameter_Total_Length: 4
    /// │  └─ Event Code: 0x0E (Command Complete)
    /// └─ H4 Indicator: 0x04 (HCI Event)
    /// ```
    ///
    /// This event must be consumed before sending any HCI commands, otherwise it may
    /// interfere with subsequent communication.
    ///
    /// ## Parameters
    ///
    /// - `config`: LCPU boot configuration
    /// - `hci_rx`: HCI receive end for reading the warmup event (typically `IpcQueueRx`)
    ///
    /// ## Example
    ///
    /// ```ignore
    /// # async fn example() -> Result<(), sifli_hal::lcpu::LcpuError> {
    /// # let p = sifli_hal::init(Default::default());
    /// # let lcpu = sifli_hal::lcpu::Lcpu::new();
    /// # let mut hci_rx: sifli_hal::ipc::IpcQueueRx = todo!();
    /// use sifli_hal::lcpu::LcpuConfig;
    ///
    /// lcpu.ble_power_on(&LcpuConfig::default(), p.DMAC2_CH8, &mut hci_rx).await?;
    /// // Now safe to send HCI commands
    /// # Ok(())
    /// # }
    /// ```
    pub async fn ble_power_on<R>(
        &self,
        config: &LcpuConfig,
        dma_ch: impl Peripheral<P = impl Channel>,
        hci_rx: &mut R,
    ) -> Result<(), LcpuError>
    where
        R: embedded_io_async::Read,
    {
        // 0. Write NVDS to LCPU shared memory (SDK: bt_stack_nvds_init)
        //    Must complete before LCPU boot; ROM reads this to initialize BT parameters.
        //    Wake LCPU first to ensure shared memory is accessible.
        {
            let _w = unsafe { WakeGuard::acquire() };
            nvds::write_default(&config.ble.bd_addr, config.boot.rom.enable_lxt);
        }

        // 1. Execute standard boot sequence
        self.power_on(config, dma_ch)?;

        // 2. Warmup + controller init (scoped wake)
        //    SDK lcpu_power_on() releases HP2LP_REQ at the end, allowing LCPU
        //    to sleep between BLE events.  We mirror that by scoping the wake.
        {
            let _w = unsafe { WakeGuard::acquire() };

            // Read and discard warmup event
            consume_warmup_event(hci_rx).await?;

            // Controller initialization (SDK: bluetooth_init)
            //    Includes: sleep timing + MAC clock + CFO tracking + sleep control
            controller::init(&config.ble.controller);
        }
        // _w drops → cancel_lcpu_active_request, LCPU can sleep between BLE events

        Ok(())
    }

    /// Blocking LCPU boot sequence.
    pub fn power_on(
        &self,
        config: &LcpuConfig,
        dma_ch: impl Peripheral<P = impl Channel>,
    ) -> Result<(), LcpuError> {
        // 1. Wake LCPU (guard ensures cancel on early return).
        debug!("Step 1: Waking up LCPU");
        let _w = unsafe { WakeGuard::acquire() };

        // 2. Reset and halt LCPU.
        debug!("Step 2: Resetting and halting LCPU");
        self.reset_and_halt_lcpu()?;

        // 3. Configure ROM parameters (bf0_lcpu_init.c:168).
        debug!("Step 3: Configuring ROM parameters");
        ram::rom_config(&config.boot.rom, &config.ble.controller)?;

        // 4. Enforce frequency limit while loading (bf0_lcpu_init.c:170-176).
        // If frequency exceeds 24MHz, automatically reduce it.
        let _original_freq_hz = if !config.boot.skip_frequency_check {
            debug!("Step 4: Ensuring LCPU frequency ≤ 24MHz during loading");
            rcc::ensure_safe_lcpu_frequency().map_err(|_| LcpuError::RccError)?
        } else {
            warn!("Step 4: Skipping frequency check (as requested by config)");
            0 // Dummy value when skipped
        };

        // 5. Install image for A3 and earlier (bf0_lcpu_init.c:178-182).
        let is_letter = syscfg::read_idr().revision().is_letter_series();
        if !is_letter {
            debug!("Step 5: Installing LCPU firmware image (A3/earlier)");

            if let Some(firmware) = config.boot.firmware {
                ram::img_install(firmware)?;
            } else {
                error!("Firmware required for A3 and earlier revisions");
                return Err(LcpuError::FirmwareMissing);
            }
        } else {
            debug!("Step 5: Skipping image install (Letter Series, firmware in ROM)");
        }

        // 6. Configure LCPU start address (bf0_lcpu_init.c:184).
        debug!(
            "Step 6: Configuring LCPU start address (0x{:08X})",
            LpsysRam::CODE_START
        );
        let (sp, pc) = read_start_vector_from_mem();
        lpaon::configure_lcpu_start(sp, pc);

        // 7. Install patches and perform RF calibration (bf0_lcpu_init.c:185).
        debug!("Step 7: Installing patches and RF calibration");
        install_patch_and_calibrate(config, dma_ch)?;

        // 8. Release LCPU to run (bf0_lcpu_init.c:186).
        debug!("Step 8: Releasing LCPU to run");
        self.release_lcpu()?;

        // _w drops here → cancel_lcpu_active_request, allow LP to enter low power.
        Ok(())
    }

    /// Blocking shutdown: reset and hold CPUWAIT.
    pub fn power_off(&self) -> Result<(), LcpuError> {
        info!("Powering off LCPU");

        self.reset_and_halt_lcpu()?;

        info!("LCPU powered off successfully");

        Ok(())
    }

    fn reset_and_halt_lcpu(&self) -> Result<(), LcpuError> {
        // Only perform reset flow when CPUWAIT is not set.
        if !lpaon::cpuwait() {
            // 1. Set CPUWAIT so LCPU stays halted.
            lpaon::set_cpuwait(true);

            // 2. Reset LCPU and MAC (SF32LB52X requires both).
            rcc::set_lp_lcpu_reset(true);
            rcc::set_lp_mac_reset(true);

            // Wait until reset bits take effect.
            while !rcc::lp_lcpu_reset_asserted() || !rcc::lp_mac_reset_asserted() {}

            // 3. If LPSYS is sleeping, wake it up.
            if lpaon::sleep_status() {
                lpaon::set_wkup_req(true);
                while lpaon::sleep_status() {}
            }

            // 4. Clear reset bits, keep CPUWAIT = 1.
            rcc::set_lp_lcpu_reset(false);
            rcc::set_lp_mac_reset(false);
        }

        Ok(())
    }

    fn release_lcpu(&self) -> Result<(), LcpuError> {
        // Clear CPUWAIT so LCPU can run.
        lpaon::set_cpuwait(false);

        Ok(())
    }
}

/// Read the first two entries `(SP, PC)` from the LCPU vector table in memory.
fn read_start_vector_from_mem() -> (u32, u32) {
    let vector_addr = LpsysRam::CODE_START as *const u32;

    unsafe {
        // vector[0] (0x20400000): initial stack pointer (SP)
        // vector[1] (0x20400004): Reset_Handler address (PC)
        let sp = core::ptr::read_volatile(vector_addr);
        let pc = core::ptr::read_volatile(vector_addr.add(1));
        (sp, pc)
    }
}

/// Install patches and perform RF calibration.
///
/// Corresponds to SDK `lcpu_ble_patch_install()` (bf0_lcpu_init.c:179):
///   1. patch install (A3 or Letter series)
///   2. bt_rf_cal() — full RF calibration
///   3. adc_resume() — restore GPADC after OSLO cal
///   4. memset(EM, 0, 0x5000) — clear Exchange Memory
///
/// Steps 2-4 are handled inside [`bt_rf_cal::bt_rf_cal`]; step 3 is a TODO
/// because OSLO calibration (which touches GPADC) is not yet implemented.
fn install_patch_and_calibrate(
    config: &LcpuConfig,
    dma_ch: impl Peripheral<P = impl Channel>,
) -> Result<(), LcpuError> {
    // SDK lcpu_ble_patch_install — step 1: patch install
    let patch_data = if syscfg::read_idr().revision().is_letter_series() {
        debug!("Using Letter Series patch data");
        config.boot.patch_letter
    } else {
        debug!("Using A3 patch data");
        config.boot.patch_a3
    };

    if let Some(data) = patch_data {
        debug!(
            "Installing patches (list: {} bytes, bin: {} bytes)",
            data.list.len(),
            data.bin.len()
        );
        patch::install(data.list, data.bin)?;
    } else {
        warn!("No patch data provided, skipping patch installation");
    }

    // SDK lcpu_ble_patch_install — steps 2-4: bt_rf_cal + adc_resume + EM clear
    if !config.boot.disable_rf_cal {
        debug!("Performing RF calibration");
        bt_rf_cal::bt_rf_cal(dma_ch);
    } else {
        warn!("RF calibration disabled by config");
    }

    Ok(())
}

/// Consume the BT warmup event.
///
/// The warmup event is an H4-formatted HCI Event packet:
/// - H4 indicator: 0x04 (HCI Event)
/// - Event code: 0x0E (Command Complete)
/// - Parameter length: 0x04
/// - Payload: 06 11 FC 00
///
/// Total 7 bytes. This function reads and discards the event.
async fn consume_warmup_event<R>(rx: &mut R) -> Result<(), LcpuError>
where
    R: embedded_io_async::Read,
{
    // H4 HCI Event format:
    // [0] H4 indicator (0x04 = Event)
    // [1] Event code
    // [2] Parameter length (N)
    // [3..3+N] Parameters

    let mut header = [0u8; 3];
    rx.read_exact(&mut header)
        .await
        .map_err(|_| LcpuError::WarmupReadError)?;

    debug!(
        "[hci] warmup header: {:02X} {:02X} {:02X}",
        header[0], header[1], header[2]
    );

    // Verify it's an HCI Event
    if header[0] != 0x04 {
        warn!(
            "[hci] unexpected H4 indicator in warmup: 0x{:02X}, expected 0x04",
            header[0]
        );
    }

    // Read remaining parameters
    let param_len = header[2] as usize;
    if param_len > 0 {
        let mut params = [0u8; 255];
        rx.read_exact(&mut params[..param_len])
            .await
            .map_err(|_| LcpuError::WarmupReadError)?;
        debug!(
            "[hci] warmup params ({} bytes): {:02X}",
            param_len,
            &params[..param_len]
        );
    }

    debug!("[hci] warmup event consumed");
    Ok(())
}
