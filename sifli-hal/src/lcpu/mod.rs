//! LCPU power-on and power management.
//!
//! Typical async usage:
//!
//! ```no_run
//! use sifli_hal::lcpu::{Lcpu, LcpuConfig};
//!
//! # async fn example() -> Result<(), sifli_hal::lcpu::LcpuError> {
//! let p = sifli_hal::init(Default::default());
//! let cfg = LcpuConfig::default();
//! let lcpu = Lcpu::new(p.HPSYS_AON, p.LPSYS_AON).into_async();
//! lcpu.power_on(&cfg).await?;
//! # Ok(()) }
//! ```

// Re-export RAM management module
pub mod ram;
pub use ram::LpsysRam;

mod bt_cal;

use core::fmt;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::peripherals;
use crate::syscfg::{self, ChipRevision};
use crate::Peripheral;
use crate::{hpaon, lpaon, patch};

#[cfg(feature = "sf32lb52x-lcpu")]
mod sf32lb52x_lcpu_data {
    include!(concat!(env!("OUT_DIR"), "/sf32lb52x_lcpu.rs"));
}

//=============================================================================
// Configuration
//=============================================================================

/// LCPU power-on configuration.
#[derive(Debug, Clone, Copy)]
pub struct LcpuConfig {
    /// LCPU firmware image bytes.
    ///
    /// - A3 and earlier: must be provided and copied to LPSYS RAM.
    /// - Letter Series: optional, firmware is in ROM.
    pub firmware: Option<&'static [u8]>,

    /// ROM configuration parameters.
    pub rom: ram::RomConfig,

    /// Patch data for A3 and earlier (record + code format).
    pub patch_a3: Option<PatchData>,

    /// Patch data for Letter Series (A4/B4) (header + code format).
    pub patch_letter: Option<PatchData>,

    /// Skip LPSYS HCLK frequency check during image loading (use with care).
    pub skip_frequency_check: bool,

    /// Disable RF calibration (normally runs after patch installation).
    pub disable_rf_cal: bool,
}

impl LcpuConfig {
    /// Create a new config with all options unset.
    pub const fn new() -> Self {
        Self {
            firmware: None,
            rom: ram::RomConfig {
                wdt_time: 10,
                wdt_clk: 32_768,
                enable_lxt: true,
            },
            patch_a3: None,
            patch_letter: None,
            skip_frequency_check: false,
            disable_rf_cal: false,
        }
    }
}

impl Default for LcpuConfig {
    fn default() -> Self {
        #[allow(unused_mut)]
        let mut cfg = Self::new();

        #[cfg(feature = "sf32lb52x-lcpu")]
        {
            cfg.firmware = Some(sf32lb52x_lcpu_data::SF32LB52X_LCPU_FIRMWARE);
            cfg.patch_a3 = Some(PatchData {
                list: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_A3_LIST,
                bin: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_A3_BIN,
            });
            cfg.patch_letter = Some(PatchData {
                list: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_LETTER_LIST,
                bin: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_LETTER_BIN,
            });
            cfg.disable_rf_cal = true;
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

/// LCPU driver type, parameterized by operation mode (Async / Blocking).
pub struct Lcpu<'d, M: crate::mode::Mode> {
    hpaon: hpaon::Hpaon<'d, peripherals::HPSYS_AON>,
    lpaon: lpaon::LpAon<'d, peripherals::LPSYS_AON>,
    _mode: PhantomData<fn() -> M>,
}

impl<'d, M: crate::mode::Mode> fmt::Debug for Lcpu<'d, M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lcpu").finish_non_exhaustive()
    }
}

impl<'d> Lcpu<'d, crate::mode::Blocking> {
    /// Create a new blocking LCPU driver instance.
    pub fn new(
        hpsys_aon: impl Peripheral<P = peripherals::HPSYS_AON> + 'd,
        lpsys_aon: impl Peripheral<P = peripherals::LPSYS_AON> + 'd,
    ) -> Self {
        Self {
            hpaon: hpaon::Hpaon::new(hpsys_aon),
            lpaon: lpaon::LpAon::new(lpsys_aon),
            _mode: PhantomData,
        }
    }

    /// Convert the blocking driver into an async driver.
    pub fn into_async(self) -> Lcpu<'d, crate::mode::Async> {
        Lcpu {
            hpaon: self.hpaon,
            lpaon: self.lpaon,
            _mode: PhantomData,
        }
    }
}

//=============================================================================
// LCPU wake guard and reference counting
//=============================================================================

/// LCPU wake reference counter.
///
/// Tracks how many modules are currently keeping LCPU awake.
/// HP2LP_REQ is only cleared when this count returns to zero.
static LCPU_WAKEUP_REF_CNT: AtomicU8 = AtomicU8::new(0);

/// Maximum allowed wake reference count (from SDK).
const MAX_REF_COUNT: u8 = 20;

/// LCPU wake guard (RAII).
///
/// Wakes LCPU on creation (if needed) and decrements the wake reference
/// count on drop, clearing the wake request when the last guard is dropped.
///
/// ```no_run
/// use sifli_hal::lcpu::Lcpu;
///
/// # async fn example() -> Result<(), sifli_hal::lcpu::LcpuError> {
/// let p = sifli_hal::init(Default::default());
/// let lcpu = Lcpu::new(p.HPSYS_AON, p.LPSYS_AON).into_async();
/// let _guard = lcpu.wake().await?;
/// # Ok(()) }
/// ```
pub struct WakeGuard<'a, 'd> {
    hpaon: &'a hpaon::Hpaon<'d, peripherals::HPSYS_AON>,
}

impl<'a, 'd> WakeGuard<'a, 'd> {
    /// Create an async LCPU wake guard; wakes LCPU on first use.
    pub async fn new(
        hpaon: &'a hpaon::Hpaon<'d, peripherals::HPSYS_AON>,
    ) -> Result<Self, LcpuError> {
        // 1. Atomically increment wake reference count.
        let prev_count = critical_section::with(|_| {
            let count = LCPU_WAKEUP_REF_CNT.load(Ordering::Relaxed);
            if count >= MAX_REF_COUNT {
                return Err(LcpuError::RefCountOverflow);
            }
            LCPU_WAKEUP_REF_CNT.store(count + 1, Ordering::Relaxed);
            Ok(count)
        })?;

        debug!(
            "LCPU wake reference count: {} -> {}",
            prev_count,
            prev_count + 1
        );

        // 2. Only perform hardware wake when transitioning 0 → 1.
        if prev_count == 0 {
            debug!("First wake, setting HP2LP_REQ and waiting for LP_ACTIVE");

            // Timeout: based on microsecond-scale wait in SDK
            // (`HAL_HPAON_WakeCore` with ~230us + LP_ACTIVE polls),
            // here we use a conservative 20ms upper bound to avoid hanging.
            if !hpaon.wake_lcpu_with_timeout(20_000).await {
                error!("Timeout waiting for LP_ACTIVE, rolling back ref count");

                // On timeout, roll back the reference count.
                critical_section::with(|_| {
                    LCPU_WAKEUP_REF_CNT.store(0, Ordering::Relaxed);
                });

                return Err(LcpuError::WakeCoreTimeout);
            }

            debug!("LP_ACTIVE set, LCPU is now awake");
        } else {
            debug!("LCPU already awake, reusing existing wake state");
        }

        Ok(Self { hpaon })
    }

    /// Blocking version of [`WakeGuard::new`].
    pub fn new_blocking(
        hpaon: &'a hpaon::Hpaon<'d, peripherals::HPSYS_AON>,
    ) -> Result<Self, LcpuError> {
        // 1. Atomically increment wake reference count.
        let prev_count = critical_section::with(|_| {
            let count = LCPU_WAKEUP_REF_CNT.load(Ordering::Relaxed);
            if count >= MAX_REF_COUNT {
                return Err(LcpuError::RefCountOverflow);
            }
            LCPU_WAKEUP_REF_CNT.store(count + 1, Ordering::Relaxed);
            Ok(count)
        })?;

        debug!(
            "LCPU wake reference count: {} -> {}",
            prev_count,
            prev_count + 1
        );

        // 2. Only perform hardware wake when transitioning 0 → 1.
        if prev_count == 0 {
            debug!("First wake, setting HP2LP_REQ and waiting for LP_ACTIVE");

            // Timeout: see `new`; use the same conservative 20ms limit.
            if !hpaon.wake_lcpu_with_timeout_blocking(20_000) {
                error!("Timeout waiting for LP_ACTIVE, rolling back ref count");

                // On timeout, roll back the reference count.
                critical_section::with(|_| {
                    LCPU_WAKEUP_REF_CNT.store(0, Ordering::Relaxed);
                });

                return Err(LcpuError::WakeCoreTimeout);
            }

            debug!("LP_ACTIVE set, LCPU is now awake");
        } else {
            debug!("LCPU already awake, reusing existing wake state");
        }

        Ok(Self { hpaon })
    }
}

impl<'a, 'd> Drop for WakeGuard<'a, 'd> {
    /// Update the wake reference count on drop and clear LP_ACTIVE when last guard is released.
    fn drop(&mut self) {
        // Atomically decrement reference count and check if this is the last guard.
        let is_last = critical_section::with(|_| {
            let count = LCPU_WAKEUP_REF_CNT.load(Ordering::Relaxed);
            assert!(count >= 1, "LCPU wake reference count underflow");
            LCPU_WAKEUP_REF_CNT.store(count - 1, Ordering::Relaxed);
            count == 1
        });

        let new_count = LCPU_WAKEUP_REF_CNT.load(Ordering::Relaxed);
        debug!("LCPU wake reference count decreased to {}", new_count);

        // Only clear HP2LP_REQ when the last guard is dropped (1→0).
        if is_last {
            debug!("Last guard dropped, clearing HP2LP_REQ");

            // Clear HP2LP_REQ (bf0_hal_aon.h:309).
            self.hpaon.cancel_lp_active_request();

            debug!("HP2LP_REQ cleared, LPSYS can now sleep");
        } else {
            debug!("Other guards still active, keeping HP2LP_REQ set");
        }
    }
}

//=============================================================================
// Power-on flow
//=============================================================================

impl<'d> Lcpu<'d, crate::mode::Async> {
    /// High-level async LCPU power-on helper.
    pub async fn power_on(&self, config: &LcpuConfig) -> Result<(), LcpuError> {
        info!("Starting LCPU power-on sequence");

        // 1. Wake LCPU via WakeGuard RAII.
        debug!("Step 1: Waking up LCPU");
        let guard = WakeGuard::new(&self.hpaon).await?;

        // 2. Reset and halt LCPU.
        debug!("Step 2: Resetting and halting LCPU");
        self.reset_and_halt_lcpu().await?;

        self.power_on_after_reset(config, guard)
    }

    /// Obtain an async LCPU wake guard.
    ///
    /// The returned [`WakeGuard`] automatically decrements the wake reference
    /// count on drop and clears LP_ACTIVE when the last guard is released.
    pub async fn wake(&self) -> Result<WakeGuard<'_, 'd>, LcpuError> {
        WakeGuard::new(&self.hpaon).await
    }

    /// Async power-off helper: reset and halt LCPU.
    pub async fn power_off(&self) -> Result<(), LcpuError> {
        info!("Powering off LCPU");

        self.reset_and_halt_lcpu().await?;

        info!("LCPU powered off successfully");

        Ok(())
    }

    async fn reset_and_halt_lcpu(&self) -> Result<(), LcpuError> {
        use crate::lpsys_rcc;
        use embassy_time::{with_timeout, Duration, Timer};

        // Only perform reset flow when CPUWAIT is not set.
        if !self.lpaon.cpuwait() {
            // 1. Set CPUWAIT so LCPU stays halted.
            self.lpaon.set_cpuwait(true);

            // 2. Reset LCPU and MAC (SF32LB52X requires both).
            lpsys_rcc::set_lcpu_reset(true);
            lpsys_rcc::set_mac_reset(true);

            // Embassy style: poll with timeout.
            let wait_reset = async {
                while lpsys_rcc::get_rstr1() == 0 {
                    Timer::after_micros(10).await;
                }
            };
            with_timeout(Duration::from_millis(20), wait_reset)
                .await
                .map_err(|_| LcpuError::RccError)?;

            // 3. If LPSYS is sleeping, wake it up (async loop + timeout).
            if self.lpaon.sleep_status() {
                self.lpaon.set_wkup_req(true);

                let wait_wakeup = async {
                    while self.lpaon.sleep_status() {
                        Timer::after_micros(10).await;
                    }
                };
                with_timeout(Duration::from_micros(20), wait_wakeup)
                    .await
                    .map_err(|_| LcpuError::RccError)?;
            }

            // 4. Clear reset bits, keep CPUWAIT = 1.
            lpsys_rcc::set_lcpu_reset(false);
            lpsys_rcc::set_mac_reset(false);
        }

        Ok(())
    }
}

impl<'d> Lcpu<'d, crate::mode::Blocking> {
    pub fn power_on(&self, config: &LcpuConfig) -> Result<(), LcpuError> {
        info!("Starting LCPU power-on sequence");

        // 1. Wake LCPU via WakeGuard RAII.
        debug!("Step 1: Waking up LCPU");
        let guard = WakeGuard::new_blocking(&self.hpaon)?;

        // 2. Reset and halt LCPU.
        debug!("Step 2: Resetting and halting LCPU");
        self.reset_and_halt_lcpu()?;

        self.power_on_after_reset(config, guard)
    }

    /// Obtain a blocking LCPU wake guard.
    pub fn wake(&self) -> Result<WakeGuard<'_, 'd>, LcpuError> {
        WakeGuard::new_blocking(&self.hpaon)
    }

    /// Blocking power-off helper: reset and halt LCPU.
    pub fn power_off(&self) -> Result<(), LcpuError> {
        info!("Powering off LCPU");

        self.reset_and_halt_lcpu()?;

        info!("LCPU powered off successfully");

        Ok(())
    }

    fn reset_and_halt_lcpu(&self) -> Result<(), LcpuError> {
        use crate::lpsys_rcc;

        // Only perform reset flow when CPUWAIT is not set.
        if !self.lpaon.cpuwait() {
            // 1. Set CPUWAIT so LCPU stays halted.
            self.lpaon.set_cpuwait(true);

            // 2. Reset LCPU and MAC (SF32LB52X requires both).
            lpsys_rcc::set_lcpu_reset(true);
            lpsys_rcc::set_mac_reset(true);

            // Wait until reset bits take effect.
            while lpsys_rcc::get_rstr1() == 0 {}

            // 3. If LPSYS is sleeping, wake it up.
            if self.lpaon.sleep_status() {
                self.lpaon.set_wkup_req(true);
                while self.lpaon.sleep_status() {}
            }

            // 4. Clear reset bits, keep CPUWAIT = 1.
            lpsys_rcc::set_lcpu_reset(false);
            lpsys_rcc::set_mac_reset(false);
        }

        Ok(())
    }
}

impl<'d, M: crate::mode::Mode> Lcpu<'d, M> {
    fn power_on_after_reset<'a>(
        &'a self,
        config: &LcpuConfig,
        _guard: WakeGuard<'a, 'd>,
    ) -> Result<(), LcpuError> {
        // 3. Configure ROM parameters (bf0_lcpu_init.c:168).
        debug!("Step 3: Configuring ROM parameters");
        let idr = syscfg::read_idr();
        let revision = idr.revision();
        ram::rom_config(revision, &config.rom)?;

        // 4. Enforce frequency limit while loading (bf0_lcpu_init.c:170-176).
        if !config.skip_frequency_check {
            debug!("Step 4: Checking LCPU frequency (must be ≤ 24MHz during loading)");
            check_lcpu_frequency()?;
        } else {
            warn!("Step 4: Skipping frequency check (as requested by config)");
        }

        // 5. Install image for A3 and earlier (bf0_lcpu_init.c:178-182).
        if !revision.is_letter_series() {
            debug!("Step 5: Installing LCPU firmware image (A3/earlier)");

            if let Some(firmware) = config.firmware {
                ram::img_install(revision, firmware)?;
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
        self.lpaon.configure_lcpu_start();

        // 7. Install patches and perform RF calibration (bf0_lcpu_init.c:185).
        debug!("Step 7: Installing patches and RF calibration");
        install_patch_and_calibrate(config, revision)?;

        // 8. Release LCPU to run (bf0_lcpu_init.c:186).
        debug!("Step 8: Releasing LCPU to run");
        self.release_lcpu()?;

        // 9. Finalize (bf0_lcpu_init.c:187); guard drops and clears LP_ACTIVE.
        debug!("Step 9: Guard will auto-cancel LP_ACTIVE request on drop");

        info!("LCPU power-on sequence completed successfully");

        Ok(())
    }

    fn release_lcpu(&self) -> Result<(), LcpuError> {
        // Clear CPUWAIT so LCPU can run.
        self.lpaon.set_cpuwait(false);

        Ok(())
    }
}

/// Ensure LPSYS HCLK stays ≤ 24 MHz while loading the LCPU image.
///
/// Mirrors `bf0_lcpu_init.c:170-176` in the SDK.
fn check_lcpu_frequency() -> Result<(), LcpuError> {
    const MAX_LOAD_FREQ_HZ: u32 = 24_000_000;

    // 1. Compute current LPSYS HCLK frequency.
    let hclk_hz = match crate::lpsys_rcc::get_hclk_lpsys_freq() {
        Some(freq) => freq.0,
        None => {
            // If we cannot get HCLK (e.g. 48MHz source not ready), conservatively skip adjustment.
            warn!("LPSYS HCLK frequency unknown, skipping load-time check");
            return Ok(());
        }
    };
    let hdiv = crate::lpsys_rcc::get_hclk_lpsys_div();

    // 2. If already within limit, nothing to do.
    if hclk_hz <= MAX_LOAD_FREQ_HZ {
        debug!(
            "LPSYS HCLK within limit for LCPU loading: {} Hz (HDIV1={})",
            hclk_hz, hdiv
        );
        return Ok(());
    }

    warn!(
        "LPSYS HCLK too high for LCPU loading: {} Hz (max {} Hz), adjusting dividers",
        hclk_hz, MAX_LOAD_FREQ_HZ
    );

    // 3. Restore dividers to SDK-safe defaults (HAL_RCC_LCPU_SetDiv(2, 1, 5)).
    let mut cfg = crate::lpsys_rcc::LpsysConfig::new_keep();
    cfg.hclk_div = crate::rcc::ConfigOption::new(2);
    cfg.pclk1_div = crate::rcc::ConfigOption::new(1);
    cfg.pclk2_div = crate::rcc::ConfigOption::new(5);

    unsafe {
        cfg.apply();
    }

    let new_hclk_hz = match crate::lpsys_rcc::get_hclk_lpsys_freq() {
        Some(freq) => freq.0,
        None => {
            error!("LPSYS HCLK frequency became unknown after applying safe dividers");
            return Err(LcpuError::FrequencyTooHigh {
                actual_hz: 0,
                max_hz: MAX_LOAD_FREQ_HZ,
            });
        }
    };
    let new_hdiv = crate::lpsys_rcc::get_hclk_lpsys_div();

    if new_hclk_hz > MAX_LOAD_FREQ_HZ {
        error!(
            "Failed to reduce LPSYS HCLK below {} Hz: now {} Hz (HDIV1={})",
            MAX_LOAD_FREQ_HZ, new_hclk_hz, new_hdiv
        );
        return Err(LcpuError::FrequencyTooHigh {
            actual_hz: new_hclk_hz,
            max_hz: MAX_LOAD_FREQ_HZ,
        });
    }

    debug!(
        "Adjusted LPSYS HCLK for LCPU loading: {} Hz -> {} Hz (HDIV1={})",
        hclk_hz, new_hclk_hz, new_hdiv
    );

    Ok(())
}

/// Install patches and perform RF calibration.
///
/// Selects the appropriate patch data based on chip revision, installs it,
/// and then runs RF calibration (not yet implemented).
///
/// # Errors
///
/// - [`LcpuError::PatchInstall`][]: patch installation failed.
fn install_patch_and_calibrate(config: &LcpuConfig, revision: ChipRevision) -> Result<(), LcpuError> {
    // Choose patch data based on revision.
    let patch_data = if revision.is_letter_series() {
        debug!("Using Letter Series patch data");
        config.patch_letter
    } else {
        debug!("Using A3 patch data");
        config.patch_a3
    };

    // Install patch if data is provided.
    if let Some(data) = patch_data {
        debug!(
            "Installing patches (list: {} bytes, bin: {} bytes)",
            data.list.len(),
            data.bin.len()
        );

        // Use patch module to install the patch.
        patch::install(revision, data.list, data.bin)?;
    } else {
        warn!("No patch data provided, skipping patch installation");
    }

    if !config.disable_rf_cal {
        debug!("Performing RF calibration");

        // Reference SDK `lcpu_ble_patch_install`, perform BT RF calibration after patch installation.
        // Here only the minimum necessary subset is implemented: reset RFC and write BT_TXPWR configuration.
        crate::lcpu::bt_cal::bt_rf_cal(revision);
    } else {
        warn!("RF calibration disabled by config");
    }

    Ok(())
}
