//! LCPU hardware driver.
//!
//! Provides low-level LCPU power control operations:
//! wake, reset, halt, release, firmware loading, start vector configuration.
//!
//! For BLE functionality, use the `sifli-radio` crate which builds on these primitives.

use core::fmt;

use crate::ram::{memory_map, RamSlice};
use crate::Peripheral;
use crate::{lpaon, patch, rcc};

//=============================================================================
// Error types
//=============================================================================

/// LCPU memory operation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ImageError {
    /// Image is empty.
    EmptyImage,
    /// Image size exceeds LPSYS RAM capacity.
    ImageTooLarge { size_bytes: usize, max_bytes: usize },
}

/// LCPU operation error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LcpuError {
    /// Image installation error.
    ImageInstall(ImageError),

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

impl From<ImageError> for LcpuError {
    fn from(err: ImageError) -> Self {
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
pub struct WakeGuard(());

impl WakeGuard {
    /// Acquire: calls `wake_lcpu()`, incrementing the reference count.
    ///
    /// # Safety
    ///
    /// Caller must ensure that the LCPU subsystem is in a valid state
    /// for wake operations. Multiple concurrent WakeGuards are reference-counted.
    pub unsafe fn acquire() -> Self {
        rcc::wake_lcpu();
        Self(())
    }
}

impl Drop for WakeGuard {
    fn drop(&mut self) {
        unsafe { rcc::cancel_lcpu_active_request() };
    }
}

/// LCPU driver.
///
/// Provides atomic hardware operations for LCPU control.
/// For BLE initialization, use `sifli_radio::bluetooth::BleController`.
pub struct Lcpu {
    _lcpu: crate::PeripheralRef<'static, crate::peripherals::LCPU>,
}

impl fmt::Debug for Lcpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lcpu").finish_non_exhaustive()
    }
}

impl Lcpu {
    /// Create a new LCPU driver.
    pub fn new(lcpu: impl Peripheral<P = crate::peripherals::LCPU> + 'static) -> Self {
        Self {
            _lcpu: lcpu.into_ref(),
        }
    }

    /// Blocking shutdown: reset and hold CPUWAIT.
    pub fn power_off(&self) -> Result<(), LcpuError> {
        self.reset_and_halt()?;
        Ok(())
    }

    /// Reset and halt LCPU. Sets CPUWAIT so LCPU stays halted after reset.
    pub fn reset_and_halt(&self) -> Result<(), LcpuError> {
        if !lpaon::cpuwait() {
            lpaon::set_cpuwait(true);

            rcc::set_lp_lcpu_reset(true);
            rcc::set_lp_mac_reset(true);
            while !rcc::lp_lcpu_reset_asserted() || !rcc::lp_mac_reset_asserted() {}

            if lpaon::sleep_status() {
                lpaon::set_wkup_req(true);
                while lpaon::sleep_status() {}
            }

            rcc::set_lp_lcpu_reset(false);
            rcc::set_lp_mac_reset(false);
        }

        Ok(())
    }

    /// Release LCPU to run. Clears CPUWAIT so LCPU starts executing.
    pub fn release(&self) -> Result<(), LcpuError> {
        lpaon::set_cpuwait(false);
        Ok(())
    }

    /// Set LCPU start vector from the vector table in LPSYS RAM.
    pub fn set_start_vector_from_image(&self) {
        let vector_addr = memory_map::shared::LPSYS_RAM_BASE as *const u32;
        let (sp, pc) = unsafe {
            let sp = core::ptr::read_volatile(vector_addr);
            let pc = core::ptr::read_volatile(vector_addr.add(1));
            (sp, pc)
        };
        lpaon::configure_lcpu_start(sp, pc);
    }

    /// Load firmware image to LPSYS RAM.
    ///
    /// Only needed for A3 and earlier revisions. Letter Series has firmware in ROM.
    pub fn load_firmware(&self, data: &[u8]) -> Result<(), LcpuError> {
        img_install(data)?;
        Ok(())
    }
}

// --- ChipRevision LCPU address methods ---
// Defined here (same crate) because address constants are in ram::memory_map.

impl crate::syscfg::ChipRevision {
    /// LCPU→HCPU 邮箱 CH1 地址（HCI RX 缓冲区）。
    #[inline]
    pub fn lcpu2hcpu_ch1(&self) -> usize {
        match self {
            Self::A3OrEarlier(_) => memory_map::a3::LCPU2HCPU_CH1,
            _ => memory_map::letter::LCPU2HCPU_CH1,
        }
    }

    /// LCPU→HCPU 邮箱 CH2 地址（系统 IPC RX 缓冲区）。
    #[inline]
    pub fn lcpu2hcpu_ch2(&self) -> usize {
        match self {
            Self::A3OrEarlier(_) => memory_map::a3::LCPU2HCPU_CH2,
            _ => memory_map::letter::LCPU2HCPU_CH2,
        }
    }

}

/// Install LCPU firmware image (A3 and earlier only).
fn img_install(image: &[u8]) -> Result<(), ImageError> {
    if image.is_empty() {
        return Err(ImageError::EmptyImage);
    }

    let size_bytes = image.len();
    let max_bytes = memory_map::a3::LPSYS_RAM_SIZE;
    if size_bytes > max_bytes {
        error!(
            "LCPU image too large: {} bytes (max {} bytes)",
            size_bytes, max_bytes
        );
        return Err(ImageError::ImageTooLarge {
            size_bytes,
            max_bytes,
        });
    }

    RamSlice::new(memory_map::shared::LPSYS_RAM_BASE, max_bytes).copy_from_slice(image);

    Ok(())
}
