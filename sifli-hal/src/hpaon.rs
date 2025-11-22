//! HPAON (High-Power Always-On) helper.
//!
//! Wraps the small subset of `HPSYS_AON` registers used to wake LCPU and
//! mirrors the SDK helpers:
//! - `HAL_HPAON_WakeCore(CORE_ID_LCPU)` (HP2LP_REQ / LP_ACTIVE logic only)
//! - `HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()`

use crate::pac::hpsys_aon::HpsysAon as Regs;
use crate::Peripheral;
use core::marker::PhantomData;

#[allow(private_interfaces)]
pub(crate) trait SealedInstance {
    fn regs() -> Regs;
}

/// HPAON peripheral instance trait.
#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {}

#[allow(private_interfaces)]
impl SealedInstance for crate::peripherals::HPSYS_AON {
    fn regs() -> Regs {
        crate::pac::HPSYS_AON
    }
}

impl Instance for crate::peripherals::HPSYS_AON {}

/// HPAON driver.
///
/// Receives the HPSYS_AON peripheral as a parameter.
/// Always-on peripheral, no clock gating or init required.
pub struct Hpaon<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Hpaon<'d, T> {
    /// Create a new HPAON driver instance.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use sifli_hal::hpaon::Hpaon;
    /// use sifli_hal::peripherals;
    ///
    /// let p = sifli_hal::init(Default::default());
    /// let hpaon = Hpaon::new(p.HPSYS_AON);
    /// ```
    pub fn new(_peri: impl Peripheral<P = T> + 'd) -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Async wake of the LPSYS/LP domain that hosts LCPU.
    ///
    /// - Sets `HP2LP_REQ`.
    /// - Polls `LP_ACTIVE` with a timeout using the Embassy timer.
    ///
    /// Returns:
    /// - `true` if `LP_ACTIVE` is seen before the timeout.
    /// - `false` if the timeout expires.
    ///
    /// Mirrors `HAL_HPAON_WakeCore(CORE_ID_LCPU)` in `bf0_hal_hpaon.c:190`.
    pub async fn wake_lcpu_with_timeout(&self, timeout_us: u32) -> bool {
        use embassy_time::{with_timeout, Duration, Timer};

        let r = T::regs();

        // Set HP2LP_REQ to request LPSYS/LP activation.
        r.issr().modify(|w| w.set_hp2lp_req(true));

        // Embassy-style timeout wrapper around a polling future.
        let fut = async {
            loop {
                if r.issr().read().lp_active() {
                    return true;
                }
                Timer::after_micros(10).await;
            }
        };

        matches!(with_timeout(Duration::from_micros(timeout_us as u64), fut).await, Ok(result) if result)
    }

    /// Blocking wake of the LPSYS/LP domain that hosts LCPU.
    ///
    /// Equivalent to the SDK implementation based on `HAL_Delay_us`.
    pub fn wake_lcpu_with_timeout_blocking(&self, timeout_us: u32) -> bool {
        let r = T::regs();

        // Set HP2LP_REQ to request LPSYS/LP activation.
        r.issr().modify(|w| w.set_hp2lp_req(true));

        // Use a simple time-based wait-with-timeout loop:
        // - Poll LP_ACTIVE on each iteration.
        // - Busy-wait for a small fixed step.
        // - Stop and return false if total wait exceeds timeout_us.
        let step_us: u32 = 10;
        let mut waited: u32 = 0;

        loop {
            if r.issr().read().lp_active() {
                return true;
            }

            if waited >= timeout_us {
                return false;
            }

            crate::cortex_m_blocking_delay_us(step_us);
            waited = waited.saturating_add(step_us);
        }
    }

    /// Cancel an LCPU LP_ACTIVE request.
    ///
    /// Clears the `HP2LP_REQ` bit; mirrors `HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()`.
    pub fn cancel_lp_active_request(&self) {
        T::regs().issr().modify(|w| w.set_hp2lp_req(false));
    }
}
