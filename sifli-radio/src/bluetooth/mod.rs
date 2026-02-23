//! Bluetooth subsystem.
//!
//! Provides BLE controller initialization, HCI transport, and RF calibration.

pub mod config;
pub(crate) mod controller;
pub mod error;
pub mod hci;
mod nvds;
pub mod rf_cal;
mod rom_config;

pub use config::BleInitConfig;
pub use error::BleInitError;
pub use hci::{HciError, IpcHciTransport};

use bt_hci::cmd;
use bt_hci::controller::{Controller, ControllerCmdAsync, ControllerCmdSync, ExternalController};
use bt_hci::data;
use bt_hci::{ControllerToHostPacket, FixedSizeValue};

use sifli_hal::dma::Channel;
use sifli_hal::ipc;
use sifli_hal::lcpu::{Lcpu, LcpuError, WakeGuard};
use sifli_hal::{interrupt, peripherals, Peripheral};
use sifli_hal::{patch, rcc, syscfg};

/// BLE initialization orchestration.
///
/// Performs:
/// 1. NVDS write (before LCPU boot)
/// 2. LCPU hardware startup (reset, ROM config, firmware, patches, RF cal)
/// 3. Warmup event consumption + controller init
pub(crate) async fn init_ble<R>(
    lcpu: &Lcpu,
    config: &BleInitConfig,
    dma_ch: impl Peripheral<P = impl Channel>,
    hci_rx: &mut R,
) -> Result<(), BleInitError>
where
    R: embedded_io_async::Read,
{
    // Phase 1: Write NVDS to LCPU shared memory (before boot)
    {
        let _w = unsafe { WakeGuard::acquire() };
        nvds::write_default(&config.ble.bd_addr, config.rom.enable_lxt);
    }

    // Phase 2: LCPU boot sequence
    {
        let _w = unsafe { WakeGuard::acquire() };

        // Reset and halt LCPU
        debug!("Step 1: Resetting and halting LCPU");
        lcpu.reset_and_halt()?;

        // Write ROM configuration
        debug!("Step 2: Writing ROM configuration");
        rom_config::write(&config.rom, &config.ble.controller);

        // Enforce frequency limit while loading
        if !config.skip_frequency_check {
            debug!("Step 3: Ensuring LCPU frequency ≤ 24MHz");
            rcc::ensure_safe_lcpu_frequency().map_err(|_| LcpuError::RccError)?;
        }

        // Load firmware for A3 and earlier
        let is_letter = syscfg::read_idr().revision().is_letter_series();
        if !is_letter {
            debug!("Step 4: Loading LCPU firmware image");
            if let Some(firmware) = config.firmware {
                lcpu.load_firmware(firmware)?;
            } else {
                return Err(LcpuError::FirmwareMissing.into());
            }
        } else {
            debug!("Step 4: Skipping firmware (Letter Series)");
        }

        // Configure start address
        debug!("Step 5: Setting LCPU start vector");
        lcpu.set_start_vector_from_image();

        // Install patches
        debug!("Step 6: Installing patches");
        let patch_data = if is_letter {
            config.patch_letter
        } else {
            config.patch_a3
        };
        if let Some(data) = patch_data {
            patch::install(data.list, data.bin).map_err(LcpuError::from)?;
        }

        // RF calibration
        if !config.disable_rf_cal {
            debug!("Step 7: RF calibration");
            rf_cal::bt_rf_cal(dma_ch);
        }

        // Release LCPU
        debug!("Step 8: Releasing LCPU");
        lcpu.release()?;
    }

    // Phase 3: Warmup event + controller init
    {
        let _w = unsafe { WakeGuard::acquire() };

        // Consume warmup event
        hci::consume_warmup_event(hci_rx).await?;

        // Controller initialization
        controller::init(&config.ble.controller);
    }

    Ok(())
}

//=============================================================================
// BLE Controller
//=============================================================================

/// High-level BLE controller that owns the LCPU and HCI transport.
///
/// # Example
///
/// ```no_run
/// use sifli_hal::{bind_interrupts, ipc};
/// use sifli_radio::bluetooth::{BleController, BleInitConfig};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => ipc::InterruptHandler;
/// });
///
/// async fn example() {
///     let p = sifli_hal::init(Default::default());
///     let controller = BleController::new(
///         p.LCPU, p.MAILBOX1_CH1, p.DMAC2_CH8, Irqs,
///         &BleInitConfig::default().sleep_enabled(true),
///     ).await.unwrap();
/// }
/// ```
pub struct BleController<const SLOTS: usize = 4> {
    lcpu: Lcpu,
    inner: ExternalController<IpcHciTransport, SLOTS>,
}

impl<const SLOTS: usize> BleController<SLOTS> {
    /// Initialize BLE and create a controller in one step.
    pub async fn new(
        lcpu_peri: impl Peripheral<P = peripherals::LCPU> + 'static,
        mailbox: impl Peripheral<P = peripherals::MAILBOX1_CH1>,
        dma_ch: impl Peripheral<P = impl Channel>,
        irq: impl interrupt::typelevel::Binding<
            interrupt::typelevel::MAILBOX2_CH1,
            ipc::InterruptHandler,
        >,
        config: &BleInitConfig,
    ) -> Result<Self, BleInitError> {
        let mut ipc_driver = ipc::Ipc::new(mailbox, irq, ipc::Config::default());
        let queue = ipc_driver.open_queue(ipc::QueueConfig::qid0_hci())?;
        let (mut rx, tx) = queue.split();
        let lcpu = Lcpu::new(lcpu_peri);

        // BLE init orchestration
        init_ble(&lcpu, config, dma_ch, &mut rx).await?;

        let transport = IpcHciTransport::from_parts(rx, tx);
        Ok(Self {
            lcpu,
            inner: ExternalController::new(transport),
        })
    }

    /// Enable or disable BLE controller sleep at runtime.
    pub fn set_sleep_enabled(&self, enabled: bool) {
        let _w = unsafe { WakeGuard::acquire() };
        sifli_hal::cortex_m_blocking_delay_us(5_000);
        if enabled {
            controller::enable_ble_sleep();
        } else {
            controller::disable_ble_sleep();
        }
    }

    /// Shut down BLE and power off LCPU.
    pub fn shutdown(self) {
        let Self { lcpu, .. } = self;
        let _ = lcpu.power_off();
    }
}

impl<const SLOTS: usize> embedded_io::ErrorType for BleController<SLOTS> {
    type Error = HciError;
}

impl<const SLOTS: usize> Controller for BleController<SLOTS> {
    async fn write_acl_data(&self, packet: &data::AclPacket<'_>) -> Result<(), Self::Error> {
        self.inner.write_acl_data(packet).await
    }

    async fn write_sync_data(&self, packet: &data::SyncPacket<'_>) -> Result<(), Self::Error> {
        self.inner.write_sync_data(packet).await
    }

    async fn write_iso_data(&self, packet: &data::IsoPacket<'_>) -> Result<(), Self::Error> {
        self.inner.write_iso_data(packet).await
    }

    async fn read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        self.inner.read(buf).await
    }
}

impl<C, const SLOTS: usize> ControllerCmdSync<C> for BleController<SLOTS>
where
    C: cmd::SyncCmd,
    C::Return: FixedSizeValue,
{
    async fn exec(&self, cmd: &C) -> Result<C::Return, cmd::Error<Self::Error>> {
        ControllerCmdSync::exec(&self.inner, cmd).await
    }
}

impl<C, const SLOTS: usize> ControllerCmdAsync<C> for BleController<SLOTS>
where
    C: cmd::AsyncCmd,
{
    async fn exec(&self, cmd: &C) -> Result<(), cmd::Error<Self::Error>> {
        ControllerCmdAsync::exec(&self.inner, cmd).await
    }
}
