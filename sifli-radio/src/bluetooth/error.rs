//! BLE initialization and HCI error types.

use sifli_hal::ipc::Error as IpcError;
use sifli_hal::lcpu::LcpuError;

/// Error returned during BLE initialization.
#[derive(Debug)]
pub enum BleInitError {
    /// IPC queue open error.
    Ipc(IpcError),
    /// LCPU power-on error.
    Lcpu(LcpuError),
}

#[cfg(feature = "defmt")]
impl defmt::Format for BleInitError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            BleInitError::Ipc(e) => defmt::write!(f, "IPC error: {}", e),
            BleInitError::Lcpu(e) => defmt::write!(f, "LCPU error: {:?}", defmt::Debug2Format(e)),
        }
    }
}

impl From<IpcError> for BleInitError {
    fn from(e: IpcError) -> Self {
        Self::Ipc(e)
    }
}

impl From<LcpuError> for BleInitError {
    fn from(e: LcpuError) -> Self {
        Self::Lcpu(e)
    }
}
