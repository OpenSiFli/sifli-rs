use crate::Peripheral;

pub(super) type Regs = crate::pac::mpi::Mpi;

pub(crate) trait SealedInstance:
    crate::rcc::RccEnableReset + crate::rcc::RccGetFreq
{
    fn regs() -> Regs;
    fn code_bus_base() -> usize;
    fn code_bus_end() -> usize;
}

#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {}

mod shared;

mod types {
    use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind};

    use super::shared::DEFAULT_DMA_THRESHOLD_BYTES;

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Error {
        Timeout,
        InvalidConfiguration,
        UnsafeResetInXip,
        CapacityExceedsWindow,
        RequiresPreconfigured4ByteMode,
        InvalidLength,
        NotAligned,
        OutOfBounds,
        DmaNotConfigured,
    }

    impl NorFlashError for Error {
        fn kind(&self) -> NorFlashErrorKind {
            match self {
                Self::NotAligned => NorFlashErrorKind::NotAligned,
                Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
                Self::Timeout
                | Self::InvalidConfiguration
                | Self::UnsafeResetInXip
                | Self::CapacityExceedsWindow
                | Self::RequiresPreconfigured4ByteMode
                | Self::InvalidLength
                | Self::DmaNotConfigured => NorFlashErrorKind::Other,
            }
        }
    }

    impl From<NorFlashErrorKind> for Error {
        fn from(value: NorFlashErrorKind) -> Self {
            match value {
                NorFlashErrorKind::NotAligned => Self::NotAligned,
                NorFlashErrorKind::OutOfBounds => Self::OutOfBounds,
                _ => Self::InvalidConfiguration,
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PhaseMode {
        None,
        Single,
        Dual,
        Quad,
        QuadDdr,
    }

    impl PhaseMode {
        pub(super) const fn bits(self) -> u8 {
            match self {
                Self::None => 0,
                Self::Single => 1,
                Self::Dual => 2,
                Self::Quad => 3,
                Self::QuadDdr => 7,
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AddressSize {
        OneByte,
        TwoBytes,
        ThreeBytes,
        FourBytes,
    }

    impl AddressSize {
        pub(super) const fn bits(self) -> u8 {
            match self {
                Self::OneByte => 0,
                Self::TwoBytes => 1,
                Self::ThreeBytes => 2,
                Self::FourBytes => 3,
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FunctionMode {
        Read,
        Write,
    }

    impl FunctionMode {
        pub(super) const fn is_write(self) -> bool {
            matches!(self, Self::Write)
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FifoClear {
        Rx,
        Tx,
        RxTx,
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum CommandSlot {
        Cmd1,
        Cmd2,
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct CommandConfig {
        pub function_mode: FunctionMode,
        pub instruction_mode: PhaseMode,
        pub address_mode: PhaseMode,
        pub address_size: AddressSize,
        pub alternate_mode: PhaseMode,
        pub alternate_size: AddressSize,
        pub data_mode: PhaseMode,
        pub dummy_cycles: u8,
    }

    impl CommandConfig {
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

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct AhbCommandConfig {
        pub instruction_mode: PhaseMode,
        pub address_mode: PhaseMode,
        pub address_size: AddressSize,
        pub alternate_mode: PhaseMode,
        pub alternate_size: AddressSize,
        pub data_mode: PhaseMode,
        pub dummy_cycles: u8,
    }

    impl AhbCommandConfig {
        pub const fn single_io(address_size: AddressSize) -> Self {
            Self {
                instruction_mode: PhaseMode::Single,
                address_mode: PhaseMode::Single,
                address_size,
                alternate_mode: PhaseMode::None,
                alternate_size: AddressSize::OneByte,
                data_mode: PhaseMode::Single,
                dummy_cycles: 0,
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct MpiInitConfig {
        pub reset: bool,
        pub apply_default_config: bool,
        pub allow_reset_from_xip: bool,
    }

    impl Default for MpiInitConfig {
        fn default() -> Self {
            Self {
                reset: true,
                apply_default_config: true,
                allow_reset_from_xip: false,
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct NorFlashCommandSet {
        pub write_enable: u8,
        pub enter_4byte_address: Option<u8>,
        pub read_status: u8,
        pub ahb_read: u8,
        pub ahb_read_4byte: Option<u8>,
        pub page_program: u8,
        pub page_program_4byte: Option<u8>,
        pub sector_erase: u8,
        pub sector_erase_4byte: Option<u8>,
        pub block_erase_32k: Option<u8>,
        pub block_erase_32k_4byte: Option<u8>,
        pub block_erase_64k: Option<u8>,
        pub block_erase_64k_4byte: Option<u8>,
        pub chip_erase: Option<u8>,
        pub read_jedec_id: u8,
        pub deep_power_down: Option<u8>,
        pub release_power_down: Option<u8>,
        pub reset_enable: Option<u8>,
        pub reset: Option<u8>,
        pub read_sfdp: Option<u8>,
        pub read_unique_id: Option<u8>,
    }

    impl NorFlashCommandSet {
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
                block_erase_32k: Some(0x52),
                block_erase_32k_4byte: None,
                block_erase_64k: Some(0xD8),
                block_erase_64k_4byte: Some(0xDC),
                chip_erase: Some(0xC7),
                read_jedec_id: 0x9f,
                deep_power_down: Some(0xB9),
                release_power_down: Some(0xAB),
                reset_enable: Some(0x66),
                reset: Some(0x99),
                read_sfdp: Some(0x5A),
                read_unique_id: Some(0x4B),
            }
        }
    }

    impl Default for NorFlashCommandSet {
        fn default() -> Self {
            Self::common_spi_nor()
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct NorFlashConfig {
        pub capacity: usize,
        pub page_size: usize,
        pub address_size: AddressSize,
        pub max_ready_polls: u32,
        pub allow_preconfigured_4byte_in_xip: bool,
        pub dma_threshold_bytes: usize,
        pub commands: NorFlashCommandSet,
    }

    impl NorFlashConfig {
        pub const fn new(capacity: usize) -> Self {
            Self {
                capacity,
                page_size: 256,
                address_size: AddressSize::ThreeBytes,
                max_ready_polls: 2_000_000,
                allow_preconfigured_4byte_in_xip: false,
                dma_threshold_bytes: DEFAULT_DMA_THRESHOLD_BYTES,
                commands: NorFlashCommandSet::common_spi_nor(),
            }
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct NorFlashPartition {
        pub offset: u32,
        pub size: u32,
    }

    impl NorFlashPartition {
        pub const fn new(offset: u32, size: u32) -> Self {
            Self { offset, size }
        }
    }
}

mod driver;
mod nor;

mod instance_impl {
    use super::shared::{
        MPI1_CODE_BUS_BASE, MPI1_CODE_BUS_END, MPI2_CODE_BUS_BASE, MPI2_CODE_BUS_END,
    };
    use super::{Instance, Regs, SealedInstance};

    macro_rules! impl_instance {
        ($periph:ty, $regs:expr, $base:expr, $end:expr) => {
            impl SealedInstance for $periph {
                #[inline(always)]
                fn regs() -> Regs {
                    $regs
                }

                #[inline(always)]
                fn code_bus_base() -> usize {
                    $base
                }

                #[inline(always)]
                fn code_bus_end() -> usize {
                    $end
                }
            }

            impl Instance for $periph {}
        };
    }

    impl_instance!(
        crate::peripherals::MPI1,
        crate::pac::MPI1,
        MPI1_CODE_BUS_BASE,
        MPI1_CODE_BUS_END
    );
    impl_instance!(
        crate::peripherals::MPI2,
        crate::pac::MPI2,
        MPI2_CODE_BUS_BASE,
        MPI2_CODE_BUS_END
    );
}


pub use driver::Mpi;
pub use nor::{MpiNorFlash, MpiNorPartition};
pub use types::{
    AddressSize, AhbCommandConfig, CommandConfig, CommandSlot, Error, FifoClear, FunctionMode,
    MpiInitConfig, NorFlashCommandSet, NorFlashConfig, NorFlashPartition, PhaseMode,
};
