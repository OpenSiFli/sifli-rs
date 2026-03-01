use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};

use crate::interrupt;
use crate::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;

pub(super) type Regs = crate::pac::mpi::Mpi;

pub(crate) trait SealedInstance:
    crate::rcc::RccEnableReset + crate::rcc::RccGetFreq
{
    fn regs() -> Regs;
    fn code_bus_base() -> usize;
    fn code_bus_end() -> usize;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    /// Interrupt for this MPI instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

dma_trait!(Dma, Instance);

pub(crate) struct State {
    waker: AtomicWaker,
}

impl State {
    const NEW: Self = Self {
        waker: AtomicWaker::new(),
    };

    #[inline(always)]
    pub(crate) fn wake(&self) {
        self.waker.wake();
    }
}

/// Typed interrupt handler for MPI async state.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        compiler_fence(Ordering::SeqCst);
        T::state().wake();
    }
}

#[cfg_attr(
    not(feature = "unstable-mpi-controller"),
    allow(dead_code, unused_imports)
)]
mod controller;
mod shared;
mod xip;

#[cfg_attr(not(feature = "unstable-mpi-controller"), allow(dead_code))]
mod types {
    use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind};

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Error {
        Timeout,
        InvalidConfiguration,
        UnsupportedProfileFeature,
        UnknownJedecId,
        CommandForbiddenInXip,
        AsyncForbiddenInXip,
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
                | Self::UnsupportedProfileFeature
                | Self::UnknownJedecId
                | Self::CommandForbiddenInXip
                | Self::AsyncForbiddenInXip
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

    impl MpiInitConfig {
        /// XIP-friendly init: skip reset and keep existing MPI timing/config.
        pub const fn xip_without_reset() -> Self {
            Self {
                reset: false,
                apply_default_config: false,
                allow_reset_from_xip: true,
            }
        }
    }
}

pub mod nor;

mod instance_impl {
    use super::shared::{
        MPI1_CODE_BUS_BASE, MPI1_CODE_BUS_END, MPI2_CODE_BUS_BASE, MPI2_CODE_BUS_END,
    };
    use super::{Instance, Regs, SealedInstance, State};

    static MPI1_STATE: State = State::NEW;
    static MPI2_STATE: State = State::NEW;

    macro_rules! impl_instance {
        ($periph:ty, $irq:ident, $regs:expr, $base:expr, $end:expr, $state:expr) => {
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

                #[inline(always)]
                fn state() -> &'static State {
                    $state
                }
            }

            impl Instance for $periph {
                type Interrupt = crate::interrupt::typelevel::$irq;
            }
        };
    }

    impl_instance!(
        crate::peripherals::MPI1,
        MPI1,
        crate::pac::MPI1,
        MPI1_CODE_BUS_BASE,
        MPI1_CODE_BUS_END,
        &MPI1_STATE
    );
    impl_instance!(
        crate::peripherals::MPI2,
        MPI2,
        crate::pac::MPI2,
        MPI2_CODE_BUS_BASE,
        MPI2_CODE_BUS_END,
        &MPI2_STATE
    );
}

pub use nor::{
    AddressingPolicy, AhbReadPolicy, AsyncNorFlash, BlockingNorFlash, BuiltInProfile,
    DetectedNorInfo, DtrPolicy, JedecId, NorConfig, NorFamily, NorFlash, NorProfile, ProfileSource,
    QePolicy, ReadMode,
};
pub use types::{AddressSize, Error};

#[cfg(feature = "unstable-mpi-controller")]
pub use controller::{
    AhbCommandConfig, CommandConfig, CommandSlot, FifoClear, FunctionMode, Mpi, MpiInitConfig,
    PhaseMode, Transfer, TransferData,
};
