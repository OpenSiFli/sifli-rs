use core::marker::PhantomData;

use embedded_storage::nor_flash::{
    ErrorType, NorFlash as EmbeddedNorFlash, ReadNorFlash as EmbeddedReadNorFlash,
};
use embedded_storage_async::nor_flash::{
    NorFlash as AsyncEmbeddedNorFlash, ReadNorFlash as AsyncEmbeddedReadNorFlash,
};

use super::controller::Mpi;
use super::types::{Error, MpiInitConfig};
use super::xip;
use super::Instance;
use crate::Peripheral;

mod r#async;
mod blocking;
mod detect;
mod profile;

pub use detect::{DetectedNorInfo, JedecId};
pub use profile::{
    AddressingPolicy, AhbReadPolicy, BuiltInProfile, DtrPolicy, NorConfig, NorFamily, NorProfile,
    ProfileSource, QePolicy, ReadMode,
};

pub type BlockingNorFlash<'d, T> = NorFlash<'d, T, crate::mode::Blocking>;
pub type AsyncNorFlash<'d, T> = NorFlash<'d, T, crate::mode::Async>;

pub struct NorFlash<'d, T: Instance, M: crate::mode::Mode> {
    core: blocking::NorBlockingCore<'d, T>,
    async_ctx: Option<r#async::NorAsyncContext<'d, T>>,
    profile: &'static dyn NorProfile,
    detected: Option<DetectedNorInfo>,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance, M: crate::mode::Mode> NorFlash<'d, T, M> {
    fn from_parts(
        core: blocking::NorBlockingCore<'d, T>,
        async_ctx: Option<r#async::NorAsyncContext<'d, T>>,
        detected: Option<DetectedNorInfo>,
    ) -> Self {
        Self {
            profile: core.profile(),
            core,
            async_ctx,
            detected,
            _mode: PhantomData,
        }
    }

    pub fn profile(&self) -> &'static dyn NorProfile {
        self.profile
    }

    pub fn detected_info(&self) -> Option<DetectedNorInfo> {
        self.detected
    }

    pub fn capacity(&self) -> usize {
        self.core.capacity()
    }

    pub fn read_jedec_id(&mut self) -> Result<JedecId, Error> {
        self.core.read_jedec_id()
    }

    pub fn read_status(&mut self) -> Result<u8, Error> {
        self.core.read_status()
    }

    pub fn read_sfdp(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        self.core.read_sfdp(offset, out)
    }

    pub fn read_unique_id(&mut self, out: &mut [u8]) -> Result<(), Error> {
        self.core.read_unique_id(out)
    }

    pub fn enter_deep_power_down(&mut self) -> Result<(), Error> {
        self.core.enter_deep_power_down()
    }

    pub fn release_deep_power_down(&mut self) -> Result<(), Error> {
        self.core.release_deep_power_down()
    }
}

impl<'d, T: Instance> BlockingNorFlash<'d, T> {
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        profile: ProfileSource,
        config: NorConfig,
    ) -> Result<Self, Error> {
        let profile = profile::resolve_profile_source(profile);
        let resolved = profile::validate_and_resolve(profile, &config)?;
        let core = blocking::NorBlockingCore::new(peri, resolved, config)?;

        Ok(Self::from_parts(core, None, None))
    }

    pub fn new_blocking_without_reset(
        peri: impl Peripheral<P = T> + 'd,
        profile: ProfileSource,
        config: NorConfig,
    ) -> Result<Self, Error> {
        let profile = profile::resolve_profile_source(profile);
        let resolved = profile::validate_and_resolve(profile, &config)?;
        let core = blocking::NorBlockingCore::new_without_reset(peri, resolved, config)?;

        Ok(Self::from_parts(core, None, None))
    }

    pub fn new_blocking_auto(
        peri: impl Peripheral<P = T> + 'd,
        config: NorConfig,
    ) -> Result<Self, Error> {
        if config.max_ready_polls == 0 || config.dma_threshold_bytes == 0 {
            return Err(Error::InvalidConfiguration);
        }

        let running_from_xip = xip::running_from_same_instance_xip::<T>();
        let init = if running_from_xip {
            MpiInitConfig::xip_without_reset()
        } else {
            MpiInitConfig::default()
        };

        let mpi = Mpi::with_config(peri, init)?;
        let jedec = detect::read_jedec_id::<T>(config.max_ready_polls)?;
        let (profile, detected) = detect::detect_profile(jedec)?;
        let resolved = profile::validate_and_resolve(profile, &config)?;
        let core = blocking::NorBlockingCore::from_mpi(mpi, resolved, config)?;

        Ok(Self::from_parts(core, None, Some(detected)))
    }

    pub fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        self.core.read_bytes(offset, out)
    }

    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        self.core.write_bytes(offset, data)
    }

    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.core.erase_range(from, to)
    }

    pub fn erase_chip(&mut self) -> Result<(), Error> {
        let capacity = u32::try_from(self.core.capacity()).map_err(|_| Error::OutOfBounds)?;
        self.core.erase_range(0, capacity)
    }

    #[cfg(feature = "unstable-mpi-controller")]
    pub fn free(self) -> Mpi<'d, T> {
        self.core.free()
    }
}

impl<'d, T: Instance> AsyncNorFlash<'d, T> {
    fn async_ctx_mut(&mut self) -> Result<&mut r#async::NorAsyncContext<'d, T>, Error> {
        self.async_ctx.as_mut().ok_or(Error::DmaNotConfigured)
    }

    fn dma_read_base_addr(&self, offset: u32, len: usize) -> Result<usize, Error> {
        let start = offset as usize;
        let end = start.checked_add(len).ok_or(Error::OutOfBounds)?;
        if end > self.core.capacity() {
            return Err(Error::OutOfBounds);
        }
        Ok(T::code_bus_base() + start)
    }

    pub fn new_async(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl super::Dma<T>> + 'd,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, super::InterruptHandler<T>> + 'd,
        profile: ProfileSource,
        config: NorConfig,
    ) -> Result<Self, Error> {
        if xip::running_from_same_instance_xip::<T>() {
            return Err(Error::AsyncForbiddenInXip);
        }

        let profile = profile::resolve_profile_source(profile);
        let resolved = profile::validate_and_resolve(profile, &config)?;
        let core = blocking::NorBlockingCore::new(peri, resolved, config)?;
        let async_ctx = r#async::NorAsyncContext::new(dma)?;

        use crate::interrupt::typelevel::Interrupt as _;
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self::from_parts(core, Some(async_ctx), None))
    }

    pub fn new_async_auto(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl super::Dma<T>> + 'd,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, super::InterruptHandler<T>> + 'd,
        config: NorConfig,
    ) -> Result<Self, Error> {
        if xip::running_from_same_instance_xip::<T>() {
            return Err(Error::AsyncForbiddenInXip);
        }
        if config.max_ready_polls == 0 || config.dma_threshold_bytes == 0 {
            return Err(Error::InvalidConfiguration);
        }

        let mpi = Mpi::with_config(peri, MpiInitConfig::default())?;
        let jedec = detect::read_jedec_id::<T>(config.max_ready_polls)?;
        let (profile, detected) = detect::detect_profile(jedec)?;
        let resolved = profile::validate_and_resolve(profile, &config)?;
        let core = blocking::NorBlockingCore::from_mpi(mpi, resolved, config)?;
        let async_ctx = r#async::NorAsyncContext::new(dma)?;

        use crate::interrupt::typelevel::Interrupt as _;
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Ok(Self::from_parts(core, Some(async_ctx), Some(detected)))
    }

    #[inline(always)]
    fn ensure_async_ready(&self) -> Result<(), Error> {
        if xip::running_from_same_instance_xip::<T>() {
            return Err(Error::AsyncForbiddenInXip);
        }
        if self.async_ctx.is_none() {
            return Err(Error::DmaNotConfigured);
        }
        Ok(())
    }

    pub async fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error> {
        self.ensure_async_ready()?;
        if out.len() >= self.core.dma_threshold_bytes() {
            let base = self.dma_read_base_addr(offset, out.len())?;
            self.async_ctx_mut()?.read_code_bus_range(base, out).await
        } else {
            self.core.read_bytes(offset, out)
        }
    }

    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        self.ensure_async_ready()?;
        self.core.write_bytes(offset, data)
    }

    pub async fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.ensure_async_ready()?;
        self.core.erase_range(from, to)
    }

    pub async fn erase_chip(&mut self) -> Result<(), Error> {
        self.ensure_async_ready()?;
        let capacity = u32::try_from(self.core.capacity()).map_err(|_| Error::OutOfBounds)?;
        self.core.erase_range(0, capacity)
    }
}

impl<T: Instance> ErrorType for BlockingNorFlash<'_, T> {
    type Error = Error;
}

impl<T: Instance> EmbeddedReadNorFlash for BlockingNorFlash<'_, T> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.core.read_bytes(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.core.capacity()
    }
}

impl<T: Instance> EmbeddedNorFlash for BlockingNorFlash<'_, T> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.core.erase_range(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.core.write_bytes(offset, bytes)
    }
}

impl<T: Instance> ErrorType for AsyncNorFlash<'_, T> {
    type Error = Error;
}

impl<T: Instance> AsyncEmbeddedReadNorFlash for AsyncNorFlash<'_, T> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        AsyncNorFlash::read(self, offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.core.capacity()
    }
}

impl<T: Instance> AsyncEmbeddedNorFlash for AsyncNorFlash<'_, T> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        AsyncNorFlash::erase(self, from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        AsyncNorFlash::write(self, offset, bytes).await
    }
}
