//! TRNG (True Random Number Generator)

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_hal_internal::Peripheral;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Binding;
use crate::interrupt::InterruptExt;
use crate::mode::{Async, Blocking, Mode};
use crate::pac::TRNG;
use crate::{interrupt, peripherals, rcc};

static WAKER: AtomicWaker = AtomicWaker::new();

/// TRNG error.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Seed generation failed or timed out.
    SeedError,
    /// Random number generation timed out.
    Timeout,
}

/// TRNG driver.
pub struct Rng<'d, M: Mode> {
    _phantom: PhantomData<(&'d peripherals::TRNG, M)>,
}

impl<'d, M: Mode> Rng<'d, M> {
    fn new_inner(_peri: impl Peripheral<P = peripherals::TRNG> + 'd) -> Self {
        rcc::enable_and_reset::<peripherals::TRNG>();

        // Clear stop bits to allow engine operation
        TRNG.ctrl().write(|w| {
            w.set_gen_seed_stop(false);
            w.set_gen_rand_num_stop(false);
        });

        Self {
            _phantom: PhantomData,
        }
    }

    /// Generate seed and wait for it to become valid (blocking).
    fn blocking_seed(&self) -> Result<(), Error> {
        TRNG.ctrl().modify(|w| w.set_gen_seed_start(true));

        // Poll for seed_valid
        let mut timeout = 100_000u32;
        while !TRNG.stat().read().seed_valid() {
            timeout -= 1;
            if timeout == 0 {
                return Err(Error::SeedError);
            }
        }
        Ok(())
    }

    /// Trigger random number generation and wait (blocking).
    fn blocking_rand(&self) -> Result<(), Error> {
        TRNG.ctrl().modify(|w| w.set_gen_rand_num_start(true));

        // Poll for rand_num_valid
        let mut timeout = 100_000u32;
        while !TRNG.stat().read().rand_num_valid() {
            timeout -= 1;
            if timeout == 0 {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    /// Read all 8 random number registers into a buffer.
    fn read_rand_nums(&self, buf: &mut [u32; 8]) {
        buf[0] = TRNG.rand_num0().read().val();
        buf[1] = TRNG.rand_num1().read().val();
        buf[2] = TRNG.rand_num2().read().val();
        buf[3] = TRNG.rand_num3().read().val();
        buf[4] = TRNG.rand_num4().read().val();
        buf[5] = TRNG.rand_num5().read().val();
        buf[6] = TRNG.rand_num6().read().val();
        buf[7] = TRNG.rand_num7().read().val();
    }

    /// Generate a single random u32 (blocking).
    pub fn blocking_generate(&mut self) -> Result<u32, Error> {
        self.blocking_seed()?;
        self.blocking_rand()?;
        Ok(TRNG.rand_num0().read().val())
    }

    /// Fill a byte buffer with random data (blocking).
    ///
    /// Each hardware generation produces 32 bytes (8 × u32).
    pub fn blocking_fill_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        let mut rand_buf = [0u32; 8];
        let mut offset = 0;

        while offset < buf.len() {
            self.blocking_seed()?;
            self.blocking_rand()?;
            self.read_rand_nums(&mut rand_buf);

            for word in &rand_buf {
                let bytes = word.to_le_bytes();
                for &b in &bytes {
                    if offset >= buf.len() {
                        return Ok(());
                    }
                    buf[offset] = b;
                    offset += 1;
                }
            }
        }
        Ok(())
    }
}

impl<'d, M: Mode> Drop for Rng<'d, M> {
    fn drop(&mut self) {
        TRNG.ctrl().write(|w| {
            w.set_gen_seed_stop(true);
            w.set_gen_rand_num_stop(true);
        });
        rcc::disable::<peripherals::TRNG>();
    }
}

impl<'d> Rng<'d, Blocking> {
    /// Create a new TRNG driver in blocking mode.
    pub fn new_blocking(peri: impl Peripheral<P = peripherals::TRNG> + 'd) -> Self {
        Self::new_inner(peri)
    }
}

/// TRNG interrupt handler.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::TRNG> for InterruptHandler {
    unsafe fn on_interrupt() {
        let irq = TRNG.irq().read();
        if irq.seed_gen_done() || irq.rand_num_avail() {
            // Disable interrupt masks to prevent repeated triggering
            TRNG.irq().modify(|w| {
                w.set_seed_gen_done_msk(false);
                w.set_rand_num_avail_msk(false);
            });
            WAKER.wake();
        }
    }
}

impl<'d> Rng<'d, Async> {
    /// Create a new TRNG driver in asynchronous mode.
    pub fn new(
        peri: impl Peripheral<P = peripherals::TRNG> + 'd,
        _irq: impl Binding<interrupt::typelevel::TRNG, InterruptHandler>,
    ) -> Self {
        let s = Self::new_inner(peri);

        let irq = interrupt::TRNG;
        irq.unpend();
        unsafe { irq.enable() };

        s
    }

    /// Wait for seed generation to complete asynchronously.
    async fn async_seed(&self) -> Result<(), Error> {
        // Enable seed_gen_done interrupt mask
        TRNG.irq().modify(|w| w.set_seed_gen_done_msk(true));

        // Start seed generation
        TRNG.ctrl().modify(|w| w.set_gen_seed_start(true));

        poll_fn(|cx| {
            WAKER.register(cx.waker());
            // Re-enable mask in case it was cleared
            TRNG.irq().modify(|w| w.set_seed_gen_done_msk(true));
            compiler_fence(Ordering::SeqCst);

            if TRNG.stat().read().seed_valid() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    /// Wait for random number generation to complete asynchronously.
    async fn async_rand(&self) -> Result<(), Error> {
        // Enable rand_num_avail interrupt mask
        TRNG.irq().modify(|w| w.set_rand_num_avail_msk(true));

        // Start random number generation
        TRNG.ctrl().modify(|w| w.set_gen_rand_num_start(true));

        poll_fn(|cx| {
            WAKER.register(cx.waker());
            TRNG.irq().modify(|w| w.set_rand_num_avail_msk(true));
            compiler_fence(Ordering::SeqCst);

            if TRNG.stat().read().rand_num_valid() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    /// Generate a single random u32 asynchronously.
    pub async fn generate(&mut self) -> Result<u32, Error> {
        self.async_seed().await?;
        self.async_rand().await?;
        Ok(TRNG.rand_num0().read().val())
    }

    /// Fill a byte buffer with random data asynchronously.
    ///
    /// Each hardware generation produces 32 bytes (8 × u32).
    pub async fn fill_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        let mut rand_buf = [0u32; 8];
        let mut offset = 0;

        while offset < buf.len() {
            self.async_seed().await?;
            self.async_rand().await?;
            self.read_rand_nums(&mut rand_buf);

            for word in &rand_buf {
                let bytes = word.to_le_bytes();
                for &b in &bytes {
                    if offset >= buf.len() {
                        return Ok(());
                    }
                    buf[offset] = b;
                    offset += 1;
                }
            }
        }
        Ok(())
    }
}

impl rand_core::RngCore for Rng<'_, Blocking> {
    fn next_u32(&mut self) -> u32 {
        self.blocking_generate().expect("TRNG failed")
    }

    fn next_u64(&mut self) -> u64 {
        let lo = self.next_u32() as u64;
        let hi = self.next_u32() as u64;
        (hi << 32) | lo
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.blocking_fill_bytes(dest).expect("TRNG failed");
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.blocking_fill_bytes(dest)
            .map_err(|_| rand_core::Error::from(core::num::NonZeroU32::new(1).unwrap()))
    }
}

impl rand_core::CryptoRng for Rng<'_, Blocking> {}
