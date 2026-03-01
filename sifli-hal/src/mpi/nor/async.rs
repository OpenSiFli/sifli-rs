use core::cmp::min;
use core::marker::PhantomData;

use crate::dma::{ChannelAndRequest, Increment, Transfer, TransferOptions};
use crate::Peripheral;

pub(super) struct NorAsyncContext<'d, T: super::super::Instance> {
    dma: ChannelAndRequest<'d>,
    phantom: PhantomData<T>,
}

impl<'d, T: super::super::Instance> NorAsyncContext<'d, T> {
    pub(super) fn new(
        dma: impl Peripheral<P = impl super::super::Dma<T>> + 'd,
    ) -> Result<Self, super::super::types::Error> {
        let dma = new_dma!(dma).ok_or(super::super::types::Error::DmaNotConfigured)?;
        Ok(Self {
            dma,
            phantom: PhantomData,
        })
    }

    pub(super) async fn read_code_bus_range(
        &mut self,
        src_code_bus_addr: usize,
        out: &mut [u8],
    ) -> Result<(), super::super::types::Error> {
        if out.is_empty() {
            return Ok(());
        }

        const DMA_MAX_XFER_BYTES: usize = 0xFFFF;
        let mut done = 0usize;

        while done < out.len() {
            let step = min(DMA_MAX_XFER_BYTES, out.len() - done);
            let src = (src_code_bus_addr + done) as *const u8;
            let dst = out[done..done + step].as_mut_ptr();

            // SAFETY: The source and destination regions are valid for `step` bytes,
            // non-overlapping, and remain alive for the transfer lifetime.
            let transfer = unsafe {
                Transfer::new_transfer_raw(
                    self.dma.channel.reborrow(),
                    src,
                    dst,
                    step,
                    Increment::Both,
                    TransferOptions::default(),
                )
            };
            transfer.await;
            done += step;
        }

        invalidate_dcache_range(out.as_mut_ptr() as usize, out.len());
        Ok(())
    }
}

fn invalidate_dcache_range(addr: usize, len: usize) {
    if len == 0 {
        return;
    }

    const DCACHE_LINE_SIZE: usize = 32;
    let aligned_start = addr & !(DCACHE_LINE_SIZE - 1);
    let aligned_end = addr.saturating_add(len).saturating_add(DCACHE_LINE_SIZE - 1)
        & !(DCACHE_LINE_SIZE - 1);
    let aligned_len = aligned_end.saturating_sub(aligned_start);

    let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
    // SAFETY: We pass an aligned address/length range derived from `out`.
    unsafe { scb.invalidate_dcache_by_address(aligned_start, aligned_len) };
}
