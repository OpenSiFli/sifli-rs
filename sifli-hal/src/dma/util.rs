// The following code is modified from embassy-stm32 under MIT license
// https://github.com/embassy-rs/embassy/tree/main/embassy-stm32
// Special thanks to the Embassy Project and its contributors for their work!

use super::word::Word;
use super::{AnyChannel, Request, Transfer, TransferOptions};
use crate::PeripheralRef;

/// Convenience wrapper, contains a channel and a request number.
///
/// Commonly used in peripheral drivers that own DMA channels.
pub(crate) struct ChannelAndRequest<'d> {
    pub channel: PeripheralRef<'d, AnyChannel>,
    pub request: Request,
}

impl<'d> ChannelAndRequest<'d> {
    pub unsafe fn read<'a, W: Word>(
        &'a mut self,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Transfer<'a> {
        Transfer::new_read(
            self.channel.reborrow(),
            self.request,
            peri_addr,
            buf,
            options,
        )
    }

    #[allow(unused)]
    pub unsafe fn read_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        peri_addr: *mut PW,
        buf: *mut [MW],
        options: TransferOptions,
    ) -> Transfer<'a> {
        Transfer::new_read_raw(
            self.channel.reborrow(),
            self.request,
            peri_addr,
            buf,
            options,
        )
    }

    pub unsafe fn write<'a, W: Word>(
        &'a mut self,
        buf: &'a [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Transfer<'a> {
        Transfer::new_write(
            self.channel.reborrow(),
            self.request,
            buf,
            peri_addr,
            options,
        )
    }

    #[allow(unused)]
    pub unsafe fn write_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        buf: *const [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        Transfer::new_write_raw(
            self.channel.reborrow(),
            self.request,
            buf,
            peri_addr,
            options,
        )
    }

    #[allow(dead_code)]
    pub unsafe fn write_repeated<'a, W: Word>(
        &'a mut self,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Transfer<'a> {
        Transfer::new_write_repeated(
            self.channel.reborrow(),
            self.request,
            repeated,
            count,
            peri_addr,
            options,
        )
    }
}
