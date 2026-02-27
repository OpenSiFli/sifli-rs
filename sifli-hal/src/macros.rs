#![macro_use]

macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name.into_ref();
        pin.set_function(pin.fsel(), $af_type);
        pin.set_cfg_pin();
        Some(pin.map_into())
    }};
}

macro_rules! init_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name.into_ref();
        pin.set_function(pin.fsel(), $af_type);
        pin.set_cfg_pin();
    }};
}

macro_rules! pin_trait {
    ($signal:ident, $instance:path $(, $mode:path)?) => {
        #[doc = concat!(stringify!($signal), " pin trait")]
        pub trait $signal<T: $instance $(, M: $mode)?>: crate::gpio::Pin {
            #[doc = concat!("Get the fsel number needed to use this pin as ", stringify!($signal))]
            fn fsel(&self) -> u8;

            #[doc = concat!("Configure HPSYS_CFG if needed", stringify!($signal))]
            fn set_cfg_pin(&self) {}
        }
    };
}

macro_rules! dma_trait {
    ($signal:ident, $instance:path$(, $mode:path)?) => {
        #[doc = concat!(stringify!($signal), " DMA request trait")]
        pub trait $signal<T: $instance $(, M: $mode)?>: crate::dma::Channel {
            #[doc = concat!("Get the DMA request number needed to use this channel as", stringify!($signal))]
            fn request(&self) -> crate::dma::Request;
        }
    };
}

#[allow(unused)]
macro_rules! dma_trait_impl {
    (crate::$mod:ident::$trait:ident$(<$mode:ident>)?, $instance:ident, $channel:ident, $request:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance $(, crate::$mod::$mode)?> for crate::peripherals::$channel {
            fn request(&self) -> crate::dma::Request {
                $request
            }
        }
    };
}

macro_rules! new_dma {
    ($name:ident) => {{
        let dma = $name.into_ref();
        let request = dma.request();
        Some(crate::dma::ChannelAndRequest {
            channel: dma.map_into(),
            request,
        })
    }};
}
