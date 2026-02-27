/// Universal Serial Bus (USB)
///
/// See more: https://github.com/decaday/musb
use core::marker::PhantomData;

use embassy_usb_driver::{self as driver, EndpointAddress, EndpointType};
use musb::MusbInstance;
use musb::{Bus, ControlPipe, Endpoint, In, MusbDriver, Out, UsbInstance};

use crate::gpio::hpsys::HpsysPin;
use crate::interrupt::typelevel::Interrupt;
use crate::pac::HPSYS_CFG;
use crate::rcc::{get_clk_usb_div, get_clk_usb_source, RccEnableReset, RccGetFreq};
use crate::{interrupt, Peripheral};

fn init<T: Instance>() {
    let freq = T::frequency();
    if let Some(f) = freq {
        if f.0 != 60_000_000 {
            panic!(
                "USB clock must be 60MHz, clock is {:?}, clock source:{:?}, div:{}",
                freq,
                get_clk_usb_source(),
                get_clk_usb_div()
            );
        }
    } else {
        panic!(
            "USB clock is not configured, clock source:{:?}",
            get_clk_usb_source()
        );
    }
    // rcc::enable_and_reset::<T>();
    T::rcc_enable();

    // 58: TODO
    // 52 & 56
    HPSYS_CFG.usbcr().modify(|w| {
        w.set_dm_pd(true);
        w.set_dp_en(true);
        w.set_usb_en(true);
    });

    // Enable USB PHY
    UsbInstance::regs().usbcfg().modify(|w| {
        w.set_avalid(true);
        w.set_avalid_dr(true);
        // w.set_usb_en(true);
    });

    musb::common_impl::endpoints_set_rx_dualpacket_enabled::<UsbInstance>(0x00);
    musb::common_impl::endpoints_set_tx_dualpacket_enabled::<UsbInstance>(0x00);

    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };

    HPSYS_CFG.usbcr().modify(|w| {
        w.set_usb_en(true);
    });

    // info!("USB power {:b}", UsbInstance::regs().power().read().0);

    UsbInstance::regs().power().modify(|w| {
        w.set_hs_enab(false);
    });

    UsbInstance::regs().power().modify(|w| {
        w.set_soft_conn(true);
    });

    // UsbInstance::regs().devctl().modify(|w| {
    //     w.set_session(true);
    // });

    UsbInstance::regs().index().write(|w| w.set_index(0));

    // TODO: Should this?
    // crate::blocking_delay_us(1000);
}

/// USB driver.
pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    inner: MusbDriver<'d, UsbInstance>,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Create a new USB driver.
    pub fn new(
        _usb: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: impl Peripheral<P = impl DpPin<T>> + 'd,
        dm: impl Peripheral<P = impl DmPin<T>> + 'd,
    ) -> Self {
        let mut dm = HpsysPin::new(dm.into_ref().pin_bank());
        dm.disable_interrupt();
        dm.set_as_analog();

        let mut dp = HpsysPin::new(dp.into_ref().pin_bank());
        dp.disable_interrupt();
        dp.set_as_analog();

        init::<T>();
        Self {
            inner: MusbDriver::new(),
            phantom: PhantomData,
        }
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, UsbInstance, Out>;
    type EndpointIn = Endpoint<'d, UsbInstance, In>;
    type ControlPipe = ControlPipe<'d, UsbInstance>;
    type Bus = Bus<'d, UsbInstance>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        self.inner
            .alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        self.inner
            .alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(
        self,
        control_max_packet_size: u16,
    ) -> (Bus<'d, UsbInstance>, ControlPipe<'d, UsbInstance>) {
        self.inner.start(control_max_packet_size)
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        musb::on_interrupt::<UsbInstance>();
    }
}

trait SealedInstance: RccEnableReset + RccGetFreq {}

/// USB instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Interrupt for this USB instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl SealedInstance for crate::peripherals::USBC {}
impl Instance for crate::peripherals::USBC {
    type Interrupt = crate::interrupt::typelevel::USBC;
}

// Internal PHY pins
pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

// SDK set them to analog.
// Datasheet shows: PA35 AF2 #USB11_DP
impl DpPin<crate::peripherals::USBC> for crate::peripherals::PA35 {
    fn fsel(&self) -> u8 {
        0x2
    }
}

impl DmPin<crate::peripherals::USBC> for crate::peripherals::PA36 {
    fn fsel(&self) -> u8 {
        0x2
    }
}

// impl SealedInstance for  {}
