//! USB HID keyboard example
//!
//! If device enumeration fails, try powering on first and wait for the
//! bootloader to finish (at least 3s) before plugging in the USB cable,
//! and check whether DP has an external pull-up resistor, then remove it.
//!
//! SF32LB52-DevKit-LCD-v1.2 and earlier versions incorrectly placed a
//! pull-up resistor on DP, causing the HOST (computer) to attempt
//! enumeration during the bootloader stage. Some hosts stop retrying
//! after four reset attempts and report an unrecognized device, while
//! others keep retrying until enumeration eventually succeeds.
//!
//! This issue is unrelated to the software (SiFli-rs).

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::Spawner;
use embassy_futures::join::join;

use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use sifli_hal::{bind_interrupts, gpio::{Input, Pull}};
use sifli_hal::rcc::{Dll, DllStage, Sysclk, Usbsel};
use sifli_hal::usb::{Driver, InterruptHandler};

bind_interrupts!(struct Irqs {
    USBC => InterruptHandler<sifli_hal::peripherals::USBC>;
});

// you can use `arch-spin` instead of `arch-cortex-m` in embassy-executor's
// feature by setting `entry="cortex_m_rt::entry"`.
// This Will NOT enter Wfi during executor idle.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World! USB HID TEST");

    // Configure 240MHz system clock using DLL1
    // DLL1 Freq = (stg + 1) * 24MHz = (9 + 1) * 24MHz = 240MHz
    // DLL2 for USB at 240MHz, USB = 240MHz / 4 = 60MHz (required by USB PHY)
    let config = sifli_hal::Config::default()
        .with_rcc(const {
            sifli_hal::rcc::ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul10))
                .with_dll2(Dll::new().with_stg(DllStage::Mul10))
                .with_mux(sifli_hal::rcc::ClockMux::new().with_usbsel(Usbsel::Dll2))
                .checked()
        });
    let p = sifli_hal::init(config);

    sifli_hal::rcc::test_print_clocks();

    // Create the driver, from the HAL
    let driver = Driver::new(p.USBC, Irqs, p.PA35, p.PA36);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("SiFli-rs");
    config.product = Some("HID keyboard example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 8,
        hid_subclass: embassy_usb::class::hid::HidSubclass::Boot,
        hid_boot_protocol: embassy_usb::class::hid::HidBootProtocol::Keyboard,
    };

    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    // SF32LB52-DevKit-LCD KEY2 pin, with a pull down resister on the board
    let mut button = Input::new(p.PA11, Pull::None);
    
    // Do stuff with the class!
    let in_fut = async {
        loop {
            button.wait_for_rising_edge().await;
            info!("Button pressed!");
            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };

            button.wait_for_falling_edge().await;

            info!("Button released!");
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, join(in_fut, out_fut)).await;
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
