#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use sifli_hal::lcdc::SpiConfig;
use sifli_hal::time::mhz;
use static_cell::StaticCell;

use sifli_hal::bind_interrupts;
use sifli_hal::rcc::{ConfigBuilder, Dll, DllStage, Sysclk};
use sifli_hal::{gpio, lcdc};

use embedded_graphics::{
    framebuffer::{buffer_size, Framebuffer},
    image::{Image, ImageRaw},
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::{
        raw::{BigEndian, RawU16},
        Rgb565,
    },
    prelude::*,
    text::Text,
};

use display_driver::bus::QspiFlashBus;
use display_driver::panel::reset::LCDResetOption;
use display_driver::{ColorFormat, DisplayDriver};
use display_driver_co5300::{
    spec::{Co5300Spec, PanelSpec},
    Co5300,
};

// Note: Although the hardware features a 390x450 display (the standard SiFli bundle), this example
// renders only a partial area to minimize SRAM usage and maintain code simplicity.
//
// To drive the full resolution, it is recommended to utilize PSRAM coupled with
// partial/block-based rendering.
//
// Additionally, due to pixel alignment requirements, the CO5300 controller mandates even-numbered
// window dimensions (i.e., the refresh area must be a multiple of 2 in both rows and columns).
const WIDTH: usize = 240;
const HEIGHT: usize = 240;

pub struct MyCo5300;
impl PanelSpec for MyCo5300 {
    const PHYSICAL_WIDTH: u16 = WIDTH as u16;
    const PHYSICAL_HEIGHT: u16 = HEIGHT as u16;

    const PHYSICAL_X_OFFSET: u16 = 0;
    const PHYSICAL_Y_OFFSET: u16 = 0;
}

impl Co5300Spec for MyCo5300 {
    const INIT_PAGE_PARAM: u8 = 0x20;
    const IGNORE_ID_CHECK: bool = false;
}

// Framebuffer configuration
// Using BigEndian for direct compatibility with the display controller's byte order
type FramebufferType =
    Framebuffer<Rgb565, RawU16, BigEndian, WIDTH, HEIGHT, { buffer_size::<Rgb565>(WIDTH, HEIGHT) }>;

static FB: StaticCell<FramebufferType> = StaticCell::new();

const IMAGE_WIDTH: u32 = 86;
const IMAGE_HEIGHT: u32 = 64;

bind_interrupts!(
    struct Irqs {
        LCDC1 => sifli_hal::lcdc::InterruptHandler<sifli_hal::peripherals::LCDC1>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Init SF32LB52 @ 240MHz...");

    // 1. Hardware Initialization
    // 240MHz: DLL1 Freq = (stg + 1) * 24MHz -> Mul10 * 24 = 240
    let config = sifli_hal::Config::default().with_rcc(
        const {
            ConfigBuilder::new()
                .with_sys(Sysclk::Dll1)
                .with_dll1(Dll::new().with_stg(DllStage::Mul10))
                .checked()
        },
    );

    let p = sifli_hal::init(config);

    // 2. LCDC Configuration
    let config = sifli_hal::lcdc::Config {
        width: WIDTH as u16,
        height: HEIGHT as u16,
        interface_config: SpiConfig {
            line_mode: lcdc::SpiLineMode::FourLine4Data,
            write_frequency: lcdc::FrequencyConfig::Freq(mhz(50)),
            ..Default::default()
        },
        ..Default::default()
    };

    let lcdc = lcdc::Lcdc::new_qspi(
        p.LCDC1, Irqs, p.PA2, p.PA3, p.PA4, p.PA5, p.PA6, p.PA7, p.PA8, config,
    );

    // Wrap the raw bus in the QspiFlashBus protocol layer (handles 0x02/0x32 prefixes)
    let disp_bus = QspiFlashBus::new(lcdc);

    let rst = gpio::Output::new(p.PA0, gpio::Level::Low);
    let mut bl = gpio::Output::new(p.PA1, gpio::Level::Low);

    // Initialize the CO5300 panel driver
    let panel = Co5300::<MyCo5300, _, _>::new(LCDResetOption::new_pin(rst));

    info!("Initializing Display...");
    let mut display = DisplayDriver::builder(disp_bus, panel)
        .with_color_format(ColorFormat::RGB565)
        .init(&mut Delay)
        .await
        .unwrap();

    display.set_brightness(200).await.unwrap();

    // Enable backlight
    bl.set_low();

    // 4. Graphics Setup
    let fb = FB.init(Framebuffer::new());
    fb.clear(Rgb565::WHITE).unwrap();

    // Load and draw image
    let image_raw: ImageRaw<Rgb565, BigEndian> =
        ImageRaw::new(include_bytes!("../../assets/ferris.raw"), IMAGE_WIDTH);

    let image = Image::new(
        &image_raw,
        Point::new(
            ((WIDTH as i32) - (IMAGE_WIDTH as i32)) / 2,
            ((HEIGHT as i32) - (IMAGE_HEIGHT as i32)) / 2,
        ),
    );
    image.draw(fb).unwrap();

    // Draw text
    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLACK);
    Text::new("Hello SF32!", Point::new(60, 50), style)
        .draw(fb)
        .unwrap();

    Text::new("Powerd by SiFli-rs", Point::new(20, 180), style)
        .draw(fb)
        .unwrap();

    info!("Starting render loop...");

    // 5. Render Loop
    loop {
        info!("Sending Frame ...");
        // Update the display with the new frame
        display.write_frame(fb.data()).await.unwrap();

        info!("Frame Finished");
        Timer::after_secs(3).await;
    }
}
