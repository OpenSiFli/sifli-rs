#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::lcpu::{self, LcpuConfig, PatchData};
use sifli_hal::syscfg;

#[path = "../lcpu_image_52x.rs"]
mod lcpu_image_52x;
#[path = "../patch_data.rs"]
mod patch_a3;
#[path = "../patch_data_rev_b.rs"]
mod patch_ls;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    // 读取芯片版本
    let idr = syscfg::read_idr();
    let revision = idr.revision();

    info!("芯片版本: {:?} (REVID: 0x{:02x})", revision, idr.revid);

    // 构建 LCPU 配置
    let config = LcpuConfig::new()
        .with_firmware(&lcpu_image_52x::G_LCPU_BIN_U32)
        .with_patch_a3(PatchData {
            record: &patch_a3::PATCH_RECORD_U32,
            code: &patch_a3::PATCH_CODE_U32,
        })
        .with_patch_letter(PatchData {
            record: &patch_ls::PATCH_RECORD_U32,
            code: &patch_ls::PATCH_CODE_U32,
        })
        // .skip_frequency_check()
        .disable_rf_cal();              // RF 校准尚未实现

    lcpu::power_on(&config).unwrap();

    use sifli_hal::gpio;
    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);

    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}
