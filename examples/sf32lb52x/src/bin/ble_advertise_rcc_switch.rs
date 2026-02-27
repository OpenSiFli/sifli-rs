//! BLE advertising with dynamic power management via runtime sysclk switching.
//!
//! Demonstrates a practical power-saving pattern:
//! - **Advertising (idle)**: 48 MHz (HXT48) — low power while waiting for connections
//! - **Connected (active)**: 240 MHz (DLL1) — high performance for data exchange
//!
//! Both BLE (IPC/MAILBOX) and USART (`clk_peri`) do not borrow `Pclk`/`Hclk`,
//! so they work across clock switches without dropping.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};
use sifli_hal::bt_hci::BleController;
use sifli_hal::lcpu::LcpuConfig;
use sifli_hal::rcc::{
    clocks, reconfigure_sysclk, Config as RccConfig, ConfigBuilder, Dll, DllStage, HclkPrescaler,
    PclkPrescaler, Sysclk,
};
use sifli_hal::rng::Rng;
use sifli_hal::usart::{self, Config as UsartConfig, Uart};
use sifli_hal::{bind_interrupts, ipc, peripherals};
use trouble_host::prelude::*;

use defmt_rtt as _;
use embedded_io_async::Write as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

/// 48 MHz — low power mode for advertising / idle
const LP_MODE: RccConfig = const {
    ConfigBuilder::sysclk()
        .with_sys(Sysclk::Hxt48)
        .with_hdiv(HclkPrescaler(1))
        .with_pdiv1(PclkPrescaler::Div1)
        .with_pdiv2(PclkPrescaler::Div16)
        .checked()
};

/// 240 MHz — high performance mode for active connections
const HP_MODE: RccConfig = const {
    ConfigBuilder::sysclk()
        .with_sys(Sysclk::Dll1)
        .with_dll1(Dll::new().with_stg(DllStage::Mul10))
        .with_hdiv(HclkPrescaler(1))
        .with_pdiv1(PclkPrescaler::Div2)
        .with_pdiv2(PclkPrescaler::Div64)
        .checked()
};

#[gatt_server]
struct Server {
    battery_service: BatteryService,
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 100)]
    level: u8,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut uart_config = UsartConfig::default();
    uart_config.baudrate = 1_000_000;
    let mut usart = Uart::new(
        p.USART1,
        p.PA18,
        p.PA19,
        Irqs,
        p.DMAC1_CH1,
        p.DMAC1_CH2,
        uart_config,
    )
    .unwrap();

    Timer::after_millis(1000).await;
    info!("=== rcc_switch example ===");
    print_clocks("boot");

    // --- BLE initialization at default 240 MHz ---
    let controller: BleController = match BleController::new(
        p.LCPU,
        p.MAILBOX1_CH1,
        p.DMAC2_CH8,
        Irqs,
        &LcpuConfig::default(),
    )
    .await
    {
        Ok(c) => {
            info!("BLE initialized");
            c
        }
        Err(e) => {
            error!("BLE init failed: {:?}", e);
            cortex_m::asm::bkpt();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    let address = Address::random([0xC0, 0x12, 0x34, 0x56, 0x78, 0x9A]);
    info!("BLE address: {:?}", address);

    let mut rng = Rng::new_blocking(p.TRNG);
    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut rng);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "SiFli-BLE",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // --- Switch to LP before entering advertising loop ---
    info!("Entering low-power advertising mode (48 MHz)");
    reconfigure_sysclk(LP_MODE);
    print_clocks("LP mode");

    let _ = join(
        async {
            loop {
                if let Err(e) = runner.run().await {
                    error!("[ble] runner error: {:?}", e);
                }
            }
        },
        async {
            loop {
                // Advertise at 48 MHz (low power)
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        // Connected — switch to 240 MHz for performance
                        info!("Connected! Switching to 240 MHz");
                        reconfigure_sysclk(HP_MODE);
                        print_clocks("HP mode");
                        unwrap!(usart.write_all(b"[240MHz] BLE connected\r\n").await);

                        let gatt_task = gatt_events_task(&server, &conn);
                        let notify_task = notification_task(&server, &conn);
                        select(gatt_task, notify_task).await;

                        // Disconnected — switch back to 48 MHz
                        info!("Disconnected, back to 48 MHz");
                        reconfigure_sysclk(LP_MODE);
                        print_clocks("LP mode");
                        unwrap!(usart.write_all(b"[48MHz] BLE disconnected\r\n").await);
                    }
                    Err(e) => {
                        error!("Advertise/accept error: {:?}", e);
                        Timer::after_secs(1).await;
                    }
                }
            }
        },
    )
    .await;
}

async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(b"SiFli-BLE"),
        ],
        &mut adv_data[..],
    )
    .unwrap();

    let params = AdvertisementParameters {
        interval_min: Duration::from_millis(100),
        interval_max: Duration::from_millis(100),
        ..Default::default()
    };

    let acceptor = peripheral
        .advertise(
            &params,
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("[48MHz] Advertising as SiFli-BLE...");
    let conn = acceptor.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

async fn gatt_events_task(server: &Server<'_>, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
    let level = server.battery_service.level;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(e) => {
                        if e.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[gatt] Read battery level: {:?}", value);
                        }
                    }
                    GattEvent::Write(e) => {
                        info!("[gatt] Write handle={}", e.handle());
                    }
                    _ => {}
                }
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error: {:?}", e),
                }
            }
            _ => {}
        }
    }
}

async fn notification_task(server: &Server<'_>, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
    let level = server.battery_service.level;
    let mut battery = 100u8;
    loop {
        Timer::after_secs(30).await;
        battery = battery.saturating_sub(1).max(20);
        if server.set(&level, &battery).is_ok() {
            info!("[notify] Battery level: {}", battery);
            if let Err(e) = level.notify(conn, &battery).await {
                warn!("[notify] error: {:?}", e);
                break;
            }
        }
    }
}

fn print_clocks(label: &str) {
    let c = clocks();
    let mhz = |f: sifli_hal::time::MaybeHertz| f.to_hertz().map(|h| h.0 / 1_000_000).unwrap_or(0);
    info!(
        "[clk] {} | sysclk {}MHz | hclk {}MHz | pclk {}MHz",
        label,
        mhz(c.sysclk),
        mhz(c.hclk),
        mhz(c.pclk),
    );
}
