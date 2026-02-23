//! BLE advertising example using the trouble-host BLE stack.
//!
//! Advertises device name "SiFli-BLE", registers Battery Service (0x180F),
//! discoverable by phone BLE scanners (e.g., nRF Connect) with Service Discovery.
//!
//! ## Boot sequence
//!
//! 1. HAL initialization
//! 2. IPC driver + HCI queue creation
//! 3. LCPU BLE startup (firmware load, patch install, warmup event consumption)
//! 4. trouble-host Stack initialization (with GATT services)
//! 5. Continuous advertising, accept connections, handle GATT events, re-advertise on disconnect

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use trouble_host::prelude::*;

use sifli_radio::bluetooth::{BleController, BleInitConfig};
use sifli_hal::rng::Rng;
use sifli_hal::{bind_interrupts, ipc};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

/// GATT service definition: Battery Service (UUID 0x180F)
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
async fn main(_spawner: embassy_executor::Spawner) {
    let p = sifli_hal::init(Default::default());

    Timer::after_secs(1).await;
    info!("=== BLE Advertise Example ===");

    // 1. BLE startup (IPC + LCPU power-on + controller)
    let controller: BleController = match BleController::new(
        p.LCPU,
        p.MAILBOX1_CH1,
        p.DMAC2_CH8,
        Irqs,
        &BleInitConfig::default().pm_enabled(true),
    )
    .await
    {
        Ok(c) => c,
        Err(e) => {
            error!("BLE init failed: {:?}", e);
            cortex_m::asm::bkpt();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    info!("LCPU BLE is ready");

    // 2. Initialize trouble-host Stack
    let address = Address::random([0xC0, 0x12, 0x34, 0x56, 0x78, 0x9A]);
    info!("Address: {:?}", address);

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

    // 3. Create GATT server
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "SiFli-BLE",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // 4. Run runner + advertise/connect loop concurrently
    info!("Starting advertising...");
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
                // Advertise
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        info!("Connected!");
                        // Run GATT event handling + battery notification concurrently
                        let gatt_task = gatt_events_task(&server, &conn);
                        let notify_task = notification_task(&server, &conn);
                        select(gatt_task, notify_task).await;
                    }
                    Err(e) => {
                        error!("Advertise/accept error: {:?}", e);
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
                info!("Re-advertising...");
            }
        },
    )
    .await;
}

/// Advertise and wait for connection.
async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]), // Battery Service
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

    info!("Advertising active, device name: SiFli-BLE");
    let conn = acceptor.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

/// GATT event handling task.
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

/// Battery level notification task: sends notification every 30 seconds.
async fn notification_task(server: &Server<'_>, conn: &GattConnection<'_, '_, DefaultPacketPool>) {
    let level = server.battery_service.level;
    let mut battery = 100u8;
    loop {
        Timer::after(Duration::from_secs(30)).await;
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
