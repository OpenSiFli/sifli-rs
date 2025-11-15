#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embedded_io_async::Write as _;
use panic_probe as _;
use sifli_hal::usart::{Config, Uart};
use sifli_hal::{bind_interrupts, peripherals, usart};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// 基于 DMA + 环形缓冲（ring buffer）的 USART 接收示例。
//
// 原理：
// - TX 仍然通过 DMA 异步发送；
// - RX 使用 DMA 循环写入一块静态缓冲区（ring buffer），驱动通过空闲(IDLE)或 DMA 半/满事件唤醒；
// - 应用通过 `read(&mut buf)` 读取 ring buffer 中“已到的数据”，一次可读若干字节；
//
// 注意：
// - 使用 defmt 日志会短暂关中断（critical section），尽量避免在高吞吐路径频繁打印，以免影响时序。

// DMA 接收环形缓冲区建议为 'static。使用 StaticCell 安全获取 &'static mut，避免 unsafe。
// 尺寸按需调整：越大越不容易溢出，但占用 RAM 越多。
static RX_DMA_RING: StaticCell<[u8; 1024]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut config = Config::default();
    // DMA + RingBuffer 更适合高波特率，这里示例使用 1Mbps。
    config.baudrate = 1_000_000;

    // Uart::new 使用异步 + DMA 模式（需要绑定 USART 中断 + TX/RX DMA 通道）。
    let uart = Uart::new(
        p.USART1,
        // 注意顺序：先 RX 引脚，再 TX 引脚。
        p.PA18,
        p.PA19,
        Irqs,
        // 先 TX DMA 通道，后 RX DMA 通道。
        p.DMAC1_CH1,
        p.DMAC1_CH2,
        config,
    )
    .unwrap();

    // 拆分为 TX/RX 两半，RX 转为环形缓冲接收。
    let (mut tx, rx) = uart.split();
    let ring = RX_DMA_RING.init([0u8; 1024]);
    let mut rx = rx.into_ring_buffered(&mut ring[..]);

    // 简单提示。
    unwrap!(
        tx.write_all(b"USART ringbuffer ready. Type and I will echo.\r\n")
            .await
    );

    // 小块读取缓冲，按需调整大小。越小则反馈更及时，越大则系统调用更少。
    let mut chunk = [0u8; 64];

    loop {
        // 读取 ring buffer 中“当前可用”的字节；若无数据，会等待 IDLE 或 DMA 半/满事件后返回。
        let n = rx.read(&mut chunk).await.unwrap();

        // Echo 回去（或在此做协议解析）。
        unwrap!(tx.write_all(&chunk[..n]).await);
    }
}
