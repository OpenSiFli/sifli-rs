# SiFli HAL

[![Crates.io][badge-license]][crates]
[![Crates.io][badge-version]][crates]
[![docs.rs][badge-docsrs]][docsrs]
[![Support status][badge-support-status]][githubrepo]

[badge-license]: https://img.shields.io/crates/l/sifli-hal?style=for-the-badge
[badge-version]: https://img.shields.io/crates/v/sifli-hal?style=for-the-badge
[badge-docsrs]: https://img.shields.io/docsrs/sifli-hal?style=for-the-badge
[badge-support-status]: https://img.shields.io/badge/Support_status-Community-yellow?style=for-the-badge
[crates]: https://crates.io/crates/sifli-hal
[docsrs]: https://docs.rs/sifli-hal
[githubrepo]: https://github.com/OpenSiFli/sifli-hal

[English](README.md) | 中文

SiFli MCU的Rust硬件抽象层(HAL)和[Embassy](https://github.com/embassy-rs/embassy)驱动。

> [!WARNING]
> 
> 此project仍在开发中，尚未准备好用于生产环境。

## 快速开始！

[嵌入式Rust介绍](../docs/intro_to_embedded_rust.md)

[入门指南](../docs/get_started.md)

[例程](examples)

[烧录与调试指南](../docs/flash_and_debug.md)

## 当前状态

<details open>
<summary><strong>HAL 实现状态 (点击展开/折叠)</strong></summary>
<div>
  <ul>
    <li>✅: 支持 & 已测试</li>
    <li>🌗: 部分支持 & 已测试</li>
    <li>❓: 已编写, 需要示例/测试</li>
    <li>📝: 计划中 & 开发中</li>
    <li>❌: 硬件不支持 (N/A)</li>
    <li>➕: Async异步</li>
  </ul>
</div>
<table style="border-collapse: collapse; width: 80%;font-size: small;padding: 4px 8px;">
    <thead>
        <tr>
            <th rowspan="2" style="la">Peripheral</th>
            <th rowspan="2">Feature</th>
            <th colspan="1">sf32lb52x</th>
            <th rowspan="2">56x</th>
            <th rowspan="2">58x</th>
        </tr>
        <tr>
            <th>hcpu</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td colspan="2"><strong>PAC (Peripheral Access Crate)</strong></td>
            <td>🌗</td><td></td><td></td>
        </tr>
        <tr>
            <td colspan="2"><strong>Startup & Interrupt</strong></td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td colspan="2"><strong>Flash Table</strong></td>
            <td>🌗</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong><a href="https://github.com/embassy-rs/embassy">embassy</a></strong></td>
            <td>GPTIM Time Driver</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>ATIM Time Driver</td>
            <td><a href="https://github.com/OpenSiFli/sifli-rs/issues/5">(#5)</a></td><td></td><td></td>
        </tr>
        <tr>
            <td>Embassy Peripheral Singleton</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>RCC</strong></td>
            <td>Peripheral RCC Codegen (enable, freq...)</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Read current RCC tree</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>RCC tree Configure</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="4"><strong>GPIO</strong></td>
            <td>Blinky</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>PinMux Codegen & AF Config</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>IO Mode & AonPE Config</td>
            <td>🌗</td><td></td><td></td>
        </tr>
        <tr>
            <td>EXTI ➕</td><td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="2"><strong>Timer</strong></td>
            <td>Simple PWM</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Complementary PWM, Input etc.</td>
            <td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="4"><strong>USART</strong></td>
            <td>Blocking</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Buffered(Interrupt) ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>DMA ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>RingBuffered(DMA) ➕</td>
            <td>❓</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>I2C</strong></td>
            <td>Blocking</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Interrupt ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>DMA ➕</td>
            <td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>DMA</strong></td>
            <td>Transfer(P2M, M2P, M2M)</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>RingBuffer</td>
            <td>❓</td><td></td><td></td>
        </tr>
        <tr>
            <td>ExtDMA</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>Bluetooth</strong></td>
            <td>RF Calibration</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>BLE (<a href="https://github.com/embassy-rs/bt-hci">bt-hci</a>, <a href="https://github.com/embassy-rs/trouble">trouble</a>) ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Classic Bluetooth</td>
            <td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>USB<br>(see also:<a href="https://github.com/decaday/musb">musb</a>)</strong></td>
            <td><a href="https://crates.io/crates/embassy-usb">embassy-usb</a> ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Device: HID, CDC_ACM ...</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>HOST / OTG</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="8"><strong>GPADC</strong></td>
            <td>Blocking</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Interrupt ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Timer Trigger</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>VBAT & External Channel</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Multi Channel & Slot</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>Differential Input</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>DMA ➕</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>Calibration</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="4"><strong>LCDC</strong></td>
            <td>Command Path / Data Path ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>SPI QSPI Interface</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>8080, RGB, JDI, MIPIDSI Interface</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>Blender, Fill, Canvas</td>
            <td></td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="4"><strong>PMU</strong></td>
            <td>DVFS Switch</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Efuse</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Charge Module</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>Buck & LDO</td>
            <td>🌗</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="3"><strong>LCPU Control/<br>IPC</strong></td>
            <td>LCPU Power On, Wakeup, Patch, RCC, Control</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>Mailbox ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>IPCQueue, HCI ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td><strong>TRNG</strong></td>
            <td>Hardware Random Number Generator ➕</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td rowspan="4"><strong>Audio</strong></td>
            <td>AudCodec/ADC, DAC</td>
            <td>✅</td><td></td><td></td>
        </tr>
        <tr>
            <td>AudPrc/Channel, Mixer, Volume</td>
            <td>🌗</td><td></td><td></td>
        </tr>
        <tr>
            <td>I2S/DMA, Master, Slave</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>PDM</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td colspan="2"><strong>SPI</strong></td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td colspan="2"><strong>ePicasso</strong></td><td></td><td></td><td></td>
        </tr>
    </tbody>
</table>
</details>


## 例程

例程在[这里](../examples)。

一个简单的SF32LB52x+slint+lcdc qspi+co5300 AMOLED示例在[这里](https://github.com/decaday/sf32-slint-example)。

## Features

- `defmt`, `log`: 调试日志输出。

- `sf32lb52x`: 目标芯片选择。目前仅支持`sf32lb52x`。

- `set-msplim`: 在`__pre_init`中设置MSPLIM寄存器。此寄存器必须在主函数的栈设置前配置（因为引导加载程序可能已将其配置为不同的值），否则将导致Hard Fault [SiFli-SDK #32](https://github.com/OpenSiFli/SiFli-SDK/issues/32)。

  该feature将在[cortex-m-rt #580](https://github.com/rust-embedded/cortex-m/pull/580)发布后移除。

- `time-driver-xxx`: 为`time-driver`配置定时器。它至少需要两个捕获/比较通道。对于`sf32lb52x hcpu`，只有`gptim1`和`gptim2`可用。`atim1`存在问题：[#5](https://github.com/OpenSiFli/sifli-rs/issues/5)。

- `unchecked-overclocking`: 启用此feature以禁用超频检查。除非你知道自己在做什么，否则不要启用此feature!

- `sf32lb52x-lcpu`: 选择 SF32LB52x LCPU。

- `edr-cal`: 启用 EDR LO 3GHz 校准 (VCO3G + OSLO)。仅 BR/EDR (经典蓝牙) 需要。纯 BLE 应用可以跳过此项以加快启动速度。

- `bt-hci`: 启用用于 IPC HCI 通信的 bt-hci 传输层。

## 许可证

本项目采用 Apache 2.0许可证（[LICENSE](../LICENSE) 或 <http://www.apache.org/licenses/LICENSE-2.0>）。