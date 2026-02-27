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

[中文](README_zh.md) | English

Rust Hardware Abstraction Layer (HAL) and [Embassy](https://github.com/embassy-rs/embassy) driver for SiFli MCUs.

> [!WARNING]
> 
> This project is working-in-progress and not ready for production use.

## Let's GO!

[Introduction to Embedded Rust](../docs/intro_to_embedded_rust.md)

[Get Started](../docs/get_started.md)

[examples](examples)

[Flash and Debug Guide](../docs/flash_and_debug.md)

## Status

<details open>
<summary><strong>HAL Implementation Status (Click to expand/collapse)</strong></summary>
<div>
  <ul>
    <li>✅: Supported & Tested</li>
    <li>🌗: Partially Supported & Tested</li>
    <li>❓: Written, needs example/test</li>
    <li>📝: Planned & WIP</li>
    <li>❌: Not supported by Hardware (N/A)</li>
    <li>➕: Async Feature</li>
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
            <td>📝</td><td></td><td></td>
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
            <td>📝</td><td></td><td></td>
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
            <td>AudCodec/ADC, DAC</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>AudPrc/Channel, Mixer, Volume</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>I2S/DMA, Master, Slave</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td>PDM</td><td></td><td></td><td></td>
        </tr>
        <tr>
            <td colspan="2"><strong>I2C</strong></td><td></td><td></td><td></td>
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

## Examples

Examples can be found [here](../examples).

A simple SF32LB52x+slint+lcdc qspi+co5300 AMOLED example can be found [here](https://github.com/decaday/sf32-slint-example).

## Features



- `defmt`, `log`: Debug log output.

- `sf32lb52x`: Target chip selection. Currently, only `sf32lb52x` is supported.

- `usb`: USB support.

- `set-msplim`: Set the MSPLIM register in `__pre_init`. This register must be set before the main function’s stack setup (since the bootloader may have already configured it to a different value), otherwise, it will cause a HardFault [SiFli-SDK #32](https://github.com/OpenSiFli/SiFli-SDK/issues/32).

  This feature will be removed after [cortex-m-rt #580](https://github.com/rust-embedded/cortex-m/pull/580)  is released.

- `time-driver-xxx`: Timer configuration for `time-driver`. It requires at least two capture/compare channels. For the `sf32lb52x hcpu`, only `gptim1` and `gptim2` are available. `atim1` has an issue: [#5](https://github.com/OpenSiFli/sifli-rs/issues/5).

- `unchecked-overclocking`: Enable this feature to disable the overclocking check. DO NOT ENABLE THIS FEATURE UNLESS YOU KNOW WHAT YOU'RE DOING.

- `sf32lb52x-lcpu`: Select SF32LB52x LCPU.

- `edr-cal`: Enable EDR LO 3GHz calibration (VCO3G + OSLO). Only needed for BR/EDR (classic Bluetooth). BLE-only applications can skip this for faster boot.

- `bt-hci`: Enable bt-hci transport layer for IPC HCI communication.

## License

This project is under Apache License, Version 2.0 ([LICENSE](../LICENSE) or <http://www.apache.org/licenses/LICENSE-2.0>).