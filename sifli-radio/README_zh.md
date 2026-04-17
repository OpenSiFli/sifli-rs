# SiFli Radio

SiFli SF32LB52x MCU 的蓝牙射频栈，基于 [sifli-hal](../sifli-hal) 和 [Embassy](https://github.com/embassy-rs/embassy) 构建。

实现了 LCPU（蓝牙协处理器）管理，并提供 [bt-hci](https://github.com/embassy-rs/bt-hci) controller 接口，可配合 [trouble](https://github.com/embassy-rs/trouble) BLE 协议栈使用。

## 功能概览

- LCPU 固件加载、补丁安装和启动流程（A3 + Letter Series）
- ROM 配置块（基于 TOML 布局定义的代码生成）
- RF 校准（VCO、TXDC、opt_cal）
- 基于 IPC 邮箱的 BLE HCI 传输层
- BLE 休眠与电源管理

## 使用

在 `Cargo.toml` 中添加：

```toml
sifli-radio = { path = "../sifli-radio", features = ["sf32lb52x-lcpu"] }
```

完整的 BLE 广播示例见 [ble_advertise](../examples/sf32lb52x/src/bin/ble_advertise.rs)。

## 特性开关

- `sf32lb52x-lcpu` — LCPU 支持（固件、补丁、RF 校准、HCI 传输）
- `edr` — EDR LO 3GHz 校准（VCO3G + OSLO），仅经典蓝牙需要
- `defmt` / `log` — 调试日志输出

## SDK 参考

本 crate 的功能对应 SiFli C SDK 中的：

- [bf0_lcpu_init.c](https://github.com/OpenSiFli/SiFli-SDK/blob/main/drivers/cmsis/sf32lb52x/bf0_lcpu_init.c) — LCPU 启动流程
- [bt_rf_fulcal.c](https://github.com/OpenSiFli/SiFli-SDK/blob/main/middleware/bluetooth/service/ble/bt_rf_fulcal/bt_rf_fulcal.c) — RF 校准
- [rom_config.h](https://github.com/OpenSiFli/SiFli-SDK/blob/main/middleware/bluetooth/include/rom_config.h) — ROM 配置 API

## 许可证

Apache License, Version 2.0 ([LICENSE](../LICENSE) or <http://www.apache.org/licenses/LICENSE-2.0>)。
