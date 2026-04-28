---
name: sifli-debug
description: 调试 SiFli SF32LB52x 嵌入式程序（Rust 和 C SDK）。支持 Rust 项目（probe-rs/defmt）和 C SDK 项目（scons/RT-Thread）的完整调试流程：编译 → 烧录 → 日志抓取 → 分析。烧录前自动提醒按 reset 按键。当用户要调试 SiFli 项目、查看日志、分析寄存器或排查嵌入式问题时应使用此 skill。

---

# SiFli Debug Skill

调试 SiFli SF32LB52x 嵌入式程序的专用工具流程，支持 Rust 和 C SDK 两种项目。

## 项目路径

- **Rust 示例程序**: `/Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x`
- **Rust HAL 库**: `/Users/haobogu/Projects/rust/sifli-rs/sifli-hal`
- **Nix 环境**: `/Users/haobogu/Projects/rust/sifli-rs/contrib/nix`
- **C SDK**: `/Users/haobogu/Projects/embedded/SiFli-SDK`
- **串口设备**: `/dev/cu.wchusbserial5ABA0714731`

## 项目类型判断

根据用户提供的路径或描述判断项目类型：

- 路径包含 `sifli-rs` 或使用 `cargo` → **Rust 项目**，走 Rust 调试流程
- 路径包含 `SiFli-SDK` 或使用 `scons` → **C SDK 项目**，走 C SDK 调试流程

---

# 一、Rust 项目调试流程

**前提**：用户已在 sifli 目录下激活 direnv 环境后打开终端，无需再检查或激活 direnv。

## 阶段 1: 编译检查（仅 debug）

```bash
# 检查 HAL 编译
cd /Users/haobogu/Projects/rust/sifli-rs/sifli-hal && cargo check

# 检查示例编译 (debug 模式)
cd /Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x && cargo build --bin <EXAMPLE_NAME>
```

编译通过后才能进入下一阶段。

## 阶段 2: Debug 和 Release 模式日志观察（必须两者都执行）

**重要**：必须同时测试 debug 和 release 两种模式，先 debug 后 release。

**烧录前提醒规则（必须遵守）**：
每次执行烧录命令前，必须先在回复文本中输出以下提醒，然后再调用 Bash 工具：

```
⚠️ 即将烧录 <EXAMPLE_NAME> (<MODE>) - 请按住 RESET 按键，授权后松开
```

### 2.1 Debug 模式

```bash
cd /Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x && timeout 10 cargo run --bin <EXAMPLE_NAME> 2>&1 || true
```

### 2.2 Release 模式

```bash
cd /Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x && timeout 10 cargo run --release --bin <EXAMPLE_NAME> 2>&1 || true
```

timeout 默认 10 秒，不够再加。**只有 debug 和 release 两种模式都测试通过，才能认为调试完成。**

## 阶段 3: 寄存器分析

### 方法 A: 插入调试代码 dump 寄存器

```rust
use sifli_hal::pac;

let rcc = unsafe { &*pac::HPSYS_RCC::ptr() };
defmt::info!("RCC ENR1: {:#010x}", rcc.enr1().read().bits());
```

### 方法 B: probe-rs 直接读取寄存器

```bash
cd /Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x && SIFLI_UART_DEBUG=1 probe-rs attach --chip SF32LB52 <ELF_PATH>
```

## Rust 环境变量

当前配置（在 `.cargo/config.toml` 中）：

| 变量                | 值                                | 说明                     |
| ------------------- | --------------------------------- | ------------------------ |
| `DEFMT_LOG`         | `debug`                           | defmt 日志级别           |
| `SIFLI_UART_DEBUG`  | `1`                               | 启用 SiFli UART 调试接口 |
| `SIFLI_SFTOOL_PORT` | `/dev/cu.wchusbserial5ABA0714731` | 串口路径                 |

## Rust 可用示例程序

```
mpi_nor_flash.rs adc_async.rs        adc_blocking.rs     blinky.rs
button_async.rs     lcpu_bt_hci.rs      lcpu_ipc_hci_reset.rs
lcpu_pwron.rs       raw_rtt.rs          rcc_240mhz.rs
usart_blocking.rs   usart_buffered.rs   usart_dma.rs
usb_hid_keyboard.rs usb_serial.rs       ws2812_dma.rs
```

---

# 二、C SDK 项目调试流程

## 阶段 0: 环境配置

C SDK 使用自己的 `export.sh` 配置环境（**不是** nix/direnv）：

```bash
source /Users/haobogu/Projects/embedded/SiFli-SDK/export.sh
```

sftool 路径：`~/.sifli/tools/sftool/0.1.16/sftool`

## 阶段 1: 编译

在示例的 `project/` 目录下执行 scons 编译：

```bash
source /Users/haobogu/Projects/embedded/SiFli-SDK/export.sh 2>&1 > /dev/null && \
cd <EXAMPLE_PATH>/project && \
scons --board=sf32lb52-lcd_n16r8 -j8 2>&1
```

编译产物生成在 `project/build_sf32lb52-lcd_n16r8_hcpu/` 目录下，包含：

- `ftab/ftab.bin` → 0x12000000（Flash 分区表）
- `bootloader/bootloader.bin` → 0x12010000（引导程序）
- `main.bin` → 0x12020000（应用程序）
- `uart_download.sh`（烧录脚本）

## 阶段 2: 烧录

**烧录前必须用 AskUserQuestion 向用户请求执行权限**，提示用户按住 RESET 按键。

获得授权后，使用编译生成的 `uart_download.sh` 脚本烧录（比手动调用 sftool 更可靠）：

```bash
source /Users/haobogu/Projects/embedded/SiFli-SDK/export.sh 2>&1 > /dev/null && \
cd <EXAMPLE_PATH>/project/build_sf32lb52-lcd_n16r8_hcpu && \
sftool -p /dev/cu.wchusbserial5ABA0714731 -c SF32LB52 -m nor write_flash \
  "bootloader/bootloader.bin@0x12010000" \
  "main.bin@0x12020000" \
  "ftab/ftab.bin@0x12000000" 2>&1
```

> 注意：sftool 的进度条在 Claude 非交互 Bash 中不显示，看起来像卡住但实际在正常工作。

## 阶段 3: 串口日志抓取

C SDK 使用 RT-Thread 的串口输出，波特率由 `rtconfig.h` 中 `RT_SERIAL_DEFAULT_BAUDRATE` 决定（通常为 **1000000**）。

**minicom 可用**（Claude Bash 中）：

```bash
timeout 80 minicom -b 1000000 -D /dev/cu.wchusbserial5ABA0714731 -o -w -C /tmp/<name>.log 2>&1 || true
```

参数说明：

- `-b 1000000`：波特率 1M
- `-D`：串口设备
- `-o`：跳过初始化（不发送 modem 初始化字符串）
- `-w`：自动换行
- `-C /tmp/<name>.log`：捕获输出到文件（关键！Claude 可读取分析）

minicom 启动后让用户按 RESET 使设备重启，即可抓取完整启动日志。

**tio 在非交互模式下不接收数据，不可用。**

抓取完成后读取日志文件分析：

```bash
cat /tmp/<name>.log | head -300
```

## 阶段 4: 日志分析

C SDK 日志格式：`[时间戳] 级别/模块 上下文: 消息`

关注点：

- `W/` 警告和 `E/` 错误信息
- `[LCPU_INIT]` LCPU 初始化流程是否完整
- `BLE ready!` BLE 栈是否成功启动
- `ADV start resutl 0` 广播是否成功（result 0 = 成功）
- `Patch record match: NO/YES` BLE ROM patch 状态

## C SDK 常用示例路径

```
SiFli-SDK/example/ble/peripheral/     # BLE 外设
SiFli-SDK/example/ble/central/         # BLE 中心
SiFli-SDK/example/ble/mesh/            # BLE Mesh
SiFli-SDK/example/bt/                  # 经典蓝牙
SiFli-SDK/example/multimedia/          # 多媒体
```

---

# 三、通用问题排查

## 无法连接探针/串口

1. 检查串口路径：`ls /dev/cu.*usb*`
2. 检查串口是否被占用：`lsof /dev/cu.wchusbserial5ABA0714731`
3. 尝试重新插拔 USB 线

## 调试不稳定（Rust）

Embassy 在空闲时使用 WFI 指令可能导致调试中断：

1. 在 `Cargo.toml` 中将 `embassy-executor` 的 feature 从 `arch-cortex-m` 改为 `arch-spin`
2. 将 `#[embassy_executor::main]` 改为 `#[embassy_executor::main(entry = "cortex_m_rt::entry")]`

## HardFault（Rust）

1. 检查是否启用了 `set-msplim` feature
2. 使用 debug 模式获取更多栈回溯信息
3. 检查 panic handler 输出

## 工具链

- **Rust**: 用户已通过 direnv 预先配置好环境 (cargo, probe-rs, sftool, defmt-print)
- **C SDK**: 通过 `source export.sh` 配置 (arm-none-eabi-gcc, scons, sftool)