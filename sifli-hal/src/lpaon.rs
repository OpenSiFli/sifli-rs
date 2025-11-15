//! LPAON (Low-Power Always-On) HAL driver
//! 注意！文档由AI生成！
//!
//! LPAON 模块提供 LPSYS (Low-Power Subsystem) Always-On 外设的硬件抽象，
//! 主要用于配置 LCPU 启动地址。
//!
//! ## 使用示例
//!
//! ```no_run
//! use sifli_hal::lpaon::LpAon;
//!
//! // 配置 LCPU 启动地址（无需参数，自动从固定地址读取）
//! LpAon::configure_lcpu_start();
//!
//! // 读取当前配置的启动向量
//! let (sp, pc) = LpAon::read_start_vector();
//! println!("SP: {:#010x}, PC: {:#010x}", sp, pc);
//! ```
//!
//! ## 功能特性
//!
//! - 配置 LCPU 启动向量（栈指针 SP 和程序计数器 PC）
//! - 电源管理和时钟请求控制（待实现）
//! - AON (Always-On) 引脚配置（待实现）
//! - 低功耗模式唤醒源管理（待实现）
//!
//! ## 硬件背景
//!
//! LPSYS_AON 是 LPSYS 的 Always-On 模块，即使在低功耗模式下也保持运行。
//! 关键寄存器：
//! - `SPR` (0x40040100): Stack Pointer Register - 存储 LCPU 启动时的栈指针
//! - `PCR` (0x40040104): Program Counter Register - 存储 LCPU 启动时的程序计数器
//!
//! 这两个寄存器用于在 LCPU 从复位/低功耗状态唤醒时，配置其初始的 SP 和 PC 值，
//! 符合 ARM Cortex-M 标准启动向量格式。
//!
//! ## LCPU 启动向量表格式
//!
//! ARM Cortex-M 处理器要求启动向量表的前两个条目为：
//!
//! | 地址 | 内容 | 说明 |
//! |------|------|------|
//! | 0x20400000 | 初始栈指针 (SP) | LCPU 启动时的栈顶地址 |
//! | 0x20400004 | Reset_Handler (PC) | LCPU 第一条指令的地址 |
//!
//! `configure_lcpu_start()` 会从固定地址 `0x20400000` 读取这两个值，
//! 并写入 LPAON 寄存器，以便硬件在 LCPU 启动/唤醒时自动加载。
//!
//! ## Safety 注意事项
//!
//! 配置 LCPU 启动地址时需要确保：
//! 1. LCPU 处于停止或复位状态（调用 `HAL_RCC_Reset_and_Halt_LCPU` 后）
//! 2. LCPU 镜像已装载到 `0x20400000`（通过 `lcpu_img::install()` 或 ROM）
//! 3. 启动向量格式符合 ARM Cortex-M 标准：`[SP, Reset_Handler, ...]`
//!
//! ## 参考资料
//!
//! - SDK: `SiFli-SDK/drivers/cmsis/sf32lb52x/lpsys_aon.h` (寄存器定义)
//! - SDK: `SiFli-SDK/drivers/hal/bf0_hal_lpaon.c` (HAL 实现)
//! - SDK: `SiFli-SDK/drivers/cmsis/sf32lb52x/bf0_lcpu_init.c:184` (使用示例)

use crate::{lcpu, pac};

/// LPAON 驱动
///
/// 提供 LPSYS_AON 外设的高层抽象，主要用于配置 LCPU 启动地址。
///
/// 注意：这是一个零大小类型（ZST），仅用作命名空间。
/// LPAON 是 Always-On 模块，无需时钟使能或初始化，可随时访问。
pub struct LpAon;

impl LpAon {
    /// 从 LCPU 代码区固定地址读取启动向量表的前两个条目。
    ///
    /// 返回值依次为 `(sp, pc)`。
    ///
    /// # Safety
    ///
    /// 调用方必须保证：
    /// - LCPU 镜像已经正确装载到 `lcpu::LCPU_CODE_START_ADDR`；
    /// - 启动向量表前两个条目是合法的栈指针和复位入口地址。
    fn read_start_vector_from_mem() -> (u32, u32) {
        let vector_addr = lcpu::LCPU_CODE_START_ADDR as *const u32;

        unsafe {
            // vector[0] (0x20400000): 初始栈指针 (SP)
            // vector[1] (0x20400004): Reset_Handler 地址 (PC)
            let sp = core::ptr::read_volatile(vector_addr);
            let pc = core::ptr::read_volatile(vector_addr.add(1));
            (sp, pc)
        }
    }

    /// 将给定的 `(sp, pc)` 写入 LPSYS_AON 的 SPR/PCR 寄存器。
    fn write_start_vector_to_regs(sp: u32, pc: u32) {
        // LCPU 启动/唤醒时，硬件会自动从这些寄存器加载 SP 和 PC
        pac::LPSYS_AON.spr().write(|w| w.set_sp(sp));
        pac::LPSYS_AON.pcr().write(|w| w.set_pc(pc));
    }

    /// 配置 LCPU 启动地址
    ///
    /// 从固定的 LCPU 代码区起始地址 (`lcpu::LCPU_CODE_START_ADDR` = 0x20400000)
    /// 读取启动向量表的前两个条目（SP, PC），并写入 LPAON 寄存器。
    ///
    /// 这是 LCPU 启动流程的关键步骤（步骤 6/9）。
    ///
    /// # 工作原理
    ///
    /// 1. 从 `0x20400000` 读取初始栈指针 (SP)
    /// 2. 从 `0x20400004` 读取 Reset_Handler 地址 (PC)
    /// 3. 将读取的值写入 `LPSYS_AON->SPR` 和 `LPSYS_AON->PCR`
    /// 4. LCPU 启动/唤醒时，硬件自动从这些寄存器加载 SP 和 PC
    ///
    /// # 调用时机
    ///
    /// 必须在以下步骤之后调用：
    /// - LCPU 已被复位并停表 (`HAL_RCC_Reset_and_Halt_LCPU`)
    /// - LCPU 镜像已装载（A3 需要 `lcpu_img::install()`，Letter Series 使用 ROM）
    ///
    /// # Example
    ///
    /// ```no_run
    /// use sifli_hal::lpaon::LpAon;
    ///
    /// // 配置 LCPU 启动地址
    /// LpAon::configure_lcpu_start();
    /// ```
    ///
    /// # 对应 SDK 函数
    ///
    /// - `HAL_LPAON_ConfigStartAddr()` in `bf0_hal_lpaon.c:436-441`
    /// - 调用位置: `bf0_lcpu_init.c:184`
    pub fn configure_lcpu_start() {
        let (sp, pc) = Self::read_start_vector_from_mem();
        Self::write_start_vector_to_regs(sp, pc);
    }

    /// 读取当前配置的 LCPU 启动向量
    ///
    /// 从 LPAON 寄存器读取当前配置的 SP 和 PC 值。
    ///
    /// # Returns
    ///
    /// `(stack_pointer, program_counter)` - 当前配置的启动向量
    ///
    /// # Example
    ///
    /// ```no_run
    /// use sifli_hal::lpaon::LpAon;
    ///
    /// let (sp, pc) = LpAon::read_start_vector();
    /// println!("Current LCPU start: SP={:#010x}, PC={:#010x}", sp, pc);
    /// ```
    pub fn read_start_vector() -> (u32, u32) {
        let spr = pac::LPSYS_AON.spr().read();
        let pcr = pac::LPSYS_AON.pcr().read();
        (spr.sp(), pcr.pc())
    }

    /// 读取当前 LPSYS_AON PMR 寄存器中的 CPUWAIT 标志。
    ///
    /// 返回 `true` 表示 LCPU 在复位后会被停表。
    #[allow(dead_code)]
    pub(crate) fn cpuwait() -> bool {
        pac::LPSYS_AON.pmr().read().cpuwait()
    }

    /// 设置 LPSYS_AON PMR 寄存器中的 CPUWAIT 标志。
    ///
    /// 当 `enable` 为 `true` 时，LCPU 在复位后会保持停表状态；
    /// 当 `enable` 为 `false` 时，LCPU 复位释放后可以正常运行。
    #[allow(dead_code)]
    pub(crate) fn set_cpuwait(enable: bool) {
        pac::LPSYS_AON.pmr().modify(|w| w.set_cpuwait(enable));
    }

    /// 读取当前 LPSYS_AON SLP_CTRL 寄存器中的 SLEEP_STATUS 标志。
    ///
    /// 返回 `true` 表示 LPSYS 正处于睡眠状态。
    #[allow(dead_code)]
    pub(crate) fn sleep_status() -> bool {
        pac::LPSYS_AON.slp_ctrl().read().sleep_status()
    }

    /// 设置 LPSYS_AON SLP_CTRL 寄存器中的 WKUP_REQ 标志。
    ///
    /// 当 `enable` 为 `true` 时，向 LPSYS 发起一次唤醒请求。
    #[allow(dead_code)]
    pub(crate) fn set_wkup_req(enable: bool) {
        pac::LPSYS_AON
            .slp_ctrl()
            .modify(|w| w.set_wkup_req(enable));
    }
}
