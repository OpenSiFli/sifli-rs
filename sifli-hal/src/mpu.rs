//! MPU 配置（HCPU）。
//!
//! 目前最关键的目标：让跨核共享 SRAM（HPSYS/LPSYS）为 non-cacheable，
//! 以匹配 SiFli SDK `mpu_config()` 的行为，避免 IPC ring buffer 因 DCache 产生不一致。

use cortex_m::asm;

/// SiFli SDK `mpu_armv8.h`: `ARM_MPU_ATTR_NON_CACHEABLE=4`, `ARM_MPU_ATTR(O,I)=(O<<4)|I`（当 O!=0）。
const ATTR_NON_CACHEABLE_NORMAL: u32 = 0x44;

// `core_cm33.h`
const MPU_CTRL_ENABLE: u32 = 1 << 0;
const MPU_CTRL_HFNMIENA: u32 = 1 << 1;
const MPU_CTRL_PRIVDEFENA: u32 = 1 << 2;

const MPU_RBAR_SH_NON: u32 = 0;
const MPU_RBAR_AP_RW_ANY: u32 = 0b01; // ARM_MPU_AP_(RO=0, NP=1)

#[inline]
const fn mpu_rbar(base: u32, sh: u32, ap: u32, xn: bool) -> u32 {
    let xn = if xn { 1 } else { 0 };
    (base & 0xFFFF_FFE0) | ((sh & 0x3) << 3) | ((ap & 0x3) << 1) | xn
}

#[inline]
const fn mpu_rlar(limit: u32, attr_idx: u32) -> u32 {
    (limit & 0xFFFF_FFE0) | ((attr_idx & 0x7) << 1) | 1
}

/// 让跨核共享 SRAM 变为 non-cacheable（最小子集）。
///
/// - HPSYS SRAM: `0x2000_0000..=0x2027_FFFF`
/// - LPSYS SRAM: `0x203F_C000..=0x204F_FFFF`
///
/// 这两个范围覆盖 52x 的 mailbox IPC ring buffers（`0x2007_FC00..` / `0x2040_2800..` 等）。
pub(crate) unsafe fn init() {
    let mpu = &*cortex_m::peripheral::MPU::PTR;

    // 确保后续 MPU 配置生效。
    asm::dsb();
    asm::isb();

    // 先关 MPU。
    mpu.ctrl.write(0);
    asm::dsb();
    asm::isb();

    // Attr0: non-cacheable normal memory。
    mpu.mair[0].write(ATTR_NON_CACHEABLE_NORMAL);

    // Region 0: HPSYS SRAM non-cacheable。
    mpu.rnr.write(0);
    mpu.rbar.write(mpu_rbar(0x2000_0000, MPU_RBAR_SH_NON, MPU_RBAR_AP_RW_ANY, false));
    mpu.rlar.write(mpu_rlar(0x2027_FFFF, 0));

    // Region 1: LPSYS SRAM non-cacheable。
    mpu.rnr.write(1);
    mpu.rbar.write(mpu_rbar(0x203F_C000, MPU_RBAR_SH_NON, MPU_RBAR_AP_RW_ANY, false));
    mpu.rlar.write(mpu_rlar(0x204F_FFFF, 0));

    // 开启 MPU：保留默认内存映射（PRIVDEFENA），避免未覆盖区域直接 fault。
    mpu.ctrl
        .write(MPU_CTRL_ENABLE | MPU_CTRL_HFNMIENA | MPU_CTRL_PRIVDEFENA);

    asm::dsb();
    asm::isb();

    // 便于在日志中确认 MPU 确实已配置/生效。
    let ctrl = mpu.ctrl.read();
    let mair0 = mpu.mair[0].read();
    mpu.rnr.write(0);
    let r0_rbar = mpu.rbar.read();
    let r0_rlar = mpu.rlar.read();
    mpu.rnr.write(1);
    let r1_rbar = mpu.rbar.read();
    let r1_rlar = mpu.rlar.read();
    debug!(
        "mpu: ctrl=0x{:08X} mair0=0x{:08X} r0=(rbar=0x{:08X} rlar=0x{:08X}) r1=(rbar=0x{:08X} rlar=0x{:08X})",
        ctrl,
        mair0,
        r0_rbar,
        r0_rlar,
        r1_rbar,
        r1_rlar
    );
}
