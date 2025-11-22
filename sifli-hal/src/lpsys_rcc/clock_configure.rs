use crate::pac::LPSYS_RCC;
use crate::rcc::ConfigOption;

use super::{ClkPeriSel, ClkSysSel, TickSel};

/// LPSYS RCC configuration.
///
/// Similar to HPSYS `rcc::Config` but only for LPSYS-related fields.
/// All fields default to [`ConfigOption::Keep`], meaning "do not change".
pub struct LpsysConfig {
    /// LPSYS main clock source (SEL_SYS); remains defined but not effective when WDT is used.
    pub sys_main_src: ConfigOption<ClkSysSel>,
    /// Whether to use the WDT domain as LPSYS clock (SEL_SYS_LP).
    pub sys_use_wdt: ConfigOption<bool>,
    /// HCLK divider: hclk_lpsys = clk_lpsys / hclk_div (0 means no division).
    pub hclk_div: ConfigOption<u8>,
    /// PCLK1 divider: pclk1_lpsys = hclk_lpsys / 2^pclk1_div.
    pub pclk1_div: ConfigOption<u8>,
    /// PCLK2 divider: pclk2_lpsys = hclk_lpsys / 2^pclk2_div.
    pub pclk2_div: ConfigOption<u8>,
    /// LPSYS peripheral clock source (SEL_PERI).
    pub peri_src: ConfigOption<ClkPeriSel>,
    /// LPSYS systick clock.
    pub tick: ConfigOption<LpsysTickConfig>,
}

/// LPSYS systick clock configuration.
pub struct LpsysTickConfig {
    /// Systick reference clock source (SEL_TICK).
    pub sel: TickSel,
    /// Divider: tick = ref / TICKDIV (0 means no division).
    pub div: u8,
}

impl LpsysConfig {
    /// Create a config that keeps all current register values.
    pub fn new_keep() -> Self {
        Self {
            sys_main_src: ConfigOption::keep(),
            sys_use_wdt: ConfigOption::keep(),
            hclk_div: ConfigOption::keep(),
            pclk1_div: ConfigOption::keep(),
            pclk2_div: ConfigOption::keep(),
            peri_src: ConfigOption::keep(),
            tick: ConfigOption::keep(),
        }
    }

    /// Apply the LPSYS RCC configuration to hardware.
    ///
    /// # Safety
    ///
    /// Changing LPSYS clocks affects LCPU / BT MAC, etc.
    /// Caller must ensure this is done at a safe time for the running firmware.
    pub unsafe fn apply(&self) {
        // Main system clock source (SEL_SYS).
        if let ConfigOption::Update(sel) = self.sys_main_src {
            LPSYS_RCC.csr().modify(|w| w.set_sel_sys(sel));
        }

        // Whether to use WDT domain clock (SEL_SYS_LP).
        if let ConfigOption::Update(use_wdt) = self.sys_use_wdt {
            LPSYS_RCC.csr().modify(|w| w.set_sel_sys_lp(use_wdt));
        }

        // HCLK / PCLK dividers.
        if let ConfigOption::Update(div) = self.hclk_div {
            LPSYS_RCC.cfgr().modify(|w| w.set_hdiv1(div));
        }
        if let ConfigOption::Update(div) = self.pclk1_div {
            LPSYS_RCC.cfgr().modify(|w| w.set_pdiv1(div));
        }
        if let ConfigOption::Update(div) = self.pclk2_div {
            LPSYS_RCC.cfgr().modify(|w| w.set_pdiv2(div));
        }

        // LPSYS peripheral clock source.
        if let ConfigOption::Update(sel) = self.peri_src {
            LPSYS_RCC.csr().modify(|w| w.set_sel_peri(sel));
        }

        // Systick clock.
        if let ConfigOption::Update(tick) = &self.tick {
            LPSYS_RCC.csr().modify(|w| w.set_sel_tick(tick.sel));
            LPSYS_RCC.cfgr().modify(|w| w.set_tickdiv(tick.div));
        }
    }
}

impl Default for LpsysConfig {
    fn default() -> Self {
        Self::new_keep()
    }
}
