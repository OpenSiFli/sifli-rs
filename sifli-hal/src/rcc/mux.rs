// pub use crate::pac::hpsys_rcc::vals::SelTick as TickSel;

#[non_exhaustive]
pub struct ClockMux {
    pub rtcsel: Rtcsel,
    pub wdtsel: Wdtsel,
    pub usbsel: Usbsel,
    pub perisel: PeriSel,
    pub mpi1sel: MpiSel,
    pub mpi2sel: MpiSel,
    pub ticksel: Ticksel,
    pub lpsel: Lpsel,
}

impl Default for ClockMux {
    fn default() -> Self {
        Self {
            rtcsel: Rtcsel::LRC10,
            wdtsel: Wdtsel::LRC32, // WDT typically uses LRC32 or LXT32
            usbsel: Usbsel::SYSCLK,
            perisel: PeriSel::HXT48,
            mpi1sel: MpiSel::PERI,
            mpi2sel: MpiSel::DLL2,
            ticksel: Ticksel::RTC, // Use RTC clock for system tick by default
            lpsel: Lpsel::SEL_SYS, // Use system clock selection
        }
    }
}

/// CSR_SEL_USB
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Usbsel {
    SYSCLK = 0,
    DLL2 = 1,
}

/// LPCKSEL
#[repr(u8)]
pub enum Rtcsel {
    LRC10 = 0,
    LXT32 = 1,
}

/// LPCLK
#[repr(u8)]
pub enum Wdtsel {
    LRC10 = 0,
    LRC32 = 1,
}

/// CSR_SEL_PERI
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeriSel {
    HRC48 = 0,
    HXT48 = 1,
}

/// CSR_SEL_MPI1/2
/// MPI (Memory Peripheral Interface) can be used for Flash or PSRAM
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpiSel {
    /// Use clk_peri_hpsys
    PERI = 0,
    /// Use DLL2 (typically 288MHz for high-speed Flash/PSRAM)
    DLL2 = 2,
    /// Use DLL3 (only available on some models)
    DLL3 = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Ticksel {
    RTC = 0,
    _RESERVED_1 = 1,
    HRC48 = 2,
    HXT48 = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Lpsel {
    SEL_SYS = 0,
    WDT = 1,
}
