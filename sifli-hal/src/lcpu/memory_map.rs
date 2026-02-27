//! LCPU subsystem memory map constants (single source of truth).

/// Addresses shared across all chip revisions.
pub mod shared {
    pub const LPSYS_RAM_BASE: usize = 0x2040_0000;
    pub const NVDS_BUFF_START: usize = 0x2040_FE00;
    pub const EM_START: usize = 0x2040_8000;
    pub const EM_SIZE: usize = 0x5000;
    pub const HCPU2LCPU_MB_CH1: usize = 0x2007_FE00;
    pub const HCPU2LCPU_MB_CH2: usize = 0x2007_FC00;
    pub const HCPU_TO_LCPU_OFFSET: usize = 0x0A00_0000;
}

/// Bluetooth RF peripheral addresses.
pub mod rf {
    pub const BT_RFC_MEM_BASE: u32 = 0x4008_2000;
    pub const CFO_PHASE_ADDR: u32 = 0x4008_2790;
    pub const PHY_RX_DUMP_ADDR: u32 = 0x400C_0000;
}

/// A3 revision specific addresses.
pub mod a3 {
    pub const ROM_CONFIG_BASE: usize = 0x2040_FDC0;
    pub const LCPU2HCPU_CH1: usize = 0x2040_5C00;
    pub const LCPU2HCPU_CH2: usize = 0x2040_5E00;
    pub const PATCH_CODE_START: usize = 0x2040_6000;
    pub const PATCH_RECORD_ADDR: usize = 0x2040_7F00;
    pub const RWIP_PROG_DELAY: usize = 0x2040_FA94;
    pub const G_ROM_CONFIG: usize = 0x2040_E48C;
}

/// Letter Series (A4/B4) specific addresses.
pub mod letter {
    pub const ROM_CONFIG_BASE: usize = 0x2040_2A00;
    pub const LCPU2HCPU_CH1: usize = 0x2040_2800;
    pub const LCPU2HCPU_CH2: usize = 0x2040_2A00;
    pub const PATCH_BUF_START: usize = 0x2040_5000;
    pub const PATCH_CODE_START: usize = 0x2040_500C;
    pub const PATCH_CODE_START_LCPU: usize = 0x0040_500C;
}
