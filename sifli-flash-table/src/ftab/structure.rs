const DFU_SIG_KEY_SIZE: usize = 294;
const DFU_KEY_SIZE: usize = 32;
const DFU_SIG_SIZE: usize = 256;
const DFU_FLASH_PARTITION: usize = 16;
const DFU_VERSION_LEN: usize = 8;

const MAGIC: u32 = 0x53454346;
#[cfg(test)]
const CORE_MAX: usize = 4;

#[repr(C)]
pub(crate) struct SecConfiguration {
    pub(crate) magic: u32,
    pub(crate) ftab: FlashTables,
    pub(crate) sig_pub_key: [u8; DFU_SIG_KEY_SIZE],

    // Align to sector boundary (4096)
    pub(crate) reserved:
        [u8; 4096 - (4 + DFU_FLASH_PARTITION * size_of::<FlashTable>() + DFU_SIG_KEY_SIZE)],

    pub(crate) imgs: Imgs,
    pub(crate) running_imgs: RunningImgs,
}

// Accroding to:
// https://docs.sifli.com/projects/sdk/v2.3/sf32lb52x/bootloader.html (2025/1/27)
// https://github.com/OpenSiFli/SiFli-SDK/issues/10#issuecomment-2614345184
// https://github.com/OpenSiFli/SiFli-SDK/blob/8f42a6916c55c6b44ec45e1c0d137b15f7fa7fa3/middleware/dfu/dfu.h#L86-L96
#[repr(C)]
#[derive(Default)]
pub(crate) struct FlashTables {
    pub(crate) secure_config: FlashTable,
    pub(crate) factory_calibration: FlashTable,
    /// LCPU-Ping
    pub(crate) lcpu: FlashTable,
    /// BCPU-Ping
    pub(crate) secondary_bl: FlashTable,
    /// HCPU-Ping
    pub(crate) hcpu: FlashTable,
    /// Flash Boot Patch
    pub(crate) primary_bl_patch: FlashTable,
    /// LCPU Pong
    pub(crate) lcpu2: FlashTable,
    /// BCPU Pong
    pub(crate) secondary_bl2: FlashTable,
    /// HCPU Pong
    pub(crate) hcpu2: FlashTable,
    /// Ram Boot Patch
    pub(crate) primary_bl_patch2: FlashTable,
    pub(crate) hcpu_ext1: FlashTable,
    pub(crate) hcpu_ext2: FlashTable,
    pub(crate) lcpu_ext1: FlashTable,
    pub(crate) lcpu_ext2: FlashTable,
    pub(crate) reserved: FlashTable,
    /// Reservd?
    pub(crate) single: FlashTable,
}

#[repr(C)]
#[derive(Debug)]
pub(crate) struct FlashTable {
    pub(crate) base: u32,
    pub(crate) size: u32,
    pub(crate) xip_base: u32,
    pub(crate) flags: u32,
}

#[repr(C)]
#[derive(Default)]
pub(crate) struct Imgs {
    /// LCPU-Ping
    pub(crate) lcpu: ImageHeaderEnc,
    /// BCPU-Ping
    pub(crate) secondary_bl: ImageHeaderEnc,
    /// HCPU-Ping
    pub(crate) hcpu: ImageHeaderEnc,
    /// Flash Boot Patch
    pub(crate) primary_bl_patch: ImageHeaderEnc,
    /// LCPU Pong
    pub(crate) lcpu2: ImageHeaderEnc,
    /// BCPU Pong
    pub(crate) secondary_bl2: ImageHeaderEnc,
    /// HCPU Pong
    pub(crate) hcpu2: ImageHeaderEnc,
    /// Ram Boot Patch
    pub(crate) primary_bl_patch2: ImageHeaderEnc,
    pub(crate) hcpu_ext1: ImageHeaderEnc,
    pub(crate) hcpu_ext2: ImageHeaderEnc,
    pub(crate) lcpu_ext1: ImageHeaderEnc,
    pub(crate) lcpu_ext2: ImageHeaderEnc,
    pub(crate) reserved: ImageHeaderEnc,
    /// Reservd?
    pub(crate) single: ImageHeaderEnc,
}

// accroding to https://github.com/OpenSiFli/SiFli-SDK/issues/10#issuecomment-2614345184
#[repr(C)]
pub(crate) struct RunningImgs {
    // *const ImageHeaderEnc
    pub(crate) lcpu: u32,
    pub(crate) secondary_bl: u32,
    pub(crate) hcpu: u32,
    pub(crate) primary_bl_patch: u32,
}

#[repr(C)]
#[derive(Debug)]
pub(crate) struct ImageHeaderEnc {
    pub(crate) length: u32,
    pub(crate) blksize: u16,
    pub(crate) flags: u16,
    pub(crate) key: [u8; DFU_KEY_SIZE],
    pub(crate) sig: [u8; DFU_SIG_SIZE],
    pub(crate) ver: [u8; DFU_VERSION_LEN],
    pub(crate) reserved: [u8; 512 - DFU_KEY_SIZE - DFU_SIG_SIZE - 8 - DFU_VERSION_LEN],
}

impl Default for FlashTable {
    fn default() -> Self {
        FlashTable {
            base: 0,
            size: 0,
            xip_base: 0,
            flags: 0,
        }
    }
}

impl Default for RunningImgs {
    fn default() -> Self {
        RunningImgs {
            lcpu: 0xFFFF_FFFF,
            secondary_bl: 0xFFFF_FFFF,
            hcpu: 0xFFFF_FFFF,
            primary_bl_patch: 0xFFFF_FFFF,
        }
    }
}

impl Default for ImageHeaderEnc {
    fn default() -> Self {
        ImageHeaderEnc {
            length: 0,
            blksize: 0,
            flags: 0,
            key: [0; DFU_KEY_SIZE],
            sig: [0; DFU_SIG_SIZE],
            ver: [0; DFU_VERSION_LEN],
            reserved: [0; 512 - DFU_KEY_SIZE - DFU_SIG_SIZE - 8 - DFU_VERSION_LEN],
        }
    }
}

impl Default for SecConfiguration {
    fn default() -> Self {
        SecConfiguration {
            magic: MAGIC,
            ftab: FlashTables::default(),
            sig_pub_key: [0; DFU_SIG_KEY_SIZE],
            reserved: [0; 4096
                - (4 + DFU_FLASH_PARTITION * size_of::<FlashTable>() + DFU_SIG_KEY_SIZE)],
            imgs: Imgs::default(),
            running_imgs: RunningImgs::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::offset_of;
    use std::mem::size_of;
    #[test]
    fn validate_struct_sizes() {
        assert_eq!(size_of::<FlashTable>(), 16);

        assert_eq!(size_of::<FlashTables>(), 16 * size_of::<FlashTable>());

        assert_eq!(offset_of!(SecConfiguration, imgs), 4096);

        assert_eq!(size_of::<ImageHeaderEnc>(), 512);

        assert_eq!(size_of::<Imgs>(), 512 * (DFU_FLASH_PARTITION - 2));

        assert_eq!(size_of::<RunningImgs>(), CORE_MAX * 4);

        assert_eq!(size_of::<SecConfiguration>(), 0x2c10)
    }
}
