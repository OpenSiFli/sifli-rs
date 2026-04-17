use std::mem::offset_of;

use crate::ptab;

pub(crate) mod structure;

const DFU_FLAG_AUTO: u16 = 2;

pub struct Ftab {
    pub(crate) structure: structure::SecConfiguration,
}

impl Ftab {
    pub fn new() -> Self {
        Self {
            structure: Default::default(),
        }
    }

    // Apply the partition table to the flash table
    pub fn apply(&mut self, table: &ptab::Ptab) {
        self.structure
            .ftab
            .secure_config
            .apply_info(&table.flash_table_info);

        self.structure
            .ftab
            .factory_calibration
            .apply_info(&table.flash_cal_info);

        let used_hcpu = if let Some(hcpu_info) = table.hcpu_code_info.clone() {
            self.structure.ftab.hcpu.apply_info(&hcpu_info);
            // To be consist with ftab.c
            self.structure.ftab.hcpu.size = 0x0020_0000;

            self.structure.ftab.hcpu2.apply_info(&hcpu_info);
            // To be consist with ftab.c
            self.structure.ftab.hcpu2.size = 0x0020_0000;
            true
        } else {
            false
        };

        // let used_lcpu = if let Some(lcpu_info) = table.lcpu_code_info.clone() {
        //     self.structure.ftab.lcpu.apply_info(&lcpu_info);
        //     // To be consist with ftab.c
        //     self.structure.ftab.lcpu.size = 0x0020_0000;

        //     self.structure.ftab.lcpu2.apply_info(&lcpu_info);
        //     // To be consist with ftab.c
        //     self.structure.ftab.lcpu2.size = 0x0020_0000;
        //     true
        // } else {
        //     false
        // };

        // they are different:
        self.structure
            .ftab
            .primary_bl_patch
            .apply_info(&table.primary_bl_patch_info);
        self.structure
            .ftab
            .primary_bl_patch2
            .apply_info(&table.primary_bl_patch2_info);

        self.structure
            .ftab
            .secondary_bl
            .apply_info(&table.secondary_bl_info);
        self.structure
            .ftab
            .secondary_bl2
            .apply_info(&table.secondary_bl_info);

        if used_hcpu {
            // TODO: real size
            self.structure.imgs.hcpu.length = 0x0020_0000;
            self.structure.imgs.hcpu.blksize = 512;
            self.structure.imgs.hcpu.flags = DFU_FLAG_AUTO;
        } else {
            self.structure.imgs.hcpu.length = 0xFFFFFFFF;
        }

        // if used_lcpu {
        //     self.structure.imgs.lcpu.length = 200000;
        //     self.structure.imgs.lcpu.blksize = 512;
        //     self.structure.imgs.lcpu.flags = DFU_FLAG_AUTO;
        // }
        // else {
        //     self.structure.imgs.lcpu.length = 0xFFFFFFFF;
        // }

        self.structure.imgs.lcpu.length = 0xFFFFFFFF;
        // TODO: real size
        self.structure.imgs.secondary_bl.length = 0xFFFF; // 64K
        self.structure.imgs.secondary_bl.blksize = 512;
        self.structure.imgs.secondary_bl.flags = DFU_FLAG_AUTO;
        self.structure.imgs.primary_bl_patch.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu2.length = 0xFFFFFFFF;
        self.structure.imgs.secondary_bl2.length = 0xFFFFFFFF;
        self.structure.imgs.hcpu2.length = 0xFFFFFFFF;
        self.structure.imgs.primary_bl_patch2.length = 0xFFFFFFFF;
        self.structure.imgs.hcpu_ext2.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu_ext1.length = 0xFFFFFFFF;
        self.structure.imgs.lcpu_ext2.length = 0xFFFFFFFF;
        self.structure.imgs.reserved.length = 0xFFFFFFFF;
        self.structure.imgs.single.length = 0xFFFFFFFF;

        self.structure.running_imgs.hcpu = if used_hcpu {
            (offset_of!(structure::SecConfiguration, imgs) + offset_of!(structure::Imgs, hcpu))
                as u32
                + table.flash_table_info.base_addr
        } else {
            0xFFFFFFFF
        };

        // self.structure.running_imgs.lcpu = if used_lcpu {
        //     (offset_of!(structure::SecConfiguration, imgs)
        //     + offset_of!(structure::Imgs, lcpu)) as u32
        //     + table.flash_table_info.base_addr
        // }
        // else {
        //     0xFFFFFFFF
        // };

        self.structure.running_imgs.secondary_bl = (offset_of!(structure::SecConfiguration, imgs)
            + offset_of!(structure::Imgs, secondary_bl))
            as u32
            + table.flash_table_info.base_addr
    }

    pub fn to_bytes(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts((self as *const Self) as *const u8, size_of::<Self>()) }
    }
}

impl structure::FlashTable {
    fn apply_info(&mut self, info: &ptab::Info) {
        self.size = info.size;
        self.base = info.base_addr;
        self.xip_base = info.xip_addr;
    }
}
