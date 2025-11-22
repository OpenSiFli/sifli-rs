use crate::pac::{EFUSEC, PMUC};

/// HAL_EFUSE_BANK_SIZE
const BANK_SIZE: u8 = 32;

static mut FACTORY_CFG_VBK_LDO: Option<FactoryCfgVbkLdo> = None;

/// Read EFUSE data into provided buffer
/// 
/// # Arguments
/// * `bit_offset` - Starting bit offset (must be 32-bit aligned)
/// * `data` - Buffer to store read data
/// * `size` - Size of data to read (must be multiple of 4)
/// 
/// # Returns
/// * `Ok(usize)` - Number of bytes read
/// * `Err(())` - If parameters are invalid or read failed
pub fn read(bit_offset: u16, data: &mut [u8], size: usize) -> Result<usize, ()> {
    // Validate input parameters
    if !validate_parameters(bit_offset, size) || data.len() < size {
        return Err(());
    }

    let byte_off = (bit_offset >> 3) as usize % BANK_SIZE as usize;
    let bank = (bit_offset >> 8) as u8;

    // Get original HPSYS VOUT value and set new value
    let org = PMUC.hpsys_vout().read().vout();
    let value = (org + 3).clamp(0xe, 0xf);
    PMUC.hpsys_vout().write(|w| w.set_vout(value));

    crate::cortex_m_blocking_delay_us(20);

    // Configure EFUSE control register
    EFUSEC.cr().modify(|w| w.set_banksel(bank));
    EFUSEC.cr().modify(|w| w.set_en(true));

    // Wait for completion
    // mininum: two cycle one bit
    // here: 1ms one bit for 48MHz clock
    let timeout = size as u32 * 8 * 48000;
    if !wait_for_completion(timeout) {
        PMUC.hpsys_vout().write(|w| w.set_vout(org));
        return Err(());
    }

    // Clear done flag
    EFUSEC.sr().modify(|w| w.set_done(false));

    // Read data
    unsafe {
        let mut rd_reg = EFUSEC.bank0_data0().as_ptr()
            .add((bank << 3) as _)
            .add(byte_off >> 2);
    
        let word_size = size >> 2;
        for i in 0..word_size {
            let val = rd_reg.read().0;
            rd_reg = rd_reg.add(4);
            let offset = i * 4;
            data[offset] = val as u8;
            data[offset + 1] = (val >> 8) as u8;
            data[offset + 2] = (val >> 16) as u8;
            data[offset + 3] = (val >> 24) as u8;
        }
    }
    // Restore original VOUT value
    PMUC.hpsys_vout().modify(| w| w.set_vout(org));

    Ok(size)
}

fn validate_parameters(bit_offset: u16, size: usize) -> bool {
    let byte_off = (bit_offset >> 3) as usize % BANK_SIZE as usize;
    
    size <= BANK_SIZE as _ 
        && (byte_off + size) <= BANK_SIZE as _ 
        && (size & 3) == 0
        && (bit_offset & 31) == 0
}

fn wait_for_completion(timeout: u32) -> bool {
    let mut ready = 0;
    while (!EFUSEC.sr().read().done()) && ready < timeout {
        ready += 1;
    }
    ready < timeout
}

/// Factory configuration ID VBUCK implementation
pub fn get_factory_cfg_id_vbuck(pvdd_v18_en: bool) -> Option<&'static FactoryCfgVbkLdo> {
    unsafe {
        if (&*&raw const FACTORY_CFG_VBK_LDO).is_none() {
            FACTORY_CFG_VBK_LDO = read_factory_cfg_id_vbuck(pvdd_v18_en);
        }
        (&*&raw const FACTORY_CFG_VBK_LDO).as_ref()
    }
}

/// Factory configuration ID VBUCK implementation
fn read_factory_cfg_id_vbuck(pvdd_v18_en: bool) -> Option<FactoryCfgVbkLdo> {
    let mut data = [0u8; 32];
    if read(0, &mut data, 32).is_err() {
        return None;
    }

    // Read IDR register directly (internal use only, no need for SysCfg driver)
    let idr_val = crate::pac::HPSYS_CFG.idr().read();
    let pid = idr_val.pid();
    let revid = idr_val.revid();
    let mut cfg = FactoryCfgVbkLdo::default();

    // Check if Letter Series (REVID == 0x07 or 0x0F)
    let is_letter_series = revid == 0x07 || revid == 0x0F;

    if is_letter_series {
        if (pid == 4 && !pvdd_v18_en) || (pid != 4 && pvdd_v18_en) {
            // 52D 3.3V or 52A 1.8V configuration
            cfg.buck_vos_trim = data[20] & 7;
            cfg.buck_vos_polar = (data[20] & 8) >> 3;
            cfg.hpsys_ldo_vout = (data[20] & 0xf0) >> 4;
            cfg.lpsys_ldo_vout = data[21] & 0xf;
            cfg.vret_trim = (data[21] & 0xf0) >> 4;
            cfg.hpsys_ldo_vout2 = (data[28] & 0x78) >> 3;
        } else {
            // Standard configuration
            cfg.buck_vos_trim = data[0] & 7;
            cfg.buck_vos_polar = (data[0] & 8) >> 3;
            cfg.hpsys_ldo_vout = (data[0] & 0xf0) >> 4;
            cfg.lpsys_ldo_vout = data[1] & 0xf;
            cfg.vret_trim = (data[1] & 0xf0) >> 4;
            //cfg->buck_vos_trim2 = data[13] & 7;
            //cfg->buck_vos_polar2 = (data[13] & 8) >> 3;
            cfg.hpsys_ldo_vout2 = (data[13] & 0xf0) >> 4;
            cfg.lpsys_ldo_vout2 = data[14] & 0xf;
        }

        if !pvdd_v18_en {
            cfg.ldo18_vref_sel = data[2] & 0xf;
        }
    } else {
        // Non-letter series configuration
        cfg.buck_vos_trim = data[0] & 7;
        cfg.buck_vos_polar = (data[0] & 8) >> 3;
        cfg.hpsys_ldo_vout = (data[0] & 0xf0) >> 4;
        cfg.lpsys_ldo_vout = data[1] & 0xf;
        cfg.vret_trim = (data[1] & 0xf0) >> 4;
        cfg.vdd33_ldo2_vout = (data[2] & 0xf0) >> 4;
        cfg.vdd33_ldo3_vout = data[3] & 0xf;
        cfg.aon_vos_trim = (data[3] & 0x70) >> 4;
        cfg.aon_vos_polar = (data[3] & 0x80) >> 7;
        cfg.ldo18_vref_sel = data[2] & 0xf;
    }

    if cfg.hpsys_ldo_vout == 0 || cfg.hpsys_ldo_vout2 == 0 {
        None
    } else {
        Some(cfg)
    }
}

/// FACTORY_CFG_VBK_LDO_T
/// sf32lb52 only
#[repr(C)]
#[derive(Default, Debug)]
pub struct FactoryCfgVbkLdo {
    pub buck_vos_trim: u8,
    pub buck_vos_polar: u8,
    pub hpsys_ldo_vout: u8,
    pub lpsys_ldo_vout: u8,
    pub vret_trim: u8,
    pub ldo18_vref_sel: u8,
    pub vdd33_ldo2_vout: u8,
    pub vdd33_ldo3_vout: u8,
    pub aon_vos_trim: u8,
    pub aon_vos_polar: u8,
    pub buck_vos_trim2: u8,
    pub buck_vos_polar2: u8,
    pub hpsys_ldo_vout2: u8,
    pub lpsys_ldo_vout2: u8,
}

impl FactoryCfgVbkLdo {
    pub fn new() -> Self {
        FactoryCfgVbkLdo::default()
    }

    pub fn get_hpsys_vout_ref(&self) -> u8 {
        self.hpsys_ldo_vout
    }
    pub fn get_hpsys_vout_ref2(&self) -> u8 {
        self.hpsys_ldo_vout2
    }
}
