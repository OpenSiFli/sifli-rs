use crate::pac::HPSYS_CFG;

// SF32LB52X
#[allow(dead_code)]
const HAL_CHIP_REV_ID_A3: u8 = 0xFF; // Not defined
const HAL_CHIP_REV_ID_A4: u8 = 0xFF;

pub fn get_pid() -> u8 {
    HPSYS_CFG.idr().read().pid()
}


pub fn get_revid() -> u8 {
    HPSYS_CFG.idr().read().revid()
}

pub fn is_letter_series() -> bool {
    get_revid() == HAL_CHIP_REV_ID_A4
}