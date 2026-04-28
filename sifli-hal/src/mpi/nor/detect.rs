use super::super::types::Error;
use super::super::xip;
use super::super::Instance;
use super::profile::{resolve_builtin_profile, BuiltInProfile, NorFamily, NorProfile};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct JedecId {
    pub manufacturer: u8,
    pub memory_type: u8,
    pub density: u8,
}

impl JedecId {
    pub(crate) fn from_raw(value: u32) -> Self {
        Self {
            manufacturer: (value & 0xff) as u8,
            memory_type: ((value >> 8) & 0xff) as u8,
            density: ((value >> 16) & 0xff) as u8,
        }
    }

    pub(crate) const fn as_raw(self) -> u32 {
        (self.manufacturer as u32)
            | ((self.memory_type as u32) << 8)
            | ((self.density as u32) << 16)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DetectedNorInfo {
    pub jedec: JedecId,
    pub profile_name: &'static str,
    pub family: NorFamily,
}

const JEDEC_COMMON_SPI_NOR_8MIB: &[u32] = &[
    0x1740EF, // Winbond W25Q64
    0x1740C8, // GigaDevice GD25Q64
    0x1720C2, // Macronix MX25L64
    0x17400B, // XTX XT25F64
    0x17701C, // EON EN25Q64
];
const JEDEC_COMMON_SPI_NOR_4MIB: &[u32] = &[
    0x1640EF, // Winbond W25Q32
    0x1640C8, // GigaDevice GD25Q32
    0x1620C2, // Macronix MX25L32
    0x16400B, // XTX XT25F32
    0x16701C, // EON EN25Q32
];
const JEDEC_COMMON_SPI_NOR_16MIB: &[u32] = &[
    0x1840EF, // Winbond W25Q128
    0x1840C8, // GigaDevice GD25Q128
    0x1820C2, // Macronix MX25L128
    0x18400B, // XTX XT25F128
    0x182085, // Puya PY25Q128HA
    0x18701C, // EON EN25Q128
];
const JEDEC_COMMON_SPI_NOR_32MIB: &[u32] = &[
    0x1940EF, // Winbond W25Q256
    0x1940C8, // GigaDevice GD25Q256
    0x1920C2, // Macronix MX25L256
    0x19400B, // XTX XT25F256
];
const JEDEC_COMMON_SPI_NOR_64MIB: &[u32] = &[
    0x2040EF, // Winbond W25Q512
    0x2040C8, // GigaDevice GD25Q512
    0x2020C2, // Macronix MX66L512
];

pub(crate) fn read_jedec_id<T: Instance>(max_ready_polls: u32) -> Result<JedecId, Error> {
    let xip_safe = xip::running_from_same_instance_xip::<T>();
    let raw = xip::read_jedec_id(T::regs(), 0x9F, max_ready_polls, xip_safe)
        .map_err(|_| Error::Timeout)?;
    Ok(JedecId::from_raw(raw))
}

#[inline(always)]
const fn swap_jedec_mfg_density(raw: u32) -> u32 {
    ((raw & 0x0000_00ff) << 16) | (raw & 0x0000_ff00) | ((raw & 0x00ff_0000) >> 16)
}

fn detect_builtin_raw(raw: u32) -> Option<BuiltInProfile> {
    if JEDEC_COMMON_SPI_NOR_4MIB.contains(&raw) {
        Some(BuiltInProfile::CommonSpiNor4MiB)
    } else if JEDEC_COMMON_SPI_NOR_8MIB.contains(&raw) {
        Some(BuiltInProfile::CommonSpiNor8MiB)
    } else if JEDEC_COMMON_SPI_NOR_16MIB.contains(&raw) {
        Some(BuiltInProfile::CommonSpiNor16MiB)
    } else if JEDEC_COMMON_SPI_NOR_32MIB.contains(&raw) {
        Some(BuiltInProfile::CommonSpiNor32MiB)
    } else if JEDEC_COMMON_SPI_NOR_64MIB.contains(&raw) {
        Some(BuiltInProfile::CommonSpiNor64MiB)
    } else {
        None
    }
}

fn detect_builtin(jedec: JedecId) -> Option<BuiltInProfile> {
    let raw = jedec.as_raw();
    detect_builtin_raw(raw).or_else(|| detect_builtin_raw(swap_jedec_mfg_density(raw)))
}

pub(crate) fn detect_profile(
    jedec: JedecId,
) -> Result<(&'static dyn NorProfile, DetectedNorInfo), Error> {
    let profile = detect_builtin(jedec).ok_or(Error::UnknownJedecId)?;
    let profile = resolve_builtin_profile(profile);
    Ok((
        profile,
        DetectedNorInfo {
            jedec,
            profile_name: profile.name(),
            family: profile.family(),
        },
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jedec_from_raw_round_trips() {
        let raw = 0x1840EF;
        let jedec = JedecId::from_raw(raw);

        assert_eq!(jedec.manufacturer, 0xEF);
        assert_eq!(jedec.memory_type, 0x40);
        assert_eq!(jedec.density, 0x18);
        assert_eq!(jedec.as_raw(), raw);
    }

    #[test]
    fn detect_known_16mib_profile() {
        let jedec = JedecId {
            manufacturer: 0xEF,
            memory_type: 0x40,
            density: 0x18,
        };
        let (profile, detected) = detect_profile(jedec).unwrap();

        assert_eq!(profile.name(), "common-spi-nor-16mib");
        assert_eq!(detected.profile_name, "common-spi-nor-16mib");
        assert_eq!(detected.family, NorFamily::Type0);
    }

    #[test]
    fn detect_known_64mib_profile() {
        let jedec = JedecId {
            manufacturer: 0xEF,
            memory_type: 0x40,
            density: 0x20,
        };
        let (profile, detected) = detect_profile(jedec).unwrap();

        assert_eq!(profile.name(), "common-spi-nor-64mib");
        assert_eq!(detected.profile_name, "common-spi-nor-64mib");
    }

    #[test]
    fn detect_known_xtx_16mib_profile() {
        let jedec = JedecId {
            manufacturer: 0x0B,
            memory_type: 0x40,
            density: 0x18,
        };
        let (profile, detected) = detect_profile(jedec).unwrap();

        assert_eq!(profile.name(), "common-spi-nor-16mib");
        assert_eq!(detected.profile_name, "common-spi-nor-16mib");
    }

    #[test]
    fn detect_swapped_xtx_16mib_profile() {
        let jedec = JedecId {
            manufacturer: 0x18,
            memory_type: 0x40,
            density: 0x0B,
        };
        let (profile, detected) = detect_profile(jedec).unwrap();

        assert_eq!(profile.name(), "common-spi-nor-16mib");
        assert_eq!(detected.profile_name, "common-spi-nor-16mib");
    }

    #[test]
    fn detect_unknown_profile_returns_explicit_error() {
        let unknown = JedecId {
            manufacturer: 0xAA,
            memory_type: 0xBB,
            density: 0xCC,
        };
        assert_eq!(detect_profile(unknown), Err(Error::UnknownJedecId));
    }
}
