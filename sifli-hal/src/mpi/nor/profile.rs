use super::super::shared::{DEFAULT_DMA_THRESHOLD_BYTES, NOR_FLASH_MAX_3B_CAPACITY_BYTES};
use super::super::types::{AddressSize, Error};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadMode {
    Single,
    Quad,
    QuadDtr,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NorConfig {
    pub read_mode: ReadMode,
    pub dma_threshold_bytes: usize,
    pub max_ready_polls: u32,
    pub allow_preconfigured_4byte_in_xip: bool,
}

impl Default for NorConfig {
    fn default() -> Self {
        Self {
            read_mode: ReadMode::Single,
            dma_threshold_bytes: DEFAULT_DMA_THRESHOLD_BYTES,
            max_ready_polls: 2_000_000,
            allow_preconfigured_4byte_in_xip: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NorFamily {
    Unknown,
    Type0,
    Type1,
    Type2,
    Type3,
    Type4,
    Type5,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum QePolicy {
    None,
    Unsupported,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddressingPolicy {
    ThreeByteOnly,
    Enter4ByteMode,
    Preconfigured4ByteOnly,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DtrPolicy {
    Disabled,
    Unsupported,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AhbReadPolicy {
    SingleIo,
    Unsupported,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NorCommandSet {
    pub write_enable: u8,
    pub enter_4byte_address: Option<u8>,
    pub read_status: u8,
    pub ahb_read: u8,
    pub ahb_read_4byte: Option<u8>,
    pub page_program: u8,
    pub page_program_4byte: Option<u8>,
    pub sector_erase: u8,
    pub sector_erase_4byte: Option<u8>,
    pub block_erase_32k: Option<u8>,
    pub block_erase_32k_4byte: Option<u8>,
    pub block_erase_64k: Option<u8>,
    pub block_erase_64k_4byte: Option<u8>,
    pub chip_erase: Option<u8>,
    pub read_jedec_id: u8,
    pub deep_power_down: Option<u8>,
    pub release_power_down: Option<u8>,
    pub reset_enable: Option<u8>,
    pub reset: Option<u8>,
    pub read_sfdp: Option<u8>,
    pub read_unique_id: Option<u8>,
}

impl NorCommandSet {
    pub const fn common_spi_nor() -> Self {
        Self {
            write_enable: 0x06,
            enter_4byte_address: Some(0xB7),
            read_status: 0x05,
            ahb_read: 0x03,
            ahb_read_4byte: Some(0x13),
            page_program: 0x02,
            page_program_4byte: Some(0x12),
            sector_erase: 0x20,
            sector_erase_4byte: Some(0x21),
            block_erase_32k: Some(0x52),
            block_erase_32k_4byte: None,
            block_erase_64k: Some(0xD8),
            block_erase_64k_4byte: Some(0xDC),
            chip_erase: Some(0xC7),
            read_jedec_id: 0x9F,
            deep_power_down: Some(0xB9),
            release_power_down: Some(0xAB),
            reset_enable: Some(0x66),
            reset: Some(0x99),
            read_sfdp: Some(0x5A),
            read_unique_id: Some(0x4B),
        }
    }
}

impl Default for NorCommandSet {
    fn default() -> Self {
        Self::common_spi_nor()
    }
}

pub trait NorProfile: Sync {
    fn name(&self) -> &'static str;
    fn family(&self) -> NorFamily;
    fn capacity_bytes(&self) -> usize;

    fn page_size(&self) -> usize {
        256
    }

    fn address_size(&self) -> AddressSize {
        if self.capacity_bytes() > NOR_FLASH_MAX_3B_CAPACITY_BYTES {
            AddressSize::FourBytes
        } else {
            AddressSize::ThreeBytes
        }
    }

    fn command_set(&self) -> NorCommandSet {
        NorCommandSet::common_spi_nor()
    }

    fn qe_policy(&self) -> QePolicy {
        QePolicy::None
    }

    fn addressing_policy(&self) -> AddressingPolicy {
        if self.capacity_bytes() > NOR_FLASH_MAX_3B_CAPACITY_BYTES {
            AddressingPolicy::Enter4ByteMode
        } else {
            AddressingPolicy::ThreeByteOnly
        }
    }

    fn dtr_policy(&self) -> DtrPolicy {
        DtrPolicy::Disabled
    }

    fn ahb_read_policy(&self) -> AhbReadPolicy {
        AhbReadPolicy::SingleIo
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BuiltInProfile {
    CommonSpiNor4MiB,
    CommonSpiNor8MiB,
    CommonSpiNor16MiB,
    CommonSpiNor32MiB,
    CommonSpiNor64MiB,
}

pub enum ProfileSource {
    BuiltIn(BuiltInProfile),
    Custom(&'static dyn NorProfile),
}

#[derive(Clone, Copy)]
pub(crate) struct ResolvedProfile {
    pub profile: &'static dyn NorProfile,
    pub commands: NorCommandSet,
    pub capacity_bytes: usize,
    pub page_size: usize,
    pub address_size: AddressSize,
}

#[derive(Clone, Copy)]
struct StaticNorProfile {
    name: &'static str,
    family: NorFamily,
    capacity_bytes: usize,
    page_size: usize,
    commands: NorCommandSet,
}

impl NorProfile for StaticNorProfile {
    fn name(&self) -> &'static str {
        self.name
    }

    fn family(&self) -> NorFamily {
        self.family
    }

    fn capacity_bytes(&self) -> usize {
        self.capacity_bytes
    }

    fn page_size(&self) -> usize {
        self.page_size
    }

    fn command_set(&self) -> NorCommandSet {
        self.commands
    }
}

static COMMON_SPI_NOR_8MIB: StaticNorProfile = StaticNorProfile {
    name: "common-spi-nor-8mib",
    family: NorFamily::Type0,
    capacity_bytes: 8 * 1024 * 1024,
    page_size: 256,
    commands: NorCommandSet::common_spi_nor(),
};

static COMMON_SPI_NOR_4MIB: StaticNorProfile = StaticNorProfile {
    name: "common-spi-nor-4mib",
    family: NorFamily::Type0,
    capacity_bytes: 4 * 1024 * 1024,
    page_size: 256,
    commands: NorCommandSet::common_spi_nor(),
};

static COMMON_SPI_NOR_16MIB: StaticNorProfile = StaticNorProfile {
    name: "common-spi-nor-16mib",
    family: NorFamily::Type0,
    capacity_bytes: 16 * 1024 * 1024,
    page_size: 256,
    commands: NorCommandSet::common_spi_nor(),
};

static COMMON_SPI_NOR_32MIB: StaticNorProfile = StaticNorProfile {
    name: "common-spi-nor-32mib",
    family: NorFamily::Type0,
    capacity_bytes: 32 * 1024 * 1024,
    page_size: 256,
    commands: NorCommandSet::common_spi_nor(),
};

static COMMON_SPI_NOR_64MIB: StaticNorProfile = StaticNorProfile {
    name: "common-spi-nor-64mib",
    family: NorFamily::Type0,
    capacity_bytes: 64 * 1024 * 1024,
    page_size: 256,
    commands: NorCommandSet::common_spi_nor(),
};

pub(crate) fn resolve_builtin_profile(profile: BuiltInProfile) -> &'static dyn NorProfile {
    match profile {
        BuiltInProfile::CommonSpiNor4MiB => &COMMON_SPI_NOR_4MIB,
        BuiltInProfile::CommonSpiNor8MiB => &COMMON_SPI_NOR_8MIB,
        BuiltInProfile::CommonSpiNor16MiB => &COMMON_SPI_NOR_16MIB,
        BuiltInProfile::CommonSpiNor32MiB => &COMMON_SPI_NOR_32MIB,
        BuiltInProfile::CommonSpiNor64MiB => &COMMON_SPI_NOR_64MIB,
    }
}

pub(crate) fn resolve_profile_source(source: ProfileSource) -> &'static dyn NorProfile {
    match source {
        ProfileSource::BuiltIn(profile) => resolve_builtin_profile(profile),
        ProfileSource::Custom(profile) => profile,
    }
}

pub(crate) fn validate_and_resolve(
    profile: &'static dyn NorProfile,
    config: &NorConfig,
) -> Result<ResolvedProfile, Error> {
    if config.max_ready_polls == 0 || config.dma_threshold_bytes == 0 {
        return Err(Error::InvalidConfiguration);
    }

    if config.read_mode != ReadMode::Single {
        return Err(Error::UnsupportedProfileFeature);
    }
    if profile.qe_policy() != QePolicy::None
        || profile.dtr_policy() != DtrPolicy::Disabled
        || profile.ahb_read_policy() != AhbReadPolicy::SingleIo
    {
        return Err(Error::UnsupportedProfileFeature);
    }

    let capacity = profile.capacity_bytes();
    let page_size = profile.page_size();
    let address_size = profile.address_size();
    let commands = profile.command_set();

    if capacity == 0 || page_size == 0 || !page_size.is_multiple_of(1) {
        return Err(Error::InvalidConfiguration);
    }
    if commands.write_enable == 0
        || commands.read_status == 0
        || commands.read_jedec_id == 0
        || commands.page_program == 0
        || commands.sector_erase == 0
        || commands.ahb_read == 0
    {
        return Err(Error::InvalidConfiguration);
    }

    let needs_4byte = capacity > NOR_FLASH_MAX_3B_CAPACITY_BYTES;
    if needs_4byte && (address_size != AddressSize::FourBytes || commands.ahb_read_4byte.is_none())
    {
        return Err(Error::UnsupportedProfileFeature);
    }
    if needs_4byte && profile.addressing_policy() == AddressingPolicy::ThreeByteOnly {
        return Err(Error::UnsupportedProfileFeature);
    }

    Ok(ResolvedProfile {
        profile,
        commands,
        capacity_bytes: capacity,
        page_size,
        address_size,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MissingWriteEnableProfile;

    impl NorProfile for MissingWriteEnableProfile {
        fn name(&self) -> &'static str {
            "missing-write-enable"
        }

        fn family(&self) -> NorFamily {
            NorFamily::Unknown
        }

        fn capacity_bytes(&self) -> usize {
            8 * 1024 * 1024
        }

        fn command_set(&self) -> NorCommandSet {
            let mut commands = NorCommandSet::common_spi_nor();
            commands.write_enable = 0;
            commands
        }
    }

    #[test]
    fn default_config_matches_contract() {
        let cfg = NorConfig::default();
        assert_eq!(cfg.read_mode, ReadMode::Single);
        assert_eq!(cfg.dma_threshold_bytes, DEFAULT_DMA_THRESHOLD_BYTES);
        assert_eq!(cfg.max_ready_polls, 2_000_000);
        assert!(!cfg.allow_preconfigured_4byte_in_xip);
    }

    #[test]
    fn resolve_builtin_64mib_uses_4byte_address() {
        let profile = resolve_builtin_profile(BuiltInProfile::CommonSpiNor64MiB);
        assert_eq!(profile.capacity_bytes(), 64 * 1024 * 1024);
        assert_eq!(profile.address_size(), AddressSize::FourBytes);
        assert_eq!(
            profile.addressing_policy(),
            AddressingPolicy::Enter4ByteMode
        );
    }

    #[test]
    fn validate_rejects_invalid_config_values() {
        let profile = resolve_builtin_profile(BuiltInProfile::CommonSpiNor16MiB);

        let mut zero_polls = NorConfig::default();
        zero_polls.max_ready_polls = 0;
        assert_eq!(
            validate_and_resolve(profile, &zero_polls),
            Err(Error::InvalidConfiguration)
        );

        let mut zero_dma_threshold = NorConfig::default();
        zero_dma_threshold.dma_threshold_bytes = 0;
        assert_eq!(
            validate_and_resolve(profile, &zero_dma_threshold),
            Err(Error::InvalidConfiguration)
        );
    }

    #[test]
    fn validate_rejects_unsupported_read_mode() {
        let profile = resolve_builtin_profile(BuiltInProfile::CommonSpiNor16MiB);
        let mut cfg = NorConfig::default();
        cfg.read_mode = ReadMode::Quad;

        assert_eq!(
            validate_and_resolve(profile, &cfg),
            Err(Error::UnsupportedProfileFeature)
        );
    }

    #[test]
    fn validate_rejects_missing_required_command() {
        static PROFILE: MissingWriteEnableProfile = MissingWriteEnableProfile;
        let cfg = NorConfig::default();

        assert_eq!(
            validate_and_resolve(&PROFILE, &cfg),
            Err(Error::InvalidConfiguration)
        );
    }

    #[test]
    fn validate_accepts_common_32mib_profile() {
        let profile = resolve_builtin_profile(BuiltInProfile::CommonSpiNor32MiB);
        let cfg = NorConfig::default();

        let resolved = validate_and_resolve(profile, &cfg).unwrap();
        assert_eq!(resolved.capacity_bytes, 32 * 1024 * 1024);
        assert_eq!(resolved.page_size, 256);
        assert_eq!(resolved.address_size, AddressSize::FourBytes);
        assert_eq!(resolved.commands.page_program_4byte, Some(0x12));
    }
}
