use std::collections::BTreeMap;
use std::fmt;

use serde::de::{self, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};

// ---------- pinmux_signals.yaml ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PinmuxSignals {
    pub hcpu: Vec<SignalDefinition>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SignalDefinition {
    pub name: String,
    #[serde(
        default = "default_peripheral_nomux",
        skip_serializing_if = "is_peripheral_nomux"
    )]
    pub r#type: SignalType,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub signals: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pin_trait: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, PartialOrd, Ord, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SignalType {
    Gpio,
    PeripheralNomux,
    PeripheralMux,
    Superimposed,
}

fn default_peripheral_nomux() -> SignalType {
    SignalType::PeripheralNomux
}

fn is_peripheral_nomux(signal_type: &SignalType) -> bool {
    *signal_type == SignalType::PeripheralNomux
}

// ---------- pinmux.yaml ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pinmux {
    pub hcpu: Vec<PinmuxPin>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PinmuxPin {
    pub pin: String,
    pub functions: Vec<PinFunction>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PinFunction {
    pub function: String,
    pub value: u8,
}

// ---------- adc.yaml ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Adc {
    pub hcpu: Vec<AdcPeripheral>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdcPeripheral {
    pub name: String,
    pub vbat_channel_id: u8,
    pub first_channel_pin: u8,
    pub vol_offset: u16,
    pub vol_ratio: u16,
    pub pins: Vec<String>,
}

// ---------- dma.yaml ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dma {
    /// HCPU DMA controllers (required, e.g., DMAC1)
    pub hcpu: DmaControllers,
    /// LCPU DMA controllers (optional, e.g., DMAC2)
    #[serde(default)]
    pub lcpu: Option<DmaControllers>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DmaControllers {
    #[serde(flatten)]
    pub controllers: BTreeMap<String, DmaController>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DmaController {
    pub channel_total: u8,
    pub request: Vec<DmaRequest>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DmaRequest {
    pub name: String,
    pub id: u8,
    pub module: Option<String>,
    #[serde(default = "default_true")]
    pub used: bool,
}

// ---------- mailbox.yaml ----------

/// Mailbox configuration: maps peripheral name (e.g., "MAILBOX1") to its config.
pub type Mailbox = BTreeMap<String, MailboxConfig>;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MailboxConfig {
    pub channel_total: u8,
}

// ---------- clocks.yaml ----------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ClockDomains {
    #[serde(default)]
    pub hpsys: Vec<ClockDomain>,
    #[serde(default)]
    pub lpsys: Vec<ClockDomain>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ClockDomain {
    pub name: String,
    pub token: String,
    /// Hardware read function for LPSYS clocks (bypasses cache).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub read_fn: Option<String>,
}

// ---------- HPSYS_xxx.yaml ----------

// Some code in this file is copied from [chiptool](https://github.com/embassy-rs/chiptool/blob/main/src/ir.rs)
// and is used under the MIT License with some simplifications and modifications.
// Since [chiptool](https://github.com/embassy-rs/chiptool/) is not published on
// [crates.io](https://crates.io), we cannot directly depend on it.
#[derive(Default, Clone, Debug, PartialEq)]
pub struct IR {
    pub blocks: BTreeMap<String, Block>,
    pub fieldsets: BTreeMap<String, FieldSet>,
    // pub enums: BTreeMap<String, Enum>,
}

impl IR {
    pub fn new() -> Self {
        Self::default()
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Block {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub items: Vec<BlockItem>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BlockItem {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub byte_offset: u32,
    #[serde(flatten)]
    pub inner: Register,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Register {
    #[serde(default = "default_readwrite", skip_serializing_if = "is_readwrite")]
    pub access: Access,
    #[serde(default = "default_32", skip_serializing_if = "is_32")]
    pub bit_size: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fieldset: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Access {
    ReadWrite,
    Read,
    Write,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldSet {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(default = "default_32", skip_serializing_if = "is_32")]
    pub bit_size: u32,
    pub fields: Vec<Field>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Field {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub bit_offset: u8,
    pub bit_size: u32,
}

// ---------- interrupts.yaml ----------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interrupts {
    pub hcpu: Vec<Interrupt>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interrupt {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub value: u32,
}

// ---------- peripherals.yaml ----------

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Peripherals {
    #[serde(default)]
    pub hcpu: Vec<Peripheral>,
    #[serde(default)]
    pub lcpu: Vec<Peripheral>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Peripheral {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// disable if a peripheral does not has ENR RSTR Field
    #[serde(default = "default_true", skip_serializing_if = "is_true")]
    pub enable_reset: bool,
    /// ignore missing enable reset field, impl `RccEnableReset` if enable or reset is none
    /// TODO: Is there a better way to handle this?
    #[serde(default = "default_false", skip_serializing_if = "is_false")]
    pub ignore_missing_enable_reset: bool,
    pub rcc_field: Option<String>,
    pub clock: Option<String>,
    // #[serde(
    //     default,
    //     skip_serializing_if = "BTreeMap::is_empty",
    //     serialize_with = "ordered_map"
    // )]
    // pub interrupts: BTreeMap<String, String>,
}

fn default_32() -> u32 {
    32
}

fn is_32(size: &u32) -> bool {
    *size == 32
}

fn default_true() -> bool {
    true
}

fn default_false() -> bool {
    false
}

fn is_true(b: &bool) -> bool {
    *b
}

fn is_false(b: &bool) -> bool {
    !*b
}

fn default_readwrite() -> Access {
    Access::ReadWrite
}

fn is_readwrite(access: &Access) -> bool {
    *access == Access::ReadWrite
}

struct IRVisitor;

impl<'de> Visitor<'de> for IRVisitor {
    type Value = IR;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("an IR")
    }

    fn visit_map<M>(self, mut access: M) -> Result<Self::Value, M::Error>
    where
        M: MapAccess<'de>,
    {
        let mut ir = IR::new();

        // While there are entries remaining in the input, add them
        // into our map.
        while let Some(key) = access.next_key()? {
            let key: String = key;
            let (kind, name) = key.split_once('/').ok_or(de::Error::custom("item names must be in form `kind/name`, where kind is `block`, `fieldset` or `enum`"))?;
            match kind {
                "block" => {
                    let val: Block = access.next_value()?;
                    if ir.blocks.insert(name.to_string(), val).is_some() {
                        return Err(de::Error::custom(format!("Duplicate item {:?}", key)));
                    }
                }
                "fieldset" => {
                    let val: FieldSet = access.next_value()?;
                    if ir.fieldsets.insert(name.to_string(), val).is_some() {
                        return Err(de::Error::custom(format!("Duplicate item {:?}", key)));
                    }
                }
                // "enum" => {
                //     let val: Enum = access.next_value()?;
                //     if ir.enums.insert(name.to_string(), val).is_some() {
                //         return Err(de::Error::custom(format!("Duplicate item {:?}", key)));
                //     }
                // }
                _ => return Err(de::Error::custom(format!("Unknown kind {:?}", kind))),
            }
        }

        Ok(ir)
    }
}

impl<'de> Deserialize<'de> for IR {
    fn deserialize<D>(deserializer: D) -> Result<IR, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_map(IRVisitor)
    }
}
