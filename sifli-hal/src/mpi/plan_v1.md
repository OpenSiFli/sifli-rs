# MPI Module Rewrite Plan (Clean, Simple, NOR-First)

## Summary
This rewrite replaces the current `mpi` module with a NOR-first architecture that has a small, explicit public API and a minimal low-level controller API. It keeps blocking and async drivers, enforces safe behavior in XIP scenarios, removes partition wrappers, and uses explicit flash profiles with rich per-command transfer metadata (opcode + lane/dummy/address/alternate/instruction mode), so behavior matches SiFli SDK command-table reality.

The API is intentionally simplified:
1. One high-level NOR type shape: `NorFlash<'d, T, M>` with `Blocking`/`Async` mode aliases.
2. Auto-detect constructors return `(driver, DetectedNorInfo)` instead of a separate “detected driver type”.
3. No public partition wrapper types in `mpi`.
4. Async is hard-forbidden when executing from the same MPI XIP window.

## Public API (Exact Design)

```rust
// sifli-hal/src/mpi.rs (facade)
pub mod nor;

pub use controller::{
    Mpi, MpiConfig, Error, LineMode, AddressBytes, TransferDirection,
    CommandSpec, TransferSpec, AhbReadSpec,
};

pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
    type Interrupt: interrupt::typelevel::Interrupt;
}
pub struct InterruptHandler<T: Instance> { /* typed IRQ handler */ }
```

```rust
// sifli-hal/src/mpi/nor/mod.rs
pub type BlockingNorFlash<'d, T> = NorFlash<'d, T, crate::mode::Blocking>;
pub type AsyncNorFlash<'d, T> = NorFlash<'d, T, crate::mode::Async>;

pub struct NorFlash<'d, T: Instance, M: crate::mode::Mode> { /* opaque */ }

pub struct NorConfig {
    pub read_mode: ReadMode,                      // Single | Quad | QuadDtr
    pub dma_threshold_bytes: usize,               // default 256
    pub max_ready_polls: u32,                     // default 2_000_000
    pub allow_preconfigured_4byte_in_xip: bool,   // default false
}
impl Default for NorConfig { /* explicit defaults */ }

pub struct JedecId { pub manufacturer: u8, pub memory_type: u8, pub density: u8 }

pub struct DetectedNorInfo {
    pub jedec: JedecId,
    pub profile_name: &'static str,
    pub family: NorFamily,
}
```

```rust
// constructors
impl<'d, T: Instance> BlockingNorFlash<'d, T> {
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        profile: &'static NorProfile,
        config: NorConfig,
    ) -> Result<Self, Error>;

    pub fn new_blocking_auto(
        peri: impl Peripheral<P = T> + 'd,
        config: NorConfig,
    ) -> Result<(Self, DetectedNorInfo), Error>;
}

impl<'d, T: Instance> AsyncNorFlash<'d, T> {
    pub fn new_async(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        profile: &'static NorProfile,
        config: NorConfig,
    ) -> Result<Self, Error>;

    pub fn new_async_auto(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: NorConfig,
    ) -> Result<(Self, DetectedNorInfo), Error>;
}
```

```rust
// common NOR ops
impl<'d, T: Instance, M: Mode> NorFlash<'d, T, M> {
    pub fn profile(&self) -> &'static NorProfile;
    pub fn detected_info(&self) -> Option<DetectedNorInfo>;
    pub fn capacity(&self) -> usize;
    pub fn read_jedec_id(&mut self) -> Result<JedecId, Error>;
    pub fn read_status(&mut self) -> Result<u8, Error>;
    pub fn read_sfdp(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error>;
    pub fn read_unique_id(&mut self, out: &mut [u8]) -> Result<(), Error>;
    pub fn enter_deep_power_down(&mut self) -> Result<(), Error>;
    pub fn release_deep_power_down(&mut self) -> Result<(), Error>;
}
```

```rust
// blocking data ops
impl<'d, T: Instance> BlockingNorFlash<'d, T> {
    pub fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error>;
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error>;
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    pub fn erase_chip(&mut self) -> Result<(), Error>;
}

// async data ops
impl<'d, T: Instance> AsyncNorFlash<'d, T> {
    pub async fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error>;
    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error>;
    pub async fn erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    pub async fn erase_chip(&mut self) -> Result<(), Error>;
}
```

```rust
// trait impls (explicit and fixed for supported NOR families)
impl embedded_storage::nor_flash::ReadNorFlash for BlockingNorFlash<'_, _> {
    const READ_SIZE: usize = 1;
}
impl embedded_storage::nor_flash::NorFlash for BlockingNorFlash<'_, _> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;
}
impl embedded_storage_async::nor_flash::ReadNorFlash for AsyncNorFlash<'_, _> {
    const READ_SIZE: usize = 1;
}
impl embedded_storage_async::nor_flash::NorFlash for AsyncNorFlash<'_, _> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;
}
```

## Flash Profile Model (Explicit and Complete)

```rust
pub struct NorProfile {
    pub name: &'static str,
    pub family: NorFamily,
    pub capacity_bytes: usize,
    pub page_size: usize,
    pub write_size: usize, // must be 1 for v1 high-level driver
    pub erase_size: usize, // must be 4096 for v1 high-level driver

    pub commands: NorCommands,       // rich per-command transfer metadata
    pub qe: QePolicy,                // QE bit location + write/read sequence
    pub addressing: AddressingPolicy,// 3B/4B opcode-mode policy
    pub dtr: DtrPolicy,              // DTR capability + command
    pub ahb_read: AhbReadPolicy,     // memory-mapped read command selection
}
```

`NorCommands` stores `CommandSpec` for all required operations (wren/rdsr/read/program/erase/chip erase/id/sfdp/uid/reset/power) and optional 4-byte variants, 32K/64K erase variants, status2/status3 variants.  
`CommandSpec` carries both opcode and full transfer parameters (instruction/address/data lane mode, dummy cycles, alternate mode/size, address size, instruction mode including DDR mode), matching the SDK command-table fidelity.

## File and Module Structure (Explicit)

```text
sifli-hal/src/mpi.rs
sifli-hal/src/mpi/controller.rs
sifli-hal/src/mpi/xip.rs
sifli-hal/src/mpi/nor/mod.rs
sifli-hal/src/mpi/nor/profile.rs
sifli-hal/src/mpi/nor/detect.rs
sifli-hal/src/mpi/nor/blocking.rs
sifli-hal/src/mpi/nor/async.rs
```

Module responsibilities:
1. `mpi.rs`: facade exports, `Instance`/`InterruptHandler`, instance impls for `MPI1`/`MPI2`.
2. `controller.rs`: small raw MPI controller API, typed command/transfer configs, AHB read configuration.
3. `xip.rs`: code-bus window detection, RAM-resident manual-command helpers, IRQ-mask wrappers.
4. `nor/profile.rs`: `NorProfile`, command/policy model, built-in core family profiles.
5. `nor/detect.rs`: JEDEC read + lookup table -> `(profile, DetectedNorInfo)`.
6. `nor/blocking.rs`: blocking NOR ops, XIP-safe backend routing, cache invalidation.
7. `nor/async.rs`: async NOR ops with DMA + typed IRQ binding + waker state.
8. `nor/mod.rs`: public NOR facade, constructor wiring, trait impl glue.

## Behavioral Rules (Decision Complete)

1. XIP safety:
`Blocking` paths always choose XIP-safe RAM wrappers when running from the same instance window; this includes manual read/status/ID/SFDP/UID/reset/power commands, not only mutating ops.
2. Async safety:
All async methods return `Error::AsyncNotAllowedInSameXip` when execution is from the same MPI window; check is performed at constructor and operation entry.
3. Async execution model:
Data movement is DMA-based for async read/write; long-latency completion is IRQ/waker driven (no spin-poll loops in async paths).
4. Interrupt typing:
`Instance` includes `type Interrupt`; async constructors require `Binding<T::Interrupt, InterruptHandler<T>>`.
5. Addressing policy:
Profile defines explicit 3-byte vs 4-byte behavior: dedicated 4-byte opcodes and/or enter-4-byte-mode command sequence; same-XIP + required mode switch is rejected unless `allow_preconfigured_4byte_in_xip` is true.
6. QE policy:
Profile defines exact QE enable sequence and bit location (including WRSR vs WRSR2 semantics).
7. DTR policy:
Profile explicitly declares support and command metadata; DTR is only enabled when profile supports it and `NorConfig::read_mode` requests it.
8. Trait geometry:
High-level NOR trait impl is fixed to `WRITE_SIZE=1` and `ERASE_SIZE=4096`; constructor validates profile geometry and rejects unsupported geometry in v1.
9. Scope:
NOR only in v1; no partition wrapper types in `mpi`; no NAND/PSRAM/AES legacy APIs in this rewrite.

## DMA and Codegen Changes

1. Update [dma.yaml](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/data/sf32lb52x/dma.yaml): set `mpi1` and `mpi2` requests to `used: true`.
2. Keep module as `mpi` so generated `dma_trait_impl!` binds `crate::mpi::Dma<crate::peripherals::MPI1/MPI2, ...>`.
3. In `mpi::nor`, define `dma_trait!(Dma, Instance)` and use `new_dma!` in async constructors.

## Migration Contract

1. Removed: `MpiNorFlash`, `MpiNorPartition`, `FlashPartition`, `NorFlashPartition`.
2. New canonical type aliases: `BlockingNorFlash<'d, T>` and `AsyncNorFlash<'d, T>`.
3. Old `NorFlashCommandSet`/old `NorFlashConfig` replaced by `NorProfile` + `NorConfig`.
4. Auto-detect now returns `(driver, DetectedNorInfo)` and never a separate “detected driver” wrapper type.
5. Examples updated to new constructors and no partition abstraction.

## Tests and Acceptance Criteria

1. Unit tests for profile validation:
reject missing mandatory commands, invalid geometry, inconsistent 4-byte policy.
2. Unit tests for command selection:
3-byte/4-byte opcode selection, QE enable sequence, DTR path selection, AHB read command selection.
3. Unit tests for erase planner:
64K/32K/4K mixed strategy with alignment and bounds checks.
4. Unit tests for detection:
JEDEC lookup hit/miss and metadata correctness.
5. Unit tests for XIP guards:
same-instance detection, blocking backend selection, async rejection behavior.
6. Compile checks:
`BlockingNorFlash` implements `embedded-storage`; `AsyncNorFlash` implements `embedded-storage-async`; async constructor requires typed IRQ binding.
7. Hardware example validation:
rewrite `examples/sf32lb52x/src/bin/mpi_nor_flash.rs` to run read/erase/write/verify on blocking path; add async example for non-XIP instance; verify async constructor fails on same-XIP instance.

Acceptance is complete when:
1. Old MPI NOR public APIs are removed and new APIs compile cleanly.
2. Blocking behavior is safe from same-XIP.
3. Async behavior is DMA-backed and denied on same-XIP.
4. JEDEC auto-detect produces deterministic profile selection for built-in core-family table.
5. Example(s) and tests pass.

## Assumptions and Defaults

1. Breaking API change is allowed for this module.
2. v1 supports NOR high-level geometry `{WRITE_SIZE=1, ERASE_SIZE=4096}`.
3. Auto-detection is JEDEC-only in v1 (no SFDP-first policy engine).
4. Built-in profile coverage is the SiFli core NOR families (type0..type5 style).
5. Partition abstractions are out of scope and intentionally removed from `mpi`.

## Implementation References

1. SiFli SDK command model: [bf0_hal_mpi.h](/Users/haobogu/Projects/embedded/SiFli-SDK/drivers/Include/bf0_hal_mpi.h)
2. SiFli SDK command tables: [flash_table.c](/Users/haobogu/Projects/embedded/SiFli-SDK/drivers/hal/flash_table.c)
3. SiFli SDK extended NOR flows (QE/4-byte/DTR): [bf0_hal_mpi_ex.c](/Users/haobogu/Projects/embedded/SiFli-SDK/drivers/hal/bf0_hal_mpi_ex.c)
4. SiFli SDK AHB/manual command setup: [bf0_hal_mpi.c](/Users/haobogu/Projects/embedded/SiFli-SDK/drivers/hal/bf0_hal_mpi.c)
5. Embassy transfer-shape reference (typed command config): [qspi/mod.rs](/Users/haobogu/Projects/rust/tmp/embassy/embassy-stm32/src/qspi/mod.rs)
6. Embassy richer transfer-shape reference (alt/address/dtr): [ospi/mod.rs](/Users/haobogu/Projects/rust/tmp/embassy/embassy-stm32/src/ospi/mod.rs)
