# MPI Rewrite Plan (Breaking, NOR-First, Simple API)

## 1. Goals
1. Deliver a clean, small, user-friendly `mpi` module centered on NOR flash.
2. Keep one obvious high-level API path for normal usage and one constrained advanced path.
3. Enforce XIP safety by default, with explicit errors for unsupported contexts.
4. Provide blocking and async NOR APIs with consistent behavior and error semantics.
5. Avoid awkward generic-heavy user APIs.

## 2. Scope
1. In scope: MPI controller lifecycle, NOR blocking + async, JEDEC auto-detect, built-in core profile families, custom profile trait, XIP-safe execution.
2. Out of scope: NAND, PSRAM, AES/alias/nonce public APIs, partition wrappers, compatibility shims.

## 3. Core API Design Decisions
1. Use a single primary flash type: `NorFlash<'d, T, M>`.
2. Remove profile generic from public type to avoid `NorFlash<'d, T, M, P>` complexity.
3. Use `ProfileSource` at construction time (`BuiltIn` or `Custom`), then store resolved profile internally.
4. Auto-detect constructors return `NorFlash` directly (no `DetectedNorFlash` enum in public API).
5. Keep an advanced typed transfer entrypoint on `Mpi` for manual commands, but remove register-style mutators.

## 4. Public API (Explicit)
```rust
pub struct Mpi<'d, T: Instance> { /* slim controller handle */ }

pub trait Instance: crate::SealedInstance {
    type Interrupt: interrupt::typelevel::Interrupt;
    // internal async state hooks
}

pub enum ProfileSource {
    BuiltIn(BuiltInProfile),
    Custom(&'static dyn NorProfile),
}

pub struct NorFlash<'d, T: Instance, M: mode::Mode> { /* ... */ }

impl<'d, T: Instance> Mpi<'d, T> {
    pub fn new(peri: impl Peripheral<P = T> + 'd, cfg: MpiConfig) -> Result<Self, Error>;

    pub fn into_nor_blocking(
        self,
        profile: ProfileSource,
        cfg: NorFlashConfig,
    ) -> Result<NorFlash<'d, T, mode::Blocking>, Error>;

    pub fn into_nor_async(
        self,
        profile: ProfileSource,
        cfg: NorFlashConfig,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<NorFlash<'d, T, mode::Async>, Error>;

    pub fn into_nor_auto_detect_blocking(
        self,
        cfg: NorFlashConfig,
    ) -> Result<NorFlash<'d, T, mode::Blocking>, Error>;

    pub fn into_nor_auto_detect_async(
        self,
        cfg: NorFlashConfig,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Result<NorFlash<'d, T, mode::Async>, Error>;

    pub fn transfer(&mut self, t: &mut Transfer<'_>) -> Result<(), Error>;
}
```

```rust
impl<'d, T: Instance, M: mode::Mode> NorFlash<'d, T, M> {
    pub fn read_jedec_id(&mut self) -> Result<JedecId, Error>;
    pub fn read_status(&mut self, reg: StatusReg) -> Result<u8, Error>;
    pub fn read_sfdp(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Error>;
    pub fn read_unique_id(&mut self, buf: &mut [u8]) -> Result<(), Error>;
    pub fn enter_deep_power_down(&mut self) -> Result<(), Error>;
    pub fn release_deep_power_down(&mut self) -> Result<(), Error>;
    pub fn reset_device(&mut self) -> Result<(), Error>;
    pub fn profile_info(&self) -> ProfileInfo;
    pub fn free(self) -> Mpi<'d, T>;
}
```

1. Blocking mode implements `embedded_storage::nor_flash::{ErrorType, ReadNorFlash, NorFlash}`.
2. Async mode implements `embedded_storage_async::nor_flash::{ErrorType, ReadNorFlash, NorFlash}`.
3. Public trait constants are fixed and predictable: `READ_SIZE=1`, `WRITE_SIZE=1`, `ERASE_SIZE=4096`.
4. Constructor validates profile geometry/policies; unsupported geometry returns `Error::UnsupportedProfileFeature`.

## 5. Profile Model
1. `NorProfile` is rich enough to represent SDK command-table semantics.
2. `CommandDescriptor` includes opcode and full transfer shape: instruction/address/alternate/data widths, dummy cycles, address bytes, mode bits.
3. `AhbReadDescriptor` models memory-mapped read path fields.
4. `QePolicy`, `AddressingPolicy`, and `DtrPolicy` are explicit enums with deterministic init/transition flow.
5. Built-in profiles are core families (`type0..type5` semantics), plus user `Custom`.

## 6. Async + DMA Architecture
1. Add `dma_trait!(Dma, Instance)` under `mpi`.
2. Mark `mpi1` and `mpi2` DMA requests as `used: true` in DMA metadata.
3. Add async `State { waker: AtomicWaker }` and `InterruptHandler<T>`.
4. Use DMA for FIFO data phases and MPI IRQ for command/status completion.
5. Async constructors require both DMA channel and typed IRQ binding.

## 7. XIP Safety Policy
1. Blocking operations are allowed in same-instance XIP and always use RAM-safe critical routines.
2. Async is forbidden in same-instance XIP at constructor time for all operations.
3. Same-instance async constructors return `Error::AsyncForbiddenInXip`.
4. 4-byte transitions follow explicit addressing policy and fail with explicit errors if unsafe.
5. Keep no-reset-safe default behavior unless explicitly configured safe.

## 8. Error Model
1. Consolidate under strict `Error` enum: `AsyncForbiddenInXip`, `UnknownJedecId`, `UnsupportedProfileFeature`, `InvalidConfig`, `Timeout`, `OutOfBounds`, `NotAligned`, plus transport/internal variants.
2. Embedded-storage error mapping remains deterministic: `NotAligned`, `OutOfBounds`, else `Other`.

## 9. File and Module Structure
1. [sifli-hal/src/mpi.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi.rs): public exports, `Mpi`, `Instance`, configs, errors, traits.
2. [sifli-hal/src/mpi/controller.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/controller.rs): typed command execution and low-level controller logic.
3. [sifli-hal/src/mpi/xip.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/xip.rs): XIP window checks, RAM-safe critical sections, IRQ save/restore helpers.
4. [sifli-hal/src/mpi/nor/mod.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor/mod.rs): `NorFlash` facade and shared helpers.
5. [sifli-hal/src/mpi/nor/blocking.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor/blocking.rs): blocking NOR implementation and embedded-storage blocking traits.
6. [sifli-hal/src/mpi/nor/async.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor/async.rs): async NOR implementation, DMA+IRQ flow, async traits.
7. [sifli-hal/src/mpi/nor/profile.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor/profile.rs): `NorProfile`, descriptors, policies, built-ins.
8. [sifli-hal/src/mpi/nor/detect.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor/detect.rs): JEDEC map and detector.
9. Remove or repurpose old files after cutover: [sifli-hal/src/mpi/driver.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/driver.rs), [sifli-hal/src/mpi/nor.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/nor.rs), [sifli-hal/src/mpi/shared.rs](/Users/haobogu/Projects/rust/sifli-rs/sifli-hal/src/mpi/shared.rs).

## 10. Migration
1. Replace `MpiNorFlash` with `NorFlash`.
2. Replace old constructors with `into_nor_blocking/async` or `into_nor_auto_detect_blocking/async`.
3. Remove partition API usage and aliases.
4. Replace direct register-style calls with `Mpi::transfer`.
5. Update example: [examples/sf32lb52x/src/bin/mpi_nor_flash.rs](/Users/haobogu/Projects/rust/sifli-rs/examples/sf32lb52x/src/bin/mpi_nor_flash.rs).
6. Add migration guide: [docs/mpi_rewrite_migration.md](/Users/haobogu/Projects/rust/sifli-rs/docs/mpi_rewrite_migration.md).

## 11. Validation Plan
1. Unit tests: JEDEC mapping, unknown ID, descriptor-to-register translation, QE/addressing/DTR policy transitions, erase planner, bounds/alignment, error-kind mapping, XIP gating.
2. Compile checks: default target feature, `no_std`, async traits, DMA trait impl generation for MPI requests.
3. Hardware tests: blocking read/write/erase/JEDEC/status, same-XIP blocking flows, async DMA read/write/erase off-XIP, async constructor rejection in same-XIP.

## 12. Implementation Sequence
1. Introduce new `Instance` interrupt typing and async state scaffolding.
2. Build typed controller transfer API in `controller.rs`.
3. Move RAM-safe/XIP logic to `xip.rs`.
4. Implement profile descriptors/policies and built-in families.
5. Implement blocking `NorFlash`.
6. Implement JEDEC detection and auto-detect constructors returning `NorFlash`.
7. Wire DMA metadata + async path + IRQ handler.
8. Enforce global async same-XIP rejection.
9. Remove old APIs/files and fix exports.
10. Update example, add migration doc, run compile + test matrix.
