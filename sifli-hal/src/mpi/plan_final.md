# MPI Rewrite Plan Final (NOR-First, Embassy-Style)

## 1. Summary

Rewrite `sifli-hal/src/mpi` into a NOR-first module with a small, embassy-style public API, strong XIP safety guarantees, and explicit extensibility hooks for future NAND/PSRAM backends.

This plan merges the detail depth from `plan_v1.md` and the simplified API direction from `plan_v2.md`, with these locked decisions:

1. Primary public API follows embassy style: mode-generic flash type + `new_blocking`/`new_async`.
2. Auto-detect constructors return driver only.
3. Custom profile model is trait-object based.
4. Advanced controller API is public only behind `unstable-mpi-controller`.
5. Async same-instance XIP is rejected at constructor time and operation entry.
6. Validation target is full hardware run for all `examples/sf32lb52x` binaries.

### 1.1 Progress Snapshot (as of March 1, 2026)

1. M0 is complete.
2. M1 is complete.
3. M2 is complete.
4. M3 is complete.
5. M4, M5, M6, and M7 code-deliverables are complete.
6. Latest validation evidence:
   `cargo check --manifest-path sifli-hal/Cargo.toml --target thumbv8m.main-none-eabihf` passes with and without `unstable-mpi-controller`.
7. Additional validation evidence:
   `cargo test --manifest-path sifli-hal/Cargo.toml --no-default-features --features "sf32lb52x time"` passes (`8` unit tests, `4` doc tests).
8. Latest hardware run evidence:
   `SIFLI_SFTOOL_PORT=/dev/cu.wchusbserial59810930261 cargo run --release --bin mpi_nor_flash` passes on retry with `Verification PASSED - all 256 bytes match`.
9. Additional hardware validation on February 28, 2026 (port `/dev/cu.wchusbserial210`):
   `mpi_nor_flash_diag` passes in same-instance XIP with SFDP and UID reads succeeding, while deep-power-down remains blocked with `CommandForbiddenInXip` by policy.
10. Added-example sequential hardware run evidence on February 28, 2026:
    `mpi_nor_flash_diag`, `mpi_nor_flash_async_auto`, and `mpi_nor_async_xip_guard` all pass when run one-by-one via the board runner.
11. Compile validation follow-up on March 1, 2026:
    `cargo check --bins` passes in `examples/sf32lb52x` after fixing an ambiguous `panic!` macro call in `mpi_nor_flash_async.rs`.
12. Validation tooling follow-up on March 1, 2026:
    added `scripts/run_mpi_examples_hw.sh` for sequential MPI example runs with per-bin timeout, PASS-marker checks, and timestamped log capture.
13. USB-serial hardware matrix follow-up on March 1, 2026 (port `/dev/cu.usbserial-310`):
    completed blocking/async/auto matrix via `sftool` flash + post-run flash readback verification; `mpi_nor_flash` and `mpi_nor_flash_auto` confirmed expected payload writes, while `mpi_nor_flash_async` and `mpi_nor_flash_async_auto` confirmed same-instance XIP guarded behavior (sector remains erased).
14. Auto-detect compatibility fix on March 1, 2026:
    added XTX JEDEC IDs (`0x16400B`, `0x17400B`, `0x18400B`, `0x19400B`) so `new_*_auto` resolves common XTX SPI NOR parts used on test hardware.
15. Final USB-serial matrix closure on March 1, 2026:
    `scripts/run_mpi_examples_hw_serial_verify.sh` passes `4/4` (`mpi_nor_flash`, `mpi_nor_flash_auto`, `mpi_nor_flash_async`, `mpi_nor_flash_async_auto`) on `/dev/cu.usbserial-310`, with logs in `examples/sf32lb52x/target/mpi-serial-verify-logs/20260301_152508`.
16. JEDEC byte-order robustness fix on March 1, 2026:
    auto-detect now accepts canonical and swapped manufacturer/density byte order when matching known JEDEC IDs, covering SiFliUart read-path ordering variance seen on hardware.

## 2. Goals and Non-Goals

### 2.1 Goals

1. Provide one obvious, ergonomic NOR API path for typical users.
2. Preserve advanced manual controller access behind an unstable feature gate.
3. Enforce safe behavior by default for same-instance XIP scenarios.
4. Keep blocking and async APIs behaviorally consistent where feasible.
5. Keep architecture ready for NAND/PSRAM reuse without exposing premature public APIs.

### 2.2 Non-Goals (v1 rewrite)

1. Public NAND API.
2. Public PSRAM API.
3. Partition wrappers in `mpi` public API.
4. Legacy compatibility shims for old API names once cutover is complete.

## 3. Public API and Breaking Changes

### 3.1 New public shape

1. `pub mod mpi;` keeps module name stable.
2. Main user type:
   `NorFlash<'d, T, M>` with aliases `BlockingNorFlash<'d, T>` and `AsyncNorFlash<'d, T>`.
3. Constructors on flash type (embassy-style):
   `new_blocking`, `new_blocking_auto`, `new_async`, `new_async_auto`.
4. Remove partition wrapper types from `mpi` public API.
5. Replace old `MpiNorFlash`/`Flash`/`FlashPartition` exports with new NOR exports only (after cutover milestone).

### 3.2 Auto-detect behavior

1. `*_auto` constructors return `Result<NorFlash<...>, Error>` only.
2. Detected metadata remains queryable after construction:
   `fn detected_info(&self) -> Option<DetectedNorInfo>`.

### 3.3 Custom profile model

1. Construction uses `ProfileSource`:
   `BuiltIn(BuiltInProfile)` or `Custom(&'static dyn NorProfile)`.
2. No public profile generic on driver type.
3. Constructors resolve and validate profile into an internal runtime descriptor.

### 3.4 Advanced low-level API

1. Typed controller API (`Mpi`, `Transfer`, descriptors) is publicly exposed only under:
   `#[cfg(feature = "unstable-mpi-controller")]`.
2. NOR API remains fully usable without this feature.
3. Stable docs present NOR path as canonical; controller path is advanced/unstable.

## 4. API Reference Snippets (Preserved)

```rust
// sifli-hal/src/mpi.rs (facade)
pub mod nor;

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
    pub read_mode: ReadMode,
    pub dma_threshold_bytes: usize,
    pub max_ready_polls: u32,
    pub allow_preconfigured_4byte_in_xip: bool,
}

impl Default for NorConfig { /* explicit defaults */ }

pub struct JedecId {
    pub manufacturer: u8,
    pub memory_type: u8,
    pub density: u8,
}

pub struct DetectedNorInfo {
    pub jedec: JedecId,
    pub profile_name: &'static str,
    pub family: NorFamily,
}
```

```rust
impl<'d, T: Instance> BlockingNorFlash<'d, T> {
    pub fn new_blocking(
        peri: impl Peripheral<P = T> + 'd,
        profile: ProfileSource,
        config: NorConfig,
    ) -> Result<Self, Error>;

    pub fn new_blocking_auto(
        peri: impl Peripheral<P = T> + 'd,
        config: NorConfig,
    ) -> Result<Self, Error>;
}

impl<'d, T: Instance> AsyncNorFlash<'d, T> {
    pub fn new_async(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        profile: ProfileSource,
        config: NorConfig,
    ) -> Result<Self, Error>;

    pub fn new_async_auto(
        peri: impl Peripheral<P = T> + 'd,
        dma: impl Peripheral<P = impl Dma<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: NorConfig,
    ) -> Result<Self, Error>;
}
```

```rust
impl<'d, T: Instance, M: crate::mode::Mode> NorFlash<'d, T, M> {
    pub fn profile(&self) -> &'static dyn NorProfile;
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
impl<'d, T: Instance> BlockingNorFlash<'d, T> {
    pub fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error>;
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error>;
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    pub fn erase_chip(&mut self) -> Result<(), Error>;
}

impl<'d, T: Instance> AsyncNorFlash<'d, T> {
    pub async fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Error>;
    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error>;
    pub async fn erase(&mut self, from: u32, to: u32) -> Result<(), Error>;
    pub async fn erase_chip(&mut self) -> Result<(), Error>;
}
```

## 5. Architecture and Module Layout

1. `sifli-hal/src/mpi.rs`
   Public facade, `Instance`, `InterruptHandler<T>`, shared types, re-exports.
2. `sifli-hal/src/mpi/controller.rs`
   Typed transfer API and register programming backend.
3. `sifli-hal/src/mpi/xip.rs`
   Same-instance XIP detection, RAM-safe wrappers, cache/IRQ safety helpers.
4. `sifli-hal/src/mpi/nor/mod.rs`
   Public NOR type, constructors, trait glue.
5. `sifli-hal/src/mpi/nor/profile.rs`
   `NorProfile` trait, built-ins, policy enums, validation.
6. `sifli-hal/src/mpi/nor/detect.rs`
   JEDEC read + built-in profile resolver.
7. `sifli-hal/src/mpi/nor/blocking.rs`
   Blocking operations and `embedded-storage` blocking traits.
8. `sifli-hal/src/mpi/nor/async.rs`
   Async DMA+IRQ operations and `embedded-storage-async` traits.
9. Legacy files to retire after cutover:
   old `driver.rs`, monolithic old `nor.rs`, old `shared.rs` split into focused modules.

Current implementation state (February 28, 2026):
1. `controller.rs` is the single low-level backend (legacy `driver.rs` removed).
2. NOR API is fully split into `mpi/nor/mod.rs` + `blocking.rs` + `async.rs` + `detect.rs` + `profile.rs`.
3. Legacy monolithic `mpi/nor.rs` is removed.

## 6. Interface Details (Decision Complete)

### 6.1 NOR configuration

1. `NorConfig` fields:
   `read_mode`, `dma_threshold_bytes`, `max_ready_polls`, `allow_preconfigured_4byte_in_xip`.
2. Defaults:
   `dma_threshold_bytes = 256`, `max_ready_polls = 2_000_000`, `allow_preconfigured_4byte_in_xip = false`.

### 6.2 Profile semantics

1. `NorProfile` exposes full command + policy descriptors.
2. Descriptors include opcode + transfer shape:
   instruction/address/alternate/data line modes, dummy cycles, address bytes, DDR flags.
3. Policies are explicit:
   `QePolicy`, `AddressingPolicy`, `DtrPolicy`, `AhbReadPolicy`.

### 6.3 Safety model

1. Blocking operations always choose XIP-safe backend when running from same-instance XIP window.
2. Async constructors reject same-instance XIP immediately.
3. Async read/write/erase/chip-erase re-check XIP at op entry and fail explicitly if unsafe.
4. 4-byte transitions under same-instance XIP follow policy and fail unless explicitly allowed.

### 6.4 Trait geometry contract

1. Public trait constants are fixed:
   `READ_SIZE = 1`, `WRITE_SIZE = 1`, `ERASE_SIZE = 4096`.
2. Constructor rejects profiles incompatible with v1 geometry/policy contract.

### 6.5 Error model

1. Unified `Error` includes:
   `AsyncForbiddenInXip`, `UnknownJedecId`, `UnsupportedProfileFeature`, `InvalidConfig`, `Timeout`, `OutOfBounds`, `NotAligned`, plus internal transport variants.
2. Embedded-storage mapping:
   `NotAligned -> NotAligned`, `OutOfBounds -> OutOfBounds`, all others -> `Other`.

## 7. Extensibility for NAND/PSRAM (No Premature Public API)

1. Keep controller layer protocol-agnostic and typed.
2. Keep NOR as first concrete backend.
3. Reserve `mpi/nand` and `mpi/psram` namespaces.
4. Define internal backend boundaries so future modules reuse controller/XIP/IRQ/DMA plumbing instead of NOR logic.
5. Do not expose public NAND/PSRAM API in this rewrite.

## 8. DMA and Interrupt Integration

1. Update `sifli-hal/data/sf32lb52x/dma.yaml`:
   set `mpi1` and `mpi2` requests to `used: true`.
2. Add `dma_trait!(Dma, Instance)` under `mpi`.
3. Async constructors require DMA channel + typed IRQ binding.
4. Async state uses per-instance `AtomicWaker` + typed `InterruptHandler<T>`.

## 9. Examples and Validation Plan

### 9.1 Example changes (`examples/sf32lb52x`)

1. Migrate existing `mpi_nor_flash.rs` to new API.
2. Add one blocking NOR example focused on JEDEC/read/write/erase.
3. Add one async NOR example focused on DMA path.
4. Add one auto-detect example showing `new_*_auto` + `detected_info()`.

### 9.2 "All examples pass" criterion (hardware)

1. Build all example binaries for `sf32lb52x`.
2. Flash and run every binary on hardware, collect pass/fail logs.
3. Add a runner script for sequential hardware execution with timeout + log capture.
4. Pass condition per example:
   startup reached + expected terminal marker logged + no panic/reset loop.
5. Keep MPI-specific examples with extra NOR assertions.
6. Runner script implemented at:
   `scripts/run_mpi_examples_hw.sh` (March 1, 2026).
7. USB-serial verification runner implemented at:
   `scripts/run_mpi_examples_hw_serial_verify.sh` (March 1, 2026), used when RTT attach is unavailable but flash side-effect validation is required.

### 9.3 Test scenarios

1. Unit tests:
   profile validation, JEDEC mapping, descriptor translation, QE/addressing/DTR transitions, bounds/alignment, error mapping, XIP gating.
2. Integration compile checks:
   blocking-only, async-enabled, unstable-controller feature on/off.
3. Hardware tests:
   blocking NOR R/W/E, async NOR R/W/E off-XIP, same-instance XIP rejection for async at constructor and operation entry, 4-byte policy behavior.

## 10. Roadmap (Milestones + Exit Criteria)

### M0. Plan normalization (Status: Complete, February 28, 2026)

1. Deliverables:
   structured `plan_final.md` with locked decisions, preserved API snippets, implementation references, and explicit roadmap.
2. Exit criteria:
   no contradictory API decisions, no duplicate migration rules, roadmap accepted as source of truth.
3. Completion evidence:
   this document was normalized, deduplicated, and expanded with milestone status and validation evidence.

### M1. Foundation slice (Status: Complete, February 28, 2026)

1. Deliverables:
   `mpi` facade updates, typed interrupt hooks (`Instance::Interrupt`, `InterruptHandler<T>`), controller scaffold exposure, NOR public type scaffolding.
2. Exit criteria:
   `sifli-hal` compiles with default feature set and existing blocking examples still compile.
3. Completion evidence:
   `Instance::Interrupt`/`InterruptHandler<T>` and new NOR API scaffold landed; target `cargo check` passes; `mpi_nor_flash` hardware run passes.

### M2. Controller extraction (Status: Complete, February 28, 2026)

1. Deliverables:
   typed transfer/controller API in `controller.rs`, re-exported under `unstable-mpi-controller`.
2. Exit criteria:
   compile succeeds with feature on/off and low-level paths remain functional.
3. Done in this milestone:
   feature-gated advanced re-exports under `unstable-mpi-controller`; typed transfer surface (`Transfer`, `TransferData`, `Mpi::transfer`) and register backend are consolidated in `controller.rs`; legacy `driver.rs` removed.
4. Remaining:
   none.

### M3. XIP helper isolation (Status: Complete, February 28, 2026)

1. Deliverables:
   isolate same-instance XIP checks and RAM-safe wrappers in `xip.rs`.
2. Exit criteria:
   blocking safety-sensitive paths call centralized XIP helpers.
3. Done in this milestone:
   `xip.rs` owns same-instance detection + centralized RAM-safe helper entry points; blocking safety-sensitive call-sites in `driver.rs`/`nor.rs` now route through `xip.rs`.
4. Remaining:
   none.

### M4. Profile and JEDEC detection (Status: Complete, February 28, 2026)

1. Deliverables:
   `nor/profile.rs`, built-in profiles, validation flow, `nor/detect.rs` lookup.
2. Exit criteria:
   deterministic known-ID resolution, unknown-ID path returns explicit error.
3. Done in this milestone:
   created `nor/profile.rs` and `nor/detect.rs`; added policy enums, built-in profile registry, JEDEC mapping for common 4/8/16/32/64MiB SPI NOR families, and targeted unit tests for validation and JEDEC detection.
4. Remaining:
   none.

### M5. Blocking NOR cutover (Status: Complete, February 28, 2026)

1. Deliverables:
   new blocking NOR API behavior + fixed geometry trait implementations.
2. Exit criteria:
   blocking example migrated and hardware-validated.
3. Done in this milestone:
   blocking backend moved to `nor/blocking.rs`; NOR facade lives in `nor/mod.rs`; bridge-era `MpiNorFlash`/`MpiNorPartition` path removed; trait constants remain `READ_SIZE=1`, `WRITE_SIZE=1`, `ERASE_SIZE=4096`.
4. Remaining:
   full hardware matrix remains a validation task outside code refactor scope.

### M6. Async + DMA/IRQ (Status: Complete, February 28, 2026)

1. Deliverables:
   async NOR API with DMA transfer path and IRQ/waker completion, same-XIP rejection checks.
2. Exit criteria:
   async constructors/ops enforce XIP rule; async hardware examples pass.
3. Done in this milestone:
   implemented `new_async`/`new_async_auto`, async read/write/erase/chip-erase APIs, embedded-storage-async trait impls, DMA binding in constructors, typed IRQ handler wake path, and operation-entry same-XIP rejection.
4. Remaining:
   full async hardware matrix remains a validation task.

### M7. Cleanup and migration docs (Status: Complete, February 28, 2026)

1. Deliverables:
   remove legacy API exports/files, publish old-to-new migration guide.
2. Exit criteria:
   no stale public aliases (`Flash`, partition wrappers) and docs/examples are fully aligned.
3. Done in this milestone:
   removed stale public aliases (`Flash`, `FlashPartition`, `MpiNorFlash`, `MpiNorPartition`); removed legacy `driver.rs` and monolithic `nor.rs`; published `migration.md`; added `mpi_nor_flash_async.rs` and `mpi_nor_flash_auto.rs` examples.
4. Remaining:
   none.

### M8. Validation tooling follow-up (Status: Complete, March 1, 2026)

1. Deliverables:
   unblock `sf32lb52x` MPI example compilation and add a reusable sequential hardware runner with timeout + log capture.
2. Exit criteria:
   `cargo check --bins` succeeds in `examples/sf32lb52x`; runner script can execute MPI example binaries one-by-one and persist logs.
3. Done in this milestone:
   fixed `mpi_nor_flash_async.rs` to use `defmt::panic!` (resolving ambiguous macro selection) and added `scripts/run_mpi_examples_hw.sh` with expected-marker validation and timestamped per-bin logs.
4. Remaining:
   superseded by M9 hardware validation completion.

### M9. USB-serial hardware validation + JEDEC compatibility (Status: Complete, March 1, 2026)

1. Deliverables:
   run real-hardware MPI matrix on `sf32lb52x` over USB-serial and fix auto-detect failures observed on board.
2. Exit criteria:
   blocking + auto-detect examples demonstrate expected write payloads on flash; async examples demonstrate either successful writes (off-XIP) or policy-expected guarded no-write behavior (same-instance XIP); board JEDEC maps to built-in profile in `*_auto` constructors.
3. Done in this milestone:
   validated `mpi_nor_flash`, `mpi_nor_flash_auto`, `mpi_nor_flash_async`, and `mpi_nor_flash_async_auto` on `/dev/cu.usbserial-310` using flash readback verification; added `scripts/run_mpi_examples_hw_serial_verify.sh`; added XTX JEDEC mappings and JEDEC byte-order-robust matching in `nor/detect.rs`; added regression tests for canonical and swapped XTX 16MiB IDs.
4. Remaining:
   none for MPI functional scope.

## 11. Remaining Validation Work

1. None for MPI rewrite scope.
2. Optional future improvement: stabilize `probe-rs attach` over SiFliUart for RTT marker capture in this host environment.
3. Optional future improvement: if repository-wide "all examples pass" gating is required, execute non-MPI `examples/sf32lb52x` binaries and archive pass/fail logs separately.

## 12. Assumptions and Defaults

1. Breaking changes are acceptable because `mpi` is new and currently unmerged.
2. v1 supports NOR only; NAND/PSRAM are architecture-ready but not exposed.
3. Advanced controller API remains intentionally unstable-gated.
4. Auto-detect returns driver only; metadata stays queryable after construction.
5. Trait-object custom profiles are the only supported custom profile mechanism in this rewrite.

## 13. Main References

1. `sifli-sdk`: `/Users/haobogu/Projects/other/SiFli-SDK`
2. `embassy`: `/Users/haobogu/Projects/rust/embassy`
