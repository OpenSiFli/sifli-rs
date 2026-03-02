# SiFli Radio

Bluetooth radio stack for SiFli SF32LB52x MCUs, built on [sifli-hal](../sifli-hal) and [Embassy](https://github.com/embassy-rs/embassy).

Implements LCPU (Bluetooth coprocessor) management and provides a [bt-hci](https://github.com/embassy-rs/bt-hci) controller interface for use with the [trouble](https://github.com/embassy-rs/trouble) BLE stack.

## What's Inside

- LCPU firmware loading, patching, and boot sequence (A3 + Letter Series)
- ROM configuration block (codegen from TOML layout definition)
- RF calibration (VCO, TXDC, opt_cal)
- BLE HCI transport over IPC mailbox
- BLE sleep and power management

## Usage

Add to your `Cargo.toml`:

```toml
sifli-radio = { path = "../sifli-radio", features = ["sf32lb52x-lcpu"] }
```

See [ble_advertise](../examples/sf32lb52x/src/bin/ble_advertise.rs) for a complete BLE advertising example.

## Features

- `sf32lb52x-lcpu` — LCPU support (firmware, patching, RF calibration, HCI transport)
- `edr` — EDR LO 3GHz calibration (VCO3G + OSLO), only needed for classic Bluetooth
- `defmt` / `log` — debug log output

## SDK Reference

This crate is functionally equivalent to:

- [bf0_lcpu_init.c](https://github.com/OpenSiFli/SiFli-SDK/blob/main/drivers/cmsis/sf32lb52x/bf0_lcpu_init.c) — LCPU boot sequence
- [bt_rf_fulcal.c](https://github.com/OpenSiFli/SiFli-SDK/blob/main/middleware/bluetooth/service/ble/bt_rf_fulcal/bt_rf_fulcal.c) — RF calibration
- [rom_config.h](https://github.com/OpenSiFli/SiFli-SDK/blob/main/middleware/bluetooth/include/rom_config.h) — ROM config API

## License

Apache License, Version 2.0 ([LICENSE](../LICENSE) or <http://www.apache.org/licenses/LICENSE-2.0>).
