Convert C arrays to Rust const arrays

Usage
- Patch aliasing (rename known SiFli patch arrays): `carray2rs --alias-patch lcpu_patch.c > patch_data.rs`

Notes
- Supports common C types: `uint8_t`/`unsigned char` -> `u8`, `uint32_t`/`unsigned int` -> `u32`.
- Strips C/C++ comments and whitespace.
- Parses hex (`0x...`) and decimal numbers with optional `U/L` suffixes.
- Unknown types default to `u32`.

