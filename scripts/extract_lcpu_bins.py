"""
Extract LCPU firmware and patch binary data from SiFli SDK C source files.

Fetches C files directly from the SiFli SDK GitHub repository, parses u32 arrays,
converts to little-endian bytes, and saves as .bin files.
Output goes to sifli-hal/data/sf32lb52x/lcpu/.

Usage:
    python scripts/extract_lcpu_bins.py [--ref main]

Source repository: https://github.com/OpenSiFli/SiFli-SDK
Source files:
    - example/rom_bin/lcpu_general_ble_img/lcpu_52x.c  -> g_lcpu_bin
    - drivers/cmsis/sf32lb52x/lcpu_patch.c              -> g_lcpu_patch_list, g_lcpu_patch_bin
    - drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c        -> g_lcpu_patch_list, g_lcpu_patch_bin
"""

import re
import struct
import os
import argparse
from urllib.request import urlopen
from urllib.error import URLError

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.path.join(REPO_ROOT, "sifli-hal", "data", "sf32lb52x", "lcpu")

SDK_REPO = "https://raw.githubusercontent.com/OpenSiFli/SiFli-SDK"

# SDK-relative paths of the source C files
SDK_FILES = {
    "example/rom_bin/lcpu_general_ble_img/lcpu_52x.c": [
        ("lcpu_firmware.bin", "g_lcpu_bin"),
    ],
    "drivers/cmsis/sf32lb52x/lcpu_patch.c": [
        ("patch_a3_list.bin", "g_lcpu_patch_list"),
        ("patch_a3_bin.bin", "g_lcpu_patch_bin"),
    ],
    "drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c": [
        ("patch_letter_list.bin", "g_lcpu_patch_list"),
        ("patch_letter_bin.bin", "g_lcpu_patch_bin"),
    ],
}


def fetch_file(sdk_path, ref):
    """Fetch a file from the SiFli SDK GitHub repository."""
    url = f"{SDK_REPO}/{ref}/{sdk_path}"
    print(f"Fetching {url}")
    try:
        with urlopen(url) as resp:
            return resp.read().decode("utf-8")
    except URLError as e:
        raise RuntimeError(f"Failed to fetch {url}: {e}") from e


def parse_c_u32_array(content, symbol, source_name):
    """Parse a C uint32 array by symbol name, return list of u32 values."""
    pos = content.find(symbol)
    if pos < 0:
        raise ValueError(f"symbol `{symbol}` not found in {source_name}")

    brace_start = content.index("{", pos)
    brace_end = content.index("}", brace_start)
    inner = content[brace_start + 1 : brace_end]

    values = []
    for tok in re.split(r"[,\s]+", inner):
        tok = tok.strip().rstrip("uUlL")
        if not tok:
            continue
        if tok.startswith("0x") or tok.startswith("0X"):
            values.append(int(tok, 16))
        else:
            values.append(int(tok))
    return values


def words_to_le_bytes(words):
    """Convert list of u32 words to little-endian bytes."""
    return b"".join(struct.pack("<I", w) for w in words)


def main():
    parser = argparse.ArgumentParser(description="Extract LCPU bins from SiFli SDK")
    parser.add_argument("--ref", default="main", help="Git ref/branch/tag (default: main)")
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    for sdk_path, extractions in SDK_FILES.items():
        content = fetch_file(sdk_path, args.ref)
        for out_name, symbol in extractions:
            print(f"  Extracting {symbol} -> {out_name}")
            words = parse_c_u32_array(content, symbol, sdk_path)
            data = words_to_le_bytes(words)
            out_path = os.path.join(OUTPUT_DIR, out_name)
            with open(out_path, "wb") as f:
                f.write(data)
            print(f"    {len(data)} bytes")

    print("\nDone.")


if __name__ == "__main__":
    main()
