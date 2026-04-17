#!/usr/bin/env python3
"""
SF32LB52x SRAM layout visualization generator.

Reads sifli-hal/data/sf32lb52x/sram_layout.toml, generates docs/sram_layout.html.

Usage (run from sifli-rs/ root):
    python3 scripts/gen_sram_map.py
    python3 scripts/gen_sram_map.py \\
        --data sifli-hal/data/sf32lb52x/sram_layout.toml \\
        --out  docs/sram_layout.html
    python3 scripts/gen_sram_map.py --zh   # Chinese notes

Dependencies: Python 3.11+ standard library (tomllib), no extra packages required.
"""

import argparse
import html as html_lib
import sys
import tomllib
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ── Color palette: color key → (fill, text) ──────────────────────────────────
PALETTE: Dict[str, Tuple[str, str]] = {
    "app":     ("#3b82f6", "#ffffff"),
    "dtcm":    ("#1d4ed8", "#ffffff"),
    "mailbox": ("#ea580c", "#ffffff"),
    "patch":   ("#7c3aed", "#ffffff"),
    "em":      ("#16a34a", "#ffffff"),
    "audio":   ("#b45309", "#ffffff"),
    "rom":     ("#475569", "#ffffff"),
    "config":  ("#b91c1c", "#ffffff"),
    "nvds":    ("#9d174d", "#ffffff"),
    "rf_cmd":  ("#0e7490", "#ffffff"),
    "rf_cal":  ("#0f766e", "#ffffff"),
    "gap":     (None,      "#6b7280"),
}

LEGEND_LABELS = {
    "app":     "Application / Code",
    "dtcm":    "DTCM / Retention",
    "mailbox": "IPC Mailbox",
    "patch":   "Patch Buffer",
    "em":      "BLE Exchange Memory",
    "audio":   "Audio Shared",
    "rom":     "ROM Private RAM",
    "config":  "ROM Config / Custom",
    "nvds":    "NVDS",
    "rf_cmd":  "RFC Command Seqs",
    "rf_cal":  "RF Calibration Table",
}

# ── Layout parameters ────────────────────────────────────────────────────────
ADDR_W  = 98   # Address label width (left side)
COL_W   = 155  # Color block column width
GAP_X   = 120  # Column gap (including right annotation space)
ANNOT_R = 115  # Rightmost column annotation area width
TICK_W  = 6    # Tick mark length
COL_H   = 680  # Column height (pixels)
TITLE_H = 62   # Column title height (up to 3 wrapped note lines)
TOP_Y   = 16
BOT_Y   = 24
MIN_H   = 22   # Minimum region height (pixels)
ANNOT_MIN_GAP = 22  # Minimum gap between annotations in same column (pixels)


# ── Svg builder ──────────────────────────────────────────────────────────────

class Svg:
    """Lightweight SVG fragment builder."""

    def __init__(self):
        self._parts: List[str] = []

    def text(self, x, y, content, *, fill, size, anchor=None,
             family=None, weight=None, opacity=None) -> None:
        attrs = f'x="{x}" y="{y}"'
        if anchor:
            attrs += f' text-anchor="{anchor}"'
        attrs += f' fill="{fill}" font-size="{size}"'
        if family:
            attrs += f' font-family="{family}"'
        if weight:
            attrs += f' font-weight="{weight}"'
        if opacity is not None:
            attrs += f' opacity="{opacity}"'
        self._parts.append(f'<text {attrs}>{content}</text>')

    def rect(self, x, y, w, h, *, fill=None, stroke=None,
             stroke_w=1, rx=0, title=None) -> None:
        attrs = f'x="{x}" y="{y}" width="{w}" height="{h}"'
        if fill is not None:
            attrs += f' fill="{fill}"'
        if stroke is not None:
            attrs += f' stroke="{stroke}" stroke-width="{stroke_w}"'
        if rx:
            attrs += f' rx="{rx}"'
        if title is not None:
            self._parts.append(f'<rect {attrs}><title>{title}</title></rect>')
        else:
            self._parts.append(f'<rect {attrs}/>')

    def line(self, x1, y1, x2, y2, *, stroke, width=1,
             dash=None, marker_end=None) -> None:
        attrs = (f'x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                 f'stroke="{stroke}" stroke-width="{width}"')
        if dash:
            attrs += f' stroke-dasharray="{dash}"'
        if marker_end:
            attrs += f' marker-end="{marker_end}"'
        self._parts.append(f'<line {attrs}/>')

    def polygon(self, points, *, fill, opacity=None, title=None) -> None:
        attrs = f'points="{points}" fill="{fill}"'
        if opacity is not None:
            attrs += f' opacity="{opacity}"'
        if title is not None:
            self._parts.append(f'<polygon {attrs}><title>{title}</title></polygon>')
        else:
            self._parts.append(f'<polygon {attrs}/>')

    def raw(self, s: str) -> None:
        self._parts.append(s)

    def __str__(self) -> str:
        return "\n".join(self._parts)


# ── Utility functions ────────────────────────────────────────────────────────

def wrap_text(text: str, max_units: int = 28) -> List[str]:
    """
    Wrap text by visual width.  CJK characters count 2 units, ASCII counts 1.
    Allow line breaks after CJK punctuation and spaces.
    """
    lines: List[str] = []
    current = ""
    units = 0
    for ch in text:
        w = 2 if '\u4e00' <= ch <= '\u9fff' or '\u3000' <= ch <= '\u303f' else 1
        current += ch
        units += w
        if units >= max_units and ch in (' ', '。', '，', '、', '；', '+'):
            lines.append(current.rstrip())
            current = ""
            units = 0
    if current.strip():
        lines.append(current.strip())
    return lines or [""]


def fmt_addr(addr: int) -> str:
    return f"0x{addr:08X}"


def fmt_size(n: int) -> str:
    if n >= 1024 * 1024:
        return f"{n // (1024 * 1024)}MB"
    if n >= 1024:
        v = n / 1024
        return f"{v:.0f}KB" if v == int(v) else f"{v:.1f}KB"
    return f"{n}B"


def fmt_const_value(val: int, typ: str) -> str:
    """Format a constant value as hex, matching Rust style."""
    if val >= 0x1_0000:
        return f"0x{val:08X}"
    elif val >= 0x100:
        return f"0x{val:04X}"
    else:
        return f"0x{val:X}"


def insert_gaps(regions: List[dict], *, zh: bool = False) -> List[dict]:
    """Insert gap regions where addresses are non-contiguous."""
    gap_label = "间隙" if zh else "Gap"
    out: List[dict] = []
    for i, r in enumerate(regions):
        if i > 0:
            prev = regions[i - 1]
            gap_start = prev["base"] + prev["size"]
            if gap_start < r["base"]:
                out.append({
                    "name":   "···",
                    "base":   gap_start,
                    "size":   r["base"] - gap_start,
                    "color":  "gap",
                    "note":   f"{gap_label} {fmt_size(r['base'] - gap_start)}",
                    "source": "",
                })
        out.append(r)
    return out


def assign_heights(regions: List[dict], total_h: int = COL_H) -> List[int]:
    """
    Assign pixel heights:
    - Each region gets at least MIN_H px
    - Remaining space distributed proportionally by size
    """
    n = len(regions)
    reserved = n * MIN_H
    remaining = max(total_h - reserved, 0)
    total_size = sum(r["size"] for r in regions)

    heights: List[int] = []
    for r in regions:
        extra = int(remaining * r["size"] / total_size) if total_size else 0
        heights.append(MIN_H + extra)

    # Adjust rounding error to match total_h exactly
    diff = total_h - sum(heights)
    if heights:
        heights[-1] += diff
    return heights


# ── Data collection ──────────────────────────────────────────────────────────

def _apply_zh_notes(banks: List[dict]) -> None:
    """Replace note with note_zh for all banks and their (nested) regions."""
    def _apply(node: dict) -> None:
        if "note_zh" in node:
            node["note"] = node["note_zh"]
        for r in node.get("regions", []):
            _apply(r)
    for bank in banks:
        _apply(bank)


def collect_exports(banks: List[dict]) -> List[dict]:
    """
    Traverse all bank/region/sub-region exports, resolve to a flat constant list.
    Each item contains module, name, value, type, note, source, bank_id.
    """
    resolved: List[dict] = []

    def _collect_region(region: dict, bank: dict):
        for exp in region.get("exports", []):
            field = exp.get("field", "base")
            if field == "base":
                value = region["base"]
            elif field == "size":
                value = region["size"]
            else:
                continue

            transform = exp.get("transform")
            if transform == "cbus":
                cbus_base = bank.get("cbus_base", 0)
                value = (value - bank["base"]) + cbus_base

            resolved.append({
                "module":  exp.get("module", ""),
                "name":    exp.get("name", ""),
                "value":   value,
                "type":    exp.get("type", "usize"),
                "note":    region.get("note", ""),
                "source":  region.get("source", ""),
                "bank_id": bank.get("id", ""),
            })
        # Recurse into sub-regions
        for sub in region.get("regions", []):
            _collect_region(sub, bank)

    for bank in banks:
        for exp in bank.get("exports", []):
            field = exp.get("field", "base")
            if field == "base":
                value = bank["base"]
            elif field == "size":
                value = bank["size"]
            elif field == "lcpu_view_offset":
                value = bank.get("lcpu_view_offset", 0)
            else:
                continue
            resolved.append({
                "module":  exp.get("module", ""),
                "name":    exp.get("name", ""),
                "value":   value,
                "type":    exp.get("type", "usize"),
                "note":    bank.get("note", ""),
                "source":  bank.get("source", ""),
                "bank_id": bank.get("id", ""),
            })
        for region in bank.get("regions", []):
            _collect_region(region, bank)

    return resolved


class BankLayout:
    """Output of render_bank, stores column geometry for annotation use."""
    __slots__ = ("block_x", "regions", "heights")
    def __init__(self, block_x: int, regions: List[dict], heights: List[int]):
        self.block_x = block_x
        self.regions = regions
        self.heights = heights

    def addr_to_y(self, addr: int) -> Optional[int]:
        """Map an absolute address to pixel Y coordinate within the column."""
        y = TOP_Y + TITLE_H
        for region, h in zip(self.regions, self.heights):
            r_base = region["base"]
            r_end  = r_base + region["size"]
            if r_base <= addr < r_end:
                frac = (addr - r_base) / region["size"] if region["size"] > 0 else 0
                return y + int(frac * h)
            y += h
        return None


# ── SVG rendering ────────────────────────────────────────────────────────────

def render_bank(bank: dict, col_x: int, *, zh: bool = False) -> Tuple[str, List[dict], BankLayout]:
    """
    Generate single-column SVG string and its source records.
    Returns (svg_fragment, source_rows, layout)
    """
    regions = sorted(bank.get("regions", []), key=lambda r: r["base"])
    regions = insert_gaps(regions, zh=zh)
    heights = assign_heights(regions)

    svg = Svg()
    source_rows: List[dict] = []
    block_x = col_x + ADDR_W

    # Column title
    title = html_lib.escape(bank["name"])
    note  = html_lib.escape(bank.get("note", ""))
    cx    = block_x + COL_W // 2
    svg.text(cx, TOP_Y + 14, title,
             fill="#f1f5f9", size=13, anchor="middle", weight="700")
    if note:
        note_lines = wrap_text(bank.get("note", ""))[:3]
        for i, line in enumerate(note_lines):
            svg.text(cx, TOP_Y + 28 + i * 11, html_lib.escape(line),
                     fill="#94a3b8", size=9, anchor="middle")

    # Region blocks
    y = TOP_Y + TITLE_H
    for region, h in zip(regions, heights):
        color_key = region.get("color", "app")
        fill, text_color = PALETTE.get(color_key, ("#3b82f6", "#fff"))
        name  = html_lib.escape(region.get("name", ""))
        size_s = fmt_size(region["size"])
        addr_s = fmt_addr(region["base"])
        tooltip = html_lib.escape(
            f"{region.get('name','')}  {size_s}\n{region.get('note','')}\n{region.get('source','')}"
        )

        # Address label + tick mark
        label_y = y + 5
        svg.text(col_x + ADDR_W - TICK_W - 3, label_y, addr_s,
                 fill="#64748b", size=8.5, anchor="end", family="monospace")
        svg.line(block_x - TICK_W, y, block_x, y,
                 stroke="#475569", width=0.8)

        if color_key == "gap":
            pid = f"hatch{abs(hash(str(region['base']))) % 9999}"
            svg.raw(
                f'<defs><pattern id="{pid}" patternUnits="userSpaceOnUse" width="10" height="10">'
                f'<path d="M-1,1 l2,-2 M0,10 l10,-10 M9,11 l2,-2" '
                f'stroke="#334155" stroke-width="1.5"/></pattern></defs>'
            )
            svg.rect(block_x, y, COL_W, h,
                     fill=f"url(#{pid})", stroke="#1e293b")
            if h >= 18:
                svg.text(block_x + COL_W // 2, y + h // 2 + 4, "···",
                         fill="#4b5563", size=11, anchor="middle")
        else:
            svg.rect(block_x, y, COL_W, h,
                     fill=fill, stroke="#0f172a", rx=2, title=tooltip)
            if h >= 46:
                svg.text(block_x + COL_W // 2, y + h // 2 - 5, name,
                         fill=text_color, size=11.5, anchor="middle", weight="600")
                svg.text(block_x + COL_W // 2, y + h // 2 + 11, size_s,
                         fill=text_color, size=9, anchor="middle", opacity=0.8)
            elif h >= 28:
                label = f"{name}  {size_s}"
                svg.text(block_x + COL_W // 2, y + h // 2 + 4,
                         html_lib.escape(label[:32]),
                         fill=text_color, size=9.5, anchor="middle", weight="500")
            else:
                svg.text(block_x + COL_W // 2, y + h // 2 + 4,
                         html_lib.escape(name[:24]),
                         fill=text_color, size=8.5, anchor="middle")

            if region.get("source"):
                source_rows.append({
                    "bank":   bank["name"],
                    "region": region.get("name", ""),
                    "addr":   fmt_addr(region["base"]),
                    "size":   size_s,
                    "source": region.get("source", ""),
                    "note":   region.get("note", ""),
                })

        y += h

    # End address label
    if regions:
        last = regions[-1]
        end_addr = fmt_addr(last["base"] + last["size"] - 1)
        end_y = TOP_Y + TITLE_H + COL_H
        svg.text(col_x + ADDR_W - TICK_W - 3, end_y + 10, end_addr,
                 fill="#64748b", size=8.5, anchor="end", family="monospace")
        svg.line(block_x - TICK_W, end_y, block_x, end_y,
                 stroke="#475569", width=0.8)

    # Column border
    svg.rect(block_x, TOP_Y + TITLE_H, COL_W, COL_H,
             fill="none", stroke="#334155", rx=2)

    layout = BankLayout(block_x, regions, heights)
    return str(svg), source_rows, layout


def render_annotations(visible_banks: List[dict], layouts: List[BankLayout],
                       exports: List[dict]) -> str:
    """
    Annotate address-type constants from exports on the diagram.

    Filtering rules:
    - Skip _LCPU suffixed names (attached as aliases to same-name annotations)
    - Skip exports whose value equals a top-level region base (already labeled)
    - Skip exports whose value doesn't fall within any visible bank

    Anti-overlap: same-column annotations sorted by Y, pushed down if gap < ANNOT_MIN_GAP.
    """
    # Build LCPU alias map: base_name → export
    lcpu_aliases: Dict[str, dict] = {}
    normal_exports: List[dict] = []
    for c in exports:
        name = c.get("name", "")
        if name.endswith("_LCPU"):
            lcpu_aliases[name[:-5]] = c
        else:
            normal_exports.append(c)

    # Collect top-level region base addresses, grouped by bank
    region_bases: Dict[str, set] = {}
    for bank in visible_banks:
        bases = set()
        bases.add(bank["base"])
        for r in bank.get("regions", []):
            bases.add(r["base"])
        region_bases[id(bank)] = bases

    # Pass 1: resolve target column and raw Y, filter out unneeded exports
    items: List[tuple] = []

    for c in normal_exports:
        val    = c.get("value", 0)
        module = c.get("module", "")

        candidates = []
        for bank, layout in zip(visible_banks, layouts):
            bank_base = bank["base"]
            bank_end  = bank_base + bank["size"]
            if not (bank_base <= val < bank_end):
                continue
            variant = bank.get("variant", "")
            bank_id = bank.get("id", "")
            exact = (module == variant) or bank_id.endswith(f"_{module}") or module == bank_id
            candidates.append((exact, bank, layout))

        if not candidates:
            continue
        candidates.sort(key=lambda t: (not t[0],))
        _, bank, layout = candidates[0]

        if val in region_bases[id(bank)]:
            continue

        y_pos = layout.addr_to_y(val)
        if y_pos is None:
            frac = (val - bank["base"]) / bank["size"]
            y_pos = TOP_Y + TITLE_H + int(frac * COL_H)

        items.append((layout, bank, y_pos, c, lcpu_aliases.get(c.get("name", ""))))

    # Pass 2: group by column, sort and push down to avoid overlap
    by_col: Dict[int, List] = defaultdict(list)
    for item in items:
        by_col[item[0].block_x].append(item)

    svg = Svg()

    for col_x in sorted(by_col):
        col_items = sorted(by_col[col_x], key=lambda t: t[2])
        adjusted_ys: List[int] = []
        for i, (layout, bank, y_raw, c, lcpu) in enumerate(col_items):
            y = y_raw
            if adjusted_ys:
                prev_y = adjusted_ys[-1]
                extra = ANNOT_MIN_GAP + (10 if lcpu else 0)
                if y < prev_y + extra:
                    y = prev_y + extra
            adjusted_ys.append(y)

        for (layout, bank, y_raw, c, lcpu), y_pos in zip(col_items, adjusted_ys):
            val  = c.get("value", 0)
            name = c.get("name", "")
            note = c.get("note", "")
            rx = layout.block_x + COL_W
            tooltip = html_lib.escape(f"{name}\n{fmt_addr(val)}\n{note}")

            svg.polygon(f"{rx+6},{y_raw-3} {rx+6},{y_raw+3} {rx+1},{y_raw}",
                        fill="#fbbf24", opacity=0.9, title=tooltip)
            if abs(y_pos - y_raw) > 2:
                svg.line(rx + 6, y_raw, rx + 12, y_pos + 3,
                         stroke="#fbbf24", width=0.5, dash="2,1")
            else:
                svg.line(rx + 6, y_pos, rx + 12, y_pos,
                         stroke="#fbbf24", width=0.7, dash="2,1")
            svg.text(rx + 14, y_pos + 3, html_lib.escape(name),
                     fill="#fbbf24", size=7.5, family="monospace", weight="500")
            svg.text(rx + 14, y_pos + 13, fmt_addr(val),
                     fill="#d97706", size=6.5, family="monospace")
            if lcpu:
                lcpu_val = lcpu.get("value", 0)
                svg.text(rx + 14, y_pos + 23, f'LCPU: {fmt_addr(lcpu_val)}',
                         fill="#22d3ee", size=6.5, family="monospace")

    return str(svg)


def render_offset_arrows(all_banks: List[dict],
                         visible_banks: List[dict],
                         layouts: List[BankLayout],
                         *, zh: bool = False) -> str:
    """
    Draw cross-column offset arrows for bank.lcpu_view_offset.

    Draws a dashed arrow from HCPU mailbox region to LCPU column with offset and example.
    """
    offset_bank = next((b for b in all_banks if b.get("lcpu_view_offset")), None)
    if not offset_bank:
        return ""
    offset_val = offset_bank["lcpu_view_offset"]

    hcpu_idx = next((i for i, b in enumerate(visible_banks)
                     if b.get("id") == offset_bank.get("id")), None)
    lcpu_idx = next((i for i, b in enumerate(visible_banks)
                     if b.get("id", "").startswith("lcpu")), None)
    if hcpu_idx is None or lcpu_idx is None:
        return ""

    example_region = next((r for r in visible_banks[hcpu_idx].get("regions", [])
                           if "MB CH1" in r.get("name", "") and "HCPU" in r.get("name", "")),
                          None)
    if not example_region:
        return ""

    example_addr = example_region["base"]
    result_addr  = example_addr + offset_val

    y_pos = layouts[hcpu_idx].addr_to_y(example_addr)
    if y_pos is None:
        return ""
    y_pos += 8

    rx1 = layouts[hcpu_idx].block_x + COL_W
    rx2 = layouts[lcpu_idx].block_x - ADDR_W
    mid_x = (rx1 + rx2) // 2

    eg_label = "例" if zh else "e.g."

    svg = Svg()
    svg.raw(
        '<defs><marker id="arr_offset" markerWidth="7" markerHeight="5" '
        'refX="6" refY="2.5" orient="auto">'
        '<path d="M0,0 L7,2.5 L0,5 Z" fill="#fbbf24"/></marker></defs>'
    )
    svg.line(rx1 + 3, y_pos, rx2 - 3, y_pos,
             stroke="#fbbf24", width=1, dash="4,3",
             marker_end="url(#arr_offset)")
    svg.text(mid_x, y_pos - 7, '+ 0x0A00_0000',
             fill="#fbbf24", size=7.5, anchor="middle", family="monospace", weight="500")
    svg.text(mid_x, y_pos + 12,
             f'{eg_label}: {fmt_addr(example_addr)} → {fmt_addr(result_addr)}',
             fill="#d97706", size=6.5, anchor="middle", family="monospace")

    return str(svg)


# ── HTML fragments and template ──────────────────────────────────────────────

def build_legend() -> str:
    items = []
    for key, label in LEGEND_LABELS.items():
        fill, _ = PALETTE.get(key, ("#3b82f6", "#fff"))
        items.append(
            f'<span style="display:inline-flex;align-items:center;gap:5px;margin:4px 10px 4px 0">'
            f'<svg width="14" height="14"><rect width="14" height="14" rx="2" fill="{fill}"/></svg>'
            f'<span style="font-size:12px;color:#cbd5e1">{html_lib.escape(label)}</span></span>'
        )
    return "".join(items)


def build_source_table(all_rows: List[dict]) -> str:
    rows_html = ""
    for r in all_rows:
        rows_html += (
            f'<tr>'
            f'<td style="color:#94a3b8;white-space:nowrap">{html_lib.escape(r["bank"])}</td>'
            f'<td style="color:#e2e8f0;font-weight:500">{html_lib.escape(r["region"])}</td>'
            f'<td style="font-family:monospace;color:#7dd3fc">{r["addr"]}</td>'
            f'<td style="color:#86efac">{r["size"]}</td>'
            f'<td style="color:#94a3b8;font-size:11px">{html_lib.escape(r["source"])}</td>'
            f'<td style="color:#64748b;font-size:11px">{html_lib.escape(r["note"])}</td>'
            f'</tr>'
        )
    return f"""
<details style="margin-top:32px">
  <summary style="cursor:pointer;color:#94a3b8;font-size:13px;user-select:none">
    Source References ({len(all_rows)} regions)
  </summary>
  <div style="overflow-x:auto;margin-top:12px">
  <table style="border-collapse:collapse;width:100%;font-size:12px">
    <thead><tr style="border-bottom:1px solid #334155">
      <th style="text-align:left;padding:6px 10px;color:#64748b">Bank</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Region</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Address</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Size</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Source</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Note</th>
    </tr></thead>
    <tbody style="color:#e2e8f0">{rows_html}</tbody>
  </table>
  </div>
</details>"""


def build_exports_table(exports: List[dict]) -> str:
    if not exports:
        return ""
    rows_html = ""
    for c in exports:
        module = html_lib.escape(c.get("module", ""))
        name   = html_lib.escape(c.get("name", ""))
        typ    = c.get("type", "usize")
        val    = c.get("value", 0)
        val_s  = fmt_const_value(val, typ)
        source = html_lib.escape(c.get("source", ""))
        note   = html_lib.escape(c.get("note", ""))
        rows_html += (
            f'<tr>'
            f'<td style="color:#c084fc">{module}</td>'
            f'<td style="color:#e2e8f0;font-weight:500;font-family:monospace">{name}</td>'
            f'<td style="font-family:monospace;color:#fbbf24">{val_s}</td>'
            f'<td style="color:#64748b">{typ}</td>'
            f'<td style="color:#94a3b8;font-size:11px">{source}</td>'
            f'<td style="color:#64748b;font-size:11px">{note}</td>'
            f'</tr>'
        )
    return f"""
<details style="margin-top:16px" open>
  <summary style="cursor:pointer;color:#94a3b8;font-size:13px;user-select:none">
    Exported Constants ({len(exports)} entries)
  </summary>
  <div style="overflow-x:auto;margin-top:12px">
  <table style="border-collapse:collapse;width:100%;font-size:12px">
    <thead><tr style="border-bottom:1px solid #334155">
      <th style="text-align:left;padding:6px 10px;color:#64748b">Module</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Name</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Value</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Type</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Source</th>
      <th style="text-align:left;padding:6px 10px;color:#64748b">Note</th>
    </tr></thead>
    <tbody style="color:#e2e8f0">{rows_html}</tbody>
  </table>
  </div>
</details>"""


_HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{chip} SRAM Layout</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      background: #0f172a;
      color: #e2e8f0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      padding: 32px 24px;
      min-width: {min_body_w}px;
    }}
    h1 {{
      font-size: 20px;
      font-weight: 700;
      color: #f8fafc;
      margin-bottom: 4px;
    }}
    .subtitle {{
      font-size: 12px;
      color: #64748b;
      margin-bottom: 24px;
    }}
    .diagram-wrap {{ overflow-x: auto; }}
    svg {{ display: block; }}
    .legend {{
      margin-top: 20px;
      padding: 12px 16px;
      background: #1e293b;
      border-radius: 8px;
      border: 1px solid #334155;
    }}
    .legend-title {{
      font-size: 11px;
      color: #64748b;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 8px;
    }}
    details summary {{ cursor: pointer; }}
  </style>
</head>
<body>
  <h1>{chip} SRAM Layout</h1>
  <p class="subtitle">
    Generated from <code style="color:#7dd3fc">{data_name}</code>.
    Hover over regions for details.  Addresses are absolute byte addresses.
    <span style="color:#fbbf24">&#9654;</span> markers show exported constants.
  </p>

  <div class="diagram-wrap">
    <svg width="{svg_w}" height="{svg_h}" xmlns="http://www.w3.org/2000/svg">
      <!-- background -->
      <rect width="{svg_w}" height="{svg_h}" fill="#0f172a"/>
      {svg_content}
    </svg>
  </div>

  <div class="legend">
    <div class="legend-title">Color Legend</div>
    {legend_html}
  </div>

  {exports_html}

  {table_html}
</body>
</html>"""


# ── Main ─────────────────────────────────────────────────────────────────────

def generate(data_path: Path, out_path: Path, *, zh: bool = False) -> None:
    with open(data_path, "rb") as f:
        data = tomllib.load(f)

    chip      = data.get("chip", "Unknown")
    all_banks = data.get("banks", [])

    if zh:
        _apply_zh_notes(all_banks)

    # Collect resolved exports from all banks (including invisible ones)
    exports = collect_exports(all_banks)

    # Filter visible banks for SVG rendering
    visible_banks = [b for b in all_banks if b.get("visible", True)]

    # Calculate total SVG width (extra annotation area for rightmost column)
    n_banks   = len(visible_banks)
    unit_w    = ADDR_W + COL_W
    svg_w     = n_banks * unit_w + (n_banks - 1) * GAP_X + 20 + ANNOT_R + 20
    svg_h     = TOP_Y + TITLE_H + COL_H + BOT_Y + 16

    bank_svgs: List[str] = []
    all_sources: List[dict] = []
    layouts: List[BankLayout] = []

    for i, bank in enumerate(visible_banks):
        col_x = 20 + i * (unit_w + GAP_X)
        svg_frag, src_rows, layout = render_bank(bank, col_x, zh=zh)
        bank_svgs.append(svg_frag)
        all_sources.extend(src_rows)
        layouts.append(layout)

    # Annotations: draw address-type exported constants to the right of bank columns
    annot_svg = render_annotations(visible_banks, layouts, exports)
    bank_svgs.append(annot_svg)

    # Cross-column offset arrows (from bank.lcpu_view_offset)
    offset_svg = render_offset_arrows(all_banks, visible_banks, layouts, zh=zh)
    bank_svgs.append(offset_svg)

    svg_content = "\n".join(bank_svgs)
    legend_html = build_legend()
    table_html  = build_source_table(all_sources)
    exports_html = build_exports_table(exports)

    html = _HTML_TEMPLATE.format(
        chip=chip,
        min_body_w=svg_w + 48,
        svg_w=svg_w,
        svg_h=svg_h,
        data_name=data_path.name,
        svg_content=svg_content,
        legend_html=legend_html,
        exports_html=exports_html,
        table_html=table_html,
    )

    out_path.write_text(html, encoding="utf-8")
    all_region_bases: set = set()
    for b in visible_banks:
        all_region_bases.add(b["base"])
        for r in b.get("regions", []):
            all_region_bases.add(r["base"])
    n_annot = sum(1 for c in exports
                  if not c.get("name", "").endswith("_LCPU")
                  and c.get("value", 0) not in all_region_bases
                  and any(b["base"] <= c.get("value", 0) < b["base"] + b["size"]
                          for b in visible_banks))
    print(f"Generated: {out_path}  ({len(visible_banks)} visible banks, "
          f"{len(all_sources)} regions, {n_annot} annotations, {len(exports)} exports)")


def main() -> None:
    here = Path(__file__).parent.parent  # scripts/ -> sifli-rs/

    parser = argparse.ArgumentParser(description="Generate SRAM layout HTML from TOML.")
    parser.add_argument(
        "--data", type=Path,
        default=here / "sifli-hal" / "data" / "sf32lb52x" / "sram_layout.toml",
        help="Input TOML file",
    )
    parser.add_argument(
        "--out", type=Path,
        default=here / "docs" / "sram_layout.html",
        help="Output HTML file",
    )
    parser.add_argument(
        "--zh", action="store_true",
        help="Use Chinese notes (note_zh) instead of English",
    )
    args = parser.parse_args()

    if not args.data.exists():
        print(f"Error: data file not found: {args.data}", file=sys.stderr)
        sys.exit(1)

    generate(args.data, args.out, zh=args.zh)


if __name__ == "__main__":
    main()
