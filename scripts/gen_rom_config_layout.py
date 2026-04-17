#!/usr/bin/env python3
"""Generate rom_config_layout.html from rom_config_layout.toml.

Output style matches sram_layout.html (same colors, SVG conventions, dark theme).

Usage:
    python3 scripts/gen_rom_config_layout.py

Reads:  sifli-radio/data/sf32lb52x/rom_config_layout.toml
Writes: docs/rom_config_layout.html
"""

import pathlib

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # Python < 3.11

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
TOML_PATH = REPO_ROOT / "sifli-radio" / "data" / "sf32lb52x" / "rom_config_layout.toml"
OUT_PATH = REPO_ROOT / "docs" / "rom_config_layout.html"

# ── Colour palette — same as sram_layout.html ────────────────────────────────

COLOR_MAP = {
    "app":     "#3b82f6",  # blue  — Application / Code
    "dtcm":    "#1d4ed8",  # dark blue
    "mailbox": "#ea580c",  # orange — IPC Mailbox
    "patch":   "#7c3aed",  # purple
    "em":      "#16a34a",  # green  — BLE Exchange Memory
    "audio":   "#b45309",  # amber
    "rom":     "#475569",  # gray   — ROM Private RAM
    "config":  "#b91c1c",  # red    — ROM Config / Custom
    "nvds":    "#9d174d",  # pink
    "rf_cmd":  "#0e7490",  # teal
    "rf_cal":  "#0f766e",  # green-teal — RF Calibration
    "bt_sub":  "#2563eb",  # brighter blue — BtRomConfig sub-fields
}

LEGEND_LABELS = {
    "config":  "Config (scalar)",
    "rf_cal":  "RF Calibration",
    "em":      "Exchange Memory",
    "app":     "Struct / Activity",
    "mailbox": "IPC Mailbox",
    "bt_sub":  "BtRomConfig field",
}

# sram_layout.html SVG conventions
BG          = "#0f172a"
STROKE_BG   = "#0f172a"
ADDR_FILL   = "#64748b"
TICK_STROKE = "#475569"
TEXT_FILL   = "#ffffff"
PAD_FILL    = "#1e293b"
PAD_STROKE  = "#334155"


def fmt_size(n: int) -> str:
    if n >= 1024 and n % 1024 == 0:
        return f"{n // 1024}KB"
    return f"{n}B"


def main():
    with open(TOML_PATH, "rb") as f:
        data = tomllib.load(f)

    bank    = data["bank"]
    regions = sorted(bank["regions"], key=lambda r: r["offset"])
    size_a3 = bank["size_a3"]
    size_lt = bank["size"]
    magic   = bank["magic"]

    # ── Find sub-region columns (BT_ROM_CONFIG etc.) ──────────────────
    sub_columns = []  # (parent_region, sub_regions_sorted)
    for r in regions:
        subs = r.get("regions", [])
        if subs:
            sub_columns.append((r, sorted(subs, key=lambda s: s["offset"])))

    # ── Layout constants (match sram_layout.html proportions) ────────────
    COL_W    = 200       # column rect width
    GAP      = 100       # gap between columns
    MARGIN_L = 120       # left margin for address labels
    MARGIN_T = 78        # top margin
    ROW_MIN  = 22        # minimum row height
    SCALE    = 2.8       # bytes → px

    num_cols = 2 + len(sub_columns)  # A3, Letter, + one per sub-column

    def col_x(ci: int) -> float:
        return MARGIN_L + ci * (COL_W + GAP)

    # Build main row list (flat — no sub-expansion)
    rows = []  # (offset, size, region_or_None)
    prev_end = 0
    for r in regions:
        off = r["offset"]
        sz  = r["size"]
        if off > prev_end:
            rows.append((prev_end, off - prev_end, None))  # padding
        rows.append((off, sz, r))
        prev_end = off + sz
    if prev_end < size_lt:
        rows.append((prev_end, size_lt - prev_end, None))  # trailing pad

    def row_h(sz: int) -> float:
        return max(sz * SCALE, ROW_MIN)

    # Build sub-column row lists
    sub_col_rows = []  # list of [(offset, size, sub_region_or_None)]
    for parent, subs in sub_columns:
        parent_sz = parent["size"]
        sub_rows = []
        prev = 0
        for sr in subs:
            sr_off = sr["offset"]
            if sr_off > prev:
                sub_rows.append((prev, sr_off - prev, None))
            sub_rows.append((sr_off, sr["size"], sr))
            prev = sr_off + sr["size"]
        if prev < parent_sz:
            sub_rows.append((prev, parent_sz - prev, None))
        sub_col_rows.append(sub_rows)

    main_col_h = MARGIN_T + sum(row_h(sz) for _, sz, _ in rows) + len(rows) + 20
    sub_col_heights = []
    for sr_rows in sub_col_rows:
        h = MARGIN_T + sum(row_h(sz) for _, sz, _ in sr_rows) + len(sr_rows) + 20
        sub_col_heights.append(h)

    svg_h = max(main_col_h, *sub_col_heights) + 30
    svg_w = MARGIN_L + num_cols * COL_W + (num_cols - 1) * GAP + 30

    parts = []

    # ── SVG header ───────────────────────────────────────────────────────
    parts.append(
        f'    <svg width="{svg_w:.0f}" height="{svg_h:.0f}" xmlns="http://www.w3.org/2000/svg">\n'
        f'      <!-- background -->\n'
        f'      <rect width="{svg_w:.0f}" height="{svg_h:.0f}" fill="{BG}"/>\n'
    )

    # ── Column headers ─────────────────────────────────────────────────
    for ci, (label, sz) in enumerate([
        ("A3 Revision", size_a3),
        ("Letter Series", size_lt),
    ]):
        cx = col_x(ci) + COL_W / 2
        parts.append(
            f'      <text x="{cx}" y="30" text-anchor="middle" '
            f'fill="#f1f5f9" font-size="13" font-weight="700">'
            f'ROM Config ({label})</text>\n'
        )
        parts.append(
            f'<text x="{cx}" y="44" text-anchor="middle" '
            f'fill="#94a3b8" font-size="9">'
            f'{fmt_size(sz)} — LCPU_CONFIG_TYPE_T</text>\n'
        )
        parts.append(
            f'<text x="{cx}" y="55" text-anchor="middle" '
            f'fill="#94a3b8" font-size="9">'
            f'magic = 0x{magic:08X}</text>\n'
        )

    # Sub-column headers
    for si, (parent, _subs) in enumerate(sub_columns):
        ci = 2 + si
        cx = col_x(ci) + COL_W / 2
        name = parent["name"]
        parent_sz = parent["size"]
        src = parent.get("source", "")
        parts.append(
            f'<text x="{cx}" y="30" text-anchor="middle" '
            f'fill="#f1f5f9" font-size="13" font-weight="700">'
            f'{name}</text>\n'
        )
        parts.append(
            f'<text x="{cx}" y="44" text-anchor="middle" '
            f'fill="#94a3b8" font-size="9">'
            f'{fmt_size(parent_sz)} — {src}</text>\n'
        )
        parts.append(
            f'<text x="{cx}" y="55" text-anchor="middle" '
            f'fill="#94a3b8" font-size="9">'
            f'at +0x{parent["offset"]:02X} in ROM Config</text>\n'
        )

    # ── Helper: draw a standard region rect ────────────────────────────
    def draw_region(x, y, w, h, region, off, vis_sz, show_addr=True):
        name  = region["name"]
        color = COLOR_MAP.get(region.get("color", ""), "#475569")
        note  = region.get("note", "")
        src   = region.get("source", "")

        if show_addr:
            parts.append(
                f'<text x="{x - 9}" y="{y + 12}" text-anchor="end" '
                f'fill="{ADDR_FILL}" font-size="8.5" font-family="monospace">'
                f'+0x{off:02X}</text>\n'
            )
            parts.append(
                f'<line x1="{x - 6}" y1="{y}" x2="{x}" y2="{y}" '
                f'stroke="{TICK_STROKE}" stroke-width="0.8"/>\n'
            )

        tooltip = f"{name}  {fmt_size(vis_sz)}\n{note}\n{src}"
        parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="{color}" stroke="{STROKE_BG}" stroke-width="1" rx="2">'
            f'<title>{tooltip}</title></rect>\n'
        )

        if h >= 28:
            parts.append(
                f'<text x="{x + w/2}" y="{y + h/2 - 2}" '
                f'text-anchor="middle" fill="{TEXT_FILL}" '
                f'font-size="11.5" font-weight="600">{name}</text>\n'
            )
            parts.append(
                f'<text x="{x + w/2}" y="{y + h/2 + 11}" '
                f'text-anchor="middle" fill="{TEXT_FILL}" '
                f'font-size="9" opacity="0.8">{fmt_size(vis_sz)}</text>\n'
            )
        else:
            parts.append(
                f'<text x="{x + w/2}" y="{y + h/2 + 4}" '
                f'text-anchor="middle" fill="{TEXT_FILL}" '
                f'font-size="9" font-weight="600">{name}  {fmt_size(vis_sz)}</text>\n'
            )

    def draw_padding(x, y, w, h, vis_sz):
        parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
            f'fill="{PAD_FILL}" stroke="{PAD_STROKE}" stroke-width="0.5" rx="2"/>\n'
        )
        if h >= 16:
            parts.append(
                f'<text x="{x + w/2}" y="{y + h/2 + 3}" '
                f'text-anchor="middle" fill="#4b5563" font-size="9">'
                f'pad {vis_sz}B</text>\n'
            )

    # ── Draw main columns (A3, Letter) ────────────────────────────────
    # Track y-positions of regions with sub-columns in Letter column for connector lines.
    # Key: region name, Value: (y, h)
    letter_region_pos = {}

    for ci, block_size in enumerate([size_a3, size_lt]):
        x = col_x(ci)
        y = MARGIN_T
        outline_y0 = y

        for off, sz, region in rows:
            if off >= block_size:
                break
            vis_sz = min(sz, block_size - off)
            h = row_h(vis_sz)

            if region is None:
                draw_padding(x, y, COL_W, h, vis_sz)
            else:
                draw_region(x, y, COL_W, h, region, off, vis_sz)
                # Record position for connector (Letter column only)
                if ci == 1 and region.get("regions"):
                    letter_region_pos[region["name"]] = (y, h)

            y += h + 1

        # End address label
        parts.append(
            f'<text x="{x - 9}" y="{y + 10}" text-anchor="end" '
            f'fill="{ADDR_FILL}" font-size="8.5" font-family="monospace">'
            f'+0x{block_size:02X}</text>\n'
        )
        parts.append(
            f'<line x1="{x - 6}" y1="{y}" x2="{x}" y2="{y}" '
            f'stroke="{TICK_STROKE}" stroke-width="0.8"/>\n'
        )

        # Column outline
        outline_h = y - outline_y0
        parts.append(
            f'<rect x="{x}" y="{outline_y0}" width="{COL_W}" height="{outline_h}" '
            f'fill="none" stroke="#334155" stroke-width="1" rx="2"/>\n'
        )

    # ── Draw sub-columns (BT_ROM_CONFIG etc.) ─────────────────────────
    for si, ((parent, _subs), sr_rows) in enumerate(zip(sub_columns, sub_col_rows)):
        ci = 2 + si
        x = col_x(ci)
        y = MARGIN_T
        outline_y0 = y
        parent_color = parent.get("color", "app")

        for sr_off, sr_sz, sr_region in sr_rows:
            h = row_h(sr_sz)

            if sr_region is None:
                draw_padding(x, y, COL_W, h, sr_sz)
            else:
                # Use bt_sub color for sub-fields
                sr_draw = dict(sr_region)
                sr_draw["color"] = "bt_sub"
                draw_region(x, y, COL_W, h, sr_draw, sr_off, sr_sz)

            y += h + 1

        # End address label
        parent_sz = parent["size"]
        parts.append(
            f'<text x="{x - 9}" y="{y + 10}" text-anchor="end" '
            f'fill="{ADDR_FILL}" font-size="8.5" font-family="monospace">'
            f'+0x{parent_sz:02X}</text>\n'
        )
        parts.append(
            f'<line x1="{x - 6}" y1="{y}" x2="{x}" y2="{y}" '
            f'stroke="{TICK_STROKE}" stroke-width="0.8"/>\n'
        )

        # Column outline
        outline_h = y - outline_y0
        parts.append(
            f'<rect x="{x}" y="{outline_y0}" width="{COL_W}" height="{outline_h}" '
            f'fill="none" stroke="#334155" stroke-width="1" rx="2"/>\n'
        )

        # ── Connector line from Letter column to sub-column ──────────
        pos = letter_region_pos.get(parent["name"])
        if pos is not None:
            parent_y, parent_h = pos
            letter_x = col_x(1)
            src_x = letter_x + COL_W
            src_y = parent_y + parent_h / 2
            dst_x = x + COL_W / 2
            dst_y = outline_y0 + outline_h + 4  # just below the column
            # L-shaped path: horizontal from source, then vertical down to arrow
            bend_y = dst_y + 20  # overshoot below, then come back up
            parts.append(
                f'<path d="M {src_x},{src_y} '
                f'L {dst_x},{src_y} '
                f'L {dst_x},{dst_y}" '
                f'fill="none" stroke="#475569" stroke-width="1.2" '
                f'stroke-dasharray="4,3" marker-end="url(#arrow)"/>\n'
            )

    # Arrow marker definition (points along path direction via orient="auto")
    parts.insert(3,
        '<defs><marker id="arrow" markerWidth="6" markerHeight="6" '
        'refX="5" refY="3" orient="auto">'
        f'<path d="M 0,0 L 6,3 L 0,6 Z" fill="#475569"/>'
        '</marker></defs>\n'
    )

    parts.append("    </svg>")
    svg_content = "".join(parts)

    # ── Legend (same format as sram_layout.html) ─────────────────────────
    legend_spans = []
    used_colors = set()
    for r in regions:
        c = r.get("color", "")
        if c and c not in used_colors:
            used_colors.add(c)
            fill = COLOR_MAP.get(c, "#475569")
            label = LEGEND_LABELS.get(c, c)
            legend_spans.append(
                f'<span style="display:inline-flex;align-items:center;gap:5px;'
                f'margin:4px 10px 4px 0">'
                f'<svg width="14" height="14"><rect width="14" height="14" rx="2" '
                f'fill="{fill}"/></svg>'
                f'<span style="font-size:12px;color:#cbd5e1">{label}</span></span>'
            )
    # Add bt_sub to legend
    if sub_columns:
        fill = COLOR_MAP["bt_sub"]
        label = LEGEND_LABELS["bt_sub"]
        legend_spans.append(
            f'<span style="display:inline-flex;align-items:center;gap:5px;'
            f'margin:4px 10px 4px 0">'
            f'<svg width="14" height="14"><rect width="14" height="14" rx="2" '
            f'fill="{fill}"/></svg>'
            f'<span style="font-size:12px;color:#cbd5e1">{label}</span></span>'
        )
    legend_html = "".join(legend_spans)

    # ── Exported constants table (same format as sram_layout.html) ───────
    table_rows = []
    for r in regions:
        for exp in r.get("exports", []):
            table_rows.append(
                f'<tr style="border-bottom:1px solid #1e293b">'
                f'<td style="padding:4px 10px;color:#7dd3fc;font-family:monospace">'
                f'{bank["module"]}</td>'
                f'<td style="padding:4px 10px;color:#fbbf24;font-family:monospace">'
                f'{exp["name"]}</td>'
                f'<td style="padding:4px 10px;color:#e2e8f0;font-family:monospace">'
                f'0x{r["offset"]:02X}</td>'
                f'<td style="padding:4px 10px;color:#64748b;font-family:monospace">'
                f'{r["source"]}</td>'
                f'<td style="padding:4px 10px;color:#94a3b8">{r["note"]}</td>'
                f'</tr>\n'
            )
        # Nested sub-fields
        sub_mod = r.get("sub_module")
        for sr in r.get("regions", []):
            for exp in sr.get("exports", []):
                mod_path = f'{bank["module"]}::{sub_mod}' if sub_mod else bank["module"]
                table_rows.append(
                    f'<tr style="border-bottom:1px solid #1e293b">'
                    f'<td style="padding:4px 10px;color:#7dd3fc;font-family:monospace">'
                    f'{mod_path}</td>'
                    f'<td style="padding:4px 10px;color:#fbbf24;font-family:monospace">'
                    f'{exp["name"]}</td>'
                    f'<td style="padding:4px 10px;color:#e2e8f0;font-family:monospace">'
                    f'+0x{sr["offset"]:02X}</td>'
                    f'<td style="padding:4px 10px;color:#64748b;font-family:monospace">'
                    f'{sr.get("source", "")}</td>'
                    f'<td style="padding:4px 10px;color:#94a3b8">{sr.get("note", "")}</td>'
                    f'</tr>\n'
                )
    exports_table = (
        '<details style="margin-top:16px" open>\n'
        '  <summary style="cursor:pointer;color:#94a3b8;font-size:13px;user-select:none">\n'
        f'    Exported Constants ({len(table_rows)} entries)\n'
        '  </summary>\n'
        '  <div style="overflow-x:auto;margin-top:12px">\n'
        '  <table style="border-collapse:collapse;width:100%;font-size:12px">\n'
        '    <thead><tr style="border-bottom:1px solid #334155">\n'
        '      <th style="text-align:left;padding:6px 10px;color:#64748b">Module</th>\n'
        '      <th style="text-align:left;padding:6px 10px;color:#64748b">Name</th>\n'
        '      <th style="text-align:left;padding:6px 10px;color:#64748b">Offset</th>\n'
        '      <th style="text-align:left;padding:6px 10px;color:#64748b">Source</th>\n'
        '      <th style="text-align:left;padding:6px 10px;color:#64748b">Note</th>\n'
        '    </tr></thead>\n'
        '    <tbody>\n'
        + "".join(table_rows) +
        '    </tbody>\n'
        '  </table>\n'
        '  </div>\n'
        '</details>\n'
    )

    # ── HTML wrapper (matches sram_layout.html exactly) ──────────────────
    html = f"""\
<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SF32LB52x ROM Config Layout</title>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
      background: #0f172a;
      color: #e2e8f0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      padding: 32px 24px;
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
  <h1>SF32LB52x ROM Config Layout</h1>
  <p class="subtitle">
    Generated from <code style="color:#7dd3fc">rom_config_layout.toml</code>.
    Hover over regions for details.  Offsets are relative to ROM config base.
  </p>

  <div class="diagram-wrap">
{svg_content}
  </div>

  <div class="legend">
    <div class="legend-title">Color Legend</div>
    {legend_html}
  </div>

  {exports_table}

</body>
</html>
"""

    OUT_PATH.write_text(html, encoding="utf-8")
    print(f"Written: {OUT_PATH}")


if __name__ == "__main__":
    main()
