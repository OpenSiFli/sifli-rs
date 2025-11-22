#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <ELF> [probe-rs extra args...]" >&2
    exit 64
fi

elf_path="$1"
shift

if [ ! -f "$elf_path" ]; then
    echo "找不到 ELF 文件: $elf_path" >&2
    exit 66
fi

sftool_port="${SIFLI_SFTOOL_PORT:-}"
if [ -z "$sftool_port" ]; then
    cat >&2 <<'EOF'
请先设置环境变量 SIFLI_SFTOOL_PORT，例如：
    export SIFLI_SFTOOL_PORT=/dev/tty.usbmodemXXXX
EOF
    exit 65
fi

echo "[runner] 使用 sftool 烧录: $elf_path"
if [ -n "${SIFLI_SFTOOL_EXTRA_ARGS:-}" ]; then
    # shellcheck disable=SC2086
    sftool -c SF32LB52 --port "$sftool_port" \
        ${SIFLI_SFTOOL_EXTRA_ARGS} write_flash --verify "$elf_path"
else
    sftool -c SF32LB52 --port "$sftool_port" \
        write_flash --verify "$elf_path"
fi

echo "[runner] 启动 probe-rs attach"
if [ -n "${SIFLI_PROBE_EXTRA_ARGS:-}" ]; then
    # shellcheck disable=SC2086
    probe-rs attach --chip SF32LB52 \
        ${SIFLI_PROBE_EXTRA_ARGS} "$elf_path" "$@"
else
    probe-rs attach --chip SF32LB52 "$elf_path" "$@"
fi
