#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export LD_LIBRARY_PATH="$PWD/.tools/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$PWD/.tools/root/usr/bin/openocd" -s "$PWD/.tools/root/usr/share/openocd/scripts" \
    -f interface/stlink.cfg -f target/stm32f7x.cfg \
    -c 'gdb_port disabled; tcl_port disabled; telnet_port disabled' "$@"
