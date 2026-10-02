#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
make -f Makefile.linux -j4
firmware=build-linux/app/MotorControlwithMicroROS.bin
# PID occupies sector 7, starting at 0x080C0000. Never erase/write this sector.
python3 - "$firmware" <<'PY'
import sys
from pathlib import Path
n = Path(sys.argv[1]).stat().st_size
if not 0 < n <= 0xC0000:
    raise SystemExit(f'Refusing to flash {n} bytes: firmware overlaps PID sector')
PY
backup="build-linux/backups/stm32-$(date +%Y%m%d-%H%M%S).bin"
mkdir -p build-linux/backups
bash tools/openocd.sh -c "init; reset halt; dump_image $backup 0x08000000 0x100000; program $firmware verify reset exit 0x08000000"
printf 'Backup: %s\n' "$backup"
