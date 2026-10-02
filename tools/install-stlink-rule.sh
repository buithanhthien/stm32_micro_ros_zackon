#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $EUID -ne 0 ]]; then
    exec sudo bash "$PWD/tools/install-stlink-rule.sh"
fi
install -m 644 tools/60-stm32-stlink.rules /etc/udev/rules.d/60-stm32-stlink.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=0483 --attr-match=idProduct=374b
udevadm settle
