#!/usr/bin/env bash
# Download Ubuntu Noble tools locally; does not require sudo or install packages globally.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
project_root="$PWD"
mkdir -p .tools/debs .tools/root
cd .tools/debs
apt-get download gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  libnewlib-dev libstdc++-arm-none-eabi-dev openocd libhidapi-hidraw0 \
  libjaylink0 libftdi1-2 libgpiod2t64 libcapstone4 libjim0.82t64
for package in ./*.deb; do dpkg-deb -x "$package" "$project_root/.tools/root"; done
cd "$project_root/.tools/root/usr/lib/arm-none-eabi"
[[ -e include ]] || ln -s ../../include/newlib include
[[ -e lib ]] || ln -s newlib lib
"$project_root/.tools/root/usr/bin/arm-none-eabi-gcc" --version
