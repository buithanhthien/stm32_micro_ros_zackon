#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $EUID -ne 0 ]]; then
    exec sudo bash "$script_dir/install-agent-service.sh"
fi
backup_dir="/var/backups/stm32-microros/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
cp -a /etc/systemd/system/microros_agent.service "$backup_dir/"
cp -a /etc/systemd/system/stm32_reset.service "$backup_dir/"
systemd-analyze verify "$script_dir/microros_agent.service"
install -m 644 "$script_dir/microros_agent.service" /etc/systemd/system/microros_agent.service
systemctl daemon-reload
systemctl disable --now stm32_reset.service
systemctl enable microros_agent.service
systemctl restart microros_agent.service
systemctl status microros_agent.service --no-pager
