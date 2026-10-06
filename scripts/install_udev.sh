#!/usr/bin/env bash
# Run on the Pi:  sudo ./scripts/install_udev.sh
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
cp "$here/udev/99-robot.rules" /etc/udev/rules.d/99-robot.rules
udevadm control --reload-rules
udevadm trigger
# ModemManager probes /dev/ttyACM* devices and can swallow ESP32 data for a few seconds
if systemctl is-active --quiet ModemManager 2>/dev/null; then
  systemctl disable --now ModemManager
  echo "ModemManager disabled"
fi
# so 'arpit' can open serial ports without sudo
if [ -n "${SUDO_USER:-}" ]; then usermod -aG dialout "$SUDO_USER"; fi
sleep 1
ls -l /dev/rplidar /dev/esp32 2>/dev/null || echo "Plug in the LiDAR and ESP32, then: ls -l /dev/rplidar /dev/esp32"
echo "Done. Log out/in once for the dialout group to apply."
