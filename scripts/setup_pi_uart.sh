#!/usr/bin/env bash
# Make the Pi 4's GPIO UART (header pin 8 = TXD, pin 10 = RXD) free for the ESP32 (firmware v2.6).
#   sudo bash ~/slam-robot/scripts/setup_pi_uart.sh      then reboot
# What it does (Ubuntu 22.04, files in /boot/firmware, backups as *.bak-pathik):
#   1. config.txt: enable_uart=1 + dtoverlay=disable-bt  -> the full PL011 UART (/dev/ttyAMA0) on pins 8/10
#      (Bluetooth is switched off; the robot doesn't use it)
#   2. cmdline.txt: remove console=serial0/ttyAMA0/ttyS0  -> the kernel stops printing its log to the ESP32
#   3. no login prompt on the UART (serial-getty), no Bluetooth UART attach (hciuart)
#   4. udev rule: /dev/esp32 -> /dev/ttyAMA0, readable by the dialout group; user added to dialout
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
here="$(cd "$(dirname "$0")/.." && pwd)"
boot=/boot/firmware
[ -d "$boot" ] || boot=/boot
cfg="$boot/config.txt"; cmd="$boot/cmdline.txt"
[ -f "$cfg" ] && [ -f "$cmd" ] || { echo "no $cfg / $cmd: is this the Pi?"; exit 1; }

cp -n "$cfg" "$cfg.bak-pathik"; cp -n "$cmd" "$cmd.bak-pathik"

# 1. config.txt (append once, at the end, under [all])
if ! grep -q '^# pathik: ESP32 on the GPIO UART' "$cfg"; then
  printf '\n[all]\n# pathik: ESP32 on the GPIO UART (pins 8/10)\nenable_uart=1\ndtoverlay=disable-bt\n' >> "$cfg"
  echo "config.txt: enable_uart=1, dtoverlay=disable-bt added"
else
  echo "config.txt: already set"
fi

# 2. cmdline.txt is ONE line: drop only the serial console entries
sed -i -E 's/ ?console=(serial0|serial1|ttyAMA0|ttyS0),[0-9]+//g' "$cmd"
echo "cmdline.txt now: $(cat "$cmd")"

# 3. nothing else may read the port
for u in serial-getty@ttyAMA0 serial-getty@ttyS0 hciuart btuart; do
  systemctl disable --now "$u.service" >/dev/null 2>&1 || true
  systemctl mask "$u.service" >/dev/null 2>&1 || true
done
echo "serial-getty / hciuart: disabled"

# 4. /dev/esp32 + permissions
cp "$here/udev/98-esp32-uart.rules" /etc/udev/rules.d/98-esp32-uart.rules
udevadm control --reload-rules
if [ -n "${SUDO_USER:-}" ]; then usermod -aG dialout "$SUDO_USER"; fi

echo
echo "Done. Reboot now:  sudo reboot"
echo "After the reboot:  ls -l /dev/esp32 /dev/ttyAMA0     (esp32 -> ttyAMA0)"
echo "                   python3 -m serial.tools.miniterm /dev/ttyAMA0 115200    (ODM lines; Ctrl+] quits)"
echo "Undo: copy $cfg.bak-pathik and $cmd.bak-pathik back, remove /etc/udev/rules.d/98-esp32-uart.rules."
