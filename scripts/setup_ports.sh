#!/usr/bin/env bash
# Give the LiDAR and the ESP32 fixed names: /dev/rplidar and /dev/esp32.
# Works for every case (CP2102 on both, or CH340/CH9102 on the ESP32).
# Run on the Pi:   sudo ~/slam-robot/scripts/setup_ports.sh
# It asks you to plug the two devices in one at a time. Always use the same Pi USB sockets afterwards.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo"; exit 1; }

list() { ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true; }
prop() { udevadm info -q property -n "$1" | sed -n "s/^$2=//p"; }

echo "1) UNPLUG the ESP32 USB. Keep ONLY the LiDAR plugged in. Then press Enter."
read -r _
sleep 1
L=$(list)
[ "$(echo "$L" | grep -c .)" = 1 ] || { echo "Expected exactly 1 serial device, found: $L"; exit 1; }
LIDAR=$L
echo "   LiDAR = $LIDAR"

echo "2) Now PLUG IN the ESP32 (leave the LiDAR in). Wait 2 s, then press Enter."
read -r _
sleep 1
ESP=$(comm -13 <(echo "$L" | sort) <(list | sort) | head -1)
[ -n "$ESP" ] || { echo "No new device appeared. Check the ESP32 cable (data cable, not charge-only)."; exit 1; }
echo "   ESP32 = $ESP"

LV=$(prop "$LIDAR" ID_VENDOR_ID); LP=$(prop "$LIDAR" ID_MODEL_ID); LPATH=$(prop "$LIDAR" ID_PATH)
EV=$(prop "$ESP" ID_VENDOR_ID);   EP=$(prop "$ESP" ID_MODEL_ID);   EPATH=$(prop "$ESP" ID_PATH)
echo "   LiDAR chip $LV:$LP  socket $LPATH"
echo "   ESP32 chip $EV:$EP  socket $EPATH"

R=/etc/udev/rules.d/99-robot.rules
{
  echo "# written by setup_ports.sh $(date -I)"
  if [ "$LV:$LP" != "$EV:$EP" ]; then
    echo "# different USB chips -> match by chip (any socket works)"
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$LV\", ATTRS{idProduct}==\"$LP\", SYMLINK+=\"rplidar\", MODE=\"0666\""
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$EV\", ATTRS{idProduct}==\"$EP\", SYMLINK+=\"esp32\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\""
  else
    echo "# same USB chip on both -> match by Pi USB socket (keep each in its socket!)"
    echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$LPATH\", SYMLINK+=\"rplidar\", MODE=\"0666\""
    echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$EPATH\", SYMLINK+=\"esp32\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\""
  fi
} > "$R"
[ "$LV:$LP" = "$EV:$EP" ] && echo "   Same chip on both: LABEL the two Pi USB sockets now. Swapping them swaps the names."

udevadm control --reload-rules
udevadm trigger
if systemctl is-active --quiet ModemManager 2>/dev/null; then systemctl disable --now ModemManager; fi
[ -n "${SUDO_USER:-}" ] && usermod -aG dialout "$SUDO_USER"
sleep 2
ls -l /dev/rplidar /dev/esp32 && echo "OK. Rules saved in $R"
