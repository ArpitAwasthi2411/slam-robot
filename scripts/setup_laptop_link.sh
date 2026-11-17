#!/usr/bin/env bash
# Laptop side of the direct ethernet cable to the Pi — makes 169.254.1.1 PERMANENT
# (no more "network unreachable" after unplugging the cable).
#   ./scripts/setup_laptop_link.sh                # auto-detect the USB-ethernet adapter
#   ./scripts/setup_laptop_link.sh enx006f0000324d
set -euo pipefail
# wired cards only, by NAME (eth*, enp*, eno*, enx* = USB adapters); never Wi-Fi (wl*). Matching the whole
# 'ip link' line picked the Wi-Fi card wlo1, because every line contains "link/ether".
IFACE="${1:-}"
if [ -z "$IFACE" ]; then
  for d in /sys/class/net/*; do
    n=$(basename "$d")
    case "$n" in eth*|en*) [ -d "$d/wireless" ] || { IFACE="$n"; break; } ;; esac
  done
fi
if [ -z "$IFACE" ]; then
  echo "No wired network card found on this laptop (only: $(ls /sys/class/net | tr '\n' ' '))."
  echo "Plug in a USB-to-Ethernet adapter and run this again (it shows up as enx...)."
  exit 1
fi
case "$IFACE" in wl*) echo "$IFACE is Wi-Fi, not the robot cable: give the wired one, e.g. enx..."; exit 1 ;; esac
echo "Using interface: $IFACE"
nmcli connection delete robot-link >/dev/null 2>&1 || true
nmcli connection add type ethernet ifname "$IFACE" con-name robot-link \
  ipv4.method manual ipv4.addresses 169.254.1.1/16 ipv4.never-default yes \
  ipv6.method ignore connection.autoconnect yes
nmcli connection up robot-link || echo "(cable unplugged? it will come up automatically when connected)"
echo
echo "Add this to ~/.bashrc so every terminal can see the robot's ROS topics:"
echo "  source /opt/ros/humble/setup.bash"
echo "  export ROS_DOMAIN_ID=0"
echo "  export ROS_LOCALHOST_ONLY=0"
echo
echo "SSH:        ssh arpit@169.254.1.2"
echo "Dashboard:  http://169.254.1.2:8080"
