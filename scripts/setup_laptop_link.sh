#!/usr/bin/env bash
# Laptop side of the direct ethernet cable to the Pi — makes 169.254.1.1 PERMANENT
# (no more "network unreachable" after unplugging the cable).
#   ./scripts/setup_laptop_link.sh                # auto-detect the USB-ethernet adapter
#   ./scripts/setup_laptop_link.sh enx006f0000324d
set -euo pipefail
IFACE="${1:-$(ip -o link show | awk -F': ' '/enx|eth|enp/{print $2; exit}')}"
[ -n "$IFACE" ] || { echo "No ethernet interface found"; exit 1; }
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
