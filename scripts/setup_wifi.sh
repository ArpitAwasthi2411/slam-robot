#!/usr/bin/env bash
# Put the robot (Pi) and the phone on the same network for the Pathik app.
#
#   sudo bash setup_wifi.sh ap  [password]          robot makes its own Wi-Fi "SLAM-Robot" (10.42.0.1)
#   sudo bash setup_wifi.sh hotspot "<SSID>" "<pw>"  robot joins your phone's hotspot (app scans for it)
#   sudo bash setup_wifi.sh extra "<SSID>" "<pw>" [priority]  remember one more network (e.g. lab Wi-Fi);
#                                                    the Pi joins whichever known one is in range
#   sudo bash setup_wifi.sh status
#   sudo bash setup_wifi.sh off                      remove both profiles
#
# The ethernet cable link to the laptop (169.254.1.2) is not touched by any of these.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo"; exit 1; }
MODE="${1:-status}"
IFACE="${WIFI_IFACE:-wlan0}"

need_nm() {
  if ! command -v nmcli >/dev/null; then
    echo "NetworkManager is not installed. With internet on the Pi (e.g. join a hotspot first), run:"
    echo "  sudo apt update && sudo apt install -y network-manager"
    echo "then run this script again."
    exit 1
  fi
  systemctl enable --now NetworkManager >/dev/null 2>&1 || true
  # our own netplan Wi-Fi file (from "hotspot" mode before NetworkManager was installed) is replaced
  # by a NetworkManager profile: move it out of the way so netplan stops managing wlan0
  if [ -f /etc/netplan/60-robot-wifi.yaml ]; then
    mv /etc/netplan/60-robot-wifi.yaml /root/60-robot-wifi.yaml.bak
    netplan apply || true
    sleep 2
  fi
  # netplan must not also manage wlan0, or the two fight over it
  if grep -lqs "$IFACE" /etc/netplan/*.yaml 2>/dev/null && ! grep -qs "renderer: NetworkManager" /etc/netplan/*.yaml; then
    echo "WARNING: a netplan file configures $IFACE:"
    grep -ls "$IFACE" /etc/netplan/*.yaml
    echo "Remove the wifis: section for $IFACE from it (keep the ethernet part), run 'sudo netplan apply', then rerun."
    exit 1
  fi
  iw reg set IN 2>/dev/null || true      # Wi-Fi country (India): needed for access-point mode
  rfkill unblock wifi 2>/dev/null || true
  nmcli radio wifi on || true
  nmcli device set "$IFACE" managed yes 2>/dev/null || true
}

case "$MODE" in
  ap)
    PW="${2:-robot1234}"
    [ ${#PW} -ge 8 ] || { echo "Wi-Fi password must be at least 8 characters"; exit 1; }
    need_nm
    nmcli connection delete robot-ap >/dev/null 2>&1 || true
    nmcli connection modify robot-hotspot connection.autoconnect no >/dev/null 2>&1 || true
    nmcli connection add type wifi ifname "$IFACE" con-name robot-ap autoconnect yes ssid SLAM-Robot \
      802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
      ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method ignore \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" connection.autoconnect-priority 10
    nmcli connection up robot-ap
    echo
    echo "Robot Wi-Fi is up:  name SLAM-Robot   password $PW"
    echo "On the phone: join SLAM-Robot, open Pathik, tap Find robot (address 10.42.0.1)."
    echo "Note: in this mode the Pi has no internet, so commands use the offline parser (no LLM)."
    ;;
  hotspot)
    SSID="${2:?usage: setup_wifi.sh hotspot \"<SSID>\" \"<password>\"}"
    PW="${3:?usage: setup_wifi.sh hotspot \"<SSID>\" \"<password>\"}"
    if command -v nmcli >/dev/null; then
      need_nm
      nmcli connection modify robot-ap connection.autoconnect no >/dev/null 2>&1 || true
      nmcli connection down robot-ap >/dev/null 2>&1 || true
      nmcli connection delete robot-hotspot >/dev/null 2>&1 || true
      nmcli connection add type wifi ifname "$IFACE" con-name robot-hotspot autoconnect yes ssid "$SSID" \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" connection.autoconnect-priority 20
      nmcli connection up robot-hotspot
    else
      # plain Ubuntu Server: netplan + wpa_supplicant, no extra packages needed
      cat > /etc/netplan/60-robot-wifi.yaml <<EOF
network:
  version: 2
  wifis:
    $IFACE:
      dhcp4: true
      optional: true
      access-points:
        "$SSID":
          password: "$PW"
EOF
      chmod 600 /etc/netplan/60-robot-wifi.yaml
      netplan apply
    fi
    sleep 4
    IP=$(ip -4 -o addr show "$IFACE" | awk '{print $4}' | cut -d/ -f1 | head -1)
    echo
    echo "Joined '$SSID'. Robot address: ${IP:-not yet - check the hotspot is on}"
    echo "On the phone: keep the hotspot on, open Pathik, tap Find robot."
    ;;
  extra)
    SSID="${2:?usage: setup_wifi.sh extra \"<SSID>\" \"<password>\" [priority]}"
    PW="${3:?usage: setup_wifi.sh extra \"<SSID>\" \"<password>\" [priority]}"
    PRIO="${4:-5}"
    need_nm
    CON="robot-wifi-$(echo "$SSID" | tr -c 'A-Za-z0-9' '_' | cut -c1-20)"
    nmcli connection delete "$CON" >/dev/null 2>&1 || true
    nmcli connection add type wifi ifname "$IFACE" con-name "$CON" autoconnect yes ssid "$SSID" \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" connection.autoconnect-priority "$PRIO"
    echo "Remembered '$SSID' (priority $PRIO; the robot hotspot profile has 20, so the phone on the robot wins)."
    ;;
  status)
    ip -4 -br addr show "$IFACE" 2>/dev/null || echo "no $IFACE"
    command -v nmcli >/dev/null && nmcli -f NAME,TYPE,DEVICE,AUTOCONNECT connection show | grep -E "robot-|NAME" || true
    ;;
  off)
    command -v nmcli >/dev/null && { nmcli connection delete robot-ap robot-hotspot >/dev/null 2>&1 || true; }
    rm -f /etc/netplan/60-robot-wifi.yaml && netplan apply || true
    echo "Robot Wi-Fi profiles removed."
    ;;
  *)
    sed -n '2,9p' "$0"; exit 1 ;;
esac
