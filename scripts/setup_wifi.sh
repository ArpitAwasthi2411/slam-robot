#!/usr/bin/env bash
# Put the robot (Pi) and the phone on the same network for the Pathik app.
#
#   sudo bash setup_wifi.sh ap  [password]          robot makes its own Wi-Fi "SLAM-Robot" (10.42.0.1)
#   sudo bash setup_wifi.sh hotspot "<SSID>" "<pw>"  robot joins your phone's hotspot (app scans for it)
#   sudo bash setup_wifi.sh extra "<SSID>" "<pw>" [priority]  remember one more network (e.g. lab Wi-Fi);
#                                                    the Pi joins whichever known one is in range
#   sudo bash setup_wifi.sh fallback "<SSID>" "<pw>" the network to use when the hotspot is gone (priority 10);
#                                                    the robot switches BACK to the hotspot as soon as it is seen
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
  # (e.g. the Pi image's 50-cloud-init.yaml): drop only its wifis: part, keep ethernet, keep a backup
  for f in $(grep -ls "wifis:" /etc/netplan/*.yaml 2>/dev/null); do
    cp "$f" "/root/$(basename "$f").bak"
    python3 - "$f" <<'PY'
import sys, yaml
p = sys.argv[1]
d = yaml.safe_load(open(p)) or {}
d.get('network', {}).pop('wifis', None)
yaml.safe_dump(d, open(p, 'w'), default_flow_style=False)
PY
    chmod 600 "$f"
    echo "Removed the Wi-Fi part of $f (backup: /root/$(basename "$f").bak)"
    # stop cloud-init from writing it back at the next boot
    [ -d /etc/cloud/cloud.cfg.d ] && echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
    netplan apply || true
    sleep 3
  done
  iw reg set IN 2>/dev/null || true      # Wi-Fi country (India): needed for access-point mode
  rfkill unblock wifi 2>/dev/null || true
  nmcli radio wifi on || true
  nmcli device set "$IFACE" managed yes 2>/dev/null || true
}

# NetworkManager never leaves a working network for a higher-priority one that comes back. This timer
# (every 30 s) switches to the robot hotspot as soon as it is visible again, so the hotspot is always first.
install_prefer() {
  cat > /usr/local/sbin/robot-wifi-prefer <<'SH'
#!/bin/sh
# installed by slam-robot/scripts/setup_wifi.sh: come back to the robot hotspot when it is in range
nmcli -t -f NAME connection show robot-hotspot >/dev/null 2>&1 || exit 0
nmcli -t -f NAME connection show --active | grep -qx robot-hotspot && exit 0
SSID=$(nmcli -g 802-11-wireless.ssid connection show robot-hotspot)
nmcli -t -f SSID device wifi list --rescan yes 2>/dev/null | grep -Fqx "$SSID" || exit 0
logger -t robot-wifi "hotspot '$SSID' is back: switching to it"
nmcli connection up robot-hotspot >/dev/null 2>&1 || true
SH
  chmod 755 /usr/local/sbin/robot-wifi-prefer
  cat > /etc/systemd/system/robot-wifi-prefer.service <<'U'
[Unit]
Description=Switch back to the robot hotspot when it is in range
[Service]
Type=oneshot
ExecStart=/usr/local/sbin/robot-wifi-prefer
U
  cat > /etc/systemd/system/robot-wifi-prefer.timer <<'U'
[Unit]
Description=Check every 30 s for the robot hotspot
[Timer]
OnBootSec=40
OnUnitActiveSec=30
[Install]
WantedBy=timers.target
U
  systemctl daemon-reload
  systemctl enable --now robot-wifi-prefer.timer >/dev/null
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
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" connection.autoconnect-priority 20 \
        connection.autoconnect-retries 0          # keep trying forever (0 = no limit)
      install_prefer
      nmcli connection up robot-hotspot || echo "Hotspot '$SSID' not in range yet: the robot joins it when it appears." 
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
  extra|fallback)
    SSID="${2:?usage: setup_wifi.sh $MODE \"<SSID>\" \"<password>\" [priority]}"
    PW="${3:?usage: setup_wifi.sh $MODE \"<SSID>\" \"<password>\" [priority]}"
    if [ "$MODE" = fallback ]; then PRIO="${4:-10}"; else PRIO="${4:-5}"; fi
    [ "$PRIO" -lt 20 ] || { echo "priority must be below 20 (the robot hotspot's) so the hotspot stays first"; exit 1; }
    need_nm
    CON="robot-wifi-$(echo "$SSID" | tr -c 'A-Za-z0-9' '_' | cut -c1-20)"
    nmcli connection delete "$CON" >/dev/null 2>&1 || true
    nmcli connection add type wifi ifname "$IFACE" con-name "$CON" autoconnect yes ssid "$SSID" \
      wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PW" connection.autoconnect-priority "$PRIO"
    echo "Remembered '$SSID' (priority $PRIO; the robot hotspot profile has 20, so the phone on the robot wins)."
    nmcli -t -f NAME connection show robot-hotspot >/dev/null 2>&1 && install_prefer && \
      echo "When the hotspot comes back, the robot switches back to it within ~30 s."
    ;;
  status)
    ip -4 -br addr show "$IFACE" 2>/dev/null || echo "no $IFACE"
    command -v nmcli >/dev/null && nmcli -f NAME,TYPE,DEVICE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show | grep -E "robot-|NAME" || true
    systemctl is-active --quiet robot-wifi-prefer.timer 2>/dev/null && echo "hotspot-first timer: on" || echo "hotspot-first timer: off" 
    ;;
  off)
    command -v nmcli >/dev/null && { nmcli connection delete robot-ap robot-hotspot >/dev/null 2>&1 || true; }
    systemctl disable --now robot-wifi-prefer.timer >/dev/null 2>&1 || true
    rm -f /etc/netplan/60-robot-wifi.yaml && netplan apply || true
    echo "Robot Wi-Fi profiles removed."
    ;;
  *)
    sed -n '2,13p' "$0"; exit 1 ;;
esac
