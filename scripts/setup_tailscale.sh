#!/usr/bin/env bash
# Reach the robot from anywhere (other lab, other floor, home) through Tailscale.
#
#   sudo bash setup_tailscale.sh            install + log in, robot name "slam-robot"
#   sudo bash setup_tailscale.sh status
#
# After this, a phone/laptop logged into the SAME Tailscale account opens Pathik at
#   http://slam-robot:8080        (or the 100.x.y.z address printed below)
# and SSH works with: ssh arpit@slam-robot
# It works over any internet link the Pi has: lab Wi-Fi, a phone hotspot riding on the robot, etc.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo"; exit 1; }
NAME="${TS_HOSTNAME:-slam-robot}"

case "${1:-up}" in
  up)
    if ! command -v tailscale >/dev/null; then
      echo "Installing Tailscale (needs internet on the Pi: join a hotspot first with setup_wifi.sh hotspot)…"
      curl -fsSL https://tailscale.com/install.sh | sh
    fi
    systemctl enable --now tailscaled
    # --ssh lets you ssh in through Tailscale even if the Pi's own sshd config changes;
    # the login URL printed here must be opened once in a browser (any device).
    tailscale up --hostname="$NAME" --ssh --accept-dns=false
    echo
    echo "Robot on Tailscale:  name $NAME   address $(tailscale ip -4 | head -1)"
    echo "Phone: install Tailscale, log in with the same account, then in Pathik type: $NAME"
    echo "(MagicDNS must be on in the Tailscale admin page, otherwise type the 100.x address.)"
    ;;
  status)
    tailscale status || true
    ;;
  *)
    sed -n '2,11p' "$0"; exit 1 ;;
esac
