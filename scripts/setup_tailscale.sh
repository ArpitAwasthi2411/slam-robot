#!/usr/bin/env bash
# Reach the robot from anywhere (other lab, other floor, home, mobile data) through Tailscale.
#
#   sudo bash setup_tailscale.sh               install + log in, robot name "slam-robot", start on every boot
#   sudo TS_AUTHKEY=tskey-auth-... bash setup_tailscale.sh     same, without a browser (headless)
#   sudo bash setup_tailscale.sh status        is it up? which names/addresses work? is the app answering?
#   sudo bash setup_tailscale.sh funnel on     ALSO a public https URL for phones WITHOUT Tailscale (logins required)
#   sudo bash setup_tailscale.sh funnel off    back to private (only phones in your Tailscale account)
#   sudo bash setup_tailscale.sh funnel status
#
# Private path (recommended): a phone with the Tailscale app, logged into the SAME account, opens Pathik and
# types  slam-robot  (MagicDNS) or the 100.x.y.z address. Works on mobile data; the traffic is encrypted.
# The auth key can also come from ~/robot.env (line TS_AUTHKEY=...). Never commit it. Full guide:
# docs/STEPS_remote_control.md
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo"; exit 1; }
NAME="${TS_HOSTNAME:-slam-robot}"
ROBOT_USER="${SUDO_USER:-arpit}"
HOME_DIR="${PATHIK_HOME:-$(getent passwd "$ROBOT_USER" | cut -d: -f6)}"     # PATHIK_HOME: tests
HOME_DIR="${HOME_DIR:-/home/$ROBOT_USER}"
FLAG="$HOME_DIR/maps/.public_url"          # the robot's dashboard reads this: public mode on/off
PORT="${PATHIK_PORT:-8080}"

authkey() {                                # env first, then ~/robot.env; never printed
  if [ -n "${TS_AUTHKEY:-}" ]; then echo "$TS_AUTHKEY"; return; fi
  [ -f "$HOME_DIR/robot.env" ] && sed -n 's/^TS_AUTHKEY=//p' "$HOME_DIR/robot.env" | tr -d '"'"'" | head -1
}
dnsname() { tailscale status --json 2>/dev/null | python3 -c 'import json,sys; print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))' 2>/dev/null || true; }
backend() { tailscale status --json 2>/dev/null | python3 -c 'import json,sys; print(json.load(sys.stdin).get("BackendState",""))' 2>/dev/null || true; }
ping_app() { curl -s -m 3 "http://127.0.0.1:$PORT/api/ping" | grep -q '"api"' && echo yes || echo no; }

case "${1:-up}" in
  up)
    if ! command -v tailscale >/dev/null; then
      echo "Installing Tailscale (needs internet on the Pi: join a hotspot first with setup_wifi.sh hotspot)…"
      curl -fsSL https://tailscale.com/install.sh | sh
    fi
    systemctl enable --now tailscaled          # starts on every boot; the login is remembered
    KEY="$(authkey)"
    # --ssh: ssh in through Tailscale even if the Pi's own sshd config changes.
    # --accept-dns=false: keep the Pi's own DNS (MagicDNS names still resolve on the phones).
    if [ -n "$KEY" ]; then
      echo "Logging in with the auth key (headless)…"
      tailscale up --reset --auth-key="$KEY" --hostname="$NAME" --ssh --accept-dns=false
    else
      echo "Open the login URL printed below on any device (once). Headless instead: TS_AUTHKEY=... (see the guide)."
      tailscale up --reset --hostname="$NAME" --ssh --accept-dns=false
    fi
    for _ in $(seq 1 30); do [ "$(backend)" = Running ] && break; sleep 1; done
    if [ "$(backend)" != Running ]; then echo "!! Tailscale is not running yet (state: $(backend)). Run: sudo tailscale up"; exit 1; fi
    IP="$(tailscale ip -4 | head -1)"; DNS="$(dnsname)"
    echo
    echo "Robot on Tailscale (starts by itself after every reboot):"
    echo "  name     $NAME          (needs MagicDNS: Tailscale admin page → DNS → Enable MagicDNS)"
    echo "  full     ${DNS:-?}"
    echo "  address  $IP          (always works)"
    echo "Phone: install Tailscale, log in with the SAME account, then in Pathik type:  $NAME   (or $IP)"
    ;;
  status)
    command -v tailscale >/dev/null || { echo "Tailscale not installed: sudo bash $0"; exit 1; }
    echo "tailscaled service : $(systemctl is-active tailscaled) / starts at boot: $(systemctl is-enabled tailscaled)"
    echo "state              : $(backend)"
    echo "name / full name   : $NAME / $(dnsname)"
    echo "address            : $(tailscale ip -4 2>/dev/null | head -1)"
    echo "Pathik app answers : $(ping_app)  (http://127.0.0.1:$PORT/api/ping)"
    if [ -f "$FLAG" ]; then echo "PUBLIC (Funnel)    : ON  -> $(cat "$FLAG")"; else echo "PUBLIC (Funnel)    : off"; fi
    echo; tailscale status 2>/dev/null | head -15 || true
    ;;
  funnel)
    case "${2:-status}" in
      on)
        # Logins must exist before the robot faces the internet: the first account becomes admin.
        USERS="$HOME_DIR/maps/users.json"
        if [ ! -f "$USERS" ] || ! python3 - "$USERS" <<'PY'
import json, sys
u = json.load(open(sys.argv[1])).get('users', {})
sys.exit(0 if any(v.get('role') == 'admin' and v.get('status') == 'active' for v in u.values()) else 1)
PY
        then
          echo "!! No admin account yet. Create it in the Pathik app on the robot's own network first,"
          echo "   otherwise the first stranger to open the public URL would become admin. Not turning Funnel on."
          exit 1
        fi
        [ "$(backend)" = Running ] || { echo "!! Tailscale is not up: sudo bash $0"; exit 1; }
        echo "Turning on Funnel (needs HTTPS certificates + the funnel attribute in the tailnet policy;"
        echo "Tailscale prints a link to enable them if missing)…"
        tailscale funnel --bg "$PORT"
        URL="https://$(dnsname)"
        mkdir -p "$(dirname "$FLAG")"
        echo "$URL" > "$FLAG"
        chown "$ROBOT_USER": "$FLAG" 2>/dev/null || true
        echo
        echo "PUBLIC: anyone on the internet can open  $URL  (they still need a login)."
        echo "Phones without Tailscale: in Pathik type the full URL above. Turn it off after the demo:"
        echo "  sudo bash $0 funnel off"
        ;;
      off)
        tailscale funnel --https=443 off 2>/dev/null || tailscale funnel reset 2>/dev/null || true
        rm -f "$FLAG"
        echo "Funnel off: the robot is private again (Tailscale phones and the local network only)."
        ;;
      status)
        tailscale funnel status 2>/dev/null || echo "no Funnel"
        [ -f "$FLAG" ] && echo "app public flag: ON ($(cat "$FLAG"))" || echo "app public flag: off"
        ;;
      *) echo "usage: $0 funnel on|off|status"; exit 1 ;;
    esac
    ;;
  *)
    sed -n '2,15p' "$0"; exit 1 ;;
esac
