#!/usr/bin/env bash
# Why does the motor board (ESP32) or the app drop for a few seconds and come back?
#
#   bash tools/link_doctor.sh            # 1) quick check of everything that can cause drops
#   bash tools/link_doctor.sh --watch    # 2) leave running while you DRIVE the robot; Ctrl+C to stop
#   bash tools/link_doctor.sh --fix      # safe software fixes: Wi-Fi power saving off, stop port-grabbing services
#
# --watch prints one timestamped line per event and saves them to ~/link_log.txt.
# Send that file (or a photo of the screen) to Claude after the driving test.
set -u
LOG="$HOME/link_log.txt"
ts() { date '+%H:%M:%S'; }

throttled_text() {      # decode vcgencmd get_throttled bits
  local v=$(( $1 ))
  local out=""
  (( v & 0x1 ))     && out+="UNDER-VOLTAGE NOW; "
  (( v & 0x4 ))     && out+="CPU THROTTLED NOW; "
  (( v & 0x10000 )) && out+="under-voltage happened since boot; "
  (( v & 0x40000 )) && out+="throttling happened since boot; "
  echo "${out:-ok}"
}

check() {
  echo "== Pi power =="
  if command -v vcgencmd >/dev/null; then
    t=$(vcgencmd get_throttled | cut -d= -f2)
    echo "get_throttled=$t -> $(throttled_text "$t")"
    echo "  (any under-voltage = the Pi's 5 V supply dips; USB devices then reset. Fix the Pi supply first.)"
    vcgencmd measure_temp
  else
    echo "vcgencmd not found (sudo apt install libraspberrypi-bin)"
  fi

  echo; echo "== Serial ports =="
  ls -l /dev/ttyUSB* /dev/ttyACM* /dev/esp32 /dev/rplidar 2>/dev/null
  echo; echo "== USB devices and the power each one asks for (Pi 4 total for all ports: 1.2 A) =="
  lsusb 2>/dev/null
  lsusb -v 2>/dev/null | awk '/^Bus /{dev=$0} /MaxPower/{print "  " $2 "  " dev}' | sort -u

  echo; echo "== Programs that grab USB-serial ports =="
  for s in ModemManager brltty brltty-udev; do
    st=$(systemctl is-active "$s" 2>/dev/null)
    [ "$st" = "active" ] && echo "  $s is RUNNING -> sudo systemctl disable --now $s" || echo "  $s: ${st:-not installed} (good)"
  done
  echo "  processes holding the ports:"
  for p in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$p" ] && echo "    $p: $(fuser "$p" 2>/dev/null | xargs -n1 -I{} ps -o comm= -p {} 2>/dev/null | tr '\n' ' ')"; done

  echo; echo "== Wi-Fi (phone <-> Pi link) =="
  if command -v iw >/dev/null; then
    for i in $(iw dev | awk '/Interface/{print $2}'); do
      echo "  $i: $(iw dev "$i" get power_save 2>/dev/null)   <- should be 'off'"
    done
  fi

  echo; echo "== CPU load (4 cores: above ~3.5 the Python nodes start to lag) =="
  uptime

  echo; echo "== USB events in the kernel log since boot =="
  sudo dmesg 2>/dev/null | grep -iE 'usb .*(disconnect|reset|new )|cp210|ch341|ch34x|ftdi|over-current|under-voltage|voltage' | tail -n 25 || echo "  (run with sudo to read dmesg)"

  echo; echo "== Link events from the robot service (last boot) =="
  journalctl -u robot -b --no-pager 2>/dev/null | grep -E 'LINK EVENT|ESP32: READY|Serial|no ODM' | tail -n 25 || true
}

watch_mode() {
  echo "Watching. Drive the robot now (5-10 min, include fast starts, reversing and turns). Ctrl+C to stop."
  echo "Saving to $LOG"
  : > "$LOG"
  trap 'kill 0 2>/dev/null; echo; echo "Saved: $LOG"; exit 0' INT TERM
  ( sudo dmesg -w --time-format iso 2>/dev/null | grep --line-buffered -iE 'usb .*(disconnect|reset|new )|cp210|ch34|ftdi|over-current|voltage' \
      | while read -r l; do echo "$(ts) KERNEL  $l"; done ) | tee -a "$LOG" &
  ( journalctl -u robot -f -n 0 2>/dev/null | grep --line-buffered -E 'LINK EVENT|ESP32: READY|Serial|no ODM' \
      | while read -r l; do echo "$(ts) ROBOT   ${l#*]: }"; done ) | tee -a "$LOG" &
  ( last=""
    while true; do
      if command -v vcgencmd >/dev/null; then
        t=$(vcgencmd get_throttled | cut -d= -f2)
        now=$(( t & 0x5 ))
        [ "$now" != "0" ] && [ "$now" != "$last" ] && echo "$(ts) POWER   $(throttled_text "$t")"
        last=$now
      fi
      l=$(cut -d' ' -f1 /proc/loadavg)
      if awk -v l="$l" 'BEGIN{exit !(l>3.5)}'; then
        [ -z "${hot:-}" ] && echo "$(ts) CPU     load $l (Pi overloaded)"; hot=1
      else hot=""; fi
      sleep 1
    done ) | tee -a "$LOG" &
  wait
}

fix() {
  echo "Turning Wi-Fi power saving off (it makes the Pi's Wi-Fi nap: app link lags/drops)"
  printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf >/dev/null
  for i in $(iw dev 2>/dev/null | awk '/Interface/{print $2}'); do sudo iw dev "$i" set power_save off 2>/dev/null; done
  for s in ModemManager brltty brltty-udev; do
    if systemctl is-active --quiet "$s" 2>/dev/null; then sudo systemctl disable --now "$s" && echo "Stopped $s (it grabs USB-serial ports)"; fi
  done
  sudo systemctl reload NetworkManager 2>/dev/null || true
  echo "Done. Hardware causes (power, motor noise) can't be fixed by software: run --watch while driving."
}

case "${1:-}" in
  --watch) watch_mode ;;
  --fix) fix ;;
  *) check ;;
esac
