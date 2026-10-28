#!/usr/bin/env bash
# One-shot health check while the robot runs: heat, power, CPU hogs, data rates, link events.
#   bash ~/slam-robot/tools/robot_check.sh            (takes ~30 s; robot service must be running)
# Send the whole output (or a photo of it) to Claude.
set -u
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ros2_ws/install/setup.bash 2>/dev/null

echo "== Heat + power =="
if command -v vcgencmd >/dev/null; then
  vcgencmd measure_temp; vcgencmd measure_clock arm; vcgencmd get_throttled
  echo "  (temp > 80 C or throttled != 0x0 = the Pi slows down: fit a heatsink + fan / fix the 5 V supply)"
fi
echo; echo "== CPU load and the biggest users =="
uptime
ps -eo pcpu,comm --sort=-pcpu | head -8

echo; echo "== Data rates (10 s each; want /odom ~50, /scan ~7, /imu ~50 if the MPU works) =="
for t in /odom /scan /imu; do
  r=$(timeout 10 ros2 topic hz "$t" 2>/dev/null | grep -m1 -o 'average rate: [0-9.]*')
  echo "  $t: ${r:-NO DATA}"
done
echo; echo "== Robot position (map -> base_link) =="
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m2 -E "Translation|Could not|does not exist" || echo "  no map->base_link (SLAM not publishing?)"

echo; echo "== Link / navigator events since the service started =="
journalctl -u robot -b --no-pager 2>/dev/null \
  | grep -E "LINK EVENT|rebooted|position lost|position back|POSE_LOST|SLAM correction|no ODM|sllidar.*(rror|timeout)" | tail -20
echo; echo "== USB / power events in the kernel log =="
sudo -n dmesg 2>/dev/null | grep -iE "under-voltage|over-current|usb .*disconnect" | tail -8 || echo "  (run with sudo to see)"
