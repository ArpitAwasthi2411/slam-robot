# Next run on the robot: how to test each phase

One section per phase of the October 2026 cloud session. Each says what changed, how to put it on the robot,
and what to check. Do them in order; every check takes a few minutes. Code first:
```bash
cd ~/slam-robot && git pull          # or copy the zip as in STEPS_esp32s3_lab.md step 5
```
```bash
sudo systemctl stop robot && cd ~/ros2_ws && rm -rf build/lidar_robot install/lidar_robot && rm -rf src/lidar_robot && cp -r ~/slam-robot/ros2_ws/src/lidar_robot src/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select lidar_robot && sudo systemctl start robot
```

## Phase 0: review fixes (docs/REVIEW_2026-10.md)

Nothing new to set up. Checks:
1. **Bad numbers can't reach the motors.** Laptop, with an operator login token from the app (or the
   simulator): `curl -X POST http://slam-robot:8080/api/cmd -H "Authorization: Bearer <token>" -d '{"v":"nan","w":0}'`
   → `400 ... must be a finite number`; `journalctl -u robot -f` shows no bridge error, the wheels don't move.
2. **"stop" ends Map by itself.** App → Map → Map by itself, then type or say "stop": the robot stops and does
   not drive to a new area 2 s later.
3. **setup_ports.sh in UART mode.** Only if you want `/dev/rplidar`:
   `sudo bash ~/slam-robot/scripts/setup_ports.sh` → it says "ESP32 is on the GPIO UART", asks for the LiDAR only,
   and `ls -l /dev/rplidar /dev/esp32` shows both (esp32 → ttyAMA0 untouched).
4. **serial_probe speeds.** `python3 ~/slam-robot/tools/serial_probe.py --port /dev/ttyAMA0 --vel 200 200`
   (robot on blocks): the measured wheel speed is now close to 200 mm/s (it used to read ~5 % high).

## Phase 1: remote control from another network (docs/STEPS_remote_control.md)

What changed: `setup_tailscale.sh` (auth key from `~/robot.env`, starts on boot, `status`, `funnel on|off`),
`setup_wifi.sh` (hotspot first + `fallback` network, a 30 s timer switches back to the hotspot), the dashboard
knows if a request came over the LAN, Tailscale or the public Funnel link (shorter sessions, login limits, no
first-admin sign-up over the public link), the app shows the path in use and pauses the joystick on a slow link.

Set up: follow `docs/STEPS_remote_control.md` steps 1–2 (`TS_AUTHKEY=` line in `~/robot.env`). Checks:
1. `sudo bash ~/slam-robot/scripts/setup_wifi.sh status` → hotspot priority 20 active, `hotspot-first timer: on`.
   Turn phone A's hotspot off: the robot joins the fallback; on again: back on the hotspot within ~1 min.
2. `sudo bash ~/slam-robot/scripts/setup_tailscale.sh status` → `Running`, `enabled`, `Pathik app answers: yes`.
   Reboot the Pi: still `Running` without logging in again.
3. Phone B on mobile data with Tailscale → Pathik → `slam-robot` → the top bar says `Internet (Tailscale)`.
   Drive with the joystick; then go somewhere with poor signal: above 600 ms it greys out and the robot stops.
4. Optional, public link: `setup_tailscale.sh funnel on` → phone B **without** Tailscale opens the printed
   `https://…ts.net` in Pathik → red PUBLIC tag + red card in Me; repeated wrong passwords → "Too many" message (wait 15 min or log in from the robot's own network).
   Then `funnel off` and the URL stops working.
