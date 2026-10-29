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

## Phase 2: the redesigned app (docs/APP_FEATURES.md)

What changed: new layout, nothing removed. Tabs **Home · Send · Map · Drive · Operate** (operator/admin) and
**Home · Send · Map · Me** (staff). STOP sits in the middle of the tab bar on every screen. Home = status card
(state, battery, link, room), current mission, "Send or call Pathik", Come back / Charge / Why stopped? / Camera,
charging. Send = command or voice with the plan preview, places (Route shows the length inline + Show on map;
operators can now **delete a place**), rooms, scheduled trips. Operate = Health, Wheel tuning, Path following,
Drive tests, Mission log, People (admin) + appearance, robot, account. The Android back button closes dialogs and
sub-pages first; the APK follows the phone's font size up to 130 %.

Install: the APK build runs when this is merged (`android.yml`, release **Pathik-N**); or open
`http://slam-robot:8080/app/` in the phone's browser (the robot serves the same app). Checks:
1. Log in as each role (staff / operator / admin): the tabs match the list above, staff has no Drive tab and no
   map tools except Measure; operator sees Operate; only admin sees Operate → People.
2. Home shows the right battery % and link delay; Home → Send or call → "go to <place>" → the plan → Send to robot;
   the mission card appears on Home with Cancel. Press STOP from Send, Map and Operate: the overlay appears.
3. Send → a place → Route: length and time appear under it; Show on map; operator: Delete place (then re-save it).
4. Map tab: tap the handle, the panel shrinks to a strip; operator: Flag / Draw / Route / Room / Locate still work;
   long-press still opens "This spot".
5. Phone back button: closes a dialog, then a sub-page, then goes to Home; one more press leaves the app.
6. Settings → Display → Font size to the largest: the app text grows and nothing important is cut off.
Without the robot: `node ros2_ws/src/lidar_robot/test/app_parity.js` (303 checks against the demo robot) or the
demo button on the connect screen.

## Phase 3: LLM error recovery (docs/recovery.md)

What changed: when a step fails (blocked way, no route, stop inside an obstacle, position lost, dock contacts),
the robot stops and decides what to do from a fixed menu: wait and retry, wider route, detour via a place, skip
the stop, do it later, ask a person in the app, go home, go to the dock. The LLM decides when `GROQ_API_KEY` is
in `~/robot.env`; otherwise (or if the LLM is slow / wrong) the rules decide. Home shows "What happened /
What I'm doing / Why" and, when asked, the answer buttons. Settings: `recovery_*` in `robot_params.yaml`.

Checks (robot running, a map with places, the app open on Home):
1. `journalctl -u robot | grep "error recovery"` → `error recovery llm (llama-3.3-70b-versatile)` with the key,
   `(rules: no GROQ_API_KEY)` without it.
2. **Person in the corridor**: send Pathik to a place; stand in front of it in a narrow spot for ~30 s. Home shows
   "Recovering", "Waiting N s, then trying again"; step aside: it carries on and the card says Recovered.
3. **Ask a person**: long-press a spot inside a table on the map → Go here. Home shows "Pathik needs your help"
   with buttons; press Go home → it drives home. (Staff accounts can answer too.)
4. **Trip with a blocked stop**: put a box in a lab door, then "go to <that lab> then HOD office". It should
   skip or postpone that lab and still reach the HOD office. Operate → Mission log shows "recovered by …".
5. Press STOP during a recovery: it ends at once; Resume continues as before.
6. Numbers for the report (laptop or Pi, with the key): `python3 tools/eval_recovery.py --rounds 3`
   → `results/recovery_eval.md` (LLM vs rules vs none).
