# Pathik, the mobile app

One app for everyday use and for research:

| Tab | What it does |
|---|---|
| **Drive** | Live map with LiDAR, route and places. Joystick, top-speed limit, Save map, Position OK |
| **Go** | Say where to go ("take this to lab 3 then come back"), places list with Route and Go, mission card with Cancel/Resume, Save place here, Set home. Long-press the map to send the robot anywhere |
| **Lab** | **Wheels**: the ESP32 speed loop (Kp, Ki, start-up PWM, top speed, acceleration ramp) with live wheel-speed chart and a step test that grades the tuning. **Path following**: look-ahead, steering gain, accelerations, turn-on-the-spot thresholds. **Drive tests**: straight-line drift test and spin test that computes the correct `wheel_separation` |
| **Status** | Link health, firmware, LiDAR rate, position, sensors, navigator, activity log |

The big **STOP** button is on every screen. Releasing it needs a 1-second hold.

The same app runs in three places, from the same files (`ros2_ws/src/lidar_robot/lidar_robot/web/app/`):
1. **APK** on Android phones (built by GitHub Actions, below).
2. **Any browser**: open `http://<robot>:8080`. On Android Chrome: menu → *Add to Home screen* gives an app icon too.
3. **Simulator** without the robot: `python3 tools/dashboard_sim.py`, then `http://localhost:8080`. The simulator includes a model of the motors and speed loop, so the Tuning Lab behaves realistically there.

The old dashboard is still at `http://<robot>:8080/classic`.

---

## 1. Update the robot (once)

1. **Firmware v2.3** (Arduino IDE): flash `firmware/robot_esp32_classic/robot_esp32_classic.ino`.
   Your calibrated directions and ticks are already in it. New: live tuning, a smooth
   acceleration ramp (the main fix for jerky starts), and wheel telemetry.
2. **Pi**: copy the new code over and rebuild (same as always):
   ```bash
   rm -rf ~/ros2_ws/src/lidar_robot && cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
   cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot && source install/setup.bash
   robot_up
   ```
3. Check from the laptop: `http://169.254.1.2:8080` shows Pathik.

## 2. Network: phone and robot on the same Wi-Fi

**Option A: the robot makes its own Wi-Fi** (most reliable, works anywhere):
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh ap robot1234
```
Phone: join **SLAM-Robot** (password `robot1234`) → Pathik → Find robot. The address is always `10.42.0.1`.
Needs NetworkManager on the Pi (the script tells you how to install it). The Pi has no internet
in this mode, so typed commands use the offline parser.

**Option B: the robot joins your phone's hotspot** (keeps internet → LLM commands work):
```bash
sudo bash ~/slam-robot/scripts/setup_wifi.sh hotspot "Arpit's phone" "hotspot-password"
```
Phone: hotspot on → Pathik → Find robot. The APK asks Android for the hotspot's network and scans it
(a few seconds).

Either way, the ethernet cable to the laptop keeps working at the same time.

## 3. Build the APK (GitHub Actions)

The build runs on GitHub's servers. Your laptop needs nothing installed.

1. Push this repo to GitHub (any name, public or private).
2. **Signing key (once, recommended):** repo → Settings → Secrets and variables → Actions → *New repository secret*, three times:
   - `PATHIK_KEYSTORE_B64`: the whole content of `keystore.b64`
   - `PATHIK_KEYSTORE_PASSWORD`: from `secrets.txt`
   - `PATHIK_KEY_ALIAS`: `pathik`

   Keep `pathik-release.jks` and the password somewhere safe (not in the repo). Every future
   update must be signed with the same key, or phones refuse to install it over the old version.
   Without these secrets the build still works but uses a throwaway key.
3. Repo → **Actions** → *Pathik APK* → **Run workflow** (it also runs by itself on every push
   that changes the app).
4. When it's green (about 4 minutes): open the run → *Artifacts* → **Pathik-apk** → download →
   unzip → `Pathik-<n>.apk`.
5. Phone: open the APK → allow "Install unknown apps" for your file manager/browser → Install.

For a public download link: create a tag `v1.0.0` (`git tag v1.0.0 && git push --tags`). The workflow
attaches the APK to a GitHub Release.

## 4. Tuning Lab: how to remove the jerks, in order

Do it with the robot **on the floor** in open space, battery charged.

**Step 1: Wheels (speed loop).** Lab → Wheels → *Run step test* at 0.20 m/s.
- The chart shows target (dashed) and each wheel's measured speed. The cards grade the result,
  and the bottom line tells you what to change.
- Typical order: set **Acceleration ramp** first (400-800 mm/s² feels smooth; 0 is the old jerky
  behaviour), then **Kp** until the wheels reach speed quickly without overshoot > 10 %, then **Ki**
  until the steady error is under ~6 mm/s.
- Change one value, run the test again, compare. When happy: **Save to robot** (kept in the ESP32's
  flash, even after power-off).

**Step 2: Spin test (wheel separation).** Lab → Drive tests → Spin → Run.
It compares how far the LiDAR map says the robot turned with what the wheels say, and prints the
correct `wheel_separation`. Put it in `config/robot_params.yaml`, then restart with `robot_up`.

**Step 3: Straight test.** Lab → Drive tests → Straight line → Run.
Under 3 cm of drift per metre is good. If it curves, the two wheels' ticks per revolution differ.
Repeat the 1 m push test and set left and right separately.

**Step 4: Path following.** Lab → Path following, then send goals with long-press:
- weaves left-right → lower **Steering gain** or raise **Look-ahead**
- cuts corners → lower **Look-ahead**
- stops and turns too often → raise **Turn on the spot above**
- *Save* keeps the values in `~/maps/tuning.json` (they override the YAML from then on).

## 5. Under the hood (for the report)

```
Phone (Pathik) ── HTTP/JSON ──► dashboard node :8080 ──► ROS 2 topics ──► navigator / esp32_bridge ──► ESP32
                 /api/state  /api/map  /api/cmd  /api/nav  /api/esp  /api/telemetry  /api/test  /api/ping
```
- **Firmware v2.3** protocol additions: `K,kp,ki,pwm_min,max_mms,accel` (live gains), `W` (save to flash),
  `X` (factory values), `T,1/T,0` (50 Hz `SPD,tgtL,measL,pwmL,tgtR,measR,pwmR` telemetry),
  `INFO,TUNE,...` reply. The ramp limits each wheel's target acceleration and keeps the
  left/right ratio, so curves stay curves while speeding up.
- **Bridge**: `/robot/esp_cmd` (JSON in), `/robot/wheel_telemetry` (JSON batches out), `tune`/`fw` in `/robot/status`.
- **Navigator**: requests `get_tune` and `tune` (with `save`), persisted to `~/maps/tuning.json`.
- **Dashboard**: CORS for the APK (file:// origin), the app served at `/`, test runner for scripted motions.
- Tests: `ros2_ws/src/lidar_robot/test/test_tuning_lab.py`.
