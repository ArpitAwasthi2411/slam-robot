# Lab day: put everything since 2 Nov on the robot and test it

Since the last lab (2 Nov) these are new and **none has run on the robot yet**: firmware v2.8 (running tick
totals), the odometry guard, the LLM fix + conversation agent, map mode in the app, Nav2 as a second planner,
the head-mode kiosk (PIN last, Go / Ask / Deliver, new face) and the person-or-object blocker.

Carry: laptop with Arduino IDE, ethernet cable, `slam-robot.zip` (latest code), the **Pathik APK** (latest
release on GitHub), charged battery, RC transmitter, tape measure, masking tape, a chair, and this page.
Robot **on blocks** (wheels in the air) until step 4.

Write every result in the table at the end. That table is what we fix the code from.

---

## 1. Flash firmware v2.8 (15 min, laptop)

v2.8 adds running tick and yaw totals to every `ODM` line, so a lost serial line no longer loses distance.
The Pi code from step 2 expects it.

1. Battery OFF. **Unplug the ESP32's 5V wire (P10)**, then plug the laptop USB into the ESP32's "USB" socket.
   Never have both connected at once.
2. Arduino IDE → open `firmware/robot_esp32s3/robot_esp32s3.ino`. Board **ESP32S3 Dev Module**,
   USB CDC On Boot **Enabled** (same settings as `STEPS_esp32s3_lab.md` step 1) → **Upload**.
   Fails? Hold BOOT, tap RST, release BOOT, upload again.
3. Serial Monitor 115200 → expect `INFO,READY,... v2.8`. Type `I` + Enter (I2C scan):
   - `0x68` found → the IMU is wired right.
   - nothing found → check SDA = GPIO 11, SCL = GPIO 12, 3V3 and GND at the MPU-6050. Photo the wiring for me.
   Either way carry on: odometry uses the wheels, not the gyro.
4. Close the Serial Monitor, unplug the laptop USB, **plug the 5V wire back in.**

## 2. New code on the Pi (15 min)

Laptop (unzip `slam-robot.zip` in Downloads first, so you have `~/Downloads/slam-robot/`):
```bash
ip addr | grep 169.254 || bash ~/Downloads/slam-robot/scripts/setup_laptop_link.sh
scp ~/Downloads/slam-robot.zip arpit@169.254.1.2:~
ssh arpit@169.254.1.2
```
Pi (the first line keeps your calibration safe):
```bash
cp ~/ros2_ws/src/lidar_robot/config/robot_params.yaml ~/robot_params.backup.yaml
sudo systemctl stop robot
rm -rf ~/slam-robot && cd ~ && unzip -q ~/slam-robot.zip      # makes ~/slam-robot
rm -rf ~/ros2_ws/src/lidar_robot ~/ros2_ws/build/lidar_robot ~/ros2_ws/install/lidar_robot
cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
diff ~/robot_params.backup.yaml ~/ros2_ws/src/lidar_robot/config/robot_params.yaml
```
`diff` should now only show new settings. The repo already has your measured ticks (5048 / 5063) and
`wheel_separation: 0.294`. If it shows a value **you** changed on the Pi after 2 Nov, put yours back in the new file.
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lidar_robot
sudo cp ~/slam-robot/scripts/robot.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl start robot
bash ~/slam-robot/tools/robot_check.sh
```
**Check:** `robot_check.sh` shows LiDAR OK, ESP32 link OK, `throttled=0x0`. Not 0x0 = under-voltage: stop and
fix the Pi's 5 V before anything else (docs/compute.md).

The Groq key stays in `~/robot.env` as before (`GROQ_API_KEY=...`). Don't paste that file anywhere.

## 3. App (5 min)

Install the new APK on the phone (uninstall the old one first if Android refuses). Log in, then:
- **Home** shows Idle and battery volts. **Map** shows the LiDAR scan.
- **Operate → AI commands → Test AI** → green. Red? Note the error text it shows (it says what's wrong).
- **Health** row: Pi temperature, CPU, no throttle warning.

## 4. Drive and the link (15 min, robot on the floor)

Pi, second ssh window:
```bash
bash ~/slam-robot/tools/link_doctor.sh --watch
```
Drive with the app joystick for 5 minutes: straight runs, turns, sudden stops.
**Pass:** 0 link drops in `~/link_log.txt`. Drops? Note when they happen (start, braking, turning).

## 5. Odometry (20 min)

1. **5 m push** (robot service stopped, wheels on the floor):
   ```bash
   sudo systemctl stop robot
   python3 ~/slam-robot/tools/serial_probe.py --push --metres 5
   sudo systemctl start robot
   ```
   Roll it along a taped 5 m line. **Pass:** both wheels within 1 % (4.95-5.05 m). Otherwise write down the
   ticks it suggests.
2. **Spin test** (app → Operate → Tuning Lab → Spin, 0.4 rad/s, at least 3 turns, twice). **Pass:** the suggested
   `wheel_separation` is within 0.29-0.30.
3. **Guard counters:** `curl -s localhost:8080/api/state | python3 -m json.tool | grep -A6 odom_guard`
   after driving: `dropped` and `recovered` should be small numbers, not hundreds.

## 6. Footprint (10 min, tape measure)

From the **wheel axle centre line**, measure to the very front of the body, to the very back, and from the
centre line to the widest side point (bumpers and screws included). Robot on a flat floor, photo the tape.
These four numbers go in `robot_params.yaml` (`footprint_front/rear/half_width`). Today it uses guesses
(0.07 / 0.37 / 0.21).

## 7. Map the lab and test trips (40 min)

1. App → **Map** → map card → **New map**. Drive slowly round the whole room once, then **Freeze map**
   (the robot restarts into localization on it).
2. Save 2 places at opposite ends of the room (drive there, Map → long-press the robot's spot → Save as a place),
   e.g. `Door` and `Window`.
3. **10 trips** Door ↔ Window from the app. For each one: reached / stopped / gave up, and any bump.
   This is the start of the 100-trip trial.
4. During the trips watch `/api/state` → `map_jumps`. A jump means the map re-matched. Note how many.

## 8. Person or object in the way (15 min, camera connected)

With a trip running:
- **Stand in its path.** Pass: it stops, says "Excuse me, may I pass?", and goes on with "Thank you" after you
  step aside (it waits up to 8 s).
- **Put a chair in the path** (no person near). Pass: it plans round the chair without speaking.
No camera today: the chair test still works (LiDAR and ultrasonics); skip the person test.

## 9. Language and the kiosk (15 min)

1. Send tab, type: `go to Window` → drives. `what can you do?` → answers, does **not** drive.
   `take this to Door, wait 1 minute, then come back` → a 3-step mission appears.
2. Mount the phone, open **Head mode**. Tap the face → "Hi, I'm Pathik!...". Tap **Go** → choose Door →
   the PIN is asked **last** → robot leaves. **Ask** → type a question → answer shown and spoken, no PIN.
   **STOP** always works without the PIN.

## 10. Nav2 (only if time is left, needs internet on the Pi)

```bash
bash ~/slam-robot/scripts/setup_nav2.sh
```
Then, **on blocks first**: set `ROBOT_ARGS="... nav_mode:=nav2"` in `~/robot.env`, `sudo systemctl restart robot`,
send a trip and check the wheels turn the right way. Then 3 trips on the floor. Put `nav_mode` back afterwards.

---

## Results (fill in, send me a photo of this)

| Step | Result | Notes |
|------|--------|-------|
| 1 firmware | v2.8 shown? I2C scan found: | |
| 2 robot_check | LiDAR / ESP32 / throttled | |
| 3 Test AI | green / error text | |
| 4 link drops in 5 min | | when |
| 5 push 5 m | left ___ m, right ___ m | suggested ticks |
| 5 spin | wheel_separation ___ | |
| 5 guard | dropped ___ recovered ___ | |
| 6 footprint | front ___ rear ___ half-width ___ | |
| 7 trips | ___ / 10 reached, bumps ___ | map_jumps ___ |
| 8 person / chair | pass / fail | |
| 9 language / kiosk | pass / fail | |
| 10 Nav2 | __ / 3 | |

Bring back also: `~/link_log.txt` and `journalctl -u robot --since today > ~/robot_today.log` (copy both to the laptop).
