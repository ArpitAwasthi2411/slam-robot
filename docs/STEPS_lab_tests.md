# Lab tests: where is Pathik lagging?

One sheet, run top to bottom (about 2 hours). Each test says **what to do**, **what to write down** and
**what counts as good**. Fill the results table at the end and send it (or photos of the screens) to Claude.

Before you start: battery full (BAT in the app ≥ 12.0 V), robot on the department map in **localization**
(`cat ~/maps/map_mode.json` must not say `mapping`), one phone connected, camera view closed unless a test asks.

```bash
ssh arpit@slam-robot          # or arpit@169.254.1.2 on the Ethernet cable
cd ~/slam-robot && git log --oneline -1     # write down which code is on the robot
```

---

## A. Wiring and wheels (robot on blocks, 10 min)

Only one program can use the ESP32 link, so stop the robot first.

```bash
sudo systemctl stop robot
cd ~/slam-robot/tools
for i in 1 2 3 4 5; do python3 serial_probe.py --pwm 120 120 --secs 3; done
for i in 1 2 3 4 5; do python3 serial_probe.py --vel 200 200 --secs 3; done
```

- Gently wiggle the encoder and driver wires on **both** sides during the runs.
- **Good:** all 10 runs show both wheels moving; `--pwm` left and right within 10 %; `--vel` both 190–210 mm/s.
- **Bad:** any `0.0` (loose wire on that side; see hardware/CONNECTIONS_esp32s3.md M1–M7, E1–E8).

```bash
sudo systemctl start robot
```

## B. Driving straight (floor, 10 min)

Stick a 5 m tape line on the floor. Start with the robot's centre on the line.

1. **Joystick:** app → Drive (or Operate → Drive tests → Straight line), push straight forward at the slowest speed for 5 m. Measure how far the
   robot's centre is from the tape at the end (cm, left or right). Do it 3 times.
2. **Autonomous:** mark two places 5 m apart on the line (Map → mark place), send a trip from one to the
   other, measure the same. 3 times.

- **Good:** under 10 cm off after 5 m in both.
- Joystick bad, autonomous bad: wheels, caster or weight (test A, then check the caster swivels freely).
- Joystick good, autonomous bad: heading or CPU (tests C, D).

## C. CPU while driving (20 min)

Run the report **while the robot is driving a trip** (send a trip, then start it within 2 s).

```bash
python3 ~/slam-robot/tools/cpu_report.py
```

Do it for each row below. Write down "robot programs together" and the top 3 programs.

| # | Setting | How |
|---|---|---|
| C1 | Standing still | no trip |
| C2 | Driving, everything on | normal |
| C3 | Driving, camera view open on the phone | open the camera card |
| C4 | Driving, 3 phones connected | log in on 3 phones |
| C5 | Driving, vision off | `nano ~/robot.env` → `vision:=false`, `sudo systemctl restart robot` |
| C6 | Driving in **mapping** mode (only if you have time) | Map → New map, then back to the saved map after |

- **Good:** under 280 of 400 while driving, "throttled 0x0", temperature under 70 °C.
- Put `vision:=true` back after C5.

## D. Is the control loop on time? (10 min)

Open a second SSH window while a trip runs:

```bash
source /opt/ros/humble/setup.bash && source ~/slam-robot/ros2_ws/install/setup.bash
ros2 topic hz /odom      # wheel odometry from the ESP32
ros2 topic hz /scan      # LiDAR
ros2 topic hz /cmd_vel   # what the navigator sends to the wheels
```

Let each run 15 s (Ctrl+C), write down the average and the **max** gap.

- **Good:** /odom about 50 Hz, /scan about 7 Hz, /cmd_vel about 20 Hz, max gap under 0.2 s.
- **Bad:** /cmd_vel below 15 Hz or gaps over 0.3 s = the navigator is starved (CPU, test C), which makes the
  robot over-steer and drift.

Also run the one-shot check and keep its output:

```bash
bash ~/slam-robot/tools/robot_check.sh
```

## E. Odometry and heading (15 min)

1. **5 m push** (robot service stopped): `python3 ~/slam-robot/tools/serial_probe.py --push --metres 5`,
   push it 5.00 m along a wall by hand. Write down the left and right ticks it suggests and compare with
   `ticks_per_rev_left: 5048` / `ticks_per_rev_right: 5063` in `config/robot_params.yaml`.
2. **Spin test** (service running): app → Operate → Drive tests → Spin, 3 times. Write down the
   suggested gyro_scale each time (now 1.043).

- **Good:** ticks within 1 % of the yaml values; the 3 spin results within ±0.01 of each other and of 1.043.
- Do **not** change the values on the robot during the test; send them first.

## F. Localization (10 min)

1. Note the position in app → Operate → Health.
2. Drive a loop with the joystick (out along the corridor, into a room, back), stop exactly where you started.
3. Note the position again.

- **Good:** back within 15 cm and 5°, and Health shows no "position lost" during the loop.
- Map jumps (SLAM moving the robot in one step): `journalctl -u robot --since "10 min ago" | grep -i jumped`.
  **Good:** none, or one small one.

## G. Ten trips (40 min): the main result

Use 3 places that cover the floor (for example Home, Lab 2, Ravi Sir's cabin). Send 10 trips one after another
(mix of single and two-stop trips). Do not help the robot unless it is about to hit something.

For each trip write: from → to, arrived (yes/no), time, wall touches, how many times it stopped, what recovery
it chose (app Home card or the log), and whether it asked the app.

After the 10 trips, print the summary from the robot's own mission log:

```bash
python3 - <<'EOF'
import json, collections, os
rows = [json.loads(l) for l in open(os.path.expanduser('~/maps/missions.jsonl')) if l.strip()][-10:]
res = collections.Counter(r.get('result') for r in rows)
acts = collections.Counter(a.get('action') for r in rows for a in (r.get('recoveries') or []) if isinstance(a, dict))
print('results:', dict(res))
print('recoveries:', dict(acts))
for r in rows:
    print(r.get('stops'), r.get('result'), r.get('duration_s'), 's', 'replans', r.get('replans'),
          'blocked_s', r.get('blocked_s'), 'bumps', r.get('bumps'))
EOF
```

- **Good:** 9 or 10 of 10 arrive, no wall touch, under 2 recoveries in total.

## H. Recovery on purpose (15 min)

Stage each once during a trip and write down which action it chose and how long until it moved again:

| # | Stage this | Expected |
|---|---|---|
| H1 | A person stands in the corridor for 10 s | "May I pass?", waits, goes on |
| H2 | A chair in the corridor, room to go round | replans round it without stopping long |
| H3 | A box fully blocking a door for 60 s, then removed | waits up to 20 s, then recovery (wait / ask) |
| H4 | Lift the robot 20 cm sideways while it drives (position lost) | stops, waits, asks if lost for long |

If almost every case ends in **ask the app**, note the failure code shown (BLOCKED, NO_PATH, POSE_LOST): for a
one-stop trip only "wait" and "ask" are possible after the first try, so frequent asking after a single wait is
expected; asking on the **first** failure points to POSE_LOST, which comes from CPU starvation (test D).

## I. Commands and voice (10 min)

Type or speak these in the app (Send), write down the robot's answer and the time it took:

1. `go to lab 2`  2. `lab 2 jao phir HOD office`  3. `go to library` (not on the map: must refuse)
4. `wait 30 seconds at lab 1 then come back`  5. `stop`

- **Good:** 1, 2, 4 correct; 3 says the place is unknown; 5 stops at once. LLM answers under 2 s.

## J. Battery (during the whole session)

Write the battery voltage at the start and every 30 minutes. **Good:** still above 11.1 V after 2 hours.

---

## Results (fill in)

| Test | Result | Good? |
|---|---|---|
| Code on the robot (commit) | | |
| A. 10 wheel runs: any 0.0? | | |
| B. Joystick drift after 5 m (cm, side) ×3 | | |
| B. Autonomous drift after 5 m (cm, side) ×3 | | |
| C1–C5 total CPU (of 400) + top 3 | | |
| D. /odom, /scan, /cmd_vel Hz and max gap | | |
| E. Push ticks L / R | | |
| E. Spin gyro_scale ×3 | | |
| F. Loop return error (cm, °) | | |
| G. Trips arrived / 10, touches, recoveries, asks | | |
| H1–H4 action chosen + time | | |
| I. Commands correct / 5, LLM time | | |
| J. Battery start → 2 h | | |

**Where the problem is, from the table:**

- A bad → wiring. B joystick bad, A good → caster, wheel or weight.
- C over 280 and D /cmd_vel slow → CPU: localization mode, vision off, then the code fixes (bridge and app
  server rates).
- E off → recalibrate (ticks, gyro_scale), then repeat B and F.
- F bad with A–E good → Cartographer localization settings.
- G/H many asks with POSE_LOST → CPU (C, D); with BLOCKED/NO_PATH on one-stop trips → expected, or change
  the recovery ladder.
