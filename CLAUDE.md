# CLAUDE.md: Pathik robot (read this first, update it last)

This file is the memory of the project for Claude (and any other agent). **Read it at the start of every
session. At the end of every session that changes something, update "Current status" and add a line to
"Session log".** Keep it true: if code and this file disagree, fix the file.

The repo is **public**: no API keys, passwords, keystores or personal data in here (or anywhere in Git).

---

## Working rules

- Owner: **Arpit Awasthi** (project lead). Team: Rashi (voice + vision), Neelaksh (markers/dock pose),
  Yogit (dock hardware). Answers should be direct and practical, with exact commands for the robot.
- **Commits:** author `Arpit Awasthi <awasthiarpit24@gmail.com>`. **No `Co-Authored-By: Claude` or
  `Claude-Session` lines.** Push to `main` of `ArpitAwasthi2411/slam-robot`.
- Run the tests before pushing: `cd ros2_ws/src/lidar_robot && python3 -m pytest -q` (CI uses pytest 6.2.5:
  never name a test helper `setup(...)`).
- Firmware can't be compiled in the Claude sandbox (toolchain downloads blocked): push and read the
  `Firmware` workflow result (`gh api repos/ArpitAwasthi2411/slam-robot/actions/runs`).
- Pin numbers live in the `.ino` **and** `hardware/CONNECTIONS_esp32s3.md` / `hardware/wiring_esp32s3.md`:
  change them together.
- Teammate tasks: say what to deliver, how it's measured and the interface (plain Python, metres/degrees,
  keys from env vars). Exploration-style, not ready-made code. Doable on a laptop when there's no lab.

## What it is

Indoor campus service robot (major project: LLM-augmented autonomous indoor robot with semantic navigation).
LiDAR SLAM map, named places + rooms with contents, English/Hinglish commands (Groq LLM + offline rules),
priority mission queue, schedules, "why did you stop?", charging dock, phone app with logins/roles,
MCP server so AI assistants can drive it, Docker + CI/CD deployment.

## Hardware (current build)

- Raspberry Pi 4 (Ubuntu 22.04, ROS 2 Humble, user `arpit`, ethernet `169.254.1.2`, Tailscale `slam-robot`).
- RPLiDAR A1M8 → `/dev/ttyUSB0`. **ESP32-S3-DevKitC-1** on the Pi's **GPIO UART** (since 2026-10-28):
  ESP GPIO 47 (TX) → Pi pin 10 (RXD), ESP GPIO 8 (RX) ← Pi pin 8 (TXD), GND → pin 6; ESP 5V pin from the buck.
  Pi side `/dev/ttyAMA0` = `/dev/esp32` after `scripts/setup_pi_uart.sh` (disable-bt, console off). ESP USB socket
  = laptop flashing only (unplug the 5V wire first). Classic ESP32 was `/dev/ttyUSB*`.
- 2 × 12 V 100 RPM encoder motors (~4970 ticks/rev), 125 mm wheels, separation 0.30 m (to calibrate),
  2 × BTS7960, MPU-6050, 3 × HC-SR04 (ECHO via BSS138 shifter), mushroom e-stop (NC → driver EN, NO → GPIO 42),
  FlySky RC, 3S Li-ion 11.1 V 7.5 Ah, XL4015 5 V buck, battery divider 100k/22k → GPIO 1.
- New (Oct 26): Camera Module 3 Wide (CSI), USB sound card + lapel mic + USB-powered speaker.
- Every wire: `hardware/CONNECTIONS_esp32s3.md`. Classic pins: `hardware/wiring_classic_esp32.md`.
- Footprint (base_link = axle): front 0.07, rear 0.37, half-width 0.21, margin 0.04 m: **not measured yet**.

## Software map

- `firmware/robot_esp32s3/` **v2.7** (S3, RGB status LED `L,r,g,b,ms`, Pi link on UART1 pins 47/8 + echo on USB,
  only well-formed command lines accepted, docking mode `D,1`, I2C scan `I`), `firmware/robot_esp32_classic/` v2.5.
  Serial protocol 115200 text: ESP→Pi `ODM, IMU, US, BAT, BMP, SPD, INFO`; Pi→ESP `V P S E R C G ? K W X T L D I`
  (`lidar_robot/protocol.py`).
- ROS package `ros2_ws/src/lidar_robot` (ament_python). Nodes: `esp32_bridge` (odom/TF/imu/ultrasonic, link-drop
  counters), `navigator` (`navigator_core.py`: A* `planner.py`, `follower.py`, missions, places/rooms, schedules,
  battery/dock, explain, exploration, error recovery `recovery.py`), `dashboard` (`dashboard_server.py`, HTTP :8080, auth). Launch:
  `bringup.launch.py esp32_port:= lidar_port:= slam_mode:=mapping|localization map:=`. Cartographer 2D.
- Config: `config/robot_params.yaml`. Users: `~/maps/users.json` (`auth.py`, roles staff < operator < admin).
- App: `lidar_robot/web/app/` (served at `/app/`, bundled in the Android WebView APK `android/`); demo mode `demo.js`.
  v3 layout (Phase 2): Home · Send · Map · Drive · Operate (staff: Home · Send · Map · Me), STOP in the tab bar.
  Feature inventory + parity checklist `docs/APP_FEATURES.md`; browser check `test/app_parity.js` (run by `test_app_parity.py`).
- MCP server: `scripts/mcp_server.py` (logs in with `ROBOT_USER`/`ROBOT_PASSWORD`), docs `docs/mcp.md`.
- Tools: `tools/dashboard_sim.py --demo-users` (whole app without the robot), `tools/link_doctor.sh`,
  `tools/robot_check.sh` (one-shot health check), `tools/serial_probe.py`, `tools/eval_commands.py`, `tools/eval_nav.py`, `scripts/find_ports.py`.
- Deploy: `scripts/robot.service` + `~/robot.env` (ROBOT_ARGS, GROQ_API_KEY) is what runs today.
  Docker (`Dockerfile`, `deploy/pi/docker-compose.yml`, `scripts/pathik.sh`) is built by CI but not installed on the Pi.
- CI: `robot-image.yml` (tests → image → ghcr.io/arpitawasthi2411/slam-robot), `android.yml` (APK release),
  `firmware.yml` (S3 + classic on cores 2.0.17 and 3.0.7).
- Vision on the robot: `vision_node.py` (Camera Module 3 via libcamera/GStreamer subprocess, YOLO11n NCNN ~4 Hz,
  LiDAR distance per person) -> `/vision/decision` + `/vision/image/compressed`; rules + `VisionGate` in
  `vision_rules.py`; navigator slows/stops (`use_vision`), app camera card (`/api/camera.jpg`). Setup:
  `scripts/setup_pi_camera.sh` (+ `setup_imx708_driver.sh`: Ubuntu kernels lack imx708), `scripts/setup_vision.sh`,
  `docs/STEPS_camera.md`.
- Vision on the laptop: `vision/robot_decision.py` (webcam, rules + one Groq vision call per capture), Rashi's `people.py`.
- Docs: `docs/STEPS_esp32s3_lab.md` (lab bring-up), `docs/deployment.md`, `docs/navigation.md`,
  `docs/troubleshooting.md`, `docs/roadmap.md`.

## Current status (update every session)

Full task list with owners and ready-to-paste prompts: **`docs/WORK_LEFT.md`**.

**Last updated: 2026-10-29**

Verified on hardware (classic ESP32): LiDAR + Cartographer mapping (15.5 × 12.2 m map), ESP32 link ~50 Hz,
odometry, ultrasonics, navigator driving, app.

Done in code, NOT yet verified on the robot:
- front docking on the ArUco marker (docking.py, marker_pose.py, vision /vision/dock_marker, firmware D docking mode)
- LLM error recovery (fixed menu, validation, rules fallback, app card, eval: rules 6/6 vs none 1/6 in sim)
- redesigned app (v3 layout, every feature kept: docs/APP_FEATURES.md); next APK build after the merge
- remote control from another network: Tailscale auth key/boot, optional Funnel with login limits, app path tag +
  slow-link joystick pause, hotspot-first Wi-Fi (`docs/STEPS_remote_control.md`)
- ESP32-S3 firmware v2.6 + UART link to the Pi (pins 47/8 ↔ Pi 10/8): ODM/US verified on the Pi; IMU not detected yet
  (v2.7 adds the `I` I2C scan + docking mode: flash it next)
- Camera Module 3 + YOLO people avoidance + app camera card (APK Pathik-12)
- accounts/roles + redesigned app (latest APK **Pathik-12**, release v1.0.12)
- link-drop diagnostics (v2.4.2), battery divider, docking, schedules, rooms + LLM on the real floor
- Docker deployment, MCP server against the real robot

Open problems / TODO (priority order):
1. Lab bring-up on the S3 (`docs/STEPS_esp32s3_lab.md`), then the drive test with `link_doctor.sh --watch`
   (ESP32 used to drop/reconnect while driving: power noise suspected; capacitors planned).
2. Measure the footprint + wheel separation; tune with the app's Spin / Straight line tests.
3. Groq key + audio on the Pi; voice node (push-to-talk, Whisper/Vosk from Rashi, Piper TTS).
4. Camera + people avoidance done in code (`docs/STEPS_camera.md`): run the two setup scripts on the Pi, tune stop_m/slow_m.
5. Front docking with the ArUco marker done in code (`docs/docking.md`, firmware v2.7 `D` mode): measure
   `dock_contact_dist_m` + the camera mount, calibrate the camera, try on Yogit's dock.
6. LLM error recovery: done in code (`docs/recovery.md`); needs the Groq-key eval run + a robot test.
7. Install Docker on the Pi (make the GHCR package public first).

Deadlines: ROSCon India proposal **8 Nov 2026**; IndiaAI Fellowship **30 Nov 2026**.

## Session log (newest first, one line each)

- 2026-10-29: Phase 4 front docking: docking.py (align/creep<=5 cm/s/contact/backoff+retry, map-fused heading),
  marker_pose.py detect() (ArUco, 4.5 + 4.7+ APIs, best-reprojection PnP), vision node marker topic, firmware S3 v2.7 /
  classic v2.5 (D docking mode, I2C scan, IMU retries), bridge /dock_mode, sim + demo front dock, marker_webcam.py.
- 2026-10-29: Phase 3 LLM error recovery: recovery.py (8-action menu, JSON validation + clamping, rules ladder, Groq
  in a thread), RECOVERING state, recovery_answer (staff), mission-log recoveries, Home card + demo, MCP answer_robot,
  tools/eval_recovery.py + sim blocks/pose drops (results/recovery_eval.md: rules 6/6, none 1/6). docs/recovery.md.
- 2026-10-29: Phase 2 app redesign: docs/APP_FEATURES.md inventory (104 rows, all mapped), new tabs + Home status
  card + Send flow + Operate hub, STOP in the tab bar, recovery/docking/voice cards ready, delete place, Android back +
  font size; test_app_parity.py (static + 303 browser checks on the demo).
- 2026-10-29: Phase 1 remote control: setup_tailscale.sh (TS_AUTHKEY from ~/robot.env, boot, status, funnel on/off
  + ~/maps/.public_url flag), auth per-address/public limits, dashboard `net` info (lan/tailscale/public), app PUBLIC
  tag + Me warning + joystick pause >600 ms, setup_wifi.sh hotspot-first timer + fallback. docs/STEPS_remote_control.md.
- 2026-10-29: Phase 0 repo review (docs/REVIEW_2026-10.md): NaN drive commands blocked at HTTP, dashboard and
  bridge; "stop" ends exploration; auto-home/explain crash fixes; setup_ports.sh UART mode; serial_probe 4970
  ticks; S3/classic pin tables fixed in hardware/README + sensors + classic wiring; README/roadmap/deploy/
  calibration/navigation docs updated; MCP end-to-end test. 126 tests. docs/STEPS_next_run.md started.

- 2026-10-28 (night): no Ubuntu 22.04 raspi kernel has imx708 (checked 1108/1110 debs): setup_imx708_driver.sh
  builds imx708 + dw9807-vcm (rpi-5.15.y) with DKMS + imx708.dtbo (compile-tested vs 1108 headers);
  setup_pi_camera.sh checks internet, calls it, libcamera v0.1.0+rpt20231122. Pi had no DNS at the time.

- 2026-10-28 (evening): camera people avoidance: vision node (libcamera, YOLO11n NCNN, LiDAR distance), navigator
  slows/stops, app camera card + map banner, setup scripts, docs/STEPS_camera.md; laptop robot_decision.py.

- 2026-10-28 (later): "position lost" pauses + hot Pi: Cartographer used all 4 cores and added a node every 0.5 s
  while standing still -> 2 threads, max_time 5 s, grid every 2 s; pose_timeout 1.5; slower turns (nav 0.30 rad/s,
  joystick 0.40); `tools/robot_check.sh` one-shot health check. Old LiDAR dropouts = USB/5 V dips.
- 2026-10-28: ESP32 ↔ Pi moved from USB to the GPIO UART (pins 6/8/10, ESP powered from the buck): firmware v2.6,
  `scripts/setup_pi_uart.sh` + `udev/98-esp32-uart.rules`, find_ports/ports/serial_probe/link_doctor know ttyAMA0, docs.

- 2026-10-27: find_ports.py recognises the S3 (/dev/ttyACM*); added `hardware/CONNECTIONS_esp32s3.md`,
  `docs/STEPS_esp32s3_lab.md` and this CLAUDE.md. Lab kit prepared (zip, S3 .ino, Pathik-11 APK).
- 2026-10-26: camera/mic/speaker arrived; team tasks set (Rashi voice→vision, Neelaksh markers, Yogit dock); dock → front docking.
- 2026-10-24: ESP32-S3 firmware v2.5 + firmware CI + S3 wiring guide.
- 2026-10-23: accounts/roles, app redesign (light/dark), Docker + CI/CD, deployment guide.
- 2026-10-22: lab day: ports, "position lost" fix, smoother motion; robot worked end to end. v2.4.2 link diagnostics.
- 2026-10-09..13: Pathik app, Tuning Lab, APK CI, exploration, Tailscale, v2.4 features, MCP server.
