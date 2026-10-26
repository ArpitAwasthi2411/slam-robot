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
- RPLiDAR A1M8 → `/dev/ttyUSB0`. **ESP32-S3-DevKitC-1** (native USB) → `/dev/ttyACM0` (moving from the
  classic ESP32, which was `/dev/ttyUSB*`).
- 2 × 12 V 100 RPM encoder motors (~4970 ticks/rev), 125 mm wheels, separation 0.30 m (to calibrate),
  2 × BTS7960, MPU-6050, 3 × HC-SR04 (ECHO via BSS138 shifter), mushroom e-stop (NC → driver EN, NO → GPIO 42),
  FlySky RC, 3S Li-ion 11.1 V 7.5 Ah, XL4015 5 V buck, battery divider 100k/22k → GPIO 1.
- New (Oct 26): Camera Module 3 Wide (CSI), USB sound card + lapel mic + USB-powered speaker.
- Every wire: `hardware/CONNECTIONS_esp32s3.md`. Classic pins: `hardware/wiring_classic_esp32.md`.
- Footprint (base_link = axle): front 0.07, rear 0.37, half-width 0.21, margin 0.04 m: **not measured yet**.

## Software map

- `firmware/robot_esp32s3/` **v2.5** (S3, RGB status LED, `L,r,g,b,ms`), `firmware/robot_esp32_classic/` v2.4.
  Serial protocol 115200 text: ESP→Pi `ODM, IMU, US, BAT, BMP, SPD, INFO`; Pi→ESP `V P S E R C G ? K W X T L`
  (`lidar_robot/protocol.py`).
- ROS package `ros2_ws/src/lidar_robot` (ament_python). Nodes: `esp32_bridge` (odom/TF/imu/ultrasonic, link-drop
  counters), `navigator` (`navigator_core.py`: A* `planner.py`, `follower.py`, missions, places/rooms, schedules,
  battery/dock, explain, exploration), `dashboard` (`dashboard_server.py`, HTTP :8080, auth). Launch:
  `bringup.launch.py esp32_port:= lidar_port:= slam_mode:=mapping|localization map:=`. Cartographer 2D.
- Config: `config/robot_params.yaml`. Users: `~/maps/users.json` (`auth.py`, roles staff < operator < admin).
- App: `lidar_robot/web/app/` (served at `/app/`, bundled in the Android WebView APK `android/`); demo mode `demo.js`.
- MCP server: `scripts/mcp_server.py` (logs in with `ROBOT_USER`/`ROBOT_PASSWORD`), docs `docs/mcp.md`.
- Tools: `tools/dashboard_sim.py --demo-users` (whole app without the robot), `tools/link_doctor.sh`,
  `tools/serial_probe.py`, `tools/eval_commands.py`, `tools/eval_nav.py`, `scripts/find_ports.py`.
- Deploy: `scripts/robot.service` + `~/robot.env` (ROBOT_ARGS, GROQ_API_KEY) is what runs today.
  Docker (`Dockerfile`, `deploy/pi/docker-compose.yml`, `scripts/pathik.sh`) is built by CI but not installed on the Pi.
- CI: `robot-image.yml` (tests → image → ghcr.io/arpitawasthi2411/slam-robot), `android.yml` (APK release),
  `firmware.yml` (S3 + classic on cores 2.0.17 and 3.0.7).
- Docs: `docs/STEPS_esp32s3_lab.md` (lab bring-up), `docs/deployment.md`, `docs/navigation.md`,
  `docs/troubleshooting.md`, `docs/roadmap.md`.

## Current status (update every session)

**Last updated: 2026-10-27**

Verified on hardware (classic ESP32): LiDAR + Cartographer mapping (15.5 × 12.2 m map), ESP32 link ~50 Hz,
odometry, ultrasonics, navigator driving, app.

Done in code, NOT yet verified on the robot:
- ESP32-S3 firmware v2.5 (compiles in CI) + S3 wiring
- accounts/roles + redesigned app (latest APK **Pathik-11**, release v1.0.11)
- link-drop diagnostics (v2.4.2), battery divider, docking, schedules, rooms + LLM on the real floor
- Docker deployment, MCP server against the real robot

Open problems / TODO (priority order):
1. Lab bring-up on the S3 (`docs/STEPS_esp32s3_lab.md`), then the drive test with `link_doctor.sh --watch`
   (ESP32 used to drop/reconnect while driving: power noise suspected; capacitors planned).
2. Measure the footprint + wheel separation; tune with the app's Spin / Straight line tests.
3. Groq key + audio on the Pi; voice node (push-to-talk, Whisper/Vosk from Rashi, Piper TTS).
4. Camera on Ubuntu 22.04 (libcamera from source + camera_ros), then people detection (YOLO11n) fused with LiDAR range.
5. **Docking is still reverse-in in the code; the design is now FRONT docking** with an ArUco/AprilTag marker
   (Neelaksh's `detect()`): change the navigator and relax `US_STOP_MM` while docking.
6. LLM error recovery (thesis novelty), not implemented yet.
7. Install Docker on the Pi (make the GHCR package public first).

Deadlines: ROSCon India proposal **8 Nov 2026**; IndiaAI Fellowship **30 Nov 2026**.

## Session log (newest first, one line each)

- 2026-10-27: find_ports.py recognises the S3 (/dev/ttyACM*); added `hardware/CONNECTIONS_esp32s3.md`,
  `docs/STEPS_esp32s3_lab.md` and this CLAUDE.md. Lab kit prepared (zip, S3 .ino, Pathik-11 APK).
- 2026-10-26: camera/mic/speaker arrived; team tasks set (Rashi voice→vision, Neelaksh markers, Yogit dock); dock → front docking.
- 2026-10-24: ESP32-S3 firmware v2.5 + firmware CI + S3 wiring guide.
- 2026-10-23: accounts/roles, app redesign (light/dark), Docker + CI/CD, deployment guide.
- 2026-10-22: lab day: ports, "position lost" fix, smoother motion; robot worked end to end. v2.4.2 link diagnostics.
- 2026-10-09..13: Pathik app, Tuning Lab, APK CI, exploration, Tailscale, v2.4 features, MCP server.
