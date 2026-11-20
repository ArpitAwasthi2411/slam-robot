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
  Push-to-talk button: Pi pin 11 (GPIO17) ↔ pin 9 (GND).
- Every wire: `hardware/CONNECTIONS_esp32s3.md`. Classic pins: `hardware/wiring_classic_esp32.md`.
- Footprint (base_link = axle): front 0.07, rear 0.37, half-width 0.21, margin 0.04 m: **not measured yet**.

## Software map

- `firmware/robot_esp32s3/` **v2.9** (GPIO 1 dock contact sense `DCK`; ODM lines carry running tick/yaw totals; S3, RGB status LED `L,r,g,b,ms`, Pi link on UART1 pins 47/8 + echo on USB,
  only well-formed command lines accepted, docking mode `D,1`, I2C scan `I`), `firmware/robot_esp32_classic/` v2.5.
  Serial protocol 115200 text: ESP→Pi `ODM, IMU, US, BAT, BMP, SPD, INFO`; Pi→ESP `V P S E R C G ? K W X T L D I`
  (`lidar_robot/protocol.py`).
- ROS package `ros2_ws/src/lidar_robot` (ament_python). Nodes: `esp32_bridge` (odom/TF/imu/ultrasonic, link-drop
  counters), `navigator` (`navigator_core.py`: A* `planner.py`, `follower.py`, missions, places/rooms, schedules,
  battery/dock, explain, exploration, error recovery `recovery.py`), `dashboard` (`dashboard_server.py`, HTTP :8080, auth). Launch:
  `bringup.launch.py esp32_port:= lidar_port:= slam_mode:=mapping|localization map:= vision:= voice:= urdf:=`.
  Cartographer 2D. Fixed frames from `urdf/pathik.urdf.xacro` (robot_state_publisher; `urdf_model.py` renders it
  without xacro too), checked against the params by `test_urdf.py`.
- Config: `config/robot_params.yaml`. Users: `~/maps/users.json` (`auth.py`, roles staff < operator < admin).
- App: `lidar_robot/web/app/` (served at `/app/`, bundled in the Android WebView APK `android/`); demo mode `demo.js`.
  v3 layout (Phase 2): Home · Send · Map · Drive · Operate (staff: Home · Send · Map · Me), STOP in the tab bar.
  Feature inventory + parity checklist `docs/APP_FEATURES.md`; browser check `test/app_parity.js` (run by `test_app_parity.py`).
- MCP server: `scripts/mcp_server.py` (logs in with `ROBOT_USER`/`ROBOT_PASSWORD`), docs `docs/mcp.md`.
- Tools: `tools/dashboard_sim.py --demo-users` (whole app without the robot), `tools/link_doctor.sh`, `tools/cpu_report.py` (CPU per program, temp, throttling),
  `tools/robot_check.sh` (one-shot health check), `tools/serial_probe.py`, `tools/eval_commands.py`, `tools/eval_nav.py`, `scripts/find_ports.py`.
- Deploy: `scripts/robot.service` + `~/robot.env` (ROBOT_ARGS, GROQ_API_KEY) is what runs today.
  Docker (`Dockerfile`, `deploy/pi/docker-compose.yml`, `scripts/pathik.sh`) is built by CI but not installed on the Pi.
- CI: `robot-image.yml` (tests → image → ghcr.io/arpitawasthi2411/slam-robot), `android.yml` (APK release),
  `firmware.yml` (S3 + classic on cores 2.0.17 and 3.0.7).
- Laptop mapping: `slam_host:=laptop` on the Pi + `scripts/laptop_slam.py` on a laptop on the Ethernet cable runs
  Cartographer (`cartographer_laptop.lua`), records, copies saved maps to the Pi (`laptop_slam.py`, `docs/laptop_slam.md`).
- Voice on the robot: `voice_node.py` (push-to-talk → arecord → `voice.py` Groq Whisper / Vosk → navigator command →
  reply via `/speech/say`), `speech.py` Speaker (Piper / espeak), `scripts/setup_voice.sh`, `docs/STEPS_voice.md`.
- Vision on the robot: `vision_node.py` (Camera Module 3 via libcamera/GStreamer subprocess, 5 frames/s to Python, YOLO11n NCNN det_rate 2/s on core 3,
  LiDAR distance per person) -> `/vision/decision` + `/vision/image/compressed`; rules + `VisionGate` in
  `vision_rules.py`; navigator slows/stops (`use_vision`), app camera card (`/api/camera.jpg`). Setup:
  `scripts/setup_pi_camera.sh` (+ `setup_imx708_driver.sh`: Ubuntu kernels lack imx708), `scripts/setup_vision.sh`,
  `docs/STEPS_camera.md`.
- Vision on the laptop: `vision/robot_decision.py` (webcam, rules + one Groq vision call per capture), Rashi's `people.py`.
- Docs: `docs/STEPS_esp32s3_lab.md` (lab bring-up), `docs/deployment.md`, `docs/navigation.md`,
  `docs/troubleshooting.md`, `docs/roadmap.md`.

## Current status (update every session)

Next lab day: **`docs/STEPS_lab_19nov.md`** (laptop mapping of the department, encoder check, CPU report); INA226/dock in `docs/STEPS_lab_18nov.md`; before that `docs/STEPS_lab_16nov.md` (everything since 2 Nov, with a results table). Full task list with owners and ready-to-paste prompts: **`docs/WORK_LEFT.md`**. Keys/accounts and where they go
in `~/robot.env`: **`docs/KEYS.md`**. Robot test steps for phases 0-6: **`docs/STEPS_next_run.md`**.

**Last updated: 2026-11-17**

Verified on hardware (classic ESP32): LiDAR + Cartographer mapping (15.5 × 12.2 m map), ESP32 link ~50 Hz,
odometry, ultrasonics, navigator driving, app.

Done in code, NOT yet verified on the robot:
- URDF model (urdf/pathik.urdf.xacro) + robot_state_publisher instead of the static TFs (same frames; urdf:=false)
- voice: push-to-talk (GPIO17 pin 11 / app), Groq Whisper + Vosk, Piper TTS, announcements (voice_node.py)
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

Lab 2026-10-30 findings (S3 board): encoder **GND was not connected** -> 0 ticks, so the speed loop ran open
(robot 2x too fast, spins 3-4x) and the map shifted; ground found, not yet re-tested. ESP32 flash holds wrong
wheel tuning (max_mms 225, ki 4.15): reset with esp_cmd `{"op": "defaults"}`. `imu=FAIL(who=0x00)`: check I2C
wiring (SDA 11, SCL 12). Pi showed under-voltage (0x50005, 4.7 V at the pins) and overload: see docs/compute.md.

Open problems / TODO (priority order):
1. Lab bring-up on the S3 (`docs/STEPS_esp32s3_lab.md`), then the drive test with `link_doctor.sh --watch`
   (ESP32 used to drop/reconnect while driving: power noise suspected; capacitors planned).
2. Measure the footprint; calibrate odometry (docs/mapping.md step 1: 5 m push, gyro_scale), then record the floor
   and build the map offline (`scripts/map_offline.sh`), check 3 lengths (< 1 %), run in localization.
3. Voice done in code (`docs/STEPS_voice.md`): wire the GPIO17 button, run `scripts/setup_voice.sh`, add the Groq key.
4. Camera + people avoidance done in code (`docs/STEPS_camera.md`): run the two setup scripts on the Pi, tune stop_m/slow_m.
5. Front docking with the ArUco marker done in code (`docs/docking.md`, firmware v2.7 `D` mode): measure
   `dock_contact_dist_m` + the camera mount, calibrate the camera, try on Yogit's dock.
6. LLM (commands + recovery) fixed in code 2026-11-10 (`docs/llm.md`): deploy, then app Operate → AI commands →
   Test AI with the real key; run `tools/eval_commands.py` / `eval_recovery.py` with the key for the report.
7. Install Docker on the Pi (make the GHCR package public first).
10. Kiosk + person/object blocker (2026-11-14): mount the phone, open Head mode, try Go / Ask / Deliver and the
   PIN-last flow; stand in the path (must ask) and put a chair there (must reroute silently).
9. Nav2 backend (2026-11-11): `bash scripts/setup_nav2.sh`, `nav_mode:=nav2`, first test on blocks (docs/nav2.md),
   then 10 trips per backend and compare.
11. Laptop mapping (docs/laptop_slam.md): laptop setup, `slam_host:=laptop`, map the department with the laptop on board.
8. Conversation agent + map fixes (2026-11-10): flash firmware v2.8, deploy, write the notebook in the app, run
   `tools/eval_chat.py` with the key; map one corridor and check `odom_guard` in /robot/status and the jump log.

Deadlines: ROSCon India proposal **8 Nov 2026**; IndiaAI Fellowship **30 Nov 2026**.

## Session log (newest first, one line each)

- 2026-11-20: lab 19 Nov ended well: right encoder fixed, Spin tests -> gyro_scale 1.043 (repo), department map built
  cleanly on the Pi until the battery died (no warning: INA226 unplugged). Speeds +10 %: navigator 0.20, joystick and
  goal_controller 0.22 m/s; turn rates unchanged (map quality). Next: full charge, map in one go, Freeze.
- 2026-11-19 (lab): the robot ran OLD code: robot.service sourced ~/ros2_ws/install while the Pi builds in
  ~/slam-robot/ros2_ws (so slam_host:=laptop was ignored, the Pi ran its own Cartographer). robot.service now
  sources ~/slam-robot/ros2_ws first, then ~/ros2_ws, and logs which. Copy it to /etc/systemd/system once.
  Cartographer aborted at start on every machine: cartographer_odom.lua (17 Nov) had the unknown key
  POSE_GRAPH.constraint_builder.global_sampling_ratio: removed, test guards constraint_builder keys.
  Motor board dropped while RViz on the laptop subscribed (bridge publishes from its serial reader thread): mapping without RViz.
- 2026-11-19 (lab, RViz): on the Dell G15 RViz froze ('not responding', Map display GLSL link error, Wayland) and
  two RViz were open (laptop_slam starts one). laptop_slam.py now opens RViz under X11 (xcb) on the NVIDIA card
  (PRIME offload) when nvidia-smi works, `--gpu auto|nvidia|intel|software`, log ~/rviz_log.txt, reopens it after a
  crash/Force Quit (3x), `--rviz-only`; hint after 30 s with no /slam_host. robot.rviz: /map Transient Local, 10 fps.
- 2026-11-19 (review): pyflakes clean; read navigator/dashboard/bridge/launch/laptop_slam. Fixed: dashboard
  `_apply_start_pose` ran in a timer and waited for Cartographer's service -> deadlock (default mutually-exclusive
  callback group), so "placed where frozen" always timed out: now a thread. Launch: respawn (3 s) for bridge, LiDAR,
  navigator, dashboard, voice, vision, grid node; Cartographer only in localization (mapping restart = blank map).
  Spoken "Going outside X" / "I am outside X". Guards in test_compute.py. Known, not changed: laptop Cartographer
  crash restarts a blank map (autosave is the backup); LiDAR stop zone uses robot_half_width 0.25 vs body 0.21
  (measure, docs/STEPS_lab_19nov.md 10b).
- 2026-11-18 (doors/rooms): lab reports: robot stopped at doors, "no path" where the map has space, rooms only as
  rectangles. Side ultrasonics heard door frames: in-path zone = body (0.21+0.03 m), echoes within `us_wall_tol`
  0.12 m of a mapped wall ignored, whole arcs explained by the map never remembered (they closed doors for 60 s);
  planner despeckle + unknown doorway streak fill + tight-radius fallback ("narrow passage"); app Room tool takes
  tapped corners (any shape, Undo, Save room); relative places ("corridor near X", "X ke bahar/paas/saamne",
  steps `side: outside`, LLM `sides`, relative.outside_spot walks out through the door). test_doors_rooms.py (13).
  Not run on the robot.
- 2026-11-18 (night): the Pi's local edits (marker 0.0725, mic/speech_device plughw:CARD=Device,DEV=0, lua slices) are
  now in the repo yaml, so `git checkout -- config/ && git pull` on the Pi never conflicts. docs/STEPS_lab_19nov.md.
- 2026-11-18 (CPU review): tf2_ros (Humble) TransformListener(buffer, node, spin_thread=True) adds the GIVEN node to a
  second executor: navigator, dashboard and goal_controller were each spun by two executors (every timer/scan//tf woke
  both: idle CPU; a slow callback on the TF thread froze the TF buffer -> 'position lost' stalls). Now
  TransformListener(buffer, None, spin_thread=True) (own small node). test_compute.py guards it. Profiled the rest:
  NavigatorCore.step idle 0.002 ms, costmap rebuild 0.08 s (x86) on an 800x600 grid: not the CPU users.
- 2026-11-18 (review of mapping/saving): .pbstream saves (Freeze, autosave) now go to <name>.pbstream.part and are
  renamed when complete (a crash/restart mid-save never leaves a broken map or overwrites the last good autosave);
  autosave pruning always keeps the biggest autosave (three quick restarts on 18 Nov could have pushed out the long
  drive); Freeze stores the robot's map pose in map_mode.json and the dashboard starts localization there after the
  restart (map_mode.startup_pose, once, <= 15 min old; _apply_start_pose retries 90 s). Tests in test_map_mode.py.
- 2026-11-18 (lab): partial CSE map frozen (Cse_department_aids, 15 MB) but localization crashed at once: Cartographer
  "Check failed: last_dispatched_time_ <= next_data->GetTime() ... Non-sorted data added to queue (scan)" (exit -6):
  10 subdivisions per A1 scan overlap the next scan by ~0.5 ms -> num_subdivisions_per_laser_scan = 1 and
  num_accumulated_range_data = 1 in cartographer_odom.lua + cartographer_localization.lua (no scan unwarping).
  Same day: INA226 wiring took the I2C bus down (IMU 'not fitted', battery 'no sensor') -> INA unplugged for now;
  jerky/too fast driving after rewiring (suspect a loose encoder wire, as on 30 Oct); marker_size_m 0.0725 on the Pi.
- 2026-11-18 (dock): no charger on the dock yet, geometry-box plates, no springs. Firmware S3 **v2.9**: GPIO 1 = dock
  contact sense (dock 12 V adapter on the plates -> 1 k / 200 R divider, DCK,<0|1>,<mV>, hysteresis 6/4 V, 0.3 s),
  `#error` if USE_BAT_ADC too (INA226 now measures the battery). protocol DockPacket, bridge status `dock`, navigator
  core.set_dock_contact, docking counts dock voltage as contact ("docked: on the contacts"). Wiring S6-S10 in
  CONNECTIONS_esp32s3.md (INA226 voltage-only with the stock R100 shunt). docs/STEPS_lab_18nov.md. Firmware CI to check.
- 2026-11-18 (later): listening test on the laptop: both Hindi voices read English badly, so back to two voices
  (`speech_one_voice: ''`), but both women: `speech_voice_hi: hi_IN-priyamvada-medium` (+ en_US-lessac for English).
  Chat prompt gets the Hindi verb gender of the voice (ChatAgent.gender_rule: ja rahi hoon / ja raha hoon).
- 2026-11-18: one robot voice: kiosk answers came in two people's voices (en_US-lessac for English, hi_IN-pratham for
  Hindi). speech.pick_lang + `speech_one_voice: hi` (default): the Hindi Piper voice reads English too (Indian accent),
  so it is always the same person; `speech_voice_hi` takes a name (hi_IN-priyamvada-medium female / hi_IN-pratham-medium
  male, Piper's only two Hindi voices); `setup_voice.sh voices [device]` plays every voice in English + Hindi to pick by ear.
- 2026-11-17 (night): laptop mapping: Pi `slam_host:=laptop` starts no Cartographer while mapping; dashboard publishes
  latched /slam_host (host, mode, map, boot, save_dir, map_dir); scripts/laptop_slam.py on the laptop (Ethernet) starts/
  restarts/stops Cartographer + grid node from it, records ~/bags/laptop_<time>, RViz, rsyncs saved .pbstream files to
  the Pi; dashboard write_state goes to the laptop's save_dir and waits for the copy; app Map card shows laptop status +
  warning when /map stops (robot stops via map_tf_timeout). cartographer_laptop.lua (10 cm nodes, 4 threads, sampling 0.3).
  test_laptop_slam.py. Not run on the robot yet.
- 2026-11-17: department mapping run on 16 Nov lost its map: no fan, Pi ~80 °C at 100 % CPU, never saved
  (map was steadier than before). map_mode.PeriodicMapSaver: dashboard autosaves the live map every 3 min
  (`map_autosave_s`) and at once when hot, as ~/maps/autosave_<start>, newest 3 kept, Map tab shows it.
  Fan on Pi pins 4 (5 V) + 14 (GND). Robot voice stays on the Pi: Piper slower (`speech_rate` 1.15,
  `speech_pause_s` 0.3) and streamed into aplay; chat answers carry `speak` (Devanagari for Hindi/Hinglish,
  checked against the reply) for the hi_IN voice; kiosk skips the second round trip for chat answers and says
  "One second." / "एक सेकंड।" after 2.5 s; voice_reply never rate-dropped. Tests in test_map_mode/test_speech/test_chat.
  Lab same day: the autosave check crashed the dashboard 5 s after start (PiHealth 'throttled' is a string on the
  Pi) -> pihealth.pi_is_hot() parses every format, the tick is wrapped so autosave can never stop the app.
  Robot mic: USB card found, button OK, but the lapel mic records very quietly (peak ~900/32767, likely a phone-
  style TRRS plug in the TRS mic jack) -> voice.boost() scales quiet recordings up to half scale (max x30, never
  plain silence); TRRS->TRS splitter suggested. mic_device/speech_device set to plughw:CARD=Device,DEV=0 on the Pi.
  Splitter fixed the mic. Then Whisper wrote spoken Hindi in Urdu script and the chat replied in Urdu script (the
  answer itself was right: HOD Dr. Shashi Gurung) -> GroqWhisper retries with language=hi when the text is Urdu
  script (Devanagari back); chat prompt: replies in English or Roman Hinglish, never Urdu script; Speaker never
  reads Arabic script (says 'Sorry, please say that again.'). Right wheel would not go forward: R_EN/RPWM wiring.
  Evening: department map half done, then 100 % CPU, map tore (labs snapped together) and Freeze/autosave failed
  ('Cartographer not answering', write_state waited only 10 s). Fixes: cartographer_odom.lua motion filter 0.20 m / 3 deg
  (was 0.05 m = a node per scan), optimize every 60, constraint sampling 0.15, min_score 0.72, max distance 10 m; grid
  every 4 s; navigator costmap rebuild <= every 8 s when idle (`map_rebuild_idle_s`); dashboard write_state waits
  `map_save_timeout_s` 120 s, Freeze saves in a background thread (switching.saving) and reports `slam_error`.

- 2026-11-16 (lab, later): Test AI works on the robot (gpt-oss-20b, ~0.8 s), but the rules were "sure" about
  long Hinglish multi-step plans and flattened them. commands.py `_multi_step_doubt` (SEQ_HI step words, a place
  named twice, a missed time, home mid-trip) now sends those to the LLM; build() keeps revisits and a `home`
  stop mid-trip (navigator drives it to the home spot). Tests in test_llm.py + test_navigator_core.py. docs/llm.md §6.

- 2026-11-16 (lab): IMU replaced and working, firmware v2.8 confirmed, /odom 50 Hz, /scan 7.2 Hz, throttled 0x0,
  robot drives well in one room; navigator + dashboard ~60 % CPU each at idle (to check). robot_check.sh fixed
  (set -u before sourcing ROS made it exit silently). Department facts: plain paragraphs in ~/maps/info.txt
  are read automatically (knowledge.parse_info, reloaded on change, read-only in the app);
  tools/import_knowledge.py; test_import_knowledge.py. Next: department map (docs/mapping.md).

- 2026-11-16: lab kit for the next robot day: docs/STEPS_lab_16nov.md (flash v2.8 + I2C scan, code to the Pi,
  APK, link watch, 5 m push + spin, guard counters, footprint, map + 10 trips, person/chair, language + kiosk,
  optional Nav2, results table). wheel_separation 0.294 now in the repo yaml + xacro default (2 Nov spin test).
  Robot image CI fixed (test_nav2_follower skips its PyYAML check when yaml is missing).

- 2026-11-14: head-mode kiosk rework: side icons Go / Ask / Deliver, **PIN asked last** (after choosing, just
  before the robot starts; questions never need it; `needPin(why, title)`), Ask sheet (dry-run command: answers
  shown + spoken at once, trips shown then PIN), tap the face = the robot introduces itself (3 lines, one per
  tap), new face (visor, glowing eyes, eased moods, lid slant by clip-path, glances, `talk()` mouth, `look()`),
  PIN pad two-column on a sideways phone, STOP above every sheet. Person or object in the way (blockers.py,
  docs/navigation.md): person -> "Excuse me, may I pass?" + wait `person_wait_s` 8 s + "Thank you"; object (no
  person for 1.2 s) -> replan at once; no camera -> old behaviour; follower `blocked_limit`; mission log
  blocked_person / blocked_object; status `blocker`. test_blockers.py. Not tried on the robot.

- 2026-11-11: Nav2 as a switchable backend (docs/nav2.md): launch `nav_mode:=nav2` starts controller/planner/
  behavior servers + bt_navigator (config/nav2_params.yaml: NavFn, Regulated Pure Pursuit, cmd_vel remapped to
  cmd_vel_nav2); navigator `_setup_nav2` swaps the core's follower for nav2_follower.Nav2Follower (NavigateToPose
  or FollowPath, Pathik's front/body/speed gate on Nav2's command, BLOCKED on abort -> usual replans/recovery);
  falls back to the built-in planner when Nav2 is not installed; scripts/setup_nav2.sh; test_nav2_follower.py.
  NOT run with ROS yet. Decision: feature freeze, one use case (document delivery), 100-trip reliability trial.

- 2026-11-10 (later): conversation agent (chat.py, docs/chat.md): notebook of college facts (knowledge.py,
  ~/maps/knowledge.json, app Operate → Knowledge, admin edits), questions routed away from the command path
  ("library timings?" no longer drives), one strict-JSON call with notebook + places + robot state + 6 turns of
  memory, optional second lookup, guards (source enum, numbers must be in the notes, trips via CommandParser.build),
  offline read-out, app conversation thread on Send, tools/eval_chat.py, test_chat.py. Map displacement review
  (docs/mapping.md last section): odom_guard.py (impossible packets dropped, lost lines recovered from totals,
  heading held when still + gyro drift learned), firmware S3 **v2.8** (ODM totals; reflash), scan unwarping
  (10 subdivisions) + stricter far matches in localization (untested on the robot), map-jump log + counter.

- 2026-11-10: LLM never worked: Groq retired llama-3.x (16 Aug 2026) -> openai/gpt-oss-20b / -120b (old names
  remapped), Cloudflare 1010 blocked the Python-urllib User-Agent (now pathik-robot/1.0, voice too), errors hidden.
  New llm.py (GroqClient: classified errors + hints, model fallback, strict json_schema with place enum, LLMLog,
  diagnose()); commands.py llm_policy smart|always|off (rules first, stop never sent, 10 min cache, urgent clamp);
  parse in a worker thread off the core lock; dashboard/app timeouts 16/20 s; app Operate → AI commands page
  (status, last error, counters, Test AI, call log); test_llm.py; docs/llm.md.

- 2026-11-02 (later): map mode in the app (Map tab card, map_mode.py): Freeze map (finish trajectory + write_state ->
  ~/maps/<name>.pbstream, then localization on it), Continue mapping (load the map unfrozen, keeps the frame),
  Use a saved map, New map. Choice in ~/maps/map_mode.json, read by bringup (wins over ROBOT_ARGS; missing map ->
  mapping). Switch = dashboard exits 42 -> launch Shutdown -> robot.service Restart=always. GET/POST /api/slam
  (operator, idle only), demo mode, test_map_mode.py. Pi needs the new robot.service copied once.

- 2026-11-02: lab: LiDAR 14 cm behind the axle (laser_x -0.14), spin test says wheel_separation 0.294 (set on the Pi).
  Ultrasonic-only blockers: replans used only the LiDAR scan, so a low box stopped the robot, every new route went
  straight back into it, and the errand gave up and came home. Navigator now remembers the ultrasonic echoes across
  each beam when blocked (grown 0.3 m deep + 0.15 m berth, kept `us_memory_s` 60 s) for every plan; `blocked_after`
  5 s before replanning (YAML). test_us_blocker.py (box invisible to the LiDAR: drives around it; without the memory it fails).
  Then: per-sensor side (each echo along its own beam -> log 'on the left/right/ahead'), outline keeps growing while
  blocked and 15 s after (us_watch_s), echoes on mapped walls skipped, 5 cm cell memory (bounded); test: box on the right -> passes left.

- 2026-11-01: head mode greeting (first touch, navigator `say` -> robot speaker) + errands (mission.js: go, door
  check, message, wait / until OK, back here (goto_pose of the start) / home / stay, spoken report; test_errand.py)
  + camera door check (doorcheck.js, teach closed/open per place, test_door_check.py) + peak test (peak.html,
  peak.js, peakcheck.js verdict, Android deviceStats/openPeak; test_peak_check.py). PersonTracker: prediction,
  best-pairs-first, low-conf boxes (track_conf 0.25) only continue tracks, same in vision.js (test_tracker.py).
  Web YOLO now in a worker (detworker.js): face 60 fps instead of 23 on the 1-thread engine.

- 2026-10-31 (late): native phone detector: android NativeDetector.java (TensorFlow Lite 2.16.1, GPU delegate vs
  XNNPACK on cores-1 threads, benchmarked at start, faster kept), bridge detectorInit/detect/hasNativeDetector ->
  window.__pathikDetDone(id, json); detector.js tries native first, else ONNX Runtime Web (1 thread in a WebView).
  Model web/app/vendor/yolo11n_320.tflite = the 320 ONNX converted by onnx2tf (fp16, pixel boxes, same as ONNX)
  in fetch_phone_detector.sh + scripts/check_phone_model.py; build.gradle noCompress 'tflite', arm ABIs only.

- 2026-10-31 (night): head mode (docs/head_mode.md): web/app/head.html + head.js + face.js (11 moods from /api/state,
  PIN kiosk, STOP without PIN, OK / recovery answers, demo + phone-camera test) + detector.js (shared with detect.js);
  phone detects on the Pi camera's plain frame (/api/camera.jpg?raw=1 -> vision/raw/compressed) and POSTs boxes to
  /api/vision/phone -> /vision/phone_boxes; vision node uses them instead of its YOLO (phone_timeout 1 s fallback),
  ActivityWatch pauses detection when idle (idle_pause_s 30); Android openHead. Tests: test_head_mode.py, test_head_face.py.

- 2026-10-31: lab day: encoder GND fixed, wheel tuning reset to defaults, speed loop verified (±200 mm/s both wheels),
  ticks from a 5 m drive (wheels 4.669 m vs real 4.590 m) -> 5048/5063 set ON THE PI (not yet in the repo yaml),
  IMU still silent on I2C. Phone detector (lab): web/app/detect.html + detect.js + vision.js (JS port of
  vision_rules.py, checked against it by test_phone_detector.py), YOLO11n ONNX 320 on ONNX Runtime Web, phone camera
  or robot camera source; APK serves it from https://appassets.androidplatform.net (COOP/COEP, CAMERA permission);
  model + ORT fetched at APK build by scripts/fetch_phone_detector.sh (web/app/vendor/ is git-ignored).

- 2026-10-30 (night): accurate mapping (docs/mapping.md): cartographer_odom.lua retuned for corridors with good
  odometry (no online correlative, ceres translation 20 / rotation 40, odometry weights 1e5, submaps 90, range 10 m,
  loop closure 0.65/0.70); offline rebuild `scripts/map_offline.sh record|build` + cartographer_mapping_hq.lua
  (4 threads, sampling 0.5, 400 final iterations) -> pbstream + pgm; bridge `gyro_scale`, app Spin test suggests it
  when the gyro is the heading source; serial_probe `--push --metres 5`.

- 2026-10-30 (evening): Pi compute pass (docs/compute.md): localization runs draw the frozen map only (grid node
  `-include_unfrozen_submaps false`, 10 s); launch `cpu_plan` (cpu_plan.py: vision on core 3 nice 10, dashboard/voice/
  grid nice 5); map->odom 20 Hz; numpy scan code (scan_points.py) in navigator + dashboard (lazy, per app request);
  dashboard state cache 0.1 s, numpy map bytes, gzip 1; camera out_fps 5 via videorate + ISP BGR (videoconvert
  fallback); ArUco only while docking (`/vision/marker_request`, core.wants_marker); YOLO never twice on one frame;
  Pi health (pihealth.py: temp/CPU/throttle) in /api/state + app Health row + warning toast; app polls 3/s when still;
  tools/cpu_report.py. Tests: test_compute.py.

- 2026-10-30: robot voice button policy (navigator_core `_button_command`): trips only from the home spot/dock,
  normal priority, 1 per 30 s; stop/OK/questions anywhere; admin switch (app Me → Robot, `button_settings`,
  saved in button.json). Network loss (`netcheck.py`, gateway ping every 10 s): accepted trips are finished,
  then idle + offline 2 min -> go home and wait (`offline_home_s`); "connection back" event lists what happened.
  voice_node tags button vs app recordings (app = the logged-in user). Tests: test_button_offline.py (16).

- 2026-10-29: wrap-up of phases 0-6: README feature list/layout/status, docs/KEYS.md (every key + account and its
  ~/robot.env line), final PR summary with robot tests and what still needs hardware.
- 2026-10-29: Phase 6 URDF: pathik.urdf.xacro (base box from the footprint, wheels, caster, laser, camera + optical,
  imu_link, us_*), robot_state_publisher in bringup (urdf:=false keeps static TFs), urdf_model.py fallback renderer,
  RViz RobotModel, test_urdf.py (frames/offsets vs launch defaults + params, same as real xacro).
- 2026-10-29: Phase 5 voice: voice.py (WAV prep, Groq Whisper multipart, Vosk EN/HI, fallback, PushToTalk, GpioButton),
  voice_node (GPIO17 + /api/voice), speech.py Piper/espeak queue + rate limit + volume + Hindi voice, "Going to X",
  spoken numbers match places, setup_voice.sh, STEPS_voice.md, WAV fixtures; docker ttyAMA0/gpiochip rules.
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
