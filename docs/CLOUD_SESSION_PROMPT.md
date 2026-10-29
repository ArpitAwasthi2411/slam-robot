# Prompt for the next cloud session (paste everything below the line)

---

You are working on **Pathik**, my autonomous indoor service robot (repo `ArpitAwasthi2411/slam-robot`).
Read `CLAUDE.md`, `docs/WORK_LEFT.md`, `README.md`, `hardware/CONNECTIONS_esp32s3.md` and skim the code before
changing anything. The robot is not available to you: everything must be built so it can be tested without it
(unit tests, `tools/dashboard_sim.py`, the app's demo mode, firmware CI), and then tried on the robot by me.

## Rules
- Commit as `Arpit Awasthi <awasthiarpit24@gmail.com>`. **No `Co-Authored-By` or `Claude-Session` lines.**
- Work in phases. After each phase: all tests pass (`cd ros2_ws/src/lidar_robot && python3 -m pytest -q`;
  pytest 6.2.5 in CI, never name a helper `setup()`), commit, push a branch and open a PR so I can merge it.
  Firmware changes must pass the `Firmware` workflow (check it with `gh api .../actions/runs`).
- Never commit keys. Keys come from `~/robot.env` / environment variables. If a feature needs a key it must still
  run without it and say clearly what is missing.
- Keep the existing HTTP API backward compatible unless you update every caller (app, classic page, MCP server, sim).
- Update `CLAUDE.md` (status + session log), `docs/WORK_LEFT.md` (tick tasks) and write a short "how to test on
  the robot" section for every phase in `docs/STEPS_next_run.md`.
- Ask me nothing mid-way unless something is truly blocking; make the sensible choice and write it down.

## Phase 0: full check of the repo (do first, report before Phase 1)
Go through every file: firmware, ROS package, launch/config, app, scripts, tools, docs, CI.
Find and fix: bugs, dead code, mismatched pin numbers between firmware and docs, params that no code reads,
docs that describe old behaviour (README status, `docs/roadmap.md` still says "no login", APK numbers),
scripts that would fail on the Pi (Ubuntu 22.04, ROS 2 Humble, user `arpit`), missing tests for core logic.
Write the findings to `docs/REVIEW_2026-10.md` (what was wrong, what you fixed, what still needs the robot).

## Phase 1: networking — control the robot from a phone on a different network
Next run: the robot joins **one phone's hotspot**, and we control it from **another phone that is NOT on that
network** (mobile data). Make this reliable and simple:
- Primary path: **Tailscale** on the Pi (already `scripts/setup_tailscale.sh`) + Tailscale app on the controlling
  phone, robot reachable as `slam-robot`. Harden the script (auth key from env `TS_AUTHKEY` for headless setup,
  MagicDNS name, start on boot, status check).
- Second path for phones without Tailscale: optional **Tailscale Funnel** (public HTTPS URL to port 8080). Logins
  are mandatory in this mode; add login rate limiting check, secure cookie/token handling over HTTPS, and a big
  warning in the app's Me tab when the robot is publicly reachable.
- The app/APK must connect by Tailscale name, Tailscale IP or Funnel HTTPS URL, remember robots, and show which
  path it is using. Latency-aware UI: joystick/teleop must stop safely if the link lags (watchdog already 0.5 s;
  show link delay; disable joystick above a threshold).
- `scripts/setup_wifi.sh hotspot` must keep working, with the hotspot as first priority and a fallback network.
- Write `docs/STEPS_remote_control.md`: exact steps for both phones.

## Phase 2: complete redesign of the Pathik app (production level)
The app (`lidar_robot/web/app/`, bundled in the Android WebView APK `android/`) has grown cluttered. Redesign it
from scratch for real users, keeping every feature reachable:
- Two clear audiences: **staff** (send/call the robot, see where it is, stop it) and **operators/admins**
  (drive, map, places/rooms, tuning, health, people). Staff should see almost nothing else.
- One primary action per screen, a calm home screen (robot status, battery, current mission, big STOP), a map
  screen, a "send / call the robot" flow (places, rooms, typed or spoken command with a plan preview), and an
  operator area that groups Lab/Tuning/Health/Logs/People/Settings instead of many tabs.
- Production quality: consistent design tokens, light and dark themes, large touch targets, works at 360 px wide,
  clear empty/loading/error states with what to do next, offline/link-lost banner, accessibility (contrast, focus,
  labels), no layout jumps, fast on a cheap Android phone. No external CDN at runtime (the robot has no internet
  on some networks).
- Keep demo mode working (`demo.js`) so the whole app can be shown without the robot.
- Show the new features from the phases below (recovery decisions, docking progress, voice, camera).
- Bump the APK through the existing `android.yml` workflow.

## Phase 3: LLM error recovery (thesis novelty, must work end to end)
When a mission step fails (`last_failure` and reason codes in `navigator_core.py`: NO_PATH, BLOCKED,
GOAL_IN_OBSTACLE, LOCALIZATION_LOST, timeouts…), build a recovery step:
- Context to the LLM: failure code, where, mission, queue, battery, time of day, recent history of that place.
- The LLM must choose from a **fixed menu of safe actions** (wait and retry after N s, re-plan with a wider
  margin, try an alternative route/waypoint, skip this stop, reorder the queue, ask a person in the app,
  go home, go dock). Validate the JSON (schema), clamp parameters, never invent actions.
- Rule-based fallback when there is no key/network; same menu.
- Every decision logged (mission log) with the reason, and shown in the app as "What happened / what I'm doing".
- `tools/eval_recovery.py`: scripted failures in simulation, LLM vs rules (recovery success rate, time to
  recover, unnecessary go-homes), results to `results/`.
- Key needed: **GROQ_API_KEY** (already used). Make the model configurable.

## Phase 4: front docking with an ArUco marker
The robot now docks **forwards** (the code still reverses in). Dock: marker centred at camera height, copper
contacts at the front of the robot (~8 cm and ~14 cm from the floor, final values come from Yogit).
- Navigator: go to a pre-dock pose in front of the dock, then visual alignment with the camera (OpenCV ArUco,
  default DICT_4X4_50, ID 0, 8 cm, all in config), creep forward ≤ 5 cm/s with lateral/heading correction,
  stop on contact (charging current from INA226 / battery voltage rise, or a timeout), back off and retry up to N
  times, report success/failure (feeds Phase 3 recovery).
- Firmware: a docking mode command that relaxes the 15 cm front ultrasonic stop (`US_STOP_MM`) only while docking
  and only at docking speed; leaving docking mode restores it. Keep the firmware CI green.
- Keep the detector interface `detect(frame) -> {found, x_m, y_m, z_m, yaw_deg}` so Neelaksh's module can drop in.
- Camera calibration loaded from a YAML (default values for Camera Module 3 Wide at 640×480).
- Tests with synthetic marker poses; a laptop webcam script to test detection without the robot.

## Phase 5: voice
- Push-to-talk: the app's mic button (already exists) **and** a physical button on the Pi (pick a free Pi GPIO,
  not 14/15 which are the ESP32 UART; document the wiring in `hardware/CONNECTIONS_esp32s3.md`).
- Record from the USB sound card → speech-to-text with **Groq Whisper** (online) and **Vosk** (offline fallback,
  small English + Hindi models, download in a setup script) → the existing CommandParser → spoken reply.
- Text-to-speech: **Piper** (offline, English + Hindi voices) replacing espeak in `speech.py`, rate-limited,
  volume configurable, robot announces events (going to X, arrived, blocked, docking, low battery, recovery decision).
- `scripts/setup_voice.sh`, tests with recorded WAV fixtures, `docs/STEPS_voice.md`.
- Keys: GROQ_API_KEY only (Vosk and Piper are offline).

## Phase 6: URDF
Replace the static TF publishers in `bringup.launch.py` with a Xacro URDF + `robot_state_publisher`: base_link
(footprint from `robot_params.yaml`), wheels, casters, laser, camera, imu_link, us_left/center/right, same frame
names and offsets as today, so nothing downstream changes. RViz config shows the model. Add a test that parses
the URDF and checks frames/offsets match the params.

## At the end
- One final PR summary: what changed, how to test each part on the robot, what still needs hardware.
- Update `docs/WORK_LEFT.md`, `CLAUDE.md`, and the README feature list.
- List every key/account I must set up and where it goes in `~/robot.env`.
