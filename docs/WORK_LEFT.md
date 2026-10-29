# Work left on Pathik (as of 29 Oct 2026)

Every task has an owner and says whether it needs the robot. The **cloud** tasks are code-only:
a Claude cloud session can do them from this repo with tests, without the robot.
To start one, open a session on this repo and paste the task's **Prompt** line.
Tick a task off here (and in `CLAUDE.md`) when it is merged.

Deadlines: ROSCon India proposal **8 Nov** · IndiaAI Fellowship **30 Nov** · team leaves for internships in ~2 months.

---

## A. Code work a cloud session can do (no robot needed)

Ordered by value for the thesis, the demo and the applications.

- [x] **A1. LLM error recovery (thesis novelty #1)**: done 29 Oct (`docs/recovery.md`, `tools/eval_recovery.py`,
  rules 6/6 vs none 1/6 in simulation). Left: run the eval with the Groq key (LLM column), try on the robot (B13).
  When a mission fails (`last_failure`, reason codes like NO_PATH, BLOCKED, GOAL_IN_OBSTACLE), send the failure
  + context to the LLM, let it choose from a fixed menu of safe actions (retry after N s, try another route,
  skip stop, reorder queue, ask a person via the app, go home), validate the choice, execute, log it, and show
  "what I decided and why" in the app. Rule-based fallback when offline. Tests + an eval script with
  scripted failures (LLM vs rules: success rate, time to recover).
  Prompt: *"Read CLAUDE.md and docs/WORK_LEFT.md, then do task A1: LLM error recovery in navigator_core with a validated action menu, offline fallback, app card, tests and tools/eval_recovery.py."*

- [x] **A2. Front docking with an ArUco marker**: done 29 Oct (`docs/docking.md`; firmware S3 v2.7 / classic v2.5 `D` docking
  mode). Left: measure `dock_contact_dist_m` + camera mount, calibrate the camera, try on the real dock (B14).
  Navigator drives to a pre-dock pose, then aligns on the marker with the camera (pose from OpenCV ArUco,
  DICT_4X4_50, size from config), creeps forward ≤ 5 cm/s, stops on contact/charging current; retry if
  misaligned. Firmware: a command that relaxes the 15 cm front ultrasonic stop (`US_STOP_MM`) only while
  docking. `set_dock` stores the marker ID. Keep the interface Neelaksh's `detect(frame) → {found, x_m, y_m, z_m, yaw_deg}`.
  Prompt: *"Do task A2 from docs/WORK_LEFT.md: forward ArUco docking (navigator, vision, firmware docking mode, tests, docs)."*

- [x] **A3. Voice on the robot**: done 29 Oct (`docs/STEPS_voice.md`: GPIO17 push-to-talk + app button, Groq Whisper /
  Vosk, Piper TTS, announcements). Left: wire the button, run setup_voice.sh, try it (B15); Rashi's Vosk vs Whisper table.
  Push-to-talk button on a free ESP32 GPIO (8 or 47) or a Pi GPIO → record from the USB mic → speech-to-text
  (Groq Whisper, Vosk offline fallback) → existing CommandParser → answer spoken with Piper TTS
  (English + Hindi) replacing espeak in `speech.py`. Wake the app's mic button to the same path.
  Prompt: *"Do task A3 from docs/WORK_LEFT.md: voice node with push-to-talk, Whisper/Vosk, Piper TTS, tests, setup script."*

- [ ] **A4. "What do you see?"**
  `describe()` using one camera frame and Groq vision (Nemotron VL as an option), from the app (button on the
  camera card), voice and MCP. Rate-limited, works without a key (says so).
  Prompt: *"Do task A4 from docs/WORK_LEFT.md: describe-the-scene endpoint, app button, MCP tool, tests."*

- [x] **A5. Firmware: IMU not detected on the S3 + I2C scan**: done 29 Oct (`I` command, scan at boot, SDA/SCL in `?`,
  3 init tries; v2.7). Robot check: B2.
  Add an `I` (I2C scan) command that lists found addresses at boot and on demand; print SDA/SCL pins in `?`;
  retry MPU-6050 init; document the checks. Bump to v2.7, keep the firmware CI green.
  Prompt: *"Do task A5 from docs/WORK_LEFT.md: I2C scan command and IMU init retries in the S3 firmware."*

- [ ] **A6. Evaluation numbers for the report, ROSCon and IndiaAI**
  Run and extend `tools/eval_commands.py` (LLM vs rules, accuracy + latency, Hinglish split) and
  `tools/eval_nav.py` (simulated floor: success rate, path length, time), save CSV + charts in `results/`,
  and write `docs/results.md` with the tables. These numbers go in every application.
  Prompt: *"Do task A6 from docs/WORK_LEFT.md: run the evals in simulation, save CSV + charts, write docs/results.md."*

- [x] **A7. URDF + robot_state_publisher**: done 29 Oct (`urdf/pathik.urdf.xacro`, same frames; `urdf:=false` = old static TFs).
  Robot check: B16.
  Replace the static TF publishers in `bringup.launch.py` with a URDF/Xacro (base, wheels, LiDAR, camera,
  ultrasonics, footprint) and `robot_state_publisher`; RViz shows the robot model.
  Prompt: *"Do task A7 from docs/WORK_LEFT.md: URDF/Xacro + robot_state_publisher, same frames as today."*

- [ ] **A8. Campus "call the robot" flow**
  Staff press "Call Pathik to me" (pick their room) → mission with requester + priority → robot arrives,
  waits for "Done", returns home; requester sees live ETA and gets a notification when it arrives.
  Prompt: *"Do task A8 from docs/WORK_LEFT.md: call-the-robot flow for staff with ETA and arrival notice."*

- [ ] **A9. RC mode switch (CH5/CH6)**
  Physical MANUAL / AUTO / E-STOP switch on the transmitter (one more GPIO + firmware + bridge mode).
  Prompt: *"Do task A9 from docs/WORK_LEFT.md: RC CH5 mode switch in the S3 firmware and bridge."*

- [x] **A12. App redesign without losing features** (done 29 Oct: `docs/APP_FEATURES.md`, parity test). Robot check: B12.
- [x] **A11. Remote control from another network** (done 29 Oct: Tailscale auth key + boot, optional Funnel
  with login limits, app remembers robots by name/IP/URL and shows the path, joystick pauses on a slow link;
  `docs/STEPS_remote_control.md`). Robot check: B11.
- [x] **A10. Docs clean-up** (done 29 Oct: docs/REVIEW_2026-10.md)
  README status checklist, `docs/roadmap.md` (still says "no login"), APK number (Pathik-11 vs 12) and
  current-status lines in CLAUDE.md, one up-to-date architecture diagram in the README.
  Prompt: *"Do task A10 from docs/WORK_LEFT.md: bring README, roadmap and CLAUDE.md up to date with the code."*

## B. Lab work (Arpit, needs the robot)

- [ ] **B1.** Drive test on the S3 + UART with `tools/link_doctor.sh --watch`: 0 drops for 10 minutes.
- [ ] **B2.** IMU wiring check (SDA 11, SCL 12, 3V3, GND), then `?` shows `imu=ok` (after A5, use `I`).
- [ ] **B3.** Measure footprint + wheel separation; run the app's Spin and Straight line tests; send the numbers.
- [ ] **B4.** Camera: Pi online, run `scripts/setup_pi_camera.sh` then `scripts/setup_vision.sh` (`docs/STEPS_camera.md`); tune `stop_m` / `slow_m`.
- [ ] **B5.** Groq key: **create a new one** (the old one was shown in a screenshot) and put it in `~/robot.env`.
- [ ] **B6.** Audio: sound card, mic and speaker test (`docs/STEPS_esp32s3_lab.md` step 9).
- [ ] **B7.** Map the full floor, label places and rooms with contents, try "take me to the room with the 3D printer".
- [ ] **B8.** Install the latest APK; MCP test: Claude sends the robot to a room.
- [ ] **B9.** Record the demo video (mapping, a spoken/typed Hinglish command, driving to a room, stopping for a person) → portfolio, ROSCon, IndiaAI.
- [ ] **B13.** Error recovery on the robot: stand in the corridor, block a door, ask-a-person flow (`docs/STEPS_next_run.md`, Phase 3); run `tools/eval_recovery.py` with the Groq key.
- [ ] **B14.** Front docking on the real dock: print the marker (`tools/marker_webcam.py --print`), measure, run `docs/STEPS_next_run.md` Phase 4.
- [ ] **B15.** Voice: button on pin 11/9, `scripts/setup_voice.sh`, then the checks in `docs/STEPS_voice.md`.
- [ ] **B16.** URDF: `ros2 run tf2_tools view_frames` shows the same frames; RViz shows the robot model (`docs/STEPS_next_run.md`, Phase 6).
- [ ] **B12.** Try the redesigned app on a phone with each role (`docs/STEPS_next_run.md`, Phase 2).
- [ ] **B11.** Remote control: robot on phone A's hotspot, phone B on mobile data (`docs/STEPS_remote_control.md`).
- [ ] **B10.** Docker on the Pi (make the GHCR package public first), `scripts/pathik.sh install`.

## C. Team

- [ ] **Rashi:** voice results (Vosk vs Groq Whisper table, Piper EN/HI) → feeds A3; then people detection + describe() comparison → feeds A4.
- [ ] **Neelaksh:** webcam calibration, ArUco vs AprilTag accuracy table, `marker_pose.py detect()`, marker size/placement → feeds A2.
- [ ] **Yogit:** dock parts list + charger check + wiring + heights, cardboard prototype, then plywood → feeds A2.

## D. Writing

- [ ] **D1. ROSCon India 2026 proposal, due 8 Nov.** Needs B9 (video) and A6 (numbers).
- [ ] **D2. IndiaAI Fellowship brief (≤ 500 words) + documents, due 30 Nov.**
- [ ] **D3.** Major project report chapters: architecture, firmware, navigation, LLM layer, recovery (A1), evaluation (A6).
- [ ] **D4.** Paper draft from A1 + A6 (LLM recovery vs rules on a low-cost robot).

## Suggested order

1. **Cloud now, in parallel:** A1, A6, A5, A10.
2. **Lab this week:** B1, B2, B3, B5, B7, then B9 (video).
3. **Cloud next:** A2 (once Neelaksh's marker table is in), A3 (once Rashi's voice results are in), A4.
4. **Writing:** D1 by 8 Nov, D2 by 30 Nov.
