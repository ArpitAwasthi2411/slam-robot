# Pathik app: feature inventory (before the Phase 2 redesign)

Inventory of the mobile app `ros2_ws/src/lidar_robot/lidar_robot/web/app/` as of **Pathik-12 (v1.0.12) + Phase 1**,
written before any redesign change. The redesign may move things, never remove them. Every row has an ID; the
column **Now lives** is filled in after the redesign (the PR carries this table as the parity checklist), and
`test/test_app_parity.py` opens the demo as staff, operator and admin and checks the key rows.

Roles: **S** = staff, **O** = operator, **A** = admin (each role can do everything of the roles before it).
Role gating in the old app: `data-min="operator|admin"` hides for lower roles, `data-max="staff"` shows only to staff;
the server enforces the same rules again (`auth.py`, `STAFF_NAV`).

**Now lives** was filled in after the redesign (Phase 2, Pathik v3 layout): tabs **Home · Send · Map · Drive ·
Operate** for operators/admins and **Home · Send · Map · Me** for staff, STOP in the middle of the tab bar on
every tab, alerts (link lost, OK?, why, camera) in one stack under the top bar on every tab.

Added in the redesign (nothing removed): Home status card (state, battery, link, room) and "Send or call" button,
quick actions for every role, Home camera summary, recovery card (Phase 3 data), docking progress (Phase 4),
robot-voice card (Phase 5), inline route preview + Show on map, **Delete place** (operator), the Map sheet can
shrink to a strip, Android back button (dialog → sub-page → Home), phone font size respected in the APK (up to
130 %), offline state on Home.

## 1. Connect screen (`#connect`)

| ID | Feature | Roles | Endpoint / storage | Now lives |
|---|---|---|---|---|
| C1 | Find robot: probes saved robots + 10.42.0.1, 169.254.1.2, robot.local, robot, slam-robot, then scans subnets (native list from the APK or common hotspot ranges), 48 parallel probes | all | `GET /api/ping` || Connect screen (same place, restyled) |
| C2 | "Available robot" card: name, network type, delay ms, firmware; button becomes "Connect to …" | all | ping result || Connect screen (same place, restyled) |
| C3 | Progress / result line ("Scanning 192.168.43.x…", "No robot found…") | all | – || Connect screen (same place, restyled) |
| C4 | Try the demo (built-in simulated robot, no network) | all | `demo.js` || Connect screen (same place, restyled) |
| C5 | Saved robots list (last 5, "Live"/"Saved"), tap to connect | all | `localStorage pathik.robots` || Connect screen (same place, restyled) |
| C6 | Manual address: Wi-Fi IP, Tailscale name / 100.x, `https://…ts.net`; normalised (`:8080`, strips `/app/`), 4 s timeout, Tailscale hint on failure | all | `GET /api/ping` || Connect screen (same place, restyled) |
| C7 | "How do I connect?" help (robot Wi-Fi, same Wi-Fi, Tailscale, public link) | all | – || Connect screen (same place, restyled) |
| C8 | Theme toggle (corner) | all | `pathik.theme` || Connect screen corner (theme button) |
| C9 | Auto-connect: served by the robot → connect to its origin; else last robot; else scan | all | ping || Connect screen (same place, restyled) |
| C10 | "Voice commands ready" note | all | – || Connect screen, bottom line |

## 2. Login screen (`#auth`)

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| L1 | Robot name + network type, "Switch robot" back to Connect | all | – || Login screen (same place, restyled) |
| L2 | Log in (username, password, Show/Hide password) | all | `POST /api/auth/login` || Login screen (same place, restyled) |
| L3 | Request access (name, username, password) → "Request sent" pending card | all | `POST /api/auth/signup` || Login screen (same place, restyled) |
| L4 | First-time setup: create the admin account (`setup_needed`) | first user | `POST /api/auth/signup` || Login screen (same place, restyled) |
| L5 | Saved session per robot, re-checked with `/api/auth/me`; offline → still enters with the saved user | all | `GET /api/auth/me` || Login screen (same place, restyled) |
| L6 | Session ended (401) → back to login with a message | all | – || Login screen (same place, restyled) |
| L7 | Demo: one-tap Admin / Operator / Staff accounts | demo | `demo.js accounts` || Login screen (same place, restyled) |
| L8 | Older robot software without accounts → enters as admin ("legacy") | all | ping `auth` || Login screen (same place, restyled) |
| L9 | Error messages from the robot (wrong password, rate limit 429, pending) | all | – || Login screen (same place, restyled) |

## 3. Main screen frame

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| M1 | Live map canvas (occupancy grid, 1 m grid, LiDAR points, ultrasonic cones, robot body from footprint, rooms with names/contents, places, home, dock bracket, goal + heading arrow, tight spots, preview route dashed, active route, exploration frontiers/target) | all | `GET /api/map?since=`, `/api/state` || Map tab (and Drive tab): full-screen map |
| M2 | Map gestures: drag pan, pinch / wheel zoom (6–260 px/m) | all | – || Map / Drive tab |
| M3 | Follow the robot toggle (tool rail) | all | – || Map / Drive tab: tool rail → Follow |
| M4 | Long-press on the map → action sheet: route preview (length, ETA), Go here, Save as a place, Set home here, Cancel | O | `nav preview / goto_pose / add_place / set_home` || Map tab (operator): long-press on the map |
| M5 | Top bar pill: link dot (ok/warn/bad), robot host name, PUBLIC tag (Funnel), link text "Internet (Tailscale) · 180 ms"; tap → Lab Health (operator) or Me (staff) | all | `st.net`, rtt || Top bar on every tab; tap → Operate → Health (operator) or Me (staff) |
| M6 | Camera chip in the top bar (only when the robot has a camera / demo), dot coloured by the vision decision, toggles the camera card | all | `st.vision`, `st.camera` || Top bar on every tab; also Home → Camera button + "What the camera sees" card |
| M7 | Battery chip (% + bar, ⚡ while charging, tooltip V/A), hidden when no sensor | all | `st.nav.battery` || Top bar on every tab; also Home status card (Battery) |
| M8 | Mode chip (Idle, Stopped, Remote control, Running test, Waiting for OK, Docking, Leaving dock, Charging / On the dock, Scanning, Mapping, Patrol round N, Blocked · why?, To <goal>, Planning, At a stop, Driving, No motor link, Failed · why?, Offline); tap → "why did you stop?" (spoken) | all | `nav explain` || Top bar on every tab; also Home status card ("Pathik is …"); tap → why |
| M9 | Map stats box (Map tab): size in m or mapped m² + open edges + time while exploring | all | `st.map`, explore || Map tab, top centre |
| M10 | Mode bar for the active map tool (instructions, Undo, Loop, Go, Cancel, Done) | O (measure: all) | – || Map / Drive tab, under the top bar |
| M11 | Banner: motor board not talking (pins 6/8/10), waiting for position (LiDAR?), confirm the position | all | `st` || Alert stack under the top bar, on every tab |
| M12 | Link lost banner (after 3 failures): "It carries on with its job by itself; reconnecting…", toast, mode chip "Offline" | all | – || Alert stack on every tab + Home status card "out of reach" + top bar "Offline" |
| M13 | "Waiting for OK" card at a stop: "Pathik is at X", countdown, OK carry on | all | `nav confirm` || Alert stack on every tab |
| M14 | "Why" answer card (answer + up to 4 details, closes after 15 s) | all | `nav explain` || Alert stack on every tab (from the mode chip or Home → Why stopped?) |
| M15 | Vision note on the map ("Waiting for a person." / "Slowing down.") when the camera card is closed | all | `st.nav.vision` || Alert stack on every tab |
| M16 | Camera card: live JPEG ~5/s (2/s on slow links), demo drawing in demo mode, action (Path clear / Slowing down / Stopped: person / Camera only), fps + LiDAR, why text, people list (bearing, distance, coming closer / moving away), bigger/smaller, close; remembered open/closed | all | `GET /api/camera.jpg` || Floating card on every tab (top-bar chip or Home → Camera); summary card on Home |
| M17 | Map hint "Pick a tool on the left…" (fades after 7 s) | O | – || Map / Drive tab (operator) |
| M18 | Big STOP button (always on screen) | all | `POST /api/estop {on:true}` || Tab bar centre on every tab (bigger on Home) |
| M19 | Stopped overlay; operator: hold 1 s to release; staff: "someone from the robot team releases it" | all / O release | `POST /api/estop {on:false}` || Full-screen overlay (unchanged; release for operator) |
| M20 | Toasts (info / error), haptics (APK `haptic`, else vibrate) | all | – || Everywhere (unchanged) |
| M21 | Keep the screen awake while connected (APK `keepAwake`) | all | native || Unchanged |
| M22 | Polling: state 140 ms (400 ms on slow links, 2 s in background), map every 1.5 s, camera while open | all | – || Unchanged (faster while an Operate tool page is open) |

## 4. Map tools (tool rail on the map)

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| T1 | Flag / goal: tap = go there, drag = also face that way; "Goal sent" + Cancel | O | `nav goto_pose`, `cancel` || Map / Drive tab: tool rail → Flag (operator) |
| T2 | Draw a path with a finger → checked path (length, ETA, too-close-to-wall warning in red) → Loop switch → Follow this path / Redraw / Discard | O | `nav check_path`, `follow_path` || Map / Drive tab: tool rail → Draw (operator) |
| T3 | Route: tap up to 30 points (drag = heading), Undo, Loop (patrol), Go | O | `nav goto_poses` || Map / Drive tab: tool rail → Route (operator) |
| T4 | Measure a distance (+ heading) | all | – || Map / Drive tab: tool rail → Measure (everyone) |
| T5 | Room: drag a box → name, contents, other names → saved | O | `nav add_room` || Map / Drive tab: tool rail → Room (operator) |
| T6 | Locate / set position: press + drag heading → confirm → relocalize | O | `POST /api/relocalize` || Map / Drive tab: tool rail → Locate (operator) |
| T7 | Home button: go home | all | `nav go_home` || Map / Drive tab: tool rail → Home; also Home → Come back |
| T8 | Follow button | all | – || Map / Drive tab: tool rail → Follow |

## 5. Drive tab (operator)

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| D1 | Joystick (10 Hz commands, speed from `st.limits` × top speed) | O | `POST /api/cmd` || Drive tab (operator) |
| D2 | Joystick pause on a slow link (> 600 ms delay or 1.5 s silent): greyed, message, one stop sent, never queues | O | – || Drive tab (operator) |
| D3 | Top speed slider 20–100 % (remembered) | O | `pathik.speed` || Drive tab, right side |
| D4 | Readout: m/s and rad/s from odometry | O | `st.odom` || Drive tab, above the joystick |
| D5 | Full-screen map with tools while driving | O | – || Drive tab |

## 6. Go tab (staff: "Home")

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| G1 | Greeting with initials + "Pathik is in <room> · <mode> · battery N%" | all | – || Home, top (greeting); the same summary also in Map → Where is Pathik? |
| G2 | Quick buttons (staff): Come back, Why stopped?, Charge | S (others have them elsewhere) | `nav go_home`, `explain`, `go_dock` || Home → Come back / Charge / Why stopped? (now for every role) |
| G3 | Typed command (Enter sends, Shift+Enter new line) → dry run → plan timeline (Start › Go › OK? › Wait › Return / Charge), "urgent", missing home/dock note, Edit / Send to robot; stop/status/explain/find/confirm run at once | all | `nav command dry_run`, `command` || Send tab: command box → plan preview → Send to robot |
| G4 | Source of the understanding ("language model" / "offline") + reply + notes | all | – || Send tab, under the command box |
| G5 | Voice: tap and speak (APK speech recogniser `en-IN`, else browser), live partial text, level wave, Thinking…, spoken reply ("Shall I go?") | all | native `listen`, `speak` || Send tab: Tap and speak |
| G6 | Mission card: goal, distance + ETA, message, N queued, progress bar, Resume (when paused), Cancel mission; "Missions paused"; "Last mission failed" + reason | all | `nav resume`, `cancel` || Home: current mission card (Resume / Cancel mission) |
| G7 | Saved places: search (names, aliases, rooms, things), sorted by distance, "N m away", Home row, Route (preview toast) / Go | all | `nav preview`, `goto_place`, `go_home` || Send tab → Places (search, Route now shows length/ETA inline + Show on map, Go) |
| G8 | Save here (name a place at the robot) / Set home here | O | `nav add_place here`, `set_home here` || Send tab → Places header (operator) |
| G9 | Rooms list: name, "robot is here", things (search hits highlighted), Go; Edit → rename, contents, aliases, Delete room | all (Edit: O) | `nav goto_room`, `update_room`, `delete_room` || Send tab → Rooms (Edit for operator) |
| G10 | Charging: battery text (%, V, A, charging / low / critical / goes below 25 %), "Dock not set yet", Charge now | all | `nav go_dock` || Home → Charging card (Charge now) |
| G11 | Robot is on the dock (save dock here) | O | `nav set_dock` || Home → Charging card (operator) |
| G12 | Schedules: list (time, days, action, name, last run), on/off switch, delete; Add (name, time, days, go to place / home / dock / command) | O | `nav schedule_add / _enable / _delete` || Send tab → Scheduled trips (operator) |

## 7. Map tab

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| P1 | "Where is Pathik?" text + legend (robot, places & route, home, dock) | S | – || Map tab sheet: Where is Pathik? + legend (now for every role) |
| P2 | Map by itself: speed Careful/Normal/Quick (remembered), turn once at every spot (remembered), Start (safety confirm) | O | `nav explore_start` || Map tab sheet → Map by itself (operator) |
| P3 | While mapping: message, tiles (mapped m², elapsed, areas visited +skipped, open edges), Stop mapping | O | `nav explore_stop` || Map tab sheet → Map by itself (operator) |
| P4 | Last run summary (area, areas, time, saved as / saving / not saved) + one toast when the auto-save finishes | O | `st.autosave` || Map tab sheet → Map by itself (operator) |
| P5 | Save map (name) | O | `POST /api/save_map` || Map tab sheet (operator) |
| P6 | Position OK (only when the localisation needs a confirm) | O | `nav confirm_localization` || Map tab sheet (operator) |
| P7 | Mode note (running on a saved map / building a new map) | O | `st.slam_mode` || Map tab sheet (operator) |

## 8. Lab tab (operator)

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| B1 | Health tiles: network + address, link delay, motor board Hz + port, link drops (USB / ESP32 reboots + reason / gaps, last age), firmware, LiDAR scans/s, position, heading source, driving mode, commands from, navigator state, map size, ultrasonics, IMU, battery, bumper, body size, room | O | `st` || Operate → Health |
| B2 | Diagnostics log (last 14 events with time and level) | O | `st.nav.events` || Operate → Health |
| B3 | Switch robot (from Health) | O | – || Operate → Health (also Operate → Robot → Switch robot) |
| B4 | Wheels: live chart target/left/right (Live switch → telemetry), step test 0.15/0.2/0.3 m/s with confirm → frozen chart + metrics (rise, overshoot, steady error, wobble, left/right difference) + tips | O | `POST /api/esp {op:telemetry}`, `GET /api/telemetry`, `POST /api/test` || Operate → Wheel tuning |
| B5 | Wheel parameters (Kp, Ki, start-up PWM, top wheel speed, ramp): number + slider, help, ranges from ping, live push (debounced), "changed" marks, Save to robot, Factory values; old-firmware note | O | `POST /api/esp {op:tune / save / defaults}`, ping `wheel_ranges` || Operate → Wheel tuning |
| B6 | Path follower parameters (cruise speed, fastest turn, look-ahead, steering gain, ramps, turn on the spot thresholds): live, Save, Defaults | O | `nav get_tune`, `tune` || Operate → Path following |
| B7 | Drive tests: Straight line / Spin, speed + time choices, confirm, live chart (drift in cm or turned degrees vs odometry), metrics + advice (curve per metre; suggested wheel_separation) | O | `POST /api/test`, `GET /api/test` || Operate → Drive tests |
| B8 | Mission log: stats tiles (missions, success, avg time, avg speed, path efficiency, replans, distance, bumps · e-stops), last 40 missions, Refresh, Export CSV (APK share, else download + clipboard) | O | `nav mission_log` || Operate → Mission log |

## 9. Me tab

| ID | Feature | Roles | Endpoint | Now lives |
|---|---|---|---|---|
| E1 | Public link warning (URL + `funnel off` command) when the robot is on Tailscale Funnel | all | `st.net.public` || Operate / Me, top (red card) + PUBLIC tag in the top bar |
| E2 | Me card: initials, name, @username (or "no accounts"), role badge | all | – || Operate / Me, top |
| E3 | People (admin): list with status (wants access / disabled / last seen / never), role select (with role help), Approve / Reject, Enable / Disable, New password, Remove; Refresh | A | `GET /api/users`, `POST /api/users/update / reset / delete` || Operate → People (admin) |
| E4 | Pending sign-ups badge on the Me tab, checked every 30 s | A | `GET /api/auth/me` || Badge on the Operate tab and on Operate → People |
| E5 | Appearance: System / Light / Dark (follows the phone when System; APK status bar colours) | all | `pathik.theme`, native `setDarkBars` || Operate / Me → Appearance |
| E6 | Robot: name, network + address, dot; Switch robot | all | – || Operate / Me → Robot |
| E7 | Change password (logs back in with the new one) | all (not legacy) | `POST /api/auth/password`, `login` || Operate / Me → Account |
| E8 | Log out (confirm) | all (not legacy) | `POST /api/auth/logout` || Operate / Me → Account |
| E9 | Role legend (staff / operator / admin) | all | – || Operate / Me, bottom |

## 10. Shared dialogs and behaviour

| ID | Feature | Roles | Now lives |
|---|---|---|---|
| X1 | Name prompt (`ask`), confirm dialog (`confirmBox`), small forms (`formBox`, day buttons) | all || Unchanged (bottom sheets) |
| X2 | Theme before first paint (no flash), fonts bundled (Bricolage, Instrument; OFL), no CDN | all || Unchanged |
| X3 | The classic page (`/classic`) reads `pathik.session` (same login) | all || Unchanged |
| X4 | Speech output: APK `speak`, else browser speechSynthesis | all || Unchanged |
| X5 | Safe-area insets (notch / gesture bar) | all || Unchanged |

## 11. Demo mode (`demo.js`)

| ID | Behaviour | Now lives |
|---|---|---|
| Z1 | Simulated corridor robot, places, rooms with contents, fixed dock, home || demo.js unchanged |
| Z2 | Accounts admin/admin123, operator/operator123, staff/staff123 with the same role rules as `auth.py` || demo.js unchanged |
| Z3 | Missions queue, priorities, waits, OK stops, patrol, drawn paths, routes, cancel/resume, e-stop || demo.js unchanged |
| Z4 | Offline command parser (dry run plans), explain, find || demo.js unchanged |
| Z5 | Battery drain / charging, dock || demo.js unchanged |
| Z6 | Vision: simulated people ahead → slow / stop, drawn camera picture || demo.js unchanged |
| Z7 | Wheel telemetry + tests, follower tuning, mission log || demo.js unchanged |
| Z8 | Map by itself refused with a pointer to the laptop simulator || demo.js unchanged |
| Z9 | Never saved as a robot || demo.js unchanged |

## 12. Server endpoints the app calls (must stay)

`GET /api/ping`, `GET /api/state`, `GET /api/map?since=`, `GET /api/camera.jpg`, `GET /api/telemetry?since=`,
`GET /api/test?since=`, `POST /api/test`, `POST /api/cmd`, `POST /api/estop`, `POST /api/nav` (types: preview,
goto_pose, goto_poses, goto_place, goto_room, go_home, go_dock, cancel, resume, confirm, confirm_localization,
explain, command (+dry_run), check_path, follow_path, add_place, set_home, set_dock, add_room, update_room,
delete_room, schedule_add/_enable/_delete, explore_start/_stop, get_tune, tune, mission_log), `POST /api/relocalize`,
`POST /api/save_map`, `POST /api/esp` (tune, save, defaults, telemetry), `GET /api/auth/me`, `POST /api/auth/login /
signup / logout / password`, `GET /api/users`, `POST /api/users/update / reset / delete`.

Known gaps (from `docs/REVIEW_2026-10.md`) the redesign adds: delete a saved place (`nav delete_place`, server
already has it), and respect the phone's font size in the APK.
