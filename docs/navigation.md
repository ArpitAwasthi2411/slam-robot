# Navigation app: places, routes, missions, commands

Everything here runs on the Pi with **no internet and no Nav2 install**. It's built into the
`navigator` node, which `bringup.launch.py` starts by default (`nav_mode:=planner`).

![Dashboard: route preview, queued mission, places](images/dashboard.png)

## The workflow (once encoder odometry is calibrated)

### 1. Map the floor once
```bash
ros2 launch lidar_robot bringup.launch.py
```
- Drive slowly around the whole floor with RC or the dashboard joystick, and come back to the
  start (loop closure).
- Dashboard → **Status → Save map**, name it e.g. `cse_floor`. You get:
  - `~/maps/cse_floor.pgm` + `.yaml`: an image of the map, which can also be used with Nav2's map_server
  - `~/maps/cse_floor.pbstream`: Cartographer's full state, needed for step 2

### 2. Every run after that: localise on the saved map
```bash
ros2 launch lidar_robot bringup.launch.py slam_mode:=localization map:=~/maps/cse_floor.pbstream
```
- Start the robot roughly where mapping started. Drive a metre or two.
- Check that the red scan points sit on the black walls of the map, then press **Position OK**.
  The navigator refuses to drive until you do: a wrong pose means a wrong route.
- Why this matters: a fresh mapping run starts a new map frame, so saved places would point at the
  wrong spots. Localization keeps the same frame every run.

### 3. Teach it places
- Drive to the HOD office door, then **Go → Places → Save here** and type `HOD office`, with
  aliases `hod, head of department`.
- Or **Add place** on the map: click the spot and drag to set the direction the robot should face.
- Places are saved in `~/maps/places.json` (one file per saved map; copy it next to the
  `.pbstream` when you back up the map).
- **Set home** marks the parking spot. "Go home" and the command "come back" drive there.

### 4. Use it
| You do | Robot does |
|---|---|
| Search "hod" → **Route** | Shows the planned route (amber), its length and ETA. Nothing moves yet |
| **Go** (on the route card or a place) | Plans an A* route around walls, follows it, turns to the place's heading |
| **Go here** tool, then click the map | Shows a route preview first, then **Go** |
| RViz **2D Goal Pose** | Same planner, so RViz goals now go around walls too |
| Type "urgent: take this to the HOD office then lab 3" | Parses the command, then queues a 2-stop mission at priority 3, which pauses the current normal one |
| Something blocks the corridor | Slows down, stops at 38 cm, waits 2 s, plans a new route around it (max 3 times), then fails with a reason |
| E-stop | Missions pause. After release they stay on **HOLD** until you press **Resume** |
| Touch the joystick or RC | Manual always wins, and the current mission is canceled |

## Commands: LLM with an offline fallback

- **With internet** (Pi on Wi-Fi): `export GROQ_API_KEY=gsk_...` before `ros2 launch`. The navigator
  sends the command plus the list of known places to Groq and gets back
  `{intent, targets, priority, reply}`.
- **Without internet, or if the LLM call fails**: a built-in rule parser handles
  "go to / take / deliver ... then ...", typos ("principl offce"), urgency words, "come back", "stop"
  and "where are you". The dashboard marks each answer **LLM** or **OFFLINE**.
- **Hallucination guard**: any place the LLM names that isn't on the map is dropped and reported,
  never driven to.

That fallback, plus the guard, is your answer to the ELLMER limitation ("cloud-dependent with no
fallback").

## Failure reasons (foundation for LLM error recovery)

Every failed mission records a code and a sentence, shown in the dashboard and the activity log:

| Code | Meaning | Typical recovery |
|---|---|---|
| `GOAL_IN_OBSTACLE` | target is inside a wall or furniture | move the place, or pick the nearest free spot |
| `OUTSIDE_MAP` | target outside the mapped area | map that area first |
| `NO_PATH` | no free route (door closed or unexplored) | try later, or another route/entrance |
| `BLOCKED` | something stayed in the way after 3 new routes | ask a human, wait, or skip the stop |
| `START_BLOCKED` | robot too close to an obstacle to plan | back off 30 cm |
| `POSE_LOST` | SLAM pose stale for over 3 s | stop, relocalize |
| `INTERNAL` | software error (logged with a traceback) | report |

Phase 7 idea: feed `{code, text, mission, places, time}` to the LLM and let it choose among a fixed set
of safe actions (retry later / skip stop / reorder stops / notify user / go home), each validated
before running. Same pattern as the command parser: the LLM proposes, the code checks.

## Request API (for the app, scripts, and your FastAPI server)

Send a JSON `std_msgs/String` to `/navigator/request`; replies arrive on `/navigator/response`. Over
HTTP, the dashboard forwards `POST /api/nav` with the same JSON.

```jsonc
{"type": "goto_place", "name": "HOD office", "priority": 1}
{"type": "goto_pose", "x": 3.2, "y": 1.0, "yaw": 1.57, "label": "lab door"}
{"type": "command", "text": "urgent take this to hod then lab 3"}
{"type": "preview", "name": "Lab 3"}             // route only, no motion
{"type": "cancel"}  {"type": "cancel", "id": 7}
{"type": "resume"}  {"type": "go_home"}  {"type": "confirm_localization"}
{"type": "add_place", "name": "Lab 3", "x": 2.2, "y": 1.8, "yaw": -1.57, "aliases": ["computer lab"]}
{"type": "add_place", "name": "Water cooler", "here": true}
{"type": "delete_place", "name": "Lab 3"}   {"type": "set_home", "here": true}
{"type": "search", "q": "hod"}
```
Status (4 Hz, `/navigator/status`): state, message, goal, remaining route, distance, ETA, missions
(active, queue, history), recent events, hold, localized.

Command-line example:
```bash
ros2 topic pub --once /navigator/request std_msgs/msg/String '{data: "{\"type\": \"goto_place\", \"name\": \"Lab 3\"}"}'
ros2 topic echo /navigator/status --once
```

## Tuning (`config/robot_params.yaml` → `navigator:`)
| Param | Default | What it does |
|---|---|---|
| `robot_radius` | 0.27 m | keep-out radius. Your corridor is 0.88 m wide, so 0.88 − 2×0.27 leaves 0.34 m of usable centre line |
| `soft_radius` | 0.60 m | prefer this much clearance from walls (keeps to the corridor centre) |
| `max_linear` | 0.22 m/s | cruise speed |
| `stop_distance` | 0.38 m | front clearance from the robot centre that stops it (440 mm chassis) |
| `dwell_s` | 3 s | wait at intermediate stops |
| `auto_return_s` | 0 | above 0: drive home after this many idle seconds |
| `allow_unknown` | false | plan only through mapped free space |

## Try it without the robot
```bash
python3 tools/dashboard_sim.py      # simulated CSE floor, same navigator code
# open http://localhost:8080
```
