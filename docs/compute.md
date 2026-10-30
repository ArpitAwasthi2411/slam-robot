# Compute on the Raspberry Pi 4: what runs where, and how it is kept light

The Pi 4 has 4 cores at 1.5 GHz. When they are all busy, three things go wrong at once: odometry and
LiDAR data arrive late (the map smears and "position lost" pauses appear), the robot reacts late (to
people as well), and the Pi heats up until it slows itself down, which makes everything later still.
This page is the CPU budget and the rules that keep the robot inside it.

## 1. Run on a saved map (the biggest saving)

| Mode | What Cartographer does | CPU |
|---|---|---|
| **Mapping** (`slam_mode:=mapping`) | builds new submaps, searches for loop closures across the whole map, re-optimises | high, and grows with the map |
| **Localization** (`slam_mode:=localization map:=…pbstream`) | only matches each scan against the frozen map, keeps 3 small live submaps | low and constant |

Map the floor once, save it (app → Map → Save map), then run every day in localization mode:

```
ROBOT_ARGS=esp32_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 slam_mode:=localization map:=/home/arpit/maps/cse_floor.pbstream
```

In localization mode the map picture is drawn from the **saved map only** (`-include_unfrozen_submaps false`)
every 10 s: it never changes, so the navigator builds its costmap once, and a person standing in front of
the robot is never painted into the map as a wall (the live LiDAR scan and the camera handle people).

## 2. The CPU plan (launch argument `cpu_plan:=true`, the default)

| Program | Priority | Cores | Why |
|---|---|---|---|
| ESP32 bridge, navigator, Cartographer, LiDAR driver | normal (nice 0) | any | motion: must never wait |
| camera + YOLO (`vision`) | low (nice 10) | **core 3 only** | can never take more than one core, and only what's left |
| app server (`dashboard`), voice, map drawing | lower (nice 5) | any | people can wait 50 ms, the wheels can't |

Defined in `lidar_robot/cpu_plan.py`, applied as launch prefixes (`taskset -c 3 nice -n 10`).
Turn it off with `cpu_plan:=false` in `ROBOT_ARGS`.

## 3. What each algorithm costs and how often it runs

| Algorithm | Where | Rate | Kept light by |
|---|---|---|---|
| Wheel speed PI loop, encoders, IMU, ultrasonics | ESP32-S3 | 50 Hz | not on the Pi at all |
| Odometry + TF (`odom→base_link`) | esp32_bridge | 50 Hz | — |
| Scan matching + pose graph | Cartographer (C++) | every scan, 7–10 Hz | 2 background threads, motion filter (no new nodes while standing still), map→odom published at 20 Hz (was 50) |
| Map picture (occupancy grid) | grid node | every 2 s mapping / 10 s localization | frozen map only when localizing |
| Costmap (distance transform) | navigator, own thread | only when the map really changes | the map is hashed; unchanged maps are skipped |
| A* route | navigator | on a new task / replan only | 10 cm planning grid |
| Path following + body (footprint) checks | navigator | 20 Hz | ~0.2 ms per step |
| Scan → obstacle points, front clearance | navigator | every scan | numpy (`scan_points.py`), ~5× faster than the old per-ray loop |
| Camera | GStreamer | sensor 15 fps, **5 frames/s** reach Python | `videorate` drops the rest inside GStreamer; the Pi's ISP makes the BGR picture (falls back to `videoconvert` when it can't) |
| People detection YOLO11n (NCNN) | vision, core 3 | `det_rate` (2/s; 3 once the Pi has headroom) | 1 thread, 320 px input, never twice on the same frame |
| Dock marker (ArUco) | vision | **only while driving to / onto the dock** | the navigator asks (`/vision/marker_request`); was 10 Hz all the time |
| Camera picture for the app | vision | 5/s, **only while the camera view is open** | `/vision/stream_request` |
| App state (`/api/state`) | dashboard | per phone: 6/s while moving, ~3/s when still | built at most every 0.1 s for all phones together; scan points made only when an app asks; gzip level 1 |
| Map for the app | dashboard | when the map changes | numpy encoding (~100× faster) |
| Pi health (temperature, throttling) | dashboard | every 2 s | reads /sys and /proc only |

## 4. Heat

A Pi 4 starts slowing itself down at **80 °C** (and some firmware already at 60 °C under a "soft limit").
In a closed body it gets there in a few minutes under load. Software can only lower the load; the heat
has to leave the box:

- a **heatsink** on the CPU (and ideally the USB/RAM chips), and a **small 5 V fan** from the buck (not the
  Pi's 5 V pin if the buck is already near its limit), blowing across the Pi;
- **vents** in the body: air in low on one side, out high on the other; never seal the Pi's compartment;
- keep the Pi away from the motor drivers and the buck converter (both get warm).

The app shows it: **Operate → Health → Raspberry Pi** ("62 °C · CPU 48 % · OK"), and warns the robot
team when the Pi is under-voltage or too hot.

## 5. Measure it

```bash
python3 ~/slam-robot/tools/cpu_report.py --secs 20     # while the robot is running / driving
```

prints the CPU share of every robot program, the whole-CPU use, temperature, CPU clock and the
throttle flags, with a one-line verdict. Run it before and after a change and compare.

Targets on a healthy run (localization mode, driving, camera on):

| Number | Target |
|---|---|
| all robot programs together | well under 300 % of 1 core (of 400) |
| temperature | < 70 °C |
| `throttled` | `0x0` |
| `/odom` | 50 Hz steady (`ros2 topic hz /odom`) |
| LiDAR | 7–10 scans/s (app Health) |
