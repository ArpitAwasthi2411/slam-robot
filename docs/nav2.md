# Nav2 on Pathik (the standard ROS 2 navigation stack)

`nav_mode:=nav2` makes **Nav2** plan the route and drive the robot. Everything else stays Pathik: missions, the
queue, waits, docking, error recovery, "why did you stop?", the app, voice. `nav_mode:=planner` (the default)
is the built-in A* + pure pursuit, kept as the fallback and for comparison.

Code: `lidar_robot/nav2_follower.py` (tested without ROS), `navigator.py` (`_setup_nav2`),
`config/nav2_params.yaml`, `launch/bringup.launch.py`. Tests: `test/test_nav2_follower.py`.

**Status: written and unit-tested, not yet run on the robot or in a ROS simulation.** Expect a tuning session.

## How the two fit together

```
app / voice ─► Pathik navigator (missions, queue, recovery)
                    │ goal (x, y, heading)
                    ▼
        NavigateToPose ─► Nav2: bt_navigator ─► planner_server (NavFn A* on the costmap)
                                             └► controller_server (Regulated Pure Pursuit)
                                             └► behavior_server (spin, back up, wait)
                    ▲                                   │ /cmd_vel_nav2
                    │ result (arrived / gave up)        ▼
              Pathik safety gate: front clearance (LiDAR + ultrasonics), body outline, speed limits
                    │ /cmd_vel
                    ▼
                 ESP32
```

- Nav2 never talks to the motors directly. An obstacle only the ultrasonics see (a low box) still stops the
  robot, and Nav2 cannot exceed the speed limits set in the app.
- When Nav2 gives up, or Pathik's gate holds the robot for `blocked_after` seconds, the navigator replans and
  then runs its usual recovery ladder (`docs/recovery.md`).
- Localization and the map still come from Cartographer (`/map`, `map → odom`). No AMCL, no map_server.
- The app draws Nav2's route (topic `/plan`). Pathik's own pre-check route is on `/navigator/plan`.

`nav2_mode` (robot_params.yaml): `navigate` = Nav2 plans and drives (default). `follow` = Pathik's route
(which knows the ultrasonic-only obstacles) is given to Nav2's controller; Nav2 only drives.

## Install and switch

```bash
bash scripts/setup_nav2.sh                      # once, on the Pi
nano ~/robot.env                                # ROBOT_ARGS="... nav_mode:=nav2"
sudo systemctl restart robot
journalctl -u robot -f | grep -iE "nav2|lifecycle|navigation backend"
```

Good start-up lines: `Managed nodes are active` (Nav2) and `navigation backend: Nav2 (navigate)` (navigator).
If Nav2 is not installed the launch says so and the built-in planner runs; nothing breaks.

Back to the built-in planner: remove `nav_mode:=nav2`, restart.

## First test (10 minutes, robot on blocks first)

1. Wheels off the floor. Send it to a place. The wheels should turn, and stop when you put a hand 30 cm in
   front of the LiDAR or the centre ultrasonic. That proves the safety gate is in the path.
2. On the floor, a 2 m straight trip. Then a trip with one corner. Then through a door.
3. Watch: `ros2 topic hz /cmd_vel_nav2` (about 10 Hz while driving), `ros2 topic echo /navigator/status --once | grep nav_backend`.

## If it does not work

| What you see | Likely cause | Do |
|---|---|---|
| "Nav2 did not answer: is it running?" in the app log | Nav2 nodes not active yet (takes ~20 s on the Pi) or crashed | `ros2 lifecycle get /controller_server`; read the first error in `journalctl -u robot` |
| Lifecycle manager: "Failed to bring up" | a parameter name Nav2 does not accept | the line above it names the node and the parameter; fix `config/nav2_params.yaml` |
| "Nav2 gave up: no route or stuck" at once | goal or robot inside an inflated wall on the costmap | lower `inflation_radius` (0.45 → 0.35); check the footprint; view the costmap in RViz |
| Costmap empty / "Timed out waiting for transform" | `/map` or TF late (Pi overloaded) | `tools/cpu_report.py`; raise `transform_tolerance`; turn vision off for the test |
| Robot oscillates left-right | lookahead too short for this speed | `lookahead_dist` 0.5 → 0.7 |
| Cuts corners, back swings into door frames | NavFn plans for a point, the body is long | raise global `inflation_radius`; or try `nav2_mode: follow` |
| Stops and never moves, Nav2 still commanding | Pathik's gate is holding it (front / body) | app "Why did you stop?"; this is the gate working |
| Pi at 100 % CPU, "position lost" | Nav2 + Cartographer + YOLO is too much | vision off, or `update_frequency` 5 → 3, local costmap 3 m → 2.5 m |

## Compare the two planners (for the report)

Same floor, same 10 trips, each backend. From the mission log (app → Operate → Log, or `~/maps/missions/`):
success rate, time per trip, replans, interventions, closest wall distance. `tools/eval_nav.py` measures the
built-in planner in simulation; Nav2 needs the real robot (or a Gazebo world).

Honest expectations: Nav2 gives smoother obstacle avoidance and a standard, well-known stack (what companies
ask about). The built-in planner knows the robot's long body and the ultrasonic memory better, and uses less CPU.
The numbers decide which one becomes the default.
