# Team reading: understand Pathik before tomorrow

Everyone reads their part tonight and **explains it tomorrow without notes**, in about 10 minutes, then answers
questions from the others. In the viva each of us has to defend our part, so read until you can answer every
question in your list, not just until you've opened the files.

All files are in the repo: https://github.com/ArpitAwasthi2411/slam-robot (branch `main`).
Python modules live in `ros2_ws/src/lidar_robot/lidar_robot/`. Every Python file starts with a short
description, and the tests in `ros2_ws/src/lidar_robot/test/` show what each module is supposed to do. Read the
test file next to each module; it's the fastest way to understand it.

Everyone first reads (20 min): **`README.md`** and **`CLAUDE.md`** (what the robot is, how the parts connect).

Some files (app, voice, docking) are being reworked this week; read today's version, the ideas stay the same.

---

## Yogit: hardware, firmware and the link to the Pi
How the robot physically works: power, motors, sensors, the ESP32, and how it talks to the Pi.

**Read**
- `hardware/CONNECTIONS_esp32s3.md`: every wire, both ends
- `hardware/wiring_esp32s3.md`: level shifters, why some pins need them
- `hardware/battery_dock.md`, `hardware/sensors.md`
- `firmware/robot_esp32s3/robot_esp32s3.ino`: the ESP32 program (read the comments at the top, `setup()`, `loop()`,
  the speed loop, e-stop, RC, ultrasonic and the command handler)
- `ros2_ws/src/lidar_robot/lidar_robot/protocol.py` + `test/test_protocol_kinematics.py`: the text messages between ESP32 and Pi
- `ros2_ws/src/lidar_robot/lidar_robot/esp32_bridge.py`: the Pi side of the link
- `ros2_ws/src/lidar_robot/lidar_robot/kinematics.py`, `battery.py`

**Be ready to explain**
1. Draw the power path from the battery to the motors, the Pi and the ESP32. Where are the fuse, switch, buck and capacitors, and why?
2. Why does the e-stop cut the motor drivers' EN pins instead of the battery? What does the ESP32 do when it's pressed?
3. How does an encoder tell speed and direction? What is "ticks per revolution", and how do ticks become metres?
4. What does the PI speed loop do? What happens with only P, and why add I?
5. What are `ODM`, `US`, `BAT` and `V` lines? Give one example of each and what the numbers mean.
6. Why did the ESP32 keep disconnecting while driving, and how did moving to the GPIO UART fix it?
7. Who wins if the RC remote, the app and the navigator all send commands at once? Why that order?
8. Why do the ultrasonic ECHO pins need a level shifter but the TRIG pins don't?

## Neelaksh: mapping, localization and navigation
How the robot knows where it is and how it gets somewhere without hitting things.

**Read**
- `docs/navigation.md`, `docs/exploration.md`
- `ros2_ws/src/lidar_robot/config/cartographer_odom.lua` and `cartographer_localization.lua` (just the comments and main settings)
- `ros2_ws/src/lidar_robot/launch/bringup.launch.py`: which nodes start, and the TF frames
- `planner.py` + `test/test_planner_follower.py`: A* on the map
- `footprint.py`: the robot's real rectangle shape
- `follower.py`: pure pursuit path following
- `navigator_core.py` + `test/test_navigator_core.py`: the brain (read the top, `goto_place`, the mission loop, failure handling, docking)
- `explorer.py`: mapping by itself
- `vision_rules.py` + `test/test_vision.py`: slowing and stopping for people

**Be ready to explain**
1. What does SLAM do? What's the difference between mapping mode and localization mode, and when do we use each?
2. Explain the TF tree `map → odom → base_link → laser`. Who publishes each link?
3. How does A* find a path, and why do we inflate the obstacles first?
4. Why is our footprint a rectangle with the axle near the front, and how does that change turning?
5. How does pure pursuit follow a path? What is the lookahead distance and what happens if it's too small or too big?
6. What happens when the path is blocked? List the failure reason codes the navigator uses.
7. How does frontier exploration decide where to go next?
8. How do the camera and the LiDAR work together to decide to slow down or stop for a person?

## Rashi: language, missions, app and deployment
How a person tells the robot what to do, and how that becomes a mission.

**Read**
- `docs/mobile_app.md`, `docs/deployment.md`, `docs/mcp.md`
- `commands.py` + `test/test_places_commands_missions.py`: English/Hinglish command → structured intent (LLM + offline rules)
- `places.py`: places, rooms with contents, aliases, fuzzy search
- `missions.py`, `schedules.py`, `missionlog.py`: the mission queue, priorities and logs
- `explain.py`: "why did you stop?"
- `auth.py` + `test/test_auth.py`: logins and roles
- `dashboard_server.py`: the HTTP API the app calls
- `scripts/mcp_server.py`: how an AI assistant controls the robot
- `speech.py`: the robot's voice today
- `tools/eval_commands.py` + `tools/commands_test.json`: how we measure command accuracy

**Be ready to explain**
1. Follow one command end to end: "lab 2 mein jao, phir HOD office" from the app until the robot moves. Which files does it pass through?
2. What does the LLM return, and why do we validate it before running it? What happens with no internet?
3. How does "the room with the 3D printer" become a place on the map?
4. How does the mission queue decide what runs first? What does an urgent mission do to a running one?
5. What can staff, operators and admins each do? Why is the first account the admin? How are passwords stored?
6. What is MCP, and what can Claude do with our robot through it?
7. How do we measure the language part (accuracy, latency), and what were the results?
8. Where should voice plug into this chain, given your voice work?

## Arpit: integration
Ask each person 2–3 of their questions tomorrow; write down what nobody could answer and we go through it together.
