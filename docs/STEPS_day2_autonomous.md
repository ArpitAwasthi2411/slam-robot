# Day 2: from power-on to the robot driving itself

Where we are: remote works, both encoders count, directions are known, ticks per revolution are
measured (left 5036, right 5140). Today: put the numbers in, check odometry, make a map, then
send the robot to places on its own.

Terminals you'll use:
- **LAPTOP**: prompt `arpit@arpit-VivoBook...`
- **PI-1, PI-2**: two SSH windows into the Pi, prompt `arpit@robot`

Rule: `serial_probe.py` and `ros2 launch` both need the ESP32 port. **Never run them at the same
time.** Stop the launch (Ctrl+C) before using the probe.

---

## Part 0: Power on and connect (5 min)
1. Robot battery ON, Pi boots (wait ~1 min). ESP32 and LiDAR plugged into **their usual Pi USB sockets**.
2. Ethernet cable laptop ↔ Pi.
3. LAPTOP:
   ```bash
   sudo ip link set enx006f0000324d up
   sudo ip addr add 169.254.1.1/16 dev enx006f0000324d     # "File exists" = already set, fine
   ping -c 2 169.254.1.2
   ssh arpit@169.254.1.2                                    # this window = PI-1
   ```
   Open a second laptop terminal and `ssh arpit@169.254.1.2` again: that one is PI-2.
4. PI-1:
   ```bash
   ls -l /dev/esp32 /dev/rplidar
   ```
   **Check:** both exist. If not: `sudo bash ~/slam-robot/scripts/setup_ports.sh`.

## Part 1: Firmware with yesterday's numbers (skip if already flashed today)
LAPTOP, Arduino IDE: open `robot_esp32_classic.ino` (the latest one I sent). These lines must read:
```cpp
#define LEFT_MOTOR_DIR    -1
#define RIGHT_MOTOR_DIR    1
#define LEFT_ENC_DIR      -1
#define RIGHT_ENC_DIR     -1
#define MAX_WHEEL_SPEED_MMS   493.0f
#define PWM_MIN               15.0f
```
Flash (Board: ESP32 Dev Module), then plug the ESP32 back into the **same** Pi socket.

## Part 2: Put the ticks into the Pi config (2 min)
PI-1:
```bash
cd ~/ros2_ws/src/lidar_robot/config
sed -i 's/ticks_per_rev_left: .*/ticks_per_rev_left: 5036.0/; s/ticks_per_rev_right: .*/ticks_per_rev_right: 5140.4/' robot_params.yaml
grep -n "ticks_per_rev\|wheel_separation\|wheel_radius" robot_params.yaml
cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot
source install/setup.bash
```
**Check:** grep shows 5036.0 and 5140.4, wheel_radius 0.0625, wheel_separation 0.30.

## Part 3: Odometry checks with the probe (5 min)
PI-1, transmitter OFF:
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --vel 200 200     # robot LIFTED
```
**Check:** both wheels turn forward, both read 180–220 mm/s.
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --push            # robot on FLOOR
```
Roll it exactly 1 m along a tape line, press Enter. **Check:** both ticks positive, ~12 800 / ~13 100.
If one is negative → flip that wheel's `*_ENC_DIR` and re-flash.

## Part 4: Start ROS and look at it (10 min)
PI-1:
```bash
ros2 launch lidar_robot bringup.launch.py
```
Leave it running. PI-2:
```bash
ros2 topic hz /odom        # ~50, Ctrl+C
ros2 topic hz /scan        # ~7-10, Ctrl+C
ros2 topic echo /robot/status --once
```
**Check:** rates OK, status shows `"connected": true`.

LAPTOP (new terminal, not SSH):
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2 -d ~/Downloads/slam-robot/slam-robot/rviz/robot.rviz
```
Browser: `http://169.254.1.2:8080` (dashboard).

## Part 5: Two odometry tests (10 min)
**5a. LiDAR direction.** RViz → Global Options → Fixed Frame = `base_link`. Stand 1 m in front of
the robot. Your legs must be on the **red (X) axis**. If they're behind: Ctrl+C in PI-1, relaunch
with `ros2 launch lidar_robot bringup.launch.py laser_yaw:=3.14159` (and use that every time).
Set Fixed Frame back to `map`.

**5b. Drive 1 m.** Tape mark next to the robot, drive 1 m straight with the remote.
**Check:** the robot arrow in RViz moves ~1 m forward, the walls stay where they are.
This is yesterday's "robot doesn't move on the map" problem. If it passes, odometry is fixed.

## Part 6: Wheel separation (10 min, makes turns accurate)
1. PI-2: `ros2 topic echo /robot/status --once` → note the number after `"turns"` = **T0**.
2. Put a tape mark where the robot's front points. Spin in place slowly (remote, steering only)
   **exactly 5 full turns**, stop facing the tape.
3. Same command again → **T1**. Counted turns = |T1 − T0|.
4. New value = 0.30 × counted / 5. Example: counted 5.20 → 0.312.
5. Ctrl+C the launch (PI-1), then:
   ```bash
   sed -i 's/wheel_separation: .*/wheel_separation: 0.312/' ~/ros2_ws/src/lidar_robot/config/robot_params.yaml
   ```
   (put your number). Relaunch: `ros2 launch lidar_robot bringup.launch.py`.

## Part 7: Build the map (15 min)
1. Put the robot at a spot you can find again (tape an X on the floor, robot facing one way). This is
   the map's origin; you'll start here every time.
2. Drive **slowly** (half stick) along the walls of the room / corridor, through doorways, and
   **come back to the X**. Slow turns. Watch RViz: walls should stay sharp, not doubled.
3. Dashboard → **Status → Save map** → name `room1` (or `cse_floor`).
4. PI-2: `ls ~/maps/`. **Check:** `room1.pgm`, `room1.yaml`, **`room1.pbstream`**.
   If the dashboard says "no .pbstream saved", tell me before going on.
5. Backup to the laptop (LAPTOP terminal):
   ```bash
   scp arpit@169.254.1.2:~/maps/room1.* ~/Downloads/slam-robot/slam-robot/maps/
   ```

## Part 8: Restart on the saved map (localization)
1. Ctrl+C the launch. Put the robot back on the X, same direction.
2. PI-1:
   ```bash
   ros2 launch lidar_robot bringup.launch.py slam_mode:=localization map:=$HOME/maps/room1.pbstream
   ```
3. Drive 1–2 m with the remote. In RViz / dashboard the red scan dots must sit **on** the black walls.
4. Dashboard → **Position OK**. The robot won't drive itself until you press this.

## Part 9: First autonomous drive
Safety: start with **open space**, you next to the robot, thumb on the remote (any stick = instant
manual takeover), and the motor battery switch in reach.

**9a. RViz goal.** RViz toolbar → **2D Goal Pose** → click a spot ~1.5 m away in free space, drag for the
direction. The robot plans a route (line in RViz), drives, turns to the heading.
Dashboard shows the state; **Check:** it ends with `SUCCEEDED`.

**9b. Dashboard goal.** Dashboard → **Go here** → click the map → it shows the route + ETA → **Go**.

**9c. Obstacle.** Send a goal, step into its path. It slows, stops ~38 cm away, waits, re-plans
around you or continues when you step away.

## Part 10: Places and commands (the major-project part)
1. Drive (remote) to a spot, e.g. the door → Dashboard → **Go → Places → Save here** → name `door`.
   Do 2–3 places, and **Set home** at the X.
2. Search `door` → **Route** (preview only) → **Go**.
3. Type a command: `go to door then come back`. It queues a 2-stop mission (OFFLINE parser works
   without internet).
4. Every later session: Part 0 → Part 8 → use places. No re-mapping needed.

---

## If something goes wrong
| Symptom | Do |
|---|---|
| Robot arrow doesn't move in RViz while driving | `ros2 topic echo /odom --once` changes? If not: Ctrl+C launch, run Part 3 again |
| Robot moves on the map the wrong way / spins when going straight | an `*_ENC_DIR` is wrong → Part 3 |
| Map walls doubled after a turn | turn slower; redo Part 6 |
| Navigator says "not localized" | press **Position OK** (Part 8.4) |
| Goal fails `NO_PATH` / `GOAL_IN_OBSTACLE` | goal too close to a wall (robot keeps 27 cm away); pick a more open spot |
| Ctrl+C doesn't stop the launch | PI-2: `pkill -f ros2` then `pkill -f cartographer` then `pkill -f sllidar`, one per line |
| Probe says port busy | the launch is still running → stop it first |

Send me a screenshot at Part 5b, Part 7.4 and Part 9a.
