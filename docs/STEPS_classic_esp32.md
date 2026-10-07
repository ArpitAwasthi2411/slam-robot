# From remote-working to a saved map (classic ESP32): every step

Sensors (ultrasonic, MPU, e-stop) are not wired yet. That's fine, the firmware ignores them.
Do the steps in order and don't skip a **Check**.

**Files you need (all in `slam-robot.zip`):**

| File | Used in | Where it runs |
|---|---|---|
| `firmware/robot_esp32_classic/robot_esp32_classic.ino` | A | Arduino IDE (laptop) → ESP32 |
| `firmware/rc_reader/rc_reader.ino` | only to re-measure the remote | Arduino IDE |
| `ros2_ws/src/lidar_robot/` (whole folder) | C, G | Pi |
| `scripts/setup_ports.sh` | D | Pi |
| `tools/serial_probe.py` | E, F, G | Pi (or laptop) |
| `ros2_ws/src/lidar_robot/config/robot_params.yaml` | G (calibration numbers) | Pi |
| `rviz/robot.rviz` | I | laptop |

---

## A. Laptop: unzip the new version
```bash
cd ~/Downloads
rm -rf slam-robot && mkdir slam-robot && cd slam-robot
unzip ~/Downloads/slam-robot.zip        # creates ~/Downloads/slam-robot/slam-robot
cd slam-robot
```

## B. Arduino IDE: flash the ESP32 (skip if already flashed with your RC values)
1. Open `firmware/robot_esp32_classic/robot_esp32_classic.ino`.
2. Board **ESP32 Dev Module**, choose the port, **Upload**.
   Upload fails with "flash chip" errors → unplug the wires, upload, reconnect.
3. **Check:** Serial Monitor 115200 shows `INFO,READY,robot_esp32 v2.2 ESP32` and `ODM,...` lines.
   RC sticks move the wheels (robot on blocks).
4. Close the Serial Monitor. Unplug the ESP32 from the laptop.

## C. Copy the code to the Pi and build
Laptop:
```bash
cd ~/Downloads/slam-robot
scp -r slam-robot arpit@169.254.1.2:~/
ssh arpit@169.254.1.2
```
Pi (run each `pkill` line separately; it's fine if they print nothing):
```bash
pkill -f ros2
pkill -f cartographer
pkill -f sllidar
```
```bash
rm -rf ~/ros2_ws/src/lidar_robot ~/ros2_ws/build/lidar_robot ~/ros2_ws/install/lidar_robot
cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot
source install/setup.bash
grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```
**Check:** `ros2 pkg executables lidar_robot` lists `dashboard`, `esp32_bridge`, `navigator`, `goal_controller`.

## D. Pi: give the LiDAR and ESP32 fixed names (once)
```bash
sudo ~/slam-robot/scripts/setup_ports.sh
```
It asks you to have only the LiDAR plugged in, then to plug in the ESP32. It works out which is
which and writes the udev rule.
**Check:** it ends with `/dev/rplidar -> ttyUSBx` and `/dev/esp32 -> ttyUSBy`.
If it says "same USB chip on both", label the two Pi USB sockets and always use the same ones.
Log out and back in once (`exit`, `ssh` again) so serial access works without sudo.

## E. Pi: is the ESP32 talking?
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32
```
**Check:** about 50 Hz, `modes={'0': ...}`, `INFO,READY,robot_esp32 v2.2 ESP32`.
Run it again while moving the RC sticks → `'1'` appears in modes and the ticks are not 0.
(The `rst:ets...` burst at the start is the board resetting when the port opens: harmless.)

## F. Wheel directions: ROBOT ON BLOCKS, wheels in the air
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --pwm 120 120
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --pwm 120 0
```
**Check:**
- `120 120`: both wheels spin **forward**, both speeds printed **positive**.
- `120 0`: only the **left** wheel moves.

Fixes (in the sketch, then re-flash from the laptop and repeat F):
| You see | Change |
|---|---|
| a wheel spins backward | that wheel's `LEFT_MOTOR_DIR` / `RIGHT_MOTOR_DIR` → `-1` |
| wheel goes forward but its speed is negative | that wheel's `LEFT_ENC_DIR` / `RIGHT_ENC_DIR` → `-1` |
| `120 0` moves the right wheel | swap the motor wires L↔R, or swap `LEFT_RPWM/LPWM` with `RIGHT_RPWM/LPWM` |

This is the step that matters for the map. If the wheels and encoders are right here, odometry will be right.

## G. Calibrate
On blocks:
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --calibrate
```
It prints `PWM_MIN` and `MAX_WHEEL_SPEED_MMS` → put them in the sketch → re-flash.
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --vel 200 200
```
**Check:** both wheels report 180–220 mm/s.

On the floor (tape a 1.000 m line):
```bash
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --push
```
Roll the robot exactly 1 m by hand along the tape, press Enter. Do it 3 times and average.
Put the numbers in the Pi's config:
```bash
nano ~/ros2_ws/src/lidar_robot/config/robot_params.yaml
#   ticks_per_rev_left:  <your number>
#   ticks_per_rev_right: <your number>
```
(`--symlink-install` means no rebuild is needed for yaml edits; just relaunch.)

## H. Pi: start everything
Terminal 1 on the Pi:
```bash
ros2 launch lidar_robot bringup.launch.py
```
Terminal 2 (new `ssh arpit@169.254.1.2`):
```bash
ros2 topic hz /odom        # ~50
ros2 topic hz /scan        # ~7-10
ros2 topic echo /robot/status --once
```
**Check:** both rates OK, status shows `"connected": true`.
Stop with **Ctrl+C once** and wait. If it hangs, open another terminal and run each `pkill` from C.

## I. Laptop: RViz and dashboard
```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/Downloads/slam-robot/slam-robot/rviz/robot.rviz
```
Browser: `http://169.254.1.2:8080`

## J. LiDAR orientation (once)
In RViz set **Fixed Frame = base_link**. Stand about 1 m **in front** of the robot.
**Check:** your legs appear on the red (+X) axis. If they appear behind:
```bash
ros2 launch lidar_robot bringup.launch.py laser_yaw:=3.14159
```
and from then on always add `laser_yaw:=3.14159` (or tell me and I'll make it the default).
Set Fixed Frame back to **map**.

## K. Odometry test: yesterday's problem
1. Put a tape mark next to the robot. Drive **1 m forward** with RC or the dashboard joystick.
   **Check:** the robot arrow in RViz moves about 1 m, and the walls stay still.
2. Spin slowly 360° in place.
   **Check:** the map doesn't smear or rotate.

If the arrow doesn't move, or moves the wrong way: go back to F. Meanwhile you can still map with
`ros2 launch lidar_robot bringup.launch.py use_odometry:=false`.

## L. Build and save the map
1. Drive **slowly** (half stick) around the room. Go along the walls, then come back to where you started (closing the loop fixes drift).
2. Dashboard → **Save map** → give it a name (for example `room1`).
   **Check:** on the Pi, `ls ~/maps/` shows `room1.pgm`, `room1.yaml` (and `room1.pbstream`).
3. Copy it to the laptop (laptop terminal):
   ```bash
   scp arpit@169.254.1.2:~/maps/room1.* ~/Downloads/slam-robot/slam-robot/maps/
   ```

## After this
- Navigate on the saved map: `ros2 launch lidar_robot bringup.launch.py slam_mode:=localization map:=~/maps/room1.pbstream`,
  then follow docs/navigation.md (Position OK → save places → Go).
- Sensors: hardware/wiring_classic_esp32.md, one at a time.
