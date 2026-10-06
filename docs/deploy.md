# Deploy guide (lab checklist)

Do these in order. Each step has a check that must pass before the next one.

## 0. Laptop: make the ethernet link permanent (once)

```bash
cd ~/slam-robot && ./scripts/setup_laptop_link.sh enx006f0000324d
```
After this, unplugging the cable no longer loses `169.254.1.1`. Use `ssh arpit@169.254.1.2`
(not `robot.local`, mDNS is unreliable on a direct cable).

## 1. Flash firmware v2 to the ESP32

Arduino IDE → open `firmware/robot_esp32/robot_esp32.ino`.

| Tools menu | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| USB Mode | Hardware CDC and JTAG |
| USB CDC On Boot | **Enabled** (the sketch also works if Disabled) |
| Upload port | the ESP32's **USB** socket |

**Check:** Serial Monitor at 115200 shows `ODM,0,0,20,0` lines ~50 per second. Type `?` +
Enter → `INFO,robot_esp32 v2.1 ...`. Move the RC sticks → last field becomes `1`.
Type `E` → last field becomes `3` and the RC sticks no longer move the wheels; `R` releases it.

### 1b. Receiver failsafe (safety — do this once)

By default a FlySky receiver **keeps outputting the last stick position** when the transmitter is
switched off or goes out of range, so the robot keeps driving. On the FS-i6: *Menu → System/Setup →
Failsafe* → CH1 **On**, CH2 **On**, hold both sticks **centred** and save.
**Check:** drive slowly with RC, switch the transmitter off → the wheels stop within ~0.1 s.
Also fit a physical switch on the motor battery: the only e-stop that doesn't depend on software.

## 2. Copy the code to the Pi

```bash
# laptop
cd ~ && scp -r slam-robot arpit@169.254.1.2:~/
# Pi
ssh arpit@169.254.1.2
pkill -f ros2 ; pkill -f cartographer ; pkill -f sllidar        # stop anything old
rm -rf ~/ros2_ws/src/lidar_robot ~/ros2_ws/build/lidar_robot ~/ros2_ws/install/lidar_robot
cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot
source install/setup.bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc         # once
sudo ~/slam-robot/scripts/install_udev.sh                       # once: /dev/esp32, /dev/rplidar
```
The old `~/cartographer_config/` folder is no longer used; configs now live in the package.

**Check:** `ros2 pkg executables lidar_robot` lists `dashboard`, `esp32_bridge`, `goal_controller`.

## 3. ESP32 ↔ Pi serial link

Plug the ESP32 into the Pi.
```bash
python3 ~/slam-robot/tools/serial_probe.py
```
**Check:** `~50 Hz`, `modes={'0': ...}`, and an `INFO,robot_esp32 v2.0` line. If it says
NOTHING RECEIVED, follow the checklist it prints (that is yesterday's bug).

## 4. Direction checks (robot on blocks, wheels in the air)

```bash
python3 ~/slam-robot/tools/serial_probe.py --pwm 120 120
```
- Both wheels must spin **forward** and both speeds must be **positive**.
- Wheel spins backward → flip that wheel's `*_MOTOR_DIR` in the sketch.
- Wheel spins forward but its speed is negative → flip that wheel's `*_ENC_DIR`.

Re-flash after any change, then repeat. Then `--pwm 120 0` must move only the **left** wheel.

## 5. Calibrate (details in calibration.md)

```bash
python3 ~/slam-robot/tools/serial_probe.py --calibrate     # on blocks -> PWM_MIN, MAX_WHEEL_SPEED_MMS
python3 ~/slam-robot/tools/serial_probe.py --vel 200 200   # on blocks -> both ~200 mm/s
python3 ~/slam-robot/tools/serial_probe.py --push          # on floor, roll 1 m -> ticks_per_rev L/R
```

## 6. Bring everything up

```bash
ros2 launch lidar_robot bringup.launch.py
```
**Check** (second SSH terminal):
```bash
ros2 topic hz /odom      # ~50 Hz
ros2 topic hz /scan      # ~7-10 Hz
ros2 topic echo /robot/status --once
ros2 run tf2_ros tf2_echo map base_link
```

## 7. LiDAR orientation check (once)

Laptop: `rviz2 -d rviz/robot.rviz`, set Fixed Frame = `base_link`. Stand ~1 m **in front** of
the robot. Your legs must appear on the **+X (red) axis**. If they appear behind the robot:
```bash
ros2 launch lidar_robot bringup.launch.py laser_yaw:=3.14159
```
(then change the default in `launch/bringup.launch.py` so you don't forget).
Also measure where the LiDAR sits relative to the axle midpoint → `laser_x`, `laser_y`, `laser_z`.

## 8. Drive tests

1. Dashboard `http://169.254.1.2:8080` → hold the joystick forward for ~1 m. The robot icon must move
   forward on the map by about the same distance it really moved.
2. Spin in place 360° slowly. The map must not smear/rotate.
3. RViz **2D Goal Pose**: click 1 m in front of the robot, drag for the heading. The robot rotates,
   drives, aligns, and the goal panel shows `SUCCEEDED`.
4. Put a box in its path: it stops ~35 cm from the box (`BLOCKED`) and resumes when you remove it.
5. Touch the RC sticks during a goal: the ESP32 switches to RC immediately (chip shows RC OVERRIDE).
6. Map the room in a loop, then **Save map** in the dashboard → `~/maps/<name>.yaml`.

## Optional: start on boot

```bash
sudo cp ~/slam-robot/scripts/robot.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now robot
journalctl -u robot -f
```
Stop it (`sudo systemctl stop robot`) before using `serial_probe.py`.
