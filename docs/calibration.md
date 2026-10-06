# Calibration

Good odometry is what makes the map and the goal controller work. Do these in order.

## 1. Motor feed-forward (firmware) — robot on blocks

```bash
python3 tools/serial_probe.py --calibrate
```
Copy the two suggested values into `firmware/robot_esp32/robot_esp32.ino`:
`PWM_MIN` (where the wheel starts turning) and `MAX_WHEEL_SPEED_MMS` (speed at `MAX_PWM`). Re-flash.

Verify: `--vel 200 200` → both wheels report 180–220 mm/s. If they oscillate, lower `KP`.
If they settle slowly or stay low, raise `KI` a little.

## 2. Ticks per revolution (each wheel) — robot on the floor

Your first measurement was 4651 (left) vs 4833 (right) by spinning each wheel by hand. That 4 %
difference is probably hand error, but if it's real it makes the robot think it's turning while
it drives straight, so measure it properly:

```bash
python3 tools/serial_probe.py --push
```
Tape a 1.000 m line on the floor, roll the robot along it, press Enter. Put the printed
`ticks_per_rev_left/right` into `ros2_ws/src/lidar_robot/config/robot_params.yaml`.
Repeat 3 times and average.

## 3. Wheel separation — robot on the floor

1. Launch (`use_odometry:=true`). Put a tape mark on the floor pointing where the robot faces.
2. Note the starting value: `ros2 topic echo /robot/status --once` → `"odom": {..., "turns": T0}`.
3. Dashboard: rotate slowly in place (joystick left/right only) **exactly 5 full turns**, stopping
   when the robot faces the tape the 5th time.
4. Read `turns` again → T1. Odometry measured `odom_turns = |T1 - T0|`.
5. `wheel_separation_new = wheel_separation_old × odom_turns / 5`

Example: odometry says 5.20 turns for 5 real ones → 0.30 × 5.20 / 5 = **0.312 m**.

## 4. LiDAR mounting

Measure the LiDAR centre relative to the midpoint between the two wheels:
forward = `laser_x`, left = `laser_y`, up = `laser_z`. Orientation: see deploy.md step 7.

## 5. Sanity test: the UMBmark square

Drive a 1 m square 4 times clockwise and 4 times counter-clockwise using goals, and compare where
the **odometry** (`/odom`) says you ended up with the start. Errors under ~5 cm per square are good
enough for Cartographer. Bigger errors mean step 2 or 3 needs redoing.
