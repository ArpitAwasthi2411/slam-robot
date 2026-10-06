# Roadmap: improvements, in the order that gives most value

## Next (makes the current system solid)
1. **Finish odometry on hardware.** Flash v2, run the direction tests, `--calibrate`, `--push`,
   then the spin test for wheel separation. Without this, everything else is tuning on sand.
2. **Measure the LiDAR pose** (`laser_x/y/z/yaw`) and the robot footprint.
3. **Pi on Wi-Fi** (or a travel router on the robot). The ethernet cable limits mapping range.
   ROS 2 DDS discovery works on the same Wi-Fi with the same `ROS_DOMAIN_ID`.
4. **Save maps every session** with the dashboard; commit the good ones to `maps/`.

## Soon (navigation)
5. **URDF + robot_state_publisher** instead of the static TF: one file with footprint, wheel
   and LiDAR positions, and a proper robot model in RViz.
6. **Nav2**: replace the straight-line goal controller with real path planning around obstacles.
   Everything it needs already exists: `/map`, `/scan`, `/odom`, TF, and `/cmd_vel` into the bridge.
   Use Cartographer in pure-localisation mode on a saved `.pbstream`, or `nav2_amcl` with the saved `.pgm`.
7. **IMU** (MPU6050/BNO055 on the ESP32 I2C): gyro yaw makes rotations far more reliable than
   wheels. Send `IMU,...` lines and set `use_imu_data = true`.
8. **Battery voltage** on an ESP32 ADC pin (divider) → shown on the dashboard, auto-stop when low.

## Later (project polish / report)
9. **RC mode switch on CH5/CH6**: a physical MANUAL / AUTO / E-STOP switch on the transmitter
   (one more GPIO + a few lines in `loop()`).
10. **ros2_control + diff_drive_controller** with a hardware interface for the ESP32: the
    "industry-standard" way, and good for the report.
11. **micro-ROS** on the ESP32 instead of the text protocol, if you want the ESP32 to be a
    first-class ROS node (more complex; the text protocol is easier to debug).
12. **Gazebo/Ignition simulation** from the URDF: test Nav2 without the robot.
13. **Semantic layer for the major project**: tag named places on the saved map
    ("lab door", "HOD office") → the LLM picks a place → `/goal_pose`.

## Known limitations of v2
- The goal controller drives straight lines: walls between robot and goal will block it (safely). That's what Nav2 fixes.
- The obstacle stop only looks forward (±25°). Reversing is never commanded automatically.
- The dashboard has no login. Anyone on the robot's network can drive it. Fine on a direct cable; add auth before using shared Wi-Fi.
