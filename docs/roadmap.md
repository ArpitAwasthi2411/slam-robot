# Roadmap: improvements, in the order that gives most value

## Next (makes the current system solid)
1. **Finish odometry on hardware.** Flash v2, run the direction tests, `--calibrate`, `--push`,
   then the spin test for wheel separation. Without this, everything else is tuning on sand.
2. **Measure the LiDAR pose** (`laser_x/y/z/yaw`) and the robot footprint
   (`footprint_front/rear/half_width` in `robot_params.yaml`: v2.4 uses the real rectangle).
3. **Pi on Wi-Fi** (or a travel router on the robot). The ethernet cable limits mapping range.
   ROS 2 DDS discovery works on the same Wi-Fi with the same `ROS_DOMAIN_ID`.
4. **Save maps every session** with the dashboard; commit the good ones to `maps/`.

## Soon (navigation)
5. **URDF + robot_state_publisher** instead of the static TF: one file with footprint, wheel
   and LiDAR positions, and a proper robot model in RViz.
6. ~~Path planning~~ **done in v0.3** without Nav2: the `navigator` node (A*, path following,
   replanning, places, missions, localization mode); see [navigation.md](navigation.md).
   Nav2 is still an option later (it would replace the navigator; topics are compatible: `/map`,
   `/scan`, `/odom`, TF, `/cmd_vel`, `/goal_pose`), mainly for its DWB local planner and costmap layers.
7. ~~IMU~~ **code done (firmware v2.2)**: the MPU-6050 gyro gives odometry heading; wire it per
   [hardware/sensors.md](../hardware/sensors.md). Ultrasonics and the mushroom e-stop are done the same way.
8. ~~Battery voltage~~ **code done (v2.4)**: INA226 voltage + current, app battery chip, goes to charge
   when low, reverses onto the dock and confirms by charging current ([hardware/battery_dock.md](../hardware/battery_dock.md)).

## Later (project polish / report)
9. **RC mode switch on CH5/CH6** (docs/WORK_LEFT.md A9): a physical MANUAL / AUTO / E-STOP switch on the transmitter
   (one more GPIO + a few lines in `loop()`).
10. **ros2_control + diff_drive_controller** with a hardware interface for the ESP32: the
    "industry-standard" way, and good for the report.
11. **micro-ROS** on the ESP32 instead of the text protocol, if you want the ESP32 to be a
    first-class ROS node (more complex; the text protocol is easier to debug).
12. **Gazebo/Ignition simulation** from the URDF: test Nav2 without the robot.
13. ~~Semantic layer~~ **done in v0.3**: named places + LLM/offline command parser → missions.
14. **LLM error recovery (your novelty #1)**: feed the failure code + context to the LLM, let it pick
    from a fixed menu of safe actions (retry later / skip / reorder / notify / go home), validate, execute.
    Hooks are ready: `last_failure` in `navigator_core.py`, reason codes in navigation.md.
15. ~~Scheduling~~ **done (v2.4)**: timed missions, "wait for OK" stops, rooms with contents,
    "why did you stop?", mission log + `tools/eval_nav.py`, MCP server for AI assistants. Still open:
    **Campus app (your novelty #2)**: a FastAPI server on the Pi that calls the same
    `/navigator/request` API; users summon the robot, missions carry requester + priority, the
    queue already handles priority pre-emption and return-home-when-idle (`auto_return_s`).
16. ~~Ultrasonic sensors~~ **code done (v2.2)**: front clearance + replanning + an ESP32-level 15 cm safety net.

## Known limitations of v2
- The goal controller drives straight lines: walls between robot and goal will block it (safely). That's what Nav2 fixes.
- The obstacle stop looks forward (±25°); the v2.4 body checks use the full 360° scan near the robot.
  Reversing is commanded only for short 3-point turns and docking, at ≤ 6 cm/s, with the scan checked first.
  Things below the LiDAR plane behind the robot are not seen (no rear ultrasonic).
- ~~No login~~ **done**: every API call needs a login with a role (staff / operator / admin, `auth.py`);
  the first account is the admin, later sign-ups wait for approval. The classic page (`/classic`) uses the same login.
- The link to the Pi is plain HTTP inside the robot's network or Tailscale (encrypted by WireGuard).
  Only Tailscale Funnel (public HTTPS) exposes it to the internet: see docs/STEPS_remote_control.md.
