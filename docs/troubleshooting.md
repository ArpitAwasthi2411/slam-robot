# Troubleshooting

Problems already hit on this robot, and what fixed them.

| Symptom | Cause | Fix |
|---|---|---|
| `executable 'esp32_odom_node' not found on the libexec directory` | v1 package was `ament_cmake` installing `.py` files without `setup.cfg` | v2 is `ament_python` with `setup.cfg`. Delete `build/lidar_robot install/lidar_robot` and rebuild |
| `/dev/ttyACM0` opens but **no ODM data**, Cartographer: `Queue waiting for data: (0, odom)` | Most likely the sketch printed to `Serial` while "USB CDC On Boot" was Disabled, so output went to UART pins, not USB. Also possible: DTR/RTS toggling resetting the S3, or ModemManager grabbing the port | Firmware v2 routes to the native USB port in every setting. The bridge opens without toggling DTR/RTS. `install_udev.sh` disables ModemManager. Diagnose with `tools/serial_probe.py` |
| `"odom" passed to lookupTransform argument source_frame does not exist` | Lua said `published_frame="odom"` but nothing published `odom→base_link` | Use the two configs as shipped: odom mode → bridge publishes it; lidar-only → `provide_odom_frame=true`, `published_frame="base_link"` |
| Edited `~/cartographer_config/robot_2d.lua` but nothing changed | Edits not saved, and the launch read a different copy | Configs now live only in the package `config/` folder. With `--symlink-install` edits apply on the next launch |
| LiDAR-only: map builds but **robot pose doesn't move** | No odometry + default Ceres matcher = "zero motion" prediction | `cartographer_lidar_only.lua` enables the online correlative scan matcher |
| `Specified options.num_threads: 7 exceeds maximum ... 4` | Cartographer default | Set to 4 in both Lua files |
| Laptop `ssh: network unreachable` after replugging cable | Manual `ip addr add` is lost on unplug | `scripts/setup_laptop_link.sh` (NetworkManager profile, permanent) |
| `Could not resolve hostname robot.local` | mDNS over a direct cable | Use `169.254.1.2` |
| Ctrl+C doesn't kill nodes started with `&` | Background jobs | Use the launch file. If needed: `pkill -f cartographer; pkill -f sllidar; pkill -f lidar_robot` |
| `Package 'sllidar_ros2' not found` in a new terminal | Workspace not sourced | `source ~/ros2_ws/install/setup.bash` (add to `~/.bashrc`) |
| RViz "not responding" on the laptop | Big map + 30 fps on an i3 | `robot.rviz` uses 15 fps; close the Views panel; or use the dashboard |
| slam_toolbox `queue is full` | (historic) chose Cartographer instead | — |

## E-stop behaviour

- Dashboard button / `Space`, or from any terminal:
  `ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"` (`false` releases).
- The e-stop is latched **inside the ESP32** (mode chip shows `ESP32 E-STOP`): it blocks the Pi
  **and the RC** until released. Pressing the ESP32 RESET button also clears it.
- If the Pi or the launch restarts while the ESP32 is latched, the bridge detects it and the dashboard
  shows the e-stop as engaged. Release it explicitly. Restarting never releases it silently.
- Robot keeps going when the transmitter is switched off → receiver failsafe not set (deploy.md 1b).

## Quick health check

```bash
ros2 node list                       # esp32_bridge, sllidar_node, cartographer_node, ... present?
ros2 topic hz /odom /scan            # ~50 Hz / ~7-10 Hz
ros2 topic echo /robot/status --once # connected, rx_hz, esp_mode, cmd_source
ros2 run tf2_tools view_frames       # writes frames.pdf: must be map->odom->base_link->laser
```

## Robot drives the wrong way / spins when told to go straight

1. `serial_probe.py --pwm 120 120` on blocks: both wheels forward, both speeds positive.
2. If the robot curves on the floor but wheels are fine on blocks: calibrate ticks_per_rev L/R.
3. Goal controller turns the wrong way: check the LiDAR yaw (deploy.md step 7). A LiDAR mounted
   backwards makes the whole map rotated 180° relative to the robot.
