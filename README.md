# SLAM Robot — ROS 2 differential-drive robot with LiDAR SLAM

An indoor mobile robot that builds a map of a room with **Cartographer SLAM**, can be driven
from an **RC transmitter, a web dashboard or RViz goals**, and fuses **wheel-encoder odometry**
from an ESP32-S3.

| | |
|---|---|
| Compute | Raspberry Pi 4, Ubuntu 22.04, ROS 2 Humble (headless) |
| LiDAR | Slamtec RPLiDAR A1M8, 360°, 10 Hz, 12 m |
| Low-level | ESP32-S3: motor PWM, quadrature encoders, PI wheel-speed loop, RC input |
| Drive | 2 × DC gear motors with encoders (4740 ticks/rev), BTS7960-style H-bridges, 125 mm wheels, 300 mm track |
| Manual | FlySky FS-i6 + FS-iA6 receiver (always overrides software) |
| SLAM | Cartographer 2D (LiDAR + odometry, or LiDAR-only fallback) |

![Web dashboard: live map, LiDAR scan, robot trail and a goal being driven to](docs/images/dashboard.png)
<sub>Web dashboard (shown here with `tools/dashboard_sim.py`): live map, LiDAR returns, trail, goal + path, joystick, e-stop.</sub>

## Architecture

```mermaid
flowchart LR
  subgraph ESP32-S3
    RC[FlySky RC] --> ARB{RC > Pi > idle}
    ENC[Encoders] --> PI[PI speed loop]
    ARB --> PI --> MOT[Motor PWM]
  end
  subgraph "Raspberry Pi 4 (ROS 2 Humble)"
    BR[esp32_bridge] -->|/odom + TF odom→base_link| CART[Cartographer]
    LID[sllidar_node] -->|/scan| CART
    CART -->|/map + TF map→odom| GC[goal_controller]
    CART --> DB[dashboard :8080]
    GC -->|/cmd_vel| BR
    DB -->|/cmd_vel_teleop, /estop, /goal_pose| BR
  end
  ESP32-S3 <-->|USB serial 115200: ODM / V,P,S,E,R| BR
  LAP[Laptop: RViz2] <-->|DDS over ethernet| CART
  PHONE[Browser / phone] <-->|HTTP| DB
```

**TF tree:** `map → odom → base_link → laser`
**Command priority:** E-stop (latched in the ESP32) › RC sticks › dashboard teleop › goal controller / Nav2 › stop.
Every layer has a watchdog: browser → 0.5 s, bridge → 0.5 s, ESP32 → 0.3 s, RC signal loss → 0.1 s,
stale SLAM pose → goal controller stops in 0.6 s.

## Repository layout

```
firmware/robot_esp32/        ESP32-S3 Arduino sketch (core 2.x and 3.x)
ros2_ws/src/lidar_robot/     ROS 2 package (ament_python)
  lidar_robot/               esp32_bridge, goal_controller, dashboard (+ pure-Python logic modules)
  lidar_robot/web/           dashboard page (single file, works offline)
  config/                    robot_params.yaml, cartographer_odom.lua, cartographer_lidar_only.lua
  launch/bringup.launch.py   one launch file for everything
  test/                      unit tests, controller simulation, bridge-over-pty integration test
tools/                       serial_probe.py (debug/calibrate ESP32), dashboard_sim.py (UI without robot)
rviz/robot.rviz              RViz2 layout for the laptop
udev/ scripts/               stable /dev names, laptop ethernet setup, systemd autostart
hardware/                    pin map, wiring, BOM, CAD, photos
docs/                        deploy, calibration, troubleshooting, roadmap
maps/                        saved maps (.pgm + .yaml)
```

## Quick start

On the Pi (full guide: [docs/deploy.md](docs/deploy.md)):

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot
source install/setup.bash
ros2 launch lidar_robot bringup.launch.py                      # encoders + LiDAR SLAM
ros2 launch lidar_robot bringup.launch.py use_odometry:=false  # LiDAR-only fallback
```

On the laptop:

```bash
rviz2 -d rviz/robot.rviz          # use the "2D Goal Pose" tool to send the robot somewhere
xdg-open http://169.254.1.2:8080  # web dashboard: joystick, e-stop, map, click-to-goal, save map
```

No robot? `python3 tools/dashboard_sim.py` runs the dashboard against a simulated robot.

## Status

- [x] LiDAR → Cartographer → live map in RViz over ethernet
- [x] ESP32 firmware v2.1: velocity commands, PI wheel loop, RC override + failsafe, e-stop latch
- [x] Bridge with reconnect, teleop/nav arbitration, e-stop
- [x] Go-to-goal controller (RViz 2D Goal Pose), obstacle stop
- [x] Web dashboard
- [ ] Encoder odometry verified on hardware (calibration: [docs/calibration.md](docs/calibration.md))
- [ ] Nav2 path planning + AMCL localisation on saved maps — see [docs/roadmap.md](docs/roadmap.md)

## Tests

```bash
cd ros2_ws/src/lidar_robot && python3 -m pytest test/     # no ROS needed
```

## License

MIT — see [LICENSE](LICENSE).
