# Front docking with the ArUco marker

The robot now docks **nose first**. The dock has an ArUco marker centred at camera height and two copper
contacts at the front of the robot (~8 cm and ~14 cm above the floor, final heights from Yogit).

Code:
- `lidar_robot/docking.py`: controller, geometry and a synthetic camera for tests.
- `lidar_robot/marker_pose.py`: the detector, which Neelaksh's module can replace.
- `vision_node.py`: publishes `/vision/dock_marker` at 10 Hz.
- `navigator_core.py`: `_front_docking`, plus backing off the dock for the next task.
- Firmware `D,1`, plus `esp32_bridge` `/dock_mode`.
- App docking card on Home.
- Tests in `test_docking.py`; laptop tool `tools/marker_webcam.py`.

## What happens on "Charge" (or a low battery)

```
   dock face  ▐█▌ marker (8 cm, DICT_4X4_50, id 0)
              ▐ ▌ contacts
               ◄── creep ≤ 5 cm/s ──── [robot] ◄─ pre-dock pose: 0.60 m in front, facing the dock
```
1. **Pre-dock**: the navigator plans to the pre-dock pose (`dock_predock_m` in front of the saved dock pose,
   facing it) like any other goal.
2. **Align**: turn on the spot until the marker is in view and within 6° of straight ahead. If it isn't
   seen, sweep ±25°. After 14 s with no marker: back off and try again.
3. **Creep**: drive at `dock_creep_speed` (0.04 m/s, never above 0.05) along the dock line, using
   pure pursuit to a point 20 cm ahead on the line. That point corrects the sideways offset and the heading
   together.
   - Docking mode is on: the navigator publishes `/dock_mode`, the bridge sends `D,1` every 0.5 s, and the
     ESP32 relaxes its 15 cm front ultrasonic stop to 3 cm, but only while it is refreshed and only at
     ≤ 6 cm/s. Any faster command, or 2 s without a refresh, and the normal stop is back.
   - If the marker is lost for more than 1 s: back off and retry.
4. **Contact**: the last 20 cm, where the camera is too close to see the marker. The robot goes straight
   on at 3 cm/s, holding the dock line's heading on odometry.
   - Contact is the INA226 charging current, or a battery voltage rise of `dock_volt_rise` (0.15 V) when
     there is only the divider, held for 1 s.
   - If no contact comes within 26 cm or 10 s: back off.
5. **Back off and retry**: reverse 25 cm, only if the LiDAR sees nothing close behind, then align again. This
   happens up to `dock_tries` (3) times.
6. **Fail**:
   - `DOCK_NO_MARKER` means the marker was never seen; `DOCK_NO_CONTACT` means it was seen but there was no contact.
   - Error recovery (`docs/recovery.md`) then decides: wait and retry (the whole approach again), or ask a person.
7. **Leaving**: the next task first backs straight off the dock (30 cm, rear checked by the LiDAR), then plans.

### Heading
The marker's own yaw is noisy until it is large in the picture: an 8 cm marker at 0.8 m is only ~30 px.
So the heading of the dock line comes from the map (the robot's SLAM yaw against the saved dock yaw), and the
marker yaw is used only closer than 0.45 m when it agrees within 20°. The marker always gives the position.
That is what removes the 2–5 cm sideways error a map-only approach has (`docking.fuse_yaw`).

## The detector interface (for Neelaksh's module)

```python
from lidar_robot.marker_pose import detect
detect(frame) -> {'found': True, 'x_m': 0.03, 'y_m': -0.01, 'z_m': 0.62, 'yaw_deg': -4.0}
```
`x_m` is to the right of the camera axis, `y_m` below it, `z_m` straight ahead (OpenCV camera frame).
`yaw_deg` is 0 when the marker faces the camera; it is > 0 when its face points to the camera's right.
Any detector returning these keys can replace `MarkerDetector.detect`: ArUco, AprilTag or a CNN.
The vision node publishes the result as JSON on `/vision/dock_marker`.

## Settings

| where | name | default | |
|---|---|---|---|
| navigator | `dock_direction` | `front` | `reverse` = the v2.4 behaviour (back onto the dock) |
| navigator | `dock_predock_m` | 0.60 | pre-dock distance |
| navigator | `dock_contact_dist_m` | 0.10 | base_link to the marker plane when docked: **measure on the dock** |
| navigator | `dock_creep_speed`, `dock_tries`, `dock_backoff_m`, `dock_volt_rise` | 0.04, 3, 0.25, 0.15 | |
| navigator | `camera_x/y/z` | 0.05, 0, 0.20 | camera position in base_link: **measure** |
| navigator | `dock_without_marker` | false | true = dock on the map pose alone (no camera) |
| vision | `dock_marker`, `marker_dict`, `marker_id`, `marker_size_m`, `marker_rate` | true, DICT_4X4_50, 0, 0.08, 10 | |
| vision | `camera_calib` | `''` | measured calibration YAML (format in `config/camera_calib.yaml`) |
| firmware | `US_STOP_DOCK_MM`, `DOCK_MAX_MMS`, `DOCK_MODE_HOLD_MS` | 30, 60, 2000 | S3 v2.7, classic v2.5 |

## Camera calibration
The built-in values are approximations for the Camera Module 3 Wide: 640×480 has fx ≈ 346 and 640×360 has
fx ≈ 259. The two sizes are different crops of the sensor, so one is not a scaled copy of the other.
Calibrate once and save the result in the same format as `config/camera_calib.yaml`:

```bash
# on the laptop with a printed 9x6 chessboard (any OpenCV calibration tutorial / tool works), or on the Pi:
python3 -c "import cv2; print(cv2.__version__)"
```
Then set `camera_calib: /home/arpit/maps/camera_calib.yaml` in the vision section, at the size the vision node
uses (640×360 by default).

## Try it without the robot
- **Laptop webcam**:
  - `python3 tools/marker_webcam.py --print marker.png`: print it at 100 % and check the 8 cm square.
  - `python3 tools/marker_webcam.py`: live view; z_m should match a tape measure.
- **Simulator**: `python3 tools/dashboard_sim.py --demo-users`, then Charge in the app. The simulated robot
  docks nose first, and the Home card shows In front → Find marker → Creep in → Contact → Charging.
- **Demo mode** in the app does the same.
- **Tests**: `test_docking.py`:
  - kinematic docking from offset and turned starts, noisy marker, search sweep, retries, both failure codes,
    voltage-rise contact;
  - the detector on rendered marker images;
  - the full simulator dock.
