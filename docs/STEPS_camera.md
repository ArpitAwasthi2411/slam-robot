# Camera: Camera Module 3 Wide + people avoidance (vision node)

What it does: the camera looks ahead, YOLO11n finds people (~4 per second), the LiDAR gives each
person's distance, and the robot **slows down** (person in the path within 1.8 m) or **stops and waits**
(closer than 0.8 m, or walking towards it), saying "Excuse me, may I pass?". Turning and reversing
are not limited except while stopped. The app shows the live picture with boxes, the decision and why.
Manual driving (joystick) is never blocked by the camera; LiDAR/ultrasonic safety work as before.

Do it once per SD card. Robot ON, Pi on Wi-Fi/ethernet with internet for the downloads.

## 1. Connect the camera (Pi OFF)
Ribbon into the Pi's **CAMERA** connector (between HDMI and audio jack), blue tab towards the
USB/Ethernet ports. Lift the latch, push the ribbon in fully, close the latch. Camera looking forward,
centred on the robot. Mounted upside down? set `flip: true` in `config/robot_params.yaml` (vision section).

## 2. libcamera for the Camera Module 3 (30–60 min)
```bash
sudo bash ~/slam-robot/scripts/setup_pi_camera.sh
sudo reboot
```
Check after the reboot:
```bash
sudo systemctl stop robot
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu cam -l          # shows imx708_wide
```
If the script stops at "no imx708 driver": `sudo apt update && sudo apt full-upgrade -y && sudo reboot`, run it again.

## 3. People detection (15–30 min the first time)
```bash
bash ~/slam-robot/scripts/setup_vision.sh        # NOT sudo
sudo systemctl restart robot
```
Check:
```bash
source ~/ros2_ws/install/setup.bash && ros2 topic echo /vision/decision --once
```
`"camera_ok": true, "detector": true` and an `action`. Stand in front of the robot: `STOP` / `SLOW_DOWN`.

## 4. App
Top bar → **camera button** → live picture. The dot on the button: green clear, orange slowing, red stopped.
⤢ makes it big, × closes it. While closed, a banner on the map says when the camera is holding the robot.

## Settings (`config/robot_params.yaml`, section `vision`)
`stop_m` / `slow_m` distances, `path_half_angle_deg` (how wide "in the path" is), `det_rate`, `threads`,
`flip`. Turn the camera off: `ROBOT_ARGS=... vision:=false` in `~/robot.env`. Keep the picture but never
slow down for it: navigator `use_vision: false`.

## Problems
| App says | Fix |
|---|---|
| Camera program is not running | `journalctl -u robot -b \| grep -i vision`; was `colcon build` run with the new code? |
| libcamera not found | step 2 not done, or no reboot after it |
| camera stream ended … | ribbon loose or wrong way round (Pi off!), then `sudo systemctl restart robot` |
| People detection is off | step 3 (`~/models/yolo11n_ncnn_model` missing or `ncnn` not installed) |
| Stops for nobody / too late | tune `stop_m`, `slow_m`; check `laser_yaw` (camera bearing ↔ LiDAR angle) |
