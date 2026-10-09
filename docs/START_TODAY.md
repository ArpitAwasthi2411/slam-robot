# Start here: from the new files to a robot that maps by itself

Order: **A** flash the ESP32 → **B** copy the code to the Pi → **C** start the robot → **D** connect
the phone → **E** checks → **F** tune → **G** let it map by itself.
Section **N** at the end compares all the ways to connect.

What you need: laptop with Arduino IDE, ethernet cable, the project zip `slam-robot.zip`,
the APK `Pathik-4.apk`, robot battery charged, the RC transmitter.

---

## A. Flash firmware v2.3 to the ESP32 (10 min, laptop)

1. Unzip `slam-robot.zip` on the laptop, e.g. into `~/Downloads/slam-robot/`.
2. Unplug the ESP32 USB cable from the Pi and plug it into the laptop.
3. Arduino IDE → open `firmware/robot_esp32_classic/robot_esp32_classic.ino`.
4. Check these lines (they're already your measured values):
   ```cpp
   #define LEFT_MOTOR_DIR    -1
   #define RIGHT_MOTOR_DIR    1
   #define LEFT_ENC_DIR      -1
   #define RIGHT_ENC_DIR     -1
   ```
5. Tools → Board: **ESP32 Dev Module**, Port: the ttyUSB that appeared → **Upload**.
   If it hangs at "Connecting…", hold the **BOOT** button on the ESP32 until it starts writing.
6. Serial Monitor at 115200: you should see `INFO,READY,robot_esp32 v2.3`. Close the Serial Monitor.
7. Plug the ESP32 back into **the same Pi USB socket** as before.

## B. Copy the new code to the Pi (10 min)

1. Robot ON, ethernet cable laptop ↔ Pi. Laptop terminal:
   ```bash
   ping -c 2 169.254.1.2
   ```
   No reply? Run once: `bash ~/Downloads/slam-robot/scripts/setup_laptop_link.sh` (makes the laptop
   address 169.254.1.1 permanent), then ping again.
2. Send the project to the Pi (laptop):
   ```bash
   cd ~/Downloads
   scp slam-robot.zip arpit@169.254.1.2:~
   ssh arpit@169.254.1.2
   ```
3. On the Pi (this SSH window = **PI-1**). The first line keeps your current calibration file safe:
   ```bash
   cp ~/ros2_ws/src/lidar_robot/config/robot_params.yaml ~/robot_params.backup.yaml
   rm -rf ~/slam-robot && mkdir ~/slam-robot && cd ~/slam-robot && unzip -q ~/slam-robot.zip
   rm -rf ~/ros2_ws/src/lidar_robot && cp -r ~/slam-robot/ros2_ws/src/lidar_robot ~/ros2_ws/src/
   diff ~/robot_params.backup.yaml ~/ros2_ws/src/lidar_robot/config/robot_params.yaml
   ```
   If `diff` shows a value you changed on the Pi (ticks, `wheel_separation`), keep yours:
   `cp ~/robot_params.backup.yaml ~/ros2_ws/src/lidar_robot/config/robot_params.yaml`
4. Build:
   ```bash
   cd ~/ros2_ws && colcon build --symlink-install --packages-select lidar_robot && source install/setup.bash
   ```
5. One-time shortcut, so `robot_up` starts everything (skip if you already have it):
   ```bash
   echo "alias robot_up='source ~/ros2_ws/install/setup.bash && ros2 launch lidar_robot bringup.launch.py'" >> ~/.bashrc
   source ~/.bashrc
   ```

## C. Start the robot (2 min)

PI-1:
```bash
ls -l /dev/esp32 /dev/rplidar        # both must exist; if not: sudo bash ~/slam-robot/scripts/setup_ports.sh
robot_up
```
Leave it running. Look for `Connected to ESP32 on /dev/esp32` and no red errors from `sllidar`.
LiDAR error `80008000`: unplug the LiDAR USB for 5 s, plug back, `robot_up` again.

## D. Connect the phone: the robot makes its own Wi-Fi (15 min the first time, then nothing)

The Pi becomes the hotspot: it creates a Wi-Fi called **SLAM-Robot**, and the phone (and laptop)
join it. No phone hotspot or lab Wi-Fi needed, and the robot's address is always **10.42.0.1**.

**One-time setup.** The Pi needs one program, NetworkManager, and has to download it once.
For those 5 minutes the Pi borrows your phone's internet:

1. Phone: turn the hotspot ON (note its name and password).
2. PI-2 (second laptop terminal: `ssh arpit@169.254.1.2`). The ethernet cable keeps SSH working
   through all of this:
   ```bash
   sudo bash ~/slam-robot/scripts/setup_wifi.sh hotspot "<phone hotspot name>" "<password>"
   ping -c 2 google.com                     # must answer: the Pi has internet now
   sudo apt update && sudo apt install -y network-manager
   ```
3. Turn the Pi into the hotspot (pick your own password, at least 8 characters):
   ```bash
   sudo bash ~/slam-robot/scripts/setup_wifi.sh ap robot1234
   ```
   It prints `Robot Wi-Fi is up: name SLAM-Robot`. You can turn the phone hotspot OFF now.
4. Check: `sudo bash ~/slam-robot/scripts/setup_wifi.sh status` shows `robot-ap` on wlan0 with
   10.42.0.1. It comes back by itself at every boot.

**Every day from now on:**
1. Install `Pathik-4.apk` on the phone (once; allow "install unknown apps").
2. Robot ON, wait ~1 minute.
3. Phone → Wi-Fi → join **SLAM-Robot** (password `robot1234`). Android may say "no internet":
   tap **stay connected / keep**, otherwise it jumps back to mobile data or another Wi-Fi.
4. Pathik → **Find robot** → tap it (or type `10.42.0.1`).

The laptop can join SLAM-Robot too (then `ssh arpit@10.42.0.1`), or keep using the cable.

**Limits of robot Wi-Fi:** no internet on the robot while it's the hotspot, so typed commands use the
offline parser instead of the LLM. Range is about 15–25 m. If the robot drives farther, the app
reconnects when it comes back, and the robot keeps doing its job meanwhile.
Need internet on the robot again (LLM, `apt`, `git`)? Run `setup_wifi.sh hotspot …` to join the
phone; run `setup_wifi.sh ap robot1234` to go back.

**If `apt install` says "Temporary failure resolving 'ports.ubuntu.com'"** (the Pi has no internet):
- `ping -c 2 8.8.8.8` works but `ping google.com` doesn't → only DNS is missing:
  `sudo resolvectl dns wlan0 8.8.8.8` and try again.
- `ip -4 addr show wlan0` shows no address → the Pi didn't join the hotspot. Set the phone hotspot
  to **2.4 GHz** (Hotspot → Advanced → AP band), check the password, rerun `setup_wifi.sh hotspot …`.
- Or skip Wi-Fi and share the **laptop's** internet through the ethernet cable (see "Internet for the
  Pi through the cable" below).

**If step 3 fails:**
- *"a netplan file configures wlan0"*: the Pi's original Wi-Fi setup is in a netplan file. The
  script prints its name. Open it with `sudo nano <file>`, delete the `wifis:` part (keep `ethernets:`),
  then `sudo netplan apply` and run step 3 again.
- *SLAM-Robot doesn't show on the phone:* `sudo nmcli connection up robot-ap` and read the error;
  `sudo journalctl -u NetworkManager -n 30` shows why.

## E. Checks before driving (5 min)

In Pathik:
- **Status** tab: Motor board ~50 Hz, Firmware **v2.3**, LiDAR 5–10 scans/s, Position shown.
- **Drive** tab: the map appears and the orange dots sit on the walls.
- Gently move the joystick: the robot moves the same way on screen. Big red **STOP** stops it.
- RC remote: move a stick, it must override the app immediately.

## F. Tune so it drives smoothly (20 min, once; values are saved)

Robot on the floor, open space. Pathik → **Lab**:
1. **Wheels** → *Run step test* at 0.20 m/s. Set **Acceleration ramp** ~600, adjust **Kp** then **Ki**
   until the cards say good (overshoot under 10 %, steady error under ~6 mm/s). **Save to robot**.
2. **Drive tests → Spin** → Run. Put the `wheel_separation` it prints into
   `~/ros2_ws/src/lidar_robot/config/robot_params.yaml`, Ctrl+C in PI-1, `robot_up` again.
3. **Drive tests → Straight line** → Run. Under 3 cm drift per metre = good.
4. **Path following**: long-press the map to send it somewhere 2–3 m away. Weaving → lower
   Steering gain. Cutting corners → lower Look-ahead. **Save**.

Details: `docs/mobile_app.md` section 4.

## G. Let it map by itself (15–20 min per area)

1. Restart in mapping mode for a fresh map: Ctrl+C in PI-1, `robot_up`.
2. Open the doors you want mapped, clear bags and chairs from the floor, cover big mirrors/glass.
3. Pathik → **Drive** → scroll to **Map by itself** → **Careful** for the first run → switch
   "Turn once at every spot" ON → **Start mapping** → confirm.
4. Walk near it with the RC remote. Magenta dots = areas still to visit, dashed ring = next target.
5. When it's done it comes back to the start and shows "Map saved as explore_…".
   Check on the Pi: `ls ~/maps/` → `explore_….yaml/.pgm/.pbstream`.
6. Copy the map to the laptop as a backup:
   ```bash
   scp arpit@169.254.1.2:~/maps/explore_* ~/Downloads/slam-robot/maps/
   ```
7. Later, to navigate on that map: `ros2 launch lidar_robot bringup.launch.py slam_mode:=localization map:=$HOME/maps/<name>.pbstream`
   → Position OK → save places → Go.

Full guide: `docs/exploration.md`.

---

## N. Network: every way to connect, and which to use

| # | Way | Setup (once) | Every day | Range | Internet on robot | Good for |
|---|---|---|---|---|---|---|
| 1 | **Ethernet cable** laptop ↔ Pi (169.254.1.2) | done | plug in | 1 cable | no | SSH, flashing, RViz, fixing things. Never drops |
| 2 | **Robot's own Wi-Fi** "SLAM-Robot" (10.42.0.1): the Pi is the hotspot | section D (one-time install) | power on, phone joins SLAM-Robot | ~15–25 m | no (offline commands) | any lab, no lab Wi-Fi needed, address never changes |
| 3 | **Robot joins your phone's hotspot** | `setup_wifi.sh hotspot "<name>" "<pw>"` | turn hotspot on, Find robot | ~10–15 m around you | yes (LLM works) | when the robot needs internet (LLM commands, installing things); you walk with the robot |
| 4 | **Lab / college Wi-Fi** | `setup_wifi.sh extra "<SSID>" "<pw>"` | Find robot | that building | yes | only if the network lets devices talk to each other (many college networks block it) |
| 5 | **A spare phone riding on the robot is the hotspot** (4G) **+ Tailscale** | spare phone with SIM + `setup_tailscale.sh` + Tailscale app on your phone | type `slam-robot` in Pathik | anywhere with 4G | yes | the robot going to other labs/floors alone |

**Easiest when you walk into a lab:** option **2, the robot's own Wi-Fi**. It doesn't care what
network the lab has. Power on, phone joins "SLAM-Robot", Pathik → Find robot, done. The address is
always 10.42.0.1.

**Most reliable:**
- For setup and debugging: option **1, the cable**. No wireless at all.
- For the robot moving between rooms or labs: option **5**. Its connection travels with the robot,
  so you can watch it from anywhere. And with any option, a mission or Map by itself keeps running
  on the Pi if the link drops.

Setting up option 2: section D above.

**Start at power-on without SSH** (recommended once everything works):
```bash
sudo cp ~/slam-robot/scripts/robot.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now robot
```
Then: battery ON → wait 1 minute → phone joins the Wi-Fi → Pathik. Stop it with
`sudo systemctl stop robot` before using `serial_probe.py` or running `robot_up` by hand.

Option 5 step by step: `docs/remote_access.md`.

---

## If something goes wrong

| Symptom | Fix |
|---|---|
| `ping 169.254.1.2` fails | cable in? `bash scripts/setup_laptop_link.sh` on the laptop |
| `/dev/esp32` or `/dev/rplidar` missing | `sudo bash ~/slam-robot/scripts/setup_ports.sh` |
| LiDAR `80008000` | unplug LiDAR 5 s, replug, `robot_up` |
| Pathik doesn't find the robot | phone and robot on the same network? `setup_wifi.sh status`; type the address by hand |
| Firmware shows 2.2 in Status | step A not done or a different ESP32 port: re-flash |
| One wheel runs away | an `*_ENC_DIR` is wrong: check with `serial_probe.py --push` (stop the robot first) |
| Map smears / doubles | drive slower; use Map by itself on Careful; check `laser_x` is -0.13 |

---

## Internet for the Pi through the cable (when Wi-Fi won't cooperate)

The laptop shares its own internet with the Pi over the ethernet cable. Nothing is permanent: it's
gone after a reboot.

LAPTOP (laptop on Wi-Fi with internet):
```bash
WAN=$(ip route show default | awk '{print $5; exit}'); echo "laptop internet is on: $WAN"
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o "$WAN" -j MASQUERADE
sudo iptables -I FORWARD -i enx006f0000324d -o "$WAN" -j ACCEPT
sudo iptables -I FORWARD -i "$WAN" -o enx006f0000324d -m state --state RELATED,ESTABLISHED -j ACCEPT
```
PI:
```bash
ETH=$(ip -o -4 addr show | awk '/169.254.1.2/{print $2}'); echo "pi cable is: $ETH"
sudo ip route replace default via 169.254.1.1 dev "$ETH"
sudo resolvectl dns "$ETH" 8.8.8.8 && sudo resolvectl domain "$ETH" '~.'
ping -c 2 google.com
sudo apt update && sudo apt install -y network-manager
```

