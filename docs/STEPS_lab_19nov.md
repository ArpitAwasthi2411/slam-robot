# Lab 19 Nov: map the department with the laptop on the robot

Goal: one clean department map, saved in three places (Pi, laptop, recording), then the robot drives on it.
INA226 and docking stay parked today (INA unplugged: it took the I2C bus down on 18 Nov).

What changed in the code since 18 Nov (all pulled in step 1):
- Cartographer no longer crashes on a saved map (1 scan slice instead of 10).
- Saves are written to `name.pbstream.part` and renamed when complete: a crash never leaves a broken map.
- Autosave never deletes the biggest (longest) drive.
- After **Freeze**, the robot is placed where it stood, no Set position needed.
- Navigator / app server no longer run twice internally (idle CPU, "position lost" stalls).
- Repo now holds the Pi's own settings (marker 72.5 mm, USB sound card), so `git pull` never conflicts.

---

## 0. Bring

Laptop (charged) + charger, USB-Ethernet adapter, Ethernet cable, phone (hotspot + app), tape/strap for the laptop.

## 1. Update the Pi (phone hotspot on, `ssh robot.local`)

```bash
cd ~/slam-robot && git checkout -- ros2_ws/src/lidar_robot/config/ && git pull && cd ros2_ws && colcon build --packages-select lidar_robot
sudo bash ~/slam-robot/scripts/setup_voice.sh install
```
The first line throws away yesterday's hand edits to the config files: the same values are in the repo now.
If `git pull` still complains about a file, send a screenshot.

## 2. Update the laptop (still on Wi-Fi with internet)

```bash
cd ~/slam-robot && git pull
```

## 3. Robot health (robot on, standing still)

App → Operate → Health:

| Card | Must show |
|---|---|
| Motor board | 50 Hz on /dev/ttyAMA0 |
| Firmware | v2.9 |
| IMU | working (not "not fitted") |
| Heading from | gyro + wheels |
| LiDAR | about 7 scans/s |

Then (Pi):
```bash
sleep 60; python3 ~/slam-robot/tools/cpu_report.py
```
Screenshot it (CPU per program at idle, after the fix).

## 4. Encoders (yesterday's jerky driving): must pass before driving

Wheels free or robot lifted:
```bash
sudo systemctl stop robot
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --push --metres 1
```
Push 1 m by hand (or turn each wheel one full turn forward). Both wheels count **up**, roughly equal
(~5000 per wheel turn). 0 or jumping = loose encoder wire (GND first); negative = A/B swapped.
```bash
sudo systemctl start robot
```
Drive 2 m slowly with the joystick: straight and smooth, no lurching.

## 5. Pi: the laptop builds the map, camera off

```bash
sed -i 's|^ROBOT_ARGS=.*|ROBOT_ARGS=esp32_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 vision:=false slam_host:=laptop|' ~/robot.env && grep ROBOT_ARGS ~/robot.env && sudo systemctl restart robot
```

## 6. Cable + laptop

USB-Ethernet adapter in the laptop, cable to the Pi's Ethernet port. On the laptop:
```bash
bash ~/slam-robot/scripts/setup_laptop_link.sh       # must say "Using interface: enx..."
ping -c 3 169.254.1.2
ssh-copy-id arpit@169.254.1.2                         # once; asks the Pi password
nmcli radio wifi off                                  # robot data only over the cable
python3 ~/slam-robot/scripts/laptop_slam.py --check   # must end with "all good"
```
If ping fails, set the Pi side once (from the phone-hotspot SSH, before Wi-Fi off):
```bash
sudo nmcli con add type ethernet ifname eth0 con-name robot-link ipv4.method manual ipv4.addresses 169.254.1.2/16 ipv4.never-default yes; sudo nmcli con up robot-link
```

## 7. Start map building on the laptop

Terminal 1:
```bash
python3 ~/slam-robot/scripts/laptop_slam.py
```
Terminal 2 (RViz, with a log in case it closes):
```bash
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
rviz2 -d ~/slam-robot/ros2_ws/src/lidar_robot/rviz/robot.rviz 2>&1 | tee ~/rviz_log.txt
```
Strap the laptop over or just in front of the wheel axle, lower than the LiDAR, nothing in its view.

## 8. Map

**Preferred: add to the saved map** (`Cse_department_aids`, 18 Nov) instead of starting over. Cartographer puts the
new part where the OLD map started (0, 0, 0°), so the robot must stand exactly there when it continues:
1. Map → **Use a saved map** → `Cse_department_aids`; after the restart check the robot's place (Set position if wrong).
2. Drive slowly until Operate → **Position** reads about 0.0, 0.0 m, 0° (within ~10 cm, a few degrees).
3. Map → **Continue mapping** → `Cse_department_aids` (the laptop fetches the map from the Pi by itself).
4. First drive a few metres along an already-mapped part (it locks on), then into the new areas; finish in a mapped
   corridor. Freeze under a NEW name (`cse_department_full`) so the old map stays as a fallback.

**Or start fresh:** App → Map → **New map**. The Map card says "Map built on the laptop"; terminal 1 says it started Cartographer and
a recording. Drive:
- stand still 5 s, then straight first; half speed, slow turns, short pause after each turn;
- corridor first; each lab: in, one slow loop, out the same way;
- finish at the start point and drive the first stretch again (closes the loop);
- phone near the robot; watch RViz: the red scan must sit on the walls.

If the cable comes out, the robot stops by itself within 10 s: plug it back in within 15 s and carry on.

## 9. Freeze

Stop the robot → Map → **Freeze map**, name `cse_department` → wait (up to 2 min). Terminal 1 says the map was copied
to the Pi. Check on the laptop:
```bash
ls -lh ~/pathik_maps/ ~/bags/ | tail
```

## 10. Drive on the frozen map

After the restart the robot appears where it was frozen (no Set position needed; if it is wrong, Map → Set position).
Push the joystick: the robot marker follows along the walls. Save places (Map tab → save place): HOD office,
labs, stairs, entrance, home spot.

## 11. Back to normal + backup

Laptop Wi-Fi on, then:
```bash
ssh arpit@169.254.1.2 "sed -i 's/ slam_host:=laptop//; s/vision:=false/vision:=true/' ~/robot.env && grep ROBOT_ARGS ~/robot.env && sudo systemctl restart robot"
rsync -av arpit@169.254.1.2:~/maps/ ~/pathik_maps/
```

---

## Results

| Step | Result | Notes |
|---|---|---|
| 3 health cards OK? | | |
| 3 cpu_report (navigator / dashboard %) | | |
| 4 encoder counts L / R per metre | | |
| 6 laptop_slam --check | | |
| 7 RViz stays open with data? | | |
| 8 map: torn / clean | | |
| 9 Freeze time + file size | | |
| 10 robot placed right after Freeze? | | |
| 10 places saved | | |
