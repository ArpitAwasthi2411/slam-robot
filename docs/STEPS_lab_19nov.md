# Lab 19 Nov: Pathik, step by step

**Today's goal:** one clean map of the department, built on the laptop, saved in 3 places (Pi, laptop,
recording). Then the robot drives on it, places get saved, and the new door / room / "outside X" code gets its
first try. INA226 and docking stay parked (the INA226 is unplugged: it took the I2C bus down on 18 Nov).

Order: **A** before leaving · **B** Pi update + health · **C** encoders · **D** laptop link · **E** mapping ·
**F** Freeze · **G** drive on the map · **H** doors / rooms / "outside X" · **I** back to normal + backup.
Fill the results table at the end as you go (photos/screenshots are fine).

---

## A. Before leaving (home, laptop on Wi-Fi)

```bash
cd ~/slam-robot && git pull
python3 ~/slam-robot/scripts/laptop_slam.py --check     # the laptop half; it may complain about the Pi: fine at home
```
Bring: laptop + charger, USB-Ethernet adapter, Ethernet cable, phone (hotspot + Pathik app), tape/strap for the
laptop, measuring tape, a marker pen.

## B. Pi: update, then health (robot on, standing still, phone hotspot on)

```bash
ssh robot.local
cd ~/slam-robot && git checkout -- ros2_ws/src/lidar_robot/config/ && git pull && cd ros2_ws && colcon build --packages-select lidar_robot && sudo systemctl restart robot
```
(The first part throws away hand edits to the config files: the same values are in the repo now.)

App → Operate → **Health**, after ~30 s:

| Card | Must show |
|---|---|
| Motor board | ~50 Hz on /dev/ttyAMA0 |
| Firmware | v2.9 |
| IMU | working (not "not fitted") |
| Heading from | gyro + wheels |
| LiDAR | about 7 scans/s |

IMU "not fitted" again? Check SDA 11 / SCL 12 and that nothing else is on the I2C wires (INA226 must be unplugged).

Then (robot idle):
```bash
sleep 60; python3 ~/slam-robot/tools/cpu_report.py
```
Screenshot it: CPU of navigator / dashboard at idle after the 18 Nov fix (was ~60 % each).

## C. Encoders: must pass before any driving (yesterday's jerky / too fast driving)

Robot lifted or wheels free:
```bash
sudo systemctl stop robot
python3 ~/slam-robot/tools/serial_probe.py --port /dev/esp32 --push --metres 1
```
Turn each wheel one full turn forward by hand. Both must count **up**, about **5000 per turn**.

| You see | Means |
|---|---|
| 0 on one wheel | encoder GND or signal wire loose (GND first, as on 30 Oct) |
| jumps / random | loose wire or 5 V to the encoder flaky |
| negative | A/B swapped |

```bash
sudo systemctl start robot
```
Then joystick 2 m slowly: straight, smooth, no lurching. **Not smooth = stop here and fix the wiring.** A map
made with bad odometry tears.

## D. Laptop on the robot, cable to the Pi

1. Pi: the laptop builds the map, camera off (saves CPU):
   ```bash
   sed -i 's|^ROBOT_ARGS=.*|ROBOT_ARGS=esp32_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 vision:=false slam_host:=laptop|' ~/robot.env && grep ROBOT_ARGS ~/robot.env && sudo systemctl restart robot
   ```
2. USB-Ethernet adapter in the laptop, cable to the Pi's Ethernet port. On the laptop:
   ```bash
   bash ~/slam-robot/scripts/setup_laptop_link.sh       # must say "Using interface: enx..."
   ping -c 3 169.254.1.2
   ssh-copy-id arpit@169.254.1.2                         # once; asks the Pi password
   nmcli radio wifi off                                  # robot data only over the cable
   python3 ~/slam-robot/scripts/laptop_slam.py --check   # must end with "all good"
   ```
   Ping fails? On the Pi (from the hotspot SSH, before Wi-Fi off):
   ```bash
   sudo nmcli con add type ethernet ifname eth0 con-name robot-link ipv4.method manual ipv4.addresses 169.254.1.2/16 ipv4.never-default yes; sudo nmcli con up robot-link
   ```
   `ssh-copy-id` says "no identities"? Run `ssh-keygen` (Enter 3 times) first.
3. Laptop, terminal 1 (leave it running all session):
   ```bash
   python3 ~/slam-robot/scripts/laptop_slam.py
   ```
   It opens RViz itself (on the NVIDIA card if the laptop has one, log in `~/rviz_log.txt`) and reopens it if it
   crashes. **Don't start a second RViz.** Closed it? `python3 ~/slam-robot/scripts/laptop_slam.py --rviz-only`.
   Still freezing? Start with `--gpu software`. RViz is only for watching: mapping runs without it.
4. Strap the laptop over / just in front of the wheel axle, **lower than the LiDAR**, nothing in its view.

Terminal 1 will just wait at first: the Pi still remembers "drive on `Cse_department_aids`" from 18 Nov. It starts
Cartographer only after you choose **Continue mapping** or **New map** in the app (next step). That is expected.

## E. Map the department

Pick one:

**E1. Add to the 18 Nov map (`Cse_department_aids`), only explore what's missing.** The new part is joined where
the OLD map started (0, 0, 0°), so the robot must stand exactly there:
1. App → Map → **Use a saved map** → `Cse_department_aids`. After the restart (~20 s) check the robot sits right
   on the map (Map → Set position if not).
2. Drive slowly until Operate → **Position** reads about 0.0, 0.0 m, 0° (within ~10 cm and a few degrees).
3. Map → **Continue mapping** → `Cse_department_aids` (the laptop fetches the map from the Pi by itself;
   terminal 1 says "continuing the saved map").
4. Drive a few metres along a part that is already mapped first (it locks on), then into the new areas.

**E2. Fresh map (if E1 looks wrong in RViz in the first minute, or you can't find 0,0,0):**
App → Map → **New map**. The Map card says "Map built on the laptop"; terminal 1 says it started Cartographer
and a recording.

How to drive (both):
- stand still 5 s at the start, then straight first; half speed, slow turns, short pause after each turn;
- corridor first; each lab: in, one slow loop, out the same way;
- end back at the start and drive the first stretch again (closes the loop);
- keep the phone close to the robot; watch RViz: the red scan must sit on the walls.

| If | Do |
|---|---|
| Cable comes out | robot stops by itself within 10 s; plug back within 15 s and carry on |
| Terminal 1: "Cartographer stopped by itself: starting it again" | the map in progress is lost. Stop. App → Map → **Use a saved map** → newest `autosave_…` → then **Continue mapping** on it (stand at 0,0,0 as in E1) |
| Map starts tearing (walls doubling) | stop, back up to the last good spot, drive slower; if it stays torn, Freeze what you have under a new name and start E2 |
| RViz closes | the map is not affected; send `~/rviz_log.txt` |

The app autosaves the live map every 3 min anyway (Map tab shows the last one).

## F. Freeze

Robot stopped → Map → **Freeze map** → name **`cse_department_full`** (a NEW name, so the 18 Nov map stays as a
fallback) → wait, up to 2 min. Terminal 1 says the map was copied to the Pi. Check on the laptop:
```bash
ls -lh ~/pathik_maps/ ~/bags/ | tail
```
Note the Freeze time and file size.

## G. Drive on the frozen map

After the restart the robot should appear **exactly where it stood when you froze** (this step was broken before
19 Nov and is fixed now: no Set position needed). If it's wrong: Map → Set position.

1. Joystick a few metres: the robot marker follows along the walls.
2. Save places (Map tab → save place) while standing at each: **HOD office, each lab, stairs, entrance, Ravi sir's
   cabin, home spot**.
3. Measure 3 long straight lengths with the tape (a corridor, a lab wall, between two doors) and the same lengths
   on the map (Map tab ruler). Write both in the table: under 1 % off is good.
4. Send 3 trips from the app (e.g. home → Lab → HOD office → home). Note: arrived / stopped / where.

## H. Doors, rooms, "outside X" (new code, first try)

1. **Measure the robot:** widest point side to side (wheels included), axle → front bumper, axle → rear edge.
   Write them in the table and send them to me. Don't edit the yaml on the Pi by hand: I'll put them in the repo
   so `git pull` stays clean.
2. **Rooms of any shape:** Map tab → Room tool → **tap** each corner of Ravi sir's cabin (Undo removes the last) →
   **Save room**. Also draw the corridor as a room named **"Corridor"**. Drag still makes a rectangle.
3. **Outside a place:** Send → `stand in the corridor near Ravi sir's cabin`. It must say "Going outside Ravi sir
   cabin" and stop in the corridor in front of his door, facing it. Try the Hindi too:
   `Ravi sir ke cabin ke bahar khade raho`.
4. **Doors:** send a trip through the narrowest door. It must not stop at the door frame. If the log says
   "narrow passage", it is planning with its real width: walk next to it.
5. **Space on the map but no path:** if a trip still says "no route", screenshot the Map tab with the start and
   goal visible + the log line.

## I. Back to normal + backup

Laptop Wi-Fi on, then:
```bash
ssh arpit@169.254.1.2 "sed -i 's/ slam_host:=laptop//; s/vision:=false/vision:=true/' ~/robot.env && grep ROBOT_ARGS ~/robot.env && sudo systemctl restart robot"
rsync -av arpit@169.254.1.2:~/maps/ ~/pathik_maps/
```
After this the Pi runs its own Cartographer again (localization on the frozen map: light, fine on the Pi).

Also keep for the report/chart: a screenshot of the full map in the app, the `.pgm` + `.yaml` of the frozen map
(in `~/pathik_maps/`), and a few photos of the robot driving in the corridor.

---

## Results (fill in, send me a photo of this)

| Step | Result | Notes |
|---|---|---|
| B health cards all OK? | | |
| B cpu_report: navigator % / dashboard % | | |
| C encoder counts per wheel turn L / R | | |
| C 2 m joystick drive smooth? | | |
| D laptop_slam --check "all good"? | | |
| D RViz stays open with data? | | |
| E E1 or E2? map torn / clean | | |
| F Freeze time + file size | | |
| G robot placed right after Freeze? | | |
| G places saved (how many) | | |
| G 3 lengths: tape / map (m) | | |
| G 3 trips: arrived / stopped where | | |
| H robot width / front / rear (cm) | | |
| H "corridor near Ravi sir's cabin": stopped where? | | |
| H narrowest door: passed / stopped | | |
| H any "no route" (screenshot) | | |
