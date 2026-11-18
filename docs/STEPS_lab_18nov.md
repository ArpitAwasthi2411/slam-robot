# Lab 18 Nov: INA226, the dock (contact sense), voice, laptop mapping

Today's order. Each part ends with a check; write the results in the table at the end.

What is new in the code (pull it first, part 0):
- Firmware **v2.9**: GPIO 1 becomes the **dock contact sense**. The dock puts 12 V on the robot's contact plates;
  the robot sees it and knows it has arrived. Line `DCK,<0|1>,<mV>`.
- Docking counts "dock voltage on the plates" as contact (no charger on the dock yet: we still charge the battery
  off the robot with our own charger).
- Voice: priyamvada (Hindi, female) + lessac (English, female); Hindi answers use female verbs ("ja rahi hoon").
- Laptop mapping (`docs/laptop_slam.md`).

> Safety for the whole day: battery **unplugged** while wiring. The 12 V adapter **never** connects to the
> battery: it only goes to the dock's plates.

---

## 0. Code to the Pi, firmware to the ESP32

On the Pi (needs internet: phone hotspot):
```bash
cd ~/slam-robot && git pull && cd ros2_ws && colcon build --packages-select lidar_robot
sudo bash ~/slam-robot/scripts/setup_voice.sh install       # downloads the priyamvada voice, rest is skipped
```

Firmware v2.9 from the laptop (Arduino IDE, `firmware/robot_esp32s3/robot_esp32s3.ino`):
1. **Unplug the ESP32's 5 V wire from the buck first**, then the USB cable into the board's "USB" socket.
2. Upload (if it fails: hold BOOT, tap RST, release BOOT).
3. Serial Monitor 115200. Expect:
   - `INFO,READY,robot_esp32 v2.9 ...`
   - `INFO,battery sensor: INA226` (after part 1; `none` before it)
   - `INFO,dock sense on GPIO 1 ...`
   - Type `I` + Enter: `INFO,I2C,0x40,0x68` (0x40 = INA226, 0x68 = IMU)
4. USB out, 5 V wire back in.

---

## 1. INA226 battery monitor

### 1.1 Look at the shunt first

The big resistor on the board is the shunt. Read its marking:
- **R010** (0.01 Ω): good, wire it in series (1.3 A).
- **R100** (0.1 Ω, what most HW-899 boards ship with): it can measure only 0.8 A, and the motors pull more, so
  it would heat up. Until you have an R010, wire it **voltage only** (1.2 B). The voltage reading is still exact.

### 1.2 Wiring (battery unplugged)

Signal side, all versions:

| INA226 pin | goes to |
|---|---|
| VCC | ESP32 **3V3** |
| GND | ESP32 **GND** (= robot GND) |
| SDA | ESP32 **GPIO 11** (same wire as the IMU's SDA) |
| SCL | ESP32 **GPIO 12** (same wire as the IMU's SCL) |

**A: R010 shunt, series (voltage + current)**
```
battery + (after BMS) ── IN+ [shunt] IN− ── main switch ── fuse ── robot
VBS (bus voltage) ── IN−    (some boards join them already)
```

**B: R100 shunt, voltage only (for now)**
```
battery + (after BMS) ──┬── main switch ── robot     (battery current does NOT go through the board)
                        └── thin wire ── IN+, IN− and VBS joined together
```
Current reads 0 in B; that's fine. Later, with an R010, move to A.

**Remove the old battery divider (100 k / 22 k) from GPIO 1**: that pin is the dock sense now (part 2). Keep the
100 nF capacitor on GPIO 1.

### 1.3 Check
- Battery in, robot on. Serial Monitor / Pi log: `battery sensor: INA226`.
- App: the battery chip shows a voltage. Measure the battery with the multimeter: within **0.1 V**.
- Only A: drive forward. The current is **positive** while driving. If the app says "charging" while driving, set
  `BAT_CURRENT_SIGN -1` in the firmware.

---

## 2. The dock

### 2.1 How it works now

```
   DOCK (wall side)                         ROBOT (front)
   ┌────────────────────┐
   │   [ArUco marker]   │  ← centred, at camera height
   │                    │
   │ ═══ + plate ═════  │  14 cm  ◄── touches ──  ═ + plate ═  → 1 k → GPIO 1 → 100 R + 100 R → GND
   │ ═══ − plate ═════  │   8 cm  ◄── touches ──  ═ − plate ═  → robot GND
   └────────────────────┘
   12 V adapter: + → top plate, − → bottom plate         (heights are examples: same on both sides)
```

The robot creeps in at 3 cm/s. When 12 V shows on its plates for 1 s, it stops: **docked**. No charging yet, so the
adapter only has to deliver ~10 mA.

### 2.2 Plates from the geometry boxes

A metal box conducts **all over**, so one box can only be **one** contact.
- **Use two separate pieces:** two boxes, or the lid and the bottom of one box. Each piece is one plate: + on top,
  − below.
- Mount them on the cardboard / wood **with a gap of at least 2 cm**. Use tape or cardboard so they never touch
  each other or any screw.
- **Rub the paint off** where the robot touches, and where the wire is fixed (or use a screw + ring terminal there).
- **Continuity check:** multimeter on beep, from the wire end to the rubbed spot. It must beep. Then from the +
  plate to the − plate: it must **not** beep.
- **Dock plates wide (25 cm or more), robot plates narrower (~10 cm):** sideways error of a few cm still touches.

Robot side: two plates at the same heights on the front, **insulated from the metal deck**. Use cardboard,
acrylic or a plywood strip between them and the deck. Strips of the box, or thick foil tape on a stiff card, work.

### 2.3 No springs: give the dock plates some travel

The plates must press, not just touch. Without springs:
- **Foam behind the dock plates (easiest):** packing foam, a sponge, or 2–3 layers of thick double-sided foam tape
  between the plate and the dock wall. The plate gives ~5 mm when the robot pushes and presses back.
- **Bent leaf:** if the metal is thin enough, bend the plate into a shallow curve (a "C" lying on its side), fixed
  at the top edge only. It flexes like a spring.
- **Pen springs:** 2 ballpoint-pen springs behind each plate, in holes or short tubes, the plate taped/hinged at
  the top.
- **Rubber bands:** plate hinged at the top with tape, a rubber band pulling it outwards.

Fix the dock so it can't slide: tape it to the wall or put weight on the base.

### 2.4 Wiring and safety check (before GPIO 1)

1. Dock: **adapter + → top plate**, **adapter − → bottom plate**. Mark the top plate "+" with a pen.
2. Robot: **+ plate → 1 kΩ → GPIO 1 → 100 Ω → 100 Ω → GND**, **− plate → robot GND**. Use the big resistor for
   1 kΩ and the two small ones in series for 200 Ω.
3. **Before the wire goes into GPIO 1:** adapter on, robot pressed onto the dock by hand. Multimeter between the
   1 k / 100 Ω junction and GND: **about 2.0 V** (12 V × 200/1200). **Must be below 2.6 V.** Above that, check the
   resistor values.
4. Wire into GPIO 1.

### 2.5 Check the contact sense
```bash
journalctl -u robot -f | grep "dock contact"          # Pi: robot running
```
- Push the robot onto the dock: `dock contact: ON (12.x V)`.
- Pull it back: `dock contact: off`.
- Press lightly and wiggle it: it should stay ON. If it flickers, add more foam travel or clean the paint better.
- Robot off the dock: the plates read ~0 V (`DCK,0,...` in the Serial Monitor).

### 2.6 Marker and measurements
1. Stick the **ArUco ID 0, 80 mm** marker (the PDF from 17 Nov) on the dock, **centred left-right on the plates**,
   at the height of the robot's camera, flat and not bent.
2. Push the robot onto the dock so the plates touch. Measure:
   - `dock_contact_dist_m`: from the wheel axle (base_link) straight forward to the dock face (the marker plane).
   - `camera_x`, `camera_z`: the camera lens forward of the axle and its height above the floor.
3. Put them in `ros2_ws/src/lidar_robot/config/robot_params.yaml`, then
   `colcon build --packages-select lidar_robot && sudo systemctl restart robot`.

### 2.7 Teach the dock and try it
1. Robot on the dock (plates touching, contact ON). App → **Go → Charging → "Robot is on the dock: save it here"**.
2. Drive it ~1 m away, roughly facing the dock. **Charge now**.
3. Watch: align on the marker → creep → "docked: on the contacts". Do it **5 times** from different starts.
   Write down each try: docked? How many tries did the robot need? Sideways offset on the plates (cm)?
4. If it stops short with `DOCK_NO_CONTACT`: increase `dock_contact_dist_m` by 2 cm. If it pushes hard and stalls:
   decrease it.

---

## 3. Voice

```bash
sudo systemctl restart robot
```
Kiosk / app Ask:
- An English question: the answer is in the English woman's voice (lessac).
- A Hindi question, e.g. "HOD kaun hain?": the answer is in the Hindi woman's voice (priyamvada), with female verb
  forms.
- Too fast or too slow: `speech_rate` in `robot_params.yaml` (1.15 now; higher = slower).

---

## 4. Laptop mapping (if you have the USB-Ethernet adapter)

`docs/laptop_slam.md`: on the laptop run `setup_laptop_link.sh`, `ssh-copy-id arpit@169.254.1.2` and
`laptop_slam.py --check`. On the Pi, add `slam_host:=laptop` to ROBOT_ARGS in `~/robot.env`. Then map one corridor
first, Freeze, check the copy reached the Pi (`ls -lt ~/maps | head`), then the department.

---

## Results

| Part | Result | Notes |
|---|---|---|
| 0 firmware v2.9, I2C scan | | |
| 1 INA226 voltage vs multimeter | app ___ V / meter ___ V | shunt R010 / R100 |
| 1 current sign while driving (A only) | | |
| 2.4 GPIO 1 junction voltage | ___ V | |
| 2.5 contact ON/off clean? | | |
| 2.6 dock_contact_dist_m / camera_x / camera_z | | |
| 2.7 docking 5 tries | ___ / 5 docked | offsets: |
| 3 voices English / Hindi OK? | | |
| 4 laptop mapping | | |
