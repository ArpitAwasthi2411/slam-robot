# Adding the sensors: MPU-6050, 3× HC-SR04, mushroom e-stop

Do this **after** the encoder odometry works (deploy.md steps 1–8). Add one sensor at a time and
check it before moving on. Firmware v2.2 supports all three. Each can stay switched on even if it
isn't wired yet (`USE_IMU`, `USE_ULTRASONIC`, `USE_ESTOP_BUTTON` at the top of the sketch).

| Sensor | What it fixes |
|---|---|
| MPU-6050 gyro | Wheels slip when turning, so heading drifts. The gyro gives heading; wheels still give distance |
| 3× HC-SR04 | The LiDAR only sees one flat plane ~50 cm up. Ultrasonics see ledges, skirting, chair legs, bags |
| Mushroom e-stop | Cuts the motor drivers **in hardware**, whatever the software is doing, and tells the software |

## Pin map (classic ESP32, additions in bold)

| Function | GPIO | Notes |
|---|---|---|
| RC CH1 / CH2 | 34 / 35 | unchanged |
| Motors L RPWM/LPWM, R RPWM/LPWM | 25, 26, 32, 33 | unchanged |
| Encoders L A/B, R A/B | 18, 19, 4, 13 | unchanged |
| **IMU SDA / SCL** | **21 / 22** | I2C |
| **E-stop sense** | **5** | internal pull-up, pressed = LOW |
| **Ultrasonic TRIG L / C / R** | **14 / 16 / 17** | 3.3 V output is enough for HC-SR04 |
| **Ultrasonic ECHO L / C / R** | **23 / 27 / 15** | **5 V signal: must go through the level shifter** |
| Free for later | 36 (battery voltage ADC), 39 | |

(ESP32-S3 pins are in the sketch: SDA 1, SCL 2, e-stop 41, TRIG 13/14/21, ECHO 38/39/40.)

The new pins were chosen around the ESP32's boot quirks. Nothing new sits on 0, 2, 12 (they affect
booting/flashing) or on the flash pins 6–11. Echoes avoid 36/39, which glitch once the ADC is used
for battery monitoring.

## 1. Mushroom e-stop (do this first: it's safety)

Your LAY37 button is self-locking: press to stop, twist to release. Most have **1 NO + 1 NC** contacts.

```
                 BTS7960 #1 R_EN ──┐
 5 V (driver     BTS7960 #1 L_EN ──┤
 logic supply) ─[ NC contact ]──┬──┤ BTS7960 #2 R_EN
                                │  └ BTS7960 #2 L_EN
                               10 kΩ
                                │
                               GND          pressed -> NC opens -> EN pulled LOW -> both drivers OFF

 ESP32 GPIO 5 ───[ NO contact ]─── GND      pressed -> NO closes -> GPIO 5 LOW -> firmware E-STOP
```
- Up to now the four EN pins were tied straight to 5 V. Move that wire so it goes **through the
  NC contact**, and add the **10 kΩ pull-down** so the EN line is definitely LOW when the contact opens.
- **Only an NC contact on your button?** Use it for the EN line as above, then sense the EN line
  instead: EN line ─10 kΩ─ GPIO 5 ─20 kΩ─ GND (divides 5 V down to 3.3 V). Same firmware setting.
- The button cuts the **drivers' enable**, not battery power. Keep the XT60 or main switch for that.
  Don't put the 10 A button in the motor power line: stall current is 20–40 A.

**Test (wheels off the ground):**
1. Drive slowly with RC and press the button. Wheels stop instantly, and the Serial Monitor shows
   `INFO,ESTOP button pressed` with ODM mode `3`.
2. RC sticks now do nothing (hardware and software both block).
3. Twist the button out. It **stays** stopped: the dashboard shows `ESP32 E-STOP`. Press
   **Release E-stop** in the dashboard (or reset the ESP32), then press **Resume** for missions.

## 2. MPU-6050 (GY-521)

| GY-521 pin | Connect to |
|---|---|
| VCC | 3.3 V (or 5 V, the GY-521 has its own regulator) |
| GND | GND |
| SCL | GPIO 22 |
| SDA | GPIO 21 |
| AD0 | GND or leave open (address 0x68) |
| INT, XDA, XCL | not used |

**Mounting matters more than wiring:**
- Flat, rigid, near the middle between the wheels. Double-sided foam tape is fine, but nothing that wobbles.
- Chip **X arrow pointing forward**, component side up.
- Keep the I2C wires short (under 20 cm) and away from the motor wires.
- **Keep the robot still for 1 second after power-up**: the firmware measures the gyro offset then.
  If you bumped it, type `G` in the Serial Monitor (or reboot).

**Test:**
1. Serial Monitor: `?` → `INFO,sensors imu=ok(who=0x68 ...)`. Clones may report 0x70, 0x72 or 0x98, which is fine.
2. Turn the robot **left** by hand (counter-clockwise seen from above). The first number in the
   `IMU,...` lines must be **positive**. If it's negative, set `#define IMU_YAW_SIGN -1` and re-flash.
3. Dashboard → Status → **Heading from: gyro**.
4. Redo the 5-turn spin test (calibration.md §3). With the gyro, `turns` should read ~5.0 even if
   the wheel separation is slightly off. That's the point of it.

The Pi uses it automatically (`use_imu_yaw: true` in robot_params.yaml): heading from the gyro,
distance from the wheels. The raw data is also on `/imu` (sensor_msgs/Imu) for later use.

## 3. HC-SR04 ×3

HC-SR04 runs on **5 V** and its ECHO output is **5 V**, which can damage a 3.3 V ESP32 pin. Use the
4-channel **BSS138 level shifter** you ordered (3 channels for the echoes, 1 spare):

```
 Level shifter:  HV = 5 V    LV = 3.3 V    GND = GND (both sides)

 HC-SR04 left   : VCC 5V, GND, TRIG <- GPIO 14,  ECHO -> HV1 ... LV1 -> GPIO 23
 HC-SR04 centre : VCC 5V, GND, TRIG <- GPIO 16,  ECHO -> HV2 ... LV2 -> GPIO 27
 HC-SR04 right  : VCC 5V, GND, TRIG <- GPIO 17,  ECHO -> HV3 ... LV3 -> GPIO 15
```
No shifter? Use a divider per echo: ECHO ─1 kΩ─┬─ GPIO, and that GPIO node ─2 kΩ─ GND.

**Mounting** (the defaults the software assumes; change the launch args if yours differ):
- Front edge of the chassis, low: **5–8 cm above the floor** (`us_z:=0.06`), 22 cm ahead of the axle
  centre (`us_x:=0.22`).
- Centre sensor pointing straight ahead. Left and right sensors 12 cm to each side (`us_y:=0.12`),
  **turned 30° outward** (`us_side_deg:=30`).
- Tilt them up 2–3° if they "see" the floor (random short readings on a smooth floor).
- Nothing in front of the transducers within a ~15° cone (no screws or bumper edges).

**Test:**
1. Serial Monitor shows `US,l,c,r` lines in mm (0 = nothing within 4 m).
2. Hold your hand 30 cm in front of each sensor in turn: the matching column reads ~300. Check left
   and right aren't swapped.
3. Dashboard → map: three cones in front of the robot turn amber, then red, as you get closer.
   RViz shows them too (`Ultrasonic left/center/right` displays).

**How the software uses them:**
- **Navigator / goal controller**: anything an ultrasonic sees inside the robot's width counts as
  an obstacle in front. The robot slows from 80 cm and stops at 38 cm (from the robot centre). The
  points are also added to the obstacles used when it re-plans around a blockage.
- **Firmware safety net**: in AUTO mode, if any sensor reads under **15 cm**, the ESP32 removes the
  forward part of the command on its own (turning in place still works), even if the Pi has hung.
  RC driving is never blocked: the human is in control.

## After all three: what changes on the Pi
Nothing to configure if you mounted things as above. `bringup.launch.py` publishes the sensor
positions as static TFs (`us_left`, `us_center`, `us_right`, `imu_link`) from the launch args, so
RViz, the navigator and the dashboard all agree where the sensors are. Example with other mount values:
```bash
ros2 launch lidar_robot bringup.launch.py us_x:=0.20 us_y:=0.15 us_side_deg:=25 us_z:=0.07
```
