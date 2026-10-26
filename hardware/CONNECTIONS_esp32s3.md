# Pathik: every connection (ESP32-S3 build, firmware v2.5)

Each wire is listed by **where it starts → where it ends**. The pin numbers match
`firmware/robot_esp32s3/robot_esp32s3.ino` exactly; don't change one without the other.
Tick each line as you wire it. **Wire everything with the battery switch OFF and the ESP32 unplugged.**

Colour suggestion: red = battery/5 V, orange = 3.3 V, black = GND, yellow = PWM, green/blue = encoder A/B,
white = sensor signals.

---

## 1. Power (thick wires, 16–18 AWG)

| # | From | To | Note |
|---|---|---|---|
| P1 | Battery **+** (XT60) | Fuse → main switch | |
| P2 | Main switch out | Left BTS7960 **B+** and Right BTS7960 **B+** | |
| P3 | Main switch out | XL4015 buck **IN+** | buck set to **5.1 V** (measure before connecting the Pi) |
| P4 | Battery **−** | Left **B−**, Right **B−**, buck **IN−** | this is the common GND |
| P5 | Left motor wires | Left BTS7960 **M+ / M−** | |
| P6 | Right motor wires | Right BTS7960 **M+ / M−** | |
| P7 | Buck **OUT+ / OUT−** | Pi 4 **USB-C** (5 V cable) | `vcgencmd get_throttled` must say `0x0` |
| P8 | Buck **OUT+** (5 V rail) | 5 V bus: BTS7960 VCC ×2, HC-SR04 VCC ×3, RC receiver +, level shifter **HV**, speaker USB power | |
| P9 | Buck **OUT−** | GND bus | GND bus also joins ESP32 GND |

### Capacitors (stripe / short leg = −)
| # | Where | Part |
|---|---|---|
| C1 | Left BTS7960 B+ ↔ B− | 1000 µF 25 V |
| C2 | Right BTS7960 B+ ↔ B− | 1000 µF 25 V |
| C3 | ESP32-S3 **5V** pin ↔ **GND** pin | 1000 µF 25 V |
| C4 | Across each motor's 2 terminals (soldered on the motor) | 100 nF ×2 (ceramic, no polarity) |
| C5 | ESP32 GPIO 1 ↔ GND (across the 22 kΩ) | 100 nF |

## 2. Motor drivers (BTS7960 ×2)

| # | From | To |
|---|---|---|
| M1 | ESP32 **GPIO 4** | Left BTS7960 **RPWM** |
| M2 | ESP32 **GPIO 5** | Left BTS7960 **LPWM** |
| M3 | ESP32 **GPIO 6** | Right BTS7960 **RPWM** |
| M4 | ESP32 **GPIO 7** | Right BTS7960 **LPWM** |
| M5 | 5 V bus → e-stop **NC** contact → | **R_EN + L_EN** of both drivers (all 4 EN pins joined), plus 10 kΩ from the EN line to GND |
| M6 | Both drivers **GND** (logic) | GND bus |
| M7 | Both drivers **VCC** (logic) | 5 V bus |
| — | R_IS / L_IS | not connected |

## 3. Encoders (one 6-pin connector per motor)

**Power the encoders from 3.3 V** → no level shifter. Before connecting A/B to the ESP32: power the encoder,
turn the wheel slowly, measure A→GND. It must stay **≤ 3.3 V**.

| # | From | To |
|---|---|---|
| E1 | Left encoder **VCC** | ESP32 **3V3** |
| E2 | Left encoder **GND** | GND |
| E3 | Left encoder **A** | ESP32 **GPIO 15** |
| E4 | Left encoder **B** | ESP32 **GPIO 16** |
| E5 | Right encoder **VCC** | ESP32 **3V3** |
| E6 | Right encoder **GND** | GND |
| E7 | Right encoder **A** | ESP32 **GPIO 17** |
| E8 | Right encoder **B** | ESP32 **GPIO 18** |

Keep encoder wires away from motor wires (twist each encoder bundle; don't run them alongside motor leads).

## 4. RC receiver (FlySky)

| # | From | To |
|---|---|---|
| R1 | Receiver **CH1 signal** (left/right) | ESP32 **GPIO 10** |
| R2 | Receiver **CH2 signal** (forward/back) | ESP32 **GPIO 9** |
| R3 | Receiver **+** | 5 V bus |
| R4 | Receiver **−** | GND |
Measure CH1/CH2 signal → GND once with the transmitter on: it must be ≤ 3.3 V (FS-iA6B is ~3.3 V).
If above 3.6 V, put a 10 kΩ/20 kΩ divider in each line.

## 5. IMU (MPU-6050 / GY-521) on I2C

| # | From | To |
|---|---|---|
| I1 | MPU **VCC** | ESP32 **3V3** |
| I2 | MPU **GND** | GND |
| I3 | MPU **SDA** | ESP32 **GPIO 11** |
| I4 | MPU **SCL** | ESP32 **GPIO 12** |
| — | AD0, INT, XDA, XCL | not connected |
(INA226, if added later, goes on the same SDA 11 / SCL 12 and 3V3.)

## 6. Ultrasonics (HC-SR04 ×3) + level shifter (BSS138, 4-channel)

Level shifter: **LV** → ESP32 3V3, **HV** → 5 V bus, **both GND** → GND.

| # | From | To |
|---|---|---|
| U1 | Left HC-SR04 **TRIG** | ESP32 **GPIO 13** (direct) |
| U2 | Centre HC-SR04 **TRIG** | ESP32 **GPIO 14** (direct) |
| U3 | Right HC-SR04 **TRIG** | ESP32 **GPIO 21** (direct) |
| U4 | Left **ECHO** → shifter **HV1**, **LV1** → | ESP32 **GPIO 39** |
| U5 | Centre **ECHO** → shifter **HV2**, **LV2** → | ESP32 **GPIO 40** |
| U6 | Right **ECHO** → shifter **HV3**, **LV3** → | ESP32 **GPIO 41** |
| U7 | All HC-SR04 **VCC** / **GND** | 5 V bus / GND |
**Never** connect an ECHO straight to the ESP32-S3 (5 V kills the pin).

## 7. E-stop, bumper, battery sensing

| # | From | To | Note |
|---|---|---|---|
| S1 | E-stop **NO** contact, one side | ESP32 **GPIO 42** | the NC contact is already used for EN (M5) |
| S2 | E-stop **NO** contact, other side | GND | pressed = LOW |
| S3 | Bumper switches (NO, in parallel) | ESP32 **GPIO 2** and GND | only if fitted; then set `USE_BUMPER 1` |
| S4 | Battery **+** (after the switch) → **100 kΩ** → | ESP32 **GPIO 1** | |
| S5 | ESP32 **GPIO 1** → **22 kΩ** → | GND | full battery 12.6 V → 2.27 V at the pin. Then set `USE_BAT_ADC 1` |
**Check S4/S5 with the multimeter before plugging the wire into GPIO 1:** the midpoint must read < 2.5 V.

## 8. ESP32-S3 ↔ Pi

| # | From | To |
|---|---|---|
| X1 | ESP32-S3 USB socket labelled **"USB"** (not "COM"/"UART") | any Pi USB port (data micro/USB-C cable) |
| X2 | ESP32-S3 **GND** | GND bus (common ground with drivers and sensors) |
| — | ESP32 **5V** pin | **not** connected to the buck (it gets 5 V from the Pi's USB). Only C3 goes there |
Pi sees it as **/dev/ttyACM0**. The ESP32 on-board RGB LED (GPIO 48) needs no wire.

## 9. Pi peripherals

| # | From | To | Note |
|---|---|---|---|
| L1 | RPLiDAR A1 USB adapter | Pi USB (blue USB 3 port) | `/dev/ttyUSB0` |
| A1 | USB sound card | Pi USB | `arecord -l` / `aplay -l` show it |
| A2 | Lapel mic (4-pole) → headset splitter **mic** plug → | sound card **mic** (pink) | |
| A3 | Speaker 3.5 mm audio plug | sound card **headphone** (green) | |
| A4 | Speaker USB **power** plug | 5 V bus / buck USB output, **not** the Pi | the Pi's USB budget is 1.2 A total |
| K1 | Camera Module 3 Wide ribbon | Pi **CAMERA** CSI connector (between HDMI and audio jack) | Pi **OFF** while connecting. Blue tab faces the USB/Ethernet ports. Lift the latch, push in, close the latch |
| N1 | Ethernet cable | laptop | Pi 169.254.1.2 / laptop 169.254.1.1 |

## 10. Pins left free (don't use)
0, 3, 45, 46 (boot straps) · 19, 20 (USB) · 26–32 (flash) · 33–37 (PSRAM) · 43, 44 (debug UART) · 38/48 (LED).
Free for later (e.g. push-to-talk button / dock): 8, 47.
