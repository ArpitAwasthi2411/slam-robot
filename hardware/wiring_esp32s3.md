# Wiring: ESP32-S3 (firmware `firmware/robot_esp32s3`, v2.5)

Board: **ESP32-S3-DevKitC-1** (N8, N8R8, N16R8 all fine). Flash with Arduino IDE → *ESP32S3 Dev Module*,
**USB Mode: Hardware CDC and JTAG**, **USB CDC On Boot: Enabled**, using the socket labelled **USB**
(not "COM"/"UART"). The Pi then sees it as **/dev/ttyACM0** and the LiDAR stays **/dev/ttyUSB0**:
they can no longer be swapped.

**Pins never used:** 0, 3, 45, 46 (boot straps) · 19, 20 (USB) · 26–32 (flash) · 33–37 (PSRAM on R8 boards) ·
43, 44 (UART0, kept for debugging) · 48 or 38 (on-board RGB LED).

## 1. Pin table

| GPIO | Connects to | Notes |
|---|---|---|
| **4** | Left BTS7960 **RPWM** | forward |
| **5** | Left BTS7960 **LPWM** | reverse |
| **6** | Right BTS7960 **RPWM** | forward |
| **7** | Right BTS7960 **LPWM** | reverse |
| **15** | Left encoder **A** | encoder VCC **3.3 V** |
| **16** | Left encoder **B** | |
| **17** | Right encoder **A** | |
| **18** | Right encoder **B** | |
| **9** | RC receiver **CH2** (forward/back) | |
| **10** | RC receiver **CH1** (left/right) | |
| **11** | MPU-6050 **SDA** + INA226 **SDA** | shared I2C bus |
| **12** | MPU-6050 **SCL** + INA226 **SCL** | |
| **13** | HC-SR04 left **TRIG** | direct (3.3 V is enough) |
| **14** | HC-SR04 centre **TRIG** | |
| **21** | HC-SR04 right **TRIG** | |
| **39** | level shifter **LV1** ← HV1 ← left **ECHO** | ECHO is 5 V: must be shifted |
| **40** | level shifter **LV2** ← HV2 ← centre **ECHO** | |
| **41** | level shifter **LV3** ← HV3 ← right **ECHO** | |
| **42** | E-stop **NO** contact (other side → GND) | pressed = LOW |
| **2** | Bumper switches (NO, all in parallel) → GND | internal pull-up, `USE_BUMPER 1` |
| **1** | Battery divider midpoint | `USE_BAT_ADC 1` |
| **48** | on-board RGB LED | already on the board (38 on DevKitC-1 v1.1: set `RGB_PIN`) |
| **3V3** | encoder VCC, MPU VCC, INA226 VCC, level shifter **LV** | |
| **5V** | from the 5 V buck (only if the board is *not* powered by the Pi's USB, see 3) | |
| **GND** | common ground: everything | |
| **USB** socket | Pi USB | power + data, /dev/ttyACM0 |

## 2. Do I need level shifters?

The rule: **an ESP32-S3 pin must never see more than 3.3 V.** It has no 5 V tolerance.

| Signal | Direction | Level shifter? |
|---|---|---|
| Encoders A/B | into the ESP32 | **No, if the encoders are powered from 3.3 V** (recommended; Hall encoders on these motors work at 3.3 V, then their outputs are 3.3 V). **Yes** if you power them from 5 V: use a BSS138 4-channel shifter (or a 10k/20k divider per line). |
| HC-SR04 ECHO | into the ESP32 | **Yes**: the sensor needs 5 V and its ECHO is 5 V. 3 channels of the BSS138 shifter. |
| HC-SR04 TRIG | out of the ESP32 | No: 3.3 V triggers it. |
| BTS7960 RPWM/LPWM | out of the ESP32 | No (same as on the classic ESP32, which works). If a driver ever acts erratic, a 74AHCT125 buffer is the clean fix. |
| RC receiver CH1/CH2 | into the ESP32 | Usually no: FS-iA6B signals are ~3.3 V. Measure once with a multimeter (signal pin to GND, transmitter on): above 3.6 V → 10k/20k divider. |
| MPU-6050, INA226 | I2C | No: power them from 3.3 V. |
| E-stop / bumper | into the ESP32 | No: switches to GND with pull-ups. |

**Check before connecting encoders:** with the encoder powered and the wheel turned slowly by hand,
measure A to GND. It must swing between 0 and **≤ 3.3 V**.

## 3. Power

```
3S battery + ── main switch ── fuse ──┬── BTS7960 B+ (both)   [1000 µF 25 V across B+/B−, stripe to B−]
                                      ├── 5 V buck ──┬── Pi (USB-C, 5.1 V 3 A)
                                      │              ├── BTS7960 VCC (logic 5 V), HC-SR04 VCC, RC receiver
                                      │              └── level shifter HV
                                      └── 100k ── GPIO 1 ── 22k ── GND   (battery %)
3S battery − ── common GND (buck, drivers, ESP32, Pi, sensors)
ESP32-S3: powered by the Pi's USB  ->  1000 µF 25 V across its 5V/GND pins (stripe to GND)
Each motor: 100 nF soldered across its two terminals
```
- All four BTS7960 **EN** pins: 5 V → e-stop **NC** contact → R_EN/L_EN (+10k to GND), unchanged.
- Do **not** connect the buck's 5 V to the ESP32's 5V pin *and* the Pi USB at the same time.

## 4. Pi side

```bash
# robot.env
ROBOT_ARGS=esp32_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0
# optional fixed names (/dev/esp32, /dev/rplidar), the S3 has its own USB ID so this now just works:
sudo bash ~/slam-robot/scripts/install_udev.sh
```

## 5. First power-up checks (in order)

1. Only USB connected → flash → Serial Monitor 115200 → `INFO,READY,robot_esp32 v2.5 ESP32-S3 reset=POWERON`.
   The LED **breathes blue** (waiting for the Pi). If it stays dark: `RGB_PIN 38`.
2. Type `?` → config lines. Type `L,255,0,255,2000` → LED magenta for 2 s.
3. Encoders (wheels by hand): `ODM,` values change; forward roll → both positive (else flip `*_ENC_DIR`).
4. Motors (robot on blocks): `P,120,120` → both wheels forward (else flip `*_MOTOR_DIR`), `S` stops.
   The LED turns **rainbow** while the Pi drives.
5. RC: sticks move the wheels, LED **magenta**. `E` → LED **flashes red**, `R` releases.
6. IMU: `?` shows `imu=ok`. Ultrasonics: `US,` lines, hand at 30 cm ≈ 300.
7. Plug into the Pi, `sudo systemctl restart robot`, app → Motor board ≈ 50 Hz. LED **teal** when idle.

Your calibrated values from the classic sketch are already in this one (`*_MOTOR_DIR`, `*_ENC_DIR`,
RC `STEERING_*` / `THROTTLE_*`, `TICKS_PER_REV`, speed-loop defaults). Re-check directions with
steps 3–4: they flip if a motor or encoder is plugged in the other way round.
