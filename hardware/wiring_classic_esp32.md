# Complete wiring — classic ESP32 (ESP32-WROOM-32 DevKit), firmware v2.2

Sketch: `firmware/robot_esp32_classic/robot_esp32_classic.ino` (board: **ESP32 Dev Module**).

## 1. ESP32 pin-by-pin

| ESP32 pin | Goes to | Notes |
|---|---|---|
| **34** | FS-iA6 **CH1** signal (steering) | input-only pin |
| **35** | FS-iA6 **CH2** signal (throttle) | input-only pin |
| **25** | Left BTS7960 **RPWM** | forward |
| **26** | Left BTS7960 **LPWM** | reverse |
| **32** | Right BTS7960 **RPWM** | forward |
| **33** | Right BTS7960 **LPWM** | reverse |
| **18** | Left encoder **A** | |
| **19** | Left encoder **B** | |
| **4** | Right encoder **A** | |
| **13** | Right encoder **B** | not 12! |
| **21** | MPU-6050 **SDA** | |
| **22** | MPU-6050 **SCL** | |
| **5** | E-stop **NO** contact (other side of NO → GND) | pressed = LOW |
| **14** | HC-SR04 left **TRIG** | direct, 3.3 V is enough |
| **16** | HC-SR04 centre **TRIG** | |
| **17** | HC-SR04 right **TRIG** | |
| **23** | level shifter **LV1** ← HV1 ← left **ECHO** | never direct (5 V) |
| **27** | level shifter **LV2** ← HV2 ← centre **ECHO** | |
| **15** | level shifter **LV3** ← HV3 ← right **ECHO** | |
| **3V3** | MPU VCC, encoder VCC, level shifter **LV** | |
| **GND** | common ground (everything) | |
| **USB** | Raspberry Pi USB | power + serial → `/dev/ttyUSB0` / `/dev/esp32` |

**Leave empty:** 6, 7, 8, 9, 10, 11 (flash), 1, 3 (TX/RX to the Pi), 0, 2, 12 (boot). Free for later: 36 (battery ADC), 39.

## 2. Motor drivers (BTS7960 ×2)

| BTS7960 pin | Left driver | Right driver |
|---|---|---|
| RPWM | ESP32 25 | ESP32 32 |
| LPWM | ESP32 26 | ESP32 33 |
| R_EN + L_EN | **EN line** (from the e-stop, below) | **EN line** |
| VCC (logic) | 5 V | 5 V |
| GND (logic) | GND | GND |
| R_IS / L_IS | not connected | not connected |
| B+ / B− | battery + / − (via main switch / fuse) | battery + / − |
| M+ / M− | left motor | right motor |

## 3. Mushroom e-stop (1 NO + 1 NC contact)

```
5 V ──► NC contact ──► EN line ──┬──► Left  R_EN, L_EN
                                 ├──► Right R_EN, L_EN
                                 └──► 10 kΩ ──► GND

GPIO 5 ──► NO contact ──► GND
```
- Not pressed: NC closed → EN = 5 V → drivers on. NO open → GPIO 5 HIGH (internal pull-up).
- Pressed: NC opens → EN pulled to 0 V → drivers off in hardware. NO closes → GPIO 5 LOW → firmware latches E-STOP.
- Button with **only NC**: EN line ─10 kΩ─► GPIO 5 ─20 kΩ─► GND (instead of the NO wiring).
- Release: twist the button out, then **Release E-stop** in the dashboard (or `R` in the Serial Monitor).

## 4. Encoders (each motor)

| Encoder wire | Connect |
|---|---|
| VCC | 3.3 V (if it needs 5 V, its A/B must be level-shifted) |
| GND | GND |
| A / B | left 18 / 19, right 4 / 13 |

## 5. MPU-6050 (GY-521)

VCC → 3.3 V, GND → GND, SDA → 21, SCL → 22, AD0 → GND. INT/XDA/XCL unused. Mount flat, X arrow forward.

## 6. HC-SR04 ×3 + BSS138 level shifter

| | Left | Centre | Right |
|---|---|---|---|
| VCC | 5 V | 5 V | 5 V |
| GND | GND | GND | GND |
| TRIG | 14 | 16 | 17 |
| ECHO | HV1 → LV1 → 23 | HV2 → LV2 → 27 | HV3 → LV3 → 15 |

Shifter: HV = 5 V, LV = 3.3 V (ESP32), both GNDs to GND.
No shifter: ECHO ─1 kΩ─┬─ GPIO, and that node ─2 kΩ─ GND.

## 7. RC receiver (FS-iA6)

CH1 signal → 34, CH2 signal → 35, receiver + → 5 V, receiver − → GND.

## 8. Power and ground

- Battery → main switch/fuse → BTS7960 B+/B− (motor power) and → 5 V buck → Pi.
- ESP32 is powered by the Pi's USB.
- 5 V for the driver logic, e-stop EN line, HC-SR04 and receiver: from the 5 V buck.
- **All grounds common:** battery −, buck GND, ESP32 GND, both drivers, receiver, encoders, sensors.

## 9. Order of work

1. Unplug everything except USB → flash the sketch → Serial Monitor 115200 → `?` shows `INFO,...`.
2. Connect motors, drivers, encoders, RC (already done) → test.
3. E-stop → test (wheels off the ground).
4. MPU-6050 → `?` shows `imu=ok`, turn left by hand → `IMU,` first value positive.
5. Ultrasonics → `US,l,c,r` lines, hand at 30 cm reads ~300.
