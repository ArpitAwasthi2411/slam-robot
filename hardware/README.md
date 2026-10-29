# Hardware

## Bill of materials

| Part | Qty | Notes |
|---|---|---|
| Raspberry Pi 4 (4 GB) + 32 GB microSD | 1 | Ubuntu 22.04 server, ROS 2 Humble |
| ESP32-S3-DevKitC-1 (or classic ESP32 DevKit) | 1 | S3: GPIO UART to the Pi (pins 6/8/10). Classic: USB, see pin map + udev notes |
| Slamtec RPLiDAR A1M8 | 1 | CP2102 USB adapter → `/dev/rplidar` |
| DC gear motor with quadrature encoder | 2 | ~4970 ticks/wheel-rev with 2× decoding (measured, `TICKS_PER_REV`) |
| BTS7960 (or similar RPWM/LPWM) motor driver | 2 | |
| Wheels Ø125 mm | 2 | track (wheel centre to centre) 300 mm |
| Caster wheel | 1 | |
| FlySky FS-i6 transmitter + FS-iA6 receiver | 1 | CH1 left/right, CH2 forward/back |
| Battery + 5 V buck converter for the Pi | 1 | fill in model / capacity |
| USB-ethernet adapter (laptop) + cable | 1 | direct link 169.254.1.1 ↔ 169.254.1.2 |

## Pin map

Each board has its own sketch: ESP32-S3 → `firmware/robot_esp32s3/` (v2.6), classic ESP32 →
`firmware/robot_esp32_classic/` (v2.4). The numbers below are the sketches' `#define`s; every wire with both
ends: [CONNECTIONS_esp32s3.md](CONNECTIONS_esp32s3.md) (S3), [wiring_classic_esp32.md](wiring_classic_esp32.md) (classic).

| Function | ESP32-S3 GPIO (v2.6) | Classic ESP32 GPIO (v2.4) | Connects to |
|---|---|---|---|
| RC steering (left/right stick) | 10 | 35 (input-only) | receiver CH1 signal |
| RC throttle (forward/back stick) | 9 | 34 (input-only) | receiver CH2 signal |
| Left motor RPWM / LPWM | 4 / 5 | 25 / 26 | left BTS7960 |
| Right motor RPWM / LPWM | 6 / 7 | 32 / 33 | right BTS7960 |
| Left encoder A / B | 15 / 16 | 18 / 19 | left motor encoder (3.3 V on the S3) |
| Right encoder A / B | 17 / 18 | 4 / 13 | right motor encoder |
| Link to the Pi | UART: TX 47 → Pi pin 10, RX 8 ← Pi pin 8 → `/dev/ttyAMA0` | USB-UART chip → `/dev/ttyUSB*` | `/dev/esp32` with udev |
| IMU + INA226 SDA / SCL | 11 / 12 | 21 / 22 | MPU-6050 (GY-521), INA226 |
| E-stop sense | 42 | 5 | mushroom NO contact to GND |
| Ultrasonic TRIG L/C/R | 13 / 14 / 21 | 14 / 16 / 17 | HC-SR04 TRIG (direct) |
| Ultrasonic ECHO L/C/R | 39 / 40 / 41 | 23 / 27 / 15 | via level shifter (5 V → 3.3 V) |
| Battery divider (100k/22k) | 1 | 36 | only without INA226, `USE_BAT_ADC 1` |
| Bumpers | 2 | 39 (+10k pull-up) | `USE_BUMPER 1` |
| On-board RGB LED | 48 (38 on DevKitC-1 v1.1) | — | status colours (S3 v2.5+) |

Wiring, mounting and tests for the IMU, ultrasonics and e-stop: **[sensors.md](sensors.md)**.
Full classic-ESP32 connection list: **[wiring_classic_esp32.md](wiring_classic_esp32.md)** (sketch: `firmware/robot_esp32_classic/`).

**Classic ESP32 — pins you must not use:** 6–11 (wired to the flash chip: using them crashes the
board, and the S3 map uses 6, 7, 9, 10), 1/3 (the USB serial to the Pi), and 0/2/5/12/15
(boot strapping: a motor driver or encoder pulling them at power-up can stop the board booting).
34–39 are input-only with no internal pull-ups.

Remember: all grounds common (ESP32, drivers, receiver, encoders, sensors). Encoders powered at
3.3 V, or level-shifted if they need 5 V. Driver R_EN/L_EN go to 5 V **through the e-stop's NC
contact** with a 10 kΩ pull-down (see sensors.md).

## Frames and dimensions

```
            +x (forward)
               ^
               |      laser at (laser_x, laser_y, laser_z) — measure & set in the launch
        L wheel|R wheel
     o---------+---------o      base_link = midpoint between the wheel contact points
     <-- 0.30 m -->            +y = left, z up
```

## Folders
- `cad/`: export **STEP** (and STL for printed parts) from SolidWorks, not only `.SLDPRT`, so
  anyone can open them. Keep files under ~50 MB.
- `photos/`: build photos, wiring close-ups (great for the README and report).
- `datasheets/`: put links in a `links.md` rather than committing vendor PDFs.
