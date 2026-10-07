# Hardware

## Bill of materials

| Part | Qty | Notes |
|---|---|---|
| Raspberry Pi 4 (4 GB) + 32 GB microSD | 1 | Ubuntu 22.04 server, ROS 2 Humble |
| ESP32-S3 dev board (or classic ESP32 DevKit) | 1 | S3: native USB port. Classic: see pin map + udev notes |
| Slamtec RPLiDAR A1M8 | 1 | CP2102 USB adapter → `/dev/rplidar` |
| DC gear motor with quadrature encoder | 2 | 4740 ticks/wheel-rev with 2× decoding |
| BTS7960 (or similar RPWM/LPWM) motor driver | 2 | |
| Wheels Ø125 mm | 2 | track (wheel centre to centre) 300 mm |
| Caster wheel | 1 | |
| FlySky FS-i6 transmitter + FS-iA6 receiver | 1 | CH1 steering, CH2 throttle |
| Battery + 5 V buck converter for the Pi | 1 | fill in model / capacity |
| USB-ethernet adapter (laptop) + cable | 1 | direct link 169.254.1.1 ↔ 169.254.1.2 |

## Pin map

The firmware picks the map automatically from the board selected in the Arduino IDE.

| Function | ESP32-S3 GPIO | Classic ESP32 GPIO | Connects to |
|---|---|---|---|
| RC CH1 steering | 4 | 34 (input-only) | FS-iA6 CH1 signal |
| RC CH2 throttle | 7 | 35 (input-only) | FS-iA6 CH2 signal |
| Left motor RPWM (forward) | 5 | 25 | left driver RPWM |
| Left motor LPWM (reverse) | 6 | 26 | left driver LPWM |
| Right motor RPWM (forward) | 9 | 32 | right driver RPWM |
| Right motor LPWM (reverse) | 10 | 33 | right driver LPWM |
| Left encoder A / B | 15 / 16 | 18 / 19 | left motor encoder |
| Right encoder A / B | 17 / 18 | 4 / 13 | right motor encoder |
| Link to the Pi | native USB → `/dev/ttyACM0` | USB-UART chip → `/dev/ttyUSB*` | Pi USB, `/dev/esp32` with udev |
| IMU SDA / SCL (v2.2) | 1 / 2 | 21 / 22 | MPU-6050 (GY-521) |
| E-stop sense (v2.2) | 41 | 5 | mushroom NO contact to GND |
| Ultrasonic TRIG L/C/R (v2.2) | 13 / 14 / 21 | 14 / 16 / 17 | HC-SR04 TRIG |
| Ultrasonic ECHO L/C/R (v2.2) | 38 / 39 / 40 | 23 / 27 / 15 | via level shifter (5 V → 3.3 V) |
| Free for later | — | 36 ADC (battery), 39 | |

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
