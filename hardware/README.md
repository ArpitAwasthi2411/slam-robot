# Hardware

## Bill of materials

| Part | Qty | Notes |
|---|---|---|
| Raspberry Pi 4 (4 GB) + 32 GB microSD | 1 | Ubuntu 22.04 server, ROS 2 Humble |
| ESP32-S3 dev board | 1 | native USB port to the Pi |
| Slamtec RPLiDAR A1M8 | 1 | CP2102 USB adapter → `/dev/rplidar` |
| DC gear motor with quadrature encoder | 2 | 4740 ticks/wheel-rev with 2× decoding |
| BTS7960 (or similar RPWM/LPWM) motor driver | 2 | |
| Wheels Ø125 mm | 2 | track (wheel centre to centre) 300 mm |
| Caster wheel | 1 | |
| FlySky FS-i6 transmitter + FS-iA6 receiver | 1 | CH1 steering, CH2 throttle |
| Battery + 5 V buck converter for the Pi | 1 | fill in model / capacity |
| USB-ethernet adapter (laptop) + cable | 1 | direct link 169.254.1.1 ↔ 169.254.1.2 |

## ESP32-S3 pin map

| Function | GPIO | Connects to |
|---|---|---|
| RC CH1 steering | 4 | FS-iA6 CH1 signal |
| RC CH2 throttle | 7 | FS-iA6 CH2 signal |
| Left motor RPWM (forward) | 5 | left driver RPWM |
| Left motor LPWM (reverse) | 6 | left driver LPWM |
| Right motor RPWM (forward) | 9 | right driver RPWM |
| Right motor LPWM (reverse) | 10 | right driver LPWM |
| Left encoder A / B | 15 / 16 | left motor encoder |
| Right encoder A / B | 17 / 18 | right motor encoder |
| USB (native) | — | Raspberry Pi USB → `/dev/esp32` (`/dev/ttyACM0`) |

Remember: all grounds common (ESP32, drivers, receiver, encoders). Encoders powered at 3.3 V,
or level-shifted if they need 5 V. Driver R_EN/L_EN tied high.

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
