# Battery monitor (INA226) and the charging dock

Firmware v2.4 reports the battery and the navigator uses it: it drives back to charge on its own
when the battery is low and checks that charging current starts.

> **Since Phase 4 the robot docks nose first**, guided by an ArUco marker on the dock: see
> [docs/docking.md](../docs/docking.md). The reverse-docking description below is the v2.4 way
> (`dock_direction: reverse`); the INA226 and contact wiring parts still apply.

## 1. INA226 battery monitor (do this first)

```
 3S pack +  ──[BMS]──●──── INA226 IN+   IN- ────●──── main switch ──── fuse ──── robot (BTS7960s, buck → Pi, ...)
                     │                          │
                     └── INA226 VBUS            └──── charge input (from the dock contacts, see 3)
 3S pack −  ──────────────────────────────────────── robot GND  (INA226 GND, ESP32 GND)

 INA226 VCC → ESP32 3.3V    SDA → GPIO 21    SCL → GPIO 22    (same bus as the MPU-6050)
 A0, A1 → GND  (address 0x40)
```

- **Swap the shunt.** Most INA226 boards ship with an `R100` (0.1 Ω) resistor: that reads only up to
  0.8 A and would cook at motor current. Replace it with an `R010` (0.01 Ω, 2512 size, ≥ 1 W):
  up to 8 A. `SHUNT_OHMS` in the firmware is already 0.01. (Or buy an INA226 module sold with R010.)
- The charge input joins on the **IN− side** (between the INA226 and the main switch). Charging
  current then flows backwards through the shunt, so it reads **negative** = charging. That is how
  the robot knows it is on the dock. Charging also works with the main switch off.
- After flashing v2.4: the app's battery chip appears at the top. If it says "charging" while
  driving, set `BAT_CURRENT_SIGN -1` in the firmware.
- No INA226 yet? A divider on GPIO 36 (battery+ → 100 k → GPIO 36 → 22 k → GND) gives the voltage
  only. Set `USE_BAT_ADC 1`. Without current the robot can't confirm docking, so it trusts the
  saved dock position instead.

Levels (in `config/robot_params.yaml`): below **25 %** it goes to charge after the current task;
below **12 %** it goes now and refuses new tasks. The voltage is averaged over ~20 s, so the
motors starting doesn't trigger a false "low battery".

## 2. How the robot docks (already in the software)

```
        wall
   ┃▓▓▓▓▓▓▓▓▓┃        1. drive to the approach point, 55 cm in front of the dock, facing away from it
   ┃ ╲ dock ╱ ┃        2. reverse slowly (5 → 3 cm/s), steering to stay on the dock's centre line
   ┃  ╲═══╱   ┃        3. stop when the INA226 sees charging current (stable for 1 s) → "Charging"
        ▲              4. no current after the full travel → "DOCK_NO_CONTACT" (check power / contacts)
     ┌──┴──┐           5. next task: drives straight off 30 cm first, then turns (so the back
     │robot│              never swings into the dock)
     └─────┘
```

Why backwards: your drive axle is at the **front** and the body hangs 37 cm behind it. Reversing
in puts the flat back face on the contacts, and the front wheels steer the back into the funnel
(like a forklift). The contacts go on the **back** of the robot.

To teach it: push the robot onto the dock by hand so the contacts touch, then **Go tab →
Charging → Robot is on the dock**. That saves the dock pose on the map. **Charge now** tests it.

## 3. Building the dock

### The idea

- **Robot:** two horizontal copper strips across the back face, one above the other.
  Horizontal strips don't care where the robot lands sideways (±10 cm still touches).
- **Dock:** two springy contacts at the same two heights, and a funnel that guides the back of the
  robot to the middle. The contacts are at different heights, so + and − can never swap.

```
  ROBOT BACK FACE (42 cm wide)                     DOCK, seen from the top
 ┌──────────────────────────────┐                 wall / heavy base
 │                              │            ┃━━━━━━━━━━━━━━━━━━━━━━━━━━━┃
 │  ════════════════════════  + │ 14 cm        ┃  ⌒⌒ spring contacts ⌒⌒   ┃
 │  ════════════════════════  − │  8 cm         ╲                        ╱   ← funnel wings,
 │                              │                ╲                      ╱      45°, 15 cm long
 └──○──────────────────────○────┘ floor           ╲   robot backs in   ╱
   (caster)            (caster)                      ↑
```

### Parts (roughly ₹1,000–1,500)

| Part | Use | Approx. |
|---|---|---|
| 18 mm plywood / MDF, ~55 × 30 cm back plate + 2 wings 15 × 30 cm + base 55 × 30 cm | the dock body | ₹200–300 (offcuts) |
| Copper or brass strip, 20 mm × 0.5 mm, ~60 cm | 2 strips on the robot (25 cm each) | ₹150 |
| 2 spring contacts: copper/phosphor-bronze leaf springs (bend strip into a "C"), or 10 A pogo / battery spring contacts | dock contacts | ₹100–200 |
| 3S Li-ion CC/CV charger, 12.6 V 2 A (already on your list) | power | ₹449 |
| Schottky diode SR560 (5 A, 60 V) or an "ideal diode" module | stops the battery feeding the exposed robot plates | ₹10–150 |
| Inline blade fuse 3 A + holder | short-circuit protection on the robot charge line | ₹50 |
| Microswitch with roller lever | dock contacts live only when a robot presses it | ₹30 |
| 14–16 AWG wire, ring terminals, screws, hot glue | wiring | ₹100 |
| Rubber feet / non-slip mat, or screw the dock to the wall | the dock mustn't slide when the robot pushes | ₹50 |

### Steps

1. **Measure your robot's back face**: width, and the height of a flat area (bumper or plate)
   between ~5 and 20 cm above the floor. Put the two strips there, 6 cm apart (centre to centre),
   on an insulating backing (acrylic, plywood, PCB blank). Strip ends rounded so they slide.
2. **Robot wiring:** `+ strip → 3 A fuse → Schottky (stripe towards the battery) → IN− side of the
   INA226` (see 1). `− strip → robot GND`. The diode means the strips are dead when undocked:
   a coin or a screwdriver across them can't short the battery.
3. **Dock back plate:** screw the two spring contacts at exactly the strip heights. They should
   stick out ~10 mm and compress ~5 mm when the robot is on. Add the funnel wings at 45° so the
   opening is ~60 cm and narrows to ~44 cm (robot width + 2 cm).
4. **Dock wiring:** `charger + → microswitch (NO) → top contact`, `charger − → bottom contact`.
   Mount the microswitch so the robot's back presses it in the last centimetre. Without a robot,
   the contacts are dead too.
5. **Weight / fixing:** the robot pushes back with a few newtons; screw the dock to the wall or
   weight the base, otherwise it slides and docking fails.
6. **Make it visible to the LiDAR:** the back plate should be at least as tall as the LiDAR's scan
   height, so the dock shows up on the map as a solid shape (and later the robot can align to it).
   Re-map that corner (or use Map by itself) after placing the dock.
7. **Charge voltage:** the Schottky drops ~0.3–0.4 V, so a 12.6 V charger stops at ~12.2–12.3 V
   (~90 %). That's fine (and kinder to the cells). For a full charge use an *ideal diode* module
   instead of the Schottky, or an adjustable CC/CV buck (e.g. XL4015) set to 12.9 V / 1.5 A.
8. **Test** with the robot on the dock: app battery chip shows ⚡ and a negative current
   (Lab → Health → Battery). Then drive away and press **Charge now**.

### Safety

- The pack must have a **3S BMS** (over-charge, over-discharge, short-circuit). Check yours has one.
- Never leave a Li-ion pack charging unattended at first; touch-test the diode and fuse after
  10 minutes of charging. Nothing should be more than warm.
- Keep the dock contacts recessed behind the funnel edge so feet and bags can't touch them.

## 4. Later: precise alignment with the camera

The dock works from the map position alone (Cartographer localization is good to ~3–5 cm, and
the funnel takes ±6 cm). When the camera comes:
- Stick an **AprilTag** (tag36h11, ~10 cm) on the dock above the contacts.
- The robot looks at it **from the approach point while facing the dock**, measures the exact
  offset, then turns round (the 55 cm approach distance leaves room for the back to swing) and reverses in.
- The LiDAR alternative needs no camera: a V-notch in the back plate gives a shape the scan can
  lock onto in the last 50 cm.
