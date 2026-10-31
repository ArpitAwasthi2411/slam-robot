# Head mode: a phone as Pathik's face (and its people detector)

A phone mounted on the robot runs the Pathik app in **Head mode** (Operate → Head mode):

| It does | How |
|---|---|
| **Face** | 11 moods from the robot's state (`face.js`): idle, resting, thinking, driving, "Excuse me, may I pass?", arrived, asking for help, charging, low battery, stopped, lost / offline |
| **STOP** | big red button, always, **no PIN** (anyone can stop the robot) |
| **Kiosk** | tap the face → **PIN** → places, rooms, typed or spoken command, go home, cancel |
| **Answers** | the OK button at "wait for OK" stops and the error-recovery questions appear on the face (no PIN, like the app) |
| **People detection** | YOLO11n on the **phone**, on the **Pi camera's plain picture**; the boxes go back to the robot |
| Voice | stays on the **robot's speaker** (Piper); the phone's mic can be used to say a command in the kiosk |

## How the detection is shared

```
Pi camera ──(plain JPEG, /api/camera.jpg?raw=1, ~5/s)──► phone: YOLO11n (~80–120 ms)
phone ──(boxes, POST /api/vision/phone, ~100 bytes)──► dashboard ──► /vision/phone_boxes ──► vision node
vision node: LiDAR distance per person + the same rules (vision_rules.decide) ──► /vision/decision ──► navigator
```

- While fresh boxes arrive from the phone, the Pi **skips its own YOLO** (saves about one core).
- **No boxes for 1 s** (`phone_timeout`): the Pi runs its own YOLO again, by itself. The LiDAR and ultrasonic
  stops never depend on the phone or the Wi-Fi.
- The phone only detects while the robot is **busy** (driving, docking, …) and 30 s after; the Pi does the same
  (`idle_pause_s`), so an idle robot spends no CPU on detection.
- App Health → camera / the head's top-right chip show who is detecting: *this phone*, *the robot (Pi)* or *resting*.

## Set up

1. Robot: the new code (vision node with `phone_timeout`, `idle_pause_s`; dashboard `/api/vision/phone`).
2. Phone: Pathik APK, log in as an operator, connect to the robot.
3. Operate → **Head mode**. The first time it asks for a **PIN** (4–8 digits); keep it for the robot team.
4. Mount the phone at the front, screen forward, tilted back ~15°, back of the phone in open air; power it from a
   separate 5 V 2–3 A supply, and limit charging to 80 % in the phone's battery settings if it has that option.
5. Leave head mode: gear icon (or Android back) → PIN → **Leave head mode**. Release a STOP there too.

## Test without the robot

Open Head mode while **not connected**: the face runs on a demo robot (choose a place in the kiosk and watch the
moods). Settings → **Test detection with this phone's camera** shows the phone's own detection speed and decision.
