# Head mode: a phone as Pathik's face (and its people detector)

A phone mounted on the robot runs the Pathik app in **Head mode** (Operate → Head mode):

| It does | How |
|---|---|
| **Face** | 12 moods from the robot's state (`face.js`): idle, resting, thinking, listening, driving, "Excuse me, may I pass?", arrived, asking for help, charging, low battery, stopped, lost / offline |
| **STOP** | big red button, always, **no PIN** (anyone can stop the robot) |
| **Side icons** | **Go** (places, rooms, go home, cancel my trip) · **Ask** (type or speak a question or a command) · **Deliver** (errand). Big, at the sides of the face (below it on a tall screen) |
| **PIN** | asked **last**: people look and choose freely; the PIN pad appears with the question ("Go to Lab 3?") just before the robot would start, or before a trip is cancelled. **Questions never need it.** |
| **Ask** | a question (college, department, the robot itself) is answered at once on the screen and by the robot's voice. A trip asked for in words is shown first, then the PIN |
| **Tap the face** | the robot says its name and asks back ("Hi, I'm Pathik! What can I do for you today?"), and the three icons bounce to invite a tap. While busy it says what it is doing. The eyes glance at where it was touched |
| **Answers** | the OK button at "wait for OK" stops and the error-recovery questions appear on the face (no PIN, like the app) |
| **People detection** | YOLO11n on the **phone**, on the **Pi camera's plain picture**; the boxes go back to the robot |
| Voice | stays on the **robot's speaker** (Piper); the phone's mic can be used to say a command in the kiosk |
| **Errands** | **Deliver** → go to a place, check its door, say a message, wait (or until OK), come back; spoken report |
| **Doors** | settings → teach each door closed / open with the camera; errands check it on arrival |

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

## Which engine runs the detection on the phone

| Engine | Where | Speed (YOLO11n 320) |
|---|---|---|
| **Native TensorFlow Lite on the GPU** | Pathik APK, phones whose GPU TFLite supports | usually fastest |
| **Native TensorFlow Lite on the CPU** | Pathik APK, XNNPACK on (cores − 1, max 4) threads | several cores |
| ONNX Runtime Web | any browser / if the native engine can't start | **1 core** in an Android WebView (~180 ms) |

At start the app times the GPU and the CPU engine on the model and keeps the faster; the Phone detector page shows
*runs on* with both start-test times (e.g. "phone GPU (start test: GPU 22 ms, CPU 64 ms)"). The model in the APK is
`vendor/yolo11n_320.tflite`, converted from the same ONNX file at build time (`scripts/fetch_phone_detector.sh`),
so both engines give the same boxes.

## Errands

Kiosk → **Errand: go, wait, come back**: choose the place, how long to wait (30 s – 5 min, or until someone
presses OK), what to do after (come back here / go home / stay) and an optional message. The robot then, speaking
each step on its own speaker:

1. "Going to Shashi sir cabin." → drives there (people, replanning and help questions as usual)
2. checks the door with the camera → "The door of Shashi sir cabin is closed." (if it is)
3. says the message ("Hello! Ravi sir sent me with the file.", or "I'll wait here outside…" when closed)
4. waits with a countdown on the face; **OK, done** ends it early
5. "Going back to where I started." → drives back (to the exact spot the errand started)
6. report: "I went to Shashi sir cabin, the door was closed, I waited 2 minutes and I am back."

If it cannot reach the place it says why and comes back. Cancel from the Deliver sheet (PIN). Code: `mission.js`
(steps, tested in `test_errand.py`), `head.js` (robot / demo operations).

## Door check with the camera

YOLO has no door class, so the robot learns **each door**: stop where the robot stops in front of it, head settings
→ Doors → choose the place → **This is CLOSED** / **This is OPEN** (2–3 times each, different light if you can).
On an errand it compares three pictures with what it learned (`doorcheck.js`: layout + edge directions, tolerant
of light changes and a few cm of stopping error) and says open / closed / unknown. Unknown = it doesn't decide
for you. The references live on the head phone. Planned: a LiDAR doorway check next to it.

## Peak test

Operate → **Peak test** runs detection flat out, the face, door checks and spoken lines together for 1–20 min and
grades: detections ≥ 5/s, no more than 25 % slower at the end (heat), face ≥ 45 fps, phone ≤ 42 °C, Android heat
state none/light, battery ≤ 25 %/h, page stalls ≤ 1 s/min. Run 10 min with the phone in its mount before trusting
it on the robot. The web engine now runs YOLO in a worker (`detworker.js`), so the face no longer stutters on it.

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


## The face (Nov 2026)

A dark visor with two glowing eyes (`face.js`). Each mood is a target shape (eye size, slant, gaze, mouth,
colour) and the face eases from one to the next, so it never jumps. It blinks, glances around when idle,
breathes slightly, moves its mouth while the robot speaks, and shows a pulsing ring when it needs attention
(blocked, asking for help, listening). When an object blocks the way and no person is there, it shows
"Something is in my way · finding another way" instead of asking to pass.
