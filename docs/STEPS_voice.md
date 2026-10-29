# Voice on the robot: push-to-talk, speech recognition, Piper voice

What you get:
- **Talk to the robot**: hold the button on the robot, or hold "Talk to the robot" in the app's Home voice card.
  The robot beeps, records from the USB sound card while you hold, turns the speech into text and runs it
  like a typed command ("take this to Lab 3 and come back", "where is the 3D printer?", "stop"). It then
  answers out loud.
- **Speech recognition**:
  - Groq **Whisper** (`whisper-large-v3-turbo`) when `GROQ_API_KEY` is in `~/robot.env` and there is internet.
    It handles English, Hindi and Hinglish.
  - Otherwise **Vosk**, offline on the Pi, with small English (India) and Hindi models; the more confident one wins.
  - Spoken numbers match places ("lab three" = Lab 3).
- **Voice**: **Piper**, offline and natural-sounding; espeak-ng is the fallback.
  - English voice `en_US-lessac-medium`; Hindi voice `hi_IN-pratham-medium` for Devanagari text.
  - One phrase at a time, at most 8 a minute (questions to people and battery warnings always get through),
    adjustable volume.
  - The robot announces: going to X, arrived, make way, docking / docked, low battery, recovery decisions,
    and questions to people.

## 1. Wire the button (2 wires)
A momentary push button between **Pi pin 11 (GPIO17)** and **Pi pin 9 (GND)**
(`hardware/CONNECTIONS_esp32s3.md`, rows B1/B2). No resistor is needed: the Pi's internal pull-up is used.
Never use pins 8/10: they are the ESP32 link.

## 2. Sound card
USB sound card in a Pi USB port, lapel mic in the pink jack, speaker in the green jack. Power the speaker
from the 5 V buck, not from the Pi.

## 3. Install (Pi, needs internet once)
```bash
sudo bash ~/slam-robot/scripts/setup_voice.sh
```
It installs ALSA tools, libgpiod and espeak-ng; GPIO access for `arpit`; Vosk with the English and Hindi models
(`~/models/vosk/en`, `~/models/vosk/hi`); Piper with two voices (`~/piper`). It also makes the USB sound card
the default output.
Then log out and in (for the groups) and:
```bash
sudo bash ~/slam-robot/scripts/setup_voice.sh test
```
```bash
sudo bash ~/slam-robot/scripts/setup_voice.sh status
```
`test` records 4 s and plays it back, then Piper speaks. `status` shows what is installed, the sound cards,
and the button: it should read 1, and 0 while you press it.

For online recognition add `GROQ_API_KEY=gsk_...` to `~/robot.env` (the same key the commands use).

## 4. Run
The voice node starts with the robot (`voice:=true` is the default in bringup):
```bash
sudo systemctl restart robot
```
```bash
journalctl -u robot | grep -i "voice\|robot voice"
```
- `voice ready: speech recognition groq + vosk, mic default, button GPIO17`
- `robot voice: piper, volume 1.0`

Settings in `config/robot_params.yaml`:
- `voice:`: `button_gpio`, `mic_device` (from `arecord -L`, e.g. `plughw:CARD=Device,DEV=0`),
  `max_record_s`, `stt_language` (`''` auto, or `en` / `hi`).
- `navigator:`: `speech_engine`, `speech_volume`, `speech_voice_en` / `_hi`, `speech_max_per_min`, `speech_device`.

## 5. Check
1. Hold the button, say "go to Lab 3", let go. Beep → the app's Home voice card shows "Heard: go to Lab 3 →
   Going to Lab 3." → the robot says it and drives.
2. Same with the app's "Talk to the robot" button (hold) while standing next to the robot.
3. Unplug the internet (or remove the key): it still works with Vosk. The card shows the engine.
4. "हॉड ऑफिस जाओ" / "HOD office jao": Hindi / Hinglish (best with the Groq key).
5. Say nothing: "I didn't hear anything". Mumble: "Sorry, I didn't catch that".
6. Volume: `speech_volume: 0.6` in the YAML, restart.

## Problems

| What you see | Do |
|---|---|
| `button off: python3-libgpiod is not installed` | run setup_voice.sh |
| button off: `Permission denied` | setup_voice.sh, then log out and in (dialout group) |
| card says "no speech recognition" | add GROQ_API_KEY, or run setup_voice.sh (Vosk models) |
| "My microphone isn't working" | `arecord -l`: set `mic_device` to the USB card (`plughw:CARD=...`) |
| robot silent | `setup_voice.sh status`; `speaker-test -t wav -c 1`; `speech_device` |
| words wrong offline | Vosk small models are basic: use short commands, or the Groq key |

Tests without the robot: `test/test_voice.py` (WAV fixtures in `test/fixtures/voice/`, made with espeak-ng),
the simulator (`POST /api/voice {"action": "stop", "text": "go to lab 3"}` pretends the mic heard it), and the
app's demo (hold "Talk to the robot").
