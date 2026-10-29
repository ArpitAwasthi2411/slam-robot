#!/usr/bin/env bash
# Voice on the robot: USB sound card (mic + speaker), push-to-talk button on GPIO17, offline speech
# recognition (Vosk, English + Hindi) and a natural offline voice (Piper, English + Hindi).
#
#   sudo bash ~/slam-robot/scripts/setup_voice.sh            install (needs internet once, ~300 MB download)
#   sudo bash ~/slam-robot/scripts/setup_voice.sh test       record 4 s, play it back, Piper says a sentence
#   sudo bash ~/slam-robot/scripts/setup_voice.sh status     what is installed, which sound cards, the button
#
# Online speech recognition (better, Hinglish) uses GROQ_API_KEY from ~/robot.env: nothing to install.
# Wiring of the button: hardware/CONNECTIONS_esp32s3.md (Pi pin 11 = GPIO17 and pin 9 = GND).
# Full guide: docs/STEPS_voice.md
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "Run with sudo"; exit 1; }
U="${SUDO_USER:-arpit}"
H="${PATHIK_HOME:-$(getent passwd "$U" | cut -d: -f6)}"
H="${H:-/home/$U}"
MODELS="$H/models/vosk"
PIPER="$H/piper"
PIPER_REL="2023.11.14-2"
VOICES="https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0"
VOICE_EN="en/en_US/lessac/medium/en_US-lessac-medium"
VOICE_HI="hi/hi_IN/pratham/medium/hi_IN-pratham-medium"
VOSK_EN="vosk-model-small-en-in-0.4"
VOSK_HI="vosk-model-small-hi-0.22"
as_user() { sudo -u "$U" -H "$@"; }

fetch() {   # url dest
  as_user curl -fL --retry 3 --connect-timeout 15 -o "$2.part" "$1" && as_user mv "$2.part" "$2"
}

card_hint() {
  echo "Sound cards (capture):"; arecord -l 2>/dev/null | sed -n 's/^card/  card/p' || true
  echo "Sound cards (playback):"; aplay -l 2>/dev/null | sed -n 's/^card/  card/p' || true
}

case "${1:-install}" in
  install)
    if ! curl -s -m 8 -o /dev/null https://huggingface.co; then
      echo "!! No internet on the Pi. Join a network first (sudo bash scripts/setup_wifi.sh hotspot NAME PASS)."; exit 1
    fi
    echo "== 1/5 packages: ALSA tools, libgpiod, espeak-ng (fallback voice)"
    apt-get install -y alsa-utils gpiod python3-libgpiod espeak-ng python3-pip unzip curl

    echo "== 2/5 push-to-talk button: GPIO access for $U"
    cat > /etc/udev/rules.d/99-pathik-gpio.rules <<'EOF'
# Pathik: the push-to-talk button (GPIO17) is read by the voice node as the robot user
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="dialout", MODE="0660"
EOF
    udevadm control --reload-rules && udevadm trigger --subsystem-match=gpio || true
    usermod -aG dialout,audio "$U"

    echo "== 3/5 offline speech recognition: Vosk + small English (India) and Hindi models"
    as_user pip3 install --user 'vosk==0.3.45'
    as_user mkdir -p "$MODELS"
    for pair in "en:$VOSK_EN" "hi:$VOSK_HI"; do
      lang="${pair%%:*}"; name="${pair#*:}"
      if [ -d "$MODELS/$lang" ]; then echo "  $lang: already there"; continue; fi
      fetch "https://alphacephei.com/vosk/models/$name.zip" "$MODELS/$name.zip"
      as_user unzip -q -o "$MODELS/$name.zip" -d "$MODELS"
      as_user mv "$MODELS/$name" "$MODELS/$lang"
      rm -f "$MODELS/$name.zip"
      echo "  $lang: $MODELS/$lang"
    done

    echo "== 4/5 offline voice: Piper + English and Hindi voices"
    case "$(uname -m)" in
      aarch64) ARCH=aarch64 ;; x86_64) ARCH=x86_64 ;; *) echo "!! Piper has no build for $(uname -m)"; ARCH="" ;;
    esac
    if [ -n "$ARCH" ] && [ ! -x "$PIPER/piper" ]; then
      fetch "https://github.com/rhasspy/piper/releases/download/$PIPER_REL/piper_linux_$ARCH.tar.gz" "$H/piper.tgz"
      as_user tar -xzf "$H/piper.tgz" -C "$H"
      rm -f "$H/piper.tgz"
    fi
    as_user mkdir -p "$PIPER/voices"
    for v in "$VOICE_EN" "$VOICE_HI"; do
      f="$PIPER/voices/$(basename "$v")"
      [ -f "$f.onnx" ] || fetch "$VOICES/$v.onnx" "$f.onnx"
      [ -f "$f.onnx.json" ] || fetch "$VOICES/$v.onnx.json" "$f.onnx.json"
    done

    echo "== 5/5 sound card"
    card_hint
    USB=$(aplay -l 2>/dev/null | sed -n 's/^card \([0-9]*\): .*USB.*/\1/p' | head -1)
    if [ -n "$USB" ] && [ ! -f "$H/.asoundrc" ]; then
      printf 'defaults.pcm.card %s\ndefaults.ctl.card %s\n' "$USB" "$USB" | as_user tee "$H/.asoundrc" > /dev/null
      echo "  USB sound card $USB is now the default (~/.asoundrc)"
    fi
    echo
    echo "Done. Log out and in (or reboot) for the GPIO/audio groups, then:"
    echo "  sudo bash $0 test"
    echo "  sudo systemctl restart robot      (the voice node starts with the robot; voice:=false turns it off)"
    ;;
  test)
    card_hint
    T=/tmp/pathik_mic_test.wav
    echo "Speak for 4 seconds after the beep…"
    as_user arecord -q -d 4 -f S16_LE -r 16000 -c 1 "$T" && echo "Playing it back:" && as_user aplay -q "$T"
    if [ -x "$PIPER/piper" ]; then
      M=$(ls "$PIPER"/voices/en_*.onnx 2>/dev/null | head -1)
      echo "Hello, I am Pathik. Where should I go?" | as_user "$PIPER/piper" --model "$M" --output_raw 2>/dev/null \
        | as_user aplay -q -r 22050 -f S16_LE -t raw -c 1 - && echo "Piper: OK"
    else
      espeak-ng "Hello, I am Pathik" && echo "espeak-ng: OK (Piper not installed)"
    fi
    ;;
  status)
    echo "Piper       : $([ -x "$PIPER/piper" ] && echo "$PIPER/piper" || echo missing)"
    echo "voices      : $(find "$PIPER/voices" -name '*.onnx' -printf '%f ' 2>/dev/null)"
    echo "Vosk        : $(as_user python3 -c 'import vosk; print("ok")' 2>/dev/null || echo missing)"
    echo "Vosk models : $(ls -d "$MODELS"/en "$MODELS"/hi 2>/dev/null | tr '\n' ' ')"
    echo "GROQ key    : $(grep -q '^GROQ_API_KEY=.' "$H/robot.env" 2>/dev/null && echo 'set in ~/robot.env (online Whisper)' || echo 'not set (offline Vosk only)')"
    echo "button      : GPIO17 = $(gpioget --bias=pull-up gpiochip0 17 2>/dev/null || gpioget gpiochip0 17 2>/dev/null || echo '?') (1 = released, 0 = pressed)"
    card_hint
    ;;
  *) sed -n '2,11p' "$0"; exit 1 ;;
esac
