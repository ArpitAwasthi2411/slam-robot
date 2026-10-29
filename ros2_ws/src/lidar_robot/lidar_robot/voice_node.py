#!/usr/bin/env python3
"""Voice node: push-to-talk on the robot -> speech to text -> robot command -> spoken reply.

    Button     a push button from GPIO17 (BCM, header pin 11) to GND (pin 9); internal pull-up.
               (GPIO14/15 = pins 8/10 are the ESP32 UART: never use them.) See hardware/CONNECTIONS_esp32s3.md.
    App        "Talk to the robot" on Home (hold) -> /api/voice -> voice/control
    Mic        the USB sound card (arecord, 16 kHz mono); speaker: the navigator's Speaker (Piper / espeak)
    STT        Groq Whisper when GROQ_API_KEY is set (online), else Vosk (offline, scripts/setup_voice.sh)

Subscribes  /voice/control       std_msgs/String  {"action": "listen" | "stop", "max_s": 8}
            /navigator/response  std_msgs/String
Publishes   /voice/status        std_msgs/String  JSON (2 Hz): ok, mic, button, stt, listening, last_heard, last_reply
            /navigator/request   std_msgs/String  {"type": "command", "text": ..., "by": "voice"}
            /speech/say          std_msgs/String  {"text": ..., "key": ...}  (spoken by the navigator)
"""
import json
import math
import os
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
from array import array

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lidar_robot.voice import (GpioButton, GroqWhisper, PushToTalk, SpeechToText, VoiceAssistant, VoskSTT,
                               write_wav)


def beep(device='', hz=880, ms=120, rate=16000):
    """Short tone through aplay: "I'm listening" (before recording, so it is not recorded)."""
    if not shutil.which('aplay'):
        return
    n = int(rate * ms / 1000)
    s = array('h', [int(9000 * math.sin(2 * math.pi * hz * i / rate) * min(1, i / 200, (n - i) / 200)) for i in range(n)])
    cmd = ['aplay', '-q'] + (['-D', device] if device else []) + ['-']
    try:
        subprocess.run(cmd, input=write_wav(rate, s), timeout=2, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        pass


class Recorder:
    def __init__(self, device=''):
        self.device = device
        self.proc = None
        self.path = None
        self.error = None if shutil.which('arecord') else 'arecord missing (sudo apt install alsa-utils)'

    def start(self):
        if self.error:
            return False
        fd, self.path = tempfile.mkstemp(suffix='.wav', prefix='ptt_')
        os.close(fd)
        cmd = ['arecord', '-q'] + (['-D', self.device] if self.device else []) + \
              ['-f', 'S16_LE', '-r', '16000', '-c', '1', '-t', 'wav', self.path]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except OSError as e:
            self.error = str(e)
            return False
        return True

    def stop(self):
        if not self.proc:
            return None
        self.proc.terminate()
        try:
            _, err = self.proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            err = b''
        self.proc = None
        try:
            with open(self.path, 'rb') as f:
                data = f.read()
            os.remove(self.path)
        except OSError:
            data = b''
        if len(data) < 100:
            self.error = (err or b'').decode(errors='ignore').strip()[:160] or 'no audio from the microphone'
            return None
        self.error = None
        return data


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice')
        d = self.declare_parameter
        d('button_gpio', 17)            # BCM number; header pin 11, button to GND (pin 9)
        d('button_chip', 'gpiochip0')
        d('mic_device', '')             # ALSA device, e.g. plughw:CARD=Device,DEV=0 ('' = default)
        d('speaker_device', '')         # for the beep ('' = default)
        d('max_record_s', 8.0)
        d('stt_model', 'whisper-large-v3-turbo')
        d('stt_language', '')           # '' = auto (English / Hindi / Hinglish), or en / hi
        d('vosk_en', '~/models/vosk/en')
        d('vosk_hi', '~/models/vosk/hi')
        d('beep', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        key = os.environ.get('GROQ_API_KEY')
        self.stt = SpeechToText(GroqWhisper(key, model=str(g('stt_model')), language=str(g('stt_language')) or None),
                                VoskSTT({'en': g('vosk_en'), 'hi': g('vosk_hi')}))
        self.mic = Recorder(str(g('mic_device')))
        self.speaker_device = str(g('speaker_device'))
        self.beep = bool(g('beep'))
        self.say_pub = self.create_publisher(String, 'speech/say', 10)
        self.req_pub = self.create_publisher(String, 'navigator/request', 10)
        self.status_pub = self.create_publisher(String, 'voice/status', 10)
        self.create_subscription(String, 'navigator/response', self._on_response, 10)
        self.create_subscription(String, 'voice/control', self._on_control, 10)
        self._waiting = {}
        self.assistant = VoiceAssistant(self.stt, self._send_command, self._say)
        self.ptt = PushToTalk(self._start, self._stop, max_s=float(g('max_record_s')))
        self.button = GpioButton(int(g('button_gpio')), str(g('button_chip')))
        if self.button.error:
            self.get_logger().warn(f'push-to-talk button off: {self.button.error} (the app button still works)')
        self.state = {'listening': False, 'thinking': False, 'message': '', 'last_heard': '', 'last_reply': ''}
        self.lock = threading.Lock()
        self.create_timer(0.02, self._poll_button)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f'voice ready: speech recognition {self.stt.describe()}, mic '
                               f'{self.mic.error or (self.mic.device or "default")}, button GPIO{g("button_gpio")}'
                               f'{"" if not self.button.error else " (off)"}')

    # ------------------------------------------------------------ inputs
    def _poll_button(self):
        v = self.button.read()
        if v is not None:
            self.ptt.update(v, time.monotonic())
        elif self.ptt.recording:
            self.ptt.update(False, time.monotonic())        # app-started recording still times out

    def _on_control(self, msg):
        try:
            c = json.loads(msg.data)
        except ValueError:
            c = {'action': msg.data.strip()}
        now = time.monotonic()
        if c.get('action') == 'listen':
            self.ptt.press(now, float(c.get('max_s') or 8.0))
        elif c.get('action') == 'stop':
            self.ptt.release(now)

    def _on_response(self, msg):
        try:
            r = json.loads(msg.data)
        except ValueError:
            return
        ev = self._waiting.get(r.get('req_id'))
        if ev:
            ev[1] = r
            ev[0].set()

    # ------------------------------------------------------------ record -> understand -> act
    def _start(self):
        if self.beep:
            beep(self.speaker_device)
        ok = self.mic.start()
        with self.lock:
            self.state.update(listening=ok, message='' if ok else self.mic.error)
        if not ok:
            self._say("My microphone isn't working.")

    def _stop(self, why, seconds):
        data = self.mic.stop()
        with self.lock:
            self.state.update(listening=False, thinking=data is not None)
        if data is None:
            with self.lock:
                self.state['message'] = self.mic.error or 'no audio'
            return
        threading.Thread(target=self._understand, args=(data,), daemon=True).start()

    def _understand(self, wav):
        r = self.assistant.handle(wav)
        with self.lock:
            self.state.update(thinking=False, last_heard=r.get('heard', ''), last_reply=r.get('reply', ''),
                              message=r.get('error') or r.get('note') or '', engine=r.get('engine'),
                              stt_ms=r.get('stt_ms'), t=round(time.time(), 1))
        self.get_logger().info(f'voice: heard "{r.get("heard")}" ({r.get("engine")}) -> "{r.get("reply")}"')

    def _send_command(self, text):
        rid = uuid.uuid4().hex[:8]
        ev = [threading.Event(), None]
        self._waiting[rid] = ev
        self.req_pub.publish(String(data=json.dumps({'type': 'command', 'text': text, 'by': 'voice', 'req_id': rid})))
        ev[0].wait(6.0)
        self._waiting.pop(rid, None)
        return ev[1]

    def _say(self, text):
        self.say_pub.publish(String(data=json.dumps({'text': text, 'key': 'voice_reply'})))

    def _publish_status(self):
        with self.lock:
            st = dict(self.state)
        st.update(ok=self.mic.error is None and bool(self.stt.engines()), mic=self.mic.error is None,
                  button=self.button.error is None, stt=self.stt.describe())
        if not st['message']:
            if not self.stt.engines():
                st['message'] = 'no speech recognition: add GROQ_API_KEY or run scripts/setup_voice.sh'
            elif self.mic.error:
                st['message'] = self.mic.error
        self.status_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
