"""Voice commands: recorded WAV -> text (Groq Whisper online, Vosk offline) -> robot command -> spoken reply.
Pure Python (no ROS), tested with WAV fixtures; the ROS side is voice_node.py.

    stt = SpeechToText(GroqWhisper(os.environ.get('GROQ_API_KEY')), VoskSTT({'en': '~/models/vosk/en', 'hi': ...}))
    r = stt.transcribe(wav_bytes)          # {'text': 'go to lab three', 'engine': 'groq', 'language': 'en', ...}

Keys: GROQ_API_KEY (Whisper on Groq). Without it, or without internet, Vosk runs on the Pi (small English
and Hindi models from scripts/setup_voice.sh). Neither available -> a clear error, never a crash.
"""
import importlib.util
import io
import json
import math
import os
import time
import urllib.request
import uuid
import wave
from array import array

GROQ_STT_URL = 'https://api.groq.com/openai/v1/audio/transcriptions'


# ------------------------------------------------------------------ WAV helpers
def read_wav(data):
    """bytes -> (rate, mono int16 array). Accepts 8/16-bit, mono/stereo PCM WAV."""
    with wave.open(io.BytesIO(data)) as w:
        rate, ch, width, n = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
        raw = w.readframes(n)
    if width == 2:
        a = array('h')
        a.frombytes(raw[:len(raw) - len(raw) % 2])
    elif width == 1:
        a = array('h', [(b - 128) << 8 for b in raw])
    else:
        raise ValueError(f'{8 * width}-bit WAV is not supported (record 16-bit)')
    if ch > 1:
        a = array('h', [sum(a[i:i + ch]) // ch for i in range(0, len(a) - ch + 1, ch)])
    return rate, a


def write_wav(rate, samples):
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(array('h', samples).tobytes())
    return buf.getvalue()


def resample(samples, rate, to=16000):
    """Linear resampling (good enough for speech recognition)."""
    if rate == to or not samples:
        return array('h', samples)
    n = int(len(samples) * to / rate)
    out = array('h', [0]) * n
    k = rate / to
    last = len(samples) - 1
    for i in range(n):
        x = i * k
        j = int(x)
        f = x - j
        out[i] = int(samples[j] * (1 - f) + samples[min(j + 1, last)] * f)
    return out


def level_dbfs(samples):
    if not samples:
        return -120.0
    rms = math.sqrt(sum(s * s for s in samples) / len(samples))
    return 20 * math.log10(max(rms, 1e-9) / 32768.0)


def speech_bounds(samples, rate, frame_ms=30, threshold_db=-42.0, pad_ms=200):
    """(start, end, loud_s): sample indexes of the part louder than threshold with pad_ms around it, and how
    many seconds of it are actually loud (None if it is all quiet)."""
    n = max(1, int(rate * frame_ms / 1000))
    loud = [i for i in range(0, len(samples), n) if level_dbfs(samples[i:i + n]) > threshold_db]
    if not loud:
        return None
    pad = int(rate * pad_ms / 1000)
    return max(0, loud[0] - pad), min(len(samples), loud[-1] + n + pad), len(loud) * n / float(rate)


def prepare(data, min_speech_s=0.25):
    """Recorded WAV -> 16 kHz mono WAV with the silence trimmed, plus facts. Raises ValueError with a
    sentence the robot can say ("I didn't hear anything")."""
    rate, s = read_wav(data)
    s = resample(s, rate, 16000)
    b = speech_bounds(s, 16000)
    if b is None:
        raise ValueError("I didn't hear anything. Hold the button and speak a bit louder.")
    s = s[b[0]:b[1]]
    dur = len(s) / 16000.0
    if b[2] < min_speech_s:
        raise ValueError("That was too short. Hold the button while you speak.")
    return write_wav(16000, s), {'seconds': round(dur, 2), 'level_db': round(level_dbfs(s), 1)}


def multipart(fields, files):
    """multipart/form-data body (fields: {name: value}, files: {name: (filename, bytes, type)})."""
    boundary = uuid.uuid4().hex
    out = io.BytesIO()
    for k, v in fields.items():
        out.write(f'--{boundary}\r\nContent-Disposition: form-data; name="{k}"\r\n\r\n{v}\r\n'.encode())
    for k, (fn, data, ctype) in files.items():
        out.write(f'--{boundary}\r\nContent-Disposition: form-data; name="{k}"; filename="{fn}"\r\n'
                  f'Content-Type: {ctype}\r\n\r\n'.encode())
        out.write(data)
        out.write(b'\r\n')
    out.write(f'--{boundary}--\r\n'.encode())
    return out.getvalue(), f'multipart/form-data; boundary={boundary}'


def _http_post(url, body, headers, timeout):
    req = urllib.request.Request(url, data=body, method='POST', headers=headers)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


# ------------------------------------------------------------------ engines
class GroqWhisper:
    """Whisper on Groq (needs GROQ_API_KEY + internet). language None = detect (English, Hindi, Hinglish)."""
    name = 'groq'

    def __init__(self, key, model='whisper-large-v3-turbo', language=None, timeout=8.0, http_post=None):
        self.key, self.model, self.language, self.timeout = key, model, language, timeout
        self._post = http_post or _http_post

    def available(self):
        return bool(self.key)

    def transcribe(self, wav):
        if not self.key:
            raise RuntimeError('no GROQ_API_KEY')
        fields = {'model': self.model, 'response_format': 'verbose_json', 'temperature': '0',
                  'prompt': 'Commands for a campus robot: go to Lab 3, HOD office, come back, wait, stop, charge.'}
        if self.language:
            fields['language'] = self.language
        body, ctype = multipart(fields, {'file': ('speech.wav', wav, 'audio/wav')})
        r = self._post(GROQ_STT_URL, body, {'Authorization': f'Bearer {self.key}', 'Content-Type': ctype},
                       self.timeout)
        return {'text': str(r.get('text', '')).strip(), 'language': r.get('language')}


class VoskSTT:
    """Offline (Vosk small models). model_dirs: {'en': path, 'hi': path}; both run, the more confident wins."""
    name = 'vosk'

    def __init__(self, model_dirs, loader=None):
        self.dirs = {k: os.path.expanduser(v) for k, v in (model_dirs or {}).items() if v}
        self._loader = loader
        self._models = {}
        self.error = None

    def _load(self, lang):
        if lang in self._models:
            return self._models[lang]
        if self._loader is not None:
            m = self._loader(lang, self.dirs[lang])
        else:
            import vosk                                   # pip install vosk (setup_voice.sh)
            vosk.SetLogLevel(-1)
            m = vosk.Model(self.dirs[lang])
        self._models[lang] = m
        return m

    def available(self):
        if not any(os.path.isdir(d) for d in self.dirs.values()) and self._loader is None:
            self.error = 'no Vosk model folder (run scripts/setup_voice.sh)'
            return False
        if self._loader is None and importlib.util.find_spec('vosk') is None:
            self.error = 'vosk is not installed (run scripts/setup_voice.sh)'
            return False
        return True

    def transcribe(self, wav):
        rate, s = read_wav(wav)
        best = None
        for lang in self.dirs:
            if self._loader is None and not os.path.isdir(self.dirs[lang]):
                continue
            model = self._load(lang)
            if self._loader is not None:
                text, conf = model(wav)
            else:
                import vosk
                rec = vosk.KaldiRecognizer(model, rate)
                rec.SetWords(True)
                rec.AcceptWaveform(s.tobytes())
                res = json.loads(rec.FinalResult())
                words = res.get('result') or []
                text = res.get('text', '')
                conf = sum(w.get('conf', 0) for w in words) / len(words) if words else 0.0
            if text and (best is None or conf > best[2]):
                best = (text, lang, conf)
        if best is None:
            return {'text': '', 'language': None, 'confidence': 0.0}
        return {'text': best[0], 'language': best[1], 'confidence': round(best[2], 3)}


class SpeechToText:
    """Online first, offline fallback. transcribe() -> {'text', 'engine', 'language', 'ms', 'note'}."""

    def __init__(self, online=None, offline=None, clock=time.monotonic):
        self.online, self.offline, self.clock = online, offline, clock

    def engines(self):
        return [e for e in (self.online, self.offline) if e is not None and e.available()]

    def describe(self):
        names = [e.name for e in self.engines()]
        return ' + '.join(names) if names else 'none'

    def transcribe(self, wav):
        notes = []
        for eng in (self.online, self.offline):
            if eng is None:
                continue
            if not eng.available():
                notes.append(f'{eng.name}: {getattr(eng, "error", None) or "not available"}')
                continue
            t0 = self.clock()
            try:
                r = eng.transcribe(wav)
            except Exception as e:                    # network, quota, model ...
                notes.append(f'{eng.name} failed ({type(e).__name__})')
                continue
            r.update(engine=eng.name, ms=round((self.clock() - t0) * 1000), note='; '.join(notes))
            return r
        raise RuntimeError('no speech recognition available: ' + ('; '.join(notes) or 'none configured')
                           + '. Add GROQ_API_KEY to ~/robot.env, or run scripts/setup_voice.sh for offline Vosk.')


# ------------------------------------------------------------------ the whole voice command
def reply_text(res):
    """Navigator response to a command -> one sentence to say back."""
    if not res:
        return "Sorry, the robot did not answer."
    if res.get('ok') is False:
        return f"Sorry, I can't do that: {res.get('error', 'unknown error')}."
    it = res.get('interpretation') or {}
    for k in ('answer', 'status'):
        if res.get(k):
            return str(res[k])
    if it.get('intent') == 'unknown':
        return it.get('reply') or "Sorry, I didn't understand. Say a place, for example: go to Lab 3."
    return it.get('reply') or 'OK.'


class VoiceAssistant:
    """handle(wav) -> {'heard', 'reply', 'engine', 'ok', ...}; send(text) -> navigator response dict."""

    def __init__(self, stt, send, speak, clock=time.monotonic):
        self.stt, self.send, self.speak, self.clock = stt, send, speak, clock
        self.last = None

    def handle(self, wav):
        t0 = self.clock()
        out = {'ok': False, 'heard': '', 'reply': '', 'engine': None}
        try:
            clean, facts = prepare(wav)
            out.update(facts)
        except Exception as e:                        # bad WAV, too quiet, too short
            out['reply'] = str(e) if isinstance(e, ValueError) else 'I could not read the recording.'
            self.speak(out['reply'])
            self.last = out
            return out
        try:
            r = self.stt.transcribe(clean)
        except RuntimeError as e:
            out['reply'] = "I can't understand speech right now: no speech recognition is set up."
            out['error'] = str(e)
            self.speak(out['reply'])
            self.last = out
            return out
        out.update(heard=r['text'], engine=r['engine'], language=r.get('language'), stt_ms=r.get('ms'),
                   note=r.get('note', ''))
        if not r['text']:
            out['reply'] = "Sorry, I didn't catch that. Please say it again."
        else:
            res = self.send(r['text'])
            out['reply'] = reply_text(res)
            out['ok'] = bool(res and res.get('ok') is not False and (res.get('interpretation') or {}).get('intent') != 'unknown')
            out['intent'] = (res.get('interpretation') or {}).get('intent') if res else None
        self.speak(out['reply'])
        out['total_ms'] = round((self.clock() - t0) * 1000)
        self.last = out
        return out


# ------------------------------------------------------------------ push-to-talk
class PushToTalk:
    """Debounced button -> start / stop recording. update(pressed, now) once per poll (~50 Hz).
    Also driven by the app ("talk to the robot"): press(now, max_s) / release(now)."""

    def __init__(self, on_start, on_stop, debounce=3, max_s=8.0, min_gap_s=0.5):
        self.on_start, self.on_stop = on_start, on_stop
        self.debounce, self.max_s, self.min_gap_s = debounce, max_s, min_gap_s
        self._count = 0
        self.down = False
        self.recording = False
        self._t0 = None
        self._limit = max_s
        self._last_end = -1e9

    def update(self, pressed, now):
        # +n after n polls pressed in a row, -n after n polls released in a row
        if pressed:
            self._count = min(self.debounce, max(0, self._count) + 1)
        else:
            self._count = max(-self.debounce, min(0, self._count) - 1)
        if self._count >= self.debounce and not self.down:
            self.down = True
            self.press(now)
        elif self._count <= -self.debounce and self.down:
            self.down = False
            self.release(now)
        if self.recording and now - self._t0 >= self._limit:
            self._stop(now, 'max_length')

    def press(self, now, max_s=None):
        if self.recording or now - self._last_end < self.min_gap_s:
            return False
        self.recording, self._t0 = True, now
        self._limit = min(self.max_s, max_s) if max_s else self.max_s
        self.on_start()
        return True

    def release(self, now):
        if self.recording:
            self._stop(now, 'released')

    def _stop(self, now, why):
        self.recording = False
        self._last_end = now
        self.on_stop(why, now - self._t0)


class GpioButton:
    """A push button between a Pi GPIO (BCM numbering) and GND, internal pull-up: pressed = 0.
    libgpiod v1 (Ubuntu 22.04: python3-libgpiod) or v2; read() -> True when pressed, None if unavailable."""

    def __init__(self, line=17, chip='gpiochip0', consumer='pathik-ptt'):
        self.line_no, self.chip_name = line, chip
        self.error = None
        self._read = None
        try:
            import gpiod
        except ImportError:
            self.error = 'python3-libgpiod is not installed (scripts/setup_voice.sh)'
            return
        try:
            if hasattr(gpiod, 'request_lines'):                    # libgpiod v2
                from gpiod.line import Bias, Direction
                req = gpiod.request_lines(f'/dev/{chip}', consumer=consumer, config={
                    line: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)})
                self._read = lambda: req.get_value(line).value == 0
            else:                                                 # libgpiod v1
                c = gpiod.Chip(chip)
                ln = c.get_line(line)
                flags = getattr(gpiod, 'LINE_REQ_FLAG_BIAS_PULL_UP', 0)
                ln.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_IN, flags=flags)
                self._read = lambda: ln.get_value() == 0
        except Exception as e:                                    # no permission / busy / wrong chip
            self.error = f'GPIO{line} on {chip}: {e}'

    def read(self):
        if self._read is None:
            return None
        try:
            return bool(self._read())
        except Exception as e:
            self.error = str(e)
            return None
