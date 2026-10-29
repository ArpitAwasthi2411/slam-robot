"""Robot voice: short spoken phrases through the speaker on the Pi (offline).

    Speaker(engine='auto')   Piper (natural offline voices, English + Hindi) when scripts/setup_voice.sh has
                             installed it, else espeak-ng, else silent (never an error)
    sp.say('Going to Lab 3.', key='going')

* One phrase at a time (a queue, never two voices on top of each other); a full queue drops the oldest.
* Rate-limited: the same key not again within gap_s (8 s), and at most max_per_min phrases a minute
  (important keys like a question to a person or a low battery always get through).
* Volume 0..1.5 (piper: samples scaled before aplay; espeak: amplitude).
* Text with Devanagari letters uses the Hindi voice; everything else (also Hinglish in Latin letters)
  the English one.
"""
import array
import glob
import os
import queue
import re
import shutil
import subprocess
import threading
import time

DEVANAGARI = re.compile('[ऀ-ॿ]')
PRIORITY_KEYS = {'recovery_ask', 'battery', 'confirm', 'estop', 'voice_reply', 'dock'}
PIPER_DIRS = ['~/piper', '/opt/piper', '/usr/local/share/piper']


def find_piper():
    exe = shutil.which('piper')
    for d in PIPER_DIRS:
        p = os.path.join(os.path.expanduser(d), 'piper')
        if not exe and os.path.isfile(p) and os.access(p, os.X_OK):
            exe = p
    return exe


def find_voice(lang, explicit=''):
    """explicit path, else the first ~/piper/voices/<lang>_*.onnx (en_US-lessac-medium, hi_IN-pratham-medium ...)."""
    if explicit and os.path.isfile(os.path.expanduser(explicit)):
        return os.path.expanduser(explicit)
    for d in PIPER_DIRS:
        hits = sorted(glob.glob(os.path.join(os.path.expanduser(d), 'voices', f'{lang}_*.onnx')))
        if hits:
            return hits[0]
    return None


def scale_pcm(raw, volume):
    """16-bit little-endian mono PCM bytes * volume, clipped."""
    if abs(volume - 1.0) < 1e-3:
        return raw
    a = array.array('h')
    a.frombytes(raw[:len(raw) - len(raw) % 2])
    for i, v in enumerate(a):
        s = int(v * volume)
        a[i] = 32767 if s > 32767 else -32768 if s < -32768 else s
    return a.tobytes()


class Speaker:
    def __init__(self, enabled=True, voice='en-in', speed=150, min_gap_s=8.0, runner=None, clock=time.monotonic,
                 engine='auto', volume=1.0, voice_en='', voice_hi='', max_per_min=8, audio_device=''):
        self.enabled = enabled
        self.voice, self.speed, self.min_gap_s = voice, speed, min_gap_s
        self.volume = max(0.0, min(1.5, float(volume)))
        self.max_per_min = max_per_min
        self.clock = clock
        self.audio_device = audio_device
        self._last = {}
        self._recent = []
        self.spoken = []                       # recent phrases, for the app's log
        self.engine = 'none'
        self._runner = runner
        if runner is None:
            self._runner = self._pick(engine, voice_en, voice_hi)
        else:
            self.engine = 'custom'
        self.available = self._runner is not None
        self._q = queue.Queue(maxsize=4)
        self._worker = None

    # ------------------------------------------------------------ engines
    def _pick(self, engine, voice_en, voice_hi):
        piper = find_piper() if engine in ('auto', 'piper') else None
        if piper and shutil.which('aplay'):
            self.piper = piper
            self.voices = {'en': find_voice('en', voice_en), 'hi': find_voice('hi', voice_hi)}
            if self.voices['en'] or self.voices['hi']:
                self.engine = 'piper'
                return self._run_piper
        exe = (shutil.which('espeak-ng') or shutil.which('espeak')) if engine in ('auto', 'espeak') else None
        if exe:
            self.engine = 'espeak'
            self._espeak = exe
            return self._run_espeak
        return None

    def _run_piper(self, text):
        lang = 'hi' if DEVANAGARI.search(text) and self.voices.get('hi') else 'en'
        model = self.voices.get(lang) or self.voices.get('en') or self.voices.get('hi')
        rate = 22050
        cfg = model + '.json'
        if os.path.isfile(cfg):
            try:
                import json
                rate = int(json.load(open(cfg)).get('audio', {}).get('sample_rate', rate))
            except (ValueError, OSError):
                pass
        p = subprocess.run([self.piper, '--model', model, '--output_raw'], input=text.encode(),
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=30)
        cmd = ['aplay', '-q', '-r', str(rate), '-f', 'S16_LE', '-t', 'raw', '-c', '1']
        if self.audio_device:
            cmd[1:1] = ['-D', self.audio_device]
        subprocess.run(cmd + ['-'], input=scale_pcm(p.stdout, self.volume), stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=60)

    def _run_espeak(self, text):
        v = 'hi' if DEVANAGARI.search(text) else self.voice
        subprocess.run([self._espeak, '-v', v, '-s', str(self.speed), '-a', str(int(100 * self.volume)), text],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)

    # ------------------------------------------------------------ public
    def say(self, text, key=None, gap_s=None):
        """Speak unless the same key was spoken less than gap_s ago (or too much was said this minute).
        Returns True if queued."""
        if not (self.enabled and text):
            return False
        key = key or text
        now = self.clock()
        if now - self._last.get(key, -1e9) < (self.min_gap_s if gap_s is None else gap_s):
            return False
        self._recent = [t for t in self._recent if now - t < 60.0]
        if key not in PRIORITY_KEYS and len(self._recent) >= self.max_per_min:
            return False
        self._last[key] = now
        self._recent.append(now)
        self.spoken.insert(0, text)
        del self.spoken[10:]
        if self._runner:
            if self._q.full():
                try:
                    self._q.get_nowait()             # drop the oldest: say what matters now
                except queue.Empty:
                    pass
            self._q.put_nowait(text)
            if self._worker is None or not self._worker.is_alive():
                self._worker = threading.Thread(target=self._drain, daemon=True)
                self._worker.start()
        return True

    def set_volume(self, v):
        self.volume = max(0.0, min(1.5, float(v)))

    def busy(self):
        return not self._q.empty() or bool(self._worker and self._worker.is_alive() and getattr(self, '_talking', False))

    def _drain(self):
        while True:
            try:
                text = self._q.get(timeout=0.5)
            except queue.Empty:
                return
            self._talking = True
            try:
                self._runner(text)
            except Exception:
                pass
            finally:
                self._talking = False

    def wait(self, timeout=10.0):
        """Tests: until everything queued has been spoken."""
        t0 = time.monotonic()
        while (not self._q.empty() or getattr(self, '_talking', False)) and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
