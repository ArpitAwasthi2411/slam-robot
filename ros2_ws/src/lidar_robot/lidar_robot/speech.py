"""Robot voice: short spoken phrases through a speaker on the Pi (offline, espeak-ng).

    sudo apt install -y espeak-ng        (any USB speaker / 3.5 mm jack / I2S amp)

Rate-limited so a blocked robot doesn't repeat itself every second. Missing espeak-ng = silent,
never an error.
"""
import shutil
import subprocess
import threading
import time


class Speaker:
    def __init__(self, enabled=True, voice='en-in', speed=150, min_gap_s=8.0, runner=None, clock=time.monotonic):
        self.enabled = enabled
        self.voice, self.speed, self.min_gap_s = voice, speed, min_gap_s
        self.clock = clock
        self._last = {}
        self._runner = runner
        if self._runner is None:
            exe = shutil.which('espeak-ng') or shutil.which('espeak')
            self._runner = (lambda text: subprocess.run([exe, '-v', self.voice, '-s', str(self.speed), text],
                                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                        timeout=20)) if exe else None
        self.available = self._runner is not None
        self.spoken = []                       # recent phrases, for the app's log

    def say(self, text, key=None, gap_s=None):
        """Speak unless the same key was spoken less than gap_s ago. Returns True if spoken."""
        if not (self.enabled and text):
            return False
        key = key or text
        now = self.clock()
        if now - self._last.get(key, -1e9) < (self.min_gap_s if gap_s is None else gap_s):
            return False
        self._last[key] = now
        self.spoken.insert(0, text)
        del self.spoken[10:]
        if self._runner:
            threading.Thread(target=self._safe_run, args=(text,), daemon=True).start()
        return True

    def _safe_run(self, text):
        try:
            self._runner(text)
        except Exception:
            pass
