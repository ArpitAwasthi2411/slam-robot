"""Raspberry Pi health: temperature, CPU use, load and the firmware's throttle flags.

A Pi 4 slows its CPU down at 80 C (and from 60 C on some firmware settings) and when its 5 V supply dips
below ~4.63 V. Both look like "the robot gets jerky and the map smears", so the app shows them.

    PiHealth().sample()  -> {'temp_c', 'cpu_pct', 'load1', 'cores', 'throttled', 'flags', 'level', 'text'}

Reads only files under /sys and /proc (no root, no subprocess every time); vcgencmd is the fallback for
the throttle flags when the kernel doesn't expose them.
"""
import os
import shutil
import subprocess
import threading
import time

TEMP_FILE = '/sys/class/thermal/thermal_zone0/temp'
THROTTLE_FILES = ('/sys/devices/platform/soc/soc:firmware/get_throttled',
                  '/sys/devices/platform/soc/soc:firmware/raspberrypi-hwmon/get_throttled')

# bit -> (now text, since-boot text)
FLAGS = {
    0: 'under-voltage now',
    1: 'CPU speed capped now',
    2: 'slowed down (throttled) now',
    3: 'soft temperature limit now',
    16: 'under-voltage happened since boot',
    17: 'CPU speed was capped since boot',
    18: 'was throttled since boot',
    19: 'soft temperature limit since boot',
}


def decode_throttled(value):
    """vcgencmd get_throttled value -> list of readable flags (now flags first)."""
    if value is None:
        return []
    return [text for bit, text in sorted(FLAGS.items()) if value & (1 << bit)]


def parse_throttled(text):
    """'throttled=0x50005' / '50005' / '0x0' -> int, or None."""
    if text is None:
        return None
    t = str(text).strip().split('=')[-1].strip()
    try:
        return int(t, 16)
    except ValueError:
        return None


def cpu_times(stat_text):
    """First line of /proc/stat -> (busy, total) jiffies."""
    parts = stat_text.split('\n', 1)[0].split()
    if not parts or parts[0] != 'cpu':
        return None
    v = [int(x) for x in parts[1:]]
    idle = v[3] + (v[4] if len(v) > 4 else 0)            # idle + iowait
    total = sum(v[:8])                                    # without guest time (already in user)
    return total - idle, total


def level_of(temp_c, throttled, cpu_pct):
    """'ok' | 'warn' | 'bad' and a short sentence for the app."""
    now = (throttled or 0) & 0xF
    if now & 0x1:
        return 'bad', 'Under-voltage now: the Pi slows down. Set the buck to 5.1-5.2 V at the Pi, use a short thick cable.'
    if temp_c is not None and temp_c >= 80:
        return 'bad', 'Too hot: the Pi slows itself down above 80 C. Add a heatsink + fan, open the vents.'
    if now:
        return 'bad', 'The Pi is slowing itself down (' + ', '.join(decode_throttled(now)) + ').'
    if temp_c is not None and temp_c >= 70:
        return 'warn', 'Getting hot: a fan or heatsink keeps it from slowing down.'
    if throttled and throttled & 0x10000:
        return 'warn', 'Under-voltage happened since boot: check the 5 V supply under load.'
    if cpu_pct is not None and cpu_pct >= 90:
        return 'warn', 'CPU almost fully used: close the camera view, or run on a saved map (localization).'
    return 'ok', 'OK'


class PiHealth:
    """Samples in the background every period_s; sample() is cheap and never blocks a web request."""

    def __init__(self, period_s=2.0, start_thread=True, root=''):
        self.period_s = period_s
        self.root = root                      # tests point this at a fake /sys + /proc tree
        self._prev = None
        self._vcgencmd = shutil.which('vcgencmd')
        self._last = {'temp_c': None, 'cpu_pct': None, 'load1': None, 'cores': os.cpu_count() or 1,
                      'throttled': None, 'flags': [], 'level': 'ok', 'text': 'measuring…'}
        self._lock = threading.Lock()
        if start_thread:
            threading.Thread(target=self._loop, daemon=True).start()

    def _read(self, path):
        try:
            with open(self.root + path) as f:
                return f.read()
        except OSError:
            return None

    def temp_c(self):
        t = self._read(TEMP_FILE)
        try:
            return round(int(t.strip()) / 1000.0, 1) if t else None
        except ValueError:
            return None

    def throttled(self):
        for p in THROTTLE_FILES:
            v = parse_throttled(self._read(p))
            if v is not None:
                return v
        if self._vcgencmd and not self.root:
            try:
                out = subprocess.run([self._vcgencmd, 'get_throttled'], capture_output=True, text=True,
                                     timeout=2).stdout
                return parse_throttled(out)
            except (OSError, subprocess.SubprocessError):
                return None
        return None

    def cpu_pct(self):
        st = self._read('/proc/stat')
        cur = cpu_times(st) if st else None
        prev, self._prev = self._prev, cur
        if not cur or not prev or cur[1] <= prev[1]:
            return None
        return round(100.0 * (cur[0] - prev[0]) / (cur[1] - prev[1]), 1)

    def load1(self):
        la = self._read('/proc/loadavg')
        try:
            return float(la.split()[0]) if la else None
        except (ValueError, IndexError):
            return None

    def measure(self):
        temp, thr, cpu, load = self.temp_c(), self.throttled(), self.cpu_pct(), self.load1()
        level, text = level_of(temp, thr, cpu)
        out = {'temp_c': temp, 'cpu_pct': cpu, 'load1': load, 'cores': os.cpu_count() or 1,
               'throttled': None if thr is None else hex(thr), 'flags': decode_throttled(thr),
               'level': level, 'text': text}
        with self._lock:
            self._last = out
        return out

    def sample(self):
        with self._lock:
            return dict(self._last)

    def _loop(self):
        while True:
            try:
                self.measure()
            except Exception:                 # never let health reporting take anything down
                pass
            time.sleep(self.period_s)
