"""Tuning Lab back-end pieces (pure Python, no ROS; used by the dashboard, navigator and simulator).

- FOLLOWER_RANGES / WHEEL_RANGES: what the app may change, with safe limits
- clamp_values(): validate + clamp a dict coming from the app
- TuningStore: follower tuning saved to ~/maps/tuning.json (survives restarts)
- TelemetryBuffer: ring buffer of 50 Hz wheel samples with a sequence number (app polls ?since=)
- TestRunner: drives a scripted motion (straight step / spin) and records pose + odometry
- WheelSim: Python copy of the ESP32 speed loop + a motor model (simulator and unit tests)
"""
import json
import math
import os
import threading
import time
from collections import deque

# key: (min, max, default, unit, help)
FOLLOWER_RANGES = {
    'max_linear': (0.05, 0.60, 0.22, 'm/s', 'cruise speed'),
    'max_angular': (0.20, 2.00, 0.60, 'rad/s', 'fastest turn'),
    'lookahead': (0.15, 1.50, 0.50, 'm', 'bigger = smoother & straighter, smaller = tighter'),
    'k_angular': (0.20, 4.00, 1.00, '', 'steering gain: lower if it weaves'),
    'linear_accel': (0.05, 2.00, 0.30, 'm/s²', 'speed-up / slow-down rate'),
    'angular_accel': (0.20, 5.00, 1.20, 'rad/s²', 'how fast turns start'),
    'rotate_in_place_above': (0.30, 3.00, 1.20, 'rad', 'turn on the spot above this heading error'),
    'rotate_exit_below': (0.05, 1.50, 0.30, 'rad', '...until below this'),
}
WHEEL_RANGES = {
    'kp': (0.0, 2.0, 0.15, 'PWM per mm/s', 'proportional gain'),
    'ki': (0.0, 5.0, 0.40, 'PWM per mm', 'integral gain (removes steady error)'),
    'pwm_min': (0.0, 120.0, 15.0, 'PWM', 'feed-forward: PWM where the wheel starts'),
    'max_mms': (100.0, 2000.0, 493.0, 'mm/s', 'feed-forward: speed at max PWM'),
    'accel': (0.0, 5000.0, 600.0, 'mm/s²', 'ramp: 0 = instant (jerky)'),
}


def clamp_values(values, ranges):
    """Keep only known keys, as finite floats clamped to their range. Raises on junk."""
    if not isinstance(values, dict):
        raise ValueError('expected an object of values')
    out = {}
    for k, v in values.items():
        if k not in ranges:
            continue
        v = float(v)
        if not math.isfinite(v):
            raise ValueError(f'{k} must be a finite number')
        lo, hi = ranges[k][0], ranges[k][1]
        out[k] = min(hi, max(lo, v))
    return out


def ranges_for_app(ranges):
    return {k: {'min': r[0], 'max': r[1], 'default': r[2], 'unit': r[3], 'help': r[4]}
            for k, r in ranges.items()}


class TuningStore:
    """Follower tuning persisted as JSON. Missing/corrupt file = no overrides."""

    def __init__(self, path):
        self.path = os.path.expanduser(path) if path else None
        self.data = {'follower': {}}
        if self.path and os.path.isfile(self.path):
            try:
                with open(self.path) as f:
                    d = json.load(f)
                self.data['follower'] = clamp_values(d.get('follower', {}), FOLLOWER_RANGES)
            except (OSError, ValueError, TypeError):
                pass

    def save_follower(self, values):
        self.data['follower'].update(clamp_values(values, FOLLOWER_RANGES))
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(self.data, f, indent=2)
        os.replace(tmp, self.path)


def apply_follower(params, values):
    """Write clamped values onto a FollowerParams instance; returns what was applied."""
    vals = clamp_values(values, FOLLOWER_RANGES)
    for k, v in vals.items():
        setattr(params, k, v)
    if params.rotate_exit_below >= params.rotate_in_place_above:      # keep the hysteresis sane
        params.rotate_exit_below = 0.5 * params.rotate_in_place_above
    return vals


def follower_values(params):
    return {k: round(float(getattr(params, k)), 4) for k in FOLLOWER_RANGES}


class TelemetryBuffer:
    """Wheel samples [t_ms, tgtL, measL, pwmL, tgtR, measR, pwmR] with a running sequence number."""

    def __init__(self, maxlen=3000):
        self.lock = threading.Lock()
        self.buf = deque(maxlen=maxlen)
        self.seq = 0                      # sequence number of the NEXT sample
        self.t0 = time.monotonic()

    def add(self, sample, t=None):
        t = time.monotonic() if t is None else t
        with self.lock:
            self.buf.append((self.seq, [int((t - self.t0) * 1000)] + [int(x) for x in sample]))
            self.seq += 1

    def since(self, seq, limit=1500):
        with self.lock:
            out = [s for n, s in self.buf if n >= seq]
            return {'seq': self.seq, 'samples': out[-limit:]}


class TestRunner:
    __test__ = False                   # not a pytest test class
    """Run one scripted motion in a background thread and record what happened.

    send_cmd(v, w)    command the base (teleop priority)
    get_sample()      -> dict(x, y, yaw, v, w) or None
    is_blocked()      -> True stops the test (e-stop, RC override...)
    """
    KINDS = ('straight', 'spin')

    def __init__(self, send_cmd, get_sample, is_blocked=lambda: False, rate=20.0):
        self.send_cmd, self.get_sample, self.is_blocked = send_cmd, get_sample, is_blocked
        self.dt = 1.0 / rate
        self.lock = threading.Lock()
        self.thread = None
        self.stop_flag = threading.Event()
        self.result = {'id': 0, 'running': False}

    def start(self, req):
        kind = req.get('kind', 'straight')
        if kind not in self.KINDS:
            raise ValueError(f'kind must be one of {self.KINDS}')
        secs = min(15.0, max(0.5, float(req.get('secs', 3.0))))
        if kind == 'straight':
            v, w = min(0.5, max(-0.5, float(req.get('v', 0.2)))), 0.0
        else:
            v, w = 0.0, min(2.0, max(-2.0, float(req.get('w', 0.8))))
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError('v and w must be finite')
        with self.lock:
            if self.thread and self.thread.is_alive():
                raise ValueError('a test is already running')
            self.stop_flag.clear()
            tid = self.result.get('id', 0) + 1
            self.result = {'id': tid, 'running': True, 'kind': kind, 'v': v, 'w': w, 'secs': secs,
                           'samples': [], 'aborted': None}
            self.thread = threading.Thread(target=self._run, args=(v, w, secs), daemon=True)
            self.thread.start()
        return {'ok': True, 'id': tid}

    def stop(self):
        self.stop_flag.set()
        return {'ok': True}

    def status(self, since=0):
        with self.lock:
            r = dict(self.result)
            r['samples'] = list(self.result.get('samples', []))[since:]
        return r

    def _run(self, v, w, secs):
        t0 = time.monotonic()
        settle = 1.0                       # keep recording after the stop command
        aborted = None
        while True:
            t = time.monotonic() - t0
            if self.stop_flag.is_set():
                aborted = 'stopped by user'
            elif self.is_blocked():
                aborted = 'blocked (e-stop or manual override)'
            if aborted or t > secs + settle:
                break
            moving = t < secs
            self.send_cmd(v if moving else 0.0, w if moving else 0.0)
            s = self.get_sample()
            if s:
                with self.lock:
                    self.result['samples'].append([round(t, 3), s.get('x'), s.get('y'), s.get('yaw'),
                                                   s.get('v'), s.get('w'), v if moving else 0.0,
                                                   w if moving else 0.0])
            time.sleep(self.dt)
        self.send_cmd(0.0, 0.0)
        with self.lock:
            self.result['running'] = False
            self.result['aborted'] = aborted


class WheelSim:
    """One wheel: the firmware's ramp + feed-forward + PI loop driving a first-order motor."""

    def __init__(self, tune=None, gain_mms_per_pwm=2.6, tau=0.12, deadband_pwm=12.0, noise=6.0, seed=1):
        self.t = dict({k: r[2] for k, r in WHEEL_RANGES.items()}, **(tune or {}))
        self.gain, self.tau, self.dead, self.noise = gain_mms_per_pwm, tau, deadband_pwm, noise
        self.speed = 0.0            # true wheel speed mm/s
        self.meas = 0.0             # filtered measurement (firmware alpha 0.5)
        self.target = 0.0
        self.integ = 0.0
        self.pwm = 0
        import random
        self.rng = random.Random(seed)

    def ramp_to(self, cmd, dt, step_scale=1.0):
        acc = self.t['accel']
        d = cmd - self.target
        step = acc * dt * step_scale
        self.target = cmd if (acc <= 0 or abs(d) <= step) else self.target + math.copysign(step, d)

    def step(self, dt):
        t = self.t
        if abs(self.target) < 1.0:
            self.integ = 0.0
            out = 0.0
        else:
            err = self.target - self.meas
            self.integ += err * dt
            ilim = 60.0 / max(t['ki'], 0.01)
            self.integ = max(-ilim, min(ilim, self.integ))
            sgn = 1.0 if self.target > 0 else -1.0
            ff = t['pwm_min'] + abs(self.target) / t['max_mms'] * (200 - t['pwm_min'])
            out = sgn * ff + t['kp'] * err + t['ki'] * self.integ
            if out * sgn < 0:
                out = 0.0
            out = max(-200.0, min(200.0, out))
        self.pwm = int(out)
        eff = 0.0 if abs(out) < self.dead else math.copysign(abs(out) - self.dead, out)
        steady = eff * self.gain
        self.speed += (steady - self.speed) * min(1.0, dt / self.tau)
        noisy = self.speed + self.rng.gauss(0.0, self.noise)
        self.meas = 0.5 * noisy + 0.5 * self.meas
        return self.pwm
