"""Battery level from the ESP32's BAT line (INA226 or a voltage divider) — pure Python, tested.

    mon = BatteryMonitor(cells=3)
    mon.update(volts=11.4, amps=1.2)        # amps > 0 = discharging, < 0 = charging (on the dock)
    mon.info()  -> {'v': 11.4, 'pct': 62, 'a': 1.2, 'charging': False, 'level': 'ok', 'src': 'ina226'}

Levels: 'ok', 'low' (go home after the current task), 'critical' (go home now, refuse new tasks),
'charging', 'unknown' (no reading for 10 s or no sensor).
Voltage sags under load, so the level uses a slow average and only changes after it has been
below the threshold for a while (no false "battery low" when the motors start).
"""
import time

# resting voltage of one Li-ion cell -> % (typical 18650 curve)
CELL_CURVE = [(3.30, 0), (3.40, 4), (3.50, 9), (3.60, 20), (3.70, 38), (3.80, 53),
              (3.90, 66), (4.00, 79), (4.10, 90), (4.20, 100)]


def percent(volts, cells=3):
    v = volts / max(1, cells)
    if v <= CELL_CURVE[0][0]:
        return 0
    for (v0, p0), (v1, p1) in zip(CELL_CURVE, CELL_CURVE[1:]):
        if v <= v1:
            return int(round(p0 + (p1 - p0) * (v - v0) / (v1 - v0)))
    return 100


class BatteryMonitor:
    def __init__(self, cells=3, low_pct=25, critical_pct=12, charge_amps=0.08, tau_s=20.0,
                 confirm_s=15.0, stale_s=10.0, clock=time.monotonic):
        self.cells = cells
        self.low_pct = low_pct
        self.critical_pct = critical_pct
        self.charge_amps = charge_amps
        self.tau = tau_s
        self.confirm_s = confirm_s
        self.stale_s = stale_s
        self.clock = clock
        self.v_avg = None
        self.v = None
        self.a = None
        self.src = None
        self._t = None
        self._below = {'low': None, 'critical': None}
        self.level = 'unknown'

    def update(self, volts, amps=None, src='ina226'):
        now = self.clock()
        if volts is None or not (2.8 * self.cells <= volts <= 4.4 * self.cells):
            return                                   # unplugged sensor / floating pin / nonsense
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        self.v, self.a, self.src = float(volts), (None if amps is None else float(amps)), src
        if self.v_avg is None:
            self.v_avg = self.v
        else:
            k = min(1.0, dt / self.tau)
            self.v_avg += (self.v - self.v_avg) * k
        self._update_level(now)

    @property
    def charging(self):
        return self.a is not None and self.a < -self.charge_amps

    def pct(self):
        return None if self.v_avg is None else percent(self.v_avg, self.cells)

    def _update_level(self, now):
        if self.charging:
            self.level = 'charging'
            self._below = {'low': None, 'critical': None}
            return
        p = self.pct()
        for name, thr in (('critical', self.critical_pct), ('low', self.low_pct)):
            if p < thr:
                self._below[name] = self._below[name] or now
            elif p >= thr + 5:                            # hysteresis
                self._below[name] = None
        if self._below['critical'] and now - self._below['critical'] >= self.confirm_s:
            self.level = 'critical'
        elif self._below['low'] and now - self._below['low'] >= self.confirm_s:
            self.level = 'low'
        elif self.level in ('unknown', 'charging') or p >= self.low_pct + 5:
            self.level = 'ok'

    def current_level(self):
        if self._t is None or self.clock() - self._t > self.stale_s:
            return 'unknown'
        return self.level

    def info(self):
        lvl = self.current_level()
        return {'v': None if self.v is None else round(self.v, 2),
                'pct': self.pct() if lvl != 'unknown' else None,
                'a': None if self.a is None else round(self.a, 2),
                'charging': self.charging and lvl != 'unknown', 'level': lvl, 'src': self.src}
