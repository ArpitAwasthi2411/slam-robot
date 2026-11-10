"""Odometry guard: cleans the ESP32's ODM packets before they become /odom (pure Python, unit-tested).

Bad odometry is the main cause of a shifted or bent map, because Cartographer trusts it strongly
(docs/mapping.md). Four things go wrong on the real robot, and each one moved the map:

1. A garbled serial line that still parses ("ODM,12,13,..." -> "ODM,12,913,...") is a 15 cm jump.
   -> packets faster than the robot can physically move are rejected.
2. A lost line loses the ticks it carried (the packets hold changes, not totals).
   -> firmware v2.8 also sends running totals; the guard takes the change from the totals, so a lost
      line costs nothing. Older firmware works as before.
3. The gyro's zero drifts with temperature; the firmware measures it only once at power-up. Standing
   still, the heading creeps and the map rotates.
   -> when the wheels do not move the heading is held, and the leftover drift is learned and removed
      while driving.
4. A gyro glitch (I2C noise) reports an impossible turn rate.
   -> that packet's heading comes from the wheels instead.
"""
import math

INT32 = 1 << 32


def _wrap32(d):
    """Difference of two int32 counters that may have wrapped."""
    d %= INT32
    return d - INT32 if d >= INT32 // 2 else d


class OdomGuard:
    def __init__(self, wheel_radius, ticks_per_rev_left, ticks_per_rev_right, wheel_separation,
                 max_speed=1.2, max_turn=6.0, still_s=0.3, bias_limit=0.02, gyro_scale=1.0):
        circ = 2.0 * math.pi * wheel_radius
        self.m_per_tick = (circ / ticks_per_rev_left, circ / ticks_per_rev_right)
        self.sep = wheel_separation
        self.max_speed = max_speed            # m/s a wheel can never exceed (the motors top out near 0.5)
        self.max_turn = max_turn              # rad/s the robot can never exceed
        self.still_s = still_s                # wheels quiet this long = standing still
        self.bias_limit = bias_limit          # rad/s: a bigger "drift" is real motion or a broken gyro
        self.gyro_scale = gyro_scale
        self.bias = 0.0                       # learned gyro drift, rad/s
        self._still_for = 0.0
        self._totals = None                   # (tl, tr, tyaw_urad) of the last accepted packet
        self._last_now = None
        self.stats = {'rejected': 0, 'recovered_ticks': 0, 'recovered_packets': 0, 'gyro_glitches': 0,
                      'still': False}

    def reset_totals(self):
        """The ESP32 rebooted (its totals start again from zero)."""
        self._totals = None

    def _plausible(self, dl, dr, dt, slack=1.0):
        lim = self.max_speed * (dt + 0.02) * slack
        return abs(dl) * self.m_per_tick[0] <= lim and abs(dr) * self.m_per_tick[1] <= lim

    def update(self, pkt, now=None):
        """-> (dl_ticks, dr_ticks, dt_s, gyro_dtheta or None), or None when the packet is rejected."""
        dt = pkt.dt_ms / 1000.0
        dl, dr, dyaw = pkt.dl, pkt.dr, pkt.dyaw
        if not self._plausible(dl, dr, dt):
            self.stats['rejected'] += 1
            return None                       # totals (if any) are not trusted either: keep the old ones
        tl, tr, tyaw = getattr(pkt, 'tl', None), getattr(pkt, 'tr', None), getattr(pkt, 'tyaw', None)
        if tl is not None and tr is not None:
            if self._totals is not None:
                ddl, ddr = _wrap32(tl - self._totals[0]), _wrap32(tr - self._totals[1])
                if (ddl, ddr) != (dl, dr):
                    # lines were lost in between. Believe the totals when the missing part is something
                    # the robot could have driven in the time since the last good packet (3 s at most);
                    # otherwise the ESP32 restarted and its totals began again.
                    gap = 3.0 if (now is None or self._last_now is None) else min(3.0, now - self._last_now + 0.1)
                    if self._plausible(ddl - dl, ddr - dr, max(gap, 0.0)):
                        self.stats['recovered_ticks'] += abs(ddl - dl) + abs(ddr - dr)
                        self.stats['recovered_packets'] += 1
                        if dyaw is not None and tyaw is not None and self._totals[2] is not None:
                            dyaw = _wrap32(tyaw - self._totals[2]) * 1e-6
                        dl, dr = ddl, ddr
            self._totals = (tl, tr, tyaw)
        self._last_now = now

        still = abs(dl) <= 1 and abs(dr) <= 1
        self._still_for = self._still_for + dt if still else 0.0
        standing = self._still_for >= self.still_s
        self.stats['still'] = standing
        if dyaw is None:
            return dl, dr, dt, None

        rate = dyaw / dt if dt > 0 else 0.0
        if abs(rate) > self.max_turn:                      # impossible: use the wheels for this one
            self.stats['gyro_glitches'] += 1
            return dl, dr, dt, None
        if standing:
            if abs(rate) < self.bias_limit:                # learn the drift slowly (about 5 s to settle)
                a = min(1.0, dt / 5.0)
                self.bias += a * (rate - self.bias)
            return dl, dr, dt, 0.0                         # wheels still = heading held
        return dl, dr, dt, (dyaw - self.bias * dt) * self.gyro_scale
