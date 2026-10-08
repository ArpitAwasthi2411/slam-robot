"""Pure-pursuit path follower for a differential-drive robot (pure Python, simulated in tests).

    f = PathFollower(FollowerParams())
    f.set_path(points, final_yaw=None)
    v, w, event = f.step(pose, front_clearance, dt)
        event: None | 'ARRIVED' | 'BLOCKED' (stuck behind an obstacle, ask for a replan)
               | 'OFF_PATH' (pushed too far from the path, ask for a replan)
"""
import math
from dataclasses import dataclass

from lidar_robot.kinematics import wrap_angle


@dataclass
class FollowerParams:
    max_linear: float = 0.22
    max_angular: float = 0.6
    min_linear: float = 0.05
    min_angular: float = 0.18
    lookahead: float = 0.50          # m (longer = straighter, smoother; shorter = hugs the path)
    rotate_in_place_above: float = 1.2   # rad: start turning on the spot above this heading error
    rotate_exit_below: float = 0.30      # rad: ...and keep turning until below this (hysteresis)
    xy_tolerance: float = 0.10
    yaw_tolerance: float = 0.10
    k_angular: float = 1.0
    linear_accel: float = 0.3
    angular_accel: float = 1.2
    slow_down_dist: float = 0.6      # m before the end of the path
    stop_distance: float = 0.38      # m front clearance from robot centre (440 mm chassis)
    slow_distance: float = 0.80
    blocked_after: float = 2.0       # s stuck before reporting BLOCKED
    off_path_dist: float = 0.6       # m cross-track error before reporting OFF_PATH


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PathFollower:
    def __init__(self, params: FollowerParams = None):
        self.p = params or FollowerParams()
        self.path = []
        self.final_yaw = None
        self.idx = 0
        self.aligning = False
        self.rotating = False
        self.blocked = False
        self._blocked_for = 0.0
        self._v = 0.0
        self._w = 0.0

    def set_path(self, pts, final_yaw=None):
        self.path = list(pts)
        self.final_yaw = final_yaw
        self.idx = 0
        self.aligning = False
        self.rotating = False
        self.blocked = False
        self._blocked_for = 0.0

    def clear(self):
        self.path = []
        self._v = self._w = 0.0

    def remaining_path(self):
        return self.path[self.idx:]

    def remaining_length(self, pose):
        if not self.path:
            return 0.0
        pts = [(pose[0], pose[1])] + self.path[self.idx + 1:]
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    def _advance(self, x, y):
        """Move idx to the closest path point at or after the current one (search a window)."""
        best_i, best_d = self.idx, math.inf
        for i in range(self.idx, min(len(self.path), self.idx + 60)):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best_i, best_d = i, d
        self.idx = best_i
        return best_d

    def _lookahead_point(self, x, y):
        for i in range(self.idx, len(self.path)):
            if math.hypot(self.path[i][0] - x, self.path[i][1] - y) >= self.p.lookahead:
                return self.path[i]
        return self.path[-1]

    def step(self, pose, front_clearance, dt):
        p = self.p
        if not self.path:
            return 0.0, 0.0, None
        x, y, yaw = pose
        gx, gy = self.path[-1]
        dist_end = math.hypot(gx - x, gy - y)
        v_t = w_t = 0.0
        event = None
        self.blocked = False

        if not self.aligning and dist_end < p.xy_tolerance:
            self.aligning = True
        if self.aligning:
            if self.final_yaw is None:
                event = 'ARRIVED'
            else:
                err = wrap_angle(self.final_yaw - yaw)
                if abs(err) < p.yaw_tolerance:
                    event = 'ARRIVED'
                else:
                    w_t = _clamp(p.k_angular * err, -p.max_angular, p.max_angular)
                    if abs(w_t) < p.min_angular:
                        w_t = math.copysign(p.min_angular, err)
        else:
            cross = self._advance(x, y)
            if cross > p.off_path_dist:
                event = 'OFF_PATH'
            else:
                lx, ly = self._lookahead_point(x, y)
                alpha = wrap_angle(math.atan2(ly - y, lx - x) - yaw)
                # hysteresis: no chattering between "turn on the spot" and "drive"
                if abs(alpha) > p.rotate_in_place_above:
                    self.rotating = True
                elif abs(alpha) < p.rotate_exit_below:
                    self.rotating = False
                if self.rotating:
                    w_t = _clamp(p.k_angular * alpha, -p.max_angular, p.max_angular)
                    if abs(w_t) < p.min_angular:
                        w_t = math.copysign(p.min_angular, alpha)
                else:
                    v_t = p.max_linear * max(0.2, 1.0 - abs(alpha) / p.rotate_in_place_above)
                    v_t = min(v_t, max(p.min_linear, p.max_linear * dist_end / p.slow_down_dist))
                    if front_clearance < p.stop_distance:
                        v_t = 0.0
                        self.blocked = True
                    elif front_clearance < p.slow_distance:
                        frac = (front_clearance - p.stop_distance) / (p.slow_distance - p.stop_distance)
                        v_t = max(p.min_linear, v_t * frac)
                    L = max(math.hypot(lx - x, ly - y), 1e-3)
                    curvature = 2.0 * math.sin(alpha) / L
                    w_t = _clamp(max(v_t, p.min_linear) * curvature, -p.max_angular, p.max_angular)
                    if self.blocked:
                        w_t = 0.0

        if self.blocked:
            self._blocked_for += dt
            if self._blocked_for > p.blocked_after:
                event = 'BLOCKED'
                self._blocked_for = 0.0
        else:
            self._blocked_for = 0.0

        if event in ('ARRIVED', 'BLOCKED', 'OFF_PATH'):
            self._v = self._w = 0.0
            return 0.0, 0.0, event
        if v_t == 0.0 and self.blocked:
            self._v = 0.0
        else:
            self._v += _clamp(v_t - self._v, -p.linear_accel * dt, p.linear_accel * dt)
        self._w += _clamp(w_t - self._w, -p.angular_accel * dt, p.angular_accel * dt)
        return self._v, self._w, None
