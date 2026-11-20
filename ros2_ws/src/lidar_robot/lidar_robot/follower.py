"""Pure-pursuit path follower for a differential-drive robot (pure Python, simulated in tests).

    f = PathFollower(FollowerParams())
    f.set_path(points, final_yaw=None)
    v, w, event = f.step(pose, front_clearance, dt, guard=None)
        guard: footprint.FootprintGuard with the live scan (optional). With it the follower
               refuses turns on the spot that would swing the body into something, turns the
               other way or creeps forward first, and slows down in tight gaps (doors).
        event: None | 'ARRIVED' | 'BLOCKED' (stuck behind an obstacle, ask for a replan)
               | 'OFF_PATH' (pushed too far from the path, ask for a replan)
"""
import math
from dataclasses import dataclass

from lidar_robot.footprint import plan_manoeuvre
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
    side_slow_gap: float = 0.15      # m: body this close to anything (doors) -> slow down
    tight_speed: float = 0.6         # x max_linear on routes marked tight (body barely fits)
    # door sills / floor joints: going slowly through a door the caster can catch on the strip and the robot sits
    # still with the wheels asking to move (20 Nov). No progress for stall_s while driving forward -> a short push.
    stall_s: float = 0.8
    stall_move: float = 0.02         # m: less than this in stall_s = stuck
    kick_speed: float = 0.20         # m/s for kick_s
    kick_s: float = 0.5
    # a wall or door frame straight ahead while the route bends away from it (beyond the front cone): turn on the
    # spot towards the route instead of standing there "blocked" (20 Nov: waits at walls)
    bend_turn_alpha: float = 0.45


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
        self.blocked_limit = None    # s: set by the core while it knows what blocks (blockers.py); None = p.blocked_after
        self.blocked = False
        self._blocked_for = 0.0
        self._v = 0.0
        self._w = 0.0
        self._rot_dir = 0
        self._nudged = 0.0
        self._man = None
        self._man_tries = 0
        self.manoeuvres = 0          # 3-point turns done (for the log)
        self.tight = False
        self._stall = None
        self._kick = 0.0
        self.kicks = 0               # door-sill pushes (for the log)
        self.why_blocked = ''        # 'front' | 'turn' | 'body' (for "why did you stop?")

    def set_path(self, pts, final_yaw=None, tight=False):
        self.path = list(pts)
        self.final_yaw = final_yaw
        self.idx = 0
        self.aligning = False
        self.rotating = False
        self.blocked = False
        self._blocked_for = 0.0
        self._rot_dir = 0
        self._nudged = 0.0
        self._man = None
        self._man_tries = 0
        self.tight = tight
        self._stall = None
        self._kick = 0.0

    def _turn(self, err, guard, front_clearance, dt, pose=None, target=None):
        """Angular speed to turn on the spot by err, respecting the body sweep. -> (v, w, blocked)"""
        p = self.p
        if guard is not None:
            if self._man is not None:
                return self._run_manoeuvre(guard, pose)
            if self._rot_dir == 0:
                alt = err - math.copysign(2 * math.pi, err)        # the long way round
                if guard.rotation_clear(err):
                    self._rot_dir = 1 if err > 0 else -1
                elif guard.rotation_clear(alt):
                    self._rot_dir = 1 if alt > 0 else -1
            if self._rot_dir == 0 or not guard.rotation_clear(
                    err if (err > 0) == (self._rot_dir > 0) else err - math.copysign(2 * math.pi, err)):
                self._rot_dir = 0
                # boxed in: no room for the back to swing round. Plan a short 3-point turn
                # (pull ahead / back out / turn) that ends facing the way we need to go.
                if pose is not None and target is not None and self._man_tries < 2:
                    self._man_tries += 1
                    x, y, yaw = pose
                    c, s = math.cos(yaw), math.sin(yaw)
                    tb = (c * (target[0] - x) + s * (target[1] - y), -s * (target[0] - x) + c * (target[1] - y))
                    steps = plan_manoeuvre(guard, tb)
                    if steps:
                        self._man = {'steps': steps, 'i': 0, 'ref': pose}
                        self.manoeuvres += 1
                        return self._run_manoeuvre(guard, pose)
                self.why_blocked = 'turn'
                return 0.0, 0.0, True
            err = err if (err > 0) == (self._rot_dir > 0) else err - math.copysign(2 * math.pi, err)
        w_t = _clamp(p.k_angular * err, -p.max_angular, p.max_angular)
        if abs(w_t) < p.min_angular:
            w_t = math.copysign(p.min_angular, err)
        return 0.0, w_t, False

    def _run_manoeuvre(self, guard, pose):
        man = self._man
        while man['i'] < len(man['steps']):
            kind, amt = man['steps'][man['i']]
            rx, ry, ryaw = man['ref']
            if kind == 'move':
                done = (pose[0] - rx) * math.cos(ryaw) + (pose[1] - ry) * math.sin(ryaw)
                if abs(done) >= abs(amt) - 0.01:
                    man['i'] += 1
                    man['ref'] = pose
                    continue
                v = math.copysign(0.06, amt)
                w = _clamp(1.5 * wrap_angle(ryaw - pose[2]), -0.2, 0.2)
            else:
                done = wrap_angle(pose[2] - ryaw)
                if abs(done) >= abs(amt) - 0.03:
                    man['i'] += 1
                    man['ref'] = pose
                    continue
                v, w = 0.0, math.copysign(max(self.p.min_angular, 0.35), amt)
            if not guard.motion_clear(v, w, horizon=0.4):
                self._man = None                 # something moved in: stop and let the replanner decide
                self.why_blocked = 'turn'
                return 0.0, 0.0, True
            return v, w, False
        self._man = None
        self._rot_dir = 0
        return 0.0, 0.0, False

    def _sill_push(self, x, y, v_t, front_clearance, dt):
        """Stuck on a door sill: wheels asked to roll forward, robot not moving -> a short faster push."""
        p = self.p
        if self._kick > 0:
            self._kick -= dt
            self._stall = None
            if front_clearance > p.slow_distance * 0.75:
                self._v = max(self._v, p.kick_speed)        # straight away, no ramp: the point is to get over it
                return max(v_t, p.kick_speed)
            return v_t
        if self._v < p.min_linear * 0.8 or v_t < p.min_linear * 0.8:
            self._stall = None
            return v_t
        if self._stall is None:
            self._stall = [x, y, 0.0]
            return v_t
        self._stall[2] += dt
        if math.hypot(x - self._stall[0], y - self._stall[1]) > p.stall_move:
            self._stall = None
        elif self._stall[2] > p.stall_s and front_clearance > p.slow_distance * 0.75:
            self._kick = p.kick_s
            self.kicks += 1
            self._stall = None
        return v_t

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

    def step(self, pose, front_clearance, dt, guard=None):
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
            self._man = None
            self._rot_dir = 0
        if self.aligning:
            if self.final_yaw is None:
                event = 'ARRIVED'
            else:
                err = wrap_angle(self.final_yaw - yaw)
                if abs(err) < p.yaw_tolerance:
                    event = 'ARRIVED'
                else:
                    v_t, w_t, self.blocked = self._turn(err, guard, front_clearance, dt, pose, None)
                    v_t = 0.0                     # no creeping off the goal spot (no manoeuvre here)
                    if self.blocked and guard is not None:
                        event = 'ARRIVED'         # can't face the requested way here: good enough
                        self.blocked = False
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
                if front_clearance < p.stop_distance and abs(alpha) > p.bend_turn_alpha:
                    self.rotating = True        # the route bends away from the thing ahead: turn to it, don't wait
                    self._v = min(self._v, 0.0)
                if self.rotating:
                    v_t, w_t, self.blocked = self._turn(alpha, guard, front_clearance, dt, pose, (lx, ly))
                else:
                    self._rot_dir = 0
                    v_t = p.max_linear * max(0.2, 1.0 - abs(alpha) / p.rotate_in_place_above)
                    v_t = min(v_t, max(p.min_linear, p.max_linear * dist_end / p.slow_down_dist))
                    if self.tight:
                        v_t = min(v_t, max(p.min_linear, p.max_linear * p.tight_speed))
                    if guard is not None and guard.min_gap < p.side_slow_gap:     # doors, narrow gaps
                        v_t = min(v_t, max(p.min_linear, p.max_linear * max(0.5, guard.min_gap / p.side_slow_gap)))
                    if front_clearance < p.stop_distance:
                        v_t = 0.0
                        self.blocked = True
                        self.why_blocked = 'front'
                    elif front_clearance < p.slow_distance:
                        frac = (front_clearance - p.stop_distance) / (p.slow_distance - p.stop_distance)
                        v_t = max(p.min_linear, v_t * frac)
                    L = max(math.hypot(lx - x, ly - y), 1e-3)
                    curvature = 2.0 * math.sin(alpha) / L
                    w_t = _clamp(max(v_t, p.min_linear) * curvature, -p.max_angular, p.max_angular)
                    if guard is not None and not self.blocked and not guard.motion_clear(v_t, w_t):
                        # the body (usually the back, swinging out on a curve) would touch something
                        if guard.motion_clear(v_t, w_t * 0.4):
                            w_t *= 0.4
                        elif guard.motion_clear(p.min_linear, 0.0):
                            v_t, w_t = p.min_linear, 0.0
                        else:
                            self.blocked = True
                            self.why_blocked = 'body'
                    if self.blocked:
                        v_t = w_t = 0.0
                    else:
                        v_t = self._sill_push(x, y, v_t, front_clearance, dt)

        if self.blocked:
            self._blocked_for += dt
            if self._blocked_for > (p.blocked_after if self.blocked_limit is None else self.blocked_limit):
                event = 'BLOCKED'
                self._blocked_for = 0.0
        else:
            self._blocked_for = 0.0

        if event in ('ARRIVED', 'BLOCKED', 'OFF_PATH'):
            self._v = self._w = 0.0
            return 0.0, 0.0, event
        if self.blocked:
            self._v = self._w = 0.0                  # stop at once: no coasting into the obstacle
            return 0.0, 0.0, None
        else:
            self._v += _clamp(v_t - self._v, -p.linear_accel * dt, p.linear_accel * dt)
        self._w += _clamp(w_t - self._w, -p.angular_accel * dt, p.angular_accel * dt)
        return self._v, self._w, None
