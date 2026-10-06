"""Go-to-goal controller for a differential-drive robot (pure Python, simulated in tests).

Behaviour (same idea as RViz '2D Goal Pose' -> robot drives there):
  ROTATE   turn in place until facing the goal
  DRIVE    drive towards it, steering proportionally; slow down near obstacles
  ALIGN    turn in place to the requested final heading
  SUCCEEDED / CANCELED / ABORTED

BLOCKED is a pause inside DRIVE: an obstacle is closer than stop_distance in front.
It resumes by itself when the path clears, and aborts after blocked_timeout.

This is NOT a path planner: it drives a straight line. Walls between the robot and
the goal will block it (safely). Nav2 is the next step for real planning.
"""
import math
from dataclasses import dataclass

from lidar_robot.kinematics import wrap_angle

IDLE, ROTATE, DRIVE, ALIGN = 'IDLE', 'ROTATE', 'DRIVE', 'ALIGN'
SUCCEEDED, CANCELED, ABORTED = 'SUCCEEDED', 'CANCELED', 'ABORTED'
ACTIVE_STATES = (ROTATE, DRIVE, ALIGN)


@dataclass
class GoalParams:
    max_linear: float = 0.20          # m/s
    max_angular: float = 0.9          # rad/s
    min_linear: float = 0.04          # below this the motors stall
    min_angular: float = 0.25
    k_linear: float = 0.7
    k_angular: float = 1.8
    linear_accel: float = 0.4         # m/s^2
    angular_accel: float = 2.0        # rad/s^2
    xy_tolerance: float = 0.08        # m
    yaw_tolerance: float = 0.08       # rad (~4.6 deg)
    rotate_threshold: float = 0.35    # heading error that forces turn-in-place
    resume_threshold: float = 0.15    # stop turning in place below this error
    stop_distance: float = 0.35       # m from robot centre: front clearance that stops forward motion
    slow_distance: float = 0.75       # m, start slowing from here
    blocked_timeout: float = 15.0     # s
    goal_timeout: float = 120.0       # s
    use_final_yaw: bool = True


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _ramp(current, target, max_step):
    return current + _clamp(target - current, -max_step, max_step)


class GoalController:
    def __init__(self, params: GoalParams = None):
        self.p = params or GoalParams()
        self.state = IDLE
        self.goal = None              # (x, y, yaw)
        self.blocked = False
        self.message = ''
        self._v = 0.0
        self._w = 0.0
        self._elapsed = 0.0
        self._blocked_for = 0.0

    # -------------------------------------------------------------- commands
    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)
        self.state = ROTATE
        self.blocked = False
        self._elapsed = 0.0
        self._blocked_for = 0.0
        self.message = f'goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)'

    def cancel(self, why='canceled'):
        if self.state in ACTIVE_STATES:
            self.state = CANCELED
            self.message = why
        self._v = self._w = 0.0

    @property
    def active(self):
        return self.state in ACTIVE_STATES

    def distance_to_goal(self, pose):
        if self.goal is None:
            return None
        return math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])

    # -------------------------------------------------------------- control step
    def step(self, pose, front_clearance, dt):
        """pose = (x, y, yaw) in the goal frame; returns (v, w) to command."""
        p = self.p
        if not self.active:
            self._v = self._w = 0.0
            return 0.0, 0.0

        self._elapsed += dt
        if self._elapsed > p.goal_timeout:
            self.state, self.message = ABORTED, 'goal timeout'
            self._v = self._w = 0.0
            return 0.0, 0.0

        x, y, yaw = pose
        gx, gy, gyaw = self.goal
        dist = math.hypot(gx - x, gy - y)
        heading_err = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        v_t, w_t = 0.0, 0.0
        self.blocked = False

        if self.state in (ROTATE, DRIVE) and dist < p.xy_tolerance:
            self.state = ALIGN if p.use_final_yaw else SUCCEEDED

        if self.state == ROTATE:
            if abs(heading_err) < p.resume_threshold:
                self.state = DRIVE
            else:
                w_t = self._turn(heading_err)

        if self.state == DRIVE:
            if abs(heading_err) > p.rotate_threshold and dist > 3 * p.xy_tolerance:
                self.state = ROTATE
                w_t = self._turn(heading_err)
            else:
                v_t = _clamp(p.k_linear * dist, p.min_linear, p.max_linear)
                v_t *= max(0.0, math.cos(heading_err))
                # obstacle handling
                if front_clearance < p.stop_distance:
                    v_t = 0.0
                    self.blocked = True
                elif front_clearance < p.slow_distance:
                    frac = (front_clearance - p.stop_distance) / (p.slow_distance - p.stop_distance)
                    v_t = max(p.min_linear, v_t * frac)
                w_t = _clamp(p.k_angular * heading_err, -p.max_angular, p.max_angular)

        if self.state == ALIGN:
            yaw_err = wrap_angle(gyaw - yaw)
            if abs(yaw_err) < p.yaw_tolerance:
                self.state = SUCCEEDED
                self.message = 'goal reached'
            else:
                w_t = self._turn(yaw_err)

        if self.state == SUCCEEDED:
            self.message = 'goal reached'
            v_t = w_t = 0.0

        # blocked bookkeeping
        if self.blocked:
            self._blocked_for += dt
            self.message = f'blocked by obstacle ({front_clearance:.2f} m)'
            if self._blocked_for > p.blocked_timeout:
                self.state, self.message = ABORTED, 'blocked too long'
                v_t = w_t = 0.0
        else:
            self._blocked_for = 0.0

        # acceleration limits (stopping is allowed to be immediate)
        if v_t == 0.0 and (self.blocked or not self.active):
            self._v = 0.0
        else:
            self._v = _ramp(self._v, v_t, p.linear_accel * dt)
        self._w = _ramp(self._w, w_t, p.angular_accel * dt) if self.active else 0.0
        return self._v, self._w

    def _turn(self, err):
        p = self.p
        w = _clamp(p.k_angular * err, -p.max_angular, p.max_angular)
        if abs(w) < p.min_angular:
            w = math.copysign(p.min_angular, err)
        return w


def front_clearance_from_scan(ranges, angle_min, angle_increment, range_min, range_max,
                              laser_yaw=0.0, half_angle=math.radians(25.0),
                              robot_half_width=None, laser_x=0.0):
    """Closest valid return inside a forward cone (base frame), minus the laser's x offset.

    If robot_half_width is given, also only counts points inside the robot's corridor.
    """
    best = float('inf')
    a = angle_min
    for r in ranges:
        if range_min < r < range_max and math.isfinite(r):
            ab = wrap_angle(a + laser_yaw)
            if abs(ab) <= half_angle:
                fx = r * math.cos(ab) + laser_x
                if robot_half_width is None or abs(r * math.sin(ab)) <= robot_half_width:
                    best = min(best, fx)
        a += angle_increment
    return best
