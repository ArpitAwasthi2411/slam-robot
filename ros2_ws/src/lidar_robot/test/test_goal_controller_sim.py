"""Closed-loop simulation of the goal controller on a diff-drive robot with motor lag,
wheel deadband and encoder-free 'perfect' pose (SLAM pose in the real system)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lidar_robot.goal_logic import (GoalController, GoalParams, front_clearance_from_scan,  # noqa: E402
                                    SUCCEEDED, ABORTED, CANCELED)
from lidar_robot.kinematics import wrap_angle  # noqa: E402


class SimRobot:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, lag=0.15):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.lag = lag

    def step(self, v_cmd, w_cmd, dt):
        a = dt / (self.lag + dt)                       # first-order motor response
        self.v += a * (v_cmd - self.v)
        self.w += a * (w_cmd - self.w)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + self.w * dt)

    @property
    def pose(self):
        return self.x, self.y, self.yaw


def run(goal, start=(0, 0, 0), wall_x=None, t_max=90.0, dt=0.05, log=None):
    r = SimRobot(*start)
    c = GoalController(GoalParams())
    c.set_goal(*goal)
    t = 0.0
    while t < t_max and c.active:
        front = float('inf')
        if wall_x is not None and abs(wrap_angle(r.yaw)) < math.radians(25):
            front = wall_x - r.x
        x_before = r.x
        v, w = c.step(r.pose, front, dt)
        if log is not None:                       # pose at the time the command was decided
            log.append((t, x_before, r.y, r.yaw, v, w, c.state))
        r.step(v, w, dt)
        t += dt
    return r, c, t


def test_reaches_goals_in_all_directions():
    goals = [(2.0, 0.0, 0.0), (1.5, 1.5, math.pi / 2), (-1.0, 0.5, math.pi),
             (0.0, -2.0, -math.pi / 2), (0.3, 0.0, math.pi), (-2.0, -1.0, 0.7)]
    for g in goals:
        r, c, t = run(g)
        assert c.state == SUCCEEDED, (g, c.state, c.message, r.pose)
        assert math.hypot(r.x - g[0], r.y - g[1]) < 0.12, (g, r.pose)
        assert abs(wrap_angle(r.yaw - g[2])) < 0.15, (g, r.pose)
        assert t < 40, (g, t)


def test_stops_before_wall_and_aborts_when_blocked():
    log = []
    r, c, t = run((3.0, 0.0, 0.0), wall_x=1.0, log=log)
    p = GoalParams()
    assert c.state == ABORTED and 'blocked' in c.message
    assert r.x < 1.0 - p.stop_distance + 0.05, r.x            # never closer than stop distance (+ small coast)
    assert max(row[4] for row in log if row[1] > 1.0 - p.stop_distance) <= 1e-9


def test_cancel_stops_immediately():
    c = GoalController()
    c.set_goal(2, 0, 0)
    c.step((0, 0, 0), float('inf'), 0.05)
    c.cancel()
    assert c.state == CANCELED
    assert c.step((0, 0, 0), float('inf'), 0.05) == (0.0, 0.0)


def test_velocity_limits_respected():
    log = []
    run((3.0, 2.0, 1.0), log=log)
    p = GoalParams()
    assert max(abs(row[4]) for row in log) <= p.max_linear + 1e-9
    assert max(abs(row[5]) for row in log) <= p.max_angular + 1e-9
    assert min(row[4] for row in log) >= 0.0                  # never reverses blindly


def test_front_clearance_from_scan():
    n = 360
    ranges = [5.0] * n
    ranges[0] = 0.8                                            # straight ahead at angle_min=0
    ranges[90] = 0.3                                           # 90 deg left: outside cone
    d = front_clearance_from_scan(ranges, 0.0, 2 * math.pi / n, 0.15, 12.0)
    assert abs(d - 0.8) < 1e-6
    # laser mounted backwards (yaw = pi): the hit at index 180 is now in front
    ranges = [5.0] * n
    ranges[180] = 0.6
    d = front_clearance_from_scan(ranges, 0.0, 2 * math.pi / n, 0.15, 12.0, laser_yaw=math.pi)
    assert abs(d - 0.6) < 1e-6
