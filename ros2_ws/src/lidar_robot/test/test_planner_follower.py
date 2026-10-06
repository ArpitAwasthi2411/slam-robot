"""Planner on a synthetic floor (two rooms + 0.9 m doorway + corridor) and closed-loop following."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.planner import Planner, PlannerParams, GridMap, path_length  # noqa: E402
from lidar_robot.follower import PathFollower  # noqa: E402
from lidar_robot.kinematics import wrap_angle  # noqa: E402

RES = 0.05


def make_floor():
    """8 x 6 m. Left room, right room, wall at x=4 with a 0.9 m door at y in [2.6, 3.5]."""
    w, h = int(8 / RES), int(6 / RES)
    g = np.zeros((h, w), dtype=np.int16)

    def box(x0, y0, x1, y1, v=100):
        g[int(y0 / RES):int(y1 / RES) + 1, int(x0 / RES):int(x1 / RES) + 1] = v
    box(0, 0, 8, 0.05)
    box(0, 5.95, 8, 6)
    box(0, 0, 0.05, 6)
    box(7.95, 0, 8, 6)
    box(4.0, 0, 4.05, 2.6)       # wall with door 2.6 .. 3.5
    box(4.0, 3.5, 4.05, 6)
    box(1.5, 1.0, 2.5, 2.0)      # table in left room
    return GridMap.from_occupancy(w, h, RES, 0.0, 0.0, g.ravel().tolist())


def planner():
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(make_floor())
    return p


def test_plans_through_doorway_and_keeps_clear_of_walls():
    p = planner()
    path = p.plan((1.0, 1.0), (6.5, 4.5))
    assert path, p.last_error
    xs = [pt[0] for pt in path]
    assert min(xs) < 4.0 < max(xs)
    crossing = [pt for pt in path if 3.95 < pt[0] < 4.1]
    assert crossing and all(2.6 + 0.2 < pt[1] < 3.5 - 0.2 for pt in crossing), crossing
    assert min(p.clearance(x, y) for x, y in path[3:-3]) >= 0.27 - 0.06
    assert path_length(path) < 1.5 * math.hypot(5.5, 3.5)


def test_rejects_goal_in_wall_and_outside_map():
    p = planner()
    # deep inside the 1 x 1 m table -> rejected
    assert p.plan((1, 1), (2.0, 1.5)) is None and 'obstacle' in p.last_error
    assert p.plan((1, 1), (20, 20)) is None and 'outside' in p.last_error
    # clicked just inside a thin wall -> moved to the nearest safe spot, and says so
    path = p.plan((1, 1), (4.02, 1.0))
    assert path and 'moved' in p.last_note
    end = path[-1]
    assert math.hypot(end[0] - 4.02, end[1] - 1.0) <= 0.6 and p.clearance(*end) >= 0.27 - 0.06


def test_no_path_when_doorway_blocked_by_dynamic_obstacle():
    p = planner()
    blockers = [(4.0 + dx * 0.05, 2.6 + k * 0.05) for k in range(19) for dx in (-1, 0, 1)]
    assert p.plan((1.0, 1.0), (6.5, 4.5), extra_obstacles=blockers) is None
    assert 'no free path' in p.last_error


def test_routes_around_table():
    p = planner()
    path = p.plan((1.0, 1.5), (3.2, 1.5))
    assert path
    for x, y in path:
        assert not (1.5 - 0.2 < x < 2.5 + 0.2 and 1.0 - 0.2 < y < 2.0 + 0.2), (x, y)


def test_follower_drives_planned_path_to_goal():
    p = planner()
    path = p.plan((1.0, 1.0), (6.5, 4.5))
    f = PathFollower()
    f.set_path(path, final_yaw=math.pi / 2)
    x, y, yaw, v, w = 1.0, 1.0, 0.0, 0.0, 0.0
    t, dt, event = 0.0, 0.05, None
    worst = 9.0
    while t < 120 and event != 'ARRIVED':
        cmd_v, cmd_w, event = f.step((x, y, yaw), float('inf'), dt)
        a = dt / (0.15 + dt)
        v += a * (cmd_v - v)
        w += a * (cmd_w - w)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        worst = min(worst, p.clearance(x, y))
        t += dt
        assert event != 'OFF_PATH', (x, y)
    assert event == 'ARRIVED', (x, y, yaw, t)
    assert math.hypot(x - 6.5, y - 4.5) < 0.15
    assert abs(wrap_angle(yaw - math.pi / 2)) < 0.15
    assert worst > 0.15, worst            # never scraped a wall (robot centre >15 cm from obstacles)


def test_follower_reports_blocked():
    f = PathFollower()
    f.set_path([(0, 0), (1, 0), (2, 0)])
    ev = None
    for _ in range(100):
        v, w, ev = f.step((0.0, 0.0, 0.0), 0.2, 0.05)
        assert v == 0.0
        if ev:
            break
    assert ev == 'BLOCKED'
