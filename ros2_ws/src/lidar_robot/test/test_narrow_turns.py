"""Narrow places (23 Nov): when the square body cannot turn on the spot, the robot shuffles forward / back /
turn as often as it needs (up to max_manoeuvres) instead of reporting BLOCKED after two tries."""
import math

import numpy as np

from lidar_robot.follower import FollowerParams, PathFollower
from lidar_robot.footprint import Footprint, FootprintGuard
from lidar_robot.kinematics import wrap_angle

FP = Footprint(0.07, 0.39, 0.23, 0.04)


def corridor(width):
    return [(x, s * width / 2) for x in np.arange(-4.0, 4.0, 0.03) for s in (1, -1)]


def to_base(pts, pose):
    x, y, t = pose
    c, s = math.cos(t), math.sin(t)
    return [(c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)) for px, py in pts]


def run(pts, pose, path, max_manoeuvres, seconds=90.0):
    f = PathFollower(FollowerParams(max_linear=0.2, max_angular=0.3, k_angular=0.6, blocked_after=2.5,
                                    max_manoeuvres=max_manoeuvres))
    f.set_path(path)
    dt, hits, ev = 0.05, 0, None
    for _ in range(int(seconds / dt)):
        g = FootprintGuard(FP, [p for p in to_base(pts, pose) if p[0] ** 2 + p[1] ** 2 < 1.0])
        v, w, ev = f.step(pose, 5.0, dt, g)
        if ev in ('ARRIVED', 'BLOCKED', 'OFF_PATH'):
            break
        x, y, t = pose
        nxt = (x + v * math.cos(t) * dt, y + v * math.sin(t) * dt, wrap_angle(t + w * dt))
        if any(FP.signed_distance(bx, by) < 0 for bx, by in to_base(pts, nxt)):
            hits += 1
        else:
            pose = nxt
    return ev, hits, f.manoeuvres


def test_u_turn_in_a_narrow_corridor_shuffles_until_it_faces_back():
    pts = corridor(0.85)                                  # too narrow to turn on the spot (back swings 0.45 m)
    path = [(x, 0.0) for x in np.arange(0.0, -2.5, -0.05)]
    ev, hits, n = run(pts, (0.0, 0.0, 0.0), path, max_manoeuvres=6)
    assert ev == 'ARRIVED' and hits == 0 and n >= 2


def test_two_tries_were_not_enough_before():
    path = [(x, 0.0) for x in np.arange(0.0, -2.5, -0.05)]
    ev, _, _ = run(corridor(0.85), (0.0, 0.0, 0.0), path, max_manoeuvres=2)
    assert ev == 'BLOCKED'
