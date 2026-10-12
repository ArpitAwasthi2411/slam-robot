"""Paths drawn with a finger in the app: clean them up, check them against the map, join them to the robot.

    prepare(points)                      -> evenly spaced, smoothed [(x, y), ...]
    check(planner, path, robot_radius)   -> list of [start_m, end_m] stretches that are too close to walls
    route_from_pose(planner, pose, path, from_nearest) -> what the follower should drive, or None

Pure Python + the Planner; unit-tested in test/test_drawn_path.py.
"""
import math

from lidar_robot.planner import path_length

MAX_POINTS = 2000          # what the app may send
MAX_LENGTH = 200.0         # m


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def resample(pts, step=0.10):
    """Points every `step` metres along the polyline (keeps the first and last point)."""
    if len(pts) < 2:
        return list(pts)
    out = [tuple(pts[0])]
    need = step
    for a, b in zip(pts, pts[1:]):
        seg = _dist(a, b)
        pos = 0.0
        while seg - pos >= need:
            pos += need
            out.append((a[0] + (b[0] - a[0]) * pos / seg, a[1] + (b[1] - a[1]) * pos / seg))
            need = step
        need -= seg - pos
    if _dist(out[-1], pts[-1]) > 1e-6:
        out.append(tuple(pts[-1]))
    return out


def chaikin(pts, iterations=2):
    """Corner cutting: rounds a shaky finger line into something a robot can drive."""
    for _ in range(iterations):
        if len(pts) < 3:
            return pts
        out = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            out.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            out.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        out.append(pts[-1])
        pts = out
    return pts


def prepare(points, step=0.10):
    pts = []
    for p in points:
        x, y = float(p[0]), float(p[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('path points must be finite numbers')
        if not pts or _dist(pts[-1], (x, y)) > 0.01:
            pts.append((x, y))
    if len(pts) < 2:
        raise ValueError('draw a longer path')
    if len(points) > MAX_POINTS:
        raise ValueError('path has too many points')
    pts = chaikin(resample(pts, 0.15), 3)          # coarse first: removes finger jitter
    pts = resample(pts, step)
    if path_length(pts) > MAX_LENGTH:
        raise ValueError(f'path is longer than {MAX_LENGTH:.0f} m')
    return pts


def check(planner, path, robot_radius):
    """Stretches (in metres along the path) where the robot would not fit."""
    bad, s, start = [], 0.0, None
    for i, p in enumerate(path):
        if i:
            s += _dist(path[i - 1], p)
        ok = planner.clearance(p[0], p[1]) >= robot_radius
        if not ok and start is None:
            start = s
        if ok and start is not None:
            bad.append([round(start, 2), round(s, 2)])
            start = None
    if start is not None:
        bad.append([round(start, 2), round(s, 2)])
    return bad


def _nearest_index(path, xy):
    return min(range(len(path)), key=lambda i: _dist(path[i], xy))


def route_from_pose(planner, pose, path, from_nearest=False, extra_obstacles=(), rejoin_m=1.2):
    """Join the robot to the drawn path.

    Start of a run: plan to the path's first point if the robot isn't already on it.
    After a blockage (from_nearest): plan to a point `rejoin_m` further along the path than where the
    robot is, so it goes around the obstacle and then continues on the drawn line.
    """
    xy = (pose[0], pose[1])
    if from_nearest:
        i = _nearest_index(path, xy)
        acc, j = 0.0, i
        while j + 1 < len(path) and acc < rejoin_m:
            acc += _dist(path[j], path[j + 1])
            j += 1
        rest = path[j:]
        if len(rest) < 2:
            rest = path[-2:]
    else:
        rest = path
        if _dist(xy, rest[0]) <= 0.3:
            return [xy] + list(rest[1:])
    lead = planner.plan(xy, rest[0], extra_obstacles)
    if lead is None:
        return None
    return list(lead) + list(rest[1:])
