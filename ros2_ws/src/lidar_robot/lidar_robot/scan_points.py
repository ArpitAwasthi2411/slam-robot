"""LiDAR scan -> points, with numpy (the per-ray Python loops cost ~5 ms per scan per node on a Pi 4).

The RPLiDAR sends ~1000-1500 rays per scan at 7-10 Hz and three nodes look at every scan, so these few
functions are among the hottest Python code on the robot. Each one gives the same answer as the
plain loop it replaces (test_scan_points.py checks that against the old code).
"""
import math

import numpy as np


class ScanGeometry:
    """cos/sin of every ray in base_link, cached: the angles are the same for every scan."""

    def __init__(self):
        self._key = None
        self.cos = self.sin = self.ang = None

    def get(self, n, angle_min, angle_increment, laser_yaw):
        key = (n, round(angle_min, 9), round(angle_increment, 12), round(laser_yaw, 9))
        if key != self._key:
            a = angle_min + np.arange(n, dtype=np.float64) * angle_increment + laser_yaw
            self.ang = np.arctan2(np.sin(a), np.cos(a))          # wrapped to (-pi, pi]
            self.cos, self.sin = np.cos(a), np.sin(a)
            self._key = key
        return self.ang, self.cos, self.sin


def _ranges(ranges):
    return np.asarray(ranges, dtype=np.float64)


def front_clearance(geo, ranges, angle_min, angle_increment, range_min, range_max,
                    laser=(0.0, 0.0, 0.0), half_angle=math.radians(25.0), robot_half_width=None):
    """Closest return ahead of the robot (base_link x), like goal_logic.front_clearance_from_scan."""
    r = _ranges(ranges)
    if r.size == 0:
        return float('inf')
    lx, _, lyaw = laser
    ang, c, s = geo.get(r.size, angle_min, angle_increment, lyaw)
    with np.errstate(invalid='ignore'):
        ok = (r > range_min) & (r < range_max) & np.isfinite(r) & (np.abs(ang) <= half_angle)
        if robot_half_width is not None:
            ok &= np.abs(r * s) <= robot_half_width
    if not ok.any():
        return float('inf')
    return float(np.min(r[ok] * c[ok]) + lx)


def obstacle_points(geo, ranges, angle_min, angle_increment, range_min, pose, laser=(0.0, 0.0, 0.0),
                    max_range=3.0, near_radius=1.0, world_every=2):
    """Nearby scan points for the navigator.

    Returns (world, near): world = every world_every-th ray closer than max_range, in the map frame
    (dynamic obstacles for replans); near = all points within near_radius of base_link, in base_link
    (body-shape checks). Lists of [x, y].
    """
    r = _ranges(ranges)
    if r.size == 0:
        return [], []
    lx, ly, lyaw = laser
    _, c, s = geo.get(r.size, angle_min, angle_increment, lyaw)
    with np.errstate(invalid='ignore'):
        ok = (r > range_min) & (r < max_range) & np.isfinite(r)
    rr = np.where(ok, r, 0.0)
    bx = lx + rr * c
    by = ly + rr * s
    near_m = ok & (bx * bx + by * by < near_radius * near_radius)
    near = np.stack([bx[near_m], by[near_m]], axis=1).tolist()
    world = []
    if pose is not None:
        idx = np.zeros(r.size, dtype=bool)
        idx[::max(1, int(world_every))] = True
        wm = ok & idx
        x, y, yaw = pose
        cy, sy = math.cos(yaw), math.sin(yaw)
        wx = x + cy * bx[wm] - sy * by[wm]
        wy = y + sy * bx[wm] + cy * by[wm]
        world = np.stack([wx, wy], axis=1).tolist()
    return world, near


def app_points(geo, ranges, angle_min, angle_increment, range_min, range_max, laser=(0.0, 0.0, 0.0),
               decimation=3):
    """Scan points in base_link for the app's map view: every decimation-th ray, rounded to 1 mm."""
    r = _ranges(ranges)
    if r.size == 0:
        return []
    lx, ly, lyaw = laser
    _, c, s = geo.get(r.size, angle_min, angle_increment, lyaw)
    idx = np.zeros(r.size, dtype=bool)
    idx[::max(1, int(decimation))] = True
    with np.errstate(invalid='ignore'):
        ok = idx & (r > range_min) & (r < range_max) & np.isfinite(r)
    pts = np.stack([lx + r[ok] * c[ok], ly + r[ok] * s[ok]], axis=1)
    return np.round(pts, 3).tolist()
