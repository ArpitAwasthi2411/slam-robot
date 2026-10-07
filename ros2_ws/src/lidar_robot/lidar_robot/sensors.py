"""Ultrasonic geometry helpers (pure Python, unit-tested).

A mount is (x, y, yaw) of the sensor in base_link (x forward, y left). A reading is a range in metres
(None = nothing within range).
"""
import math


def range_point(rng, mount):
    """Obstacle point in base_link for a range reading along the sensor axis."""
    mx, my, myaw = mount
    return mx + rng * math.cos(myaw), my + rng * math.sin(myaw)


def front_clearance_from_ranges(readings, half_width, max_age=None):
    """readings: iterable of (range_m_or_None, mount). Returns the forward distance (from the robot
    centre) to the nearest ultrasonic obstacle that lies inside the robot's corridor, or inf."""
    best = float('inf')
    for rng, mount in readings:
        if rng is None or mount is None:
            continue
        px, py = range_point(rng, mount)
        if px > 0 and abs(py) <= half_width + 0.05:
            best = min(best, px)
    return best


def to_world(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]
