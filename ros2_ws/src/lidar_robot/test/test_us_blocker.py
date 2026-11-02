"""A low box the LiDAR can't see: only the ultrasonics stop the robot. The navigator must remember the
ultrasonic blocker and plan around it (not replan the same straight line until it gives up)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_planner_follower import make_floor  # noqa: E402
from lidar_robot.kinematics import wrap_angle  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.sensors import front_clearance_from_ranges, range_arc, range_point, to_world  # noqa: E402

BOX = (5.7, 2.75, 6.1, 3.25)          # x0, y0, x1, y1: below the LiDAR plane, so not in the map or scan
MOUNTS = [(0.07, 0.0, 0.0), (0.07, 0.12, math.radians(30)), (0.07, -0.12, math.radians(-30))]
HALF_CONE = math.radians(15)


def in_box(x, y):
    return BOX[0] <= x <= BOX[2] and BOX[1] <= y <= BOX[3]


def ultrasonic(pose, max_range=2.0):
    """HC-SR04-like: nearest hit anywhere in a 30 deg cone, reported along the axis."""
    x, y, yaw = pose
    out = []
    for mx, my, myaw in MOUNTS:
        sx = x + mx * math.cos(yaw) - my * math.sin(yaw)
        sy = y + mx * math.sin(yaw) + my * math.cos(yaw)
        best = None
        for k in range(-5, 6):
            a = yaw + myaw + HALF_CONE * k / 5
            r = 0.02
            while r < max_range:
                if in_box(sx + r * math.cos(a), sy + r * math.sin(a)):
                    best = r if best is None else min(best, r)
                    break
                r += 0.02
        out.append((best, (mx, my, myaw)))
    return out


def make_nav(tmp_path, memory_s):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Window', 7.3, 3.0)
    s.set_home(4.6, 3.0, 0.0)

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t
    clk = Clock()
    nav = NavigatorCore(s, clock=clk, dwell_s=1.0)
    nav.set_map(make_floor())
    nav.us_memory_s = memory_s
    return nav, clk


def drive(nav, clk, pose, seconds, dt=0.05):
    x, y, yaw = pose
    v = w = 0.0
    closest = 9.0
    for _ in range(int(seconds / dt)):
        rd = ultrasonic((x, y, yaw))
        front = front_clearance_from_ranges(rd, 0.25)
        arc = [p for r, m in rd if r is not None and r <= 1.2 for p in range_arc(r, m)]
        nav.update_us_world(to_world(arc, (x, y, yaw)))
        nav.update_scan_base([range_point(r, m) for r, m in rd if r is not None])
        cv, cw = nav.step((x, y, yaw), front, dt)
        a = dt / (0.15 + dt)
        v += a * (cv - v)
        w += a * (cw - w)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        clk.t += dt
        cx, cy = min(max(x, BOX[0]), BOX[2]), min(max(y, BOX[1]), BOX[3])
        closest = min(closest, math.hypot(x - cx, y - cy))
        if nav.state == 'IDLE' and not nav.queue.active and not nav.queue.queue:
            break
    return (x, y, yaw), closest


def test_range_arc_spans_the_beam():
    pts = range_arc(1.0, (0.0, 0.0, 0.0), math.radians(15), step=0.05)
    ys = [p[1] for p in pts]
    assert min(ys) < -0.25 and max(ys) > 0.25 and len(pts) >= 10
    assert all(abs(math.hypot(*p) - 1.0) < 1e-9 for p in pts)


def test_memory_expires(tmp_path):
    nav, clk = make_nav(tmp_path, 60.0)
    nav.update_us_world([(1.0, 1.0)])
    nav.remember_us()
    assert nav.us_memory() == [(1.0, 1.0)]
    clk.t = 61.0
    assert nav.us_memory() == []


def test_drives_around_a_box_only_the_ultrasonics_see(tmp_path):
    nav, clk = make_nav(tmp_path, 60.0)
    r = nav.handle({'type': 'command', 'text': 'go to window'}, (4.6, 3.0, 0.0))
    assert r['ok'], r
    (x, y, _), closest = drive(nav, clk, (4.6, 3.0, 0.0), 240)
    texts = ' | '.join(e['text'] for e in nav.events)
    assert math.hypot(x - 7.3, y - 3.0) < 0.25, (x, y, texts)
    assert nav.queue.history[-1].state == 'DONE', texts
    assert 'ultrasonic blocker remembered' in texts
    assert closest > 0.1, closest                      # never drove into the box


def test_without_the_memory_it_keeps_hitting_the_same_box(tmp_path):
    nav, clk = make_nav(tmp_path, 0.0)                 # old behaviour: replans only see the LiDAR
    nav.rec.mode = 'off'
    nav.handle({'type': 'command', 'text': 'go to window'}, (4.6, 3.0, 0.0))
    (x, y, _), _ = drive(nav, clk, (4.6, 3.0, 0.0), 240)
    assert nav.queue.history[-1].state == 'FAILED'
    assert x < BOX[0]
