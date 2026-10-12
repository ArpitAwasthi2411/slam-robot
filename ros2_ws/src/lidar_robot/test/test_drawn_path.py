"""Draw-a-path: smoothing, wall check, joining the robot to the path, following it, loops, blockages."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_navigator_core import setup, run  # noqa: E402
from test_planner_follower import planner  # noqa: E402
from lidar_robot import drawn_path  # noqa: E402
from lidar_robot.planner import path_length  # noqa: E402


def zigzag(x0, y0, x1, y1, n=40, wobble=0.04):
    """A shaky finger line from (x0, y0) to (x1, y1)."""
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n + (wobble if i % 2 else -wobble)) for i in range(n + 1)]


def test_prepare_smooths_and_spaces_points():
    pts = drawn_path.prepare(zigzag(1, 1, 3, 1))
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    assert max(gaps) <= 0.101 and min(gaps[:-1]) > 0.09
    assert max(abs(y - 1) for _, y in pts[2:-2]) < 0.03       # wobble mostly gone (ends stay where drawn)
    assert abs(pts[0][0] - 1) < 1e-6 and abs(pts[-1][0] - 3) < 1e-6


def test_prepare_rejects_junk():
    for bad in ([], [(1, 1)], [(1, 1), (float('nan'), 2)]):
        try:
            drawn_path.prepare(bad)
            assert False, bad
        except ValueError:
            pass


def test_check_flags_the_part_through_a_wall():
    p = planner()
    ok = drawn_path.prepare([(1.0, 3.0), (3.0, 3.0)])
    assert drawn_path.check(p, ok, 0.27) == []
    through = drawn_path.prepare([(3.0, 1.0), (5.0, 1.0)])     # crosses the wall at x=4
    bad = drawn_path.check(p, through, 0.27)
    assert len(bad) == 1 and 0.4 < bad[0][0] < 1.0 and 1.0 < bad[0][1] < 1.6


def test_route_from_pose_joins_robot_to_the_path():
    p = planner()
    path = drawn_path.prepare([(2.0, 3.0), (3.5, 3.0), (5.5, 3.0), (6.5, 4.5)])  # through the door
    r = drawn_path.route_from_pose(p, (1.0, 1.0, 0.0), path)
    assert r[0] == (1.0, 1.0) and path_length(r) > path_length(path)
    on = drawn_path.route_from_pose(p, (2.05, 3.0, 0.0), path)
    assert abs(path_length(on) - path_length(path)) < 0.1
    mid = drawn_path.route_from_pose(p, (3.0, 3.0, 0.0), path, from_nearest=True)
    assert path_length(mid) < path_length(path) - 0.9


def test_robot_follows_drawn_path_and_reports_errors(tmp_path):
    nav, clk = setup(tmp_path)
    pts = zigzag(1.0, 3.0, 3.5, 3.0) + zigzag(3.5, 3.0, 6.0, 3.0)[1:] + [(6.5, 4.5)]
    r = nav.handle({'type': 'check_path', 'points': pts}, (1.0, 1.0, 0.0))
    assert r['ok'] and r['blocked'] == [] and 5 < r['length'] < 8
    r = nav.handle({'type': 'follow_path', 'points': pts, 'yaw': math.pi / 2}, (1.0, 1.0, 0.0))
    assert r['ok'], r
    x, y, yaw = run(nav, clk, (1.0, 1.0, 0.0), 200)
    assert math.hypot(x - 6.5, y - 4.5) < 0.25
    assert nav.queue.history[0].state == 'DONE'
    bad = nav.handle({'type': 'follow_path', 'points': [(3.0, 1.0), (5.0, 1.0)]}, (x, y, yaw))
    assert not bad['ok'] and 'too close to a wall' in bad['error']


def test_drawn_path_loop(tmp_path):
    nav, clk = setup(tmp_path)
    loop = [(1.0, 0.6), (3.3, 0.6), (3.3, 4.5), (1.0, 4.5), (1.0, 0.6)]
    r = nav.handle({'type': 'follow_path', 'points': loop, 'repeat': 1}, (1.0, 0.6, 0.0))
    assert r['ok'], r
    run(nav, clk, (1.0, 0.6, 0.0), 400)
    m = nav.queue.history[0]
    assert m.state == 'DONE' and m.round == 2
