"""Map tools in the app: goal with heading (replace), multi-point routes, patrol loops."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_navigator_core import setup, run  # noqa: E402


def test_goal_with_heading_replaces_current_goal(tmp_path):
    nav, clk = setup(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 6.0, 'y': 1.0}, (1.0, 1.0, 0.0))
    run(nav, clk, (1.0, 1.0, 0.0), 2)
    r = nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 4.5, 'yaw': math.pi / 2, 'replace': True}, (1.0, 1.0, 0.0))
    assert r['ok']
    assert nav.queue.active is None and len(nav.queue.queue) == 1        # old goal gone, new one waiting
    x, y, yaw = run(nav, clk, (1.0, 1.0, 0.0), 120)
    assert math.hypot(x - 2.0, y - 4.5) < 0.25
    assert abs(math.atan2(math.sin(yaw - math.pi / 2), math.cos(yaw - math.pi / 2))) < 0.35


def test_route_through_points_in_order(tmp_path):
    nav, clk = setup(tmp_path)
    pts = [{'x': 2.0, 'y': 4.5}, {'x': 6.0, 'y': 1.0, 'yaw': 0.0}]
    r = nav.handle({'type': 'goto_poses', 'waypoints': pts}, (1.0, 1.0, 0.0))
    assert r['ok'] and r['mission']['stops'] == ['point 1', 'point 2']
    x, y, _ = run(nav, clk, (1.0, 1.0, 0.0), 300)
    assert math.hypot(x - 6.0, y - 1.0) < 0.25
    texts = [e['text'] for e in reversed(nav.events)]
    assert texts.index('arrived at point 1') < texts.index('arrived at point 2')
    assert nav.queue.history[0].state == 'DONE'


def test_route_repeats_then_stops(tmp_path):
    nav, clk = setup(tmp_path)
    pts = [{'x': 2.0, 'y': 1.0}, {'x': 1.0, 'y': 3.0}]
    nav.handle({'type': 'goto_poses', 'waypoints': pts, 'repeat': 1}, (1.0, 1.0, 0.0))
    run(nav, clk, (1.0, 1.0, 0.0), 300)
    m = nav.queue.history[0]
    assert m.state == 'DONE' and m.round == 2
    assert sum(1 for e in nav.events if e['text'] == 'arrived at point 1') == 2


def test_patrol_runs_until_canceled(tmp_path):
    nav, clk = setup(tmp_path)
    pts = [{'x': 2.0, 'y': 1.0}, {'x': 1.0, 'y': 3.0}]
    nav.handle({'type': 'goto_poses', 'waypoints': pts, 'repeat': -1}, (1.0, 1.0, 0.0))
    pose = (1.0, 1.0, 0.0)
    for _ in range(12):
        pose = run(nav, clk, pose, 20)
    assert nav.queue.active and nav.queue.active.round >= 3
    nav.handle({'type': 'cancel'}, pose)
    assert nav.queue.active is None


def test_route_input_checks(tmp_path):
    nav, _ = setup(tmp_path)
    assert not nav.handle({'type': 'goto_poses', 'waypoints': []})['ok']
    assert not nav.handle({'type': 'goto_poses', 'waypoints': [{'x': 1, 'y': 1}], 'repeat': 500})['ok']
    assert not nav.handle({'type': 'goto_poses', 'waypoints': [{'x': 'nan', 'y': 1}]})['ok']


def test_route_skips_unreachable_point(tmp_path):
    nav, clk = setup(tmp_path)
    pts = [{'x': 20.0, 'y': 1.5}, {'x': 2.0, 'y': 4.5}]         # first point is off the map
    nav.handle({'type': 'goto_poses', 'waypoints': pts}, (1.0, 1.0, 0.0))
    x, y, _ = run(nav, clk, (1.0, 1.0, 0.0), 200)
    assert math.hypot(x - 2.0, y - 4.5) < 0.25
    assert any(e['text'].startswith('skipping point 1') for e in nav.events)
    assert nav.queue.history[0].state == 'DONE'


def test_short_position_loss_pauses_then_continues(tmp_path):
    nav, clk = setup(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 6.0, 'y': 1.0}, (1.0, 1.0, 0.0))
    pose = run(nav, clk, (1.0, 1.0, 0.0), 3)
    for _ in range(int(5 / 0.05)):                     # 5 s without a position (ESP32 reset, SLAM hiccup)
        assert nav.step(None, float('inf'), 0.05) == (0.0, 0.0)
        clk.t += 0.05
    assert nav.queue.active is not None and nav.state == 'WAITING'
    x, y, _ = run(nav, clk, pose, 200)
    assert math.hypot(x - 6.0, y - 1.0) < 0.25 and nav.queue.history[0].state == 'DONE'
    assert any('position back' in e['text'] for e in nav.events)


def test_long_position_loss_fails_the_mission(tmp_path):
    nav, clk = setup(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 6.0, 'y': 1.0}, (1.0, 1.0, 0.0))
    run(nav, clk, (1.0, 1.0, 0.0), 2)
    for _ in range(int(31 / 0.05)):
        nav.step(None, float('inf'), 0.05)
        clk.t += 0.05
    assert nav.queue.active is None and nav.last_failure['code'] == 'POSE_LOST'
