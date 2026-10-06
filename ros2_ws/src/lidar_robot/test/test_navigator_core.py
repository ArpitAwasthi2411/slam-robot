"""End-to-end NavigatorCore on the synthetic floor: commands -> missions -> plan -> drive -> arrive."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_planner_follower import make_floor  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.kinematics import wrap_angle  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def setup(tmp_path, **kw):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('HOD office', 6.5, 4.5, yaw=math.pi / 2, aliases=['hod'])
    s.add('Lab 3', 6.0, 1.0)
    s.add('Store', 1.0, 4.5)
    s.set_home(1.0, 1.0, 0.0)
    clk = Clock()
    nav = NavigatorCore(s, clock=clk, dwell_s=1.0, **kw)
    nav.set_map(make_floor())
    return nav, clk


def run(nav, clk, pose, seconds, front=lambda p: float('inf'), dt=0.05):
    x, y, yaw = pose
    v = w = 0.0
    for _ in range(int(seconds / dt)):
        cv, cw = nav.step((x, y, yaw), front((x, y, yaw)), dt)
        a = dt / (0.15 + dt)
        v += a * (cv - v)
        w += a * (cw - w)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        clk.t += dt
        if nav.state == 'IDLE' and not nav.queue.active and not nav.queue.queue:
            break
    return x, y, yaw


def test_command_multi_stop_then_home(tmp_path):
    nav, clk = setup(tmp_path)
    r = nav.handle({'type': 'command', 'text': 'take this to hod and then lab 3'}, (1.0, 1.0, 0.0))
    assert r['ok'] and r['interpretation']['targets'] == ['HOD office', 'Lab 3']
    x, y, yaw = run(nav, clk, (1.0, 1.0, 0.0), 200)
    assert math.hypot(x - 6.0, y - 1.0) < 0.2, (x, y)
    assert nav.queue.history[0].state == 'DONE'
    texts = ' | '.join(e['text'] for e in nav.events)
    assert 'arrived at HOD office' in texts and 'arrived at Lab 3' in texts
    nav.handle({'type': 'command', 'text': 'go home'}, (x, y, yaw))
    x, y, yaw = run(nav, clk, (x, y, yaw), 200)
    assert math.hypot(x - 1.0, y - 1.0) < 0.2


def test_preview_shows_path_without_moving(tmp_path):
    nav, clk = setup(tmp_path)
    nav.step((1.0, 1.0, 0.0), float('inf'), 0.05)
    r = nav.handle({'type': 'preview', 'name': 'hod'}, (1.0, 1.0, 0.0))
    assert r['ok'] and r['length'] > 5 and len(r['path']) > 5 and r['eta_s'] > 0
    assert nav.queue.active is None and nav.step((1.0, 1.0, 0.0), 9, 0.05) == (0.0, 0.0)


def test_unreachable_goal_fails_with_reason(tmp_path):
    nav, clk = setup(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    h = nav.queue.history[0]
    assert h.state == 'FAILED' and 'GOAL_IN_OBSTACLE' in h.message


def test_persistent_obstacle_gives_up_after_replans(tmp_path):
    nav, clk = setup(tmp_path, max_replans=2)
    nav.handle({'type': 'goto_place', 'name': 'Lab 3'})
    # something always 0.2 m in front of the robot
    run(nav, clk, (1.0, 1.0, 0.0), 60, front=lambda p: 0.2)
    h = nav.queue.history[0]
    assert h.state == 'FAILED' and 'BLOCKED' in h.message


def test_urgent_preempts_and_estop_pauses(tmp_path):
    nav, clk = setup(tmp_path)
    nav.handle({'type': 'goto_place', 'name': 'Lab 3', 'priority': 1})
    x, y, yaw = run(nav, clk, (1.0, 1.0, 0.0), 3)
    r = nav.handle({'type': 'command', 'text': 'urgent go to the store'}, (x, y, yaw))
    assert r['interpretation']['priority'] == 3
    queued = [(m.stops[0].label, m.message) for m in nav.queue.queue]
    assert queued[0][0] == 'Store'                                   # urgent one first
    assert ('Lab 3', 'paused for an urgent task') in queued           # interrupted one waits
    nav.set_estop(True)
    assert nav.step((x, y, yaw), 9, 0.05) == (0.0, 0.0) and nav.state == 'WAITING'
    nav.set_estop(False)
    assert nav.step((x, y, yaw), 9, 0.05) == (0.0, 0.0) and nav.hold      # no auto-resume
    assert nav.handle({'type': 'resume'})['ok']
    x, y, yaw = run(nav, clk, (x, y, yaw), 300)
    labels = [m.stops[0].label for m in nav.queue.history]
    assert labels[:2] == ['Lab 3', 'Store'] and all(m.state == 'DONE' for m in nav.queue.history[:2])


def test_places_editing_via_api(tmp_path):
    nav, clk = setup(tmp_path)
    nav.step((2.0, 3.0, 0.5), 9, 0.05)
    assert nav.handle({'type': 'add_place', 'name': 'Water cooler', 'here': True})['ok']
    p = nav.places.get('water cooler')
    assert p['x'] == 2.0 and p['y'] == 3.0
    res = nav.handle({'type': 'search', 'q': 'cooler'})['results']
    assert res[0]['name'] == 'Water cooler'
    assert nav.handle({'type': 'delete_place', 'name': 'Water cooler'})['ok']
    assert not nav.handle({'type': 'goto_place', 'name': 'cafeteria'})['ok']


def test_hold_is_only_cleared_by_resume_and_bad_input_rejected(tmp_path):
    nav, clk = setup(tmp_path)
    nav.set_estop(True)
    nav.set_estop(False)
    assert nav.handle({'type': 'goto_place', 'name': 'Lab 3'})['ok']          # queued...
    assert nav.step((1.0, 1.0, 0.0), 9, 0.05) == (0.0, 0.0) and nav.hold     # ...but not driving
    assert not nav.handle({'type': 'goto_pose', 'x': float('nan'), 'y': 1})['ok']
    assert not nav.handle(['not', 'a', 'dict'])['ok']
    assert nav.handle({'type': 'resume'})['ok']
    run(nav, clk, (1.0, 1.0, 0.0), 120)
    assert nav.queue.history[0].state == 'DONE'


def test_localization_needs_confirmation(tmp_path):
    nav, clk = setup(tmp_path)
    nav.localized = False
    nav.handle({'type': 'goto_place', 'name': 'Store'})
    assert nav.step((1.0, 1.0, 0.0), 9, 0.05) == (0.0, 0.0) and 'Position OK' in nav.message
    nav.handle({'type': 'confirm_localization'})
    run(nav, clk, (1.0, 1.0, 0.0), 120)
    assert nav.queue.history[0].state == 'DONE'
