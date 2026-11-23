"""Guide mode: while staff have switched it on, a visitor (no PIN) can ask to be walked to a place;
the robot waits there a little and goes back home. Off = refused. Only one trip at a time."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_navigator_core import make_setup, run  # noqa: E402
from lidar_robot.auth import required_role  # noqa: E402

HOME = (1.0, 1.0, 0.0)


def guide_on(nav, **kw):
    req = {'type': 'guide_settings', 'on': True, 'by': 'arpit'}
    req.update(kw)
    return nav.handle(req, HOME)


def test_guide_off_by_default_and_refused(tmp_path):
    nav, clk = make_setup(tmp_path, data_dir=str(tmp_path))
    nav.step(HOME, float('inf'), 0.05)
    st = nav.status()['guide']
    assert st['on'] is False and st['ready'] is False
    r = nav.handle({'type': 'guide_to', 'name': 'Lab 3'}, HOME)
    assert r['ok'] is False and r['refused'] and 'staff' in r['error']
    assert nav.queue.active is None and not nav.queue.queue


def test_guide_trip_goes_there_waits_and_comes_home(tmp_path):
    nav, clk = make_setup(tmp_path, data_dir=str(tmp_path))
    nav.step(HOME, float('inf'), 0.05)
    assert guide_on(nav, wait_s=3)['guide']['ready']
    r = nav.handle({'type': 'guide_to', 'name': 'lab 3'}, HOME)
    assert r['ok'] and r['mission']['source'] == 'guide'
    m = nav.queue.active or nav.queue.queue[0]
    assert [s.label for s in m.stops] == ['Lab 3', 'home'] and m.stops[0].wait_s == 3
    assert any('follow me' in t for _, t in nav.say_queue)
    # a second visitor while it is out: refused, nothing queued behind
    r2 = nav.handle({'type': 'guide_to', 'name': 'Store'}, HOME)
    assert r2['ok'] is False and 'busy' in r2['error']
    x, y, _ = run(nav, clk, HOME, 300)
    assert math.hypot(x - 1.0, y - 1.0) < 0.25, (x, y)
    said = [t for _, t in nav.say_queue]
    assert any(t.startswith('This is Lab 3') for t in said), said


def test_only_allowed_places_and_never_home(tmp_path):
    nav, clk = make_setup(tmp_path, data_dir=str(tmp_path))
    nav.step(HOME, float('inf'), 0.05)
    st = guide_on(nav, places=['HOD office'])['guide']
    assert st['places'] == ['HOD office'] and st['limited']
    assert nav.handle({'type': 'guide_to', 'name': 'Lab 3'}, HOME)['ok'] is False
    assert nav.handle({'type': 'guide_to', 'name': 'home'}, HOME)['ok'] is False
    assert nav.handle({'type': 'guide_to', 'name': 'hod'}, HOME)['ok']      # alias of an allowed place
    r = nav.handle({'type': 'guide_settings', 'places': ['Nowhere']}, HOME)
    assert r['ok'] is False and 'not on this map' in r['error']


def test_time_limit_switches_off_and_setting_survives_restart(tmp_path):
    nav, clk = make_setup(tmp_path, data_dir=str(tmp_path))
    wall = [1000.0]
    nav.wall_clock = lambda: wall[0]
    st = guide_on(nav, hours=2)['guide']
    assert st['on'] and st['left_min'] == 120
    nav2, _ = make_setup(tmp_path, data_dir=str(tmp_path))     # restart: still on
    nav2.wall_clock = lambda: wall[0]
    assert nav2.status()['guide']['on']
    wall[0] += 2 * 3600 + 1
    assert nav2.status()['guide']['on'] is False
    assert nav2.handle({'type': 'guide_to', 'name': 'Lab 3'}, HOME)['ok'] is False


def test_roles():
    assert required_role('POST', '/api/nav', {'type': 'guide_to'}) == 'staff'
    assert required_role('POST', '/api/nav', {'type': 'guide_settings'}) == 'operator'
