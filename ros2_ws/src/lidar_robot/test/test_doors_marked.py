"""21 Nov: glass doors are invisible to the LiDAR and pots sit under its beam. Doors and pots can be marked on the
saved map (app Map -> Paint): routes go through the middle of a marked door; a marked door found closed is a door
to be opened (the robot asks, waits up to 20 s and goes on the moment it opens), never a new wall, and the next
trip tries it again; a marked pot is always kept clear."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
import test_us_blocker as usb  # noqa: E402
from lidar_robot.map_edits import MapEdits  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.planner import GridMap, Planner, PlannerParams  # noqa: E402
from test_planner_follower import make_floor  # noqa: E402

DOOR = [[4.0, 2.55], [4.0, 3.55]]          # the 0.9 m door of make_floor(), frame to frame
CLOSED = (3.85, 2.6, 4.2, 3.5)             # the door leaf, seen only by the ultrasonics (glass)
GONE = (50.0, 50.0, 51.0, 51.0)


def nav_with_door(tmp_path, **door):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Window', 7.3, 3.0)

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t
    clk = Clock()
    nav = NavigatorCore(s, clock=clk, dwell_s=1.0)
    nav.map_edits = MapEdits(str(tmp_path / 'map_edits' / 'floor.json'))
    nav.map_edits.add('door', DOOR, **door)
    nav.set_map(nav.map_edits.apply(make_floor()))
    return nav, clk


def test_closed_door_robot_asks_waits_and_goes_when_opened(tmp_path, monkeypatch):
    nav, clk = nav_with_door(tmp_path, name='Lab 2', glass=True)
    start = (2.4, 3.05, 0.0)
    assert nav.handle({'type': 'command', 'text': 'go to window'}, start)['ok']
    monkeypatch.setattr(usb, 'BOX', CLOSED)
    pose, closest = usb.drive(nav, clk, start, 15.0)
    texts = ' | '.join(e['text'] for e in nav.events)
    said = [t for k, t in nav.say_queue if k == 'door']
    assert said == ['I am at the Lab 2 door and it is closed. Could someone open it, please?'], said
    assert 'waiting up to 20 s' in texts and nav._replans == 0, texts
    assert 'waiting at the Lab 2 door' in nav.message
    assert not any(abs(x - 3.9) < 0.4 and 2.5 < y < 3.6 for x, y in nav.us_memory())   # the door is not a wall
    assert closest > 0.05
    monkeypatch.setattr(usb, 'BOX', GONE)                                    # someone opens it
    moved = []
    pose2, _ = usb.drive(nav, clk, pose, 1.5, track=moved)
    assert math.hypot(moved[-1][0] - pose[0], moved[-1][1] - pose[1]) > 0.05, 'did not go on at once'
    pose3, _ = usb.drive(nav, clk, pose2, 60.0)
    assert nav.queue.history[-1].state == 'DONE'


def test_a_door_closed_too_long_is_closed_for_this_trip_only(tmp_path, monkeypatch):
    nav, clk = nav_with_door(tmp_path)
    start = (2.4, 3.05, 0.0)
    nav.handle({'type': 'command', 'text': 'go to window'}, start)
    monkeypatch.setattr(usb, 'BOX', CLOSED)
    pose, _ = usb.drive(nav, clk, start, 1.0)
    first = nav.queue.active.id
    usb.drive(nav, clk, pose, 25.0)
    texts = ' | '.join(e['text'] for e in nav.events)
    assert 'the door is still closed: looking for another way for this trip' in texts, texts
    assert list(nav._closed_doors.values()) == [first]
    m = nav.queue.active
    if m is not None and m.id == first:
        assert nav._closed_door_points()                                   # blocks this trip's replans
    nav.handle({'type': 'stop'}, start)
    monkeypatch.setattr(usb, 'BOX', GONE)
    nav.handle({'type': 'command', 'text': 'go to window'}, start)           # a new trip tries the door again
    assert nav._closed_door_points() == []
    nav.step(start, 5.0, 0.05)
    assert nav.follower.path and max(p[0] for p in nav.follower.path) > 6.0


def door_closed_while_mapping():
    """Corridor 6 x 3 m with a wall across at x = 3 and a 1 m door in it that was CLOSED while mapping."""
    res = 0.05
    occ = np.zeros((60, 120), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[:, 59:61] = True                                                    # wall with the closed door in it
    return GridMap(120, 60, res, 0.0, 0.0, occ, np.zeros_like(occ))


def test_a_marked_door_opens_the_map_and_routes_take_its_middle(tmp_path):
    grid = door_closed_while_mapping()
    p = Planner(PlannerParams())
    p.set_map(grid)
    assert p.plan((1.0, 1.5), (5.0, 1.5)) is None                           # mapped closed: no way through
    ed = MapEdits(str(tmp_path / 'e.json'))
    ed.add('door', [[3.0, 1.0], [3.0, 2.0]], glass=True)
    p.set_map(ed.apply(grid))
    path = p.plan((1.0, 0.6), (5.0, 2.4))
    assert path is not None
    at_door = [y for x, y in path if 2.9 <= x <= 3.1]
    assert at_door and all(abs(y - 1.5) < 0.15 for y in at_door), at_door  # through the middle, not by a post


def test_a_marked_pot_is_kept_clear(tmp_path):
    occ = np.zeros((60, 200), dtype=bool)                                  # 10 x 3 m corridor
    occ[0, :] = occ[-1, :] = True
    grid = GridMap(200, 60, 0.05, 0.0, 0.0, occ, np.zeros_like(occ))
    ed = MapEdits(str(tmp_path / 'e.json'))
    rec = ed.add('spot', [[5.0, 1.5]])
    assert rec['r'] == 0.25
    p = Planner(PlannerParams())
    p.set_map(ed.apply(grid))
    path = p.plan((1.0, 1.5), (9.0, 1.5))
    assert path and min(math.hypot(x - 5.0, y - 1.5) for x, y in path) > 0.25 + 0.30 - 0.05


def test_door_and_spot_input_is_checked(tmp_path):
    ed = MapEdits(str(tmp_path / 'e.json'))
    with pytest.raises(ValueError):
        ed.add('door', [[0, 0], [0, 0.2]])                                  # 20 cm: not a doorway
    with pytest.raises(ValueError):
        ed.add('door', [[0, 0], [5, 0]])
    with pytest.raises(ValueError):
        ed.add('spot', [])
    assert ed.add('spot', [[1, 1]], r=9)['r'] == 0.6
    d = ed.add('door', [[0, 0], [1, 0]], glass=1, name='  HOD cabin  ')
    assert d['glass'] is True and d['name'] == 'HOD cabin'
    again = MapEdits(str(tmp_path / 'e.json'))
    assert again.doors()[0]['name'] == 'HOD cabin' and again.edits[0]['r'] == 0.6
    assert again.near_door(0.5, 0.2)['id'] == d['id'] and again.near_door(0.5, 1.0) is None


def test_core_marks_doors_and_pots_from_the_app(tmp_path):
    from lidar_robot.navigator_core import NavigatorCore
    core = NavigatorCore(PlaceStore(str(tmp_path / 'places.json')))
    core.map_edits = MapEdits(str(tmp_path / 'map_edits' / 'm.json'))
    r = core.handle({'type': 'add_map_edit', 'kind': 'door', 'pts': [[0, 0], [0.9, 0]], 'glass': True,
                     'name': 'Lab 2'})
    assert r['ok'] and r['edit']['glass'] and r['edit']['name'] == 'Lab 2'
    r = core.handle({'type': 'add_map_edit', 'kind': 'spot', 'pts': [[2, 2]]})
    assert r['ok'] and r['edit']['r'] == 0.25
    assert any('glass door marked' in e['text'] for e in core.events)
