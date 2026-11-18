"""18 Nov lab: narrow doors ("space on the map but no path"), ultrasonic stops at door frames, specks, rooms of
any shape."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot.planner import Planner, PlannerParams, GridMap, despeckle, fill_unknown_holes  # noqa: E402
from lidar_robot.sensors import front_clearance_from_ranges  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402

RES = 0.05


def floor(door=0.62, speck=False, unknown_streak=False):
    """8 x 6 m, wall at x = 4 with one door of the given width centred at y = 3."""
    w, h = int(8 / RES), int(6 / RES)
    g = np.zeros((h, w), dtype=np.int16)

    def box(x0, y0, x1, y1, v=100):
        g[int(round(y0 / RES)):int(round(y1 / RES)) + 1, int(round(x0 / RES)):int(round(x1 / RES)) + 1] = v
    box(0, 0, 8, 0.05)
    box(0, 5.95, 8, 6)
    box(0, 0, 0.05, 6)
    box(7.95, 0, 8, 6)
    box(4.0, 0, 4.05, 3 - door / 2 - RES)
    box(4.0, 3 + door / 2, 4.05, 6)
    if speck:                                    # one 5 cm speck in the middle of the corridor
        g[int(3.0 / RES), int(2.0 / RES)] = 100
    if unknown_streak:                           # the doorway's floor was never seen: a 1-cell unknown line
        g[int((3 - door / 2) / RES):int((3 + door / 2) / RES), int(4.0 / RES)] = -1
    return GridMap.from_occupancy(w, h, RES, 0.0, 0.0, g.ravel().tolist())


def test_despeckle_keeps_walls_and_drops_specks():
    occ = np.zeros((20, 20), bool)
    occ[5, 2:15] = True                          # a wall
    occ[12, 12] = True                           # a speck
    occ[15, 3] = occ[15, 4] = True               # a 2-cell speck
    out = despeckle(occ)
    assert out[5, 2:15].all()                    # whole wall, both ends kept
    assert not out[12, 12] and not out[15, 3] and not out[15, 4]


def test_unknown_doorway_streak_is_filled_but_frontier_kept():
    unk = np.zeros((10, 10), bool)
    unk[2:8, 5] = True                           # a 1-cell unseen streak between seen floor
    unk[:, 9] = True                             # the edge of the explored map
    occ = np.zeros_like(unk)
    out = fill_unknown_holes(unk, occ)
    assert not out[3:7, 5].any()
    assert out[:, 9].all()


def test_speck_no_longer_blocks_and_unknown_streak_no_longer_closes_a_door():
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(floor(door=0.9, speck=True, unknown_streak=True))
    assert p.plan((1.0, 3.0), (6.5, 3.0)) is not None


def _core(tmp_path, door=0.9):
    from lidar_robot.navigator_core import NavigatorCore
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Lab', 6.5, 3.0)
    t = [0.0]
    nav = NavigatorCore(s, clock=lambda: t[0], dwell_s=1.0)
    nav.set_map(floor(door=door))
    return nav, t


def test_door_frame_echo_does_not_close_the_door_for_60_s(tmp_path):
    """Stopped at a door, the left ultrasonic hears the frame. Its +-15 deg arc used to be remembered (grown 0.3 m
    deep + 0.15 m berth) and the next plans found "no path" through a door that is on the map."""
    from lidar_robot.sensors import range_arc, to_world
    nav, t = _core(tmp_path)
    pose = (3.75, 3.2, 0.0)
    nav._pose = pose
    arc = to_world(range_arc(0.21, (0.07, 0.12, math.radians(30))), pose)   # the echo off the door frame
    nav.update_us_world([arc])
    assert nav.remember_us(log=False) == 0                    # the map explains it: nothing remembered
    assert nav.planner.plan((3.0, 3.0), (6.5, 3.0), nav.us_memory()) is not None
    for pose in [(3.3, 3.25, 0.0), (3.5, 2.75, 0.0), (3.6, 3.3, 0.3)]:      # other approaches, both sides
        nav._pose = pose
        for mount in [(0.07, 0.12, math.radians(30)), (0.07, -0.12, math.radians(-30))]:
            for rng in (0.3, 0.5, 0.7):
                a = to_world(range_arc(rng, mount), pose)
                if any(nav.planner.near_wall(x, y, 0.12) for x, y in a):
                    nav.update_us_world([a])
                    nav.remember_us(log=False)
    assert nav.planner.plan((3.0, 3.0), (6.5, 3.0), nav.us_memory()) is not None


def test_box_in_the_open_is_still_remembered(tmp_path):
    from lidar_robot.sensors import range_arc, to_world
    nav, t = _core(tmp_path)
    pose = (1.5, 3.0, 0.0)
    nav._pose = pose
    nav.update_us_world([to_world(range_arc(0.35, (0.07, 0.0, 0.0)), pose)])   # a box ahead, far from walls
    assert nav.remember_us(log=False) > 0


def test_navigator_still_drives_through_a_normal_door(tmp_path):
    nav, t = _core(tmp_path, door=0.8)
    nav.handle({'type': 'goto_place', 'name': 'Lab'})
    for _ in range(20):
        t[0] += 0.05
        nav.step((1.0, 3.0, 0.0), float('inf'), 0.05)
    assert nav.state == 'DRIVING', nav.message


def test_door_frame_beside_the_robot_does_not_stop_it():
    mounts = [(0.07, 0.12, math.radians(30))]            # left sensor, angled 30 deg out
    frame = [(0.30, mounts[0])]                          # echo from the door frame on the left
    old = front_clearance_from_ranges(frame, 0.25)       # old: corridor half width 0.25 + 5 cm
    assert old < 0.4                                     # -> it stopped (stop distance 0.38)
    assert front_clearance_from_ranges(frame, 0.21, margin=0.03) == float('inf')   # real body: outside the path
    ahead = [(0.30, (0.07, 0.0, 0.0))]                   # a box straight ahead still stops it
    assert front_clearance_from_ranges(ahead, 0.21, margin=0.03) < 0.4
    assert front_clearance_from_ranges(ahead, 0.21, margin=0.03, ignore=lambda x, y: True) == float('inf')


def test_near_wall_ignores_unknown_space():
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(floor(door=0.9))
    assert p.near_wall(4.02, 1.0)                        # on the wall
    assert not p.near_wall(2.0, 3.0)                     # open floor


def test_room_of_any_shape(tmp_path):
    s = PlaceStore(str(tmp_path / 'p.json'))
    L = [[0, 0], [4, 0], [4, 1], [1, 1], [1, 3], [0, 3]]   # an L-shaped room
    s.add_room('Store', L, contents=['chairs'])
    assert s.room_at(0.5, 2.5)['name'] == 'Store'          # in the long arm
    assert s.room_at(3.0, 0.5)['name'] == 'Store'          # in the short arm
    assert s.room_at(3.0, 2.5) is None                     # in the corner the L leaves out


# ---------------------------------------------------------------- relative places ("corridor near Ravi sir's cabin")
def _cabin_floor(tmp_path):
    """Corridor y 0..2 (8 m long); cabin above it (x 2..5, y 2..5) with a 0.9 m door at x 3.5; the next lab
    (x 5..8, y 2..5) shares the cabin's right wall."""
    w, h = int(8 / RES), int(5 / RES)
    g = np.zeros((h, w), dtype=np.int16)

    def box(x0, y0, x1, y1):
        g[int(round(y0 / RES)):int(round(y1 / RES)) + 1, int(round(x0 / RES)):int(round(x1 / RES)) + 1] = 100
    box(0, 0, 8, 0.05)
    box(0, 4.95, 8, 5)
    box(0, 0, 0.05, 5)
    box(7.95, 0, 8, 5)
    box(2.0, 2.0, 3.05, 2.05)                     # cabin front wall, left of the door
    box(3.95, 2.0, 8.0, 2.05)                     # right of the door (and the lab's front wall)
    box(2.0, 2.0, 2.05, 5.0)
    box(5.0, 2.0, 5.05, 5.0)                      # wall between the cabin and the lab
    box(6.5, 2.0, 7.5, 2.05)
    grid = GridMap.from_occupancy(w, h, RES, 0.0, 0.0, g.ravel().tolist())
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Ravi sir cabin', 4.6, 3.5, aliases=['ravi sir'])
    s.add('Lab 3', 6.5, 3.5)
    s.add_room('Ravi sir cabin', [[2, 2], [5, 2], [5, 5], [2, 5]])
    s.add_room('Lab 3', [[5, 2], [8, 2], [8, 5], [5, 5]])
    s.add_room('Main corridor', [[0, 0], [8, 0], [8, 2], [0, 2]])
    return s, grid


def test_outside_spot_is_in_the_corridor_out_through_the_door(tmp_path):
    from lidar_robot.relative import outside_spot
    s, grid = _cabin_floor(tmp_path)
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(grid)
    spot = outside_spot(s.target('Ravi sir cabin'), s, p.is_free, p.clearance, need=0.39)
    assert spot is not None
    assert spot['y'] < 2.0 and s.room_at(spot['x'], spot['y'])['name'] == 'Main corridor'
    assert 2.6 < spot['x'] < 4.6                  # in front of the cabin door, not in the lab next door
    assert math.sin(spot['yaw']) > 0.5            # facing the cabin
    assert p.plan((1.0, 1.0), (spot['x'], spot['y'])) is not None


def test_rules_understand_near_and_ke_bahar(tmp_path):
    from lidar_robot.commands import CommandParser
    s, _ = _cabin_floor(tmp_path)
    c = CommandParser(s, policy='off')
    for text in ["stand in the corridor near Ravi sir's cabin", 'go outside ravi sir cabin and wait 2 minutes',
                 'ravi sir ke cabin ke bahar khade raho', 'Ravi sir cabin ke paas jao']:
        r = c.parse(text)
        assert r['intent'] == 'navigate', (text, r)
        assert r['targets'] == ['Ravi sir cabin'], (text, r)
        assert r['steps'][0].get('side') == 'outside', (text, r)
    r = c.parse('go to ravi sir cabin')
    assert r['steps'][0].get('side') is None
    r = c.parse('go outside ravi sir cabin and wait 2 minutes')
    assert r['steps'][0]['wait_s'] == 120


def test_llm_sides_pass_the_checks(tmp_path):
    from lidar_robot.commands import CommandParser
    s, _ = _cabin_floor(tmp_path)
    c = CommandParser(s, policy='off')
    d = {'intent': 'navigate', 'targets': ['Ravi sir cabin', 'Lab 3'], 'priority': 1, 'reply': 'ok',
         'waits': [0, 0], 'then_home': False, 'confirms': [False, False], 'sides': ['outside', 'at']}
    r = c.build(d, 'cabin ke bahar, phir lab 3', 'llm')
    assert r['steps'][0]['side'] == 'outside' and r['steps'][1].get('side') is None


def test_navigator_drives_to_the_corridor_spot(tmp_path):
    from lidar_robot.navigator_core import NavigatorCore
    s, grid = _cabin_floor(tmp_path)
    t = [0.0]
    nav = NavigatorCore(s, clock=lambda: t[0], dwell_s=1.0)
    nav.set_map(grid)
    res = nav.handle({'type': 'command', 'text': "wait in the corridor near Ravi sir's cabin"})
    assert res['ok'], res
    stop = nav.queue.active.stops[0] if nav.queue.active else nav.queue.queue[0].stops[0]
    assert stop.label == 'outside Ravi sir cabin'
    assert stop.y < 2.0
