"""20 Nov lab: a door handle / a thing in a door stopped the robot; it then took a long way round (also on the way
back, from the remembered obstacle), and it stood still at walls. Now:
  - the only way (a door) is blocked: wait, and drive on the moment it clears (no failed trip, no detour),
  - a remembered obstacle is forgotten as soon as a sensor looks through its place and sees nothing,
  - a fresh plan never takes a long way round for an obstacle that is only remembered,
  - something straight ahead while the route bends away: turn towards the route, don't wait."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
import test_us_blocker as usb  # noqa: E402
from lidar_robot.follower import FollowerParams, PathFollower  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from test_planner_follower import make_floor  # noqa: E402

DOOR_BOX = (3.85, 2.6, 4.2, 3.5)          # fills the 0.9 m door at x = 4 (only the ultrasonics see it)
GONE = (50.0, 50.0, 51.0, 51.0)


def nav_left_room(tmp_path):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Window', 7.3, 3.0)

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t
    clk = Clock()
    nav = NavigatorCore(s, clock=clk, dwell_s=1.0)
    nav.set_map(make_floor())
    return nav, clk


def test_waits_at_a_blocked_door_and_goes_the_moment_it_clears(tmp_path, monkeypatch):
    nav, clk = nav_left_room(tmp_path)
    start = (2.4, 3.05, 0.0)
    assert nav.handle({'type': 'command', 'text': 'go to window'}, start)['ok']
    monkeypatch.setattr(usb, 'BOX', DOOR_BOX)
    track = []
    pose, closest = usb.drive(nav, clk, start, 12.0, track=track)
    texts = ' | '.join(e['text'] for e in nav.events)
    assert 'waiting up to 20 s for the way to clear' in texts, texts
    assert nav._replans == 0 and nav.queue.active is not None, texts       # no failed trip, no detour
    assert 'waiting for the way to clear' in nav.message
    assert closest > 0.05
    monkeypatch.setattr(usb, 'BOX', GONE)                                     # it is taken away
    moved = []
    pose2, _ = usb.drive(nav, clk, pose, 1.5, track=moved)
    assert math.hypot(moved[-1][0] - pose[0], moved[-1][1] - pose[1]) > 0.05, 'did not go on at once'
    pose3, _ = usb.drive(nav, clk, pose2, 60.0)
    assert nav.queue.history[-1].state == 'DONE'
    assert math.hypot(pose3[0] - 7.3, pose3[1] - 3.0) < 0.25


def test_a_remembered_obstacle_is_forgotten_when_a_beam_sees_through_it(tmp_path):
    nav, clk = nav_left_room(tmp_path)
    nav._pose = (2.0, 3.0, 0.0)
    nav.update_us_world([[(2.6, 3.0)]])
    assert nav.remember_us() == 1 and nav.us_memory()
    clk.t = 5.0
    assert nav.update_us_rays([(2.07, 3.0, 0.0, 0.53)]) >= 0                 # still echoing there: the thing stays
    assert any(abs(x - 2.6) < 0.06 and abs(y - 3.0) < 0.06 for x, y in nav.us_memory())
    nav.update_us_rays([(2.07, 3.0, 0.0, None)])                             # no echo any more
    near = [(x, y) for x, y in nav.us_memory() if abs(y - 3.0) < 0.06 and x < 2.9]   # inside the beam
    assert not near, near
    assert any('remembered obstacle gone' in e['text'] for e in nav.events)


def test_a_fresh_plan_does_not_go_the_long_way_round_a_remembered_obstacle(tmp_path):
    nav, clk = nav_left_room(tmp_path)
    nav._pose = (2.4, 3.05, 0.0)
    nav.update_us_world([[(3.5, y / 10) for y in range(25, 37)]])   # a thing that closed the door
    nav.us_margin_m = 0.15
    nav.remember_us()
    nav.handle({'type': 'command', 'text': 'go to window'}, (2.4, 3.05, 0.0))
    nav.step((2.4, 3.05, 0.0), 5.0, 0.05)
    assert nav.follower.path, nav.message
    assert any('trying it' in e['text'] for e in nav.events)
    xs = [p[0] for p in nav.follower.path]
    assert max(xs) > 6.0                                                     # straight through the door


def test_turns_towards_a_route_that_bends_away_from_a_wall():
    f = PathFollower(FollowerParams())
    f.set_path([(0.0, 0.0), (0.05, 0.3), (0.1, 0.6), (0.1, 1.2), (0.1, 2.0)])   # route goes left, wall ahead
    ws = []
    for _ in range(20):
        v, w, ev = f.step((0.0, 0.0, 0.0), 0.2, 0.05)
        ws.append(w)
        assert v <= 0.0 and ev is None
    assert max(ws) > 0.1 and not f.blocked                                 # turning left, not waiting
