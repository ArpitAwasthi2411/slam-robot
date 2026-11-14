"""Person or object in the way (blockers.py): ask and wait for a person, route around an object at once."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot.blockers import BlockerWatch  # noqa: E402
from lidar_robot.vision_rules import decide  # noqa: E402
from test_navigator_core import make_setup  # noqa: E402

PERSON = {'action': 'SLOW_DOWN', 'reason': 'person 1.2 m ahead', 'people_in_path': 1,
          'closest': {'bearing_deg': 3.0, 'distance_m': 1.2, 'in_path': True}}
NOBODY = {'action': 'CONTINUE', 'reason': 'no people in view', 'people_in_path': 0, 'closest': None}
ASIDE = {'action': 'CONTINUE', 'reason': 'people in view, none in my path', 'people_in_path': 0,
         'closest': {'bearing_deg': 40.0, 'distance_m': 1.5, 'in_path': False}}
FAR = {'action': 'CONTINUE', 'reason': 'people far ahead', 'people_in_path': 1,
       'closest': {'bearing_deg': 0.0, 'distance_m': 6.0, 'in_path': True}}


def run(w, seen, seconds, t0=0.0, dt=0.1):
    """Blocked for `seconds` with the camera saying `seen`; -> (last update, everything said)."""
    said, out, t = [], None, t0
    for _ in range(int(seconds / dt)):
        t += dt
        w.see_people(seen, t)
        out = w.update(True, t)
        said += out['say']
    return out, said, t


def test_person_is_asked_once_and_given_time():
    w = BlockerWatch(person_wait_s=8.0)
    out, said, t = run(w, PERSON, 3.0)
    assert out['kind'] == 'person' and out['limit'] == 8.0
    assert said == [('make_way', 'Excuse me, may I pass?')]            # once, not every tick
    end = w.update(False, t + 0.1)                                     # they stepped aside
    assert end['say'] == [('thanks', 'Thank you.')] and end['ended']['kind'] == 'person'
    assert w.counts['person'] == 1 and 2.9 < w.lost_s['person'] < 3.3


def test_object_is_not_spoken_to_and_rerouted_quickly():
    w = BlockerWatch(object_wait_s=1.2)
    out, said, _ = run(w, NOBODY, 0.8)
    assert out['kind'] is None and out['limit'] > 1.2 and said == []   # still deciding: no replan under it
    out, said, _ = run(w, NOBODY, 1.0, t0=0.8)
    assert out['kind'] == 'object' and out['limit'] == 1.2 and said == []
    assert w.replanned(2.0)['kind'] == 'object' and w.counts['object'] == 1


def test_person_beside_or_far_away_does_not_count():
    for seen in (ASIDE, FAR):
        w = BlockerWatch()
        assert run(w, seen, 2.0)[0]['kind'] == 'object'


def test_a_person_who_appears_late_changes_the_answer_and_stays():
    w = BlockerWatch()
    run(w, NOBODY, 1.0)
    out, said, t = run(w, PERSON, 0.6, t0=1.0)
    assert out['kind'] == 'person' and said == [('make_way', 'Excuse me, may I pass?')]
    out, _, _ = run(w, NOBODY, 1.0, t0=t)                              # missed for a few frames: still a person
    assert out['kind'] == 'person'


def test_without_a_camera_it_behaves_as_before():
    w = BlockerWatch()
    out, said, _ = run(w, None, 3.0)
    assert out['kind'] == 'unknown' and out['limit'] is None           # the follower's own blocked_after
    assert said == [('make_way', 'Excuse me, may I pass?')]
    w = BlockerWatch()
    assert run(w, decide([], 0), 3.0)[0]['kind'] == 'unknown'          # "no camera picture" is not "nobody"
    w = BlockerWatch()
    w.see_people(NOBODY, 0.0)                                          # an old decision does not count
    assert w.update(True, 5.0)['kind'] == 'unknown'


def test_real_decisions_from_the_vision_rules():
    w = BlockerWatch()
    person = {'id': 1, 'box': [300, 100, 380, 460], 'bearing_deg': 2.0, 'approaching': False, 'distance_m': 1.0}
    assert run(w, decide([person], 480), 1.0)[0]['kind'] == 'person'
    w = BlockerWatch()
    assert run(w, decide([], 480), 2.0)[0]['kind'] == 'object'


def test_a_flicker_is_not_counted_and_switch_off_works():
    w = BlockerWatch()
    w.see_people(PERSON, 0.0)
    w.update(True, 0.0)
    assert w.update(False, 0.2)['ended'] is None and w.counts['person'] == 0
    off = BlockerWatch(enabled=False)
    assert off.update(True, 1.0) == {'kind': None, 'limit': None, 'say': [], 'ended': None}


# ------------------------------------------------------------------ in the robot's core
class Bot:
    """The robot in the test room: moves as commanded, so it can turn onto its route and then meet the blocker."""
    def __init__(self, nav, clk, pose):
        self.nav, self.clk, self.pose = nav, clk, list(pose)

    def drive(self, seconds, front, people=None, dt=0.05):
        said = []
        for _ in range(int(seconds / dt)):
            if people is not None:
                self.nav.see_people(people)
            v, w = self.nav.step(tuple(self.pose), front, dt)
            self.pose[0] += v * math.cos(self.pose[2]) * dt
            self.pose[1] += v * math.sin(self.pose[2]) * dt
            self.pose[2] += w * dt
            said += [t for k, t in self.nav.say_queue if k in ('make_way', 'thanks')]
            self.nav.say_queue.clear()
            self.clk.t += dt
        return said


def start_trip(tmp_path):
    nav, clk = make_setup(tmp_path)
    nav.follower.p.blocked_after = 5.0
    bot = Bot(nav, clk, (1.0, 1.0, 0.0))
    assert nav.handle({'type': 'goto_place', 'name': 'Lab 3'}, tuple(bot.pose))['ok']
    for _ in range(200):                                               # until it is rolling along the route
        bot.drive(0.1, 5.0)
        if nav.state == 'DRIVING' and nav.follower._v > 0.05:
            break
    assert nav.state == 'DRIVING' and nav.follower._v > 0.05
    return nav, bot


def test_core_waits_for_a_person_then_thanks(tmp_path):
    nav, bot = start_trip(tmp_path)
    said = bot.drive(4.0, 0.2, PERSON)
    assert said == ['Excuse me, may I pass?']
    assert nav.state == 'DRIVING' and nav._replans == 0                # 4 s: still waiting, no new route
    assert 'person' in nav.status()['message'] and nav.status()['blocker']['now'] == 'person'
    said = bot.drive(1.0, 5.0, NOBODY)                     # they moved
    assert said == ['Thank you.'] and nav._replans == 0
    assert any('let me pass' in e['text'] for e in nav.events)
    assert nav.mlog.rec(nav.queue.active)['blocked_person'] == 1


def test_core_reroutes_around_an_object_without_the_long_wait(tmp_path):
    nav, bot = start_trip(tmp_path)
    said = bot.drive(2.0, 0.2, NOBODY)                     # 2 s is far less than blocked_after (5 s)
    assert said == [] and nav._replans == 1
    assert any('an object is in my way' in e['text'] for e in nav.events)
    assert nav.mlog.rec(nav.queue.active)['blocked_object'] == 1


def test_core_gives_up_on_a_person_who_stays(tmp_path):
    nav, bot = start_trip(tmp_path)
    bot.drive(9.5, 0.2, PERSON)                            # longer than person_wait_s (8 s)
    assert nav._replans == 1 and any('did not move' in e['text'] for e in nav.events)


def test_core_without_camera_keeps_the_old_timing(tmp_path):
    nav, bot = start_trip(tmp_path)
    said = bot.drive(4.0, 0.2)
    assert nav._replans == 0 and said == ['Excuse me, may I pass?']
    bot.drive(1.6, 0.2)
    assert nav._replans == 1
