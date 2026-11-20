"""20 Nov lab: (1) the trip speed stayed low after the YAML was raised, (2) ~30 s stopped before it found a way
past someone, (3) the robot drove round a corridor spot the LiDAR missed while mapping, (4) going slowly through
a door the back wheel caught on the sill and the robot sat still."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot.follower import FollowerParams, PathFollower  # noqa: E402
from lidar_robot.planner import GridMap, Planner, PlannerParams, fill_enclosed_holes  # noqa: E402
from lidar_robot.tuning import saved_overrides  # noqa: E402
from test_blockers import PERSON, start_trip  # noqa: E402


# ------------------------------------------------------------------ (1) speed
def test_a_speed_raised_in_the_yaml_beats_an_old_saved_tuning_value():
    yaml_now = {'max_linear': 0.25, 'lookahead': 0.5}
    old_file = {'max_linear': 0.18, 'lookahead': 0.7}                 # saved before 20 Nov: no record of the YAML
    assert saved_overrides(old_file, yaml_now) == {'lookahead': 0.7}   # speed from the YAML, own look-ahead kept
    # saved after 20 Nov: the YAML at save time is recorded
    assert saved_overrides({'max_linear': 0.15}, yaml_now, {'max_linear': 0.25}) == {'max_linear': 0.15}
    assert saved_overrides({'max_linear': 0.15}, {'max_linear': 0.30}, {'max_linear': 0.25}) == {}


def test_navigator_uses_the_yaml_speed_over_an_old_tuning_file(tmp_path):
    import json
    from lidar_robot.navigator_core import NavigatorCore
    from lidar_robot.places import PlaceStore
    path = tmp_path / 'tuning.json'
    path.write_text(json.dumps({'follower': {'max_linear': 0.18, 'k_angular': 1.3}}))
    core = NavigatorCore(PlaceStore(str(tmp_path / 'p.json')), tuning_path=str(path),
                         follower_params=FollowerParams(max_linear=0.25))
    assert core.follower.p.max_linear == 0.25 and core.follower.p.k_angular == 1.3
    core.handle({'type': 'tune', 'follower': {'max_linear': 0.22}, 'save': True})
    again = NavigatorCore(PlaceStore(str(tmp_path / 'p.json')), tuning_path=str(path),
                          follower_params=FollowerParams(max_linear=0.25))
    assert again.follower.p.max_linear == 0.22                         # a value saved on purpose stays


# ------------------------------------------------------------------ (2) faster past a person
def test_a_person_who_stays_is_routed_round_and_forgotten_soon(tmp_path):
    nav, bot = start_trip(tmp_path)
    x, y, yaw = bot.pose
    nav.update_us_world([[[x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw)]]])
    bot.drive(nav.blockers.person_wait_s + 0.5, 0.2, PERSON)
    assert nav._replans == 1 and any('did not move' in e['text'] for e in nav.events)
    assert nav.us_memory()                                             # the new route goes round them...
    nav.update_us_world([])
    bot.clk.t += nav.person_memory_s + 0.5
    assert nav.us_memory() == []                                       # ...and they are forgotten in seconds


def test_block_timings_are_short():
    import yaml
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'robot_params.yaml')
    rp = yaml.safe_load(open(cfg))['navigator']['ros__parameters']
    assert rp['blocked_after'] <= 3.0 and rp['person_wait_s'] <= 5.0


# ------------------------------------------------------------------ (3) spots missed while mapping
def corridor_with_a_hole():
    res, w, h = 0.05, 240, 40                                          # 12 x 2 m corridor
    occ = np.zeros((h, w), dtype=bool)
    occ[0, :] = occ[-1, :] = True
    unk = np.zeros_like(occ)
    unk[14:26, 110:122] = True                                         # 60 x 60 cm never seen, mid-corridor
    return res, occ, unk


def test_a_small_unseen_spot_in_a_corridor_is_floor():
    res, occ, unk = corridor_with_a_hole()
    assert not fill_enclosed_holes(unk, occ, res).any()
    p = Planner(PlannerParams())
    p.set_map(GridMap(unk.shape[1], unk.shape[0], res, 0.0, 0.0, occ, unk))
    path = p.plan((1.0, 1.0), (11.0, 1.0))
    assert path and all(abs(y - 1.0) < 0.12 for x, y in path if 5.0 < x < 7.0)   # straight through, not round


def test_pillars_rooms_and_the_outside_stay_blocked():
    res = 0.05
    occ = np.zeros((100, 100), dtype=bool)
    unk = np.zeros_like(occ)
    unk[:, :10] = True                                                 # outside the map (touches the edge)
    occ[40:52, 40:52] = True
    unk[41:51, 41:51] = True                                           # inside a pillar: walls all round
    occ[60:61, 60:95] = occ[95:96, 60:95] = occ[60:96, 60:61] = occ[60:96, 95:96] = True
    occ[60:61, 70:80] = False                                          # a room seen only through its door
    unk[61:95, 61:95] = True
    out = fill_enclosed_holes(unk, occ, res)
    assert out[:, :10].all() and out[41:51, 41:51].all() and out[61:95, 61:95].all()


# ------------------------------------------------------------------ (4) door sill
def test_stuck_on_a_sill_gets_a_short_push():
    f = PathFollower(FollowerParams(max_linear=0.25, linear_accel=0.2))
    f.set_path([(0.1 * i, 0.0) for i in range(40)], tight=True)
    pose = (0.0, 0.0, 0.0)
    vs = []
    for _ in range(60):                                                # 3 s, the robot never moves
        v, w, ev = f.step(pose, 3.0, 0.05)
        vs.append(v)
    assert f.kicks >= 1 and max(vs) >= 0.2, max(vs)
    g = PathFollower(FollowerParams(max_linear=0.25, linear_accel=0.2))
    g.set_path([(0.1 * i, 0.0) for i in range(40)])
    x = 0.0
    for _ in range(60):                                                # rolling normally: no push
        v, w, ev = g.step((x, 0.0, 0.0), 3.0, 0.05)
        x += v * 0.05
    assert g.kicks == 0
