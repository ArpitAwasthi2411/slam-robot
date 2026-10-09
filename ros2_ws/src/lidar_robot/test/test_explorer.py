"""Frontier exploration: frontier detection, goal choice, and a full simulated run that maps both rooms."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_planner_follower import make_floor, RES  # noqa: E402
from lidar_robot.explorer import ExploreParams, find_frontiers, choose_goal, mapped_area  # noqa: E402
from lidar_robot.planner import GridMap, Planner, PlannerParams  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.kinematics import wrap_angle  # noqa: E402

TRUTH = make_floor()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class Reveal:
    """A LiDAR that marks free/occupied cells of the true floor (like SLAM building the map)."""

    def __init__(self, max_range=2.5):
        h, w = TRUTH.occ.shape
        self.known = np.zeros((h, w), dtype=np.int8)     # 0 unknown, 1 free, 2 occupied
        self.max_range = max_range

    def scan(self, x, y):
        h, w = self.known.shape
        for i in range(240):
            a = i * 2 * math.pi / 240
            r = 0.0
            while r < self.max_range:
                gx, gy = int((x + r * math.cos(a)) / RES), int((y + r * math.sin(a)) / RES)
                if not (0 <= gx < w and 0 <= gy < h):
                    break
                if TRUTH.occ[gy, gx]:
                    self.known[gy, gx] = 2
                    break
                if self.known[gy, gx] != 2:
                    self.known[gy, gx] = 1
                r += RES * 0.7

    def grid(self):
        h, w = self.known.shape
        return GridMap(w, h, RES, 0.0, 0.0, self.known == 2, self.known == 0)


def known_free_fraction(rev):
    free_truth = ~TRUTH.occ
    return np.count_nonzero((rev.known == 1) & free_truth) / np.count_nonzero(free_truth)


def test_frontiers_on_half_known_map():
    rev = Reveal(max_range=3.0)
    rev.scan(1.0, 3.0)
    fr = find_frontiers(rev.grid(), min_cells=5)
    assert fr, 'expected frontiers at the edge of what the LiDAR saw'
    assert fr == sorted(fr, key=lambda f: -f.size)
    # fully known map: no frontiers
    full = GridMap(TRUTH.width, TRUTH.height, RES, 0, 0, TRUTH.occ, np.zeros_like(TRUTH.unknown))
    assert find_frontiers(full) == []
    assert mapped_area(full) > 40


def test_choose_goal_is_reachable_and_respects_blacklist():
    rev = Reveal(max_range=3.0)
    rev.scan(1.0, 3.0)
    g = rev.grid()
    p = Planner(PlannerParams(robot_radius=0.27))
    p.set_map(g)
    fr = find_frontiers(g, 5)
    pick = choose_goal(fr, p, (1.0, 3.0, 0.0), [])
    assert pick is not None
    x, y, f, path = pick
    assert path and math.hypot(path[-1][0] - x, path[-1][1] - y) < 0.2
    # blacklisting every frontier leaves nothing
    bl = [(f.cx, f.cy) for f in fr] + [(pt[0], pt[1]) for f in fr for pt in f.pts]
    assert choose_goal(fr, p, (1.0, 3.0, 0.0), bl, ExploreParams(blacklist_radius=0.8)) is None


def simulate(nav, clk, rev, pose, seconds, dt=0.05):
    x, y, yaw = pose
    v = w = 0.0
    last_map = -1.0
    for _ in range(int(seconds / dt)):
        if clk.t - last_map >= 1.0:
            rev.scan(x, y)
            nav.set_map(rev.grid())
            last_map = clk.t
        cv, cw = nav.step((x, y, yaw), float('inf'), dt)
        a = dt / (0.15 + dt)
        v += a * (cv - v)
        w += a * (cw - w)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + w * dt)
        clk.t += dt
        if (not nav.explore['active'] and nav.explore.get('state') in ('done', 'stopped')
                and not nav.queue.active and not nav.queue.queue):
            break
    return x, y, yaw


def make_nav(tmp_path):
    s = PlaceStore(str(tmp_path / 'p.json'))
    clk = Clock()
    nav = NavigatorCore(s, clock=clk, dwell_s=0.5)
    return nav, clk


def test_explores_both_rooms_and_returns(tmp_path):
    nav, clk = make_nav(tmp_path)
    rev = Reveal(max_range=2.5)
    start = (1.0, 3.0, 0.0)
    rev.scan(*start[:2])
    nav.set_map(rev.grid())
    before = known_free_fraction(rev)
    r = nav.handle({'type': 'explore_start', 'speed': 0.25}, start)
    assert r['ok'] and r['explore']['active']
    assert nav.follower.p.max_linear == 0.25
    states = set()
    x, y, yaw = start
    for _ in range(60):                                   # 60 x 20 s of simulated time max
        x, y, yaw = simulate(nav, clk, rev, (x, y, yaw), 20)
        states.add(nav.state)
        if not nav.explore['active'] and not nav.queue.active and not nav.queue.queue:
            break
    after = known_free_fraction(rev)
    st = nav.status()['explore']
    assert st['state'] == 'done', st
    assert st['goals_done'] >= 2, st
    assert after > 0.9 and after > before + 0.3, (before, after)
    # right room (behind the door) got mapped
    assert np.count_nonzero(rev.known[:, int(5 / RES):int(7.5 / RES)] == 1) > 500
    # came back to where it started and speeds were restored
    assert math.hypot(x - start[0], y - start[1]) < 0.3, (x, y)
    assert nav.follower.p.max_linear != 0.25
    texts = ' | '.join(e['text'] for e in nav.events)
    assert 'exploration finished' in texts


def test_scan_spin_turns_a_full_circle(tmp_path):
    nav, clk = make_nav(tmp_path)
    rev = Reveal(max_range=2.5)
    rev.scan(1.0, 3.0)
    nav.set_map(rev.grid())
    nav.handle({'type': 'explore_start'}, (1.0, 3.0, 0.0))
    x, y, yaw = 1.0, 3.0, 0.0
    turned, scanning_seen, dt = 0.0, False, 0.05
    for _ in range(int(200 / dt)):
        cv, cw = nav.step((x, y, yaw), float('inf'), dt)
        if nav.state == 'SCANNING':
            scanning_seen = True
            turned += abs(cw) * dt
        elif scanning_seen:
            break
        x += cv * math.cos(yaw) * dt
        y += cv * math.sin(yaw) * dt
        yaw = wrap_angle(yaw + cw * dt)
        clk.t += dt
    assert scanning_seen
    assert 2 * math.pi * 0.9 < turned < 2 * math.pi * 1.2, turned


def test_stop_and_cancel_end_exploration(tmp_path):
    nav, clk = make_nav(tmp_path)
    rev = Reveal(max_range=2.5)
    rev.scan(1.0, 3.0)
    nav.set_map(rev.grid())
    nav.handle({'type': 'explore_start'}, (1.0, 3.0, 0.0))
    simulate(nav, clk, rev, (1.0, 3.0, 0.0), 3)
    assert nav.queue.active and nav.queue.active.source == 'explore'
    r = nav.handle({'type': 'explore_stop'}, (1.0, 3.0, 0.0))
    assert r['explore']['state'] == 'stopped' and not nav.queue.active
    # cancel (the big app button) also ends exploration
    nav.handle({'type': 'explore_start'}, (1.0, 3.0, 0.0))
    simulate(nav, clk, rev, (1.0, 3.0, 0.0), 3)
    nav.handle({'type': 'cancel'}, (1.0, 3.0, 0.0))
    assert not nav.explore['active']


def test_explore_needs_a_map(tmp_path):
    nav, _ = make_nav(tmp_path)
    r = nav.handle({'type': 'explore_start'}, (1.0, 3.0, 0.0))
    assert not r['ok']


def test_autosave_once_when_exploration_finishes():
    import time
    from lidar_robot.explorer import MapAutoSaver
    saved = []
    a = MapAutoSaver(lambda n: saved.append(n) or f'/maps/{n}.yaml')
    a.update({'active': True, 'state': 'running'})
    a.update({'active': False, 'state': 'stopped'})          # user stop: no save
    a.update({'active': True, 'state': 'running'})
    a.update({'active': False, 'state': 'done'})
    a.update({'active': False, 'state': 'done'})             # repeated status: still one save
    for _ in range(50):
        if a.info['state'] == 'saved':
            break
        time.sleep(0.01)
    assert len(saved) == 1 and saved[0].startswith('explore_')
    assert a.info['state'] == 'saved' and a.info['result'].endswith('.yaml')
