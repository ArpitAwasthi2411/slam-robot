"""20 Nov: flower pots and plants on the department floor are below the LiDAR, only the ultrasonics see them.
Stopped nose-to-pot the robot must back up a little (only when the space behind is empty), then go round the
side. Routes should run down the middle of corridors (the pots stand along the walls)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_us_blocker import drive, make_nav  # noqa: E402
from lidar_robot.planner import GridMap, Planner, PlannerParams  # noqa: E402


def test_backs_up_then_goes_round_a_pot_only_the_ultrasonics_see(tmp_path):
    nav, clk = make_nav(tmp_path, 60.0)
    assert nav.handle({'type': 'command', 'text': 'go to window'}, (4.6, 3.0, 0.0))['ok']
    track = []
    (x, y, _), closest = drive(nav, clk, (4.6, 3.0, 0.0), 240, track=track)
    texts = ' | '.join(e['text'] for e in nav.events)
    assert nav.queue.history[-1].state == 'DONE', texts
    assert math.hypot(x - 7.3, y - 3.0) < 0.25, texts
    assert 'backing up 25 cm' in texts and 'going round the side' in texts, texts
    assert closest > 0.1, closest
    xs = [p[0] for p in track]
    stop_at = max(xs[:len(xs) // 3])                           # where it stopped in front of the pot
    assert any(stop_at - px > 0.15 for px in xs[xs.index(stop_at):]), 'never backed up'


def test_no_backing_into_something_behind(tmp_path):
    nav, clk = make_nav(tmp_path, 60.0)
    nav.handle({'type': 'command', 'text': 'go to window'}, (4.6, 3.0, 0.0))
    m = nav.queue.active or nav.queue.next()
    nav._backup = {'mid': m.id, 'x': 4.6, 'y': 3.0, 't0': 0.0, 'said': False}
    nav.update_scan_base([(0.3, 0.0), (-0.5, 0.05)])           # a chair 11 cm behind the back edge
    assert nav._backing_up(m, (4.6, 3.0, 0.0), 0.0) is None and nav._backup is None
    assert any('no room to back up' in e['text'] for e in nav.events)
    nav._backup = {'mid': m.id, 'x': 4.6, 'y': 3.0, 't0': 0.0, 'said': False}
    nav.update_scan_base([])                                   # no scan at all: never reverse blind
    assert nav._backing_up(m, (4.6, 3.0, 0.0), 0.0) is None
    nav._backup = {'mid': m.id, 'x': 4.6, 'y': 3.0, 't0': 0.0, 'said': False}
    nav.update_scan_base([(0.3, 0.0), (-1.5, 0.0)])            # wall far behind: back up
    v, w = nav._backing_up(m, (4.6, 3.0, 0.0), 0.0)
    assert v < 0 and w == 0.0


def test_person_in_the_way_is_not_backed_away_from(tmp_path):
    src = open(os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'navigator_core.py')).read()
    assert "if event == 'BLOCKED' and blocker != 'person' and self.backup_m > 0" in src


def test_routes_run_down_the_middle_of_a_corridor():
    """2 m wide corridor, start and goal 40 cm off the bottom wall (where the pots stand): the route
    moves to the middle in between."""
    res, w, h = 0.05, 240, 40                                  # 12 m x 2 m
    occ = np.zeros((h, w), dtype=bool)
    occ[0, :] = occ[-1, :] = True
    grid = GridMap(w, h, res, 0.0, 0.0, occ, np.zeros_like(occ))
    p = Planner(PlannerParams())
    p.set_map(grid)
    path = p.plan((1.0, 0.45), (11.0, 0.45))
    assert path is not None
    mid = [y for x, y in path if 4.0 < x < 8.0]
    assert mid and all(abs(y - 1.0) < 0.12 for y in mid), (min(mid), max(mid))


def test_footprint_is_the_46_cm_square():
    yaml = pytest.importorskip("yaml")
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'robot_params.yaml')
    rp = yaml.safe_load(open(cfg))['navigator']['ros__parameters']
    assert abs(rp['footprint_front'] + rp['footprint_rear'] - 0.46) < 1e-6
    assert abs(2 * rp['footprint_half_width'] - 0.46) < 1e-6
    assert rp['robot_radius'] > rp['footprint_half_width'] + 0.05
