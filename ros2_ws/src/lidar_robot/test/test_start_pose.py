"""Fixed start position per saved map (23 Nov): the operator marks where the robot is parked, and after every
restart on that map the dashboard starts localization there (map_mode.get_start_pose / set_start_pose)."""
import json
import os

import pytest

from lidar_robot import map_mode


def test_set_get_clear(tmp_path):
    m = str(tmp_path / 'cse.pbstream')
    assert map_mode.get_start_pose(str(tmp_path), m) is None
    p = map_mode.set_start_pose(str(tmp_path), m, {'x': 1.234567, 'y': -2.0, 'yaw': 1.57}, by='arpit', clock=lambda: 100)
    assert (p['x'], p['y'], p['yaw'], p['by'], p['t']) == (1.2346, -2.0, 1.57, 'arpit', 100)
    assert map_mode.get_start_pose(str(tmp_path), m)['x'] == 1.2346
    assert map_mode.get_start_pose(str(tmp_path), str(tmp_path / 'other.pbstream')) is None   # one per map
    assert map_mode.set_start_pose(str(tmp_path), m, None) is None
    assert map_mode.get_start_pose(str(tmp_path), m) is None


def test_each_map_keeps_its_own(tmp_path):
    a, b = str(tmp_path / 'a.pbstream'), str(tmp_path / 'b.pbstream')
    map_mode.set_start_pose(str(tmp_path), a, {'x': 1, 'y': 1, 'yaw': 0})
    map_mode.set_start_pose(str(tmp_path), b, {'x': 5, 'y': 5, 'yaw': 3})
    assert map_mode.get_start_pose(str(tmp_path), a)['x'] == 1.0
    assert map_mode.get_start_pose(str(tmp_path), b)['yaw'] == 3.0


def test_bad_values_and_no_map(tmp_path):
    with pytest.raises(ValueError):
        map_mode.set_start_pose(str(tmp_path), '', {'x': 0, 'y': 0, 'yaw': 0})
    with pytest.raises(ValueError):
        map_mode.set_start_pose(str(tmp_path), 'm.pbstream', {'x': float('nan'), 'y': 0, 'yaw': 0})
    with open(os.path.join(str(tmp_path), map_mode.START_FILE), 'w') as f:
        json.dump({'m': {'x': 'oops'}}, f)                       # a broken file never breaks the start-up
    assert map_mode.get_start_pose(str(tmp_path), 'm.pbstream') is None
    with open(os.path.join(str(tmp_path), map_mode.START_FILE), 'w') as f:
        f.write('not json')
    assert map_mode.get_start_pose(str(tmp_path), 'm.pbstream') is None


def test_dashboard_uses_it_after_restart():
    src = open(os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'dashboard.py')).read()
    assert 'map_mode.get_start_pose(self.map_dir, self.map_file)' in src
    assert "self._start_kind = 'fixed'" in src            # a fresh Freeze pose still wins, then the fixed one
    app = open(os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app', 'app.js')).read()
    assert "action: 'set_start'" in app and "action: 'clear_start'" in app
