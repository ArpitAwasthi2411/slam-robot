"""Hand edits of a saved map (app Map tab -> Paint), for route planning only.

20 Nov: the department map saw the cabins only from their doors, so trips into a cabin said "no route". Painting
the cabin floor as drivable must let the planner in, without touching scanned walls or the map file."""
import json
import os

import numpy as np
import pytest

from lidar_robot.map_edits import MapEdits, edits_path
from lidar_robot.planner import GridMap, Planner, PlannerParams


def floor_with_cabin():
    """10 x 6 m at 5 cm. Corridor y 0..2 m, free. Above it a cabin x 3..7, y 2..6: walls all round, a 1 m door
    in its bottom wall at x 4.5..5.5, and its inside never scanned (unknown)."""
    res, w, h = 0.05, 200, 120
    data = np.full((h, w), 0, dtype=np.int16)
    data[:, :] = 0
    ys, xs = np.mgrid[0:h, 0:w]
    X, Y = (xs + 0.5) * res, (ys + 0.5) * res
    inside = (X > 3.1) & (X < 6.9) & (Y > 2.1) & (Y < 5.9)
    data[inside] = -1                                         # never scanned
    wall = ((np.abs(X - 3.05) < 0.05) | (np.abs(X - 6.95) < 0.05)) & (Y >= 2.0) & (Y <= 6.0)
    wall |= (np.abs(Y - 5.95) < 0.05) & (X >= 3.0) & (X <= 7.0)
    wall |= (np.abs(Y - 2.05) < 0.05) & (X >= 3.0) & (X <= 7.0) & ~((X > 4.5) & (X < 5.5))
    data[wall] = 100
    data[(Y > 5.0) & (Y < 5.2) & (X > 3.9) & (X < 4.1)] = 100   # a scanned table leg inside: stays a wall
    return GridMap.from_occupancy(w, h, res, 0.0, 0.0, data.ravel().tolist())


def planner_for(grid):
    p = Planner(PlannerParams(robot_radius=0.27, soft_radius=0.4))
    p.set_map(grid)
    return p


def test_painting_the_cabin_floor_lets_the_planner_in(tmp_path):
    grid = floor_with_cabin()
    goal = (5.0, 4.0)
    assert planner_for(grid).plan((1.0, 1.0), goal) is None                 # unknown inside: no route
    ed = MapEdits(str(tmp_path / 'map_edits' / 'cse.json'))
    ed.add('free', [[3.1, 2.1], [6.9, 2.1], [6.9, 5.9], [3.1, 5.9]])
    painted = ed.apply(grid)
    path = planner_for(painted).plan((1.0, 1.0), goal)
    assert path is not None and abs(path[-1][0] - 5.0) < 0.15 and abs(path[-1][1] - 4.0) < 0.15
    for x, y in path:                                         # it goes in through the door, not through a wall
        if 2.0 <= y <= 2.1:
            assert 4.5 <= x <= 5.5
    assert painted.occ[int(5.1 / 0.05), int(4.0 / 0.05)]   # the scanned table leg is still a wall
    assert grid.unknown[int(4.0 / 0.05), int(5.0 / 0.05)]  # the original grid is untouched


def test_a_painted_wall_blocks_the_route(tmp_path):
    grid = floor_with_cabin()
    ed = MapEdits(str(tmp_path / 'e.json'))
    ed.add('free', [[3.1, 2.1], [6.9, 2.1], [6.9, 5.9], [3.1, 5.9]])
    assert planner_for(ed.apply(grid)).plan((1.0, 1.0), (5.0, 4.0)) is not None
    ed.add('wall', [[4.4, 2.05], [5.6, 2.05]])                 # a glass door the LiDAR cannot see
    assert planner_for(ed.apply(grid)).plan((1.0, 1.0), (5.0, 4.0)) is None


def test_edits_are_saved_per_map_and_survive_a_restart(tmp_path):
    path = edits_path('~/maps/Cse_department_aids.pbstream', str(tmp_path))
    assert path == str(tmp_path / 'map_edits' / 'Cse_department_aids.json')
    assert edits_path('', str(tmp_path)) == ''                 # mapping: no edits
    a = MapEdits(path)
    r1 = a.add('free', [[0, 0], [2, 0], [2, 2], [0, 2]])
    r2 = a.add('wall', [[0, 0], [1, 1]])
    b = MapEdits(path)                                         # the navigator restarts
    assert [e['id'] for e in b.edits] == [r1['id'], r2['id']]
    b.remove(r1['id'])
    assert [e['id'] for e in MapEdits(path).edits] == [r2['id']]
    b.clear()
    assert MapEdits(path).edits == [] and json.load(open(path))['edits'] == []
    assert not os.path.exists(path + '.part')                  # written atomically


def test_bad_edits_are_refused(tmp_path):
    ed = MapEdits(str(tmp_path / 'e.json'))
    with pytest.raises(ValueError):
        ed.add('free', [[0, 0], [1, 1]])                       # an area needs 3 corners
    with pytest.raises(ValueError):
        ed.add('lava', [[0, 0], [1, 1], [0, 1]])
    with pytest.raises(ValueError):
        ed.add('free', [[0, 0], [float('nan'), 1], [0, 1]])
    with pytest.raises(ValueError):
        ed.add('free', [[0, 0], [20, 0], [20, 20], [0, 20]])   # 400 m^2: a slip of the finger
    with pytest.raises(ValueError):
        ed.add('wall', [[0, 0]])
    with pytest.raises(ValueError):
        MapEdits('').add('free', [[0, 0], [1, 0], [1, 1]])     # mapping mode: nothing to paint on
    with pytest.raises(ValueError):
        ed.remove(99)
    assert ed.edits == []


def test_a_broken_edits_file_never_stops_the_robot(tmp_path):
    p = tmp_path / 'e.json'
    p.write_text('{not json')
    ed = MapEdits(str(p))
    assert ed.edits == [] and ed.error
    grid = floor_with_cabin()
    assert ed.apply(grid) is grid


def test_navigator_core_paint_commands(tmp_path):
    from lidar_robot.navigator_core import NavigatorCore
    from lidar_robot.places import PlaceStore
    core = NavigatorCore(PlaceStore(str(tmp_path / 'places.json')))
    r = core.handle({'type': 'add_map_edit', 'kind': 'free', 'pts': [[0, 0], [1, 0], [1, 1]]})
    assert not r['ok'] and 'saved map' in r['error']           # no saved map loaded
    core.map_edits = MapEdits(str(tmp_path / 'map_edits' / 'm.json'))
    hits = []
    core.on_edits_changed = lambda: hits.append(1)
    r = core.handle({'type': 'add_map_edit', 'kind': 'free', 'pts': [[0, 0], [1, 0], [1, 1]]})
    assert r['ok'] and r['edit']['id'] == 1 and hits == [1]
    st = core.status()['map_edits']
    assert st['enabled'] and st['map'] == 'm' and len(st['edits']) == 1
    assert core.handle({'type': 'delete_map_edit', 'id': 1})['ok'] and len(hits) == 2
    assert core.handle({'type': 'clear_map_edits'})['ok']
    assert core.status()['map_edits']['edits'] == []


def test_launch_gives_the_navigator_the_saved_map_only_in_localization():
    src = open(os.path.join(os.path.dirname(__file__), '..', 'launch', 'bringup.launch.py')).read()
    assert "'map_file': map_file if (slam and slam_mode == 'localization') else ''" in src
