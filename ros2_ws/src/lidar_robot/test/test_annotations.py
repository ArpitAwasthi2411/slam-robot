"""21 Nov: places and rooms belong to one map. A new map (the exhibition hall) must start without the department's
places floating over it, and going back to the department map brings them back."""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.annotations import NEW, copy_set, map_key, switch_places  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402


def dept(tmp_path):
    pp = str(tmp_path / 'places.json')
    s = PlaceStore(pp)
    s.add('HOD office', 3.0, 4.0)
    s.add_room('Lab 2', [[0, 0], [2, 0], [2, 2], [0, 2]], contents=['oscilloscope'])
    s.set_home(0.5, 0.5)
    s.save()
    return pp


def names(pp):
    return [p['name'] for p in PlaceStore(pp).places]


def test_keys():
    assert map_key('/home/arpit/maps/Cse_department_aids.pbstream', 'localization') == 'Cse_department_aids'
    assert map_key('', 'mapping') == NEW
    assert map_key('', 'localization') is None


def test_old_file_is_claimed_by_the_map_in_use(tmp_path):
    pp = dept(tmp_path)
    assert 'belong to map Cse' in switch_places(pp, 'Cse')
    assert json.load(open(pp))['map'] == 'Cse' and names(pp) == ['HOD office']
    assert switch_places(pp, 'Cse') == ''                       # same map again: nothing happens


def test_new_map_starts_empty_and_the_department_comes_back(tmp_path):
    pp = dept(tmp_path)
    switch_places(pp, 'Cse')
    switch_places(pp, NEW)                                      # exhibition: New map
    s = PlaceStore(pp)
    assert s.places == [] and s.rooms == [] and s.home is None
    s.add('Stall 4', 1.0, 1.0)                                  # marked while mapping the hall
    s.save()
    assert 'now belong to map Expo' in switch_places(pp, 'Expo')   # frozen as Expo: the marks stay
    assert names(pp) == ['Stall 4']
    switch_places(pp, 'Cse')                                    # back in the department
    assert names(pp) == ['HOD office'] and PlaceStore(pp).rooms[0]['name'] == 'Lab 2'
    switch_places(pp, 'Expo')                                   # and the hall again
    assert names(pp) == ['Stall 4']
    switch_places(pp, NEW)                                      # another new map: empty again
    assert names(pp) == []


def test_nothing_is_lost_and_a_snapshot_can_borrow_a_set(tmp_path):
    pp = dept(tmp_path)
    switch_places(pp, 'Cse')
    switch_places(pp, 'Cse_s1530')                              # an autosave snapshot of the same run
    assert names(pp) == []
    assert os.path.isfile(tmp_path / 'annotations' / 'Cse.json')
    assert copy_set(pp, 'Cse') == 2 and names(pp) == ['HOD office']
    assert copy_set(pp, 'Cse') == 0                             # no duplicates
