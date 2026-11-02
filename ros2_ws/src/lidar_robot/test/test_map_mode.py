"""Map mode from the app (map_mode.py): freeze / continue / new / use, the launch reading it, the API."""
import json
import os
import sys
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot import map_mode  # noqa: E402
from lidar_robot.auth import required_role  # noqa: E402

PKG = os.path.join(os.path.dirname(__file__), '..')
REPO = os.path.join(PKG, '..', '..', '..')


def touch(path, data=b'x' * 2048):
    with open(path, 'wb') as f:
        f.write(data)
    return str(path)


def test_read_write_and_garbage(tmp_path):
    f = str(tmp_path / 'map_mode.json')
    assert map_mode.read_mode(f) is None
    map_mode.write_mode(f, 'localization', '/x/a.pbstream', clock=lambda: 5)
    assert map_mode.read_mode(f) == {'mode': 'localization', 'map': '/x/a.pbstream', 'since': 5}
    with pytest.raises(ValueError):
        map_mode.write_mode(f, 'frozen')
    with open(f, 'w') as fh:
        fh.write('{"mode": "banana"}')
    assert map_mode.read_mode(f) is None
    with open(f, 'w') as fh:
        fh.write('not json')
    assert map_mode.read_mode(f) is None


def test_resolve_file_wins_and_missing_map_falls_back(tmp_path):
    f = str(tmp_path / 'map_mode.json')
    pb = touch(tmp_path / 'cse.pbstream')
    assert map_mode.resolve('mapping', '', f) == ('mapping', '', '')          # no file: ROBOT_ARGS as before
    map_mode.write_mode(f, 'localization', pb)
    mode, m, note = map_mode.resolve('mapping', '', f)
    assert (mode, m) == ('localization', pb) and 'map mode' in note
    map_mode.write_mode(f, 'mapping', pb)                                    # continue mapping on it
    assert map_mode.resolve('localization', '/old.pbstream', f)[:2] == ('mapping', pb)
    os.remove(pb)                                                            # map deleted: still boots
    map_mode.write_mode(f, 'localization', pb)
    mode, m, note = map_mode.resolve('localization', pb, f)
    assert (mode, m) == ('mapping', '') and 'missing' in note


def test_list_maps_newest_first(tmp_path):
    a = touch(tmp_path / 'old.pbstream')
    os.utime(a, (1000, 1000))
    touch(tmp_path / 'new.pbstream')
    touch(tmp_path / 'new.pgm')
    touch(tmp_path / 'notes.txt')
    maps = map_mode.list_maps(str(tmp_path))
    assert [m['name'] for m in maps] == ['new', 'old']
    assert maps[0]['picture'] and not maps[1]['picture'] and maps[0]['size_kb'] == 2
    assert map_mode.list_maps(str(tmp_path / 'nope')) == []


def test_plan_switch_rules(tmp_path):
    d = str(tmp_path)
    pb = touch(tmp_path / 'cse.pbstream')
    p = map_mode.plan_switch('freeze', 'mapping', '', d, name='cse floor!')
    assert p['save_as'] == 'cse_floor_' and p['mode'] == 'localization'
    assert p['map'] == os.path.join(d, 'cse_floor_.pbstream')
    assert map_mode.plan_switch('freeze', 'mapping', '', d)['save_as'].startswith('map_')
    with pytest.raises(ValueError, match='already frozen'):
        map_mode.plan_switch('freeze', 'localization', pb, d)
    assert map_mode.plan_switch('continue', 'localization', pb, d)['map'] == pb
    assert map_mode.plan_switch('continue', 'mapping', '', d, map_name='cse')['map'] == pb
    with pytest.raises(ValueError, match='No saved map'):
        map_mode.plan_switch('continue', 'mapping', '', d)
    assert map_mode.plan_switch('use', 'mapping', '', d, map_name='cse') == dict(
        map_mode.plan_switch('use', 'mapping', '', d, map_name='cse'), mode='localization', map=pb, save_as=None)
    with pytest.raises(ValueError, match='Choose'):
        map_mode.plan_switch('use', 'mapping', '', d, map_name='../etc/passwd')
    n = map_mode.plan_switch('new', 'localization', pb, d)
    assert n['mode'] == 'mapping' and n['map'] == '' and 'Places' in n['say']
    with pytest.raises(ValueError):
        map_mode.plan_switch('delete', 'mapping', '', d)


def test_api_routes_and_roles():
    from lidar_robot.dashboard_server import DashboardServer

    class B:
        def get_state(self):
            return {}

        def slam_info(self):
            return {'mode': 'mapping', 'maps': []}

        def slam_switch(self, body):
            if body.get('action') == 'bad':
                raise ValueError('Not now: it is busy.')
            self.last = body
            return {'ok': True, 'say': 'restarting'}

    be = B()
    srv = DashboardServer(be, host='127.0.0.1', port=0)
    srv.start()
    base = f'http://127.0.0.1:{srv.httpd.server_address[1]}'
    try:
        with urllib.request.urlopen(base + '/api/slam') as r:
            assert json.load(r)['mode'] == 'mapping'

        def post(body):
            req = urllib.request.Request(base + '/api/slam', data=json.dumps(body).encode(), method='POST',
                                         headers={'Content-Type': 'application/json'})
            return urllib.request.urlopen(req)
        with post({'action': 'freeze', 'name': 'cse'}) as r:
            assert json.load(r)['say'] == 'restarting' and be.last['name'] == 'cse'
        with pytest.raises(urllib.error.HTTPError) as e:
            post({'action': 'bad'})
        assert e.value.code == 400 and 'busy' in json.load(e.value)['error']
    finally:
        srv.stop()
    assert required_role('POST', '/api/slam', {'action': 'freeze'}) == 'operator'      # not staff


def test_launch_and_service_wiring():
    src = open(os.path.join(PKG, 'launch', 'bringup.launch.py')).read()
    assert "('map_mode_file', '~/maps/map_mode.json')" in src
    assert 'resolve(slam_mode, map_file, mode_file)' in src
    assert 'ev.returncode == RESTART_CODE' in src and 'Shutdown(' in src
    assert "'-load_frozen_state=false'" in src                 # continue mapping loads the map unfrozen
    svc = open(os.path.join(REPO, 'scripts', 'robot.service')).read()
    assert 'Restart=always' in svc


def test_app_has_the_map_mode_card():
    app = os.path.join(PKG, 'lidar_robot', 'web', 'app')
    html = open(os.path.join(app, 'index.html')).read()
    js = open(os.path.join(app, 'app.js')).read()
    demo = open(os.path.join(app, 'demo.js')).read()
    for i in ('mm-card', 'mm-freeze', 'mm-continue', 'mm-use', 'mm-new', 'mm-switching'):
        assert f'id="{i}"' in html
    assert "post('/api/slam'" in js and "get('/api/slam')" in js and 'renderMapMode(st)' in js
    assert "p === '/api/slam'" in demo
