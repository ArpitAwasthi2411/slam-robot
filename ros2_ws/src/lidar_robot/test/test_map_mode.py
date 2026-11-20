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
    assert map_mode.read_mode(f) == {'mode': 'localization', 'map': '/x/a.pbstream', 'since': 5, 'pose': None}
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
    svc_file = os.path.join(REPO, 'scripts', 'robot.service')
    if os.path.exists(svc_file):          # the Docker image holds only the ROS package, not scripts/
        assert 'Restart=always' in open(svc_file).read()


def test_app_has_the_map_mode_card():
    app = os.path.join(PKG, 'lidar_robot', 'web', 'app')
    html = open(os.path.join(app, 'index.html')).read()
    js = open(os.path.join(app, 'app.js')).read()
    demo = open(os.path.join(app, 'demo.js')).read()
    for i in ('mm-card', 'mm-freeze', 'mm-continue', 'mm-use', 'mm-new', 'mm-switching'):
        assert f'id="{i}"' in html
    assert "post('/api/slam'" in js and "get('/api/slam')" in js and 'renderMapMode(st)' in js
    assert "p === '/api/slam'" in demo


# ------------------------------------------------------------------ autosave while mapping
def test_autosave_while_mapping_only_when_the_map_changed(tmp_path):
    from lidar_robot.map_mode import PeriodicMapSaver
    saved = []
    s = PeriodicMapSaver(lambda n: saved.append(n) or 'ok', period_s=180, map_dir=str(tmp_path), threaded=False)
    assert not s.tick(0, True, 1)                        # just started
    assert s.tick(91, True, 2) and saved[0].startswith(s.name + '_s')   # first save after half a period
    assert not s.tick(200, True, 2)                       # standing still: same map, no save
    assert not s.tick(200, True, 3)                       # changed, but not due yet
    assert s.tick(272, True, 3) and len(saved) == 2
    assert not s.tick(1000, False, 9)                     # localization: never
    assert s.info['state'] == 'saved' and s.name.startswith('autosave_')


def test_autosave_at_once_when_the_pi_is_hot_and_never_crashes(tmp_path):
    from lidar_robot.map_mode import PeriodicMapSaver
    saved = []
    s = PeriodicMapSaver(lambda n: saved.append(n) or 'ok', period_s=180, map_dir=str(tmp_path), threaded=False)
    s.tick(0, True, 1)
    assert s.tick(10, True, 2, hot=True)                  # hot: now
    assert not s.tick(40, True, 3, hot=True)              # at most once a minute
    assert s.tick(71, True, 4, hot=True)

    def boom(n):
        raise ValueError('no /map received yet')
    b = PeriodicMapSaver(boom, period_s=10, map_dir=str(tmp_path), threaded=False)
    b.tick(0, True, 1)
    assert b.tick(5, True, 2) and b.info['state'] == 'error' and 'no /map' in b.info['error']
    assert PeriodicMapSaver(boom, period_s=0, map_dir=str(tmp_path), threaded=False).tick(99, True, 5) is False


def test_autosave_keeps_only_the_newest_drives(tmp_path):
    from lidar_robot.map_mode import PeriodicMapSaver
    for i in range(5):
        for ext in ('.pbstream', '.pgm', '.yaml'):
            (tmp_path / f'autosave_2026-11-1{i}_1000{ext}').write_text('x')
    (tmp_path / 'cse_floor.pbstream').write_text('x')
    PeriodicMapSaver(lambda n: 'ok', map_dir=str(tmp_path), keep=3).prune()
    left = sorted(p.name for p in tmp_path.iterdir())
    assert 'cse_floor.pbstream' in left and 'autosave_2026-11-10_1000.pbstream' not in left
    assert len([n for n in left if n.endswith('.pbstream') and n.startswith('autosave_')]) == 3


def test_pi_is_hot_accepts_every_throttled_format():
    from lidar_robot.pihealth import pi_is_hot as hot
    assert not hot({'temp_c': 50, 'throttled': '0x0'}) and not hot({'temp_c': 50, 'throttled': 0})
    assert not hot({'temp_c': None, 'throttled': None}) and not hot({})
    assert hot({'temp_c': 50, 'throttled': '0x50005'}) and hot({'temp_c': 50, 'throttled': 'throttled=0x4'})
    assert hot({'temp_c': 79, 'throttled': '0x0'}) and hot({'temp_c': 50, 'throttled': 2})


def test_freeze_pose_is_kept_once_and_only_for_that_map(tmp_path):
    f = str(tmp_path / 'map_mode.json')
    m = str(tmp_path / 'cse.pbstream')
    map_mode.write_mode(f, 'localization', m, clock=lambda: 1000, pose={'x': 3.2, 'y': -1.0, 'yaw': 1.57})
    saved = map_mode.read_mode(f)
    assert saved['pose'] == {'x': 3.2, 'y': -1.0, 'yaw': 1.57}
    assert map_mode.startup_pose(saved, 'localization', m, 1100) == saved['pose']
    assert map_mode.startup_pose(saved, 'localization', m, 1000 + 2000) is None        # old: robot may have moved
    assert map_mode.startup_pose(saved, 'localization', str(tmp_path / 'other.pbstream'), 1100) is None
    assert map_mode.startup_pose(saved, 'mapping', m, 1100) is None
    map_mode.write_mode(f, 'localization', m)                                          # used once, cleared
    assert map_mode.read_mode(f)['pose'] is None


def test_prune_never_drops_the_long_drive(tmp_path):
    def mk(name, size):
        for ext in ('.pbstream', '.pgm', '.yaml'):
            (tmp_path / (name + ext)).write_bytes(b'x' * (size if ext == '.pbstream' else 1))
    mk('autosave_2026-11-18_1513', 15000)          # the long drive
    for n in ('autosave_2026-11-18_1601', 'autosave_2026-11-18_1605', 'autosave_2026-11-18_1610'):
        mk(n, 100)                                 # three quick restarts, nearly empty maps
    s = map_mode.PeriodicMapSaver(lambda n: 'ok', map_dir=str(tmp_path), keep=3, threaded=False)
    s.prune()
    left = sorted(p.name for p in tmp_path.glob('*.pbstream'))
    assert 'autosave_2026-11-18_1513.pbstream' in left and len(left) == 4
    mk('autosave_2026-11-18_1620', 100)
    s.prune()
    left = sorted(p.name for p in tmp_path.glob('*.pbstream'))
    assert 'autosave_2026-11-18_1513.pbstream' in left and 'autosave_2026-11-18_1601.pbstream' not in left


def test_half_written_saves_are_not_listed_as_maps(tmp_path):
    (tmp_path / 'cse.pbstream.part').write_bytes(b'x')
    (tmp_path / 'good.pbstream').write_bytes(b'x')
    assert [m['name'] for m in map_mode.list_maps(str(tmp_path))] == ['good']


def test_autosave_keeps_snapshots_so_a_map_that_goes_bad_late_is_not_lost(tmp_path):
    """20 Nov: the department map distorted at the end of the drive and the one autosave of the run was
    overwritten with it. Every save is now its own snapshot; the last 5 of the running drive are kept."""
    import time as _t
    from lidar_robot import map_mode
    clock = {'t': _t.struct_time((2026, 11, 20, 9, 3, 0, 4, 324, -1))}
    saved = []

    def save(n):
        (tmp_path / (n + '.pbstream')).write_text('x' * (len(saved) + 1))
        saved.append(n)
        return 'ok'
    s = map_mode.PeriodicMapSaver(save, period_s=180, map_dir=str(tmp_path), threaded=False,
                                  clock=lambda: clock['t'], snapshots=5)
    (tmp_path / 'autosave_2026-11-19_1758_s1801.pbstream').write_text('older drive')
    s.tick(0, True, None)
    for k in range(8):
        clock['t'] = _t.struct_time((2026, 11, 20, 9, 10 + 3 * k, 0, 4, 324, -1))
        assert s.tick(100 + 200 * k, True, k)
    assert saved[0] == 'autosave_2026-11-20_0903_s0910' and len(set(saved)) == 8
    left = sorted(p.name for p in tmp_path.iterdir())
    run = [n for n in left if n.startswith('autosave_2026-11-20_0903')]
    assert len(run) == 5 and 'autosave_2026-11-20_0903_s0931.pbstream' in run      # newest five
    assert 'autosave_2026-11-19_1758_s1801.pbstream' in left                        # the drive before survives
    assert s.info['name'] == saved[-1]
    assert s.snapshot_name() == 'autosave_2026-11-20_0903_s0931_2'                  # same minute: never overwrite
