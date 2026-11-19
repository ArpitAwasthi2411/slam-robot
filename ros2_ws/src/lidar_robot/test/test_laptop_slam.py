"""Map building on the laptop riding on the robot (laptop_slam.py, scripts/laptop_slam.py, docs/laptop_slam.md)."""
import json
import os
import re
import time

from lidar_robot import laptop_slam as ls

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PKG)))


def msg(host='laptop', mode='mapping', map_file='', boot=100):
    return ls.parse_status(ls.status_message(host, mode, map_file, boot, '/tmp/pathik_maps', '/home/arpit/maps'))


def test_status_message_round_trip():
    s = msg(map_file='/home/arpit/maps/cse.pbstream')
    assert s['host'] == 'laptop' and s['mode'] == 'mapping' and s['map'] == 'cse.pbstream'
    assert s['save_dir'] == '/tmp/pathik_maps' and s['map_dir'] == '/home/arpit/maps'
    assert s['lua'] == 'cartographer_odom.lua'
    assert json.loads(ls.status_message('x', 'mapping', '', 1, '', '/m', use_odom=False))['host'] == 'pi'
    assert json.loads(ls.status_message('pi', 'mapping', '', 1, '', '/m', use_odom=False))['lua'] == \
        'cartographer_lidar_only.lua'
    assert ls.parse_status('not json') is None and ls.parse_status('{"host": "moon"}') is None


def test_laptop_runs_only_while_the_pi_asks_for_laptop_mapping():
    assert ls.wanted(msg()) == {'key': (100, ''), 'load': ''}
    assert ls.wanted(msg(host='pi')) is None
    assert ls.wanted(msg(mode='localization', map_file='/m/a.pbstream')) is None   # frozen: the Pi localizes
    assert ls.wanted(msg(map_file='/m/a.pbstream'))['load'] == 'a.pbstream'          # continue mapping
    assert ls.wanted(None) is None


def test_decide_start_keep_restart_stop():
    s = msg()
    key = ls.wanted(s)['key']
    assert ls.decide(s, None, 1.0) == 'start'
    assert ls.decide(s, key, 1.0) == 'keep'
    assert ls.decide(msg(boot=200), key, 1.0) == 'restart'          # the Pi restarted: odometry from zero
    assert ls.decide(msg(map_file='/m/b.pbstream'), key, 1.0) == 'restart'
    assert ls.decide(msg(mode='localization'), key, 1.0) == 'stop'  # frozen in the app
    assert ls.decide(s, key, 10.0) == 'keep'                         # a short blip loses nothing
    assert ls.decide(s, key, 20.0) == 'stop'                         # Pi silent: stop
    assert ls.decide(s, None, None) == 'keep'                        # never heard the Pi: wait
    assert ls.decide(msg(host='pi'), None, 1.0) == 'keep'


def test_ready_files_waits_until_written_and_sends_once(tmp_path):
    f = tmp_path / 'cse.pbstream'
    f.write_bytes(b'x' * 100)
    (tmp_path / 'cse.pgm').write_bytes(b'p')                         # not a Cartographer state: not sent
    now = time.time()
    assert ls.ready_files(str(tmp_path), {}, now) == []              # still being written (just changed)
    ready = ls.ready_files(str(tmp_path), {}, now + 5)
    assert [n for n, _ in ready] == ['cse.pbstream']
    sent = dict(ready)
    assert ls.ready_files(str(tmp_path), sent, now + 5) == []        # copied already
    f.write_bytes(b'y' * 200)                                       # saved again (autosave name reused)
    assert [n for n, _ in ls.ready_files(str(tmp_path), sent, time.time() + 5)] == ['cse.pbstream']
    (tmp_path / 'empty.pbstream').write_bytes(b'')
    assert 'empty.pbstream' not in [n for n, _ in ls.ready_files(str(tmp_path), {}, time.time() + 5)]
    assert ls.ready_files(str(tmp_path / 'missing'), {}, now) == []


def test_file_arrived_needs_a_new_copy(tmp_path):
    p = tmp_path / 'a.pbstream'
    assert not ls.file_arrived(str(p), time.time())
    p.write_bytes(b'abc')
    assert ls.file_arrived(str(p), time.time())
    old = time.time() - 100
    os.utime(p, (old, old))
    assert not ls.file_arrived(str(p), time.time())                 # an older map of the same name


def test_clock_note():
    assert 'agree' in ls.clock_note(0.05)
    assert 'sync' in ls.clock_note(-3.0)
    assert 'no /odom' in ls.clock_note(None)


def test_launch_skips_cartographer_on_the_pi_only_while_mapping_on_the_laptop():
    src = open(os.path.join(PKG, 'launch', 'bringup.launch.py')).read()
    assert "('slam_host', 'pi')" in src
    assert "laptop_maps = slam and slam_mode == 'mapping' and slam_host == 'laptop'" in src
    assert 'elif slam:' in src                                       # Cartographer nodes only without the laptop
    assert "'slam_host': slam_host if laptop_maps else 'pi'" in src


def test_laptop_lua_extends_the_pi_settings():
    lua = open(os.path.join(PKG, 'config', 'cartographer_laptop.lua')).read()
    assert 'include "cartographer_odom.lua"' in lua and lua.strip().endswith('return options')
    assert re.search(r'motion_filter\.max_distance_meters = 0\.10', lua)
    assert 'map_builder.lua' not in lua                              # Cartographer refuses a file included twice


def test_laptop_script_uses_the_shared_logic():
    if not os.path.isfile(os.path.join(REPO, 'scripts', 'laptop_slam.py')):   # robot image: package only
        import pytest
        pytest.skip('scripts/ not in this checkout')
    src = open(os.path.join(REPO, 'scripts', 'laptop_slam.py')).read()
    for name in ('decide', 'wanted', 'ready_files', 'parse_status', "'slam_host'", 'rsync', 'cartographer_laptop.lua'):
        assert name in src


def test_dashboard_saves_through_the_laptop():
    src = open(os.path.join(PKG, 'lidar_robot', 'dashboard.py')).read()
    assert "create_publisher(String, 'slam_host', latched)" in src
    assert 'os.path.join(self.slam_save_dir, os.path.basename(path)) if laptop else part' in src
    assert 'laptop_slam.file_arrived(path, t_req)' in src
    assert "'slam_warning': self._slam_warning()" in src


def test_rviz_env_x11_and_gpu_choice():
    from lidar_robot.laptop_slam import rviz_env
    base = {'QT_QPA_PLATFORM': 'wayland', 'LIBGL_ALWAYS_SOFTWARE': '1', 'HOME': '/h'}
    e = rviz_env(base, 'auto', has_nvidia=True)
    assert e['QT_QPA_PLATFORM'] == 'xcb' and e['__NV_PRIME_RENDER_OFFLOAD'] == '1'
    assert e['__GLX_VENDOR_LIBRARY_NAME'] == 'nvidia' and 'LIBGL_ALWAYS_SOFTWARE' not in e and e['HOME'] == '/h'
    e = rviz_env(base, 'auto', has_nvidia=False)
    assert '__NV_PRIME_RENDER_OFFLOAD' not in e and 'LIBGL_ALWAYS_SOFTWARE' not in e
    assert rviz_env(base, 'software')['LIBGL_ALWAYS_SOFTWARE'] == '1'
    assert base['QT_QPA_PLATFORM'] == 'wayland'   # caller's env untouched
