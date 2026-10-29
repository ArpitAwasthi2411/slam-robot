"""Regression tests for the bugs found in the October 2026 review (docs/REVIEW_2026-10.md)."""
import json
import os
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.dashboard_server import DashboardServer  # noqa: E402
from lidar_robot.explain import explain  # noqa: E402
from lidar_robot.mapsave import safe_name, save_map  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.planner import GridMap  # noqa: E402


class Backend:
    def __init__(self):
        self.calls = []

    def get_state(self):
        return {}

    def get_map(self, since):
        return None

    def command(self, v, w):
        self.calls.append(('cmd', v, w))

    def set_estop(self, on):
        self.calls.append(('estop', on))

    def send_goal(self, x, y, yaw):
        self.calls.append(('goal', x, y, yaw))

    def cancel_goal(self):
        pass

    def save_map(self, name):
        return name

    def nav_request(self, req):
        return {'ok': True}


def _post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method='POST',
                                 headers={'Content-Type': 'text/plain'})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, json.load(r)
    except urllib.error.HTTPError as e:
        return e.code, json.load(e)


def test_http_rejects_non_finite_drive_and_goal():
    be = Backend()
    srv = DashboardServer(be, host='127.0.0.1', port=0, users=None)
    srv.start()
    base = f'http://127.0.0.1:{srv.httpd.server_address[1]}'
    try:
        for body in ({'v': 'nan', 'w': 0}, {'v': 0.1, 'w': 'inf'}, {'v': '-Infinity'}):
            code, r = _post(base, '/api/cmd', body)
            assert code == 400 and 'finite' in r['error']
        assert _post(base, '/api/goal', {'x': 'nan', 'y': 1})[0] == 400
        assert _post(base, '/api/goal', {'y': 1})[0] == 400                   # x missing
        assert be.calls == []                                                 # nothing reached the robot
        assert _post(base, '/api/cmd', {'v': 0.1, 'w': -0.2})[0] == 200
        assert _post(base, '/api/goal', {'x': 1, 'y': 2})[0] == 200
        assert be.calls == [('cmd', 0.1, -0.2), ('goal', 1.0, 2.0, 0.0)]
    finally:
        srv.stop()


def _core(tmp_path, **kw):
    places = PlaceStore(str(tmp_path / 'places.json'))
    places.add('Lab', 2.0, 0.0, 0.0)
    core = NavigatorCore(places, data_dir=str(tmp_path), **kw)
    w = h = 60
    core.set_map(GridMap.from_occupancy(w, h, 0.1, -3.0, -3.0, [0] * (w * h)))
    return core


def test_stop_command_also_ends_exploration(tmp_path):
    core = _core(tmp_path)
    core.step((0.0, 0.0, 0.0), 5.0, 0.05)
    core.explore = {'active': True, 'state': 'running', 'saved_speed': (0.2, 0.4)}
    r = core.handle({'type': 'command', 'text': 'stop'})
    assert r['ok'] and r['interpretation']['intent'] == 'stop'
    assert core.explore['active'] is False and core.state == 'IDLE'


def test_auto_home_never_crashes_the_loop(tmp_path):
    t = [0.0]
    core = _core(tmp_path, auto_return_s=1.0, clock=lambda: t[0])
    core.places.set_home(-2.0, 0.0, 0.0)

    def refuse(priority, source):                  # what _enqueue does when the battery is critical
        raise ValueError('battery critical (9 %): charge me first')
    core._go_home = refuse
    t[0] = 5.0
    core._maybe_auto_home((1.5, 0.0, 0.0), t[0])  # used to raise out of step()
    assert any('auto-return home skipped' in e['text'] for e in core.events)
    core.step((1.5, 0.0, 0.0), 5.0, 0.05)


def test_explain_failure_text_with_braces(tmp_path):
    core = _core(tmp_path)
    core.last_failure = {'code': 'INTERNAL', 'text': "internal error: {'x': 1}", 'mission': None,
                         't': __import__('time').time()}
    assert "{'x': 1}" in explain(core)['answer']          # used to raise KeyError in str.format


def test_explain_covers_main_states(tmp_path):
    core = _core(tmp_path)
    assert 'not doing anything' in explain(core)['answer']
    core.set_estop(True)
    assert 'emergency stop' in explain(core)['answer']
    core.set_estop(False)
    assert 'Resume' in explain(core)['answer'] or 'paused' in explain(core)['answer'] or \
        'not doing anything' in explain(core)['answer']
    core.handle({'type': 'resume'})
    core.localized = False
    assert 'position' in explain(core)['answer']


def test_save_map_files_and_names(tmp_path):
    assert safe_name('../../etc/passwd') == '______etc_passwd'
    assert safe_name('   ') == 'map'
    pgm, yml = save_map(str(tmp_path), 'lab floor', 3, 2, 0.05, -1.0, 2.0, float('nan'),
                        [0, 100, -1, 50, 0, 100])
    assert os.path.basename(pgm) == 'lab_floor.pgm'
    data = open(pgm, 'rb').read()
    assert data.startswith(b'P5') and data.endswith(bytes([205, 254, 0, 254, 0, 205]))   # top row (row 1) first
    y = open(yml).read()
    assert 'image: lab_floor.pgm' in y and 'origin: [-1.000000, 2.000000, 0.000000]' in y   # NaN yaw -> 0
