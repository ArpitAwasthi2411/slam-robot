"""Tuning Lab: protocol, clamping, persistence, telemetry buffer, test runner, wheel model, HTTP API."""
import json
import math
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest  # noqa: E402

from lidar_robot.protocol import parse_line, parse_tune, format_tune, format_telemetry, SpdPacket  # noqa: E402
from lidar_robot.tuning import (clamp_values, WHEEL_RANGES, FOLLOWER_RANGES, TuningStore,  # noqa: E402
                                TelemetryBuffer, TestRunner, WheelSim, apply_follower)
from lidar_robot.follower import FollowerParams  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402


def test_spd_and_tune_lines():
    assert parse_line('SPD,200,195,80,-10,-12,-30') == SpdPacket(200, 195, 80, -10, -12, -30)
    assert parse_line('SPD,1,2,3') is None
    assert parse_tune('TUNE,0.1500,0.4000,15.0,493,600') == {
        'kp': 0.15, 'ki': 0.4, 'pwm_min': 15.0, 'max_mms': 493.0, 'accel': 600.0}
    assert parse_tune('READY,robot_esp32') is None
    assert format_tune(0.15, 0.4, 15, 493, 600) == b'K,0.1500,0.4000,15.0,493,600\n'
    assert format_telemetry(True) == b'T,1\n'
    with pytest.raises(ValueError):
        format_tune(float('nan'), 0, 0, 0, 0)


def test_clamp_values():
    out = clamp_values({'kp': 9, 'ki': -1, 'bogus': 3, 'accel': '700'}, WHEEL_RANGES)
    assert out == {'kp': 2.0, 'ki': 0.0, 'accel': 700.0}
    with pytest.raises(ValueError):
        clamp_values({'kp': float('inf')}, WHEEL_RANGES)
    with pytest.raises(ValueError):
        clamp_values({'kp': 'abc'}, WHEEL_RANGES)


def test_follower_hysteresis_kept_sane():
    p = FollowerParams()
    apply_follower(p, {'rotate_in_place_above': 0.5, 'rotate_exit_below': 0.9})
    assert p.rotate_exit_below < p.rotate_in_place_above


def test_tuning_store_roundtrip(tmp_path):
    path = tmp_path / 'tuning.json'
    st = TuningStore(str(path))
    st.save_follower({'lookahead': 0.8, 'k_angular': 99})
    st2 = TuningStore(str(path))
    assert st2.data['follower']['lookahead'] == 0.8
    assert st2.data['follower']['k_angular'] == FOLLOWER_RANGES['k_angular'][1]
    path.write_text('{corrupt')
    assert TuningStore(str(path)).data['follower'] == {}


def test_navigator_tune_and_persist(tmp_path):
    places = PlaceStore(str(tmp_path / 'p.json'))
    nav = NavigatorCore(places, tuning_path=str(tmp_path / 'tuning.json'))
    r = nav.handle({'type': 'tune', 'follower': {'lookahead': 0.9}, 'save': True})
    assert r['ok'] and r['follower']['lookahead'] == 0.9
    nav2 = NavigatorCore(places, tuning_path=str(tmp_path / 'tuning.json'))
    assert nav2.follower.p.lookahead == 0.9
    g = nav2.handle({'type': 'get_tune'})
    assert g['ok'] and 'lookahead' in g['ranges']


def test_telemetry_buffer_since():
    b = TelemetryBuffer(maxlen=5)
    for i in range(8):
        b.add([i, 0, 0, 0, 0, 0])
    d = b.since(0)
    assert d['seq'] == 8 and len(d['samples']) == 5          # ring buffer keeps the newest
    assert [s[1] for s in b.since(6)['samples']] == [6, 7]


def test_test_runner_records_and_stops():
    sent = []
    pose = {'x': 0.0}

    def send(v, w):
        sent.append((v, w))
        pose['x'] += v * 0.01

    tr = TestRunner(send, lambda: {'x': pose['x'], 'y': 0, 'yaw': 0, 'v': 0.2, 'w': 0}, rate=100)
    assert tr.start({'kind': 'straight', 'v': 0.2, 'secs': 0.5})['ok']
    with pytest.raises(ValueError):
        tr.start({'kind': 'straight'})                          # one at a time
    tr.thread.join(3)
    st = tr.status()
    assert not st['running'] and st['aborted'] is None and len(st['samples']) > 20
    assert sent[-1] == (0.0, 0.0)                              # always ends with a stop
    blocked = TestRunner(send, lambda: None, is_blocked=lambda: True, rate=100)
    blocked.start({'kind': 'spin', 'w': 0.5, 'secs': 1})
    blocked.thread.join(2)
    assert blocked.status()['aborted'].startswith('blocked')
    with pytest.raises(ValueError):
        tr.start({'kind': 'teleport'})


def _settle(tune, steps=150):
    w = WheelSim(tune, noise=0.0)
    for _ in range(steps):
        w.ramp_to(200.0, 0.02)
        w.step(0.02)
    return w


def test_wheel_sim_tracks_target_and_ki_removes_error():
    good = _settle({'kp': 0.15, 'ki': 0.4})
    assert abs(good.meas - 200) < 8
    no_i = _settle({'kp': 0.15, 'ki': 0.0, 'pwm_min': 0.0})   # no feed-forward offset, no I -> error
    assert abs(no_i.meas - 200) > abs(good.meas - 200)


def test_ramp_limits_acceleration():
    w = WheelSim({'accel': 500}, noise=0.0)
    w.ramp_to(400.0, 0.02)
    assert w.target == pytest.approx(10.0)
    w.t['accel'] = 0
    w.ramp_to(400.0, 0.02)
    assert w.target == 400.0


def test_http_api_cors_ping_and_static(tmp_path):
    from lidar_robot.dashboard_server import DashboardServer

    class B:
        def get_state(self):
            return {'ok': 1}

        def ping(self):
            return {'ok': True, 'api': 2}

        def esp_command(self, req):
            self.last = req
            return {'ok': True}

    be = B()
    srv = DashboardServer(be, host='127.0.0.1', port=0)
    srv.start()
    port = srv.httpd.server_address[1]
    try:
        base = f'http://127.0.0.1:{port}'
        with urllib.request.urlopen(base + '/api/ping') as r:
            assert r.headers['Access-Control-Allow-Origin'] == '*'
            assert json.load(r)['api'] == 2
        req = urllib.request.Request(base + '/api/esp', data=b'{"op":"save"}', method='POST',
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req) as r:
            assert json.load(r)['ok'] and be.last == {'op': 'save'}
        req = urllib.request.Request(base + '/api/cmd', method='OPTIONS')
        with urllib.request.urlopen(req) as r:
            assert r.status == 204 and 'POST' in r.headers['Access-Control-Allow-Methods']
        with pytest.raises(urllib.error.HTTPError) as e:   # backend without telemetry -> 404
            urllib.request.urlopen(base + '/api/telemetry')
        assert e.value.code == 404
        with pytest.raises(urllib.error.HTTPError) as e:   # no path traversal out of the app dir
            urllib.request.urlopen(base + '/app/../dashboard.py')
        assert e.value.code == 404
    finally:
        srv.stop()
        time.sleep(0.05)
    assert math.isfinite(1.0)
