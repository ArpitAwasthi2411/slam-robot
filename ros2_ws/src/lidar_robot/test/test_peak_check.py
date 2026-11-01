"""Peak test verdict (web/app/peakcheck.js) on made-up runs: a phone that keeps up, one that throttles when hot,
one with a stuttering face, and a plain browser without phone health data."""
import json
import os
import shutil
import subprocess

import pytest

APP = os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app')
RUN = r"""
const P = require(process.argv[1]);
const runs = JSON.parse(require('fs').readFileSync(0, 'utf8'));
console.log(JSON.stringify(runs.map(r => P.evaluate(r.samples, r.opts))));
"""


def evaluate(*runs):
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    r = subprocess.run([node, '-e', RUN, os.path.abspath(os.path.join(APP, 'peakcheck.js'))],
                       input=json.dumps(list(runs)), capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def make(dur=300, det_ms=lambda t: 90, det_n=lambda t: 9, face=lambda t: 60, temp=lambda t: 33 + t / 60,
         bat=lambda t: 80 - t / 300, thermal=lambda t: 0, health=True, long_ms=lambda t: 5):
    out = []
    for t in range(1, dur + 1):
        s = {'t': t, 'det_n': det_n(t), 'det_ms': det_ms(t), 'face_fps': face(t), 'long_ms': long_ms(t),
             'temp_c': None, 'battery_pct': None, 'charging': False, 'thermal': None, 'app_mb': 180}
        if health:
            s.update(temp_c=round(temp(t), 1), battery_pct=round(bat(t)), thermal=thermal(t))
        out.append(s)
    return {'samples': out, 'opts': {}}


def by_name(res):
    return {c['name']: c for c in res['checks']}


def test_good_phone_passes():
    r = evaluate(make())[0]
    assert r['verdict'] == 'PASS', r
    assert by_name(r)['Detection keeps up']['value'] == '9 per second'
    assert r['line'].startswith('5 min: 9 detections/s at 90 ms, face 60 fps, peak 38')


def test_heat_throttling_fails():
    r = evaluate(make(det_ms=lambda t: 90 if t < 120 else 190, det_n=lambda t: 9 if t < 120 else 4.5,
                      temp=lambda t: 35 + t / 25, thermal=lambda t: 3 if t > 200 else 0))[0]
    c = by_name(r)
    assert r['verdict'] == 'FAIL'
    assert c['Slows down when hot']['status'] == 'bad' and '× 2.11' in c['Slows down when hot']['value']
    assert c['Phone temperature']['status'] == 'bad' and c['Android heat state']['value'] == 'severe'
    assert c['Detection keeps up']['status'] == 'warn'


def test_stuttering_face_and_stalls_warn():
    r = evaluate(make(face=lambda t: 38, long_ms=lambda t: 30))[0]
    c = by_name(r)
    assert r['verdict'] == 'WARN'
    assert c['Face smoothness']['status'] == 'warn' and c['Page stalls']['status'] == 'warn'


def test_browser_without_phone_health():
    r = evaluate(make(dur=90, health=False))[0]
    c = by_name(r)
    assert c['Phone temperature']['status'] == 'na' and 'Pathik app' in c['Phone temperature']['value']
    assert 'Battery use' not in c
    assert r['verdict'] == 'PASS'


def test_battery_drain_and_charging():
    hungry, charging = evaluate(make(bat=lambda t: 80 - t / 30), make())
    assert by_name(hungry)['Battery use']['status'] == 'bad'          # 10 % in 5 min = 120 %/h
    r = evaluate({'samples': [dict(s, charging=True) for s in make()['samples']], 'opts': {}})[0]
    assert by_name(r)['Battery use']['status'] == 'na'
    assert by_name(charging)['Battery use']['status'] == 'ok'
