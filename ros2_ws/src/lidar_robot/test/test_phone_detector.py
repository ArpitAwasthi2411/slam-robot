"""The phone detector (web/app/vision.js) must decide exactly like the robot (vision_rules.py).

Runs the JavaScript in Node on the same random inputs as the Python and compares: letterbox, YOLO decoding +
NMS, the person tracker (ids, approaching), bearings and the STOP / SLOW_DOWN / CONTINUE decision.
Skipped where Node isn't installed."""
import json
import math
import os
import random
import shutil
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot import vision_rules as vr  # noqa: E402

APP = os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app')
VISION_JS = os.path.join(APP, 'vision.js')


def run_js(body, payload):
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    script = (f"const V = require({json.dumps(os.path.abspath(VISION_JS))});\n"
              "const IN = JSON.parse(require('fs').readFileSync(0, 'utf8'));\n" + body)
    r = subprocess.run([node, '-e', script], input=json.dumps(payload), capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def _people(rng, n, with_dist):
    out = []
    for i in range(n):
        x1, y1 = rng.uniform(0, 500), rng.uniform(0, 250)
        p = {'id': i + 1, 'box': [x1, y1, x1 + rng.uniform(10, 140), y1 + rng.uniform(20, 300)],
             'bearing_deg': rng.uniform(-50, 50), 'approaching': rng.random() < 0.3}
        if with_dist and rng.random() < 0.7:
            p['distance_m'] = round(rng.uniform(0.2, 4.0), 2)
        out.append(p)
    return out


def test_decide_matches_python():
    rng = random.Random(7)
    cases = []
    for k in range(400):
        cases.append({'people': _people(rng, rng.randint(0, 4), with_dist=k % 2 == 0),
                      'h': rng.choice([360, 480, 0]) if k % 50 else 0})
    js = run_js("console.log(JSON.stringify(IN.map(c => V.decide(c.people, c.h))));", cases)
    for c, j in zip(cases, js):
        p = vr.decide(c['people'], c['h'])
        assert (j['action'], j['speed_factor'], j['steer'], j['reason'], j['people_in_path']) == \
               (p['action'], p['speed_factor'], p['steer'], p['reason'], p['people_in_path']), (c, j, p)
        assert (j['closest'] or {}).get('id') == (p['closest'] or {}).get('id')


def test_decode_and_letterbox_match_python():
    np = pytest.importorskip('numpy')
    rng = np.random.default_rng(3)
    n = 300
    cases = []
    for _ in range(20):
        a = np.zeros((84, n), dtype=np.float32)
        a[0] = rng.uniform(0, 320, n)
        a[1] = rng.uniform(0, 320, n)
        a[2] = rng.uniform(5, 120, n)
        a[3] = rng.uniform(5, 200, n)
        a[4:] = rng.uniform(0, 0.3, (80, n))
        hot = rng.choice(n, 25, replace=False)
        a[4, hot] = rng.uniform(0.3, 0.99, 25)
        w, h = int(rng.choice([640, 480, 810])), int(rng.choice([360, 480, 1080]))
        cases.append({'flat': a.flatten().tolist(), 'n': n, 'w': w, 'h': h, 'a': a})
    js = run_js("console.log(JSON.stringify(IN.map(c => [V.decodeYolo(Float32Array.from(c.flat), c.n, c.w, c.h, 320, 0.4, 0), "
                "V.letterboxParams(c.w, c.h, 320)])));",
                [{k: v for k, v in c.items() if k != 'a'} for c in cases])
    for c, (boxes, lb) in zip(cases, js):
        py = vr.decode_yolo(c['a'], c['w'], c['h'], 320, conf=0.4)
        assert len(boxes) == len(py)
        for b, q in zip(boxes, py):
            assert all(abs(x - y) < 1e-3 for x, y in zip(b, q)), (b, q)
        s, px, py_, nw, nh = vr.letterbox_params(c['w'], c['h'], 320)
        assert lb[1:] == [px, py_, nw, nh] and abs(lb[0] - s) < 1e-12


def test_tracker_and_geometry_match_python():
    rng = random.Random(11)
    frames, t = [], 0.0
    box = [100.0, 50.0, 160.0, 200.0]
    for k in range(40):
        t += 0.1
        grow = 1.0 + 0.04 * (k % 10)
        b1 = [box[0], box[1], box[2], box[1] + (box[3] - box[1]) * grow, 0.9]
        boxes = [b1] + ([[400.0 + rng.uniform(-3, 3), 60.0, 460.0, 220.0, 0.7]] if k % 7 else [])
        frames.append({'t': round(t, 3), 'boxes': boxes})
    js = run_js("const tr = new V.PersonTracker(); console.log(JSON.stringify(IN.map(f => tr.update(f.boxes, f.t))));",
                frames)
    tr = vr.PersonTracker()
    for f, j in zip(frames, js):
        p = tr.update([tuple(b) for b in f['boxes']], now=f['t'])
        assert [(x['id'], x['box'], x['approaching'], x['moving_away']) for x in j] == \
               [(x['id'], x['box'], x['approaching'], x['moving_away']) for x in p]
    pts = [([10, 0, 50, 100], 640, 102.0), ([600, 0, 640, 100], 640, 68.0), ([300, 0, 340, 100], 640, 68.0)]
    js = run_js("console.log(JSON.stringify(IN.map(c => V.bearingDeg(c[0], c[1], c[2]))));", pts)
    for (b, w, f), j in zip(pts, js):
        assert abs(j - vr.bearing_deg(b, w, f)) < 1e-9


def test_camera_distance_estimate():
    # 640 px wide, 68 deg: f = 320 / tan(34 deg) = 474.4 px; a 1.7 m person 400 px tall -> ~2.0 m
    js = run_js("console.log(JSON.stringify([V.cameraDistance([0, 0, 50, 400], 640, 68), V.cameraDistance([0, 5, 9, 5], 640, 68)]));",
                {})
    assert abs(js[0] - 320 / math.tan(math.radians(34)) * 1.7 / 400) < 1e-9 and js[1] is None


def test_detector_page_files():
    html = open(os.path.join(APP, 'detect.html')).read()
    for f in ('vision.js', 'vendor/ort/ort.wasm.min.js', 'detect.js'):
        assert f'src="{f}"' in html
    js = open(os.path.join(APP, 'detect.js')).read()
    assert 'vendor/yolo11n_320.onnx' in js and 'class PhoneCamera' in js and 'class RobotCamera' in js
    assert '__pathikBack' in js
