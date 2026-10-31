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
    for f in ('vision.js', 'vendor/ort/ort.wasm.min.js', 'detector.js', 'detect.js'):
        assert f'src="{f}"' in html
    assert html.index('src="detector.js"') < html.index('src="detect.js"')
    det = open(os.path.join(APP, 'detector.js')).read()
    assert 'vendor/yolo11n_320.onnx' in det and 'class PhoneCamera' in det and 'class RobotCamera' in det
    assert 'raw=1' in det                                   # the head / lab use the plain Pi picture
    js = open(os.path.join(APP, 'detect.js')).read()
    assert '__pathikBack' in js and 'new RobotCamera(robot.base, robot.token, true)' in js


ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
DETECTOR_JS = os.path.join(APP, 'detector.js')
needs_repo = pytest.mark.skipif(not os.path.isdir(os.path.join(ROOT, 'android')),
                                reason='full repo only (the robot image has just the ROS package)')


@needs_repo
def test_native_detector_wiring():
    """The app's TensorFlow Lite engine: model path, packaging and the JS <-> Java bridge must agree."""
    java = open(os.path.join(ROOT, 'android/app/src/main/java/com/arpit/pathik/NativeDetector.java')).read()
    main = open(os.path.join(ROOT, 'android/app/src/main/java/com/arpit/pathik/MainActivity.java')).read()
    gradle = open(os.path.join(ROOT, 'android/app/build.gradle')).read()
    fetch = open(os.path.join(ROOT, 'scripts/fetch_phone_detector.sh')).read()
    ci = open(os.path.join(ROOT, '.github/workflows/android.yml')).read()
    assert 'MODEL = "app/vendor/yolo11n_320.tflite"' in java          # web/ is the APK's assets folder
    assert 'lidar_robot/web"]' in gradle and "noCompress 'tflite'" in gradle  # memory-mapped: must stay uncompressed
    for dep in ('tensorflow-lite:', 'tensorflow-lite-gpu:', 'tensorflow-lite-gpu-api:'):
        assert f"org.tensorflow:{dep}" in gradle
    assert 'GpuDelegate' in java and 'setNumThreads' in java and 'setUseXNNPACK(true)' in java
    assert 'tfl="$dest/yolo11n_320.tflite"' in fetch and 'check_phone_model.py" tflite' in fetch
    assert 'vendor/yolo11n_320.tflite' in ci
    for fn in ('detectorInit(final int id)', 'detect(final int id, final String jpegB64, final float conf)',
               'hasNativeDetector()'):
        i = main.index(fn)
        assert '@JavascriptInterface' in main[i - 80:i], fn
    assert 'window.__pathikDetDone(' in main
    det = open(DETECTOR_JS).read()
    assert "nativeCall('detectorInit')" in det and "nativeCall('detect', b64, conf)" in det
    assert 'root.__pathikDetDone' in det


NATIVE_MOCK = r"""
// detector.js in Node with a fake canvas and a fake Java bridge (window.PathikNative)
const fs = require('fs');
global.window = global;
global.performance = { now: () => Date.now() };
const ctx = { fillRect() {}, drawImage() {}, getImageData: () => ({ data: new Uint8ClampedArray(320 * 320 * 4) }) };
global.document = { createElement: () => ({ getContext: () => ctx, toDataURL: () => 'data:image/jpeg;base64,QUJD' }) };
const calls = [];
window.PathikNative = {
  hasNativeDetector: () => true,
  detectorInit: id => { calls.push('init'); setTimeout(() => window.__pathikDetDone(id,
    JSON.stringify(IN.fail ? { ok: false, error: 'no native model' } : { ok: true, engine: 'phone GPU', ms_gpu: 21, ms_cpu: 60 }))); },
  detect: (id, b64, conf) => { calls.push(['detect', b64, conf]); setTimeout(() => window.__pathikDetDone(id,
    JSON.stringify({ ok: true, engine: 'phone GPU', ms: 30, infer_ms: 20, boxes: IN.boxes }))); },
};
window.PathikVision = V;
eval(fs.readFileSync(IN.det, 'utf8'));
(async () => {
  const D = window.PathikDetector;
  let backend = null, err = null;
  try { backend = await D.load(); } catch (e) { err = e.message; }
  const out = { backend, err, native: D.native, info: D.info, nativeError: D.nativeError || null, calls };
  if (!err) out.run = await D.run({ img: {}, w: IN.w, h: IN.h }, 0.4);
  console.log(JSON.stringify(out));
})();
"""


def test_native_engine_path_in_js():
    """Boxes from Java (320 px letterbox space) come back in frame pixels, duplicates removed, like the web engine."""
    boxes = [[100, 60, 140, 260, 0.9], [101, 61, 141, 259, 0.8], [10, 70, 12, 72, 0.5]]
    r = run_js(NATIVE_MOCK, {'det': os.path.abspath(DETECTOR_JS), 'boxes': boxes, 'w': 640, 'h': 360})
    assert r['backend'] == 'phone GPU' and r['native'] is True and r['info']['ms_gpu'] == 21
    assert r['calls'][0] == 'init' and r['calls'][1] == ['detect', 'QUJD', 0.4]
    s, px, py, _, _ = vr.letterbox_params(640, 360, 320)
    want = [max(0, (100 - px) / s), max(0, (60 - py) / s), min(640, (140 - px) / s), min(360, (260 - py) / s)]
    got = r['run']['boxes']
    assert len(got) == 2                                    # the near-duplicate is suppressed, the far box kept
    assert all(abs(a - b) < 1e-6 for a, b in zip(got[0][:4], want)) and got[0][4] == 0.9
    assert r['run']['inferMs'] == 20


def test_native_engine_failure_falls_back():
    """No model in the APK (or TFLite fails): the page says why and uses the web engine (here: missing -> error)."""
    r = run_js(NATIVE_MOCK, {'det': os.path.abspath(DETECTOR_JS), 'boxes': [], 'w': 640, 'h': 480, 'fail': True})
    assert r['native'] is False and r['nativeError'] == 'no native model'
    assert 'vendor/ort' in r['err']                         # ONNX Runtime Web isn't loaded in this test


@needs_repo
def test_check_phone_model_script():
    src = open(os.path.join(ROOT, 'scripts/check_phone_model.py')).read()
    assert '[1, 320, 320, 3]' in src and '[1, 84, 2100]' in src and '[1, 2100, 84]' in src
