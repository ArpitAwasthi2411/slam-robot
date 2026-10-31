"""Head mode (web/app/face.js + head.html): the face shows the right mood for each robot state, the page has
what it needs, and the app / Android open it. faceFor runs in Node on states shaped like /api/state."""
import json
import os
import shutil
import subprocess

import pytest

APP = os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app')
ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')


def faces(cases):
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    script = (f"const F = require({json.dumps(os.path.abspath(os.path.join(APP, 'face.js')))});\n"
              "const IN = JSON.parse(require('fs').readFileSync(0, 'utf8'));\n"
              "console.log(JSON.stringify(IN.map(c => F.faceFor(c[0], c[1]))));")
    r = subprocess.run([node, '-e', script], input=json.dumps(cases), capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def st(state='IDLE', **kw):
    nav = {'state': state, 'localized': True, 'battery': {'pct': 80, 'level': 'ok', 'charging': False}}
    nav.update({k: v for k, v in kw.items() if k not in ('estop', 'vision')})
    return {'estop': kw.get('estop', False), 'vision': kw.get('vision'), 'nav': nav}


def test_moods_for_robot_states():
    ask = {'question': 'The door is blocked. What should I do?', 'options': [{'id': 'retry', 'label': 'Try again', 'primary': True}]}
    cases = [
        (None, {}),                                                            # no robot
        (st(estop=True), {}),
        (st('RECOVERING', recovery={'status': 'asking', 'id': 4, 'ask': ask}), {}),
        (st(localized=False), {}),
        (st('CONFIRM', confirm={'label': 'Lab 2', 'left_s': 100}), {}),
        (st('DRIVING', goal={'label': 'HOD office'}, distance=6.2, eta_s=40, vision={'action': 'STOP', 'reason': 'person 0.6 m ahead'}), {}),
        (st('DRIVING', goal={'label': 'HOD office'}, blocked=True), {}),
        (st('PLANNING', goal={'label': 'HOD office'}), {}),
        (st('DRIVING', goal={'label': 'HOD office'}, distance=6.2, eta_s=40), {}),
        (st('CHARGING', battery={'pct': 55, 'level': 'ok', 'charging': True}), {}),
        (st(battery={'pct': 20, 'level': 'low', 'charging': False}), {}),
        (st(), {'idleS': 300}),
        (st(), {'idleS': 5}),
        (st('DRIVING', goal={'label': 'Lab 3'}, vision={'action': 'SLOW_DOWN'}), {}),
    ]
    got = faces(cases)
    moods = [g['mood'] for g in got]
    assert moods == ['offline', 'stopped', 'help', 'lost', 'happy', 'blocked', 'blocked', 'thinking', 'driving',
                     'charging', 'lowbat', 'sleepy', 'idle', 'blocked']
    assert got[2]['ask']['options'][0]['id'] == 'retry' and got[2]['recId'] == 4
    assert got[4]['ok'] is True and 'Lab 2' in got[4]['line']
    assert got[5]['line'] == 'Excuse me, may I pass?'
    assert got[8]['line'] == 'Going to HOD office' and got[8]['sub'] == '6.2 m to go · about 40 s'
    assert '55 %' in got[9]['line']


def test_head_page_and_entry_points():
    html = open(os.path.join(APP, 'head.html')).read()
    order = [html.index(f'src="{f}"') for f in ('vision.js', 'vendor/ort/ort.wasm.min.js', 'detector.js', 'face.js', 'head.js')]
    assert order == sorted(order)
    js = open(os.path.join(APP, 'head.js')).read()
    for need in ("'/api/vision/phone'", "new DT.RobotCamera(R.base, R.token, true)", "'/api/estop', { on: true }",
                 "needPin(", "type: 'goto_place'", "type: 'goto_room'", "type: 'command'", "recovery_answer", "keepAwake(true)"):
        assert need in js, need
    # STOP never asks for the PIN
    stop = js[js.index("$('stop').onclick"):js.index('\n', js.index("$('stop').onclick"))]
    assert 'Pin' not in stop and 'pin' not in stop
    app = open(os.path.join(APP, 'app.js')).read()
    assert "PathikNative.openHead" in app and 'id="hub-head"' in open(os.path.join(APP, 'index.html')).read()
    java = os.path.join(ROOT, 'android', 'app', 'src', 'main', 'java', 'com', 'arpit', 'pathik', 'MainActivity.java')
    if os.path.exists(java):
        j = open(java).read()
        assert 'public void openHead' in j and '/app/head.html' in j and 'RESOURCE_VIDEO_CAPTURE' in j
