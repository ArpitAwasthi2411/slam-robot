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
        (st('DRIVING', goal={'label': 'Lab 3'}, blocked=True, blocker={'now': 'object'}), {}),
        (st('DRIVING', goal={'label': 'Lab 3'}, blocked=True, blocker={'now': 'person'}), {}),
    ]
    got = faces(cases)
    moods = [g['mood'] for g in got]
    assert moods == ['offline', 'stopped', 'help', 'lost', 'happy', 'blocked', 'blocked', 'thinking', 'driving',
                     'charging', 'lowbat', 'sleepy', 'idle', 'blocked', 'thinking', 'blocked']
    assert got[14]['line'] == 'Something is in my way' and got[15]['line'] == 'Excuse me, may I pass?'
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


def test_pin_is_asked_last_and_never_for_questions():
    html = open(os.path.join(APP, 'head.html')).read()
    js = open(os.path.join(APP, 'head.js')).read()
    for need in ('id="ic-go"', 'id="ic-ask"', 'id="ic-send"', 'id="ask"', 'id="deliver"'):
        assert need in html, need

    def body(name):
        a = js.index(name)
        return js[a:js.index('\n  }\n', a)]
    # opening a sheet never asks for the PIN: people look and choose first
    for fn in ('function openKiosk()', 'function openAsk()', 'function openDeliver()'):
        assert 'needPin' not in body(fn) and 'pinPad' not in body(fn), fn
    # sending a trip asks for it before the request goes to the robot
    send = body('async function send(req, label)')
    assert 0 < send.index('needPin(') < send.index('await nav(')
    # Ask: answers (chat / status / explain / find) go straight through; a trip needs the PIN first
    ask = body('async function ask(text)')
    answers = ask[ask.index('ANSWERS.includes'):ask.index('it wants the robot to move')]
    assert 'needPin' not in answers and 'say(reply)' in answers
    trip = ask[ask.index('it wants the robot to move'):]
    assert trip.index('needPin(') < trip.index("await nav({ type: 'command', text, source: 'head', priority: 1 })")
    errand = body('async function startErrand()')
    assert errand.index('needPin(') < errand.index('M.plan(e)')
    # tapping the face introduces the robot, it does not open the place list
    tap = body("$('face-wrap').onclick")
    assert 'ABOUT[' in tap and 'openKiosk' not in tap
    # the PIN pad sits above the sheet the choice was made in, and STOP above everything
    assert '#pin { z-index: 20; }' in html and 'z-index: 30' in html[html.index('.h-stop {'):html.index('body.estop')]
