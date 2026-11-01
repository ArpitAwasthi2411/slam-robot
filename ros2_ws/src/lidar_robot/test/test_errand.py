"""Head-mode errands (web/app/mission.js): the step list, what the robot says, and the final spoken report,
for the normal case, a closed door, OK pressed early, an unreachable place and a cancel."""
import json
import os
import shutil
import subprocess

import pytest

APP = os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'web', 'app')

RUN = r"""
const M = require(process.argv[1]);
const c = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const said = [], waits = [];
let calls = 0;
const ops = {
  say: async t => { said.push(t); },
  go: async p => (c.goFail ? { result: 'failed', why: c.goFail } : { result: 'arrived' }),
  goBack: async b => ({ result: 'arrived' }),
  door: async p => ({ state: c.door || 'open' }),
  wait: async (s, l) => { waits.push(s); return c.waitEnd || 'done'; },
  cancelled: () => !!c.cancelAt && ++calls >= c.cancelAt,
};
const steps = M.plan(c.errand);
new M.Errand(ops).run(steps, () => {}).then(r => console.log(JSON.stringify({ steps: steps.map(s => s.kind), said, waits, r })));
"""


def run(case):
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    r = subprocess.run([node, '-e', RUN, os.path.abspath(os.path.join(APP, 'mission.js'))],
                       input=json.dumps(case), capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


SHASHI = {'to': 'Shashi sir cabin', 'waitS': 120, 'message': 'Ravi sir sent me with the file.', 'back': 'here'}


def test_plan_and_normal_run():
    o = run({'errand': SHASHI})
    assert o['steps'] == ['say', 'go', 'door', 'deliver', 'wait', 'say', 'back', 'report']
    assert o['said'][0] == 'Going to Shashi sir cabin.'
    assert 'Hello! Ravi sir sent me with the file.' in o['said']
    assert o['waits'] == [120]
    assert o['r']['ok'] and o['r']['report'] == \
        'I went to Shashi sir cabin, the door was open, I waited 2 minutes and I am back.'


def test_closed_door_still_waits_and_reports_it():
    o = run({'errand': SHASHI, 'door': 'closed'})
    assert 'The door of Shashi sir cabin is closed.' in o['said']
    assert any(s.startswith("I'll wait here outside") for s in o['said'])
    assert 'the door was closed' in o['r']['report']


def test_ok_pressed_early_and_until_ok():
    o = run({'errand': dict(SHASHI, waitS=0), 'waitEnd': 'ok'})
    assert o['waits'] == [300]                      # "until OK" waits at most 5 min
    assert 'Thank you!' in o['said'] and 'someone pressed OK' in o['r']['report']


def test_unreachable_place_goes_back_and_says_why():
    o = run({'errand': SHASHI, 'goFail': 'the corridor is blocked'})
    assert 'I could not reach Shashi sir cabin: the corridor is blocked.' in o['said']
    assert o['waits'] == []                         # no waiting at a place it never reached
    assert o['r']['report'].startswith('I could not reach Shashi sir cabin (the corridor is blocked)')
    assert o['r']['report'].endswith('and I am back.')


def test_cancel_and_stay():
    o = run({'errand': SHASHI, 'cancelAt': 3})
    assert not o['r']['ok'] and o['said'][-1] == 'Errand cancelled.'
    o = run({'errand': dict(SHASHI, back='stay')})
    assert 'back' not in o['steps'] and o['r']['report'].endswith('I waited 2 minutes.')
