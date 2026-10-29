"""Pathik app: no feature lost in the redesign (docs/APP_FEATURES.md).

Two layers:
- static: every element id and every server call the inventory relies on is still in the app files
  (runs everywhere, also inside the CI Docker image);
- browser: test/app_parity.js opens the demo robot in Chromium as staff, operator and admin and clicks
  through the features (skipped when node or the playwright package is missing).
"""
import json
import os
import re
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, '..', 'lidar_robot', 'web', 'app')
DOC = os.path.join(HERE, '..', '..', '..', '..', 'docs', 'APP_FEATURES.md')


def _read(name):
    with open(os.path.join(APP, name), encoding='utf-8') as f:
        return f.read()


# element ids for the inventory rows (C = connect, L = login, M = frame, T = tools, D = drive, G = home/send,
# P = map sheet, B = operator area, E = me/settings)
IDS = '''
c-scan c-card-name c-card-net c-card-lat c-card-fw c-progress c-demo c-list c-manual c-addr
au-back au-user au-pass au-show au-name au-setup au-go au-pending au-switch-b au-demo-row au-msg
map t-robot t-dot t-host t-public t-link t-cam t-bat t-mode m-stats m-mode m-banner m-confirm m-why m-vis
cam cam-img cam-demo cam-act cam-fps cam-why cam-people cam-size cam-x m-hint estop estopped es-release
action a-go a-save a-home a-cancel a-route toast
m-home m-follow path-sheet ps-len ps-eta ps-warn ps-loop ps-go ps-redraw ps-close
joy joy-lock d-speed d-speed-out d-v d-w
g-hello g-where g-ava q-home q-why q-charge g-cmd g-cmd-in g-voice g-plan g-reply g-mission
g-here g-home g-search g-places g-rooms g-charge g-dock g-bat g-sched-add g-sched
sm-where x-card x-go x-stop x-spin x-area x-time x-done x-left x-last d-savemap d-locok map-mode-note
s-list s-events s-disconnect w-chart w-live w-step w-step-v w-metrics w-params w-save w-reset
f-params f-save f-reset t-speed t-secs t-run t-chart t-metrics l-stats l-list l-refresh l-csv
me-public me-public-url me-ava me-name me-user me-role people pp-refresh tab-me-badge
me-robot me-net me-dot me-switch me-pass me-out
h-state h-bat h-link h-room h-recovery h-docking h-send q-cam h-vis-card h-voice-card sheet-grip
'''.split()
TOOLS = ['goal', 'draw', 'route', 'measure', 'room', 'locate']
CALLS = ['/api/ping', '/api/state', '/api/map?since=', '/api/camera.jpg', '/api/telemetry?since=', '/api/test',
         '/api/cmd', '/api/estop', '/api/nav', '/api/relocalize', '/api/save_map', '/api/esp', '/api/auth/me',
         '/api/auth/login', '/api/auth/signup', '/api/auth/logout', '/api/auth/password', '/api/users',
         '/api/users/update', '/api/users/reset', '/api/users/delete']
NAV = ['preview', 'goto_pose', 'goto_poses', 'goto_place', 'goto_room', 'go_home', 'go_dock', 'cancel', 'confirm',
       'confirm_localization', 'explain', 'command', 'check_path', 'follow_path', 'add_place', 'delete_place',
       'set_home', 'set_dock', 'add_room', 'update_room', 'delete_room', 'schedule_add', 'schedule_enable',
       'schedule_delete', 'explore_start', 'explore_stop', 'get_tune', 'tune', 'mission_log']
ESP_OPS = ['tune', 'save', 'defaults', 'telemetry']


def test_every_inventory_element_still_exists():
    html = _read('index.html')
    ids = set(re.findall(r'id="([^"]+)"', html))
    missing = [i for i in IDS if i not in ids]
    assert not missing, f'elements gone from index.html: {missing}'
    for t in TOOLS:
        assert f'data-tool="{t}"' in html, t
    for sub in ['health', 'wheels', 'path', 'tests', 'log', 'people']:
        assert f'data-sub="{sub}"' in html and f'id="lab-{sub}"' in html, sub
    for theme in ['system', 'light', 'dark']:
        assert f'data-theme-pick="{theme}"' in html


def test_every_server_call_is_still_made():
    js = _read('app.js')
    for c in CALLS:
        assert c in js, f'the app no longer calls {c}'
    for t in NAV:
        assert f"type: '{t}'" in js, f'the app no longer sends nav {t}'
    for op in ESP_OPS:
        assert f"op: '{op}'" in js, op
    assert 'resume' in js and "data-a=\"resume\"" in js


def test_role_gating_kept():
    html = _read('index.html')
    css = _read('app.css')
    assert 'body.role-staff [data-min="operator"]' in css and 'body:not(.role-staff) [data-max="staff"]' in css
    for sel in ['data-tab="drive" data-min="operator"', 'data-sub="people" data-min="admin"',
                'id="es-release" class="btn hold" data-min="operator"', 'id="x-card" class="card stack xcard" data-min="operator"']:
        assert sel in html, sel


def test_no_external_resources():
    for name in ['index.html', 'app.css', 'app.js', 'demo.js']:
        text = _read(name)
        assert not re.search(r'(src|href)\s*=\s*["\']https?://', text), name
        assert '@import' not in text and 'fonts.googleapis' not in text, name


def test_inventory_is_complete():
    if not os.path.exists(DOC):
        pytest.skip('docs/ not in this checkout')
    with open(DOC, encoding='utf-8') as f:
        rows = [line for line in f if re.match(r'\| [A-Z]\d+ \|', line)]
    assert len(rows) > 90
    empty = [r.split('|')[1].strip() for r in rows if not r.rstrip().rstrip('|').split('|')[-1].strip()]
    assert not empty, f'APP_FEATURES.md rows without "Now lives": {empty}'


def test_parity_in_a_browser():
    node = shutil.which('node')
    if not node:
        pytest.skip('node not installed')
    try:
        out = subprocess.run([node, os.path.join(HERE, 'app_parity.js')], capture_output=True, text=True, timeout=400)
    except subprocess.TimeoutExpired:
        pytest.fail('app_parity.js timed out')
    line = (out.stdout.strip().splitlines() or ['{}'])[-1]
    res = json.loads(line)
    if res.get('skip'):
        pytest.skip(res['skip'])
    assert res.get('ok'), '\n'.join(res.get('failures', [])) or out.stderr[-2000:]
    assert res['checked'] > 200
