"""Error recovery: fixed action menu, JSON validation + clamping, rules fallback, and the navigator acting on it."""
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from test_navigator_core import Clock, make_setup, run  # noqa: E402
from lidar_robot.auth import required_role  # noqa: E402
from lidar_robot.recovery import MENU, Recovery, RecoveryLLM, allowed_actions, rule_decide, validate  # noqa: E402


def ctx(code='BLOCKED', stops=('Lab 3',), next_stop=0, queue=(), alts=(), bat='ok', tries=0, home=True, dock=True):
    c = {'failure': {'code': code, 'text': 'x', 'place': stops[next_stop], 'stop_index': next_stop},
         'mission': {'id': 1, 'stops': list(stops), 'next_stop': next_stop},
         'queue': list(queue), 'battery': {'level': bat, 'pct': 50}, 'alternatives': list(alts),
         'tries_here': tries, 'home_set': home, 'dock_set': dock, 'going_home': False}
    c['allowed'] = allowed_actions(c)
    return c


def fake_llm(answer=None, exc=None):
    calls = []

    def post(url, body, headers, timeout):
        calls.append(body)
        if exc:
            raise exc
        return {'choices': [{'message': {'content': json.dumps(answer)}}]}
    return RecoveryLLM('test-key', model='m', http_post=post), calls


# ------------------------------------------------------------------ menu, validation, rules
def test_menu_is_the_fixed_list():
    assert list(MENU) == ['wait_retry', 'replan_wide', 'alt_route', 'skip_stop', 'reorder', 'ask_person',
                          'go_home', 'go_dock']


def test_validate_rejects_invented_or_impossible_actions_and_clamps():
    c = ctx(stops=('Lab 3',))
    with pytest.raises(ValueError):
        validate({'action': 'drive_through_wall'}, c)
    with pytest.raises(ValueError):
        validate({'action': 'skip_stop'}, c)              # one stop only: nothing to skip to
    with pytest.raises(ValueError):
        validate('wait', c)
    assert validate({'action': 'wait_retry', 'params': {'wait_s': 999}}, c)['params'] == {'wait_s': 120.0}
    assert validate({'action': 'wait_retry', 'params': {'wait_s': 'nan'}}, c)['params'] == {'wait_s': 15.0}
    assert validate({'action': 'wait_retry', 'params': {'wait_s': -4}}, c)['params'] == {'wait_s': 3.0}
    assert validate({'action': 'replan_wide', 'params': {'margin_m': 5}}, c)['params'] == {'margin_m': 0.25}
    c2 = ctx(alts=[{'id': 'via1', 'label': 'Store', 'x': 1, 'y': 2}])
    assert validate({'action': 'alt_route', 'params': {'via': 'via1'}}, c2)['params'] == {'via': 'via1'}
    with pytest.raises(ValueError):
        validate({'action': 'alt_route', 'params': {'via': 'secret door'}}, c2)
    q = validate({'action': 'ask_person', 'params': {'question': 'x' * 500}}, c)['params']['question']
    assert len(q) == 160


def test_allowed_depends_on_the_situation():
    assert 'skip_stop' not in ctx(stops=('A',))['allowed']
    assert 'skip_stop' in ctx(stops=('A', 'B'))['allowed'] and 'reorder' in ctx(stops=('A', 'B'))['allowed']
    assert 'reorder' in ctx(queue=[{'id': 2}])['allowed']
    assert 'go_home' not in ctx(home=False)['allowed'] and 'go_dock' not in ctx(dock=False)['allowed']
    assert 'replan_wide' not in ctx(code='NO_PATH')['allowed']
    assert 'go_dock' not in ctx(code='DOCK_NO_CONTACT')['allowed']


def test_rules_escalate_and_charge_on_low_battery():
    assert rule_decide(ctx(tries=0))['action'] == 'wait_retry'
    assert rule_decide(ctx(tries=1))['action'] == 'replan_wide'          # no alternatives listed
    assert rule_decide(ctx(tries=1, alts=[{'id': 'via1', 'label': 'S', 'x': 0, 'y': 0}]))['params'] == {'via': 'via1'}
    assert rule_decide(ctx(tries=2))['action'] == 'ask_person'           # single stop, no queue
    assert rule_decide(ctx(tries=9)) is None
    assert rule_decide(ctx(code='GOAL_IN_OBSTACLE', stops=('A', 'B')))['action'] == 'skip_stop'
    assert rule_decide(ctx(bat='low'))['action'] == 'go_dock'
    assert rule_decide(ctx(bat='critical', dock=False))['action'] == 'go_home'


def test_llm_decision_is_used_and_logged_with_its_reason():
    llm, calls = fake_llm({'action': 'wait_retry', 'params': {'wait_s': 30}, 'reason': 'people in the corridor'})
    rec = Recovery(llm=llm, threaded=False)
    rec.start(ctx())
    d = rec.poll()
    assert d['action'] == 'wait_retry' and d['params'] == {'wait_s': 30.0}
    assert rec.current['source'] == 'llm' and rec.current['why'] == 'people in the corridor'
    sent = json.loads(calls[0]['messages'][1]['content'])
    assert sent['failure']['code'] == 'BLOCKED' and 'allowed' in sent and 'home_set' not in sent
    assert 'wait_retry' in calls[0]['messages'][0]['content'] and calls[0]['model'] == 'm'


@pytest.mark.parametrize('answer,exc,note', [
    ({'action': 'teleport'}, None, 'rejected'),
    ({'action': 'skip_stop'}, None, 'rejected'),                  # not possible with one stop
    (None, OSError('no network'), 'unavailable'),
])
def test_bad_llm_answers_fall_back_to_rules(answer, exc, note):
    llm, _ = fake_llm(answer, exc)
    rec = Recovery(llm=llm, threaded=False)
    rec.start(ctx())
    d = rec.poll()
    assert d['action'] == 'wait_retry' and rec.current['source'] == 'rules' and note in rec.current['note']


def test_no_key_means_rules_and_says_so():
    rec = Recovery(llm=RecoveryLLM(None), threaded=False)
    rec.start(ctx())
    assert rec.poll()['action'] == 'wait_retry' and rec.current['source'] == 'rules'


def test_slow_llm_does_not_block_and_times_out_to_rules():
    import threading
    gate = threading.Event()

    def slow(url, body, headers, timeout):
        gate.wait(5)
        return {'choices': [{'message': {'content': '{"action": "go_home"}'}}]}
    clk = Clock()
    rec = Recovery(llm=RecoveryLLM('k', timeout=1.0, http_post=slow), clock=clk)
    rec.start(ctx())
    assert rec.poll() is None                 # still thinking: the navigator keeps the robot stopped
    clk.t = 3.5                               # past timeout + 2 s
    d = rec.poll()
    gate.set()
    assert d['action'] == 'wait_retry' and 'too slow' in rec.current['note']


# ------------------------------------------------------------------ the navigator acting on decisions
def nav_with_recovery(tmp_path, mode='rules', llm=None, **kw):
    nav, clk = make_setup(tmp_path, recovery=Recovery(llm=llm, mode=mode, threaded=False, **kw))
    return nav, clk


def test_unreachable_stop_in_a_trip_is_skipped_and_the_trip_finishes(tmp_path):
    nav, clk = nav_with_recovery(tmp_path)
    nav.handle({'type': 'goto_poses', 'waypoints': [{'x': 2.0, 'y': 1.5, 'label': 'table'}, {'x': 6.0, 'y': 1.0, 'label': 'Lab 3'}]})
    nav.queue.queue[0].skip_unreachable = False        # a command trip, not a patrol route
    x, y, _ = run(nav, clk, (1.0, 1.0, 0.0), 200)
    assert math.hypot(x - 6.0, y - 1.0) < 0.25
    h = nav.queue.history[0]
    assert h.state == 'DONE'
    rec = nav.mlog.recent(1)[0]['recoveries'][0]
    assert rec['action'] == 'skip_stop' and rec['source'] == 'rules' and rec['code'] == 'GOAL_IN_OBSTACLE'
    assert nav.status()['recovery']['outcome'] == 'skipped'
    assert nav.mlog.stats()['recoveries'] == 1 and nav.mlog.stats()['recovered'] == 1


def test_blocked_way_waits_then_recovers_when_the_person_moves(tmp_path):
    nav, clk = nav_with_recovery(tmp_path)
    nav.handle({'type': 'goto_place', 'name': 'Lab 3'})
    # a person stands in front of the robot for the first 40 s
    x, y, yaw = run(nav, clk, (1.0, 1.0, 0.0), 300, front=lambda p: 0.25 if clk.t < 40 else float('inf'))
    assert math.hypot(x - 6.0, y - 1.0) < 0.25, (x, y)
    assert nav.queue.history[0].state == 'DONE'
    recs = nav.mlog.recent(1)[0]['recoveries']
    assert recs and recs[0]['action'] == 'wait_retry'
    assert nav.rec.history[0]['outcome'] == 'recovered'


def test_ask_a_person_then_the_answer_from_the_app_is_carried_out(tmp_path):
    nav, clk = nav_with_recovery(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    st = nav.status()
    assert nav.state == 'RECOVERING' and st['recovery']['status'] == 'asking'
    ids = [o['id'] for o in st['recovery']['ask']['options']]
    assert 'retry' in ids and 'home' in ids and 'cancel' in ids and 'skip' not in ids
    assert required_role('POST', '/api/nav', {'type': 'recovery_answer'}) == 'staff'
    assert nav.handle({'type': 'recovery_answer', 'id': 999, 'choice': 'home'})['ok'] is False
    assert nav.handle({'type': 'recovery_answer', 'id': st['recovery']['id'], 'choice': 'jump'})['ok'] is False
    r = nav.handle({'type': 'recovery_answer', 'id': st['recovery']['id'], 'choice': 'home', 'by': 'staff'})
    assert r['ok'] and nav.queue.history[0].state == 'FAILED'
    assert (nav.queue.active or nav.queue.queue[0]).stops[0].label == 'home'
    assert nav.rec.history[0]['outcome'] == 'gave_up_home' and nav.rec.history[0]['source'] == 'person'


def test_nobody_answers_then_the_rules_decide(tmp_path):
    nav, clk = nav_with_recovery(tmp_path, ask_timeout_s=30)
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    assert nav.status()['recovery']['status'] == 'asking'
    for _ in range(int(35 / 0.05)):
        nav.step((1.0, 1.0, 0.0), float('inf'), 0.05)
        clk.t += 0.05
    assert nav.queue.history and nav.queue.history[0].state == 'FAILED'   # nothing else possible: it stops
    assert 'nobody answered' in nav.rec.history[0]['why']


def test_recovery_budget_then_the_mission_fails(tmp_path):
    nav, clk = nav_with_recovery(tmp_path, max_per_mission=1)
    nav.handle({'type': 'goto_place', 'name': 'Lab 3'})
    run(nav, clk, (1.0, 1.0, 0.0), 400, front=lambda p: 0.25)        # blocked for good
    h = nav.queue.history[0]
    assert h.state == 'FAILED' and 'BLOCKED' in h.message
    assert len(nav.mlog.recent(1)[0]['recoveries']) == 1


def test_estop_and_cancel_end_a_recovery(tmp_path):
    nav, clk = nav_with_recovery(tmp_path)
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    assert nav.rec.busy()
    nav.set_estop(True)
    assert not nav.rec.busy() and nav.state != 'RECOVERING'
    assert nav.step((1.0, 1.0, 0.0), 9, 0.05) == (0.0, 0.0)


def test_robot_stands_still_while_recovering_and_explain_says_why(tmp_path):
    llm, _ = fake_llm({'action': 'wait_retry', 'params': {'wait_s': 30}, 'reason': 'a trolley'})
    nav, clk = nav_with_recovery(tmp_path, mode='llm', llm=llm)
    nav.handle({'type': 'goto_place', 'name': 'Lab 3'})
    x, y, yaw = 1.0, 1.0, 0.0
    for _ in range(int(200 / 0.05)):                  # something stays in front of the robot
        v, w = nav.step((x, y, yaw), 0.25, 0.05)
        x += v * math.cos(yaw) * 0.05
        y += v * math.sin(yaw) * 0.05
        yaw += w * 0.05
        clk.t += 0.05
        if nav.state == 'RECOVERING' and nav.rec.current['status'] == 'running':
            break
    pose = (x, y, yaw)
    assert nav.state == 'RECOVERING' and nav.rec.current['source'] == 'llm'
    assert nav.step(pose, 0.25, 0.05) == (0.0, 0.0)
    ans = nav.handle({'type': 'explain'})['answer']
    assert 'language model' in ans and 'Waiting 30 s' in ans
    assert nav.mlog.rec(nav.queue.active)['recoveries'][0]['reason'] == 'a trolley'
    assert ('recovery', '') not in nav.say_queue


def test_status_reports_the_recovery_for_the_app(tmp_path):
    nav, clk = nav_with_recovery(tmp_path)
    assert nav.status()['recovery'] is None and nav.status()['recovery_mode'] == 'rules'
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    r = nav.status()['recovery']
    for k in ('id', 'code', 'what_happened', 'doing', 'source', 'status', 'ask'):
        assert k in r
    assert not any(k.startswith('_') for k in r)
    json.dumps(nav.status())                                 # the app gets it as JSON


def test_off_mode_keeps_the_old_behaviour(tmp_path):
    nav, clk = nav_with_recovery(tmp_path, mode='off')
    nav.handle({'type': 'goto_pose', 'x': 2.0, 'y': 1.5, 'label': 'inside table'})
    run(nav, clk, (1.0, 1.0, 0.0), 10)
    assert nav.queue.history[0].state == 'FAILED' and nav.status()['recovery'] is None
