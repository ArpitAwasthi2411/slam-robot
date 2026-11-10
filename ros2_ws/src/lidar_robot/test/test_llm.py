"""The Groq client and the LLM command path: why it failed on the robot, and the guards (llm.py, commands.py)."""
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.llm import (USER_AGENT, GroqClient, LLMError, LLMLog, classify_http, clean_key,  # noqa: E402
                             http_json, mask)
from lidar_robot.places import PlaceStore  # noqa: E402

KEY = 'gsk_' + 'a' * 40


def store(tmp_path):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Library', 1, 1, 0)
    s.add('Lab 3', 2, 2, 0, ['computer lab'])
    s.add('HOD office', 3, 3, 0, ['head of dept'])
    s.add_room('Lab 2', [[5, 5], [8, 5], [8, 8], [5, 8]], '3D printer, oscilloscope')
    return s


def reply(obj):
    return {'choices': [{'message': {'content': json.dumps(obj)}, 'finish_reason': 'stop'}],
            'usage': {'total_tokens': 123}}


def answer(intent='navigate', targets=(), priority=1, **kw):
    d = {'intent': intent, 'query': None, 'targets': list(targets), 'priority': priority, 'reply': 'ok',
         'waits': [], 'then_home': False, 'confirms': []}
    d.update(kw)
    return d


class Fake:
    """Scripted post(): a list of replies or exceptions, in order; records every body."""

    def __init__(self, *script):
        self.script, self.bodies, self.headers = list(script), [], []

    def __call__(self, url, body, headers, timeout):
        self.bodies.append(body)
        self.headers.append(headers)
        item = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        if isinstance(item, Exception):
            raise item
        return item


# ------------------------------------------------------------------ the three reasons it never worked
def test_real_http_sends_our_user_agent_and_reads_groq_errors():
    """Cloudflare blocks 'Python-urllib/x'. Our requests carry USER_AGENT; HTTP errors keep Groq's reason."""
    seen = {}

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            seen['ua'] = self.headers.get('User-Agent')
            n = int(self.headers.get('Content-Length', 0))
            seen['body'] = json.loads(self.rfile.read(n))
            if seen['body'].get('model') == 'old':
                msg = json.dumps({'error': {'message': 'The model `old` has been decommissioned',
                                            'code': 'model_decommissioned'}}).encode()
                self.send_response(400)
            else:
                msg = json.dumps(reply(answer())).encode()
                self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(msg)))
            self.end_headers()
            self.wfile.write(msg)

    srv = HTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{srv.server_address[1]}/x'
    try:
        assert http_json(url, {'model': 'm'}, {}, 3)['choices'][0]['message']
        assert seen['ua'] == USER_AGENT and 'urllib' not in seen['ua']
        with pytest.raises(LLMError) as e:
            http_json(url, {'model': 'old'}, {}, 3)
        assert e.value.kind == 'model' and e.value.status == 400 and 'decommissioned' in e.value.detail
    finally:
        srv.shutdown()


def test_retired_models_are_replaced_and_a_dead_model_falls_back():
    c = GroqClient(KEY, model='llama-3.1-8b-instant')
    assert c.model == 'openai/gpt-oss-20b' and c.config()['retired_model_replaced']
    post = Fake(LLMError('model', 'model_not_found', 404), reply({'a': 1}))
    c = GroqClient(KEY, model='some/new-model', post=post)
    assert c.chat_json('command', [{'role': 'user', 'content': 'x'}]) == {'a': 1}
    assert [b['model'] for b in post.bodies] == ['some/new-model', 'openai/gpt-oss-20b']
    assert 'some/new-model' in c.dead_models
    c.chat_json('command', [{'role': 'user', 'content': 'x'}])
    assert post.bodies[-1]['model'] == 'openai/gpt-oss-20b'          # never asks the dead one again


def test_error_classes_and_hints():
    assert classify_http(401, b'{"error":{"message":"Invalid API Key","code":"invalid_api_key"}}')[0] == 'auth'
    assert classify_http(403, b'error code: 1010')[0] == 'blocked'
    assert classify_http(403, b'{"error":{"message":"no access"}}')[0] == 'forbidden'
    assert classify_http(404, b'{"error":{"message":"model x does not exist"}}')[0] == 'model'
    assert classify_http(429, b'{}')[0] == 'rate_limit'
    assert classify_http(503, b'')[0] == 'server'
    assert classify_http(400, b'{"error":{"code":"json_validate_failed"}}')[0] == 'bad_output'
    assert 'robot.env' in LLMError('auth').hint and 'hotspot' in LLMError('blocked').hint


def test_auth_error_does_not_try_other_models_and_is_logged_with_hint():
    post = Fake(LLMError('auth', 'Invalid API Key', 401))
    log = LLMLog()
    c = GroqClient(KEY, post=post, log=log)
    with pytest.raises(LLMError) as e:
        c.chat_json('command', [{'role': 'user', 'content': 'go'}], input_text='go')
    assert e.value.kind == 'auth' and len(post.bodies) == 1
    snap = log.snapshot()
    assert snap['totals']['failed'] == 1 and snap['last_error']['kind'] == 'auth'
    assert 'console.groq.com' in snap['calls'][0]['hint']


def test_request_is_cheap_and_fast_and_strict():
    post = Fake(reply({'a': 1}))
    GroqClient(KEY, post=post).chat_json('command', [{'role': 'user', 'content': 'x'}],
                                         schema={'type': 'object'})
    b = post.bodies[0]
    assert b['temperature'] == 0 and b['reasoning_effort'] == 'low' and b['include_reasoning'] is False
    assert b['response_format']['type'] == 'json_schema' and b['response_format']['json_schema']['strict']
    assert post.headers[0]['User-Agent'] == USER_AGENT


def test_strict_schema_refused_retries_in_json_mode_and_empty_answer_is_an_error():
    post = Fake(LLMError('bad_output', 'json_validate_failed', 400), reply({'ok': True}))
    c = GroqClient(KEY, post=post)
    assert c.chat_json('command', [{'role': 'user', 'content': 'x'}], schema={'type': 'object'}) == {'ok': True}
    assert post.bodies[1]['response_format'] == {'type': 'json_object'}
    empty = Fake({'choices': [{'message': {'content': ''}, 'finish_reason': 'length'}]})
    with pytest.raises(LLMError) as e:
        GroqClient(KEY, post=empty).chat_json('command', [{'role': 'user', 'content': 'x'}])
    assert e.value.kind == 'bad_output' and 'length' in e.value.detail


def test_key_cleaning_and_masking():
    assert clean_key(' "gsk_abc" ') == 'gsk_abc'
    assert clean_key('export GROQ_API_KEY=gsk_xyz') == 'gsk_xyz'
    assert clean_key('') is None and clean_key(None) is None
    assert GroqClient(None).key_problem() == 'no_key'
    assert GroqClient('sk-openai-key').key_problem() == 'bad_key_format'
    assert KEY not in mask(f'Bearer {KEY} failed', KEY) and 'gsk_' in mask(f'x {KEY}', KEY)
    assert KEY not in json.dumps(GroqClient(KEY).config())


def test_diagnose_reports_each_step():
    ok = GroqClient(KEY, get=lambda u, h, t: {'data': [{'id': 'openai/gpt-oss-20b'}, {'id': 'whisper-large-v3'}]})
    d = ok.diagnose()
    assert d['ok'] and [s['name'] for s in d['steps']][:3] == ['key', 'reach Groq', 'models']

    def blocked(u, h, t):
        raise LLMError('blocked', 'error code: 1010', 403)
    d = GroqClient(KEY, get=blocked).diagnose()
    assert not d['ok'] and 'Cloudflare' in d['summary']
    assert not GroqClient(None).diagnose()['ok']
    gone = GroqClient(KEY, get=lambda u, h, t: {'data': [{'id': 'something-else'}]}).diagnose()
    assert not gone['ok'] and 'retired' in gone['summary']


# ------------------------------------------------------------------ the command parser policy and guards
def test_smart_policy_skips_the_llm_when_the_rules_are_sure(tmp_path):
    post = Fake(reply(answer(targets=['Library'])))
    p = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post)
    c = p.parse('go to the library')
    assert c['targets'] == ['Library'] and c['source'] == 'rules' and not post.bodies
    assert c['llm']['used'] is False and 'sure' in c['llm']['why']
    assert p.log.snapshot()['totals']['skipped'] == 1


def test_stop_never_waits_for_the_network(tmp_path):
    post = Fake(reply(answer(intent='navigate', targets=['Library'])))
    for pol in ('smart', 'always'):
        c = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post, policy=pol).parse('stop right now')
        assert c['intent'] == 'stop'
    assert not post.bodies


def test_unsure_commands_go_to_the_llm_with_place_enum(tmp_path):
    post = Fake(reply(answer(targets=['HOD office', 'Lab 3'])))
    p = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post)
    c = p.parse('drop these at the head of dept, the canteen, and the computer lab')
    assert c['source'] == 'llm' and c['targets'] == ['HOD office', 'Lab 3'] and c['llm']['used']
    schema = post.bodies[0]['response_format']['json_schema']['schema']
    assert set(schema['properties']['targets']['items']['enum']) >= {'Library', 'Lab 3', 'HOD office', 'Lab 2'}
    assert schema['additionalProperties'] is False and 'navigate' in schema['properties']['intent']['enum']


def test_hallucinated_place_dropped_and_urgent_needs_urgent_words(tmp_path):
    post = Fake(reply(answer(targets=['Lab 3', 'Moon base'], priority=3)))
    c = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post, policy='always').parse('lab 3 pls')
    assert c['targets'] == ['Lab 3'] and any('Moon base' in n for n in c['notes'])
    assert c['priority'] == 2 and any('urgent' in n for n in c['notes'])
    post2 = Fake(reply(answer(targets=['Lab 3'], priority=3)))
    c = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post2, policy='always').parse('lab 3 jaldi')
    assert c['priority'] == 3


def test_llm_unknown_never_overrides_a_rules_answer(tmp_path):
    post = Fake(reply(answer(intent='unknown')))
    c = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post, policy='always').parse('go to library')
    assert c['intent'] == 'navigate' and c['targets'] == ['Library'] and c['source'] == 'rules'


def test_errors_fall_back_with_the_reason_and_the_preview_is_cached(tmp_path):
    p = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=Fake(LLMError('model', 'decommissioned', 404)),
                      policy='always')
    c = p.parse('go to library')
    assert c['source'] == 'rules' and c['llm']['error']['kind'] == 'model' and 'offline' in c['notes'][0]
    post = Fake(reply(answer(targets=['Lab 2'])))
    p = CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post)
    a = p.parse('take me to the 3D printer')              # app preview (dry run) ...
    b = p.parse('take me to the 3D printer')              # ... then the real send: one LLM call
    assert a['targets'] == b['targets'] == ['Lab 2'] and len(post.bodies) == 1 and b['llm']['cached']


def test_no_key_and_off_policy_never_call(tmp_path):
    post = Fake(reply(answer(targets=['Lab 3'])))
    assert CommandParser(store(tmp_path), http_post=post).parse('the canteen and lab 3')['llm']['why'] == \
        'no GROQ_API_KEY'
    CommandParser(store(tmp_path), groq_api_key=KEY, http_post=post, policy='off').parse('the canteen and lab 3')
    assert not post.bodies


def test_navigator_core_serves_the_log_and_uses_a_preparsed_command(tmp_path):
    from lidar_robot.navigator_core import NavigatorCore
    s = store(tmp_path)
    p = CommandParser(s, groq_api_key=KEY, http_post=Fake(reply(answer(targets=['Lab 3']))))
    core = NavigatorCore(s, parser=p)
    p.parse('go to the library')
    r = core.handle({'type': 'llm_log'}, None)
    assert r['ok'] and r['llm']['policy'] == 'smart' and r['llm']['config']['model'] == 'openai/gpt-oss-20b'
    assert r['llm']['calls'][0]['outcome'] == 'skipped'
    pre = {'intent': 'stop', 'targets': [], 'priority': 3, 'reply': 'Stopping.', 'source': 'llm', 'notes': [],
           'steps': [], 'then_home': False}
    res = core.handle({'type': 'command', 'text': 'whatever', '_parsed': pre}, None)
    assert res['interpretation']['intent'] == 'stop'
