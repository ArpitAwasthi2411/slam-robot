import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.places import PlaceStore  # noqa: E402
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.missions import MissionQueue, Mission, Stop  # noqa: E402


def store(tmp_path):
    s = PlaceStore(str(tmp_path / 'places.json'))
    s.add('HOD office', 5, 1, aliases=['hod', 'head of department'])
    s.add('Lab 3', 2, 4, aliases=['computer lab', 'lab three'])
    s.add('Principal office', 9, 2)
    s.add('Library', 1, 8)
    s.set_home(0, 0, 0)
    return s


def test_persistence_roundtrip(tmp_path):
    store(tmp_path)
    s2 = PlaceStore(str(tmp_path / 'places.json'))
    assert {p['name'] for p in s2.places} == {'HOD office', 'Lab 3', 'Principal office', 'Library'}
    assert s2.home == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    s2.remove('library')
    assert PlaceStore(str(tmp_path / 'places.json')).get('Library') is None


def test_search_handles_aliases_typos_and_partial(tmp_path):
    s = store(tmp_path)
    assert s.search('hod')[0][1]['name'] == 'HOD office'
    assert s.search('head of department')[0][1]['name'] == 'HOD office'
    assert s.search('principl ofice')[0][1]['name'] == 'Principal office'
    assert s.search('lab three')[0][1]['name'] == 'Lab 3'
    assert s.search('libary')[0][1]['name'] == 'Library'
    assert s.search('cafeteria') == []


def test_rules_parser_offline(tmp_path):
    p = CommandParser(store(tmp_path))          # no API key -> rules
    c = p.parse('Urgent: take this file to the HOD office and then lab 3')
    assert c['intent'] == 'navigate' and c['targets'] == ['HOD office', 'Lab 3'] and c['priority'] == 3
    assert c['source'] == 'rules'
    assert p.parse('go to principl offce')['targets'] == ['Principal office']
    assert p.parse('stop now')['intent'] == 'stop'
    assert p.parse('come back home')['intent'] == 'return_home'
    assert p.parse('where are you?')['intent'] == 'status'
    c = p.parse('go to the cafeteria')
    assert c['intent'] == 'unknown' and 'cafeteria' in c['reply']
    assert p.parse('library when you are free')['priority'] == 0


def test_llm_path_and_hallucination_guard(tmp_path):
    calls = []

    def fake_post(url, body, headers, timeout):
        calls.append(body)
        out = {'intent': 'navigate', 'targets': ['HOD Office', 'Cafeteria', 'lab 3'],
               'priority': 2, 'reply': 'On my way!'}
        return {'choices': [{'message': {'content': json.dumps(out)}}]}
    p = CommandParser(store(tmp_path), groq_api_key='x', http_post=fake_post)
    c = p.parse('drop these at the head of dept, the canteen, and the computer lab')
    assert c['source'] == 'llm'
    assert c['targets'] == ['HOD office', 'Lab 3']
    assert any('Cafeteria' in n for n in c['notes'])
    assert 'HOD office' in calls[0]['messages'][0]['content']     # map places given to the LLM


def test_llm_failure_falls_back_to_rules(tmp_path):
    def broken(*a, **k):
        raise TimeoutError('no internet')
    c = CommandParser(store(tmp_path), groq_api_key='x', http_post=broken).parse('go to library')
    assert c['source'] == 'rules' and c['targets'] == ['Library']
    assert 'offline' in c['notes'][0]


def test_mission_priorities_and_preemption():
    q = MissionQueue()
    a = Mission([Stop('A', 1, 0)], priority=1)
    b = Mission([Stop('B', 2, 0)], priority=0)
    c = Mission([Stop('C', 3, 0)], priority=1)
    for m in (a, b, c):
        q.add(m)
    assert q.next() is a
    urgent = Mission([Stop('U', 9, 9)], priority=3)
    assert q.add(urgent) is True              # should pre-empt
    q.preempt()
    assert q.next() is urgent
    q.finish('DONE')
    assert q.next() is a                      # interrupted mission resumes before c
    q.finish('DONE')
    assert q.next() is c
    q.finish('DONE')
    assert q.next() is b
    assert q.cancel() == 1 and q.active is None
