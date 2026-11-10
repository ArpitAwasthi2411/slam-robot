"""Conversation agent + notebook (chat.py, knowledge.py): grounded answers, memory, trips from talk, guards."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.auth import required_role  # noqa: E402
from lidar_robot.chat import DONT_KNOW, NOT_SURE, ChatAgent  # noqa: E402
from lidar_robot.commands import CommandParser  # noqa: E402
from lidar_robot.knowledge import Knowledge  # noqa: E402
from lidar_robot.llm import GroqClient, LLMError  # noqa: E402
from lidar_robot.navigator_core import NavigatorCore  # noqa: E402
from lidar_robot.places import PlaceStore  # noqa: E402

KEY = 'gsk_' + 'a' * 40


def store(tmp_path):
    s = PlaceStore(str(tmp_path / 'p.json'))
    s.add('Library', 1, 1, 0)
    s.add('Lab 3', 8, 1, 0, ['computer lab'])
    s.add('HOD office', 3, 3, 0, ['head of dept'])
    s.add('Canteen', 20, 3, 0)
    s.add_room('Lab 2', [[5, 5], [8, 5], [8, 8], [5, 8]], '3D printer, oscilloscope')
    return s


def notebook(tmp_path):
    k = Knowledge(str(tmp_path / 'k.json'))
    k.add('Head of the department', 'The head of the CSE department is Dr. Meera Rao. Her cabin is the HOD office.',
          ['hod', 'head'])
    k.add('Library timings', 'The library is open from 9 am to 5 pm on working days.', ['library', 'timing'])
    return k


def say(reply='ok', basis='general', sources=(), search=None, action='none', targets=(), **kw):
    d = {'reply': reply, 'basis': basis, 'sources': list(sources), 'search': search, 'action': action,
         'targets': list(targets), 'waits': [], 'confirms': [], 'then_home': False, 'shortest_order': False,
         'priority': 1}
    d.update(kw)
    return {'choices': [{'message': {'content': json.dumps(d)}, 'finish_reason': 'stop'}], 'usage': {}}


class Fake:
    def __init__(self, *script):
        self.script, self.bodies = list(script), []

    def __call__(self, url, body, headers, timeout):
        self.bodies.append(body)
        item = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        if isinstance(item, Exception):
            raise item
        return item


def make(tmp_path, *script, key=KEY):
    fake = Fake(*script) if script else Fake(say())
    places, k = store(tmp_path), notebook(tmp_path)
    agent = ChatAgent(GroqClient(key, post=fake), k, places, wall=lambda: 1794000000.0)
    parser = CommandParser(places, client=GroqClient(key, post=fake))
    return agent, parser, fake, k


# ------------------------------------------------------------------ the notebook
def test_notebook_search_finds_by_words_tags_and_typos(tmp_path):
    k = notebook(tmp_path)
    assert k.search('who is the hod')[0][1]['title'] == 'Head of the department'
    assert k.search('libary timing kya hai')[0][1]['title'] == 'Library timings'      # typo + Hinglish
    assert k.search('who won the cricket world cup') == []
    assert k.search('kya hai') == []                                                  # only filler words


def test_notebook_is_saved_and_built_in_entries_are_protected(tmp_path):
    k = notebook(tmp_path)
    e = k.add('Canteen', 'Lunch is served from 12:30.', 'food, lunch')
    assert e['tags'] == ['food', 'lunch']
    k2 = Knowledge(str(tmp_path / 'k.json'))
    assert [x['title'] for x in k2.entries] == ['Head of the department', 'Library timings', 'Canteen']
    k2.update(e['id'], text='Lunch is served from 1 pm.')
    assert 'from 1 pm' in k2.get(e['id'])['text']
    k2.remove(e['id'])
    with pytest.raises(ValueError):
        k2.remove('pathik-who')
    with pytest.raises(ValueError):
        k2.add('', 'no title')
    assert any(x['id'] == 'pathik-who' for x in k2.all())


# ------------------------------------------------------------------ routing
def test_question_that_names_a_place_is_not_a_trip(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('It is open 9 am to 5 pm.', 'knowledge', ['k2']))
    assert parser._parse_rules('library timings?')['intent'] == 'navigate'     # what the rules alone would do
    assert agent.handle('library timings?', parser)['intent'] == 'chat'
    assert agent.handle('Library', parser)['intent'] == 'navigate'             # a bare place name is still a trip
    assert agent.handle('can you take me to the library?', parser)['intent'] == 'navigate'


def test_clear_commands_never_reach_the_conversation_model(tmp_path):
    agent, parser, fake, _ = make(tmp_path)
    assert agent.handle('stop', parser)['intent'] == 'stop'
    assert agent.handle('go to the library', parser)['targets'] == ['Library']
    assert agent.handle('where is the 3D printer', parser)['intent'] == 'find'
    assert fake.bodies == []


def test_greeting_and_time_are_answered_offline(tmp_path):
    agent, parser, fake, _ = make(tmp_path)
    r = agent.handle('hello', parser)
    assert r['intent'] == 'chat' and 'Pathik' in r['reply']
    assert agent.handle('what can you do?', parser)['chat']['sources'][0]['id'] == 'pathik-can'
    assert agent.handle('what time is it', parser)['reply'].startswith('It is ')
    assert fake.bodies == []


def test_college_question_is_answered_from_the_notebook_with_its_source(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('The head of the CSE department is Dr. Meera Rao.',
                                                 'knowledge', ['k1']))
    r = agent.handle('who is the head of the department?', parser, user='arpit',
                     ctx={'robot': {'state': 'IDLE', 'battery_pct': 80}})
    assert r['intent'] == 'chat' and 'Meera Rao' in r['reply']
    assert r['chat']['basis'] == 'knowledge' and r['chat']['sources'] == [{'id': 'k1',
                                                                          'title': 'Head of the department'}]
    body = fake.bodies[0]
    system = body['messages'][0]['content']
    assert 'Dr. Meera Rao' in system and '"battery_pct":80' in system and 'HOD office' in system
    schema = body['response_format']['json_schema']['schema']
    assert 'k1' in schema['properties']['sources']['items']['enum']            # only the entries it was shown
    assert 'Canteen' in schema['properties']['targets']['items']['enum']       # only places on the map


def test_where_is_a_person_goes_to_the_notebook_not_the_thing_finder(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('Dr. Meera Rao sits in the HOD office.', 'knowledge', ['k1']))
    r = agent.handle('where is the head of department sitting', parser)
    assert r['intent'] == 'chat' and len(fake.bodies) == 1


# ------------------------------------------------------------------ guards
def test_made_up_source_is_not_trusted(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('The principal is Mr. X.', 'knowledge', ['k99']))
    r = agent.handle('who is the principal', parser)
    assert r['reply'] == DONT_KNOW and r['chat']['basis'] == 'unknown'
    assert agent.stats['guarded'] == 1


def test_number_that_is_not_in_the_notes_replaces_the_answer(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('The library is open from 8 am to 10 pm, call 9876543210.',
                                             'knowledge', ['k2']))
    r = agent.handle('library timings?', parser)
    assert r['reply'] == NOT_SURE and 'number not in my notes' in r['chat']['flags'][0]
    # the same check lets a correct answer through, and leaves general knowledge alone
    agent, parser, _, _ = make(tmp_path, say('It is open from 9 am to 5 pm.', 'knowledge', ['k2']))
    assert '9 am to 5 pm' in agent.handle('library timings?', parser)['reply']
    agent, parser, _, _ = make(tmp_path, say('ROS 2 was first released in 2017.', 'general'))
    assert '2017' in agent.handle('when was ros 2 released', parser)['reply']


def test_reply_is_plain_and_short(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('**SLAM** means:\n- mapping\n- localising. ' + 'More words. ' * 80))
    r = agent.handle('what is slam', parser)['reply']
    assert '*' not in r and '\n' not in r and len(r) <= 421


# ------------------------------------------------------------------ memory + planning
def test_follow_up_take_me_there_becomes_a_checked_trip(tmp_path):
    agent, parser, fake, _ = make(
        tmp_path,
        say('Dr. Meera Rao is the head. Her cabin is the HOD office.', 'knowledge', ['k1']),
        say('Sure, follow me to the HOD office.', 'robot', action='navigate', targets=['HOD office']))
    agent.handle('who is the hod?', parser, user='rashi')
    r = agent.handle('ok mujhe wahan le chalo', parser, user='rashi')
    assert r['intent'] == 'navigate' and r['targets'] == ['HOD office'] and r['source'] == 'agent'
    turns = fake.bodies[-1]['messages']
    assert [m['role'] for m in turns] == ['system', 'user', 'assistant', 'user']    # it saw the talk so far
    assert 'HOD office' in turns[2]['content']
    # another person's talk is separate
    agent.handle('what is a lidar', parser, user='yogit')
    assert [m['role'] for m in fake.bodies[-1]['messages']] == ['system', 'user']


def test_trip_to_a_place_that_is_not_on_the_map_stays_a_reply(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('Taking you to the swimming pool.', 'robot', action='navigate',
                                             targets=['Swimming pool']))
    r = agent.handle('can we see the swimming pool', parser)
    assert r['intent'] == 'chat' and r['targets'] == []
    assert any('not on the map' in f for f in r['chat']['flags'])


def test_urgent_only_when_the_person_said_so(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('On my way.', 'robot', action='navigate', targets=['Library'],
                                             priority=3))
    assert agent.handle('i would like to read something', parser)['priority'] == 1


def test_shortest_order_is_computed_not_guessed(tmp_path):
    agent, parser, _, _ = make(tmp_path, say('I will visit them in the shortest order.', 'robot',
                                             action='navigate', targets=['Canteen', 'Library', 'Lab 3'],
                                             shortest_order=True))
    r = agent.handle('what is the best order for canteen, library and lab 3? take me', parser,
                     ctx={'pose': [0.0, 0.0]})
    assert r['targets'] == ['Library', 'Lab 3', 'Canteen']


def test_second_lookup_when_the_model_asks_for_it(tmp_path):
    agent, parser, fake, k = make(
        tmp_path,
        say("I don't know that yet.", 'unknown', search='scholarship'),
        say('Forms are given at the accounts window.', 'knowledge', ['k3']))
    k.add('Scholarship forms', 'Forms are given at the accounts window.', [])
    r = agent.handle('how do students get money help for studies', parser)
    assert len(fake.bodies) == 2 and r['chat']['rounds'] == 2 and 'accounts window' in r['reply']
    assert 'Scholarship forms' in fake.bodies[1]['messages'][0]['content']


def test_preview_then_send_is_one_model_call_and_one_memory(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('A robot that maps and finds its way.'))
    a = agent.handle('what is slam', parser, user='u')
    b = agent.handle('what is slam', parser, user='u')
    assert a['reply'] == b['reply'] and len(fake.bodies) == 1 and len(agent.history('u')) == 2


# ------------------------------------------------------------------ no key / no internet
def test_without_a_key_the_notebook_is_read_out(tmp_path):
    agent, parser, fake, _ = make(tmp_path, key=None)
    r = agent.handle('library timings?', parser)
    assert '9 am to 5 pm' in r['reply'] and r['source'] == 'offline' and fake.bodies == []
    assert agent.handle('who won the match', parser)['reply'] == DONT_KNOW


def test_model_failure_falls_back_and_keeps_the_reason(tmp_path):
    agent, parser, _, _ = make(tmp_path, LLMError('rate_limit', 'slow down', 429))
    r = agent.handle('library timings?', parser)
    assert '9 am to 5 pm' in r['reply'] and r['llm']['error']['kind'] == 'rate_limit'


def test_chat_can_be_turned_off(tmp_path):
    agent, parser, _, _ = make(tmp_path, key=None)
    agent.enabled = False
    assert agent.handle('hello', parser)['intent'] == 'unknown'


# ------------------------------------------------------------------ in the robot's core
def test_core_answers_questions_and_edits_the_notebook(tmp_path):
    agent, parser, fake, k = make(tmp_path, say('It is open 9 am to 5 pm.', 'knowledge', ['k2']))
    core = NavigatorCore(agent.places, parser=parser, knowledge=k, chat=agent, data_dir=str(tmp_path))
    res = core.handle({'type': 'command', 'text': 'library timings?', 'by': 'arpit'}, (0.0, 0.0, 0.0))
    assert res['interpretation']['intent'] == 'chat' and '9 am' in res['interpretation']['reply']
    assert not core.queue.active                                    # a question never moves the robot
    assert 'IDLE' in fake.bodies[0]['messages'][0]['content']       # it was told its own state
    # dry run (the app's preview) shows no trip
    assert core.handle({'type': 'command', 'text': 'library timings?', 'by': 'arpit', 'dry_run': True},
                       (0.0, 0.0, 0.0))['plan']['steps'] == []
    # the voice button on the robot may ask questions anywhere
    r = core.handle({'type': 'command', 'text': 'hello', 'source': 'button'}, (9.0, 9.0, 0.0))
    assert r['interpretation']['intent'] == 'chat' and not r.get('refused')
    # notebook requests
    r = core.handle({'type': 'knowledge_add', 'title': 'Canteen', 'text': 'Lunch from 12:30.', 'by': 'arpit'})
    assert r['knowledge']['count'] == 3
    assert core.handle({'type': 'knowledge', 'q': 'lunch'})['hits'][0]['title'] == 'Canteen'
    assert core.handle({'type': 'knowledge_remove', 'id': 'pathik-who'})['ok'] is False


def test_only_admins_change_what_the_robot_tells_people():
    assert required_role('POST', '/api/nav', {'type': 'knowledge'}) == 'staff'
    assert required_role('POST', '/api/nav', {'type': 'knowledge_add'}) == 'admin'
    assert required_role('POST', '/api/nav', {'type': 'knowledge_remove'}) == 'admin'
    assert required_role('POST', '/api/nav', {'type': 'command'}) == 'staff'
