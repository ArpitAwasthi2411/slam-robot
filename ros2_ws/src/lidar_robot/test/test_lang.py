"""Reply language is decided by rules from what the person said (27 Nov: it was left to the model and was
not predictable). Hindi only when the person spoke Hindi or asked for it; English otherwise."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from test_chat import make, say  # noqa: E402

from lidar_robot.lang import reply_language  # noqa: E402


def test_language_rule():
    hindi = ['lab 2 kahan hai', 'लैब 2 कहाँ है', 'hod office jao', 'tumhe kisne banaya', 'main gate kidhar hai',
             'HOD kaun hai', 'kitne baje hain', 'namaste', 'shukriya', 'where is lab 2, hindi mein batao',
             'tell me in Hindi', 'हिंदी में बताओ']
    english = ['where is lab 2', 'go to HOD office', 'who made you', 'where is the main gate', 'Hello',
               'take me to the library please', 'What is the time', 'thank you', 'Hi Pathik',
               'lab 2 kahan hai, in English', 'Go to lab 2 and come back', 'is the library open today', '']
    for t in hindi:
        assert reply_language(t) == 'hi', t
    for t in english:
        assert reply_language(t) == 'en', t


def test_small_talk_follows_the_language_of_the_question(tmp_path):
    agent, parser, fake, _ = make(tmp_path)
    assert agent.handle('hello', parser)['reply'].startswith('Hello!')
    r = agent.handle('namaste', parser)
    assert r['reply'].startswith('Namaste!') and r['speak'].startswith('नमस्ते')
    r = agent.handle('kaise ho', parser)
    assert 'theek' in r['reply'] and 'सकती' in r['speak']
    agent.voice_gender = 'male'
    assert 'सकता' in agent.handle('aap kaun ho', parser)['speak']
    assert agent.handle('kitne baje hain', parser)['reply'].startswith('Abhi ')
    assert agent.handle('what time is it', parser)['reply'].startswith('It is ')
    assert fake.bodies == []                                   # all offline


def test_model_is_told_the_language_and_english_gets_no_hindi_voice(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('The library is open from 9 am to 5 pm.', 'knowledge', ['k2'],
                                                speak='लाइब्रेरी 9 से 5 बजे तक खुली है।'))
    r = agent.handle('when is the library open?', parser)
    system = fake.bodies[0]['messages'][0]['content']
    assert 'plain English only' in system
    assert r.get('speak') is None                              # English question -> English voice


def test_hindi_question_tells_the_model_hinglish(tmp_path):
    agent, parser, fake, _ = make(tmp_path, say('Library 9 se 5 baje tak khuli hai.', 'knowledge', ['k2'],
                                                speak='लाइब्रेरी 9 से 5 बजे तक खुली है।'))
    r = agent.handle('library kab khulti hai?', parser)
    system = fake.bodies[0]['messages'][0]['content']
    assert 'Hinglish' in system and 'fixed' in system
    assert r['speak'] == 'लाइब्रेरी 9 से 5 बजे तक खुली है।'
