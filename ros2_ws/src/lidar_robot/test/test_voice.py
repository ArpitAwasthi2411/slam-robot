"""Voice: WAV fixtures (spoken commands made with espeak-ng, silence, a tap), speech-to-text engines and
fallback, the whole voice command through the real navigator, push-to-talk, and the robot's speaker."""
import json
import math
import os
import sys
import time
from array import array

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot import voice  # noqa: E402
from lidar_robot.auth import required_role  # noqa: E402
from lidar_robot.speech import Speaker, find_voice, scale_pcm  # noqa: E402
from lidar_robot.voice import (GroqWhisper, PushToTalk, SpeechToText, VoiceAssistant, VoskSTT, multipart,  # noqa: E402
                               prepare, read_wav, reply_text, resample, write_wav)

FIX = os.path.join(os.path.dirname(__file__), 'fixtures', 'voice')


def wav(name):
    with open(os.path.join(FIX, name), 'rb') as f:
        return f.read()


# ------------------------------------------------------------------ audio
def test_fixtures_read_and_resample():
    rate, s = read_wav(wav('go_lab3_22k.wav'))
    assert rate == 22050 and len(s) > rate
    r16 = resample(s, rate, 16000)
    assert abs(len(r16) - len(s) * 16000 / 22050) <= 1
    assert read_wav(wav('hod_come_back.wav'))[0] == 16000


def test_prepare_trims_silence_and_rejects_nothing():
    clean, facts = prepare(wav('go_lab3_22k.wav'))
    rate, s = read_wav(clean)
    assert rate == 16000 and 0.8 < facts['seconds'] < 2.0 and facts['level_db'] > -35
    with pytest.raises(ValueError, match="didn't hear"):
        prepare(wav('silence.wav'))
    with pytest.raises(ValueError, match='too short'):
        prepare(wav('tap.wav'))
    padded = write_wav(16000, [0] * 32000 + list(read_wav(clean)[1]) + [0] * 32000)
    assert prepare(padded)[1]['seconds'] < facts['seconds'] + 0.5      # leading/trailing silence cut


def test_stereo_and_8bit_wavs():
    import io
    import wave
    rate, s = read_wav(wav('hindi_lab.wav'))
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:                    # stereo
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(array('h', [v for x in s for v in (x, x)]).tobytes())
    r2, m = read_wav(buf.getvalue())
    assert r2 == rate and len(m) == len(s) and m[1000] == s[1000]
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:                    # 8-bit
        w.setnchannels(1)
        w.setsampwidth(1)
        w.setframerate(8000)
        w.writeframes(bytes(128 + int(100 * math.sin(i / 5)) for i in range(8000)))
    assert prepare(buf.getvalue())[1]['seconds'] > 0.5


def test_multipart_body():
    body, ctype = multipart({'model': 'whisper-large-v3-turbo'}, {'file': ('speech.wav', b'RIFFxxxx', 'audio/wav')})
    b = ctype.split('boundary=')[1]
    assert body.count(f'--{b}'.encode()) == 3 and b'name="model"\r\n\r\nwhisper-large-v3-turbo' in body
    assert b'filename="speech.wav"' in body and b'RIFFxxxx' in body and body.endswith(f'--{b}--\r\n'.encode())


# ------------------------------------------------------------------ speech to text
def test_groq_whisper_request():
    calls = []

    def post(url, body, headers, timeout):
        calls.append((url, body, headers))
        return {'text': ' Go to lab three. ', 'language': 'english'}
    clean, _ = prepare(wav('go_lab3_22k.wav'))
    r = GroqWhisper('k', http_post=post).transcribe(clean)
    assert r['text'] == 'Go to lab three.'
    url, body, headers = calls[0]
    assert url.endswith('/audio/transcriptions') and headers['Authorization'] == 'Bearer k'
    assert b'whisper-large-v3-turbo' in body and clean[:44] in body and 'multipart/form-data' in headers['Content-Type']
    with pytest.raises(RuntimeError):
        GroqWhisper(None).transcribe(clean)


def test_vosk_picks_the_more_confident_language(tmp_path):
    def loader(lang, path):
        return (lambda w: ('go to lab three', 0.91)) if lang == 'en' else (lambda w: ('गो टू लैब', 0.4))
    v = VoskSTT({'en': str(tmp_path / 'en'), 'hi': str(tmp_path / 'hi')}, loader=loader)
    assert v.available()
    r = v.transcribe(prepare(wav('go_lab3_22k.wav'))[0])
    assert r == {'text': 'go to lab three', 'language': 'en', 'confidence': 0.91}
    none = VoskSTT({'en': str(tmp_path / 'missing')})
    assert not none.available() and 'setup_voice' in none.error


def test_online_failure_falls_back_to_offline_and_none_says_what_to_do(tmp_path):
    def boom(*a):
        raise OSError('no internet')
    off = VoskSTT({'en': str(tmp_path)}, loader=lambda lang, p: (lambda w: ('come back', 0.8)))
    stt = SpeechToText(GroqWhisper('k', http_post=boom), off)
    r = stt.transcribe(prepare(wav('hod_come_back.wav'))[0])
    assert r['text'] == 'come back' and r['engine'] == 'vosk' and 'groq failed' in r['note']
    with pytest.raises(RuntimeError, match='GROQ_API_KEY'):
        SpeechToText(GroqWhisper(None), VoskSTT({'en': str(tmp_path / 'x')})).transcribe(b'')
    assert SpeechToText(GroqWhisper('k'), None).describe() == 'groq'


# ------------------------------------------------------------------ the whole voice command
class FakeSTT:
    """Returns what the fixture says (the real engines are tested above)."""
    def __init__(self, texts):
        self.texts = texts

    def transcribe(self, w):
        secs = len(read_wav(w)[1]) / 16000.0
        return {'text': self.texts(secs), 'engine': 'fake', 'language': 'en', 'ms': 5}


def test_voice_command_goes_to_the_navigator_and_the_reply_is_spoken(tmp_path):
    from test_navigator_core import make_setup
    nav, clk = make_setup(tmp_path)
    said = []
    send = lambda text: nav.handle({'type': 'command', 'text': text, 'by': 'voice'}, (1.0, 1.0, 0.0))  # noqa: E731
    va = VoiceAssistant(FakeSTT(lambda s: 'go to lab three'), send, said.append)
    r = va.handle(wav('go_lab3_22k.wav'))
    assert r['ok'] and r['heard'] == 'go to lab three' and r['intent'] == 'navigate'
    assert (nav.queue.active or nav.queue.queue[0]).stops[0].label == 'Lab 3'
    assert said and 'Lab 3' in said[-1]
    # silence: nothing is sent, the robot says why
    sent = []
    va2 = VoiceAssistant(FakeSTT(lambda s: 'x'), sent.append, said.append)
    r = va2.handle(wav('silence.wav'))
    assert not r['ok'] and not sent and "didn't hear" in said[-1]
    # nonsense: not understood
    r = VoiceAssistant(FakeSTT(lambda s: 'banana helicopter'), send, said.append).handle(wav('hod_come_back.wav'))
    assert not r['ok'] and r['intent'] == 'unknown'
    # no speech recognition at all
    r = VoiceAssistant(SpeechToText(None, None), send, said.append).handle(wav('go_lab3_22k.wav'))
    assert not r['ok'] and 'no speech recognition' in said[-1]


def test_reply_text():
    assert reply_text(None).startswith('Sorry')
    assert 'no place' in reply_text({'ok': False, 'error': 'no place matches "x"'})
    assert reply_text({'ok': True, 'answer': 'The printer is in Lab 2.'}) == 'The printer is in Lab 2.'
    assert reply_text({'ok': True, 'interpretation': {'intent': 'navigate', 'reply': 'Going to Lab 3.'}}) == 'Going to Lab 3.'


# ------------------------------------------------------------------ push-to-talk
def test_push_to_talk_debounce_max_length_and_app_button():
    ev = []
    p = PushToTalk(lambda: ev.append('start'), lambda why, s: ev.append(('stop', why, round(s, 2))), debounce=3, max_s=2.0)
    t = 0.0
    for v in [1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1]:             # contact bounce, then a real press
        p.update(bool(v), t)
        t += 0.02
    assert ev == ['start'] and p.recording
    for _ in range(10):
        p.update(False, t)
        t += 0.02
    assert ev[1][0:2] == ('stop', 'released') and not p.recording
    t += 1.0
    for _ in range(150):                                       # held for 3 s: stops itself at max_s
        p.update(True, t)
        t += 0.02
    assert ev[-1][1] == 'max_length' and abs(ev[-1][2] - 2.0) < 0.05
    ev.clear()
    p2 = PushToTalk(lambda: ev.append('start'), lambda why, s: ev.append(('stop', why)), max_s=8.0)
    assert p2.press(10.0, max_s=5.0) and not p2.press(10.1)   # app button; a second press is ignored
    p2.update(False, 15.1)                                     # app limit 5 s
    assert ev == ['start', ('stop', 'max_length')]


def test_gpio_button_without_libgpiod_is_off_not_an_error(monkeypatch):
    import builtins
    real = builtins.__import__
    monkeypatch.setattr(builtins, '__import__', lambda n, *a, **k: (_ for _ in ()).throw(ImportError(n)) if n == 'gpiod' else real(n, *a, **k))
    b = voice.GpioButton(17)
    assert b.read() is None and 'libgpiod' in b.error


# ------------------------------------------------------------------ speaker (TTS)
def test_speaker_queue_never_overlaps_and_rate_limits():
    played, active = [], []

    def runner(text):
        active.append(1)
        assert len(active) == 1, 'two phrases at once'
        time.sleep(0.01)
        played.append(text)
        active.pop()
    t = [0.0]
    sp = Speaker(runner=runner, clock=lambda: t[0], max_per_min=3, min_gap_s=0.0)
    for i in range(6):
        sp.say(f'phrase {i}', key=f'k{i}')
    assert sp.say('I need help. Please check the app.', key='recovery_ask')     # priority: always
    sp.wait()
    assert len(played) <= 4 and played[-1].startswith('I need help')
    t[0] = 61.0
    assert sp.say('Going to Lab 3.', key='going')


def test_speaker_volume_and_voice_choice(tmp_path):
    raw = array('h', [1000, -1000, 30000, -30000]).tobytes()
    out = array('h')
    out.frombytes(scale_pcm(raw, 1.5))
    assert list(out) == [1500, -1500, 32767, -32768]
    assert scale_pcm(raw, 1.0) == raw
    v = tmp_path / 'hi_IN-pratham-medium.onnx'
    v.write_bytes(b'x')
    assert find_voice('hi', str(v)) == str(v) and find_voice('xx', '') is None
    sp = Speaker(runner=lambda t: None, volume=9)
    assert sp.volume == 1.5


def test_speaker_engine_choice_without_piper(monkeypatch):
    import lidar_robot.speech as speech
    monkeypatch.setattr(speech, 'find_piper', lambda: None)
    monkeypatch.setattr(speech.shutil, 'which', lambda n: None)
    sp = Speaker(engine='auto')
    assert sp.engine == 'none' and not sp.available and sp.say('hello') is True     # silently not played


def test_navigator_announces_going_once_per_stop(tmp_path):
    from test_navigator_core import make_setup, run
    nav, clk = make_setup(tmp_path)
    nav.handle({'type': 'command', 'text': 'go to lab 3 then hod office'}, (1.0, 1.0, 0.0))
    run(nav, clk, (1.0, 1.0, 0.0), 200)
    going = [t for k, t in nav.say_queue if k == 'going']
    assert going == ['Going to Lab 3.', 'Going to HOD office.']


# ------------------------------------------------------------------ API
def test_voice_api_is_for_any_logged_in_user_and_the_simulator_pretends(tmp_path):
    assert required_role('POST', '/api/voice', {'action': 'listen'}) == 'staff'
    tools = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'tools')
    if not os.path.exists(os.path.join(tools, 'dashboard_sim.py')):
        pytest.skip('tools/ not in this checkout')
    sys.path.insert(0, tools)
    import dashboard_sim as sim
    b = sim.SimBackend(start_thread=False, data_dir=str(tmp_path))
    r = b.voice_control({'action': 'stop', 'text': 'go to lab 3'})
    assert r['ok'] and 'Lab 3' in r['reply']
    st = b.get_state()['voice']
    assert st['last_heard'] == 'go to lab 3' and st['mic'] is False
    json.dumps(st)
