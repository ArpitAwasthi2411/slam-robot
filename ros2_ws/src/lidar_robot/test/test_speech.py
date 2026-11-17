import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.speech import Speaker  # noqa: E402


def test_speaker_rate_limits_and_survives_missing_espeak():
    said, t = [], [0.0]
    sp = Speaker(runner=said.append, clock=lambda: t[0], min_gap_s=8.0)
    assert sp.say('Excuse me, please make way.', key='make_way')
    assert not sp.say('Excuse me, please make way.', key='make_way')
    t[0] = 9.0
    assert sp.say('Excuse me, please make way.', key='make_way')
    off = Speaker(enabled=False, runner=said.append)
    assert not off.say('hello')
    nothing = Speaker(runner=None)
    nothing._runner = None
    assert nothing.say('no speaker installed') is True        # logged, silently not played


# ------------------------------------------------------------------ slower, streamed Piper
def test_piper_is_slowed_down_and_pauses_between_sentences():
    from lidar_robot.speech import piper_args
    a = piper_args('/p/piper', '/v/hi.onnx', 1.15, 0.3)
    assert a[a.index('--length_scale') + 1] == '1.15' and a[a.index('--sentence_silence') + 1] == '0.30'
    assert '--output_raw' in a


def test_audio_is_streamed_in_whole_samples_with_volume():
    import array
    import io
    from lidar_robot.speech import stream_pcm
    samples = array.array('h', [1000, -1000, 30000, -30000, 7]).tobytes()

    class Odd(io.BytesIO):                       # a pipe that hands out odd-sized pieces
        def read1(self, n=-1):
            return super().read(3)
    out = io.BytesIO()
    n = stream_pcm(Odd(samples), out, 2.0)
    got = array.array('h')
    got.frombytes(out.getvalue())
    assert n == len(samples) and list(got) == [2000, -2000, 32767, -32768, 14]


def test_speaker_rate_is_clamped():
    from lidar_robot.speech import Speaker
    assert Speaker(runner=lambda t: None, rate=5).rate == 2.0
    assert Speaker(runner=lambda t: None, rate=0.1).rate == 0.6
