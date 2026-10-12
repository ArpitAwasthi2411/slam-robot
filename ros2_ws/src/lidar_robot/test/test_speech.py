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
