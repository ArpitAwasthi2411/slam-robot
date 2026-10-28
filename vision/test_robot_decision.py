"""Tests for robot_decision.py: no camera, YOLO or internet needed.   Run:  python -m pytest -q"""
from unittest import mock

import numpy as np

import robot_decision as rd

SHAPE = (480, 640, 3)   # frame.shape: height 480


def person(pid=1, bearing=0.0, height=100, approaching=False):
    return {"id": pid, "box": [300, 480 - height, 340, 480], "conf": 0.9,
            "bearing_deg": bearing, "approaching": approaching, "moving_away": False}


class Resp:
    def __init__(self, status=200, content="", headers=None):
        self.status_code, self.text, self.headers = status, content, headers or {}
        self._content = content

    def json(self):
        return {"choices": [{"message": {"content": self._content}}]}


def test_no_people_continue():
    d = rd.decide_from_people([], SHAPE)
    assert d["action"] == "CONTINUE" and d["speed_factor"] == 1.0


def test_no_frame_stops():
    assert rd.decide_from_people([], None)["action"] == "STOP"


def test_very_close_in_front_stops():
    d = rd.decide_from_people([person(height=320)], SHAPE)          # 0.67 of the frame
    assert d["action"] == "STOP" and d["speed_factor"] == 0.0 and d["people_in_path"] == 1


def test_near_in_path_slows_and_steers_away():
    d = rd.decide_from_people([person(bearing=-5, height=200)], SHAPE)   # near, slightly left
    assert d["action"] == "SLOW_DOWN" and d["steer"] == "right"


def test_near_and_approaching_stops():
    assert rd.decide_from_people([person(height=200, approaching=True)], SHAPE)["action"] == "STOP"


def test_far_off_path_continue():
    d = rd.decide_from_people([person(bearing=25, height=80)], SHAPE)
    assert d["action"] == "CONTINUE" and d["people_in_path"] == 0


def test_parse_llm_reply_with_scene():
    d = rd.parse_llm_reply('```json\n{"scene": "corridor", "obstacles": ["chair"], "action": "slow down", '
                           '"speed_factor": 0.3, "steer": "left", "reason": "chair near the path"}\n```')
    assert d["action"] == "SLOW_DOWN" and d["speed_factor"] == 0.3 and d["obstacles"] == ["chair"]
    assert d["scene"] == "corridor"
    assert rd.parse_llm_reply("no json here") is None
    assert rd.parse_llm_reply('{"action": "DANCE"}') is None


def test_groq_can_only_make_it_more_careful():
    rule = rd.decide_from_people([], SHAPE)                                   # CONTINUE
    stop = rd.parse_llm_reply('{"action": "STOP", "obstacles": ["box"], "reason": "box on the floor"}')
    m = rd.merge(rule, stop)
    assert m["action"] == "STOP" and m["obstacles"] == ["box"]
    rule_stop = rd.decide_from_people([person(height=320)], SHAPE)            # STOP
    go = rd.parse_llm_reply('{"action": "CONTINUE", "speed_factor": 1.0, "reason": "clear"}')
    m = rd.merge(rule_stop, go)
    assert m["action"] == "STOP" and m["speed_factor"] == 0.0


def test_one_groq_call_gives_the_decision():
    frame = np.zeros(SHAPE, dtype=np.uint8)
    reply = '{"scene":"empty corridor, chair ahead","obstacles":["chair"],"action":"STOP","speed_factor":0,' \
            '"steer":"left","reason":"chair blocks the path"}'
    env = {"GROQ_API_KEY": "x", "GROQ_VISION_MODEL": "m"}
    with mock.patch.dict("os.environ", env), mock.patch.object(rd.requests, "post", return_value=Resp(200, reply)) as post:
        d, err = rd.decide(frame, [])
    assert post.call_count == 1 and err is None
    assert d["action"] == "STOP" and d["source"] == "rules+llm" and d["obstacles"] == ["chair"]
    assert "ROBOT DECISION: STOP" in rd.report(1, [], SHAPE, d, err)


def test_rate_limit_falls_back_to_rules_with_a_clear_message():
    frame = np.zeros(SHAPE, dtype=np.uint8)
    env = {"GROQ_API_KEY": "x", "GROQ_VISION_MODEL": "m"}
    with mock.patch.dict("os.environ", env), \
            mock.patch.object(rd.requests, "post", return_value=Resp(429, "", {"retry-after": "12"})):
        d, err = rd.decide(frame, [])
    assert d["action"] == "CONTINUE" and d["source"] == "rules" and "429" in err and "12" in err


def test_missing_key_is_reported():
    with mock.patch.dict("os.environ", {}, clear=True):
        d, err = rd.decide(np.zeros(SHAPE, dtype=np.uint8), [])
    assert d["source"] == "rules" and "GROQ_API_KEY" in err
