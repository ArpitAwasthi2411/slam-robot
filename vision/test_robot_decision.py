"""Tests for robot_decision.py: no camera, YOLO or internet needed.   Run:  python -m pytest -q"""
from unittest import mock

import robot_decision as rd

SHAPE = (480, 640, 3)   # frame.shape: height 480


def person(pid=1, bearing=0.0, height=100, approaching=False):
    return {"id": pid, "box": [300, 480 - height, 340, 480], "conf": 0.9,
            "bearing_deg": bearing, "approaching": approaching, "moving_away": False}


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


def test_very_close_at_side_slows():
    assert rd.decide_from_people([person(bearing=25, height=320)], SHAPE)["action"] == "SLOW_DOWN"


def test_parse_llm_reply():
    d = rd.parse_llm_reply('Sure: {"action": "slow down", "speed_factor": 0.3, "steer": "left", "reason": "chair"}')
    assert d["action"] == "SLOW_DOWN" and d["speed_factor"] == 0.3 and d["steer"] == "left"
    assert rd.parse_llm_reply("no json here") is None
    assert rd.parse_llm_reply('{"action": "DANCE"}') is None


def test_llm_can_only_make_it_more_careful():
    rule = rd.decide_from_people([], SHAPE)                                   # CONTINUE
    stop = rd.parse_llm_reply('{"action": "STOP", "reason": "box on the floor"}')
    assert rd.merge(rule, stop)["action"] == "STOP"
    rule_stop = rd.decide_from_people([person(height=320)], SHAPE)            # STOP
    go = rd.parse_llm_reply('{"action": "CONTINUE", "speed_factor": 1.0, "reason": "clear"}')
    m = rd.merge(rule_stop, go)
    assert m["action"] == "STOP" and m["speed_factor"] == 0.0


def test_llm_failure_falls_back_to_rules():
    rule = rd.decide_from_people([person(height=200)], SHAPE)
    assert rd.merge(rule, None) == rule
    assert rd.decide_with_llm("Vision API error: 401", [], rule) is None


def test_decide_with_mocked_groq():
    class Resp:
        def raise_for_status(self): pass
        def json(self): return {"choices": [{"message": {"content":
                                '{"action":"STOP","speed_factor":0,"steer":"none","reason":"stairs ahead"}'}}]}
    frame = mock.Mock(size=1, shape=SHAPE)
    with mock.patch.dict("os.environ", {"GROQ_API_KEY": "x"}), mock.patch.object(rd.requests, "post", return_value=Resp()):
        d = rd.decide(frame, [], "A corridor with stairs going down ahead.")
    assert d["action"] == "STOP" and d["source"] == "rules+llm" and "stairs" in d["reason"]
