import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.vision_rules import (PersonTracker, VisionGate, bearing_deg, decide,  # noqa: E402
                                      decode_yolo, letterbox_params, lidar_distance)

H = 360


def person(pid=1, bearing=0.0, height=100, dist=None, approaching=False):
    return {'id': pid, 'box': [300, H - height, 340, H], 'bearing_deg': bearing,
            'distance_m': dist, 'approaching': approaching}


def test_no_people_continue_and_no_frame_stop():
    assert decide([], H)['action'] == 'CONTINUE'
    assert decide([], 0)['action'] == 'STOP'


def test_lidar_distance_rules():
    assert decide([person(dist=0.6)], H)['action'] == 'STOP'
    assert decide([person(dist=1.4)], H)['action'] == 'SLOW_DOWN'
    assert decide([person(dist=1.4, approaching=True)], H)['action'] == 'STOP'
    d = decide([person(dist=3.0)], H)
    assert d['action'] == 'CONTINUE' and d['speed_factor'] == 0.8
    assert decide([person(bearing=30, dist=0.5)], H)['action'] == 'SLOW_DOWN'     # beside, very close
    assert decide([person(bearing=30, dist=1.5)], H)['action'] == 'CONTINUE'


def test_box_size_fallback_without_lidar():
    assert decide([person(height=250)], H)['action'] == 'STOP'                    # 0.69 of the frame
    d = decide([person(bearing=-5, height=150)], H)                                # near, slightly left
    assert d['action'] == 'SLOW_DOWN' and d['steer'] == 'right'


def test_closest_person_in_path_wins():
    d = decide([person(1, bearing=2, dist=3.0), person(2, bearing=-4, dist=0.7)], H)
    assert d['action'] == 'STOP' and '0.7 m' in d['reason'] and d['people_in_path'] == 2


def test_bearing_and_lidar_lookup():
    assert bearing_deg([0, 0, 64, 10], 640, 102) < -40                              # far left
    assert abs(bearing_deg([310, 0, 330, 10], 640, 102)) < 0.01
    n = 360
    ranges = [5.0] * n
    inc = 2 * math.pi / n
    ranges[30] = 1.2                     # scan angle +30 deg = to the LEFT of the robot
    assert lidar_distance(-30.0, ranges, 0.0, inc) == 1.2      # camera bearing -30 = left
    assert lidar_distance(30.0, ranges, 0.0, inc) == 5.0
    ranges[30] = float('inf')
    assert lidar_distance(-30.0, ranges, 0.0, inc) == 5.0
    assert lidar_distance(0.0, [], 0.0, inc) is None


def test_decode_yolo_letterbox_and_nms():
    size, fw, fh = 320, 640, 360
    s, px, py, _, _ = letterbox_params(fw, fh, size)
    out = np.zeros((84, 3), dtype=np.float32)
    # person at frame box (100, 50)-(200, 350) -> letterbox coords
    cx, cy, w, h = 150 * s + px, 200 * s + py, 100 * s, 300 * s
    out[:4, 0] = [cx, cy, w, h]
    out[4, 0] = 0.9
    out[:4, 1] = [cx + 1, cy, w, h]      # duplicate: removed by NMS
    out[4, 1] = 0.8
    out[:4, 2] = [cx, cy, w, h]
    out[5, 2] = 0.95                     # a bicycle, not a person
    boxes = decode_yolo(out[None], fw, fh, size)
    assert len(boxes) == 1
    x1, y1, x2, y2, c = boxes[0]
    assert abs(x1 - 100) < 1 and abs(y2 - 350) < 1 and c == np.float32(0.9)
    assert decode_yolo(out.T, fw, fh, size)[0][:4] == boxes[0][:4]                  # N x 84 layout too


def test_tracker_ids_and_approaching():
    t = PersonTracker()
    a = t.update([(100, 100, 150, 200, 0.9)], now=0.0)
    b = t.update([(101, 90, 151, 200, 0.9)], now=0.3)
    c = t.update([(102, 60, 152, 200, 0.9)], now=0.7)            # 100 -> 140 px tall: approaching
    assert a[0]['id'] == b[0]['id'] == c[0]['id'] and c[0]['approaching']
    d = t.update([(400, 100, 450, 200, 0.9)], now=0.8)            # somebody else
    assert d[0]['id'] != a[0]['id']
    t.update([], now=5.0)
    assert t.tracks == {}


def test_vision_gate():
    g = VisionGate(timeout_s=1.0)
    g.update({'action': 'STOP', 'speed_factor': 0.0, 'reason': 'person 0.6 m ahead'}, now=10.0)
    assert g.apply(0.15, 0.2, now=10.5) == (0.0, 0.0) and 'person' in g.note
    assert g.apply(-0.1, 0.0, now=10.5) == (-0.1, 0.0)            # reversing is not limited
    assert g.apply(0.15, 0.2, now=12.0) == (0.15, 0.2)            # stale: no effect
    g.update({'action': 'SLOW_DOWN', 'speed_factor': 0.4, 'reason': 'slow'}, now=20.0)
    v, w = g.apply(0.15, 0.2, now=20.1)
    assert abs(v - 0.06) < 1e-9 and w == 0.2
    g.update({'action': 'UNKNOWN'}, now=21.0)                     # detector off -> ignored
    assert g.apply(0.15, 0.0, now=21.1) == (0.15, 0.0)
    off = VisionGate(enabled=False)
    off.update({'action': 'STOP', 'speed_factor': 0.0, 'reason': 'x'}, now=0.0)
    assert off.apply(0.15, 0.0, now=0.1) == (0.15, 0.0)
