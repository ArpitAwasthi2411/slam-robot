"""Person tracker (vision_rules.PersonTracker, same in web/app/vision.js): ids must survive people crossing,
short hiding and weak detections, and both languages must give exactly the same ids."""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from lidar_robot import vision_rules as vr  # noqa: E402
from test_phone_detector import run_js  # noqa: E402


def _walk(x, y=60.0, w=60.0, h=160.0, conf=0.9):
    return [x, y, x + w, y + h, conf]


def crossing_frames():
    """Two people walk towards each other at 30 px per frame (300 px/s) and pass; boxes overlap mid-way."""
    out, t = [], 0.0
    for k in range(20):
        t += 0.1
        a, b = 100.0 + 30 * k, 670.0 - 30 * k
        out.append({'t': round(t, 3), 'boxes': [_walk(a, conf=0.9), _walk(b, y=70.0, conf=0.8)]})
    return out


def test_crossing_keeps_ids():
    tr = vr.PersonTracker()
    first = None
    for f in crossing_frames():
        p = tr.update([tuple(b) for b in f['boxes']], now=f['t'])
        ids = {round(x['box'][0]): x['id'] for x in p}
        left_mover = min(p, key=lambda x: abs(x['box'][1] - 60))['id']    # the y=60 person moves right
        if first is None:
            first = left_mover
        assert left_mover == first, 'ids swapped when the two people crossed'
        assert len(set(ids.values())) == len(p)
    assert tr.next_id == 3                                       # never a third id


def test_short_hiding_keeps_id_and_fast_walker_matches():
    tr = vr.PersonTracker()
    t, ids = 0.0, set()
    for k in range(25):
        t += 0.1
        if 8 <= k <= 11:                                           # hidden 0.4 s (behind someone)
            tr.update([], now=t)
            continue
        p = tr.update([tuple(_walk(50.0 + 40 * k))], now=t)        # 400 px/s: boxes barely overlap frame to frame
        ids.update(x['id'] for x in p)
    assert ids == {1}


def test_weak_box_continues_but_never_starts_a_track():
    tr = vr.PersonTracker()
    tr.update([tuple(_walk(100.0, conf=0.8))], now=0.1)
    p = tr.update([tuple(_walk(104.0, conf=0.3))], now=0.2)        # half hidden: weak, still the same person
    assert [x['id'] for x in p] == [1]
    p = tr.update([tuple(_walk(400.0, conf=0.3))], now=0.3)        # weak box elsewhere: not a new person
    assert p == []


def test_js_and_python_identical_on_random_scenes():
    rng = random.Random(5)
    frames, t = [], 0.0
    people = [[rng.uniform(0, 500), rng.uniform(20, 120), rng.uniform(-40, 40), rng.uniform(40, 90)] for _ in range(4)]
    for k in range(60):
        t += rng.uniform(0.05, 0.2)
        boxes = []
        for p in people:
            p[0] += p[2] * 0.1
            if rng.random() < 0.85:
                boxes.append([p[0], p[1], p[0] + p[3], p[1] + 2.5 * p[3] * (1 + 0.01 * k),
                              round(rng.uniform(0.2, 0.95), 3)])
        if rng.random() < 0.1:
            boxes.append([rng.uniform(0, 600), 30.0, rng.uniform(600, 640), 200.0, 0.5])
        frames.append({'t': round(t, 4), 'boxes': boxes})
    js = run_js("const tr = new V.PersonTracker(); console.log(JSON.stringify(IN.map(f => tr.update(f.boxes, f.t))));",
                frames)
    tr = vr.PersonTracker()
    for f, j in zip(frames, js):
        p = tr.update([tuple(b) for b in f['boxes']], now=f['t'])
        assert [(x['id'], x['box'], x['approaching'], x['moving_away']) for x in j] == \
               [(x['id'], x['box'], x['approaching'], x['moving_away']) for x in p]
