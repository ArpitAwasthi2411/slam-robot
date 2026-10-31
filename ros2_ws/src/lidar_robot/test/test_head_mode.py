"""Robot side of the phone "head": plain camera frames for the phone, the phone's people boxes used instead of the
Pi's YOLO (with a 1 s fallback), detection paused while the robot is idle, and the dashboard endpoints."""
import json
import math
import os
import stat
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
import fake_ros  # noqa: E402

fake_ros.install()

from lidar_robot.vision_rules import ActivityWatch, clean_phone_detections, phone_boxes  # noqa: E402
from lidar_robot.auth import required_role  # noqa: E402

W, H = 64, 36
FAKE_GST = f"""#!/usr/bin/env python3
import sys, time
frame = bytes([90]) * ({W} * {H} * 3)
for _ in range(600):
    sys.stdout.buffer.write(frame); sys.stdout.buffer.flush()
    time.sleep(0.02)
"""


def _wait(cond, secs=5.0):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_activity_watch():
    a = ActivityWatch(idle_pause_s=30)
    assert a.wanted(100.0)                          # no navigator yet: always on
    a.update('IDLE', 100.0)
    assert not a.wanted(100.5)                      # idle from the start
    a.update('DRIVING', 101.0)
    a.update('IDLE', 129.0)
    assert a.wanted(130.0)                          # stopped 29 s ago: still on
    a.update('IDLE', 131.5)
    assert not a.wanted(131.5)                      # 30 s after the last busy moment: paused
    a.poke(131.5)
    assert a.wanted(131.6)                          # camera view open / phone fetching
    a.update('IDLE', 170.0)
    assert not a.wanted(170.0)
    assert a.wanted(176.0)                          # navigator silent > 5 s: on, to be safe
    assert ActivityWatch(0).wanted(0.0)


def test_phone_boxes_scaling_and_freshness():
    m = {'boxes': [[10, 20, 30, 60, 0.9], [5, 5, 4, 9, 0.8], [1, 2, float('nan'), 4, 0.5]], 'w': 320, 'h': 180, 't_mono': 10.0}
    assert phone_boxes(m, 640, 360, 10.5) == [(20.0, 40.0, 60.0, 120.0, 0.9)]   # scaled x2, bad boxes dropped
    assert phone_boxes(m, 640, 360, 11.2) is None                               # older than 1 s
    assert phone_boxes(m, 0, 0, 10.1) == [(10.0, 20.0, 30.0, 60.0, 0.9)]        # no own frame: phone's size
    assert phone_boxes({'w': 0, 'h': 1, 'boxes': [], 't_mono': 10}, 64, 36, 10) is None
    assert phone_boxes({'nonsense': 1, 't_mono': 10}, 64, 36, 10) is None
    assert phone_boxes(m, 640, 360, 10.0, max_boxes=1) == [(20.0, 40.0, 60.0, 120.0, 0.9)]


def test_vision_node_uses_phone_boxes_then_falls_back(tmp_path, monkeypatch):
    pytest.importorskip('cv2')
    for name, body in (('gst-launch-1.0', FAKE_GST), ('gst-inspect-1.0', '#!/bin/sh\nexit 0\n')):
        p = tmp_path / name
        p.write_text(body)
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv('PATH', f'{tmp_path}:{os.environ["PATH"]}')
    fake_ros.Node.overrides = {'width': W, 'height': H, 'det_rate': 20.0, 'model_dir': str(tmp_path / 'none'),
                               'dock_marker': False}
    try:
        from lidar_robot import vision_node
        node = vision_node.VisionNode()
        try:
            assert _wait(lambda: node.camera.frame is not None), node.logs
            pub = node.pubs['vision/decision']
            last = lambda: json.loads(pub.msgs[-1].data)   # noqa: E731
            assert _wait(lambda: pub.msgs and last()['detected_by'] == 'pi')    # no phone: own path (no model here)

            # LiDAR: something 0.6 m straight ahead
            n = 360
            ranges = [4.0] * n
            ranges[0] = ranges[1] = ranges[-1] = 0.6
            node._on_scan(fake_ros.LaserScan(ranges=ranges, angle_min=0.0, angle_increment=2 * math.pi / n,
                                             range_min=0.15, range_max=8.0))

            def send():   # the phone saw a person in the middle of a 2x bigger frame
                node._on_phone_boxes(fake_ros.String(data=json.dumps(
                    {'boxes': [[W * 0.8, H * 0.2, W * 1.2, H * 1.8, 0.9]], 'w': W * 2, 'h': H * 2, 'ms': 70})))
            send()
            assert _wait(lambda: last()['detected_by'] == 'phone')
            d = last()
            assert d['action'] == 'STOP' and d['people'][0]['distance_m'] == 0.6 and d['phone_ms'] == 70
            assert d['people'][0]['box'] == [int(W * 0.4), int(H * 0.1), int(W * 0.6), int(H * 0.9)]
            # phone goes quiet -> after 1 s the Pi detects again by itself
            assert _wait(lambda: last()['detected_by'] == 'pi', 3.0)

            # robot idle for longer than idle_pause_s -> paused; busy again -> detecting
            node.activity.idle_pause_s = 0.2
            node._on_nav_status(fake_ros.String(data=json.dumps({'state': 'IDLE'})))
            assert _wait(lambda: last().get('paused') is True, 3.0)
            assert last()['action'] == 'UNKNOWN'
            node._on_nav_status(fake_ros.String(data=json.dumps({'state': 'DRIVING'})))
            assert _wait(lambda: last().get('paused') is False, 3.0)

            # plain frames for the phone only while asked for
            period, cb = node.timers[0]
            cb()
            assert not node.pubs['vision/raw/compressed'].msgs
            node._on_raw_request(fake_ros.Empty())
            cb()
            raw = node.pubs['vision/raw/compressed'].msgs[-1]
            assert raw.format == 'jpeg' and bytes(raw.data[:2]) == b'\xff\xd8'
            assert not node.pubs['vision/image/compressed'].msgs      # annotated stream still off
        finally:
            node.destroy_node()
    finally:
        fake_ros.Node.overrides = {}


def test_phone_detections_endpoint_validation_and_roles():
    assert required_role('POST', '/api/vision/phone', {}) == 'operator'
    assert required_role('GET', '/api/camera.jpg', None) == 'staff'
    assert clean_phone_detections({'boxes': [[1, 2, 3, 4, 0.5]], 'w': 640, 'h': 360, 'ms': 80}) == \
        {'boxes': [[1.0, 2.0, 3.0, 4.0, 0.5]], 'w': 640, 'h': 360, 'ms': 80}
    assert clean_phone_detections({'boxes': [], 'w': 640, 'h': 360, 'ms': 'x'})['ms'] is None
    for bad in ({'boxes': 'x', 'w': 1, 'h': 1}, {'boxes': [], 'w': 0, 'h': 1}, {'boxes': [], 'w': True, 'h': 1},
                {'boxes': [[1, 2, 3]], 'w': 9, 'h': 9}, {'boxes': [[1, 2, 3, float('inf'), 1]], 'w': 9, 'h': 9},
                {'boxes': [[0, 0, 1, 1, 1]] * 31, 'w': 9, 'h': 9}):
        with pytest.raises(ValueError):
            clean_phone_detections(bad)
    src = open(os.path.join(os.path.dirname(__file__), '..', 'lidar_robot', 'dashboard_server.py')).read()
    assert "'/api/vision/phone'" in src and "camera_jpeg', None)" in src and "fn(raw=True)" in src
