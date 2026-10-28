"""vision_node end to end with a fake libcamera (a gst-launch-1.0 script that streams raw frames),
a fake detector and the fake ROS layer. Needs OpenCV (skipped where it isn't installed, e.g. CI).
The skip happens inside the test, never at import: in the ROS image a module-level skip escapes through
the launch_testing pytest plugin and stops the whole collection."""
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

W, H = 64, 36
FAKE_GST = f"""#!/usr/bin/env python3
import sys, time
frame = bytes([90]) * ({W} * {H} * 3)
for _ in range(400):
    sys.stdout.buffer.write(frame[:100]); sys.stdout.buffer.flush()   # partial writes on purpose
    sys.stdout.buffer.write(frame[100:]); sys.stdout.buffer.flush()
    time.sleep(0.02)
"""


def _wait(cond, secs=5.0):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


class FakeDetector:
    def detect(self, frame, conf):
        h, w = frame.shape[:2]
        return [(w * 0.4, h * 0.1, w * 0.6, h * 0.9, 0.9)]        # one person straight ahead


def test_vision_node_with_fake_camera(tmp_path, monkeypatch):
    cv2 = pytest.importorskip('cv2')
    import numpy as np
    for name, body in (('gst-launch-1.0', FAKE_GST), ('gst-inspect-1.0', '#!/bin/sh\nexit 0\n')):
        p = tmp_path / name
        p.write_text(body)
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv('PATH', f'{tmp_path}:{os.environ["PATH"]}')
    fake_ros.Node.overrides = {'width': W, 'height': H, 'det_rate': 20.0, 'model_dir': str(tmp_path / 'none')}
    try:
        from lidar_robot import vision_node
        node = vision_node.VisionNode()
        try:
            assert _wait(lambda: node.camera.frame is not None), node.logs
            assert node.camera.kind == 'libcamera' and node.camera.frame.shape == (H, W, 3)
            pub = node.pubs['vision/decision']
            # no ncnn/model here: camera works, detection reports why it's off
            assert _wait(lambda: pub.msgs and json.loads(pub.msgs[-1].data)['camera_ok'])
            d = json.loads(pub.msgs[-1].data)
            assert d['action'] == 'UNKNOWN' and 'detection off' in d['reason'] and d['detector'] is False

            # a person straight ahead, the LiDAR says 0.6 m -> STOP
            node.detector = FakeDetector()
            n = 360
            ranges = [4.0] * n
            ranges[0] = ranges[1] = ranges[-1] = 0.6
            node._on_scan(fake_ros.LaserScan(ranges=ranges, angle_min=0.0, angle_increment=2 * math.pi / n,
                                             range_min=0.15, range_max=8.0))
            assert _wait(lambda: json.loads(pub.msgs[-1].data)['action'] == 'STOP')
            d = json.loads(pub.msgs[-1].data)
            assert d['people'][0]['distance_m'] == 0.6 and d['lidar'] is True and '0.6 m' in d['reason']

            # annotated JPEG for the app
            period, cb = node.timers[0]
            cb()
            img = node.pubs['vision/image/compressed'].msgs[-1]
            assert img.format == 'jpeg' and bytes(img.data[:2]) == b'\xff\xd8'
            assert cv2.imdecode(np.frombuffer(img.data, np.uint8), cv2.IMREAD_COLOR).shape == (H, W, 3)
        finally:
            node.destroy_node()
    finally:
        fake_ros.Node.overrides = {}
