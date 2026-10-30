"""Compute savings on the Pi 4 (docs/compute.md): the numpy scan code gives the same answers as the old loops,
Pi health readings, the launch CPU plan, the camera pipeline and its ISP fallback, marker search on demand."""
import json
import math
import os
import random
import stat
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
import fake_ros  # noqa: E402

fake_ros.install()

from lidar_robot import scan_points  # noqa: E402
from lidar_robot.goal_logic import front_clearance_from_scan  # noqa: E402
from lidar_robot.pihealth import PiHealth, cpu_times, decode_throttled, level_of, parse_throttled  # noqa: E402


# ------------------------------------------------------------------ the old per-ray loops (reference)
def old_navigator(ranges, amin, inc, rmin, laser, pose):
    lx, ly, lyaw = laser
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    pts, near, a = [], [], amin
    for i, r in enumerate(ranges):
        if rmin < r < 3.0 and math.isfinite(r):
            bx = lx + r * math.cos(a + lyaw)
            by = ly + r * math.sin(a + lyaw)
            if bx * bx + by * by < 1.0:
                near.append((bx, by))
            if i % 2 == 0:
                pts.append((x + c * bx - s * by, y + s * bx + c * by))
        a += inc
    return pts, near


def old_dashboard(ranges, amin, inc, rmin, rmax, laser, dec):
    lx, ly, lyaw = laser
    pts, a = [], amin
    for i, r in enumerate(ranges):
        if i % dec == 0 and rmin < r < rmax and math.isfinite(r):
            ab = a + lyaw
            pts.append((round(lx + r * math.cos(ab), 3), round(ly + r * math.sin(ab), 3)))
        a += inc
    return pts


def _scan(n, seed):
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        u = rng.random()
        out.append(float('inf') if u < 0.05 else float('nan') if u < 0.08 else 0.05 if u < 0.1 else
                   rng.uniform(0.16, 9.0))
    return out


def _close(a, b, tol=1e-6):
    assert len(a) == len(b)
    for p, q in zip(a, b):
        assert abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol, (p, q)


@pytest.mark.parametrize('n,laser,seed', [(1440, (-0.13, 0.0, 0.0), 1), (720, (-0.13, 0.02, math.pi), 2),
                                          (1081, (0.05, -0.01, 0.4), 3)])
def test_numpy_scan_matches_the_old_loops(n, laser, seed):
    ranges = _scan(n, seed)
    amin, inc = -math.pi, 2 * math.pi / n
    geo = scan_points.ScanGeometry()
    for hw in (None, 0.25):
        old = front_clearance_from_scan(ranges, amin, inc, 0.15, 8.0, laser_yaw=laser[2],
                                        half_angle=math.radians(25), robot_half_width=hw, laser_x=laser[0])
        new = scan_points.front_clearance(geo, ranges, amin, inc, 0.15, 8.0, laser=laser,
                                          half_angle=math.radians(25), robot_half_width=hw)
        assert (old == new == float('inf')) or abs(old - new) < 1e-6
    pose = (1.5, -2.0, 0.7)
    w_old, n_old = old_navigator(ranges, amin, inc, 0.15, laser, pose)
    w_new, n_new = scan_points.obstacle_points(geo, ranges, amin, inc, 0.15, pose, laser=laser)
    _close(w_old, w_new)
    _close(n_old, n_new)
    for dec in (1, 3):
        _close(old_dashboard(ranges, amin, inc, 0.15, 8.0, laser, dec),
               scan_points.app_points(geo, ranges, amin, inc, 0.15, 8.0, laser=laser, decimation=dec), 1.01e-3)


def test_scan_edge_cases_and_geometry_cache():
    geo = scan_points.ScanGeometry()
    assert scan_points.front_clearance(geo, [], 0, 0.01, 0.1, 8) == float('inf')
    assert scan_points.obstacle_points(geo, [], 0, 0.01, 0.1, (0, 0, 0)) == ([], [])
    assert scan_points.app_points(geo, [], 0, 0.01, 0.1, 8) == []
    world, near = scan_points.obstacle_points(geo, [0.5] * 8, 0, math.pi / 4, 0.1, None)
    assert world == [] and len(near) == 8               # no pose: body points only
    scan_points.front_clearance(geo, [1.0] * 360, -math.pi, math.pi / 180, 0.1, 8)
    first = geo.cos
    scan_points.front_clearance(geo, [2.0] * 360, -math.pi, math.pi / 180, 0.1, 8)
    assert geo.cos is first                              # same angles: no recompute
    scan_points.front_clearance(geo, [2.0] * 720, -math.pi, math.pi / 360, 0.1, 8)
    assert geo.cos is not first and len(geo.cos) == 720
    # a ROS array.array works like a list
    from array import array
    ranges = array('f', [1.0] * 720)
    assert scan_points.front_clearance(geo, ranges, -math.pi, math.pi / 360, 0.1, 8) == pytest.approx(
        front_clearance_from_scan(ranges, -math.pi, math.pi / 360, 0.1, 8), abs=1e-6)


# ------------------------------------------------------------------ Pi health
def test_throttle_flags_and_levels():
    assert parse_throttled('throttled=0x50005') == 0x50005
    assert parse_throttled('0x0') == 0 and parse_throttled('junk') is None and parse_throttled(None) is None
    f = decode_throttled(0x50005)
    assert f[0] == 'under-voltage now' and 'under-voltage happened since boot' in f and len(f) == 4
    assert decode_throttled(0) == [] and decode_throttled(None) == []
    assert level_of(55, 0x50005, 30)[0] == 'bad' and 'Under-voltage' in level_of(55, 0x1, 30)[1]
    assert level_of(82, 0, 30)[0] == 'bad'
    assert level_of(72, 0, 30)[0] == 'warn'
    assert level_of(55, 0x50000, 30)[0] == 'warn'          # happened earlier, not now
    assert level_of(55, 0, 95)[0] == 'warn'
    assert level_of(55, 0, 30) == ('ok', 'OK')
    assert level_of(None, None, None) == ('ok', 'OK')


def test_pi_health_reads_a_fake_pi(tmp_path):
    root = tmp_path
    (root / 'sys/class/thermal/thermal_zone0').mkdir(parents=True)
    (root / 'sys/class/thermal/thermal_zone0/temp').write_text('81234\n')
    fw = root / 'sys/devices/platform/soc/soc:firmware'
    fw.mkdir(parents=True)
    (fw / 'get_throttled').write_text('50005\n')
    (root / 'proc').mkdir()
    (root / 'proc/loadavg').write_text('3.52 2.10 1.00 2/300 1234\n')
    stat_f = root / 'proc/stat'
    stat_f.write_text('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 1 2 3\n')
    h = PiHealth(start_thread=False, root=str(root))
    first = h.measure()
    assert first['cpu_pct'] is None                       # needs two samples
    stat_f.write_text('cpu  250 0 150 900 0 0 0 0 0 0\n')  # +200 busy, +100 idle
    m = h.measure()
    assert m['temp_c'] == 81.2 and m['load1'] == 3.52 and m['cpu_pct'] == pytest.approx(66.7, abs=0.1)
    assert m['throttled'] == '0x50005' and m['level'] == 'bad' and 'Under-voltage' in m['text']
    assert h.sample() == m
    assert cpu_times('cpu  1 2 3 4 5 6 7 8') == (36 - 9, 36) and cpu_times('intr 1') is None
    empty = PiHealth(start_thread=False, root=str(tmp_path / 'nothing'))
    e = empty.measure()
    assert e['temp_c'] is None and e['throttled'] is None and e['level'] == 'ok'


# ------------------------------------------------------------------ launch CPU plan
def test_cpu_plan_prefixes(monkeypatch):
    from lidar_robot import cpu_plan
    have = lambda n: '/usr/bin/' + n   # noqa: E731
    monkeypatch.setattr(cpu_plan.os, 'cpu_count', lambda: 4)
    assert cpu_plan.cpu_prefix('vision', True, have) == 'taskset -c 3 nice -n 10'
    assert cpu_plan.cpu_prefix('dashboard', True, have) == 'nice -n 5'
    assert cpu_plan.cpu_prefix('esp32_bridge', True, have) is None       # motion-critical: untouched
    assert cpu_plan.cpu_prefix('vision', False, have) is None
    assert cpu_plan.cpu_prefix('vision', True, lambda n: None) is None   # tools missing: just run it
    monkeypatch.setattr(cpu_plan.os, 'cpu_count', lambda: 2)
    assert cpu_plan.cpu_prefix('vision', True, have) == 'nice -n 10'     # no core 3 on this machine
    launch = open(os.path.join(os.path.dirname(__file__), '..', 'launch', 'bringup.launch.py')).read()
    for n in ('vision', 'dashboard', 'voice', 'cartographer_occupancy_grid_node'):
        assert f"prefix=pre('{n}')" in launch
    assert "'-include_unfrozen_submaps', 'false'" in launch       # localization: the saved map only


# ------------------------------------------------------------------ camera pipeline
def test_camera_pipeline_strings(monkeypatch):
    from lidar_robot import vision_node
    monkeypatch.setattr(vision_node.threading.Thread, 'start', lambda self: None)   # no capture thread
    cam = vision_node.Camera('libcamera', 640, 360, 15, print, out_fps=5)
    isp, conv = cam.pipeline(True), cam.pipeline(False)
    assert 'video/x-raw,width=640,height=360,framerate=15/1,format=BGR' in isp and 'videoconvert' not in isp
    assert isp.index('videorate') < isp.index('fdsink') and 'video/x-raw,framerate=5/1' in isp
    assert 'videoconvert' in conv and conv.index('videorate') < conv.index('videoconvert')
    assert cam.isp_ok is None and cam.out_fps == 5
    assert vision_node.Camera('libcamera', 650, 360, 15, print, out_fps=30).isp_ok is False   # padded rows
    assert vision_node.Camera('libcamera', 640, 360, 15, print, out_fps=30).out_fps == 15     # never above fps


W, H = 64, 36     # 64 x 3 = 192 bytes per row: a multiple of 64, so the ISP path is tried first
REFUSE_ISP_GST = f"""#!/usr/bin/env python3
import sys, time
if any('width=' in a and 'format=BGR' in a for a in sys.argv):
    sys.exit(1)                                     # like a libcamera that can't give BGR
frame = bytes([90]) * ({W} * {H} * 3)
for _ in range(300):
    sys.stdout.buffer.write(frame); sys.stdout.buffer.flush()
    time.sleep(0.02)
"""


def _wait(cond, secs=6.0):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_camera_falls_back_to_videoconvert_when_the_isp_refuses_bgr(tmp_path, monkeypatch):
    pytest.importorskip('numpy')
    for name, body in (('gst-launch-1.0', REFUSE_ISP_GST), ('gst-inspect-1.0', '#!/bin/sh\nexit 0\n')):
        p = tmp_path / name
        p.write_text(body)
        p.chmod(p.stat().st_mode | stat.S_IEXEC)
    monkeypatch.setenv('PATH', f'{tmp_path}:{os.environ["PATH"]}')
    from lidar_robot import vision_node
    logs = []
    cam = vision_node.Camera('libcamera', W, H, 15, logs.append, out_fps=5)
    try:
        assert _wait(lambda: cam.frame is not None), logs
        assert cam.mode == 'convert' and cam.isp_ok is False and cam.frame.shape == (H, W, 3)
        assert any('ISP BGR output refused' in m for m in logs)
    finally:
        cam.running = False
        cam.close()


def test_marker_search_only_when_asked(monkeypatch):
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV without aruco')
    from lidar_robot import vision_node
    fake_ros.Node.overrides = {'camera': '/dev/null-camera', 'model_dir': '/nonexistent'}
    try:
        node = vision_node.VisionNode()
        try:
            if node.marker is None or node.marker.error:
                pytest.skip('marker detector unavailable here')
            assert not node.marker_wanted()
            node._on_marker_request(fake_ros.Empty())
            assert node.marker_wanted()
            assert not node.marker_wanted(time.monotonic() + 3.5)       # stops 3 s after the last request
            node.marker_always = True
            assert node.marker_wanted(time.monotonic() + 100)
        finally:
            node.destroy_node()
    finally:
        fake_ros.Node.overrides = {}


def test_dashboard_map_bytes_match_the_old_loop():
    np = pytest.importorskip('numpy')
    data = [-1, 0, 1, 50, 99, 100, -1, 7]
    old = bytes((v + 1) if v >= 0 else 0 for v in data)
    d = np.asarray(data, dtype=np.int16)
    assert np.where(d >= 0, d + 1, 0).astype(np.uint8).tobytes() == old
    from array import array
    d = np.asarray(array('b', data), dtype=np.int16)                    # what rclpy hands over
    assert np.where(d >= 0, d + 1, 0).astype(np.uint8).tobytes() == old
    assert json.dumps({'ok': True})


# ------------------------------------------------------------------ accurate mapping (docs/mapping.md)
CFG = os.path.join(os.path.dirname(__file__), '..', 'config')


def _lua(name):
    with open(os.path.join(CFG, name)) as f:
        return f.read()


def test_mapping_configs_trust_good_odometry_in_corridors():
    live = _lua('cartographer_odom.lua')
    assert 'use_online_correlative_scan_matching = false' in live
    assert 'ceres_scan_matcher.translation_weight = 20.' in live
    assert 'odometry_rotation_weight = 1e5' in live and 'odometry_rotation_weight = 1e3' not in live
    assert 'constraint_builder.min_score = 0.65' in live and live.rstrip().endswith('return options')
    hq = _lua('cartographer_mapping_hq.lua')
    assert hq.index('include "cartographer_odom.lua"') < hq.index('num_background_threads = 4')
    assert hq.rstrip().endswith('return options') and 'max_num_final_iterations' in hq
    loc = _lua('cartographer_localization.lua')            # localization keeps its own (unchanged) matcher
    assert 'pure_localization_trimmer' in loc


def test_map_offline_script():
    import subprocess
    script = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'scripts', 'map_offline.sh')
    if not os.path.exists(script):
        pytest.skip('scripts/ not in this checkout')
    assert subprocess.run(['bash', '-n', script]).returncode == 0
    r = subprocess.run(['bash', script], capture_output=True, text=True)
    assert r.returncode == 1 and 'record' in r.stdout and 'build' in r.stdout and 'set -e' not in r.stdout
    r = subprocess.run(['bash', script, 'record', 'bad name!'], capture_output=True, text=True)
    assert r.returncode == 1 and 'map name' in r.stderr
    text = open(script).read()
    assert 'cartographer_mapping_hq.lua' in text and '-save_state_filename' in text
    assert 'cartographer_pbstream_to_ros_map' in text and '/tf_static' in text and ' /tf ' not in text
