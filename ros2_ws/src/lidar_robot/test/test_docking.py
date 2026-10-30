"""Front docking: marker geometry, the docking controller on synthetic marker poses, and the ArUco detector
on rendered images (needs OpenCV; skipped without it)."""
import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.docking import DockParams, FrontDocker, dock_errors, synth_observation  # noqa: E402

MARKER = (0.0, 0.0, 0.0)          # marker at the origin, its normal pointing +x (towards the room)
P = DockParams()
DOCKED = (P.contact_dist_m, 0.0)  # base_link when the contacts touch, facing -x


def test_errors_are_zero_when_lined_up():
    obs = synth_observation((0.70, 0.0, math.pi), MARKER, P)
    along, lat, head = dock_errors(obs, P)
    assert abs(along - 0.60) < 1e-9 and abs(lat) < 1e-9 and abs(head) < 1e-9


def test_error_signs():
    # robot 5 cm to the +y side of the line, facing the dock: seen facing the dock (-x), +y is its RIGHT
    along, lat, head = dock_errors(synth_observation((0.70, 0.05, math.pi), MARKER, P), P)
    assert lat > 0.04 and abs(head) < 1e-9
    # robot turned 10 deg to the left of the dock direction -> must turn right (head < 0)
    along, lat, head = dock_errors(synth_observation((0.70, 0.0, math.pi + math.radians(10)), MARKER, P), P)
    assert abs(math.degrees(head) + 10) < 1e-6


def test_out_of_view():
    assert synth_observation((0.70, 0.0, 0.0), MARKER, P) is None                 # facing away
    assert synth_observation((0.70, 1.5, math.pi), MARKER, P) is None             # far off to the side
    assert synth_observation((-0.5, 0.0, 0.0), MARKER, P) is None                 # behind the dock


def simulate(start, noise=0.0, contact=True, seed=1, max_s=90.0, hide_after=None, blackout=None):
    rnd = random.Random(seed)
    d = FrontDocker(DockParams())
    x, y, yaw = start
    t, dt = 0.0, 0.05
    d.start(t)
    vmax = 0.0
    modes = []

    def jitter(o):
        if not noise:
            return o
        o = dict(o)
        o['x_m'] += rnd.gauss(0, noise)
        o['z_m'] += rnd.gauss(0, noise)
        o['yaw_deg'] += rnd.gauss(0, noise * 100)
        return o
    while t < max_s and d.result is None:
        obs = None if (blackout and blackout[0] <= t < blackout[1]) else synth_observation((x, y, yaw), MARKER, d.p, jitter)
        if hide_after is not None and t > hide_after:
            obs = None
        touching = contact and x <= DOCKED[0] + 0.005 and abs(y) < 0.03 and abs(math.cos(yaw) + 1) < 0.02
        v, w = d.step(t, obs, (x, y, yaw), {'charging': touching, 'v': 11.8})
        if x - (0.07 if math.cos(yaw) < 0 else 0) <= DOCKED[0] - 0.07 and v > 0:
            v = 0.0                                   # the dock body: the robot can't go further
        vmax = max(vmax, v)
        modes.append(d.mode_on())
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        t += dt
    return d, (x, y, yaw), vmax, modes, t


@pytest.mark.parametrize('start', [(0.70, 0.0, math.pi), (0.72, 0.06, math.pi + 0.15), (0.68, -0.07, math.pi - 0.2),
                                   (0.75, 0.10, math.pi - 0.3)])
def test_docks_from_the_predock_pose_within_a_centimetre(start):
    d, (x, y, yaw), vmax, modes, t = simulate(start)
    assert d.result == 'docked', d.message
    assert abs(y) < 0.012 and abs(x - DOCKED[0]) < 0.01
    assert abs(math.degrees(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi)))) < 3
    assert vmax <= 0.05 + 1e-9
    assert any(modes) and not modes[-1]              # docking mode while creeping in, off once docked


def test_noisy_marker_still_docks():
    d, (x, y, yaw), *_ = simulate((0.70, 0.05, math.pi + 0.1), noise=0.004, seed=3)
    assert d.result == 'docked' and abs(y) < 0.02


def test_marker_found_after_a_search_sweep():
    d, *_ = simulate((0.70, 0.0, math.pi + math.radians(60)))      # marker out of view at the start
    assert d.result == 'docked'


def test_no_contact_backs_off_retries_then_fails():
    d, (x, y, yaw), vmax, modes, t = simulate((0.70, 0.0, math.pi), contact=False)
    assert d.result[0] == 'fail' and d.result[1] == 'DOCK_NO_CONTACT' and d.try_n == d.p.tries
    assert 'after 3 tries' in d.result[2]


def test_no_marker_at_all_fails_with_no_marker():
    d, *_ = simulate((0.70, 0.0, math.pi), hide_after=-1)
    assert d.result[0] == 'fail' and d.result[1] == 'DOCK_NO_MARKER'


def test_marker_lost_while_creeping_backs_off_and_recovers():
    d, *_ = simulate((0.72, 0.03, math.pi), blackout=(3.0, 6.0))
    assert d.result == 'docked' and d.try_n == 2


def test_voltage_rise_counts_as_contact():
    d = FrontDocker(DockParams())
    d.start(0.0)
    d.phase, d._v_start, d._t_phase, d._contact_from = 'contact', 11.6, 0.0, (0, 0, 0)
    t = 0.0
    while d.result is None and t < 5:
        d.step(t, None, (0, 0, 0), {'charging': False, 'v': 11.8})
        t += 0.1
    assert d.result == 'docked'


# ------------------------------------------------------------------ detector on rendered marker images


def _cv():
    try:
        import cv2
        import numpy as np
        if not hasattr(cv2, 'aruco'):
            return None
        return cv2, np
    except ImportError:
        return None


def render_scene(cv2, np, K, x_m, z_m, yaw_deg, size=0.08, w=640, h=480):
    """Project a printed ArUco marker (on a white card) at a known pose into a grey 640x480 image."""
    from lidar_robot.marker_pose import render_marker
    mk = render_marker('DICT_4X4_50', 0, 200)
    card = np.full((300, 300), 255, np.uint8)
    card[50:250, 50:250] = mk
    hs = size / 2 * 300 / 200                         # card half-size in metres (marker = 200 of 300 px)
    a = math.radians(yaw_deg)
    # marker frame: x right, y up, normal towards the camera; rotate about the vertical axis by yaw
    right = np.array([math.cos(a), 0, math.sin(a)])
    up = np.array([0, -1.0, 0])
    c = np.array([x_m, 0.0, z_m])
    corners3 = [c - right * hs + up * hs, c + right * hs + up * hs, c + right * hs - up * hs, c - right * hs - up * hs]
    pts = np.array([[K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]] for p in corners3], np.float32)
    src = np.array([[0, 0], [299, 0], [299, 299], [0, 299]], np.float32)
    M = cv2.getPerspectiveTransform(src, pts)
    img = cv2.warpPerspective(card, M, (w, h), borderValue=128)
    return img


# Position is accurate at every range; the marker's own yaw only when it is big in the picture (an 8 cm
# marker at 0.8 m is ~30 px: its perspective is lost in the corner noise). The navigator therefore takes the
# dock's heading from the map and trusts the marker yaw only close in (docking.fuse_yaw).
@pytest.mark.parametrize('x_m,z_m,yaw,yaw_ok', [(0.0, 0.5, 0.0, True), (0.0, 0.5, 30.0, True), (0.03, 0.3, -25.0, True),
                                                (0.05, 0.4, 15.0, False), (-0.08, 0.8, -20.0, False)])
def test_detector_recovers_the_pose_from_a_synthetic_image(x_m, z_m, yaw, yaw_ok):
    env = _cv()
    if env is None:
        pytest.skip('OpenCV with aruco not installed')
    cv2, np = env
    from lidar_robot.marker_pose import DEFAULT_CALIB, MarkerDetector
    calib = dict(DEFAULT_CALIB, dist_coeffs=[0, 0, 0, 0, 0])
    det = MarkerDetector(calib)
    K = np.array(calib['camera_matrix'])
    r = det.detect(render_scene(cv2, np, K, x_m, z_m, yaw))
    assert r['found'], r
    assert abs(r['x_m'] - x_m) < 0.01 and abs(r['z_m'] - z_m) < 0.02 + 0.03 * z_m
    if yaw_ok:
        assert abs(r['yaw_deg'] - yaw) < 4.0
    assert abs(r['y_m']) < 0.01


def test_detector_scales_calibration_and_ignores_other_ids():
    env = _cv()
    if env is None:
        pytest.skip('OpenCV with aruco not installed')
    cv2, np = env
    from lidar_robot.marker_pose import DEFAULT_CALIB, MarkerDetector
    calib = dict(DEFAULT_CALIB, dist_coeffs=[0, 0, 0, 0, 0])
    K = np.array(calib['camera_matrix'])
    img = render_scene(cv2, np, K, 0.0, 0.5, 0.0)
    big = cv2.resize(img, (1280, 960))
    r = MarkerDetector(calib).detect(big)
    assert r['found'] and abs(r['z_m'] - 0.5) < 0.03
    assert MarkerDetector(calib, marker_id=7).detect(img)['found'] is False
    assert MarkerDetector(calib).detect(np.full((480, 640), 200, np.uint8))['found'] is False


def test_detector_without_opencv_says_why(monkeypatch):
    import lidar_robot.marker_pose as mp
    monkeypatch.setattr(mp, 'cv2', None)
    r = mp.MarkerDetector().detect(object())
    assert r['found'] is False and 'OpenCV' in r['error']


def test_calibration_yaml(tmp_path):
    pytest.importorskip('yaml')
    from lidar_robot.marker_pose import load_calibration
    assert load_calibration(None)['width'] == 640
    f = tmp_path / 'c.yaml'
    f.write_text('width: 1280\nheight: 960\ncamera_matrix: [[600, 0, 640], [0, 600, 480], [0, 0, 1]]\ndist_coeffs: [0, 0, 0, 0, 0]\n')
    c = load_calibration(str(f))
    assert c['width'] == 1280 and c['camera_matrix'][0][0] == 600
    f.write_text('camera_matrix: [[1, 2], [3, 4]]\n')
    with pytest.raises(ValueError):
        load_calibration(str(f))


# ------------------------------------------------------------------ the whole thing on the simulated floor
def test_simulator_robot_docks_nose_first_and_backs_off_for_the_next_task(tmp_path):
    tools = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'tools')
    if not os.path.exists(os.path.join(tools, 'dashboard_sim.py')):
        pytest.skip('tools/ not in this checkout')
    sys.path.insert(0, tools)
    import dashboard_sim as sim

    class Clk:
        t = 0.0

        def __call__(self):
            return self.t
    clock = Clk()
    b = sim.SimBackend(start_thread=False, clock=clock, data_dir=str(tmp_path))
    b.drain = 0.0
    b.x, b.y, b.yaw = 6.0, 4.25, 0.0
    assert not b.nav.wants_marker()                  # idle: the camera doesn't search for the marker
    assert b.nav.handle({'type': 'go_dock'})['ok']
    phases, mode_on, dt = [], False, 0.02
    while clock.t < 240 and not b.nav.docked:
        b.tick(dt, clock.t)
        clock.t += dt
        assert b.nav.wants_marker() or b.nav.docked  # on the way to the dock and while docking: it does
        d = b.nav.status()['docking']
        if d and (not phases or phases[-1] != d['phase']):
            phases.append(d['phase'])
        mode_on = mode_on or b.nav.dock_mode()
    assert b.nav.docked and b.nav.state == 'CHARGING', (phases, b.nav.message)
    assert abs(b.y - sim.DOCK[1]) < 0.025 and b.bumps == 0
    for ph in ('predock', 'align', 'creep', 'contact'):
        assert ph in phases, phases
    assert mode_on and not b.nav.dock_mode()
    assert not b.nav.wants_marker()                  # docked: search off again
    # next task: back straight off the dock first
    b.nav.handle({'type': 'goto_place', 'name': 'Lab 2'})
    for _ in range(50):
        b.tick(dt, clock.t)
        clock.t += dt
    assert b.nav.state == 'UNDOCKING' and b.cmd[0] < 0
