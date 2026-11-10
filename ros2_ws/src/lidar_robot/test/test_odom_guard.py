"""Odometry guard: garbled lines, lost lines, gyro drift (the causes of a shifted map)."""
import math

from lidar_robot.kinematics import DiffDriveOdometry
from lidar_robot.odom_guard import OdomGuard
from lidar_robot.protocol import parse_line, OdomPacket


def make_guard(**kw):
    return OdomGuard(0.0625, 5000.0, 5000.0, 0.30, **kw)


def test_parser_reads_old_and_new_odm_lines():
    assert parse_line('ODM,10,12,20,2').dyaw is None
    assert abs(parse_line('ODM,10,12,20,2,1500').dyaw - 0.0015) < 1e-9
    p = parse_line('ODM,10,12,20,2,x,700,712,x')
    assert (p.tl, p.tr, p.dyaw, p.tyaw) == (700, 712, None, None)
    p = parse_line('ODM,10,12,20,2,1500,700,712,90000')
    assert (p.tl, p.tr, p.tyaw) == (700, 712, 90000) and abs(p.dyaw - 0.0015) < 1e-9
    assert parse_line('ODM,10,12,20,2,1500,700') is None          # cut-off line: not a known layout


def test_garbled_line_is_rejected_not_driven():
    g = make_guard()
    assert g.update(OdomPacket(12, 13, 20, 2)) is not None
    assert g.update(OdomPacket(12, 913, 20, 2)) is None            # 913 ticks in 20 ms = 3.6 m/s
    assert g.stats['rejected'] == 1
    # a long packet after the Pi was busy is fine: same speed, more time
    assert g.update(OdomPacket(600, 600, 1000, 2)) is not None


def test_lost_lines_cost_no_distance_with_totals():
    g = make_guard()
    odo = DiffDriveOdometry(0.0625, 0.30, 5000.0, 5000.0)
    tl = tr = 0
    for i in range(100):                                           # 2 s at 50 Hz, 20 ticks per packet
        tl += 20
        tr += 20
        if i in (10, 11, 40, 77):                                  # these lines never arrive
            continue
        dl, dr, dt, gy = g.update(OdomPacket(20, 20, 20, 2, None, tl, tr), now=i * 0.02)
        odo.update(dl, dr, dt, gy)
    want = 100 * 20 * 2 * math.pi * 0.0625 / 5000.0
    assert abs(odo.x - want) < 1e-9
    assert g.stats['recovered_packets'] == 3 and g.stats['recovered_ticks'] == 4 * 40


def test_without_totals_behaves_like_before():
    g = make_guard()
    assert g.update(OdomPacket(20, 22, 20, 2)) == (20, 22, 0.02, None)


def test_esp32_restart_does_not_drive_backwards():
    g = make_guard()
    g.update(OdomPacket(20, 20, 20, 2, None, 50000, 50000), now=0.0)
    # rebooted without the banner being seen: totals start again near zero
    assert g.update(OdomPacket(5, 5, 20, 2, None, 5, 5), now=2.0)[:2] == (5, 5)
    g.reset_totals()
    assert g.update(OdomPacket(5, 5, 20, 2, None, 10, 10), now=2.02)[:2] == (5, 5)


def test_int32_wrap_of_the_totals():
    g = make_guard()
    g.update(OdomPacket(20, 20, 20, 2, None, 2**31 - 10, 2**31 - 10), now=0.0)
    dl, dr, _, _ = g.update(OdomPacket(20, 20, 20, 2, None, -2**31 + 10, -2**31 + 10), now=0.02)
    assert (dl, dr) == (20, 20)


def test_heading_is_held_while_standing_and_drift_is_learned():
    g = make_guard()
    drift = 0.006                                                  # rad/s: 20 degrees a minute
    yaw = 0.0
    for _ in range(50 * 30):                                       # 30 s standing still
        _, _, dt, gy = g.update(OdomPacket(0, 0, 20, 0, drift * 0.02))
        yaw += gy
    assert abs(yaw) < 0.003                                        # without the guard: 0.18 rad
    assert abs(g.bias - drift) < 0.0005 and g.stats['still']
    # driving straight: the learned drift is taken off
    _, _, dt, gy = g.update(OdomPacket(20, 20, 20, 2, drift * 0.02))
    assert abs(gy) < 1e-5


def test_real_turn_is_not_mistaken_for_drift():
    g = make_guard()
    for _ in range(100):                                           # turning on the spot at 0.3 rad/s
        _, _, _, gy = g.update(OdomPacket(-11, 11, 20, 2, 0.3 * 0.02))
        assert abs(gy - 0.006) < 1e-9
    assert g.bias == 0.0
    # pushed round by hand with the wheels locked: far above any drift, never learned
    for _ in range(100):
        g.update(OdomPacket(0, 0, 20, 0, 0.2 * 0.02))
    assert g.bias == 0.0


def test_gyro_glitch_uses_the_wheels_for_that_packet():
    g = make_guard()
    assert g.update(OdomPacket(20, 22, 20, 2, 0.5))[3] is None     # 25 rad/s
    assert g.stats['gyro_glitches'] == 1
    assert g.update(OdomPacket(20, 22, 20, 2, 0.002))[3] is not None


def test_gyro_scale_applied_once():
    g = make_guard(gyro_scale=1.02)
    assert abs(g.update(OdomPacket(-11, 11, 20, 2, 0.006))[3] - 0.006 * 1.02) < 1e-12
