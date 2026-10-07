import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lidar_robot.protocol import parse_line, OdomPacket, InfoPacket, format_velocity, format_pwm  # noqa: E402
from lidar_robot.kinematics import (DiffDriveOdometry, twist_to_wheels, limit_wheels,  # noqa: E402
                                    yaw_from_quaternion, quaternion_from_yaw, wrap_angle)
from lidar_robot.mapsave import occupancy_to_pgm_bytes  # noqa: E402


def test_parse_v2_and_v1_lines():
    assert parse_line('ODM,12,-3,20,2\n') == OdomPacket(12, -3, 20, 2)
    assert parse_line('ODM,0,0,50') == OdomPacket(0, 0, 50, -1)          # v1 firmware
    assert isinstance(parse_line('INFO,READY,robot_esp32 v2.0'), InfoPacket)
    assert isinstance(parse_line('ESP32_ROBOT_READY'), InfoPacket)


def test_parse_rejects_junk():
    for bad in ['', 'ODM,1,2', 'ODM,a,b,c', 'ODM,1,2,0', 'ODM,1,2,99999', 'xx', 'ODM,1,2,3,4,5,6', 'IMU,1,2', 'US,a,b,c']:
        assert parse_line(bad) is None, bad


def test_format_commands():
    assert format_velocity(0.1234, -0.05) == b'V,123,-50\n'
    assert format_pwm(300, -300) == b'P,255,-255\n'


def test_straight_line_one_metre():
    o = DiffDriveOdometry(0.0625, 0.30, 4740, 4740)
    ticks_per_m = 4740 / (2 * math.pi * 0.0625)
    for _ in range(100):
        o.update(round(ticks_per_m / 100), round(ticks_per_m / 100), 0.02)
    assert abs(o.x - 1.0) < 0.01 and abs(o.y) < 1e-9 and abs(o.theta) < 1e-9
    assert abs(o.v - 0.5) < 0.01


def test_spin_in_place_full_turn():
    o = DiffDriveOdometry(0.0625, 0.30, 4740, 4740)
    wheel_arc = math.pi * 0.30                        # each wheel travels half the circle of diameter sep
    ticks = wheel_arc / (2 * math.pi * 0.0625) * 4740
    n = 200
    for _ in range(n):
        o.update(-ticks / n, ticks / n, 0.02)
    assert abs(wrap_angle(o.theta)) < 1e-6           # back to start heading (2*pi)
    assert abs(o.theta_total - 2 * math.pi) < 1e-6  # but the unwrapped counter saw one full turn
    assert math.hypot(o.x, o.y) < 1e-6


def test_quarter_turn_arc_ends_left():
    o = DiffDriveOdometry(0.0625, 0.30, 4740, 4740)
    v, w = 0.2, 0.5
    vl, vr = twist_to_wheels(v, w, 0.30)
    tpm = 4740 / (2 * math.pi * 0.0625)
    t = (math.pi / 2) / w
    n = 500
    for _ in range(n):
        o.update(vl * t / n * tpm, vr * t / n * tpm, t / n)
    r = v / w
    assert abs(o.x - r) < 0.01 and abs(o.y - r) < 0.01 and abs(o.theta - math.pi / 2) < 0.01


def test_twist_wheel_limits_keep_curvature():
    vl, vr = twist_to_wheels(0.4, 2.0, 0.30)
    lvl, lvr = limit_wheels(vl, vr, 0.45)
    assert max(abs(lvl), abs(lvr)) <= 0.45 + 1e-9
    assert abs(lvl / lvr - vl / vr) < 1e-9


def test_quaternion_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        assert abs(yaw_from_quaternion(*quaternion_from_yaw(yaw)) - yaw) < 1e-9


def test_pgm_orientation_and_values():
    # 2x2 map, ROS row 0 = bottom: [free, occ] bottom, [unknown, free] top
    pgm = occupancy_to_pgm_bytes(2, 2, [0, 100, -1, 0])
    body = pgm.split(b'255\n', 1)[1]
    assert list(body) == [205, 254, 254, 0]          # top row first in PGM


def test_v22_lines_and_gyro_odometry():
    from lidar_robot.protocol import ImuPacket, UsPacket
    from lidar_robot.sensors import front_clearance_from_ranges
    p = parse_line('ODM,10,12,20,2,-15000')
    assert p.dyaw == -0.015 and p.mode == 2
    assert parse_line('IMU,-250,100,0,9810') == ImuPacket(-0.25, 0.1, 0.0, 9.81)
    assert parse_line('US,350,0,5000') == UsPacket(0.35, None, None)
    o = DiffDriveOdometry(0.0625, 0.30, 4740, 4740)
    o.update(100, 100, 0.02, gyro_dtheta=0.1)               # straight wheels, gyro says it turned
    assert abs(o.theta - 0.1) < 1e-12
    mounts = [(0.22, 0.12, math.radians(30)), (0.22, 0.0, 0.0), (0.22, -0.12, -math.radians(30))]
    # centre sensor sees 0.30 m -> obstacle 0.52 m ahead of the robot centre
    assert abs(front_clearance_from_ranges([(None, mounts[0]), (0.30, mounts[1]), (None, mounts[2])], 0.25) - 0.52) < 1e-9
    # left sensor sees 1.0 m at +30 deg -> point at y = 0.62, outside the robot corridor -> ignored
    assert front_clearance_from_ranges([(1.0, mounts[0])], 0.25) == float('inf')
    # left sensor sees 0.15 m -> y = 0.195 (inside corridor), x = 0.35
    assert abs(front_clearance_from_ranges([(0.15, mounts[0])], 0.25) - (0.22 + 0.15 * math.cos(math.radians(30)))) < 1e-9
