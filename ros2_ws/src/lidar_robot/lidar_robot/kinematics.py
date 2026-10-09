"""Differential-drive kinematics (pure Python, unit-tested)."""
import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float):
    """Returns (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def twist_to_wheels(v: float, w: float, wheel_separation: float):
    """Body twist (m/s, rad/s) -> (left, right) wheel surface speeds in m/s."""
    half = wheel_separation / 2.0
    return v - w * half, v + w * half


def limit_wheels(vl: float, vr: float, vmax: float):
    """Scale both wheels together so neither exceeds vmax (keeps the curvature)."""
    m = max(abs(vl), abs(vr))
    if m > vmax > 0:
        s = vmax / m
        return vl * s, vr * s
    return vl, vr


@dataclass
class DiffDriveOdometry:
    wheel_radius: float
    wheel_separation: float
    ticks_per_rev_left: float
    ticks_per_rev_right: float
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0
    theta_total: float = 0.0      # unwrapped heading, for the wheel-separation spin test

    def reset(self):
        self.x = self.y = self.theta = self.v = self.w = self.theta_total = 0.0

    def update(self, dl_ticks: int, dr_ticks: int, dt: float, gyro_dtheta=None):
        """Wheel odometry; if gyro_dtheta is given (rad), it replaces the wheel-based heading change.

        Wheels measure distance well but heading badly (they slip when turning); a gyro measures
        heading well. Using each for what it is good at is the standard fix for diff-drive drift.
        """
        circ = 2.0 * math.pi * self.wheel_radius
        dist_l = dl_ticks * circ / self.ticks_per_rev_left
        dist_r = dr_ticks * circ / self.ticks_per_rev_right
        d_center = 0.5 * (dist_l + dist_r)
        d_theta = (dist_r - dist_l) / self.wheel_separation
        if gyro_dtheta is not None:
            d_theta = gyro_dtheta
        # midpoint integration
        self.x += d_center * math.cos(self.theta + 0.5 * d_theta)
        self.y += d_center * math.sin(self.theta + 0.5 * d_theta)
        self.theta = wrap_angle(self.theta + d_theta)
        self.theta_total += d_theta
        if dt > 0:
            self.v = d_center / dt
            self.w = d_theta / dt
        return self.x, self.y, self.theta, self.v, self.w


def compose_2d(a, b):
    """Pose b expressed in a's parent frame: (x, y, yaw) of a ∘ b (e.g. map->odom ∘ odom->base_link)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap_angle(ath + bth)
