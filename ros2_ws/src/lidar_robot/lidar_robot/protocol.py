"""ESP32 <-> Pi serial protocol (pure Python, no ROS imports, unit-tested).

ESP32 -> Pi:  ODM,<dl>,<dr>,<dt_ms>,<mode>[,<dyaw_urad>]   (v1: 4 fields, v2.2+IMU: 6 fields)
              IMU,<gz_mrad_s>,<ax_mm_s2>,<ay_mm_s2>,<az_mm_s2>
              US,<left_mm>,<center_mm>,<right_mm>       (0 = no echo)
              INFO,<text>          (INFO,TUNE,<kp>,<ki>,<pwm_min>,<max_mms>,<accel> since v2.3)
              SPD,<tgtL>,<measL>,<pwmL>,<tgtR>,<measR>,<pwmR>   (v2.3, after T,1)
Pi -> ESP32:  V,<left_mm_s>,<right_mm_s>
              K,<kp>,<ki>,<pwm_min>,<max_mms>,<accel>  W (save)  X (defaults)  T,<0|1>  ?
              P,<left_pwm>,<right_pwm>
              S
"""
import math
from dataclasses import dataclass
from typing import Optional, Union

MODE_IDLE, MODE_RC, MODE_AUTO, MODE_ESTOP = 0, 1, 2, 3
MODE_NAMES = {0: 'IDLE', 1: 'RC', 2: 'AUTO', 3: 'ESTOP', -1: 'UNKNOWN'}


@dataclass
class OdomPacket:
    dl: int
    dr: int
    dt_ms: int
    mode: int = -1
    dyaw: Optional[float] = None       # rad, from the gyro (None if no IMU)


@dataclass
class ImuPacket:
    gz: float                          # rad/s (yaw rate, CCW positive)
    ax: float                          # m/s^2
    ay: float
    az: float


@dataclass
class UsPacket:
    left: Optional[float]              # metres, None = nothing in range
    center: Optional[float]
    right: Optional[float]


@dataclass
class InfoPacket:
    text: str


@dataclass
class SpdPacket:
    tl: int       # target mm/s (after the ramp)
    ml: int       # measured mm/s
    pl: int       # PWM applied
    tr: int
    mr: int
    pr: int


Packet = Union[OdomPacket, InfoPacket, ImuPacket, UsPacket, SpdPacket]


def parse_line(line: str) -> Optional[Packet]:
    """Parse one line from the ESP32. Returns None for junk / partial lines."""
    line = line.strip()
    if not line:
        return None
    if line.startswith('ODM,'):
        parts = line.split(',')
        if len(parts) not in (4, 5, 6):
            return None
        try:
            dl, dr, dt = int(parts[1]), int(parts[2]), int(parts[3])
            mode = int(parts[4]) if len(parts) >= 5 else -1
            dyaw = int(parts[5]) * 1e-6 if len(parts) == 6 else None
        except ValueError:
            return None
        if dt <= 0 or dt > 60000:
            return None
        return OdomPacket(dl, dr, dt, mode, dyaw)
    if line.startswith('IMU,'):
        parts = line.split(',')
        if len(parts) != 5:
            return None
        try:
            gz, ax, ay, az = (int(p) * 1e-3 for p in parts[1:])
        except ValueError:
            return None
        return ImuPacket(gz, ax, ay, az)
    if line.startswith('US,'):
        parts = line.split(',')
        if len(parts) != 4:
            return None
        try:
            vals = [int(p) for p in parts[1:]]
        except ValueError:
            return None
        return UsPacket(*[(v / 1000.0 if 20 <= v <= 4000 else None) for v in vals])
    if line.startswith('SPD,'):
        parts = line.split(',')
        if len(parts) != 7:
            return None
        try:
            return SpdPacket(*[int(p) for p in parts[1:]])
        except ValueError:
            return None
    if line.startswith('INFO,'):
        return InfoPacket(line[5:])
    if line == 'ESP32_ROBOT_READY':          # v1 firmware banner
        return InfoPacket('READY (v1 firmware)')
    return None


def format_velocity(left_mps: float, right_mps: float) -> bytes:
    return f'V,{int(round(left_mps * 1000))},{int(round(right_mps * 1000))}\n'.encode()


def format_pwm(left: int, right: int) -> bytes:
    left = max(-255, min(255, int(left)))
    right = max(-255, min(255, int(right)))
    return f'P,{left},{right}\n'.encode()


def format_stop() -> bytes:
    return b'S\n'


def format_estop(on: bool) -> bytes:
    """E = latch e-stop on the ESP32 (also blocks RC), R = release."""
    return b'E\n' if on else b'R\n'


TUNE_KEYS = ('kp', 'ki', 'pwm_min', 'max_mms', 'accel')


def parse_tune(info_text: str):
    """'TUNE,0.15,0.4,15,493,600' -> dict, else None."""
    if not info_text.startswith('TUNE,'):
        return None
    parts = info_text.split(',')[1:]
    if len(parts) != len(TUNE_KEYS):
        return None
    try:
        return {k: float(v) for k, v in zip(TUNE_KEYS, parts)}
    except ValueError:
        return None


def format_tune(kp, ki, pwm_min, max_mms, accel) -> bytes:
    for v in (kp, ki, pwm_min, max_mms, accel):
        if not math.isfinite(float(v)):
            raise ValueError('tuning values must be finite numbers')
    return (f'K,{float(kp):.4f},{float(ki):.4f},{float(pwm_min):.1f},'
            f'{float(max_mms):.0f},{float(accel):.0f}\n').encode()


def format_telemetry(on: bool) -> bytes:
    return b'T,1\n' if on else b'T,0\n'
