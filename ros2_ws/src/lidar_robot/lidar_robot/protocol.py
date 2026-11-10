"""ESP32 <-> Pi serial protocol (pure Python, no ROS imports, unit-tested).

ESP32 -> Pi:  ODM,<dl>,<dr>,<dt_ms>,<mode>[,<dyaw_urad>]   (v1: 4 fields, v2.2+IMU: 6 fields)
              v2.8 adds running totals so a lost line loses no motion:
              ODM,<dl>,<dr>,<dt_ms>,<mode>,<dyaw_urad>,<total_l>,<total_r>,<total_yaw_urad>
              (no IMU: dyaw and total_yaw are the letter x). Always 8 fields, so a cut-off line is never
              mistaken for another layout.
              IMU,<gz_mrad_s>,<ax_mm_s2>,<ay_mm_s2>,<az_mm_s2>
              US,<left_mm>,<center_mm>,<right_mm>       (0 = no echo)
              INFO,<text>          (INFO,TUNE,<kp>,<ki>,<pwm_min>,<max_mms>,<accel> since v2.3)
              SPD,<tgtL>,<measL>,<pwmL>,<tgtR>,<measR>,<pwmR>   (v2.3, after T,1)
              BAT,<mV>,<mA>,<src>   (v2.4; mA > 0 discharging, < 0 charging; src 1 INA226, 2 divider)
              BMP,<0|1>             (v2.4; bumper released / pressed)
Pi -> ESP32:  V,<left_mm_s>,<right_mm_s>
              K,<kp>,<ki>,<pwm_min>,<max_mms>,<accel>  W (save)  X (defaults)  T,<0|1>  ?
              P,<left_pwm>,<right_pwm>
              S
              D,<0|1>               (S3 v2.7 / classic v2.5: front-docking mode, refresh < 2 s)
              I                     (I2C scan -> INFO,I2C,0x40,0x68)
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
    tl: Optional[int] = None           # running tick totals (firmware v2.8+), int32, may wrap
    tr: Optional[int] = None
    tyaw: Optional[int] = None         # running gyro yaw, micro-rad, int32, may wrap


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


@dataclass
class BatPacket:
    volts: float
    amps: Optional[float]     # None when the source can't measure current (divider)
    src: str                  # 'ina226' | 'adc'


@dataclass
class BumpPacket:
    pressed: bool


Packet = Union[OdomPacket, InfoPacket, ImuPacket, UsPacket, SpdPacket, BatPacket, BumpPacket]


def parse_line(line: str) -> Optional[Packet]:
    """Parse one line from the ESP32. Returns None for junk / partial lines."""
    line = line.strip()
    if not line:
        return None
    if line.startswith('ODM,'):
        parts = line.split(',')
        n = len(parts) - 1                       # fields after 'ODM'
        if n not in (3, 4, 5, 8):
            return None
        try:
            dl, dr, dt = int(parts[1]), int(parts[2]), int(parts[3])
            mode = int(parts[4]) if n >= 4 else -1
            dyaw = int(parts[5]) * 1e-6 if (n in (5, 8) and parts[5] != 'x') else None
            tl = tr = tyaw = None
            if n == 8:
                tl, tr = int(parts[6]), int(parts[7])
                tyaw = int(parts[8]) if parts[8] != 'x' else None
        except ValueError:
            return None
        if dt <= 0 or dt > 60000:
            return None
        return OdomPacket(dl, dr, dt, mode, dyaw, tl, tr, tyaw)
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
    if line.startswith('BAT,'):
        parts = line.split(',')
        if len(parts) != 4:
            return None
        try:
            mv, ma, src = int(parts[1]), int(parts[2]), int(parts[3])
        except ValueError:
            return None
        if not 1000 <= mv <= 60000 or src not in (1, 2):
            return None
        return BatPacket(mv / 1000.0, ma / 1000.0 if src == 1 else None, 'ina226' if src == 1 else 'adc')
    if line.startswith('BMP,'):
        v = line[4:]
        return BumpPacket(v == '1') if v in ('0', '1') else None
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


def format_dock(on: bool) -> bytes:
    """Docking mode: the ESP32 relaxes its 15 cm front ultrasonic stop at creep speed (needs a refresh every < 2 s)."""
    return b'D,1\n' if on else b'D,0\n'


def format_i2c_scan() -> bytes:
    return b'I\n'
