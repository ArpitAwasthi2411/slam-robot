"""ESP32 <-> Pi serial protocol (pure Python, no ROS imports, unit-tested).

ESP32 -> Pi:  ODM,<dl>,<dr>,<dt_ms>,<mode>      (v1 firmware sends 4 fields, no mode)
              INFO,<text>
Pi -> ESP32:  V,<left_mm_s>,<right_mm_s>
              P,<left_pwm>,<right_pwm>
              S
"""
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


@dataclass
class InfoPacket:
    text: str


Packet = Union[OdomPacket, InfoPacket]


def parse_line(line: str) -> Optional[Packet]:
    """Parse one line from the ESP32. Returns None for junk / partial lines."""
    line = line.strip()
    if not line:
        return None
    if line.startswith('ODM,'):
        parts = line.split(',')
        if len(parts) not in (4, 5):
            return None
        try:
            dl, dr, dt = int(parts[1]), int(parts[2]), int(parts[3])
            mode = int(parts[4]) if len(parts) == 5 else -1
        except ValueError:
            return None
        if dt <= 0 or dt > 60000:
            return None
        return OdomPacket(dl, dr, dt, mode)
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
