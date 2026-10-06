"""Find the ESP32's serial port without grabbing the LiDAR's (pure Python, unit-tested).

Search order:
  1. /dev/esp32           udev symlink (scripts/install_udev.sh) — the reliable way
  2. /dev/ttyACM*         ESP32-S3 native USB
  3. /dev/ttyUSB*         classic ESP32 (CP2102/CH340 bridge) — same family as the RPLiDAR,
                          so the LiDAR's port (and /dev/rplidar's target) is always skipped
"""
import glob
import os

SEARCH = ['/dev/esp32', '/dev/ttyACM*', '/dev/ttyUSB*']
ALWAYS_EXCLUDE = ['/dev/rplidar']


def pick_esp32_port(exclude=(), glob_fn=glob.glob, realpath=os.path.realpath):
    """Returns (port, fallback_used). fallback_used=True means a /dev/ttyUSB* guess."""
    banned = set()
    for e in list(exclude) + ALWAYS_EXCLUDE:
        e = (e or '').strip()
        if e:
            banned.add(e)
            banned.add(realpath(e))
    for pattern in SEARCH:
        for port in sorted(glob_fn(pattern)):
            if port in banned or realpath(port) in banned:
                continue
            return port, pattern == '/dev/ttyUSB*'
    return None, False
