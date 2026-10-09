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


def detect_ports(ports=None, open_fn=None, listen_s=3.0, wait_devices_s=15.0, clock=None, sleep=None):
    """Find the ESP32 and the LiDAR by what they send, whatever USB socket they're in.

    The ESP32 firmware streams 'ODM,' lines; the RPLiDAR stays silent until asked to scan.
    Every candidate port is opened at the same time and listened to for `listen_s` seconds.
    Returns (esp32_port, lidar_port); either can be None if it isn't found.
    """
    import time as _t
    clock = clock or _t.monotonic
    sleep = sleep or _t.sleep
    if open_fn is None:
        def open_fn(p):
            import serial
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = p, 115200, 0
            s.dtr = False                        # don't reset / hold the ESP32 in its bootloader
            s.rts = False
            s.open()
            return s
    if ports is None:
        end = clock() + wait_devices_s           # at boot the USB devices can appear late
        while True:
            ports = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
            if len(ports) >= 2 or clock() > end:
                break
            sleep(0.5)
    opened, bufs = {}, {}
    for p in ports:
        try:
            opened[p] = open_fn(p)
            bufs[p] = b''
        except Exception:                        # busy (another launch running?) or no permission
            pass
    esp = None
    end = clock() + listen_s
    try:
        while esp is None and clock() < end and opened:
            for p, s in opened.items():
                try:
                    bufs[p] = (bufs[p] + (s.read(512) or b''))[-2048:]
                except Exception:
                    continue
                if b'ODM,' in bufs[p]:
                    esp = p
                    break
            sleep(0.05)
    finally:
        for s in opened.values():
            try:
                s.close()
            except Exception:
                pass
    others = [p for p in ports if p != esp and p.startswith('/dev/ttyUSB')] or [p for p in ports if p != esp]
    return esp, (others[0] if others else None)
