"""esp32_bridge end-to-end over a real pseudo-terminal, against an emulated ESP32 firmware v2.

Checks: odometry from ticks, cmd_vel -> 'V' commands, teleop priority, stale-command stop,
e-stop, v1-firmware compatibility, and reconnect after the cable is pulled.
"""
import math
import os
import sys
import threading
import time

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

import fake_ros  # noqa: E402
fake_ros.install()

serial = pytest.importorskip('serial')
from lidar_robot import esp32_bridge  # noqa: E402

TICKS_PER_M = 4740 / (2 * math.pi * 0.0625)


class FakeEsp32:
    """Speaks the v2 protocol on the master side of a pty. Wheel speed = commanded speed."""

    def __init__(self, v1=False):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self.slave = slave
        self.v1 = v1
        self.target = (0.0, 0.0)          # m/s
        self.last_cmd = 0.0
        self.rx_lines = []
        self.running = True
        self.estop = False
        self.lock = threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._tx, daemon=True).start()

    def _rx(self):
        import select
        buf = b''
        while self.running:
            r, _, _ = select.select([self.master], [], [], 0.05)
            if not r:
                continue
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.decode()
                with self.lock:
                    self.rx_lines.append(line)
                    if line == 'E':
                        self.estop = True
                    elif line == 'R':
                        self.estop = False
                    elif line.startswith('V,') and self.estop:
                        pass                             # latched: ignore commands
                    elif line.startswith('V,'):
                        _, l, r = line.split(',')
                        self.target = (int(l) / 1000.0, int(r) / 1000.0)
                        self.last_cmd = time.monotonic()
                    elif line == 'S':
                        self.target = (0.0, 0.0)
                    elif line == '?' and not self.v1:
                        os.write(self.master, b'INFO,robot_esp32 v2.0 (fake)\n')

    def _tx(self):
        os.write(self.master, b'INFO,READY,fake v2\n' if not self.v1 else b'ESP32_ROBOT_READY\n')
        while self.running:
            time.sleep(0.02)
            with self.lock:
                fresh = time.monotonic() - self.last_cmd < 0.3 and not self.estop
                vl, vr = self.target if fresh else (0.0, 0.0)
                mode = 3 if self.estop else (2 if fresh else 0)
            dl, dr = round(vl * 0.02 * TICKS_PER_M), round(vr * 0.02 * TICKS_PER_M)
            line = f'ODM,{dl},{dr},20' + ('' if self.v1 else f',{mode}') + '\n'
            try:
                os.write(self.master, line.encode())
            except OSError:
                return

    def drive_by_hand(self, metres):
        """Robot pushed forward by hand: raw ticks, no command."""
        os.write(self.master, f'ODM,{round(metres * TICKS_PER_M)},{round(metres * TICKS_PER_M)},20,0\n'.encode())

    def unplug(self):
        self.running = False
        time.sleep(0.1)                     # let the reader thread leave read()
        os.close(self.master)

    def sent(self, prefix):
        with self.lock:
            return [line for line in self.rx_lines if line.startswith(prefix)]


def make_bridge(port):
    fake_ros.Node.overrides = {'serial_port': port}
    node = esp32_bridge.Esp32Bridge()
    timers = {round(p, 2): cb for p, cb in node.timers}
    return node, timers


_SCHEDULES = {}


def pump(timers, seconds):
    """Run the node's timers like an executor would (schedule persists across calls)."""
    end = time.monotonic() + seconds
    nxt = _SCHEDULES.setdefault(id(timers), {p: time.monotonic() + p for p in timers})
    while time.monotonic() < end:
        now = time.monotonic()
        for p, cb in timers.items():
            if now >= nxt[p]:
                cb()
                nxt[p] = now + p
        time.sleep(0.005)


def twist(v, w):
    t = fake_ros.Twist()
    t.linear.x, t.angular.z = v, w
    return t


def test_full_bridge_cycle():
    esp = FakeEsp32()
    node, timers = make_bridge(esp.path)
    try:
        pump(timers, 0.6)
        assert any('robot_esp32 v2.0' in m for _, m in node.logs), node.logs   # firmware identified itself
        odom = node.pubs['odom'].msgs
        assert len(odom) > 10, 'no odometry published'

        # 1) hand push 0.5 m -> odom x ~ 0.5
        esp.drive_by_hand(0.5)
        pump(timers, 0.2)
        assert abs(node.odom.x - 0.5) < 0.01, node.odom.x

        # 2) nav cmd_vel 0.2 m/s for 1 s -> ~0.2 m more, 'V,200,200' sent
        x0 = node.odom.x
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            node.subs['cmd_vel'](twist(0.2, 0.0))
            pump(timers, 0.05)
        assert 'V,200,200' in esp.sent('V,'), esp.sent('V,')[-3:]
        assert 0.15 < node.odom.x - x0 < 0.26, node.odom.x - x0
        assert node.cmd_source == 'nav'

        # 3) teleop wins over nav
        for _ in range(10):
            node.subs['cmd_vel'](twist(0.2, 0.0))
            node.subs['cmd_vel_teleop'](twist(0.0, 1.0))
            pump(timers, 0.05)
        assert node.cmd_source == 'teleop'
        assert esp.sent('V,')[-1] == 'V,-150,150', esp.sent('V,')[-1]     # pure rotation, sep 0.30

        # 4) stale commands -> explicit stops then silence
        n_before = len(esp.sent('V,'))
        pump(timers, 1.2)
        assert len(esp.sent('V,')) <= n_before + 10      # only the ~0.5 s before timeout
        assert len(esp.sent('S')) >= 3
        assert node.cmd_source == 'none'

        # 5) e-stop blocks everything and is latched INSIDE the ESP32 (mode 3)
        node.subs['estop'](fake_ros.Bool(data=True))
        n_before = len(esp.sent('V,'))
        for _ in range(10):
            node.subs['cmd_vel_teleop'](twist(0.3, 0.0))
            pump(timers, 0.05)
        assert len(esp.sent('V,')) == n_before
        assert node.cmd_source == 'estop'
        assert esp.sent('E') and esp.estop and node.esp_mode == 3
        node.subs['estop'](fake_ros.Bool(data=False))
        pump(timers, 0.3)
        assert esp.sent('R') and not esp.estop and not node.estop   # release not undone by stale mode-3 packets

        # 6) status JSON is well formed (status is published at 2 Hz)
        pump(timers, 0.6)
        import json
        st = json.loads(node.pubs['robot/status'].msgs[-1].data)
        assert st['connected'] and st['esp_mode'] in ('IDLE', 'AUTO') and st['rx_hz'] > 30

        # 7) TF published alongside odom
        assert len(node.tf_pub.sent) == len(odom)
    finally:
        node.running = False
        esp.unplug()


def test_v1_firmware_still_works():
    esp = FakeEsp32(v1=True)
    node, timers = make_bridge(esp.path)
    try:
        pump(timers, 0.5)
        esp_lines_ok = len(node.pubs['odom'].msgs) > 10
        assert esp_lines_ok
        assert node.bad_lines == 0
    finally:
        node.running = False
        esp.unplug()


def test_reconnects_after_unplug():
    esp = FakeEsp32()
    node, timers = make_bridge(esp.path)
    try:
        pump(timers, 0.4)
        esp.unplug()                                   # cable pulled
        pump(timers, 0.8)
        assert any('reconnect' in m.lower() or 'failed' in m.lower() for _, m in node.logs), node.logs
        n = len(node.pubs['odom'].msgs)
        esp2 = FakeEsp32()                             # plugged back in (new pty, same role)
        node.port_param = esp2.path
        pump(timers, 3.5)
        assert len(node.pubs['odom'].msgs) > n + 20, 'did not resume after reconnect'
        esp2.unplug()
    finally:
        node.running = False


def test_esp32_latched_before_restart_is_detected():
    """Pi/bridge restarted while the ESP32 was still e-stopped: bridge must show it, not drive."""
    esp = FakeEsp32()
    esp.estop = True
    node, timers = make_bridge(esp.path)
    try:
        pump(timers, 0.5)
        assert node.estop and any('latched E-STOP' in m for _, m in node.logs)
        n = len(esp.sent('V,'))
        for _ in range(6):
            node.subs['cmd_vel_teleop'](twist(0.2, 0.0))
            pump(timers, 0.05)
        assert len(esp.sent('V,')) == n                  # nothing sent while latched
        node.subs['estop'](fake_ros.Bool(data=False))   # operator releases
        for _ in range(6):
            node.subs['cmd_vel_teleop'](twist(0.2, 0.0))
            pump(timers, 0.05)
        assert not node.estop and len(esp.sent('V,')) > n
    finally:
        node.running = False
        esp.unplug()
