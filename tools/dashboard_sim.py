#!/usr/bin/env python3
"""Run the robot dashboard against a simulated robot — no ROS, no hardware.

    python3 tools/dashboard_sim.py            # then open http://localhost:8080

Simulates a 6 x 5 m room, a ray-cast LiDAR, a map that is revealed as the robot
'explores' (like SLAM), and uses the REAL goal controller code from the ROS package.
Good for trying the UI, practising, and demoing on a laptop.
"""
import argparse
import base64
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'lidar_robot'))

from lidar_robot.dashboard_server import DashboardServer          # noqa: E402
from lidar_robot.goal_logic import GoalController, GoalParams          # noqa: E402
from lidar_robot.kinematics import wrap_angle                     # noqa: E402
from lidar_robot.mapsave import save_map                          # noqa: E402

RES = 0.05
W, H = 140, 120                       # 7 m x 6 m grid
ORIGIN = (-1.5, -1.5)


def build_world():
    occ = [[False] * W for _ in range(H)]

    def box(x0, y0, x1, y1):
        for gy in range(int((y0 - ORIGIN[1]) / RES), int((y1 - ORIGIN[1]) / RES) + 1):
            for gx in range(int((x0 - ORIGIN[0]) / RES), int((x1 - ORIGIN[0]) / RES) + 1):
                if 0 <= gx < W and 0 <= gy < H:
                    occ[gy][gx] = True
    t = 0.06
    box(-1.0, -1.0, 5.0, -1.0 + t)         # outer walls
    box(-1.0, 4.0 - t, 5.0, 4.0)
    box(-1.0, -1.0, -1.0 + t, 4.0)
    box(5.0 - t, -1.0, 5.0, 4.0)
    box(2.0, -1.0, 2.0 + t, 1.0)           # partition with a doorway
    box(2.0, 1.9, 2.0 + t, 4.0)
    box(0.3, 2.4, 1.2, 3.0)                # table
    box(3.4, 0.4, 3.8, 0.8)                # box
    box(3.6, 2.8, 4.4, 3.5)                # cupboard
    return occ


class SimBackend:
    def __init__(self):
        self.world = build_world()
        self.known = bytearray(W * H)            # 0 unknown, 1 free, 101 occupied (dashboard encoding)
        self.lock = threading.Lock()
        self.x, self.y, self.yaw, self.v, self.w = 0.0, 0.0, 0.3, 0.0, 0.0
        self.teleop = ((0.0, 0.0), 0.0)
        self.estop = False
        self.ctrl = GoalController(GoalParams())
        self.scan_pts, self.front = [], float('inf')
        self.map_version = 0
        self.cmd = (0.0, 0.0)
        self.source = 'none'
        threading.Thread(target=self._loop, daemon=True).start()

    # ---------------------------------------------------------- simulation
    def _occupied(self, x, y):
        gx, gy = int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)
        if not (0 <= gx < W and 0 <= gy < H):
            return True
        return self.world[gy][gx]

    def _scan(self):
        pts, front = [], float('inf')
        for i in range(180):
            a = -math.pi + i * 2 * math.pi / 180
            ca, sa = math.cos(self.yaw + a), math.sin(self.yaw + a)
            r = 0.15
            while r < 8.0:
                px, py = self.x + r * ca, self.y + r * sa
                gx, gy = int((px - ORIGIN[0]) / RES), int((py - ORIGIN[1]) / RES)
                if self._occupied(px, py):
                    if 0 <= gx < W and 0 <= gy < H:
                        self.known[gy * W + gx] = 101
                    pts.append((round(r * math.cos(a), 3), round(r * math.sin(a), 3)))
                    if abs(a) < math.radians(25) and abs(r * math.sin(a)) < 0.2:
                        front = min(front, r * math.cos(a))
                    break
                if 0 <= gx < W and 0 <= gy < H and self.known[gy * W + gx] != 101:
                    self.known[gy * W + gx] = 1
                r += RES * 0.7
        return pts, front

    def _loop(self):
        dt, last_scan, last_map = 0.02, 0.0, 0.0
        while True:
            t0 = time.monotonic()
            with self.lock:
                if t0 - last_scan > 0.1:
                    self.scan_pts, self.front = self._scan()
                    last_scan = t0
                if t0 - last_map > 1.0:
                    self.map_version += 1
                    last_map = t0
                (tv, tw), tt = self.teleop
                if self.estop:
                    cmd, self.source = (0.0, 0.0), 'estop'
                elif t0 - tt < 0.5:
                    cmd, self.source = (tv, tw), 'teleop'
                elif self.ctrl.active:
                    cmd = self.ctrl.step((self.x, self.y, self.yaw), self.front, dt)
                    self.source = 'nav'
                else:
                    cmd, self.source = (0.0, 0.0), 'none'
                self.cmd = cmd
                a = dt / (0.15 + dt)
                self.v += a * (cmd[0] - self.v)
                self.w += a * (cmd[1] - self.w)
                nx = self.x + self.v * math.cos(self.yaw) * dt
                ny = self.y + self.v * math.sin(self.yaw) * dt
                if not any(self._occupied(nx + 0.17 * math.cos(k), ny + 0.17 * math.sin(k))
                           for k in (0, 1.57, 3.14, 4.71)):
                    self.x, self.y = nx, ny       # bump = stop (no tunnelling through walls)
                else:
                    self.v = 0.0
                self.yaw = wrap_angle(self.yaw + self.w * dt)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    # ---------------------------------------------------------- backend API
    def get_state(self):
        with self.lock:
            c = self.ctrl
            g = c.goal
            mode = 'AUTO' if self.source in ('nav', 'teleop') and (abs(self.cmd[0]) + abs(self.cmd[1])) > 0 else 'IDLE'
            return {
                'time': time.time(),
                'link': {'connected': True, 'port': '/dev/sim', 'rx_hz': 50.0, 'rx_age': 0.02,
                         'esp_mode': mode, 'esp_info': 'simulator', 'cmd_source': self.source, 'bad_lines': 0},
                'bridge_alive': True,
                'estop': self.estop,
                'pose': {'frame': 'map', 'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw': round(self.yaw, 4)},
                'odom': {'v': round(self.v, 3), 'w': round(self.w, 3)},
                'cmd': {'v': round(self.cmd[0], 3), 'w': round(self.cmd[1], 3)},
                'scan': {'hz': 10.0, 'points': list(self.scan_pts)},
                'goal': {'state': c.state, 'message': c.message, 'blocked': c.blocked,
                         'goal': None if g is None else {'x': g[0], 'y': g[1], 'yaw': g[2]},
                         'distance': None if g is None else round(c.distance_to_goal((self.x, self.y)), 3),
                         'front_clearance': None if not math.isfinite(self.front) else round(self.front, 2),
                         'frame': 'map'},
                'map': {'version': self.map_version, 'width': W, 'height': H, 'resolution': RES},
                'limits': {'max_v': 0.3, 'max_w': 1.2},
            }

    def get_map(self, since):
        with self.lock:
            if since == self.map_version:
                return None
            data = base64.b64encode(bytes(self.known)).decode()
            return {'version': self.map_version, 'width': W, 'height': H, 'resolution': RES,
                    'origin': [ORIGIN[0], ORIGIN[1], 0.0], 'data': data}

    def command(self, v, w):
        with self.lock:
            if not self.estop:
                self.teleop = ((max(-0.3, min(0.3, v)), max(-1.2, min(1.2, w))), time.monotonic())

    def set_estop(self, on):
        with self.lock:
            self.estop = on
            if on:
                self.ctrl.cancel('e-stop')

    def send_goal(self, x, y, yaw):
        with self.lock:
            self.ctrl.set_goal(x, y, yaw)

    def cancel_goal(self):
        with self.lock:
            self.ctrl.cancel('canceled by user')

    def save_map(self, name):
        with self.lock:
            data = [(-1 if v == 0 else v - 1) for v in self.known]
        _, y = save_map(os.path.join(HERE, '..', 'maps'), name, W, H, RES, ORIGIN[0], ORIGIN[1], 0.0, data)
        return os.path.relpath(y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8080)
    args = ap.parse_args()
    srv = DashboardServer(SimBackend(), host='0.0.0.0', port=args.port)
    srv.start()
    print(f'Simulated robot dashboard: http://localhost:{args.port}   (Ctrl+C to quit)')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        srv.stop()


if __name__ == '__main__':
    main()
