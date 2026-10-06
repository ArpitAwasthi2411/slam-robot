#!/usr/bin/env python3
"""Run the robot dashboard against a simulated robot — no ROS, no hardware.

    python3 tools/dashboard_sim.py            # then open http://localhost:8080
    python3 tools/dashboard_sim.py --explore  # start with an unknown map (SLAM-style reveal)

Simulates a CSE-floor-like layout (corridor, labs, HOD office, seminar hall), a ray-cast LiDAR,
and runs the REAL navigator code from the ROS package (A* planner, path follower, places,
command parser, mission queue). Use it to practise, demo, and test the app on a laptop.
Set GROQ_API_KEY to try LLM command parsing (needs internet); otherwise the offline parser runs.
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

from lidar_robot.commands import CommandParser                    # noqa: E402
from lidar_robot.dashboard_server import DashboardServer          # noqa: E402
from lidar_robot.kinematics import wrap_angle                     # noqa: E402
from lidar_robot.mapsave import save_map                          # noqa: E402
from lidar_robot.navigator_core import NavigatorCore              # noqa: E402
from lidar_robot.places import PlaceStore                         # noqa: E402
from lidar_robot.planner import GridMap                           # noqa: E402

RES = 0.05
W, H = 300, 200                       # 15 m x 10 m
ORIGIN = (-0.5, -0.5)
HOME = (1.0, 4.25, 0.0)

PLACES = [
    ('HOD office', 2.5, 7.2, math.pi / 2, ['hod', 'head of department']),
    ('Lab 3', 2.2, 1.8, -math.pi / 2, ['computer lab', 'lab three']),
    ('Lab 2', 6.8, 1.6, -math.pi / 2, ['lab two']),
    ('Store', 12.0, 1.8, -math.pi / 2, ['store room']),
    ('Faculty room', 7.5, 7.0, math.pi / 2, ['staff room']),
    ('Seminar hall', 12.4, 7.2, math.pi / 2, ['auditorium']),
    ('Water cooler', 13.2, 4.25, 0.0, ['water']),
]


def build_world():
    occ = [[False] * W for _ in range(H)]

    def box(x0, y0, x1, y1):
        for gy in range(int((y0 - ORIGIN[1]) / RES), int((y1 - ORIGIN[1]) / RES) + 1):
            for gx in range(int((x0 - ORIGIN[0]) / RES), int((x1 - ORIGIN[0]) / RES) + 1):
                if 0 <= gx < W and 0 <= gy < H:
                    occ[gy][gx] = True
    t = 0.08
    box(0, 0, 14, t)                    # outer walls
    box(0, 9 - t, 14, 9)
    box(0, 0, t, 9)
    box(14 - t, 0, 14, 9)
    # corridor south wall (y=3.5) with three doors
    for a, b in ((0, 2.0), (3.0, 6.5), (7.5, 11.0), (12.0, 14)):
        box(a, 3.5, b, 3.5 + t)
    # corridor north wall (y=5.0) with three doors
    for a, b in ((0, 2.5), (3.5, 7.0), (8.0, 12.0), (13.0, 14)):
        box(a, 5.0, b, 5.0 + t)
    box(4.5, 0, 4.5 + t, 3.5)           # room partitions south
    box(9.0, 0, 9.0 + t, 3.5)
    box(5.0, 5.0, 5.0 + t, 9)           # room partitions north
    box(10.0, 5.0, 10.0 + t, 9)
    # furniture
    box(0.6, 0.6, 3.8, 1.1)             # lab 3 benches
    box(0.6, 2.4, 1.6, 2.9)
    box(5.4, 0.5, 8.4, 1.0)             # lab 2 benches
    box(9.6, 0.4, 10.6, 1.2)            # store racks
    box(12.8, 0.4, 13.7, 2.8)
    box(1.0, 7.8, 3.8, 8.6)             # HOD desk
    box(6.0, 8.0, 9.4, 8.6)             # faculty desks
    box(10.8, 7.9, 13.4, 8.4)           # seminar stage
    box(10.6, 6.2, 11.2, 6.6)           # seminar chairs
    box(13.6, 6.2, 13.9, 6.6)
    return occ


class SimBackend:
    def __init__(self, explore=False):
        self.world = build_world()
        self.known = bytearray(W * H)            # 0 unknown, 1 free, 101 occupied (dashboard encoding)
        if not explore:                          # "localization mode": the whole floor is already mapped
            for gy in range(H):
                for gx in range(W):
                    self.known[gy * W + gx] = 101 if self.world[gy][gx] else 1
        self.lock = threading.Lock()
        self.x, self.y, self.yaw = HOME
        self.v = self.w = 0.0
        self.teleop = ((0.0, 0.0), 0.0)
        self.estop = False
        places_file = os.path.join(HERE, '..', 'maps', 'sim_places.json')
        if os.path.exists(places_file):
            os.remove(places_file)
        self.places = PlaceStore(places_file)
        self.places.data['map'] = 'simulated CSE floor'
        for name, x, y, yaw, aliases in PLACES:
            self.places.add(name, x, y, yaw, aliases)
        self.places.set_home(*HOME)
        parser = CommandParser(self.places, groq_api_key=os.environ.get('GROQ_API_KEY'))
        self.nav = NavigatorCore(self.places, parser=parser, dwell_s=2.0)
        self.scan_pts, self.front = [], float('inf')
        self.map_version = 0
        self.cmd = (0.0, 0.0)
        self.source = 'none'
        self._push_map()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---------------------------------------------------------- simulation
    def _occupied(self, x, y):
        gx, gy = int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)
        if not (0 <= gx < W and 0 <= gy < H):
            return True
        return self.world[gy][gx]

    def _scan(self):
        pts, world_pts, front = [], [], float('inf')
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
                    if r < 3.0:
                        world_pts.append((px, py))
                    if abs(a) < math.radians(25) and abs(r * math.sin(a)) < 0.25:
                        front = min(front, r * math.cos(a))
                    break
                if 0 <= gx < W and 0 <= gy < H and self.known[gy * W + gx] != 101:
                    self.known[gy * W + gx] = 1
                r += RES * 0.7
        return pts, world_pts, front

    def _push_map(self):
        data = [(-1 if v == 0 else v - 1) for v in self.known]
        self.nav.set_map(GridMap.from_occupancy(W, H, RES, ORIGIN[0], ORIGIN[1], data))
        self.map_version += 1

    def _loop(self):
        dt, last_scan, last_map = 0.02, 0.0, 0.0
        while True:
            t0 = time.monotonic()
            with self.lock:
                if t0 - last_scan > 0.1:
                    self.scan_pts, world_pts, self.front = self._scan()
                    self.nav.update_scan_world(world_pts)
                    last_scan = t0
                if t0 - last_map > 1.0:
                    self._push_map()
                    last_map = t0
                pose = (self.x, self.y, self.yaw)
                nav_cmd = self.nav.step(pose, self.front, dt)
                (tv, tw), tt = self.teleop
                if self.estop:
                    cmd, self.source = (0.0, 0.0), 'estop'
                elif t0 - tt < 0.5:
                    cmd, self.source = (tv, tw), 'teleop'
                elif self.nav.queue.active:
                    cmd, self.source = nav_cmd, 'nav'
                else:
                    cmd, self.source = (0.0, 0.0), 'none'
                self.cmd = cmd
                a = dt / (0.15 + dt)
                self.v += a * (cmd[0] - self.v)
                self.w += a * (cmd[1] - self.w)
                nx = self.x + self.v * math.cos(self.yaw) * dt
                ny = self.y + self.v * math.sin(self.yaw) * dt
                if not any(self._occupied(nx + 0.22 * math.cos(k), ny + 0.22 * math.sin(k))
                           for k in (0, 0.785, 1.57, 2.36, 3.14, 3.93, 4.71, 5.5)):
                    self.x, self.y = nx, ny       # bump = stop (no tunnelling through walls)
                else:
                    self.v = 0.0
                self.yaw = wrap_angle(self.yaw + self.w * dt)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    # ---------------------------------------------------------- backend API
    def get_state(self):
        with self.lock:
            moving = abs(self.cmd[0]) + abs(self.cmd[1]) > 0
            return {
                'time': time.time(),
                'link': {'connected': True, 'port': '/dev/sim', 'rx_hz': 50.0, 'rx_age': 0.02,
                         'esp_mode': 'ESTOP' if self.estop else ('AUTO' if moving else 'IDLE'),
                         'esp_info': 'simulator', 'cmd_source': self.source, 'bad_lines': 0},
                'bridge_alive': True,
                'estop': self.estop,
                'pose': {'frame': 'map', 'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw': round(self.yaw, 4)},
                'odom': {'v': round(self.v, 3), 'w': round(self.w, 3)},
                'cmd': {'v': round(self.cmd[0], 3), 'w': round(self.cmd[1], 3)},
                'scan': {'hz': 10.0, 'points': list(self.scan_pts)},
                'goal': {},
                'map': {'version': self.map_version, 'width': W, 'height': H, 'resolution': RES},
                'limits': {'max_v': 0.3, 'max_w': 1.2},
                'nav_mode': 'planner',
                'nav': self.nav.status(),
                'places': self.places.summary(),
            }

    def get_map(self, since):
        with self.lock:
            if since == self.map_version:
                return None
            data = base64.b64encode(bytes(self.known)).decode()
            return {'version': self.map_version, 'width': W, 'height': H, 'resolution': RES,
                    'origin': [ORIGIN[0], ORIGIN[1], 0.0], 'data': data}

    def nav_request(self, req):
        with self.lock:
            return self.nav.handle(req, (self.x, self.y, self.yaw))

    def command(self, v, w):
        with self.lock:
            if not self.estop:
                self.teleop = ((max(-0.3, min(0.3, v)), max(-1.2, min(1.2, w))), time.monotonic())

    def set_estop(self, on):
        with self.lock:
            self.estop = on
            self.nav.set_estop(on)

    def send_goal(self, x, y, yaw):
        return self.nav_request({'type': 'goto_pose', 'x': x, 'y': y, 'yaw': yaw})

    def cancel_goal(self):
        return self.nav_request({'type': 'cancel'})

    def save_map(self, name):
        with self.lock:
            data = [(-1 if v == 0 else v - 1) for v in self.known]
        _, y = save_map(os.path.join(HERE, '..', 'maps'), name, W, H, RES, ORIGIN[0], ORIGIN[1], 0.0, data)
        return os.path.relpath(y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--explore', action='store_true', help='start with an unknown map')
    args = ap.parse_args()
    srv = DashboardServer(SimBackend(explore=args.explore), host='0.0.0.0', port=args.port)
    srv.start()
    print(f'Simulated robot dashboard: http://localhost:{args.port}   (Ctrl+C to quit)')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        srv.stop()


if __name__ == '__main__':
    main()
