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
from lidar_robot.explorer import MapAutoSaver                    # noqa: E402
from lidar_robot.footprint import Footprint                       # noqa: E402
from lidar_robot.kinematics import wrap_angle                     # noqa: E402
from lidar_robot.mapsave import save_map                          # noqa: E402
from lidar_robot.navigator_core import NavigatorCore              # noqa: E402
from lidar_robot.places import PlaceStore                         # noqa: E402
from lidar_robot.planner import GridMap                           # noqa: E402
from lidar_robot.tuning import (TelemetryBuffer, TestRunner, WheelSim, WHEEL_RANGES,  # noqa: E402
                                clamp_values, ranges_for_app)

SEP = 0.30                            # wheel separation (m)

RES = 0.05
W, H = 300, 200                       # 15 m x 10 m
ORIGIN = (-0.5, -0.5)
HOME = (1.0, 4.25, 0.0)
US_MOUNTS = [[0.07, 0.12, math.radians(30)], [0.07, 0.0, 0.0], [0.07, -0.12, -math.radians(30)]]
FP = Footprint(front=0.07, rear=0.37, half_width=0.21)      # the real body: axle near the FRONT
DOCK = (0.45, 4.25, 0.0)              # sitting on the dock: back against the west wall, facing east
ROOMS = [
    ('Lab 3', [[0.1, 0.1], [4.5, 0.1], [4.5, 3.5], [0.1, 3.5]], ['computers', 'projector', 'whiteboard']),
    ('Lab 2', [[4.6, 0.1], [9.0, 0.1], [9.0, 3.5], [4.6, 3.5]], ['3D printer', 'soldering station', 'oscilloscope']),
    ('Store', [[9.1, 0.1], [13.9, 0.1], [13.9, 3.5], [9.1, 3.5]], ['spare batteries', 'toolbox', 'cables']),
    ('Corridor', [[0.1, 3.6], [13.9, 3.6], [13.9, 4.95], [0.1, 4.95]], ['water cooler', 'notice board']),
    ('HOD office', [[0.1, 5.1], [5.0, 5.1], [5.0, 8.9], [0.1, 8.9]], ['printer', 'files']),
    ('Faculty room', [[5.1, 5.1], [10.0, 5.1], [10.0, 8.9], [5.1, 8.9]], ['coffee machine', 'lockers']),
    ('Seminar hall', [[10.1, 5.1], [13.9, 5.1], [13.9, 8.9], [10.1, 8.9]], ['stage', 'speakers', 'chairs']),
]

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
    def __init__(self, explore=False, start_thread=True, clock=time.monotonic):
        self.explore_mode = explore
        self.clock = clock
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
        self.places.set_dock(*DOCK)
        for name, poly, contents in ROOMS:
            self.places.add_room(name, poly, contents)
        parser = CommandParser(self.places, groq_api_key=os.environ.get('GROQ_API_KEY'))
        sim_dir = os.path.join(HERE, '..', 'maps', 'sim')
        os.makedirs(sim_dir, exist_ok=True)
        for f in ('missions.jsonl', 'schedules.json'):
            if os.path.exists(os.path.join(sim_dir, f)):
                os.remove(os.path.join(sim_dir, f))
        self.nav = NavigatorCore(self.places, parser=parser, dwell_s=2.0, footprint=FP, data_dir=sim_dir, clock=clock,
                                 tuning_path=os.path.join(HERE, '..', 'maps', 'sim_tuning.json'))
        self.outline = FP.outline(0.05)
        self.bumps = 0
        self.bat_v = 11.9                         # 3S pack, ~70 %
        self.bat_a = 0.4
        self.drain = 1.0
        self.use_body_checks = True
        self._contact = False
        # wheels: the firmware's speed loop driving two slightly different motors
        self.tune = {k: r[2] for k, r in WHEEL_RANGES.items()}
        self.saved_tune = dict(self.tune)
        self.wl = WheelSim(self.tune, gain_mms_per_pwm=2.65, seed=1)
        self.wr = WheelSim(self.tune, gain_mms_per_pwm=2.45, deadband_pwm=14.0, seed=2)
        self.telemetry_on = False
        self.odom_turns = 0.0
        self.telemetry = TelemetryBuffer()
        self.tests = TestRunner(self.command, self._sample, is_blocked=lambda: self.estop)
        self.scan_pts, self.front = [], float('inf')
        self.us = [None, None, None]
        self.map_version = 0
        self.cmd = (0.0, 0.0)
        self.source = 'none'
        self.autosave = MapAutoSaver(self.save_map)
        self._last_scan = self._last_map = -1e9
        self._push_map()
        if start_thread:
            threading.Thread(target=self._loop, daemon=True).start()

    # ---------------------------------------------------------- simulation
    def _occupied(self, x, y):
        gx, gy = int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)
        if not (0 <= gx < W and 0 <= gy < H):
            return True
        return self.world[gy][gx]

    def _body_hits(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return any(self._occupied(x + c * bx - s * by, y + s * bx + c * by) for bx, by in self.outline)

    def _on_dock(self):
        return (math.hypot(self.x - DOCK[0], self.y - DOCK[1]) < 0.04
                and abs(wrap_angle(self.yaw - DOCK[2])) < 0.2)

    def _battery(self, dt):
        moving = abs(self.v) > 0.01 or abs(self.w) > 0.05
        if self._on_dock():
            self.bat_a = -1.5                                        # charger current through the contacts
            self.bat_v = min(12.6, self.bat_v + 0.004 * dt)
        else:
            self.bat_a = 1.6 if moving else 0.45
            self.bat_v = max(9.6, self.bat_v - (0.0012 if moving else 0.0002) * self.drain * dt)
        sag = 0.25 if moving else 0.0
        return self.bat_v - sag, self.bat_a

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

    def _ray(self, mount, max_r=4.0):
        mx, my, myaw = mount
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = self.x + c * mx - s * my, self.y + s * mx + c * my
        a = self.yaw + myaw
        r = 0.02
        while r < max_r:
            if self._occupied(ox + r * math.cos(a), oy + r * math.sin(a)):
                return round(r, 3)
            r += 0.02
        return None

    def _push_map(self):
        data = [(-1 if v == 0 else v - 1) for v in self.known]
        self.nav.set_map(GridMap.from_occupancy(W, H, RES, ORIGIN[0], ORIGIN[1], data))
        self.map_version += 1

    def _loop(self):
        dt = 0.02
        while True:
            t0 = time.monotonic()
            with self.lock:
                self.tick(dt, t0)
            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    def tick(self, dt, t0):
        """One 20 ms physics + navigator step (also used by tools/eval_nav.py, faster than real time)."""
        if t0 - self._last_scan > 0.1:
            self.scan_pts, world_pts, self.front = self._scan()
            self.us = [self._ray(m) for m in US_MOUNTS]
            self.nav.update_scan_world(world_pts)
            if self.use_body_checks:
                self.nav.update_scan_base([p for p in self.scan_pts if p[0] * p[0] + p[1] * p[1] < 1.0])
            self.nav.set_battery(*self._battery(t0 - self._last_scan if self._last_scan > 0 else 0.1))
            self._last_scan = t0
        if t0 - self._last_map > 1.0:
            self._push_map()
            self._last_map = t0
        pose = (self.x, self.y, self.yaw)
        nav_cmd = self.nav.step(pose, self.front, dt)
        self.autosave.update(self.nav.explore)
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
        # (v, w) -> wheel targets -> firmware ramp + PI loop -> motors -> (v, w)
        cl, cr = (cmd[0] - cmd[1] * SEP / 2) * 1000, (cmd[0] + cmd[1] * SEP / 2) * 1000
        dl, dr = cl - self.wl.target, cr - self.wr.target
        big = max(abs(dl), abs(dr), 1e-9)
        for wheel, d in ((self.wl, dl), (self.wr, dr)):       # ramp keeps the wheel ratio
            wheel.ramp_to(wheel.target + d, dt, abs(d) / big)
        self.wl.step(dt)
        self.wr.step(dt)
        if self.telemetry_on:
            self.telemetry.add([self.wl.target, self.wl.meas, self.wl.pwm,
                                self.wr.target, self.wr.meas, self.wr.pwm])
        self.v = (self.wl.speed + self.wr.speed) / 2000.0
        self.w = (self.wr.speed - self.wl.speed) / 1000.0 / SEP
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        nyaw = wrap_angle(self.yaw + self.w * dt)
        docking = self._on_dock() and self.v <= 0
        if (abs(self.v) > 1e-4 or abs(self.w) > 1e-4) and not docking \
                and self._body_hits(nx, ny, nyaw) and not self._body_hits(self.x, self.y, self.yaw):
            self.v = self.w = 0.0          # the body (any part of the rectangle) touched a wall
            if not self._contact:
                self.bumps += 1
                self.nav.note_bump()
            self._contact = True
        elif docking or not self._body_hits(nx, ny, nyaw):
            self.x, self.y, self.yaw = nx, ny, nyaw
            self._contact = False
        # wheel odometry over-counts turns by 4 % (as if wheel_separation were set 4 % too small)
        self.odom_turns += self.w * dt * 1.04 / (2 * math.pi)

    # ---------------------------------------------------------- tuning lab
    def _sample(self):
        with self.lock:
            return {'x': round(self.x, 4), 'y': round(self.y, 4), 'yaw': round(self.yaw, 4),
                    'v': round(self.v, 3), 'w': round(self.w, 3), 'turns': round(self.odom_turns, 4)}

    def ping(self):
        return {'ok': True, 'robot': 'slam-robot (simulator)', 'api': 2, 'fw': '2.3-sim',
                'host': 'simulator', 'nav_mode': 'planner', 'wheel_ranges': ranges_for_app(WHEEL_RANGES)}

    def esp_command(self, req):
        op = req.get('op')
        with self.lock:
            if op == 'tune':
                self.tune.update(clamp_values(req.get('values', {}), WHEEL_RANGES))
                self.wl.t.update(self.tune)
                self.wr.t.update(self.tune)
                self.wl.integ = self.wr.integ = 0.0
            elif op == 'save':
                self.saved_tune = dict(self.tune)
            elif op == 'defaults':
                self.tune.update({k: r[2] for k, r in WHEEL_RANGES.items()})
                self.wl.t.update(self.tune)
                self.wr.t.update(self.tune)
            elif op == 'telemetry':
                self.telemetry_on = bool(req.get('on'))
            elif op == 'sim_battery':                 # simulator only: set the pack voltage for demos
                self.bat_v = max(9.6, min(12.6, float(req.get('v', self.bat_v))))
                self.drain = float(req.get('drain', self.drain))
            elif op != 'query':
                raise ValueError('unknown op')
        return {'ok': True}

    def get_telemetry(self, since):
        return self.telemetry.since(since)

    def test_start(self, req):
        if self.estop:
            raise ValueError('release the e-stop first')
        return self.tests.start(req)

    def test_stop(self):
        return self.tests.stop()

    def test_status(self, since):
        return self.tests.status(since)

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
                'sensors': {'us': list(self.us), 'imu_ok': True, 'yaw_source': 'gyro', 'us_mounts': US_MOUNTS,
                            'bumper': False, 'bumps': self.bumps},
                'nav': self.nav.status(),
                'places': self.places.summary(),
                'tune': dict(self.tune), 'fw': '2.3-sim', 'telemetry': self.telemetry_on,
                'turns': round(self.odom_turns, 3), 'sep': SEP, 'test_running': bool(self.tests.result.get('running')),
                'autosave': self.autosave.info,
                'slam_mode': 'mapping' if self.explore_mode else 'localization',
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

    def relocalize(self, body):
        x, y, yaw = float(body['x']), float(body['y']), float(body.get('yaw', 0.0))
        if self.explore_mode:
            raise ValueError('Set position is for a saved map (localization mode).')
        with self.lock:                     # the simulator "relocalizes" by moving its robot there
            self.x, self.y, self.yaw = x, y, yaw
        return {'ok': True, 'trajectory': 1}

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
