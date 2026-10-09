#!/usr/bin/env python3
"""Web teleop dashboard node. Open http://<pi-ip>:8080 from the laptop or a phone.

Publishes   /cmd_vel_teleop (Twist)  /estop (Bool, latched)  /goal_pose  /goal_cancel
            /navigator/request (String JSON)
Subscribes  /map  /scan  /robot/status  /goal_controller/status  /navigator/status
            /navigator/places  /navigator/response  + TF map->base_link
Map saving writes ~/maps/<name>.pgm/.yaml (map_server format) from /map, plus
~/maps/<name>.pbstream (Cartographer state, for slam_mode:=localization) when SLAM is running.
"""
import base64
import itertools
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener, TransformException

from lidar_robot.dashboard_server import DashboardServer
from lidar_robot.explorer import MapAutoSaver
from lidar_robot.kinematics import yaw_from_quaternion, quaternion_from_yaw, wrap_angle
from lidar_robot.mapsave import save_map
from lidar_robot.tuning import TelemetryBuffer, TestRunner, ranges_for_app, WHEEL_RANGES

APP_API = 2


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('map_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('scan_decimation', 3)
        self.declare_parameter('nav_mode', 'planner')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.global_frame = gp('global_frame')
        self.fallback_frame = gp('fallback_frame')
        self.base_frame = gp('base_frame')
        self.max_v = float(gp('max_linear'))
        self.max_w = float(gp('max_angular'))
        self.map_dir = gp('map_dir')
        self.scan_dec = max(1, int(gp('scan_decimation')))
        self.nav_mode = gp('nav_mode')

        self.lock = threading.Lock()
        self.map_msg = None
        self.map_version = 0
        self.map_cache = None          # (version, dict)
        self.scan_pts = []
        self.scan_times = []
        self.laser_tf = None
        self.robot_status = {}
        self.robot_status_t = 0.0
        self.goal_status = {}
        self.estop = False                 # mirrors the bridge's value (bridge is authoritative)
        self.nav_status = {}
        self.nav_status_t = 0.0
        self.autosave = MapAutoSaver(self.save_map)      # exploration finished -> save the map
        self.places = {'places': [], 'home': None, 'version': -1}
        self.pending = {}                  # req_id -> [Event, response]
        self.req_ids = itertools.count(1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', latched)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, 'goal_cancel', 10)
        self.nav_req_pub = self.create_publisher(String, 'navigator/request', 10)
        self.esp_cmd_pub = self.create_publisher(String, 'robot/esp_cmd', 10)
        self.telemetry = TelemetryBuffer()
        self.create_subscription(String, 'robot/wheel_telemetry', self._on_wheel_telemetry, 20)
        self.tests = TestRunner(self._test_cmd, self._test_sample,
                                is_blocked=lambda: self.estop or self.robot_status.get('esp_mode') == 'RC')
        # NOTE: never publish /estop at startup: a restarted dashboard must not release an e-stop

        # volatile+reliable matches both cartographer (volatile) and map_server (latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, 1)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'robot/status', self._on_robot_status, 10)
        self.create_subscription(String, 'goal_controller/status', self._on_goal_status, 10)
        self.create_subscription(String, 'navigator/status', self._on_nav_status, 10)
        self.create_subscription(String, 'navigator/places', self._on_places, latched)
        self.create_subscription(String, 'navigator/response', self._on_nav_response, 10)
        self.write_state_cli = None
        try:                                # Cartographer service to save a .pbstream (optional)
            from cartographer_ros_msgs.srv import WriteState
            self._WriteState = WriteState
            self.write_state_cli = self.create_client(WriteState, 'write_state')
        except ImportError:
            self._WriteState = None

        port = int(gp('port'))
        self.server = DashboardServer(self, port=port)
        self.server.start()
        self.get_logger().info(f'Dashboard on http://0.0.0.0:{port}  (open http://<pi-ip>:{port})')

    # ------------------------------------------------------------ ROS callbacks
    def _on_map(self, msg):
        with self.lock:
            self.map_msg = msg
            self.map_version += 1

    def _on_scan(self, msg: LaserScan):
        if self.laser_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                t = tf.transform
                self.laser_tf = (t.translation.x, t.translation.y,
                                 yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
            except TransformException:
                return
        lx, ly, lyaw = self.laser_tf
        pts = []
        a = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if i % self.scan_dec == 0 and msg.range_min < r < msg.range_max and math.isfinite(r):
                ab = a + lyaw
                pts.append((round(lx + r * math.cos(ab), 3), round(ly + r * math.sin(ab), 3)))
            a += msg.angle_increment
        now = time.monotonic()
        with self.lock:
            self.scan_pts = pts
            self.scan_times = [t for t in self.scan_times if now - t < 2.0] + [now]

    def _on_robot_status(self, msg):
        try:
            self.robot_status = json.loads(msg.data)
            self.robot_status_t = time.monotonic()
            self.estop = bool(self.robot_status.get('estop', False))
        except ValueError:
            pass

    def _on_wheel_telemetry(self, msg):
        try:
            batch = json.loads(msg.data).get('s', [])
        except ValueError:
            return
        now = time.monotonic()
        n = len(batch)
        for i, smp in enumerate(batch):       # 50 Hz samples arrive in 10 Hz batches: spread the stamps
            if isinstance(smp, list) and len(smp) == 6:
                self.telemetry.add(smp, now - (n - 1 - i) * 0.02)

    def _on_goal_status(self, msg):
        try:
            self.goal_status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_nav_status(self, msg):
        try:
            self.nav_status = json.loads(msg.data)
            self.nav_status_t = time.monotonic()
        except ValueError:
            return
        self.autosave.update(self.nav_status.get('explore'))

    def _on_places(self, msg):
        try:
            self.places = json.loads(msg.data)
        except ValueError:
            pass

    def _on_nav_response(self, msg):
        try:
            res = json.loads(msg.data)
        except ValueError:
            return
        slot = self.pending.get(res.get('req_id'))
        if slot:
            slot[1] = res
            slot[0].set()

    def _us_mounts(self):
        """[x, y, yaw] of us_left/center/right in base_link from TF (cached once found)."""
        if getattr(self, '_mounts', None):
            return self._mounts
        out = []
        for f in ('us_left', 'us_center', 'us_right'):
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, f, Time()).transform
            except TransformException:
                return None
            out.append([t.translation.x, t.translation.y,
                        yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)])
        self._mounts = out
        return out

    def _pose(self):
        for frame in (self.global_frame, self.fallback_frame):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
                t = tf.transform
                yaw = yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
                return {'frame': frame, 'x': round(t.translation.x, 3),
                        'y': round(t.translation.y, 3), 'yaw': round(yaw, 4)}
            except TransformException:
                continue
        return None

    # ------------------------------------------------------------ backend API
    def get_state(self):
        alive = time.monotonic() - self.robot_status_t < 1.5
        rs = self.robot_status if alive else {}
        with self.lock:
            m = self.map_msg
            scan_hz = len(self.scan_times) / 2.0
            pts = list(self.scan_pts)
        odom = rs.get('odom', {})
        return {
            'time': time.time(),
            'link': {k: rs.get(k) for k in ('connected', 'port', 'rx_hz', 'rx_age', 'esp_mode',
                                            'esp_info', 'cmd_source', 'bad_lines')},
            'bridge_alive': alive,
            'estop': self.estop,
            'pose': self._pose(),
            'odom': {'v': odom.get('v'), 'w': odom.get('w')},
            'cmd': {'v': rs.get('cmd_v'), 'w': rs.get('cmd_w')},
            'scan': {'hz': scan_hz, 'points': pts},
            'goal': self.goal_status,
            'map': None if m is None else {'version': self.map_version, 'width': m.info.width,
                                           'height': m.info.height, 'resolution': m.info.resolution},
            'limits': {'max_v': self.max_v, 'max_w': self.max_w},
            'nav_mode': self.nav_mode,
            'sensors': {'us': rs.get('us'), 'imu_ok': rs.get('imu_ok'), 'yaw_source': rs.get('yaw_source'),
                        'us_mounts': self._us_mounts()},
            'nav': self.nav_status if time.monotonic() - self.nav_status_t < 2.0 else None,
            'tune': rs.get('tune'), 'fw': rs.get('fw'), 'telemetry': rs.get('telemetry'),
            'turns': odom.get('turns'), 'sep': rs.get('sep'), 'test_running': bool(self.tests.result.get('running')),
            'autosave': self.autosave.info,
            'places': self.places,
        }

    # ------------------------------------------------------------ mobile app / Tuning Lab
    def ping(self):
        rs = self.robot_status
        return {'ok': True, 'robot': 'slam-robot', 'api': APP_API, 'fw': rs.get('fw'),
                'host': os.uname().nodename, 'nav_mode': self.nav_mode,
                'wheel_ranges': ranges_for_app(WHEEL_RANGES)}

    def esp_command(self, req):
        op = req.get('op')
        if op not in ('tune', 'save', 'defaults', 'telemetry', 'query'):
            raise ValueError('unknown op')
        self.esp_cmd_pub.publish(String(data=json.dumps(req)))
        return {'ok': True}

    def get_telemetry(self, since):
        return self.telemetry.since(since)

    def _test_cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.cmd_pub.publish(t)

    def _test_sample(self):
        p = self._pose()
        odom = self.robot_status.get('odom', {})
        if not p:
            return None
        return {'x': p['x'], 'y': p['y'], 'yaw': p['yaw'], 'v': odom.get('v'), 'w': odom.get('w'),
                'turns': odom.get('turns')}

    def test_start(self, req):
        if self.estop:
            raise ValueError('release the e-stop first')
        return self.tests.start(req)

    def test_stop(self):
        return self.tests.stop()

    def test_status(self, since):
        return self.tests.status(since)

    def get_map(self, since):
        with self.lock:
            m, ver = self.map_msg, self.map_version
            if m is None or since == ver:
                return None
            if self.map_cache and self.map_cache[0] == ver:
                return self.map_cache[1]
        # unknown(-1)->0, 0..100 -> 1..101 so it fits in a byte
        raw = bytes((v + 1) if v >= 0 else 0 for v in m.data)
        o = m.info.origin
        out = {
            'version': ver, 'width': m.info.width, 'height': m.info.height,
            'resolution': m.info.resolution,
            'origin': [o.position.x, o.position.y,
                       yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)],
            'data': base64.b64encode(raw).decode(),
        }
        with self.lock:
            self.map_cache = (ver, out)
        return out

    def command(self, v, w):
        if self.estop:
            return
        t = Twist()
        t.linear.x = max(-self.max_v, min(self.max_v, v))
        t.angular.z = max(-self.max_w, min(self.max_w, w))
        self.cmd_pub.publish(t)

    def set_estop(self, on):
        self.estop = bool(on)                  # immediate local block; bridge status confirms
        self.estop_pub.publish(Bool(data=self.estop))
        if self.estop:
            self.cmd_pub.publish(Twist())
            if self.nav_mode != 'planner':        # the navigator pauses its missions itself
                self.cancel_pub.publish(Empty())

    def send_goal(self, x, y, yaw):
        pose = self._pose()
        frame = pose['frame'] if pose else self.global_frame
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        _, _, qz, qw = quaternion_from_yaw(wrap_angle(yaw))
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.goal_pub.publish(msg)

    def cancel_goal(self):
        self.cancel_pub.publish(Empty())
        self.cmd_pub.publish(Twist())

    def nav_request(self, req):
        """Forward a JSON request to the navigator node and wait (max 3 s) for its reply."""
        if self.nav_mode != 'planner':
            return {'ok': False, 'error': 'navigator not running (bringup nav_mode:=planner)'}
        rid = next(self.req_ids)
        req = dict(req, req_id=rid)
        ev = threading.Event()
        self.pending[rid] = [ev, None]
        self.nav_req_pub.publish(String(data=json.dumps(req)))
        ok = ev.wait(3.0)
        res = self.pending.pop(rid)[1]
        if not ok or res is None:
            return {'ok': False, 'error': 'navigator did not answer (is it running?)'}
        return res

    def save_map(self, name):
        with self.lock:
            m = self.map_msg
        if m is None:
            raise ValueError('no /map received yet')
        o = m.info.origin
        yaw = yaw_from_quaternion(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
        _, yaml_path = save_map(self.map_dir, name, m.info.width, m.info.height, m.info.resolution,
                                o.position.x, o.position.y, yaw, m.data)
        self.get_logger().info(f'Map saved: {yaml_path}')
        saved = yaml_path
        pb = self._write_pbstream(yaml_path[:-5] + '.pbstream')
        if pb:
            saved += f' + {os.path.basename(pb)}'
        else:
            saved += ' (WARNING: no .pbstream saved: Cartographer write_state unavailable, '
            saved += 'localization mode will not work with this map)'
        return saved

    def _write_pbstream(self, path):
        """Ask Cartographer to save its full state (needed for slam_mode:=localization)."""
        if not self.write_state_cli or not self.write_state_cli.service_is_ready():
            return None
        req = self._WriteState.Request()
        req.filename = path
        req.include_unfinished_submaps = True
        with self.lock:
            fut = self.write_state_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        if fut.done() and fut.result() is not None and fut.result().status.code == 0:
            self.get_logger().info(f'Cartographer state saved: {path}')
            return path
        self.get_logger().warn('Could not save .pbstream (Cartographer busy or not running)')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
