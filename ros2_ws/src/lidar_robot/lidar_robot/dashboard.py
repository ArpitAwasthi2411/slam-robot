#!/usr/bin/env python3
"""Web teleop dashboard node. Open http://<pi-ip>:8080 from the laptop or a phone.

Publishes   /cmd_vel_teleop (Twist)  /estop (Bool, latched)  /goal_pose  /goal_cancel
Subscribes  /map  /scan  /robot/status  /goal_controller/status  + TF map->base_link
Map saving writes ~/maps/<name>.pgm/.yaml (map_server format) directly from /map.
"""
import base64
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
from lidar_robot.kinematics import yaw_from_quaternion, quaternion_from_yaw, wrap_angle
from lidar_robot.mapsave import save_map


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
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.global_frame = gp('global_frame')
        self.fallback_frame = gp('fallback_frame')
        self.base_frame = gp('base_frame')
        self.max_v = float(gp('max_linear'))
        self.max_w = float(gp('max_angular'))
        self.map_dir = gp('map_dir')
        self.scan_dec = max(1, int(gp('scan_decimation')))

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', latched)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, 'goal_cancel', 10)
        # NOTE: never publish /estop at startup: a restarted dashboard must not release an e-stop

        # volatile+reliable matches both cartographer (volatile) and map_server (latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, 1)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, 'robot/status', self._on_robot_status, 10)
        self.create_subscription(String, 'goal_controller/status', self._on_goal_status, 10)

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

    def _on_goal_status(self, msg):
        try:
            self.goal_status = json.loads(msg.data)
        except ValueError:
            pass

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
        }

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
        return yaml_path


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
