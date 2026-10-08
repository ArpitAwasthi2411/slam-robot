#!/usr/bin/env python3
"""Navigator node: named places + A* path planning + path following + mission queue.

Replaces goal_controller when bringup runs with nav_mode:=planner (the default).

Subscribes  /map                  nav_msgs/OccupancyGrid
            /scan                 sensor_msgs/LaserScan   (obstacle stop + replanning around new obstacles)
            /goal_pose            geometry_msgs/PoseStamped (RViz 2D Goal Pose -> planned route)
            /goal_cancel          std_msgs/Empty
            /estop                std_msgs/Bool
            /navigator/request    std_msgs/String  JSON request (see navigator_core.py)
Publishes   /cmd_vel              geometry_msgs/Twist
            /plan                 nav_msgs/Path    current route (RViz: add Path display)
            /navigator/status     std_msgs/String  JSON, 4 Hz
            /navigator/places     std_msgs/String  JSON, latched, on change
            /navigator/response   std_msgs/String  JSON replies to requests ({"req_id": ...})

Optional: export GROQ_API_KEY=... before launching for LLM command parsing (needs internet).
Without it (or offline) the built-in rule parser is used.
"""
import hashlib
import json
import math
import os
import threading
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener, TransformException

from lidar_robot.commands import CommandParser
from lidar_robot.follower import FollowerParams
from lidar_robot.goal_logic import front_clearance_from_scan
from lidar_robot.kinematics import yaw_from_quaternion
from lidar_robot.navigator_core import NavigatorCore
from lidar_robot.places import PlaceStore
from lidar_robot.planner import GridMap, Planner, PlannerParams
from lidar_robot.sensors import to_world
from lidar_robot.us_listener import UltrasonicListener


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')
        d = self.declare_parameter
        d('global_frame', 'map')
        d('base_frame', 'base_link')
        d('places_file', os.path.expanduser('~/maps/places.json'))
        d('rate', 20.0)
        d('robot_radius', 0.27)
        d('soft_radius', 0.60)
        d('allow_unknown', False)
        d('max_linear', 0.22)
        d('max_angular', 0.6)
        d('lookahead', 0.50)
        d('k_angular', 1.0)
        d('linear_accel', 0.3)
        d('angular_accel', 1.2)
        d('rotate_in_place_above', 1.2)
        d('rotate_exit_below', 0.30)
        d('stop_distance', 0.38)
        d('slow_distance', 0.80)
        d('robot_half_width', 0.25)
        d('front_cone_deg', 25.0)
        d('dwell_s', 3.0)
        d('auto_return_s', 0.0)
        d('max_replans', 3)
        d('pose_timeout', 1.0)
        d('require_localization_confirm', False)   # bringup sets true in slam_mode:=localization
        d('llm_model', 'llama-3.1-8b-instant')
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.global_frame = g('global_frame')
        self.base_frame = g('base_frame')
        self.half_width = float(g('robot_half_width'))
        self.cone = math.radians(float(g('front_cone_deg')))
        self.pose_timeout = float(g('pose_timeout'))

        self.places = PlaceStore(g('places_file'))
        key = os.environ.get('GROQ_API_KEY')
        parser = CommandParser(self.places, groq_api_key=key, model=g('llm_model'))
        self.planner_params = PlannerParams(robot_radius=float(g('robot_radius')),
                                            soft_radius=float(g('soft_radius')),
                                            allow_unknown=bool(g('allow_unknown')))
        self.core = NavigatorCore(
            self.places,
            self.planner_params,
            FollowerParams(max_linear=float(g('max_linear')), max_angular=float(g('max_angular')),
                           lookahead=float(g('lookahead')), k_angular=float(g('k_angular')),
                           linear_accel=float(g('linear_accel')), angular_accel=float(g('angular_accel')),
                           rotate_in_place_above=float(g('rotate_in_place_above')),
                           rotate_exit_below=float(g('rotate_exit_below')),
                           stop_distance=float(g('stop_distance')), slow_distance=float(g('slow_distance'))),
            parser=parser, dwell_s=float(g('dwell_s')), auto_return_s=float(g('auto_return_s')),
            max_replans=int(g('max_replans')))
        self.core.localized = not bool(g('require_localization_confirm'))
        self.core_lock = threading.RLock()       # callbacks + map worker thread share the core
        self.topic_estop = False
        self.bridge_estop = False
        self.map_hash = None
        self.map_job = None                      # latest map waiting for the worker
        self.map_event = threading.Event()
        threading.Thread(target=self._map_worker, daemon=True).start()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.us = UltrasonicListener(self, self.tf_buffer, self.base_frame)   # low obstacles (v2.2)
        self.front = float('inf')
        self.scan_stamp = None
        self.laser_tf = None
        self.map_stamp = None
        self.places_version_sent = -1
        self.last_plan_sent = None
        self._was_active = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        volatile = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 1)
        self.status_pub = self.create_publisher(String, 'navigator/status', 5)
        self.places_pub = self.create_publisher(String, 'navigator/places', latched)
        self.resp_pub = self.create_publisher(String, 'navigator/response', 10)

        self.create_subscription(OccupancyGrid, 'map', self._on_map, 1)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(Empty, 'goal_cancel', self._on_cancel, 10)
        # latched + volatile: catch the dashboard's latched value AND plain `ros2 topic pub`
        self.create_subscription(Bool, 'estop', self._on_estop_topic, latched)
        self.create_subscription(Bool, 'estop', self._on_estop_topic, volatile)
        # the bridge also knows about e-stops latched inside the ESP32 (e.g. after a restart)
        self.create_subscription(String, 'robot/status', self._on_bridge_status, 10)
        self.create_subscription(String, 'navigator/request', self._on_request, 10)

        self.dt = 1.0 / float(g('rate'))
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.25, self._publish_status)
        self.get_logger().info(
            f'navigator ready: {len(self.places.places)} places from {g("places_file")}, '
            f'LLM {"ON (" + g("llm_model") + ")" if key else "OFF (offline rule parser)"}')

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg: OccupancyGrid):
        # Cartographer republishes /map every second even when nothing changed: skip those.
        i = msg.info
        o = i.origin.position
        raw = msg.data.tobytes() if hasattr(msg.data, 'tobytes') else bytes(b & 0xFF for b in msg.data)
        h = hashlib.blake2b(raw, digest_size=16)
        h.update(f'{i.width},{i.height},{i.resolution},{o.x:.3f},{o.y:.3f}'.encode())
        digest = h.digest()
        if digest == self.map_hash:
            return
        self.map_hash = digest
        self.map_job = (i.width, i.height, i.resolution, o.x, o.y, msg.data)
        self.map_event.set()

    def _map_worker(self):
        """Builds the costmap off the executor thread, then swaps it in (keeps the 20 Hz loop smooth)."""
        while rclpy.ok():
            if not self.map_event.wait(1.0):
                continue
            self.map_event.clear()
            job = self.map_job
            if job is None:
                continue
            try:
                planner = Planner(self.planner_params)
                planner.set_map(GridMap.from_occupancy(*job))
                with self.core_lock:
                    self.core.swap_planner(planner)
                self.map_stamp = self.get_clock().now()
            except Exception as e:
                self.get_logger().error(f'map processing failed: {e}')

    def _on_cancel(self, _msg):
        with self.core_lock:
            self.core.handle({'type': 'cancel'})

    def _apply_estop(self):
        on = self.topic_estop or self.bridge_estop
        with self.core_lock:
            if on != self.core.estopped:
                self.core.set_estop(on)

    def _on_estop_topic(self, msg: Bool):
        self.topic_estop = bool(msg.data)
        self._apply_estop()

    def _on_bridge_status(self, msg: String):
        try:
            self.bridge_estop = bool(json.loads(msg.data).get('estop', False))
        except (ValueError, AttributeError):
            return
        self._apply_estop()

    def _on_scan(self, msg: LaserScan):
        if self.laser_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                r = tf.transform.rotation
                self.laser_tf = (tf.transform.translation.x, tf.transform.translation.y,
                                 yaw_from_quaternion(r.x, r.y, r.z, r.w))
            except TransformException:
                return
        lx, ly, lyaw = self.laser_tf
        self.front = front_clearance_from_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_yaw=lyaw, half_angle=self.cone, robot_half_width=self.half_width, laser_x=lx)
        self.scan_stamp = self.get_clock().now()
        pose = self._pose()
        if pose:                                  # nearby scan points -> dynamic obstacles for replans
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            pts = []
            a = msg.angle_min
            for i, r in enumerate(msg.ranges):
                if i % 2 == 0 and msg.range_min < r < 3.0 and math.isfinite(r):
                    bx = lx + r * math.cos(a + lyaw)
                    by = ly + r * math.sin(a + lyaw)
                    pts.append((x + c * bx - s * by, y + s * bx + c * by))
                a += msg.angle_increment
            pts += to_world(self.us.points_base(), pose)       # things below the LiDAR plane
            with self.core_lock:
                self.core.update_scan_world(pts)

    def _on_goal(self, msg: PoseStamped):
        o = msg.pose.orientation
        with self.core_lock:
            self.core.handle({'type': 'goto_pose', 'x': msg.pose.position.x, 'y': msg.pose.position.y,
                              'yaw': yaw_from_quaternion(o.x, o.y, o.z, o.w), 'label': 'RViz goal',
                              'source': 'rviz'}, self._pose())

    def _on_request(self, msg: String):
        try:
            req = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(req, dict):
            return
        pose = self._pose()
        with self.core_lock:
            try:
                res = self.core.handle(req, pose)
            except Exception as e:                      # never let a request kill the node
                self.get_logger().error(f'request {req.get("type")} failed: {e}')
                res = {'ok': False, 'error': f'internal error: {e}'}
        res['req_id'] = req.get('req_id')
        self.resp_pub.publish(String(data=json.dumps(res)))
        self._publish_places()

    # ---------------------------------------------------------------- loop
    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.pose_timeout:
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y,
                yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def _tick(self):
        pose = self._pose()
        front = self.front
        if self.scan_stamp is None or (self.get_clock().now() - self.scan_stamp).nanoseconds > 1e9:
            front = 0.0                                  # no fresh scan: never drive forward
        front = min(front, self.us.front_clearance(self.half_width))
        try:
            with self.core_lock:
                v, w = self.core.step(pose, front, self.dt)
                active = bool(self.core.queue.active)
        except Exception as e:                           # stop, report, keep the node alive
            self.get_logger().error(f'navigator step failed: {e}\n{traceback.format_exc()}')
            self.cmd_pub.publish(Twist())
            with self.core_lock:
                self.core._fail('INTERNAL', f'internal error: {e}')
            return
        if active or v or w or self._was_active:       # keep streaming while busy, one stop after
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.cmd_pub.publish(cmd)
        self._was_active = active

    def _publish_status(self):
        with self.core_lock:
            st = self.core.status()
        self.status_pub.publish(String(data=json.dumps(st)))
        key = tuple(map(tuple, st['path'][:3])) + (len(st['path']),)
        if key != self.last_plan_sent:
            path = Path()
            path.header.frame_id = self.global_frame
            path.header.stamp = self.get_clock().now().to_msg()
            for x, y in st['path']:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            self.plan_pub.publish(path)
            self.last_plan_sent = key
        self._publish_places()

    def _publish_places(self):
        if self.places.version != self.places_version_sent:
            self.places_pub.publish(String(data=json.dumps(self.places.summary())))
            self.places_version_sent = self.places.version


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
