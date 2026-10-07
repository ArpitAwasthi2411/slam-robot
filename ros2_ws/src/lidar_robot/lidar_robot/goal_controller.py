#!/usr/bin/env python3
"""Goal controller node: RViz '2D Goal Pose' (or the dashboard) -> robot drives there.

Subscribes  /goal_pose    geometry_msgs/PoseStamped   (RViz 2D Goal Pose tool publishes here)
            /goal_cancel  std_msgs/Empty
            /scan         sensor_msgs/LaserScan        obstacle stop
            /estop        std_msgs/Bool                cancels the goal
Publishes   /cmd_vel                  geometry_msgs/Twist
            /goal_controller/plan     nav_msgs/Path      straight line, for RViz
            /goal_controller/status   std_msgs/String    JSON
Pose comes from TF: <global_frame> -> base_link (map when SLAM runs).
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener, TransformException

from lidar_robot.goal_logic import (GoalController, GoalParams, front_clearance_from_scan,
                                    IDLE, ACTIVE_STATES)
from lidar_robot.kinematics import yaw_from_quaternion
from lidar_robot.us_listener import UltrasonicListener


class GoalControllerNode(Node):
    def __init__(self):
        super().__init__('goal_controller')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('robot_half_width', 0.20)
        self.declare_parameter('front_cone_deg', 25.0)
        self.declare_parameter('pose_timeout', 0.6)        # s: older TF pose -> stop
        self.declare_parameter('pose_lost_abort', 3.0)     # s: stale this long -> abort goal
        defaults = GoalParams()
        for name, val in vars(defaults).items():
            self.declare_parameter(name, val)
        params = GoalParams(**{k: self.get_parameter(k).value for k in vars(defaults)})

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.half_width = float(self.get_parameter('robot_half_width').value)
        self.cone = math.radians(float(self.get_parameter('front_cone_deg').value))
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.pose_lost_abort = float(self.get_parameter('pose_lost_abort').value)
        self.pose_age = None
        self.stale_for = 0.0
        self.ctrl = GoalController(params)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.us = UltrasonicListener(self, self.tf_buffer, self.base_frame)
        self.front = float('inf')
        self.scan_stamp = None
        self.laser_tf = None          # (x, yaw) of laser in base frame
        self.pose = None
        self.last_state = IDLE

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'goal_controller/plan', 1)
        self.status_pub = self.create_publisher(String, 'goal_controller/status', 5)

        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(Empty, 'goal_cancel', lambda _: self._cancel('canceled by user'), 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        # VOLATILE+RELIABLE is compatible with both latched and plain publishers
        self.create_subscription(Bool, 'estop', self._on_estop,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                            reliability=ReliabilityPolicy.RELIABLE))

        self.dt = 1.0 / float(self.get_parameter('rate').value)
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(f'goal_controller ready (frame={self.global_frame}). '
                               'Use RViz "2D Goal Pose" or the dashboard.')

    # ---------------------------------------------------------------- inputs
    def _on_goal(self, msg: PoseStamped):
        frame = msg.header.frame_id or self.global_frame
        pose = msg.pose
        if frame != self.global_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.global_frame, frame, Time(),
                                                     timeout=Duration(seconds=0.2))
                yaw_t = yaw_from_quaternion(*_q(tf.transform.rotation))
                tx, ty = tf.transform.translation.x, tf.transform.translation.y
                px, py = pose.position.x, pose.position.y
                gx = tx + math.cos(yaw_t) * px - math.sin(yaw_t) * py
                gy = ty + math.sin(yaw_t) * px + math.cos(yaw_t) * py
                gyaw = yaw_t + yaw_from_quaternion(*_q(pose.orientation))
            except TransformException as e:
                self.get_logger().error(f'Goal in frame "{frame}" cannot be converted: {e}')
                return
        else:
            gx, gy = pose.position.x, pose.position.y
            gyaw = yaw_from_quaternion(*_q(pose.orientation))
        self.ctrl.set_goal(gx, gy, gyaw)
        self.get_logger().info(f'New {self.ctrl.message}')
        self._publish_plan()

    def _on_scan(self, msg: LaserScan):
        if self.laser_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                self.laser_tf = (tf.transform.translation.x,
                                 yaw_from_quaternion(*_q(tf.transform.rotation)))
            except TransformException:
                return
        lx, lyaw = self.laser_tf
        self.front = front_clearance_from_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            laser_yaw=lyaw, half_angle=self.cone, robot_half_width=self.half_width, laser_x=lx)
        self.scan_stamp = self.get_clock().now()

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._cancel('e-stop')

    def _cancel(self, why):
        if self.ctrl.active:
            self.get_logger().info(f'Goal canceled: {why}')
        self.ctrl.cancel(why)
        self.cmd_pub.publish(Twist())

    # ---------------------------------------------------------------- loop
    def _lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'No TF {self.global_frame}->{self.base_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return None
        # lookup with Time() returns the LATEST transform, however old: if Cartographer or the
        # bridge stalls this pose freezes, so track its age and refuse to drive on it.
        self.pose_age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_from_quaternion(*_q(t.rotation)))

    def _tick(self):
        self.pose = self._lookup_pose()
        if not self.ctrl.active:
            if self.last_state in ACTIVE_STATES:
                self.cmd_pub.publish(Twist())       # one final stop
            self.last_state = self.ctrl.state
            return
        stale = self.pose is None or self.pose_age is None or self.pose_age > self.pose_timeout
        if stale:
            self.stale_for += self.dt
            self.ctrl._v = self.ctrl._w = 0.0         # restart the speed ramp from zero later
            self.cmd_pub.publish(Twist())
            self.ctrl.message = (f'pose stale ({self.pose_age:.1f}s) - waiting for SLAM'
                                 if self.pose_age is not None else 'no pose (TF) - waiting')
            self.get_logger().warn(self.ctrl.message, throttle_duration_sec=2.0)
            if self.stale_for > self.pose_lost_abort:
                self._cancel('pose lost (is Cartographer running?)')
                self.ctrl.state = 'ABORTED'
            return
        self.stale_for = 0.0
        front = self.front
        if self.scan_stamp is None or \
                (self.get_clock().now() - self.scan_stamp).nanoseconds > 1e9:
            front = 0.0                             # no fresh scan -> don't drive forward
        front = min(front, self.us.front_clearance(self.half_width))
        v, w = self.ctrl.step(self.pose, front, self.dt)
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        if self.ctrl.state != self.last_state:
            self.get_logger().info(f'{self.last_state} -> {self.ctrl.state} {self.ctrl.message}')
            self.last_state = self.ctrl.state

    def _publish_plan(self):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        pts = []
        if self.pose:
            pts.append(self.pose[:2])
        pts.append(self.ctrl.goal[:2])
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)

    def _publish_status(self):
        g = self.ctrl.goal
        d = self.ctrl.distance_to_goal(self.pose) if self.pose else None
        st = {
            'state': self.ctrl.state,
            'message': self.ctrl.message,
            'blocked': self.ctrl.blocked,
            'goal': None if g is None else {'x': g[0], 'y': g[1], 'yaw': g[2]},
            'distance': None if d is None else round(d, 3),
            'front_clearance': None if not math.isfinite(self.front) else round(self.front, 2),
            'pose_age': None if self.pose_age is None else round(self.pose_age, 2),
            'frame': self.global_frame,
        }
        self.status_pub.publish(String(data=json.dumps(st)))


def _q(q):
    return q.x, q.y, q.z, q.w


def main(args=None):
    rclpy.init(args=args)
    node = GoalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.cmd_pub.publish(Twist())   # the bridge/ESP32 watchdogs stop it anyway
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
