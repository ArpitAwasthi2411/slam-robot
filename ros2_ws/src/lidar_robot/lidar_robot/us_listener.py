"""Shared ROS helper: listens to /ultrasonic/{left,center,right} and turns them into a front clearance
and obstacle points. Sensor positions come from TF (static transforms base_link -> us_* set by the
bringup launch args us_x / us_y / us_side_deg / us_z).
"""
import math
import time

from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from tf2_ros import TransformException

from lidar_robot.kinematics import yaw_from_quaternion
from lidar_robot.sensors import front_clearance_from_ranges, range_point

NAMES = ('left', 'center', 'right')


class UltrasonicListener:
    def __init__(self, node, tf_buffer, base_frame='base_link', max_age=0.5):
        self.node = node
        self.tf = tf_buffer
        self.base = base_frame
        self.max_age = max_age
        self.latest = {n: (None, 0.0, None) for n in NAMES}     # name -> (range|None, t, frame)
        self.mounts = {}
        for n in NAMES:
            node.create_subscription(Range, f'ultrasonic/{n}', lambda m, n=n: self._on_range(n, m),
                                     qos_profile_sensor_data)

    def _on_range(self, name, msg):
        r = msg.range
        ok = math.isfinite(r) and msg.min_range <= r <= msg.max_range
        self.latest[name] = (r if ok else None, time.monotonic(), msg.header.frame_id)

    def _mount(self, frame):
        if frame in self.mounts:
            return self.mounts[frame]
        try:
            t = self.tf.lookup_transform(self.base, frame, Time()).transform
        except TransformException:
            return None
        m = (t.translation.x, t.translation.y,
             yaw_from_quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        self.mounts[frame] = m
        return m

    def readings(self):
        now = time.monotonic()
        out = []
        for rng, t, frame in self.latest.values():
            if frame and now - t < self.max_age:
                out.append((rng, self._mount(frame)))
        return out

    def front_clearance(self, half_width):
        return front_clearance_from_ranges(self.readings(), half_width)

    def points_base(self):
        return [range_point(r, m) for r, m in self.readings() if r is not None and m is not None]
