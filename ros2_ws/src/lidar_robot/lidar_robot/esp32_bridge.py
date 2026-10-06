#!/usr/bin/env python3
"""ESP32 bridge: encoder odometry in, velocity commands out.

Subscribes
  /cmd_vel          geometry_msgs/Twist   autonomous source (goal controller, Nav2)
  /cmd_vel_teleop   geometry_msgs/Twist   dashboard / keyboard — has priority
  /estop            std_msgs/Bool         emergency stop. Latched INSIDE the ESP32 too (blocks RC)
                                          until released with data:false. Works from the CLI:
                                          ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
Publishes
  /odom             nav_msgs/Odometry
  TF odom->base_link                      only if publish_tf (i.e. Cartographer uses odometry)
  /robot/status     std_msgs/String       JSON, 2 Hz (used by the dashboard)
"""
import glob
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

import serial

from lidar_robot.kinematics import (DiffDriveOdometry, quaternion_from_yaw,
                                    twist_to_wheels, limit_wheels)
from lidar_robot.protocol import (parse_line, OdomPacket, InfoPacket, MODE_NAMES, MODE_ESTOP,
                                  format_velocity, format_stop, format_estop)

PORT_CANDIDATES = ['/dev/esp32', '/dev/ttyACM*']


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        p = self.declare_parameter
        p('serial_port', 'auto')
        p('baud_rate', 115200)
        p('wheel_radius', 0.0625)
        p('wheel_separation', 0.30)
        p('ticks_per_rev_left', 4740.0)
        p('ticks_per_rev_right', 4740.0)
        p('publish_tf', True)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('cmd_timeout', 0.5)
        p('max_wheel_speed', 0.45)      # m/s, hard clamp on what we ask the ESP32
        p('cmd_rate', 20.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.port_param = g('serial_port')
        self.baud = int(g('baud_rate'))
        self.publish_tf = bool(g('publish_tf'))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.cmd_timeout = float(g('cmd_timeout'))
        self.max_wheel = float(g('max_wheel_speed'))
        self.sep = float(g('wheel_separation'))
        self.odom = DiffDriveOdometry(float(g('wheel_radius')), self.sep,
                                      float(g('ticks_per_rev_left')),
                                      float(g('ticks_per_rev_right')))

        self.lock = threading.Lock()
        self.ser = None
        self.port = None
        self.running = True
        self.rx_count = 0
        self.rx_hz = 0.0
        self.bad_lines = 0
        self.last_rx = 0.0
        self.esp_mode = -1
        self.esp_info = ''
        self.cmd = {'teleop': (Twist(), 0.0), 'nav': (Twist(), 0.0)}
        self.cmd_source = 'none'
        self.cmd_out = (0.0, 0.0)
        self.zero_frames_left = 0
        self.estop = False
        self.estop_released_at = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.status_pub = self.create_publisher(String, 'robot/status', 5)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, 'cmd_vel', lambda m: self._on_cmd('nav', m), 10)
        self.create_subscription(Twist, 'cmd_vel_teleop', lambda m: self._on_cmd('teleop', m), 10)
        # Two subscriptions: TRANSIENT_LOCAL gets the dashboard's latched value even if we start
        # later; VOLATILE is compatible with plain `ros2 topic pub` from the command line.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        volatile = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, 'estop', self._on_estop, latched)
        self.create_subscription(Bool, 'estop', self._on_estop, volatile)

        self.create_timer(1.0 / float(g('cmd_rate')), self._send_cmd)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(2.0, self._health_check)

        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(
            f'esp32_bridge up: r={self.odom.wheel_radius} sep={self.sep} '
            f'ticks L/R={self.odom.ticks_per_rev_left}/{self.odom.ticks_per_rev_right} '
            f'publish_tf={self.publish_tf}')

    # ------------------------------------------------------------------ serial
    def _resolve_port(self):
        if self.port_param != 'auto':
            return self.port_param
        for pattern in PORT_CANDIDATES:
            hits = sorted(glob.glob(pattern))
            if hits:
                return hits[0]
        return None

    def _open(self):
        port = self._resolve_port()
        if not port:
            return False
        s = serial.Serial()
        s.port = port
        s.baudrate = self.baud
        s.timeout = 0.2
        s.write_timeout = 0.1
        # Do NOT toggle DTR/RTS: on the ESP32-S3 USB port that can reset the chip
        # or hold it in the bootloader -> silent port.
        s.dtr = False
        s.rts = False
        s.open()
        s.reset_input_buffer()
        with self.lock:
            self.ser = s
            self.port = port
        self.get_logger().info(f'Connected to ESP32 on {port} @ {self.baud}')
        # The boot banner was printed before we opened the port; ask the firmware to
        # identify itself so the log shows which version is flashed (v1 ignores this).
        self._write(b'?\n')
        return True

    def _close(self):
        with self.lock:
            s, self.ser = self.ser, None
        if s:
            try:
                s.close()
            except Exception:
                pass

    def _reader_loop(self):
        warned_missing = False
        while self.running:
            if self.ser is None:
                try:
                    if not self._open():
                        if not warned_missing:
                            self.get_logger().error(
                                'No ESP32 serial port found (tried /dev/esp32, /dev/ttyACM*). '
                                'Is the USB cable in? Retrying every 2 s.')
                            warned_missing = True
                        time.sleep(2.0)
                        continue
                    warned_missing = False
                except (serial.SerialException, OSError) as e:
                    self.get_logger().error(f'Cannot open ESP32 port: {e}', throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read failed ({e}); reconnecting')
                self._close()
                time.sleep(1.0)
                continue
            if not raw:
                continue
            pkt = parse_line(raw.decode('ascii', errors='ignore'))
            if pkt is None:
                self.bad_lines += 1
                continue
            if isinstance(pkt, InfoPacket):
                self.esp_info = pkt.text
                self.get_logger().info(f'ESP32: {pkt.text}')
                continue
            self._handle_odom(pkt)

    def _handle_odom(self, pkt: OdomPacket):
        now_t = time.monotonic()
        with self.lock:
            x, y, th, v, w = self.odom.update(pkt.dl, pkt.dr, pkt.dt_ms / 1000.0)
            self.rx_count += 1
            self.last_rx = now_t
            self.esp_mode = pkt.mode
        # ESP32 still latched from before a Pi/bridge restart -> show it, require explicit release
        if pkt.mode == MODE_ESTOP and not self.estop and now_t - self.estop_released_at > 1.0:
            self.estop = True
            self.get_logger().warn('ESP32 reports a latched E-STOP. Release it from the dashboard '
                                   '(or publish false on /estop).')
        if not self.running:
            return
        try:
            self._publish_odom(x, y, th, v, w)
        except Exception:            # context shutting down while the reader thread runs
            if self.running:
                raise

    def _publish_odom(self, x, y, th, v, w):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(th)

        if self.tf_pub:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_pub.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        o.pose.covariance[0] = o.pose.covariance[7] = 0.01
        o.pose.covariance[35] = 0.03
        for i in (14, 21, 28):            # z, roll, pitch: not estimated
            o.pose.covariance[i] = 1e6
        o.twist.covariance[0] = 0.01
        o.twist.covariance[35] = 0.03
        self.odom_pub.publish(o)

    def _write(self, data: bytes):
        with self.lock:
            s = self.ser
        if s is None:
            return
        try:
            s.write(data)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Serial write failed: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ commands
    def _on_cmd(self, source, msg: Twist):
        self.cmd[source] = (msg, time.monotonic())

    def _on_estop(self, msg: Bool):
        on = bool(msg.data)
        self._write(format_estop(on))          # idempotent on the ESP32, always (re)send
        if on == self.estop:
            return
        self.estop = on
        if not on:
            self.estop_released_at = time.monotonic()
        self.get_logger().warn(f'E-STOP {"ENGAGED (ESP32 latched, RC blocked)" if on else "released"}')

    def _send_cmd(self):
        now = time.monotonic()
        twist, source = None, 'none'
        if not self.estop:
            for src in ('teleop', 'nav'):            # priority order
                msg, t = self.cmd[src]
                if now - t < self.cmd_timeout:
                    twist, source = msg, src
                    break
        self.cmd_source = 'estop' if self.estop else source

        if self.estop:
            self._write(format_estop(True))    # keep the ESP32 latched even if it rebooted
        if twist is None:
            # send a few explicit zeros, then go quiet so the ESP32 drops to IDLE
            if self.zero_frames_left > 0:
                self._write(format_stop())
                self.zero_frames_left -= 1
            self.cmd_out = (0.0, 0.0)
            return

        vl, vr = twist_to_wheels(twist.linear.x, twist.angular.z, self.sep)
        vl, vr = limit_wheels(vl, vr, self.max_wheel)
        self.cmd_out = (twist.linear.x, twist.angular.z)
        self._write(format_velocity(vl, vr))
        self.zero_frames_left = 5

    # ------------------------------------------------------------------ health
    def _health_check(self):
        with self.lock:
            self.rx_hz = self.rx_count / 2.0
            self.rx_count = 0
            connected = self.ser is not None
            age = time.monotonic() - self.last_rx if self.last_rx else float('inf')
        if connected and age > 2.0:
            self.get_logger().warn(
                f'Port {self.port} is open but no ODM data for {age:.0f}s. Check: '
                '(1) firmware v2 flashed, (2) Arduino IDE "USB CDC On Boot" setting, '
                '(3) press the ESP32 RESET button, '
                '(4) run tools/serial_probe.py to see raw output.',
                throttle_duration_sec=10.0)

    def _publish_status(self):
        with self.lock:
            age = time.monotonic() - self.last_rx if self.last_rx else None
            st = {
                'connected': self.ser is not None,
                'port': self.port,
                'rx_hz': round(self.rx_hz, 1),
                'rx_age': None if age is None else round(age, 2),
                'esp_mode': MODE_NAMES.get(self.esp_mode, '?'),
                'esp_info': self.esp_info,
                'bad_lines': self.bad_lines,
                'cmd_source': self.cmd_source,
                'cmd_v': round(self.cmd_out[0], 3),
                'cmd_w': round(self.cmd_out[1], 3),
                'estop': self.estop,
                'odom': {'x': round(self.odom.x, 3), 'y': round(self.odom.y, 3),
                         'yaw': round(self.odom.theta, 3),
                         'v': round(self.odom.v, 3), 'w': round(self.odom.w, 3),
                         'turns': round(self.odom.theta_total / (2 * math.pi), 3)},
            }
        self.status_pub.publish(String(data=json.dumps(st)))

    def destroy_node(self):
        self.running = False
        self._write(format_stop())
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
