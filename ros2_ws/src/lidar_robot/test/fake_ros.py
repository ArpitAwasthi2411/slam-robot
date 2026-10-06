"""Minimal, strict stand-in for the parts of rclpy / msgs / tf2_ros the nodes use.

Message classes use __slots__, so a misspelled field in node code raises AttributeError
here just like it would with real ROS messages. Only used by the integration tests
(the real robot uses real ROS 2 Humble).
"""
import sys
import time
import types


class _Msg:
    __slots__ = ()

    def __init__(self, **kw):
        for k in self.__slots__:
            default = getattr(type(self), '_defaults', {}).get(k, 0.0)
            setattr(self, k, default() if callable(default) else default)
        for k, v in kw.items():
            setattr(self, k, v)


class Vector3(_Msg):
    __slots__ = ('x', 'y', 'z')


class Point(Vector3):
    __slots__ = ()


class Quaternion(_Msg):
    __slots__ = ('x', 'y', 'z', 'w')
    _defaults = {'w': 1.0}


class Pose(_Msg):
    __slots__ = ('position', 'orientation')
    _defaults = {'position': Point, 'orientation': Quaternion}


class Transform(_Msg):
    __slots__ = ('translation', 'rotation')
    _defaults = {'translation': Vector3, 'rotation': Quaternion}


class Header(_Msg):
    __slots__ = ('stamp', 'frame_id')
    _defaults = {'stamp': None, 'frame_id': ''}


class Twist(_Msg):
    __slots__ = ('linear', 'angular')
    _defaults = {'linear': Vector3, 'angular': Vector3}


class TransformStamped(_Msg):
    __slots__ = ('header', 'child_frame_id', 'transform')
    _defaults = {'header': Header, 'child_frame_id': '', 'transform': Transform}


class PoseWithCovariance(_Msg):
    __slots__ = ('pose', 'covariance')
    _defaults = {'pose': Pose, 'covariance': lambda: [0.0] * 36}


class TwistWithCovariance(_Msg):
    __slots__ = ('twist', 'covariance')
    _defaults = {'twist': Twist, 'covariance': lambda: [0.0] * 36}


class Odometry(_Msg):
    __slots__ = ('header', 'child_frame_id', 'pose', 'twist')
    _defaults = {'header': Header, 'child_frame_id': '', 'pose': PoseWithCovariance,
                 'twist': TwistWithCovariance}


class Bool(_Msg):
    __slots__ = ('data',)
    _defaults = {'data': False}


class String(_Msg):
    __slots__ = ('data',)
    _defaults = {'data': ''}


# ---------------------------------------------------------------- rclpy
class _Logger:
    def __init__(self, sink):
        self.sink = sink

    def _log(self, level, msg, **kw):
        self.sink.append((level, msg))

    def info(self, m, **kw): self._log('INFO', m, **kw)
    def warn(self, m, **kw): self._log('WARN', m, **kw)
    def warning(self, m, **kw): self._log('WARN', m, **kw)
    def error(self, m, **kw): self._log('ERROR', m, **kw)
    def debug(self, m, **kw): self._log('DEBUG', m, **kw)


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: time.time(), nanoseconds=int(time.time() * 1e9))


class Node:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self.params = {}
        self.pubs, self.subs, self.timers, self.logs = {}, {}, [], []

    def declare_parameter(self, n, v):
        self.params[n] = _Param(self.overrides.get(n, v))

    def get_parameter(self, n):
        return self.params[n]

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _Pub(topic)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_logger(self):
        return _Logger(self.logs)

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


class TransformBroadcaster:
    def __init__(self, node):
        self.sent = []

    def sendTransform(self, t):
        self.sent.append(t)


def install():
    """Register the fake modules under the real import names."""
    m = lambda n: sys.modules.setdefault(n, types.ModuleType(n))  # noqa: E731
    rclpy = m('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    m('rclpy.node').Node = Node
    qos = m('rclpy.qos')
    qos.QoSProfile = lambda **kw: kw
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL='tl', VOLATILE='v')
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE='r', BEST_EFFORT='be')
    qos.qos_profile_sensor_data = 'sensor'
    gm = m('geometry_msgs.msg')
    gm.Twist, gm.TransformStamped, gm.Quaternion = Twist, TransformStamped, Quaternion
    m('geometry_msgs').msg = gm
    nm = m('nav_msgs.msg')
    nm.Odometry = Odometry
    m('nav_msgs').msg = nm
    sm = m('std_msgs.msg')
    sm.Bool, sm.String = Bool, String
    m('std_msgs').msg = sm
    m('tf2_ros').TransformBroadcaster = TransformBroadcaster
    rclpy.node = sys.modules['rclpy.node']
    rclpy.qos = qos
