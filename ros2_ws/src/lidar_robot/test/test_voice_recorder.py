"""Recorder in voice_node: one empty recording must not block the next ones (27 Nov bug: after one phone tap
the robot's button only said "My microphone isn't working" until a restart)."""
import sys
import types


def _load(monkeypatch):
    # voice_node imports ROS; the Recorder itself does not need it (stubbed only where ROS is not installed)
    try:
        import rclpy.node  # noqa: F401
        import std_msgs.msg  # noqa: F401
    except ImportError:
        for name in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg'):
            monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
        sys.modules['rclpy.node'].Node = object
        sys.modules['std_msgs.msg'].String = object
        monkeypatch.delitem(sys.modules, 'lidar_robot.voice_node', raising=False)
    import lidar_robot.voice_node as vn
    return vn


class FakeProc:
    def __init__(self, path, audio):
        self.path, self.audio = path, audio

    def terminate(self):
        with open(self.path, 'wb') as f:
            f.write(self.audio)

    def communicate(self, timeout=None):
        return b'', b''

    def kill(self):
        pass


def test_empty_recording_does_not_block_the_next(monkeypatch):
    vn = _load(monkeypatch)
    monkeypatch.setattr(vn.shutil, 'which', lambda n: '/usr/bin/' + n)
    audio = {'next': b''}
    monkeypatch.setattr(vn.subprocess, 'Popen', lambda cmd, **kw: FakeProc(cmd[-1], audio['next']))
    rec = vn.Recorder('plughw:CARD=Device,DEV=0')
    assert rec.start()
    assert rec.stop() is None and rec.error            # the phone tap: no audio
    audio['next'] = b'RIFF' + b'\0' * 2000
    assert rec.start()                                 # used to return False here, forever
    assert rec.stop() is not None and rec.error is None


def test_missing_arecord_still_refuses(monkeypatch):
    vn = _load(monkeypatch)
    monkeypatch.setattr(vn.shutil, 'which', lambda n: None)
    rec = vn.Recorder()
    assert not rec.start() and 'arecord' in rec.error


def test_start_while_recording_ends_the_old_one(monkeypatch):
    vn = _load(monkeypatch)
    monkeypatch.setattr(vn.shutil, 'which', lambda n: '/usr/bin/' + n)
    monkeypatch.setattr(vn.subprocess, 'Popen', lambda cmd, **kw: FakeProc(cmd[-1], b'RIFF' + b'\0' * 2000))
    rec = vn.Recorder()
    assert rec.start()
    first = rec.proc
    assert rec.start() and rec.proc is not first
