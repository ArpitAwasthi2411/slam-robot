import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lidar_robot.ports import pick_esp32_port  # noqa: E402


def fake_fs(devices, links=None):
    links = links or {}

    def g(pattern):
        import fnmatch
        return [d for d in devices if fnmatch.fnmatch(d, pattern)]
    return g, (lambda p: links.get(p, p))


def test_udev_symlink_wins():
    g, rp = fake_fs(['/dev/esp32', '/dev/ttyACM0', '/dev/ttyUSB0'], {'/dev/esp32': '/dev/ttyUSB1'})
    assert pick_esp32_port(['/dev/ttyUSB0'], g, rp) == ('/dev/esp32', False)


def test_s3_native_usb():
    g, rp = fake_fs(['/dev/ttyACM0', '/dev/ttyUSB0'])
    assert pick_esp32_port(['/dev/ttyUSB0'], g, rp) == ('/dev/ttyACM0', False)


def test_classic_skips_lidar_port():
    g, rp = fake_fs(['/dev/ttyUSB0', '/dev/ttyUSB1'])
    assert pick_esp32_port(['/dev/ttyUSB0'], g, rp) == ('/dev/ttyUSB1', True)
    assert pick_esp32_port(['/dev/ttyUSB1'], g, rp) == ('/dev/ttyUSB0', True)


def test_lidar_given_as_symlink_is_resolved():
    g, rp = fake_fs(['/dev/rplidar', '/dev/ttyUSB0', '/dev/ttyUSB1'], {'/dev/rplidar': '/dev/ttyUSB1'})
    assert pick_esp32_port(['/dev/rplidar'], g, rp) == ('/dev/ttyUSB0', True)


def test_only_lidar_present_means_no_esp32():
    g, rp = fake_fs(['/dev/ttyUSB0'])
    assert pick_esp32_port(['/dev/ttyUSB0'], g, rp) == (None, False)


class _FakePort:
    def __init__(self, data):
        self.data = data

    def read(self, n):
        d, self.data = self.data[:n], self.data[n:]
        return d

    def close(self):
        pass


def _clock():
    t = [0.0]

    def clock():
        return t[0]

    def sleep(s):
        t[0] += s
    return clock, sleep


def test_detect_ports_finds_esp32_by_its_data_whatever_the_socket():
    from lidar_robot.ports import detect_ports
    clock, sleep = _clock()
    data = {'/dev/ttyUSB0': b'', '/dev/ttyUSB1': b'INFO,READY\nODM,1,2,3\n'}
    esp, lidar = detect_ports(['/dev/ttyUSB0', '/dev/ttyUSB1'], lambda p: _FakePort(data[p]), clock=clock, sleep=sleep)
    assert (esp, lidar) == ('/dev/ttyUSB1', '/dev/ttyUSB0')
    data = {'/dev/ttyUSB0': b'ODM,1,2,3\n', '/dev/ttyUSB1': b''}          # swapped sockets
    esp, lidar = detect_ports(['/dev/ttyUSB0', '/dev/ttyUSB1'], lambda p: _FakePort(data[p]), clock=clock, sleep=sleep)
    assert (esp, lidar) == ('/dev/ttyUSB0', '/dev/ttyUSB1')


def test_detect_ports_when_esp32_is_silent_or_busy():
    from lidar_robot.ports import detect_ports
    clock, sleep = _clock()

    def busy(p):
        if p == '/dev/ttyUSB1':
            raise OSError('busy')
        return _FakePort(b'')
    esp, lidar = detect_ports(['/dev/ttyUSB0', '/dev/ttyUSB1'], busy, clock=clock, sleep=sleep)
    assert esp is None and lidar == '/dev/ttyUSB0'
