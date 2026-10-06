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
