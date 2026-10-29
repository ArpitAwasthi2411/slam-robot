"""The robot model (urdf/pathik.urdf.xacro) has the same frames and offsets as the old static TF publishers,
built from the same launch defaults and robot_params.yaml, so nothing downstream changes."""
import ast
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, PKG)
from lidar_robot.urdf_model import frames, launch_args_from_params, links, render, render_simple  # noqa: E402

XACRO = os.path.join(PKG, 'urdf', 'pathik.urdf.xacro')
LAUNCH = os.path.join(PKG, 'launch', 'bringup.launch.py')
PARAMS = os.path.join(PKG, 'config', 'robot_params.yaml')


def launch_defaults():
    """The ('name', 'default') pairs of generate_launch_description() in bringup.launch.py."""
    tree = ast.parse(open(LAUNCH).read())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'generate_launch_description')
    lst = next(n.value for n in ast.walk(fn) if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'args')
    return {e.elts[0].value: e.elts[1].value for e in lst.elts}


def params():
    yaml = pytest.importorskip('yaml')
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def model():
    a = launch_defaults()
    args = {k: a[k] for k in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw', 'us_x', 'us_y', 'us_side_deg', 'us_z',
                              'imu_x', 'imu_y', 'imu_z')}
    args.update(launch_args_from_params(params()))
    with open(XACRO) as f:
        return render_simple(f.read(), args), a, params()


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_same_frames_and_offsets_as_the_old_static_publishers():
    xml, a, p = model()
    f = frames(xml)
    f_ = float
    assert close(f['laser']['xyz'], (f_(a['laser_x']), f_(a['laser_y']), f_(a['laser_z'])))
    assert close(f['laser']['rpy'], (0, 0, f_(a['laser_yaw'])))
    side = math.radians(f_(a['us_side_deg']))
    assert close(f['us_center']['xyz'], (f_(a['us_x']), 0.0, f_(a['us_z']))) and close(f['us_center']['rpy'], (0, 0, 0))
    assert close(f['us_left']['xyz'], (f_(a['us_x']), f_(a['us_y']), f_(a['us_z']))) and close(f['us_left']['rpy'], (0, 0, side))
    assert close(f['us_right']['xyz'], (f_(a['us_x']), -f_(a['us_y']), f_(a['us_z']))) and close(f['us_right']['rpy'], (0, 0, -side))
    assert close(f['imu_link']['xyz'], (f_(a['imu_x']), f_(a['imu_y']), f_(a['imu_z'])))
    for name in ('laser', 'us_center', 'us_left', 'us_right', 'imu_link'):
        assert f[name]['parent'] == 'base_link' and f[name]['type'] == 'fixed'


def test_model_matches_the_parameters():
    xml, a, p = model()
    f = frames(xml)
    nav = p['navigator']['ros__parameters']
    br = p['esp32_bridge']['ros__parameters']
    # camera: the same mount the docking controller uses
    assert close(f['camera_link']['xyz'], (nav['camera_x'], nav['camera_y'], nav['camera_z']))
    assert close(f['camera_optical_frame']['rpy'], (-math.pi / 2, 0, -math.pi / 2))
    # wheels on the axle at +- separation / 2
    assert close(f['left_wheel_link']['xyz'], (0, br['wheel_separation'] / 2, 0))
    assert close(f['right_wheel_link']['xyz'], (0, -br['wheel_separation'] / 2, 0))
    # body = the navigator's footprint rectangle
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml.split('?>', 1)[1])
    base = next(ln for ln in root.findall('link') if ln.get('name') == 'base_link')
    size = [float(v) for v in base.find('visual/geometry/box').get('size').split()]
    assert close(size[:2], (nav['footprint_front'] + nav['footprint_rear'], 2 * nav['footprint_half_width']))
    cx = float(base.find('visual/origin').get('xyz').split()[0])
    assert abs(cx - (nav['footprint_front'] - nav['footprint_rear']) / 2) < 1e-6


def test_tree_is_rooted_at_base_link_and_names_match_the_nodes():
    xml, *_ = model()
    f = frames(xml)
    names = set(links(xml))
    assert {'base_link', 'laser', 'imu_link', 'us_left', 'us_center', 'us_right', 'camera_link'} <= names
    roots = names - set(f)
    assert roots == {'base_link'}
    for child, j in f.items():                       # every parent exists, no cycles
        seen, cur = set(), child
        while cur != 'base_link':
            assert cur not in seen
            seen.add(cur)
            cur = f[cur]['parent']
    src = open(os.path.join(PKG, 'lidar_robot', 'esp32_bridge.py')).read()
    assert "'imu_link'" in src and "f'us_{name}'" in src                # frames the bridge stamps
    assert "'frame_id': 'laser'" in open(LAUNCH).read()                 # the LiDAR driver's frame


def test_args_change_the_model_and_missing_args_are_reported():
    with open(XACRO) as f:
        text = f.read()
    f1 = frames(render_simple(text, {'laser_x': '-0.2', 'us_side_deg': '45'}))
    assert close(f1['laser']['xyz'][:1], (-0.2,)) and abs(f1['us_left']['rpy'][2] - math.pi / 4) < 1e-9
    with pytest.raises(ValueError):
        render_simple('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><link name="$(arg nope)"/></robot>')


def test_real_xacro_gives_the_same_frames():
    pytest.importorskip('xacro')
    xml, a, p = model()
    args = {k: a[k] for k in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw', 'us_x', 'us_y', 'us_side_deg', 'us_z',
                              'imu_x', 'imu_y', 'imu_z')}
    args.update(launch_args_from_params(p))
    real = frames(render(XACRO, args))
    mine = frames(xml)
    assert set(real) == set(mine)
    for k in real:
        assert close(real[k]['xyz'], mine[k]['xyz']) and close(real[k]['rpy'], mine[k]['rpy'])


def test_launch_uses_robot_state_publisher_and_keeps_the_static_fallback():
    src = open(LAUNCH).read()
    assert "executable='robot_state_publisher'" in src and "'robot_description'" in src
    assert "static_transform_publisher" in src and "cfg('urdf')" in src
    assert launch_defaults()['urdf'] == 'true'
    rviz = open(os.path.join(PKG, 'rviz', 'robot.rviz')).read()
    assert 'rviz_default_plugins/RobotModel' in rviz and '/robot_description' in rviz
