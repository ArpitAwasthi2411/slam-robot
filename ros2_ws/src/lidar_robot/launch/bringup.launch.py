"""One launch file for the whole robot.

  # 1) build a map (drive around), then press "Save map" in the dashboard
  ros2 launch lidar_robot bringup.launch.py
  # 2) every later run: localise on the saved map -> saved places stay valid
  ros2 launch lidar_robot bringup.launch.py slam_mode:=localization map:=~/maps/cse_floor.pbstream

  ros2 launch lidar_robot bringup.launch.py use_odometry:=false   # LiDAR-only SLAM (no ESP32 odometry)
  ros2 launch lidar_robot bringup.launch.py slam:=false           # just drive: bridge + dashboard (+ lidar)
  ros2 launch lidar_robot bringup.launch.py nav_mode:=direct      # old straight-line goal controller

Arguments
  use_odometry  true        fuse ESP32 encoder odometry into Cartographer
  slam          true        start Cartographer + occupancy grid
  slam_mode     mapping     mapping | localization (needs map:=<file>.pbstream, odometry on)
  map           ''          saved Cartographer state for localization
  nav_mode      planner     planner (navigator: A* routes, places, missions) | direct (goal_controller) | none
  dashboard     true        web dashboard on http://<pi>:8080
  places        ~/maps/places.json
  lidar_port    auto           the USB-serial port that is NOT the ESP32 (or give a path, e.g. /dev/ttyUSB0)
  esp32_port    auto           the port that streams ODM lines (found at every start, any USB socket,
                               or the GPIO UART /dev/ttyAMA0 when /dev/esp32 points at it)
  laser_x/y/z/yaw              LiDAR pose on the robot (base_link -> laser)
  us_x / us_y / us_side_deg / us_z   ultrasonic mounts: centre at (us_x, 0), left/right at
                               (us_x, +-us_y) angled +-us_side_deg outwards, height us_z
  imu_x/imu_y/imu_z            MPU-6050 position (orientation: chip X forward, Z up)
  vision        true        camera people detection (vision node); the navigator slows / stops for people
  camera        auto        auto | libcamera (Camera Module 3) | /dev/videoN (USB webcam)
  urdf          true        robot model (urdf/pathik.urdf.xacro) + robot_state_publisher for the fixed frames;
                            false = the old static_transform_publishers (same frames)
  voice         true        push-to-talk voice commands (voice node: GPIO17 button / app, USB mic, Whisper/Vosk)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truthy(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    share = get_package_share_directory('lidar_robot')
    params = os.path.join(share, 'config', 'robot_params.yaml')
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731

    use_odom = _truthy(cfg('use_odometry'))
    slam = _truthy(cfg('slam'))
    slam_mode = cfg('slam_mode')
    map_file = os.path.expanduser(cfg('map'))
    nav_mode = cfg('nav_mode')
    places = os.path.expanduser(cfg('places'))
    vision = _truthy(cfg('vision'))
    voice = _truthy(cfg('voice'))

    if slam and slam_mode == 'localization':
        if not map_file or not os.path.isfile(map_file):
            raise RuntimeError(f'slam_mode:=localization needs map:=<file>.pbstream (got "{map_file}"). '
                               'Save one from the dashboard ("Save map") while mapping.')
        if not use_odom:
            raise RuntimeError('localization mode is configured with wheel odometry; use use_odometry:=true')

    # ---- serial ports: found by what each device sends, so USB sockets / ttyUSB numbers don't matter
    lidar_port, esp_port = cfg('lidar_port'), cfg('esp32_port')
    port_msg = ''
    if lidar_port == 'auto' or esp_port == 'auto':
        from lidar_robot.ports import detect_ports
        try:
            found_esp, found_lidar = detect_ports()
        except Exception as e:  # noqa: BLE001 - never block the launch on detection
            found_esp, found_lidar = None, None
            port_msg = f' (port detection failed: {e})'
        if esp_port == 'auto' and found_esp:
            esp_port = found_esp
        if lidar_port == 'auto':
            lidar_port = found_lidar or ('/dev/rplidar' if os.path.exists('/dev/rplidar') else '/dev/ttyUSB0')
        if not found_esp:
            port_msg += (' ESP32 not detected (no ODM data on any port): check its USB cable, or that no other '
                         'launch/serial_probe is holding the port. The bridge keeps searching.')
    actions_ports = [LogInfo(msg=f'[bringup] ports: ESP32={esp_port}  LiDAR={lidar_port}{port_msg}')]

    actions = actions_ports + [LogInfo(msg=f'[bringup] odom={use_odom} slam={slam} slam_mode={slam_mode} '
                           f'map={map_file or "-"} nav_mode={nav_mode} dashboard={cfg("dashboard")} '
                           f'vision={vision} camera={cfg("camera")} voice={voice}')]

    # fixed frames (laser, camera, imu_link, us_*, wheels): the URDF model + robot_state_publisher; same frame
    # names and offsets as the static publishers before (urdf:=false brings those back)
    model_args = {n: cfg(n) for n in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw', 'us_x', 'us_y', 'us_side_deg',
                                      'us_z', 'imu_x', 'imu_y', 'imu_z')}
    if _truthy(cfg('urdf')):
        import yaml
        from lidar_robot.urdf_model import launch_args_from_params, render
        with open(params) as f:
            model_args.update(launch_args_from_params(yaml.safe_load(f)))
        description = render(os.path.join(share, 'urdf', 'pathik.urdf.xacro'), model_args)
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name='robot_state_publisher', output='screen',
                            parameters=[{'robot_description': description, 'publish_frequency': 10.0}]))
    else:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
            arguments=['--x', cfg('laser_x'), '--y', cfg('laser_y'), '--z', cfg('laser_z'),
                       '--yaw', cfg('laser_yaw'), '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser']))

        sd = cfg('us_side_deg')
        import math as _m
        side = str(_m.radians(float(sd)))
        for name, y, yaw in (('us_center', '0.0', '0.0'), ('us_left', cfg('us_y'), side),
                             ('us_right', '-' + cfg('us_y'), '-' + side)):
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher', name=f'base_to_{name}',
                arguments=['--x', cfg('us_x'), '--y', y, '--z', cfg('us_z'), '--yaw', yaw,
                           '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', name]))
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_imu',
            arguments=['--x', cfg('imu_x'), '--y', cfg('imu_y'), '--z', cfg('imu_z'), '--yaw', '0',
                       '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']))

    actions.append(Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', output='screen',
        parameters=[{'channel_type': 'serial', 'serial_port': lidar_port,
                     'serial_baudrate': 115200, 'frame_id': 'laser', 'inverted': False,
                     'angle_compensate': True, 'scan_mode': 'Sensitivity'}]))

    # In LiDAR-only mode Cartographer owns odom->base_link, so the bridge must not publish it.
    actions.append(Node(
        package='lidar_robot', executable='esp32_bridge', name='esp32_bridge', output='screen',
        parameters=[params, {'serial_port': esp_port, 'publish_tf': use_odom,
                             'exclude_ports': lidar_port}]))

    if slam:
        if slam_mode == 'localization':
            lua = 'cartographer_localization.lua'
            extra = [f'-load_state_filename={map_file}', '-load_frozen_state=true']
        else:
            lua = 'cartographer_odom.lua' if use_odom else 'cartographer_lidar_only.lua'
            extra = []
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', os.path.join(share, 'config'),
                       '-configuration_basename', lua] + extra,
            remappings=[('scan', '/scan'), ('odom', '/odom')]))
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node', output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '2.0']))

    goal_frame = 'map' if slam else 'odom'
    if nav_mode == 'planner':
        actions.append(Node(
            package='lidar_robot', executable='navigator', name='navigator', output='screen',
            parameters=[params, {'global_frame': goal_frame, 'places_file': places,
                                 'require_localization_confirm': slam and slam_mode == 'localization',
                                 'use_vision': vision}]))
    elif nav_mode == 'direct':
        actions.append(Node(
            package='lidar_robot', executable='goal_controller', name='goal_controller',
            output='screen', parameters=[params, {'global_frame': goal_frame}]))

    if vision:
        actions.append(Node(
            package='lidar_robot', executable='vision', name='vision', output='screen',
            parameters=[params, {'camera': cfg('camera'), 'laser_yaw': float(cfg('laser_yaw'))}]))

    if voice:
        actions.append(Node(package='lidar_robot', executable='voice', name='voice', output='screen',
                            parameters=[params]))

    if _truthy(cfg('dashboard')):
        actions.append(Node(
            package='lidar_robot', executable='dashboard', name='dashboard', output='screen',
            parameters=[params, {'global_frame': goal_frame, 'nav_mode': nav_mode,
                                 'slam_mode': slam_mode if slam else 'off',
                                 'carto_config_dir': os.path.join(share, 'config')}]))
    return actions


def generate_launch_description():
    args = [
        ('use_odometry', 'true'), ('slam', 'true'), ('slam_mode', 'mapping'), ('map', ''),
        ('nav_mode', 'planner'), ('dashboard', 'true'), ('places', '~/maps/places.json'),
        ('lidar_port', 'auto'),
        ('esp32_port', 'auto'),
        ('laser_x', '-0.13'), ('laser_y', '0.0'), ('laser_z', '0.10'), ('laser_yaw', '0.0'),
        ('us_x', '0.07'), ('us_y', '0.12'), ('us_side_deg', '30'), ('us_z', '0.06'),
        ('imu_x', '0.0'), ('imu_y', '0.0'), ('imu_z', '0.05'),
        ('vision', 'true'), ('camera', 'auto'), ('voice', 'true'), ('urdf', 'true'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d) for n, d in args] + [OpaqueFunction(function=_setup)])
