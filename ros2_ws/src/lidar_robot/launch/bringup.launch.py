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
  lidar_port    /dev/ttyUSB0   (or /dev/rplidar after installing the udev rules)
  esp32_port    auto           (/dev/esp32, /dev/ttyACM* [S3], /dev/ttyUSB* except lidar_port [classic])
  laser_x/y/z/yaw              LiDAR pose on the robot (base_link -> laser)
  us_x / us_y / us_side_deg / us_z   ultrasonic mounts: centre at (us_x, 0), left/right at
                               (us_x, +-us_y) angled +-us_side_deg outwards, height us_z
  imu_x/imu_y/imu_z            MPU-6050 position (orientation: chip X forward, Z up)
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

    if slam and slam_mode == 'localization':
        if not map_file or not os.path.isfile(map_file):
            raise RuntimeError(f'slam_mode:=localization needs map:=<file>.pbstream (got "{map_file}"). '
                               'Save one from the dashboard ("Save map") while mapping.')
        if not use_odom:
            raise RuntimeError('localization mode is configured with wheel odometry; use use_odometry:=true')

    actions = [LogInfo(msg=f'[bringup] odom={use_odom} slam={slam} slam_mode={slam_mode} '
                           f'map={map_file or "-"} nav_mode={nav_mode} dashboard={cfg("dashboard")}')]

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
        parameters=[{'channel_type': 'serial', 'serial_port': cfg('lidar_port'),
                     'serial_baudrate': 115200, 'frame_id': 'laser', 'inverted': False,
                     'angle_compensate': True, 'scan_mode': 'Sensitivity'}]))

    # In LiDAR-only mode Cartographer owns odom->base_link, so the bridge must not publish it.
    actions.append(Node(
        package='lidar_robot', executable='esp32_bridge', name='esp32_bridge', output='screen',
        parameters=[params, {'serial_port': cfg('esp32_port'), 'publish_tf': use_odom,
                             'exclude_ports': cfg('lidar_port')}]))

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
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']))

    goal_frame = 'map' if slam else 'odom'
    if nav_mode == 'planner':
        actions.append(Node(
            package='lidar_robot', executable='navigator', name='navigator', output='screen',
            parameters=[params, {'global_frame': goal_frame, 'places_file': places,
                                 'require_localization_confirm': slam and slam_mode == 'localization'}]))
    elif nav_mode == 'direct':
        actions.append(Node(
            package='lidar_robot', executable='goal_controller', name='goal_controller',
            output='screen', parameters=[params, {'global_frame': goal_frame}]))

    if _truthy(cfg('dashboard')):
        actions.append(Node(
            package='lidar_robot', executable='dashboard', name='dashboard', output='screen',
            parameters=[params, {'global_frame': goal_frame, 'nav_mode': nav_mode}]))
    return actions


def generate_launch_description():
    args = [
        ('use_odometry', 'true'), ('slam', 'true'), ('slam_mode', 'mapping'), ('map', ''),
        ('nav_mode', 'planner'), ('dashboard', 'true'), ('places', '~/maps/places.json'),
        ('lidar_port', '/dev/ttyUSB0'), ('esp32_port', 'auto'),
        ('laser_x', '0.0'), ('laser_y', '0.0'), ('laser_z', '0.10'), ('laser_yaw', '0.0'),
        ('us_x', '0.22'), ('us_y', '0.12'), ('us_side_deg', '30'), ('us_z', '0.06'),
        ('imu_x', '0.0'), ('imu_y', '0.0'), ('imu_z', '0.05'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d) for n, d in args] + [OpaqueFunction(function=_setup)])
