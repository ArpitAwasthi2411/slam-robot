"""One launch file for the whole robot.

  ros2 launch lidar_robot bringup.launch.py                       # SLAM + encoders + controller + dashboard
  ros2 launch lidar_robot bringup.launch.py use_odometry:=false   # LiDAR-only SLAM (ESP32 not needed for mapping)
  ros2 launch lidar_robot bringup.launch.py slam:=false           # just drive: bridge + dashboard (+ lidar)

Arguments
  use_odometry  true   fuse ESP32 encoder odometry into Cartographer
  slam          true   start Cartographer + occupancy grid
  controller    true   start the go-to-goal controller (RViz 2D Goal Pose)
  dashboard     true   web dashboard on http://<pi>:8080
  lidar_port    /dev/ttyUSB0   (or /dev/rplidar after installing the udev rules)
  esp32_port    auto           (/dev/esp32, then /dev/ttyACM*)
  laser_x/y/z/yaw              LiDAR pose on the robot (base_link -> laser)
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
    actions = [LogInfo(msg=f'[bringup] use_odometry={use_odom} slam={cfg("slam")} '
                           f'controller={cfg("controller")} dashboard={cfg("dashboard")}')]

    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
        arguments=['--x', cfg('laser_x'), '--y', cfg('laser_y'), '--z', cfg('laser_z'),
                   '--yaw', cfg('laser_yaw'), '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser']))

    actions.append(Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', output='screen',
        parameters=[{'channel_type': 'serial', 'serial_port': cfg('lidar_port'),
                     'serial_baudrate': 115200, 'frame_id': 'laser', 'inverted': False,
                     'angle_compensate': True, 'scan_mode': 'Sensitivity'}]))

    # In LiDAR-only mode Cartographer owns odom->base_link, so the bridge must not publish it.
    actions.append(Node(
        package='lidar_robot', executable='esp32_bridge', name='esp32_bridge', output='screen',
        parameters=[params, {'serial_port': cfg('esp32_port'), 'publish_tf': use_odom}]))

    if _truthy(cfg('slam')):
        lua = 'cartographer_odom.lua' if use_odom else 'cartographer_lidar_only.lua'
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', os.path.join(share, 'config'),
                       '-configuration_basename', lua],
            remappings=[('scan', '/scan'), ('odom', '/odom')]))
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node', output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']))

    goal_frame = 'map' if _truthy(cfg('slam')) else 'odom'
    if _truthy(cfg('controller')):
        actions.append(Node(
            package='lidar_robot', executable='goal_controller', name='goal_controller',
            output='screen', parameters=[params, {'global_frame': goal_frame}]))

    if _truthy(cfg('dashboard')):
        actions.append(Node(
            package='lidar_robot', executable='dashboard', name='dashboard', output='screen',
            parameters=[params, {'global_frame': goal_frame}]))
    return actions


def generate_launch_description():
    args = [
        ('use_odometry', 'true'), ('slam', 'true'), ('controller', 'true'), ('dashboard', 'true'),
        ('lidar_port', '/dev/ttyUSB0'), ('esp32_port', 'auto'),
        ('laser_x', '0.0'), ('laser_y', '0.0'), ('laser_z', '0.10'), ('laser_yaw', '0.0'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d) for n, d in args] + [OpaqueFunction(function=_setup)])
