from glob import glob
from setuptools import setup

package_name = 'lidar_robot'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    package_data={package_name: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/web', glob('lidar_robot/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arpit Awasthi',
    maintainer_email='awasthiarpit24@gmail.com',
    description='SLAM robot: ESP32 bridge, goal controller, web dashboard',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = lidar_robot.esp32_bridge:main',
            'esp32_odom_node = lidar_robot.esp32_bridge:main',   # old name, kept for muscle memory
            'goal_controller = lidar_robot.goal_controller:main',
            'dashboard = lidar_robot.dashboard:main',
        ],
    },
)
