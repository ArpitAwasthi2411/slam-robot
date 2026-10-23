#!/usr/bin/env bash
# Container entrypoint.
#   robot              (default) start everything: LiDAR, ESP32 bridge, Cartographer, navigator, app server
#   users <args>       manage accounts, e.g.  users list   |   users reset-admin arpit 'new password'
#   version            print the image version
#   anything else      run it as a command inside the ROS environment (e.g. bash, ros2 topic list)
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/pathik_ws/install/setup.bash

case "${1:-robot}" in
  robot)
    echo "[pathik] version ${PATHIK_VERSION:-dev}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  args: ${ROBOT_ARGS:-<none>}"
    # shellcheck disable=SC2086  (ROBOT_ARGS is a list of launch arguments)
    exec ros2 launch lidar_robot bringup.launch.py ${ROBOT_ARGS}
    ;;
  users)
    shift
    exec python3 -m lidar_robot.auth /root/maps/users.json "$@"
    ;;
  version)
    echo "${PATHIK_VERSION:-dev}"
    ;;
  *)
    exec "$@"
    ;;
esac
