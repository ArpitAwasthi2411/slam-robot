#!/usr/bin/env bash
# Install Nav2 (ROS 2 Humble) on the Pi for nav_mode:=nav2.   docs/nav2.md
# About 250 MB of packages. Run once, with internet:   bash scripts/setup_nav2.sh
set -euo pipefail
sudo apt-get update
sudo apt-get install -y ros-humble-navigation2
source /opt/ros/humble/setup.bash
for p in nav2_bt_navigator nav2_controller nav2_planner nav2_behaviors nav2_lifecycle_manager \
         nav2_regulated_pure_pursuit_controller nav2_navfn_planner; do
  ros2 pkg prefix "$p" >/dev/null && echo "ok  $p" || { echo "MISSING $p"; exit 1; }
done
echo
echo "Nav2 is installed. To use it, add  nav_mode:=nav2  to ROBOT_ARGS in ~/robot.env, then:"
echo "  sudo systemctl restart robot"
echo "To go back to the built-in planner: remove nav_mode:=nav2 (or set nav_mode:=planner) and restart."
