#!/usr/bin/env bash
# Build a map with the real robot (drive it around with teleop).
#
#   ./start_mapping.sh
# then in a second terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
# and when the map looks complete:
#   ros2 run nav2_map_server map_saver_cli -f ~/dock_ws/maps/my_map
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
    colcon build --symlink-install
fi
source install/setup.bash

exec ros2 launch compliance_bringup real_mapping.launch.py "$@"
