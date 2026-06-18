#!/usr/bin/env bash
# ONE COMMAND to start the real robot (Raspberry Pi 4 + RPLIDAR + L298N).
# Requires a saved map (see start_mapping.sh) and calibrated CCTV homography
# (src/compliance_core/tools/calibrate_homography.py).
#
# Usage:
#   ./start_robot.sh
#   ./start_robot.sh map:=/home/desh/dock_ws/maps/my_map.yaml
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace..."
    colcon build --symlink-install
fi
source install/setup.bash

exec ros2 launch compliance_bringup real_robot.launch.py "$@"
