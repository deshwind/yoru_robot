#!/usr/bin/env bash
# ONE COMMAND to start the full compliance robot simulation:
#   Gazebo (world with CCTV camera + walking person) + SLAM + Nav2 + RViz
#   + YOLO detection + tracking + event confirmation + coordinate transform
#   + Nav2 goal sender + escalation FSM + audio + incident logger + patrol.
#
# Usage:
#   ./start_sim.sh                       # default smoking scenario
#   ./start_sim.sh scenario:=vaping      # any extra args are passed to ros2 launch
#   ./start_sim.sh gui:=false rviz:=false
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash

# Build once if the workspace has not been built yet
if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace..."
    colcon build --symlink-install
fi
source install/setup.bash

exec ros2 launch compliance_bringup sim.launch.py "$@"
