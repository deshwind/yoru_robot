#!/usr/bin/env bash
# MAIN SERVER (this PC) side of the distributed deployment.
# Runs: CCTV perception, FSM, dashboard (auto-opens), email, admin joystick.
# The robot itself runs ./start_pi.sh (deployed with ./deploy_to_pi.sh).
#
# Usage:
#   ./start_server.sh
#   ./start_server.sh use_cctv2:=true        # second CCTV camera configured
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env
if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace..."
    colcon build --symlink-install
fi
source install/setup.bash

exec ros2 launch compliance_bringup server.launch.py "$@"
