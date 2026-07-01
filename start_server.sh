#!/usr/bin/env bash
# MAIN SERVER (this PC) side of the distributed deployment.
# Runs: FastDDS discovery server + CCTV perception, FSM, dashboard, email.
# The robot runs ./start_pi_minimal.sh on the Pi.
#
# Usage:
#   ./start_server.sh
#   ./start_server.sh use_joystick:=false
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env

echo "========================================"
echo "  This laptop:  $(hostname).local  ($(hostname -I | awk '{print $1}'))"
echo "  Discovery server (on the Pi): ${ROS_DISCOVERY_SERVER}"
echo "  Start the Pi first: ./start_pi.sh (it hosts the discovery server)"
echo "========================================"

if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace..."
    colcon build --symlink-install
fi
source install/setup.bash

exec ros2 launch compliance_bringup server.launch.py "$@"
