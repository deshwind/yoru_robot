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

# Start the FastDDS discovery server (lets Pi and laptop find each other
# reliably on Wi-Fi without depending on multicast).
pkill -f "fastdds discovery" 2>/dev/null || true
fastdds discovery -i 0 -p 11811 &
DISCOVERY_PID=$!
echo "FastDDS discovery server started (PID $DISCOVERY_PID)"
sleep 1

if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace..."
    colcon build --symlink-install
fi
source install/setup.bash

# Kill discovery server on exit
trap "kill $DISCOVERY_PID 2>/dev/null; true" EXIT

exec ros2 launch compliance_bringup server.launch.py "$@"
