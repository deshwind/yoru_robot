#!/usr/bin/env bash
# Pi-side start: camera + audio only (motors and lidar not needed yet).
# Run AFTER start_server.sh is already running on the laptop — the Pi
# connects to the laptop's FastDDS discovery server to find all topics.
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env
source install/setup.bash

PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "========================================"
echo "  Pi camera + audio ready"
echo "  Pi IP:    ${PI_IP}"
echo "  Discovery server: ${ROS_DISCOVERY_SERVER}"
echo "  Laptop:   run ./start_server.sh use_joystick:=false"
echo "========================================"
echo ""

exec ros2 launch compliance_bringup pi_camera_only.launch.py
