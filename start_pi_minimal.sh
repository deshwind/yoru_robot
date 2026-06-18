#!/usr/bin/env bash
# Pi-side start for camera + audio only (motors and lidar not needed).
# Run this on the Pi while your laptop runs ./start_server.sh.
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env
source install/setup.bash

PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "========================================"
echo "  Pi camera + audio ready"
echo "  Pi IP: ${PI_IP}"
echo "  Laptop: source ros_network.env then ./start_server.sh use_joystick:=false"
echo "========================================"
echo ""

exec ros2 launch compliance_bringup pi_camera_only.launch.py
