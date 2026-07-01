#!/usr/bin/env bash
# Pi-side start: camera + audio only (motors and lidar not needed yet).
# The Pi hosts the FastDDS discovery server; start this BEFORE the laptop.
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env
source install/setup.bash

if ! ss -uln 2>/dev/null | grep -q ':11811 '; then
    setsid nohup fastdds discovery -i 0 -p 11811 \
        > /tmp/fastdds_discovery.log 2>&1 < /dev/null &
    echo "FastDDS discovery server started on this Pi (port 11811)"
    sleep 1
fi

PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "========================================"
echo "  Pi camera + audio ready"
echo "  Pi IP:    ${PI_IP}"
echo "  Discovery server: ${ROS_DISCOVERY_SERVER}"
echo "  Laptop:   git pull && ./start_server.sh"
echo "========================================"
echo ""

exec ros2 launch compliance_bringup pi_camera_only.launch.py
