#!/usr/bin/env bash
# Stream the Pi HQ Camera to a browser over WiFi.
# Open http://172.18.16.26:8080/stream?topic=/camera/image_raw on your laptop.
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source install/setup.bash
source ros_network.env

PI_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "========================================"
echo "  Camera stream ready — open on laptop:"
echo "  http://${PI_IP}:8080/stream?topic=/camera/image_raw"
echo "========================================"
echo ""

# Start camera node and web video server in parallel
ros2 run camera_ros camera_node --ros-args \
    -p width:=1280 -p height:=720 -p format:=RGB888 &
CAMERA_PID=$!

sleep 3

ros2 run web_video_server web_video_server \
    --ros-args -p port:=8080 -p address:=0.0.0.0 &
WEB_PID=$!

trap "kill $CAMERA_PID $WEB_PID 2>/dev/null" EXIT INT TERM
wait
