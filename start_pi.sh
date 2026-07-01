#!/usr/bin/env bash
# RASPBERRY PI (robot) side of the distributed deployment - run ON the Pi.
# Runs: motors, lidar, Pi camera, audio, localization + Nav2 onboard.
# The main PC runs ./start_server.sh.
#
# Usage (on the Pi):
#   ./start_pi.sh                              # localization with saved map
#   ./start_pi.sh mode:=mapping                # build a map first
#   ./start_pi.sh camera:=usb                  # USB webcam instead of Pi cam
#   ./start_pi.sh map:=/home/desh/dock_ws/maps/my_map.yaml
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
source ./ros_network.env
if [ ! -f install/setup.bash ]; then
    echo "First run: building workspace (this takes a while on the Pi)..."
    colcon build --symlink-install
fi
source install/setup.bash

# The Pi hosts the FastDDS discovery server (the laptop connects to it).
# Reuse an already-running instance so restarts don't orphan the graph.
# Check the PORT, not the process name (pgrep -f can match unrelated shells).
if ! ss -uln 2>/dev/null | grep -q ':11811 '; then
    setsid nohup fastdds discovery -i 0 -p 11811 \
        > /tmp/fastdds_discovery.log 2>&1 < /dev/null &
    echo "FastDDS discovery server started on this Pi (port 11811)"
    sleep 1
else
    echo "FastDDS discovery server already running"
fi

PI_IP=$(hostname -I | awk '{print $1}')
echo "Pi IP: ${PI_IP}  |  laptop: git pull && ./start_server.sh"

exec ros2 launch compliance_bringup pi_hardware.launch.py "$@"
