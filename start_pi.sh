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

exec ros2 launch compliance_bringup pi_hardware.launch.py "$@"
