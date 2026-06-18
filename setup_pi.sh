#!/usr/bin/env bash
# One-time Raspberry Pi 4 provisioning - run ON the Pi (Ubuntu 22.04 Server
# arm64, ROS 2 Humble already installed per docs.ros.org).
#
# Installs everything the robot side needs. The Pi does NOT need ultralytics
# or YOLO - perception runs on the main server.
set -e

echo ">>> apt packages (ROS drivers + tools) ..."
sudo apt update
sudo apt install -y \
    ros-humble-rplidar-ros \
    ros-humble-camera-ros \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    python3-pip python3-opencv python3-numpy \
    espeak-ng alsa-utils \
    joystick bluetooth bluez

echo ">>> Python packages (GPIO for the L298N driver) ..."
pip3 install --user RPi.GPIO

echo ">>> Permissions: serial (RPLIDAR), GPIO, video (camera), audio ..."
sudo usermod -aG dialout,video,audio,gpio "$USER" 2>/dev/null || \
sudo usermod -aG dialout,video,audio "$USER"

echo ">>> Pi Camera Module on Ubuntu 22.04:"
echo "    1. sudo nano /boot/firmware/config.txt   and ensure these lines:"
echo "         camera_auto_detect=1"
echo "         dtoverlay=imx219        # imx219 = Camera v2; ov5647 = v1; imx708 = v3"
echo "    2. reboot, then test:  ros2 run camera_ros camera_node"
echo ""
echo ">>> Bluetooth joystick (optional, if pairing with the Pi instead of the PC):"
echo "    bluetoothctl -> scan on -> hold SHARE+PS -> pair/trust/connect <MAC>"
echo ""
echo ">>> Done. Log out and back in (group changes), then from the PC run:"
echo "    ./deploy_to_pi.sh <this-pi-ip> $USER"
