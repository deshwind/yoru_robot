#!/usr/bin/env bash
# Connect-check ONLY: verify the link to the Raspberry Pi. Launches no
# hardware, drives nothing - just confirms the Pi is reachable and ready.
#
# Usage:
#   ./connect_pi.sh [pi-ip] [user]
cd "$(dirname "$0")"

PI_IP="${1:-172.18.16.26}"     # default: this robot's Pi
PI_USER="${2:-desh}"
DEST="$PI_USER@$PI_IP"

echo "=== Compliance robot: Raspberry Pi connection check ==="
echo "Target: $DEST"
echo

echo "1) Network reachable (ping)..."
if ping -c 2 -W 2 "$PI_IP" >/dev/null 2>&1; then
    echo "   OK - Pi is on the network"
else
    echo "   FAIL - no reply. Check the Pi is powered on and on the same Wi-Fi."
    exit 1
fi

echo "2) SSH service (port 22)..."
if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PI_IP/22" 2>/dev/null; then
    echo "   OK - SSH port open"
else
    echo "   FAIL - SSH refused. On the Pi (keyboard/monitor) run:"
    echo "          sudo apt install -y openssh-server && sudo systemctl enable --now ssh"
    exit 1
fi

echo "3) SSH login + identity..."
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$DEST" \
       'echo "   OK - logged in: $(hostname) ($(uname -m)), ROS $(. /opt/ros/humble/setup.bash 2>/dev/null; echo $ROS_DISTRO)"' 2>/dev/null; then
    :
else
    echo "   FAIL - could not log in without a password."
    echo "          Set up a key once:  ssh-copy-id $DEST"
    exit 1
fi

echo
echo "All good. Next:  ./deploy_to_pi.sh $PI_IP $PI_USER"
