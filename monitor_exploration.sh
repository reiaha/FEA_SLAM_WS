#!/bin/bash
# Monitor exploration system status
echo "========================================="
echo "EXPLORATION SYSTEM MONITOR"
echo "========================================="
echo ""
echo "This will check the system status in 70 seconds..."
echo "DO NOT PRESS CTRL+C - Let it complete!"
echo ""
echo "Starting system..."
sleep 70

echo ""
echo "Checking Nav2 status..."
source /home/pi/FEA_SLAM_WS/install/setup.bash
ros2 action list | grep navigate_to_pose

echo ""
echo "Checking if cmd_vel_nav is publishing..."
timeout 2 ros2 topic hz /cmd_vel_nav 2>&1 | head -5

echo ""
echo "Checking motor bridge output (last 20 lines)..."
ros2 topic echo /cmd_vel_nav --once 2>&1

echo ""
echo "========================================="
echo "Monitor complete - check output above"
echo "========================================="
