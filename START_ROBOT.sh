#!/bin/bash
# FEA-SLAM Robot Startup Script
# This script starts the complete autonomous exploration system

cd /home/pi/FEA_SLAM_WS
source install/setup.bash

echo "========================================="
echo "FEA-SLAM Autonomous Exploration Starting"
echo "========================================="
echo ""
echo "Timeline:"
echo "  0-5s:   System initialization"
echo "  5-15s:  Nav2 activation"
echo "  10-20s: Exploration coordinator starts"
echo "  20s+:   MOTORS START MOVING!"
echo ""
echo "Watch for:"
echo "  ✅ Nav2 /navigate_to_pose is READY!"
echo "  ✅ Navigation server is READY! Starting frontier exploration"
echo "  🚀 Motor commands: MOTOR:XXX,XXX"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch fea_slam robot_full.launch.py exploration:=true
