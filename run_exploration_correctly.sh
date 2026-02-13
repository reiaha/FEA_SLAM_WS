#!/bin/bash
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

echo "╔══════════════════════════════════════════════════════════╗"
echo "║          AUTONOMOUS EXPLORATION STARTUP                  ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "⚠️  CRITICAL: Do NOT press Ctrl+C for at least 90 seconds!"
echo "    System needs 70+ seconds to fully initialize Nav2"
echo ""
echo "What you should see:"
echo "  1. [0-30s]  SLAM and sensors initializing"
echo "  2. [30-60s] Nav2 lifecycle nodes configuring"  
echo "  3. [60-70s] Nav2 activating and creating bonds"
echo "  4. [70s+]   Exploration starts, motors should move"
echo ""
echo "If motors don't move after 90 seconds, then press Ctrl+C and report"
echo ""
echo "Press ENTER to start..."
read

ros2 launch fea_slam robot_full.launch.py exploration:=true rviz:=false
