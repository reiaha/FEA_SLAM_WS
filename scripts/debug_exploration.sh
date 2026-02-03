#!/bin/bash
# Debug exploration - check if nodes are working correctly

echo "═══════════════════════════════════════════════════════════"
echo "  EXPLORATION SYSTEM DIAGNOSTICS"
echo "═══════════════════════════════════════════════════════════"
echo ""

source /home/pi/FEA_SLAM_WS/install/setup.bash

echo "1️⃣  Checking if nodes are running..."
echo "───────────────────────────────────────────────────────────"
ros2 node list | grep -E "frontier_detector|exploration_coordinator|lidar_explorer|slam_toolbox"
echo ""

echo "2️⃣  Checking frontier detection..."
echo "───────────────────────────────────────────────────────────"
timeout 3 ros2 topic echo /frontiers --once 2>/dev/null | head -10 || echo "No /frontiers topic yet"
echo ""

echo "3️⃣  Checking map publishing..."
echo "───────────────────────────────────────────────────────────"
timeout 2 ros2 topic hz /map 2>&1 | head -3 || echo "Map topic not publishing"
echo ""

echo "4️⃣  Checking current goals..."
echo "───────────────────────────────────────────────────────────"
timeout 2 ros2 topic echo /navigate_to_pose/_action/status --once 2>/dev/null | head -5 || echo "No navigation goals"
echo ""

echo "5️⃣  Checking for errors in nodes..."
echo "───────────────────────────────────────────────────────────"
echo "Recent log messages (warnings/errors only):"
find /home/pi/.ros/log -name "*.log" -type f -mmin -5 -exec grep -h "ERROR\|WARN" {} \; 2>/dev/null | tail -5 || echo "No recent errors"
echo ""

echo "═══════════════════════════════════════════════════════════"
