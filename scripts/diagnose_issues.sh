#!/bin/bash
# Comprehensive Issue Diagnostics

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║          FEA-SLAM SYSTEM DIAGNOSTICS                      ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

source /home/pi/FEA_SLAM_WS/install/setup.bash

echo "1️⃣  CHECKING NODE STATUS"
echo "────────────────────────────────────────────────────────────"
ros2 node list 2>/dev/null | grep -E "slam|frontier|exploration|lidar" || echo "⚠️  Key nodes missing!"
echo ""

echo "2️⃣  CHECKING TOPIC RATES"
echo "────────────────────────────────────────────────────────────"
echo "Map update rate:"
timeout 5 ros2 topic hz /map 2>&1 | grep "average" || echo "⚠️  /map not publishing or slow"
echo ""
echo "Frontier update rate:"
timeout 5 ros2 topic hz /frontiers 2>&1 | grep "average" || echo "⚠️  /frontiers not publishing"
echo ""
echo "Scan rate:"
timeout 5 ros2 topic hz /scan 2>&1 | grep "average" || echo "⚠️  /scan not publishing"
echo ""

echo "3️⃣  CHECKING EXPLORATION COORDINATOR LOGS"
echo "────────────────────────────────────────────────────────────"
echo "Looking for recent exploration_coordinator activity..."
timeout 3 ros2 topic echo /rosout --once 2>/dev/null | grep -A2 "exploration_coordinator" | head -10 || echo "⚠️  No exploration_coordinator logs"
echo ""

echo "4️⃣  CHECKING NAV2 STATUS"
echo "────────────────────────────────────────────────────────────"
ros2 action list 2>/dev/null | grep navigate_to_pose && echo "✅ Nav2 action server running" || echo "⚠️  Nav2 action server NOT running"
echo ""

echo "5️⃣  CHECKING TF TREE"
echo "────────────────────────────────────────────────────────────"
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5 || echo "⚠️  TF map→base_link broken"
echo ""

echo "6️⃣  CHECKING CURRENT FRONTIER DATA"
echo "────────────────────────────────────────────────────────────"
timeout 3 ros2 topic echo /frontiers --once 2>/dev/null | grep -E "markers:|position:" | head -5 || echo "⚠️  No frontier data"
echo ""

echo "═══════════════════════════════════════════════════════════"
echo "DIAGNOSIS COMPLETE"
echo "═══════════════════════════════════════════════════════════"
