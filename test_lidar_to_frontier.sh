#!/bin/bash
# Test LiDAR to Frontier Detection Pipeline

source /home/pi/FEA_SLAM_WS/install/setup.bash

echo "======================================"
echo "LiDAR to Frontier Detection Pipeline Test"
echo "======================================"
echo ""

echo "1. Checking if ROS2 nodes are running..."
ros2 node list 2>/dev/null | grep -E "ydlidar|scan_timestamp|slam_toolbox|frontier" || echo "   ❌ No nodes found"
echo ""

echo "2. Checking topic existence..."
echo "   /scan_raw (YDLidar output):"
ros2 topic info /scan_raw 2>&1 | grep -E "Type:|Publisher count:|Subscription count:" | sed 's/^/      /'
echo "   /scan (timestamp fixed):"
ros2 topic info /scan 2>&1 | grep -E "Type:|Publisher count:|Subscription count:" | sed 's/^/      /'
echo "   /map (SLAM output):"
ros2 topic info /map 2>&1 | grep -E "Type:|Publisher count:|Subscription count:" | sed 's/^/      /'
echo "   /frontiers (frontier detector output):"
ros2 topic info /frontiers 2>&1 | grep -E "Type:|Publisher count:|Subscription count:" | sed 's/^/      /'
echo ""

echo "3. Testing /scan_raw data rate (3 second sample)..."
timeout 3 ros2 topic hz /scan_raw 2>&1 | grep "average rate" | sed 's/^/   /' || echo "   ❌ No data on /scan_raw"
echo ""

echo "4. Testing /scan data rate (3 second sample)..."
timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" | sed 's/^/   /' || echo "   ❌ No data on /scan"
echo ""

echo "5. Checking /map updates (5 second sample)..."
timeout 5 ros2 topic hz /map 2>&1 | grep "average rate" | sed 's/^/   /' || echo "   ⚠️  Map updates slowly (normal)"
echo ""

echo "6. Testing frontier detection output..."
timeout 3 ros2 topic echo /frontiers --once 2>&1 | head -20 | sed 's/^/   /' || echo "   ❌ No frontiers published"
echo ""

echo "7. Checking for errors in logs..."
echo "   YDLidar errors:"
ros2 topic echo /rosout --once 2>&1 | grep -i "ydlidar.*error" | head -2 | sed 's/^/      /' || echo "      ✅ No YDLidar errors"
echo ""

echo "======================================"
echo "Pipeline Test Complete"
echo "======================================"
