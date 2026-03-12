#!/bin/bash
cd /home/pi/FEA_SLAM_WS

set -e

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
	echo "[ERROR] Missing ROS setup: /opt/ros/humble/setup.bash"
	exit 1
fi

if [[ ! -f /home/pi/FEA_SLAM_WS/install/setup.bash ]]; then
	echo "[ERROR] Missing workspace setup: /home/pi/FEA_SLAM_WS/install/setup.bash"
	echo "[HINT] Build first: colcon build --packages-select localization fea_slam"
	exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# Verify we launch with the freshly built workspace overlay.
for pkg in localization fea_slam; do
	prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
	if [[ -z "$prefix" ]]; then
		echo "[ERROR] Package '$pkg' not found after sourcing setup files"
		exit 1
	fi
	if [[ "$prefix" != /home/pi/FEA_SLAM_WS/install/* ]]; then
		echo "[ERROR] Package '$pkg' resolves to stale overlay: $prefix"
		echo "[HINT] Open a fresh shell and run: source /home/pi/FEA_SLAM_WS/install/setup.bash"
		exit 1
	fi
	echo "[SOURCE] $pkg -> $prefix"
done

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
