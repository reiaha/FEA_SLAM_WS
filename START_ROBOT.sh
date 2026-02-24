#!/bin/bash
# FEA-SLAM Robot Startup Script
# This script starts the complete autonomous exploration system

set -eo pipefail

cd /home/pi/FEA_SLAM_WS

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
	echo "[ERROR] Missing ROS setup: /opt/ros/humble/setup.bash"
	exit 1
fi

if [[ ! -f install/setup.bash ]]; then
	echo "[ERROR] Missing workspace setup: /home/pi/FEA_SLAM_WS/install/setup.bash"
	echo "[HINT] Build first: colcon build --packages-select localization fea_slam --symlink-install"
	exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[ERROR] ros2 command not found after sourcing setup files"
	exit 1
fi

if [[ ! -e /dev/ttyACM0 && ! -e /dev/ttyUSB0 ]]; then
	echo "[WARN] No Arduino/LiDAR serial device found at /dev/ttyACM0 or /dev/ttyUSB0"
	echo "[WARN] Launch will continue, but hardware nodes may fail/retry"
fi

cleanup_stop() {
	echo "[STOP] Sending zero velocity before shutdown..."
	timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
		'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
	timeout 2 ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
		'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
}

trap cleanup_stop INT TERM EXIT

post_launch_healthcheck() {
	local timeout_sec="${HEALTHCHECK_TIMEOUT_SEC:-20}"
	local interval_sec=1
	local elapsed=0
	local got_scan_raw=0
	local got_scan=0
	local got_map=0

	echo "[CHECK] Waiting up to ${timeout_sec}s for /scan_raw, /scan, /map ..."
	while [[ "$elapsed" -lt "$timeout_sec" ]]; do
		local topics
		topics="$(ros2 topic list 2>/dev/null || true)"
		if grep -q '^/scan_raw$' <<< "$topics"; then got_scan_raw=1; fi
		if grep -q '^/scan$' <<< "$topics"; then got_scan=1; fi
		if grep -q '^/map$' <<< "$topics"; then got_map=1; fi

		if [[ "$got_scan_raw" -eq 1 && "$got_scan" -eq 1 && "$got_map" -eq 1 ]]; then
			echo "[CHECK] OK: /scan_raw /scan /map detected"
			return 0
		fi

		sleep "$interval_sec"
		elapsed=$((elapsed + interval_sec))
	done

	echo "[WARN] Health check timeout after ${timeout_sec}s"
	echo "[WARN] /scan_raw=$got_scan_raw /scan=$got_scan /map=$got_map"
	return 0
}

# Run health check in background so launch remains foreground and interruptible
post_launch_healthcheck &

ros2 launch fea_slam robot_full.launch.py slam:=true exploration:=true map_odom_fallback:=false nav2_lifecycle_override:=false
