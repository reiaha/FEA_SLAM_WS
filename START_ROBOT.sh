#!/bin/bash
# FEA-SLAM Robot Startup Script
# This script starts the complete autonomous exploration system


set -eo pipefail

cd /home/pi/FEA_SLAM_WS


# --- Source the workspace after build ---
echo "[SOURCE] Sourcing ROS and workspace setup files..."

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

	for dev in /dev/ttyACM0 /dev/ttyACM1; do
		if [[ -w "$dev" ]]; then
			timeout 1 bash -lc "printf 'MOTOR:0,0\\nSTOP\\n' > '$dev'" >/dev/null 2>&1 || true
		fi
	done
}

cleanup_done=0
stop_requested=0
LAUNCH_MATCH='exploration_coordinator_simple|arduino_motor_bridge_simple|frontier_detector|scan_timestamp_fix|ydlidar_ros2_driver_node|async_slam_toolbox_node|sync_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|lifecycle_manager_navigation|lifecycle_manager_navigation_override|rviz2|joint_state_publisher|robot_state_publisher|static_transform_publisher|ekf_node|ros2 launch fea_slam robot_full.launch.py'

cleanup_all() {
	if [[ "$cleanup_done" -eq 1 ]]; then
		return
	fi
	cleanup_done=1

	cleanup_stop

	if [[ -n "${HEALTHCHECK_PID:-}" ]]; then
		kill "$HEALTHCHECK_PID" 2>/dev/null || true
	fi

	local launch_pgid
	local script_pgid
	launch_pgid="${LAUNCH_PGID:-}"
	script_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d '[:space:]' || true)"
	if [[ -z "$launch_pgid" && -n "${LAUNCH_PID:-}" ]]; then
		launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d '[:space:]' || true)"
	fi

	if [[ -n "$launch_pgid" && "$launch_pgid" != "$script_pgid" ]]; then
		kill -INT -- "-$launch_pgid" 2>/dev/null || true
	fi

	if [[ -n "${LAUNCH_PID:-}" ]]; then
		# Prefer graceful launch-managed shutdown first
		for _ in {1..10}; do
			if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
				break
			fi
			sleep 0.2
		done

		# Escalate only if still alive
		if kill -0 "$LAUNCH_PID" 2>/dev/null; then
			kill -TERM "$LAUNCH_PID" 2>/dev/null || true
			for _ in {1..10}; do
				if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
					break
				fi
				sleep 0.2
			done
		fi

		if kill -0 "$LAUNCH_PID" 2>/dev/null; then
			kill -KILL "$LAUNCH_PID" 2>/dev/null || true
		fi
	fi

	# Fallback cleanup for orphaned processes in the launch process group only
	if [[ -n "$launch_pgid" && "$launch_pgid" != "$script_pgid" ]]; then
		pkill -TERM -g "$launch_pgid" 2>/dev/null || true
		sleep 0.5
		pkill -KILL -g "$launch_pgid" 2>/dev/null || true
	fi

	# Final safety fallback: terminate any known stack nodes (including duplicates)
	pkill -TERM -f "$LAUNCH_MATCH" 2>/dev/null || true
	sleep 0.5
	pkill -KILL -f "$LAUNCH_MATCH" 2>/dev/null || true

	cleanup_stop
}

on_signal() {
	stop_requested=1
	echo "[STOP] Ctrl+C received. Stopping all ROS processes..."
	cleanup_all
	exit 130
}

trap on_signal INT TERM
trap cleanup_all EXIT

post_launch_healthcheck() {
	local timeout_sec="${HEALTHCHECK_TIMEOUT_SEC:-8}"
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

SMALL_TEST_MODE="${SMALL_TEST_MODE:-false}"
echo "[MODE] small_test_mode=${SMALL_TEST_MODE}"

post_launch_healthcheck &
HEALTHCHECK_PID=$!

# Run launch in a dedicated session/process-group so Ctrl+C handler can reliably terminate it
setsid ros2 launch fea_slam robot_full.launch.py slam:=true exploration:=true rviz:=true map_odom_fallback:=false nav2_lifecycle_override:=false small_test_mode:=${SMALL_TEST_MODE} &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d '[:space:]' || true)"



# --- Wait for /initialpose topic to exist, then for subscriber, then publish initial pose ---
echo "[POSE] Waiting for /initialpose topic to be created..."
sleep 3
topic_timeout=20
topic_elapsed=0
topic_interval=1
topic_exists=""
pose_published=0
while [[ $topic_elapsed -lt $topic_timeout ]]; do
	if [[ "$stop_requested" -eq 1 ]]; then
		echo "[STOP] Interrupted while waiting for /initialpose topic"
		exit 130
	fi
	topic_exists="$(ros2 topic list 2>/dev/null | grep -w "/initialpose" || true)"
	if [[ -n "$topic_exists" ]]; then
		echo "[POSE] /initialpose topic detected. Waiting for subscription..."
		break
	fi
	sleep $topic_interval
	topic_elapsed=$((topic_elapsed + topic_interval))
done
if [[ -z "$topic_exists" ]]; then
	echo "[POSE][ERROR] Timeout waiting for /initialpose topic. Initial pose not published."
        echo "[POSE][STATUS] initialized=false reason=topic_timeout"
else
	timeout=20
	interval=1
	elapsed=0
	subs_count=""
	while [[ $elapsed -lt $timeout ]]; do
		if [[ "$stop_requested" -eq 1 ]]; then
			echo "[STOP] Interrupted while waiting for /initialpose subscriber"
			exit 130
		fi
		subs_count="$(ros2 topic info /initialpose 2>/dev/null | awk '/Subscription count:/ {print $3}' || true)"
		if [[ -n "$subs_count" && "$subs_count" -gt 0 ]]; then
			echo "[POSE] /initialpose subscription detected. Publishing initial pose..."
			timeout 3 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}' >/dev/null 2>&1 || true
			pose_published=1
			echo "[POSE][STATUS] initialized=true reason=published_once"
			echo "[NOTE] Exploration will NOT start even if nav2 is ready until the initial pose is initialized."
			break
		fi
		sleep $interval
		elapsed=$((elapsed + interval))
	done
	if [[ $elapsed -ge $timeout ]]; then
		echo "[POSE][ERROR] Timeout waiting for /initialpose subscriber. Initial pose not published."
		echo "[POSE][STATUS] initialized=false reason=no_subscription"
	fi
fi

wait $LAUNCH_PID || true
