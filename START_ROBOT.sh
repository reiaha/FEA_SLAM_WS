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

# Ensure this shell resolves packages from THIS workspace install overlay.
check_overlay_pkg_prefix() {
	local pkg="$1"
	local prefix
	prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
	if [[ -z "$prefix" ]]; then
		echo "[ERROR] Package '$pkg' not found after sourcing workspace overlay"
		exit 1
	fi
	if [[ "$prefix" != /home/pi/FEA_SLAM_WS/install/* ]]; then
		echo "[ERROR] Package '$pkg' resolves to stale overlay: $prefix"
		echo "[HINT] Open a fresh shell and run: source /home/pi/FEA_SLAM_WS/install/setup.bash"
		exit 1
	fi
	echo "[SOURCE] $pkg -> $prefix"
}

check_overlay_pkg_prefix localization
check_overlay_pkg_prefix fea_slam

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[ERROR] ros2 command not found after sourcing setup files"
	exit 1
fi

if [[ ! -e /dev/ttyACM0 ]]; then
	echo "[WARN] Arduino not found at /dev/ttyACM0 — motor bridge will retry on startup"
fi
if [[ ! -e /dev/ttyUSB0 ]]; then
	echo "[WARN] LiDAR not found at /dev/ttyUSB0 — ydlidar node will fail"
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

	# Remove readiness flag file
	rm -f "${READY_FLAG_FILE:-}" 2>/dev/null || true

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

	# rviz2 hangs on SIGINT - force-kill it before the generic sweep
	pkill -INT -x rviz2 2>/dev/null || true
	sleep 0.3
	pkill -KILL -x rviz2 2>/dev/null || true

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
	local timeout_sec="${HEALTHCHECK_TIMEOUT_SEC:-90}"
	local interval_sec=2
	local elapsed=0

	# ── Topics that MUST appear before the system is considered ready ──────────
	local -a REQUIRED_TOPICS=(
		/scan_raw
		/scan
		/map
		/odom
		/imu/data_raw
		/tf
		/cmd_vel
	)

	# ── Nodes that MUST be active ───────────────────────────────────────────────
	local -a REQUIRED_NODES=(
		/ydlidar_ros2_driver_node
		/ekf_node
		/async_slam_toolbox_node
		/controller_server
		/planner_server
		/bt_navigator
		/arduino_motor_bridge
	)

	# ── TF chains that MUST be resolvable (child → parent) ─────────────────────
	# Format: "child_frame parent_frame"
	local -a REQUIRED_TF=(
		"base_footprint odom"
		"odom map"
		"laser_frame base_footprint"
	)

	echo ""
	echo "[CHECK] ════════════════════════════════════════════════════"
	echo "[CHECK] System readiness check (timeout=${timeout_sec}s)"
	echo "[CHECK] Checking: ${#REQUIRED_TOPICS[@]} topics | ${#REQUIRED_NODES[@]} nodes | ${#REQUIRED_TF[@]} TF chains"
	echo "[CHECK] ════════════════════════════════════════════════════"

	# Track per-item ready state
	declare -A topic_ok node_ok tf_ok
	for t in "${REQUIRED_TOPICS[@]}"; do topic_ok[$t]=0; done
	for n in "${REQUIRED_NODES[@]}"; do node_ok[$n]=0; done
	for tf in "${REQUIRED_TF[@]}"; do tf_ok[$tf]=0; done

	local all_ready=0

	while [[ "$elapsed" -lt "$timeout_sec" ]]; do
		if [[ "${stop_requested:-0}" -eq 1 ]]; then
			echo "[CHECK] Interrupted."
			return 0
		fi

		# ── Check topics ─────────────────────────────────────────────────────
		local topics
		topics="$(ros2 topic list 2>/dev/null || true)"
		for t in "${REQUIRED_TOPICS[@]}"; do
			if [[ "${topic_ok[$t]}" -eq 0 ]] && grep -qF "$t" <<< "$topics"; then
				topic_ok[$t]=1
				echo "[CHECK] ✓ topic    $t"
			fi
		done

		# ── Check nodes ──────────────────────────────────────────────────────
		local nodes
		nodes="$(ros2 node list 2>/dev/null || true)"
		for n in "${REQUIRED_NODES[@]}"; do
			if [[ "${node_ok[$n]}" -eq 0 ]] && grep -qF "$n" <<< "$nodes"; then
				node_ok[$n]=1
				echo "[CHECK] ✓ node     $n"
			fi
		done

		# ── Check TF chains ──────────────────────────────────────────────────
		for tf in "${REQUIRED_TF[@]}"; do
			if [[ "${tf_ok[$tf]}" -eq 0 ]]; then
				local child parent
				child="${tf%% *}"
				parent="${tf##* }"
				if ros2 run tf2_ros tf2_echo "$parent" "$child" --timeout 0.5 2>/dev/null | grep -q 'Translation:'; then
					tf_ok[$tf]=1
					echo "[CHECK] ✓ tf       $child → $parent"
				fi
			fi
		done

		# ── Evaluate overall readiness ────────────────────────────────────────
		all_ready=1
		for t in "${REQUIRED_TOPICS[@]}"; do
			[[ "${topic_ok[$t]}" -eq 1 ]] || { all_ready=0; break; }
		done
		if [[ "$all_ready" -eq 1 ]]; then
			for n in "${REQUIRED_NODES[@]}"; do
				[[ "${node_ok[$n]}" -eq 1 ]] || { all_ready=0; break; }
			done
		fi
		if [[ "$all_ready" -eq 1 ]]; then
			for tf in "${REQUIRED_TF[@]}"; do
				[[ "${tf_ok[$tf]}" -eq 1 ]] || { all_ready=0; break; }
			done
		fi

		if [[ "$all_ready" -eq 1 ]]; then
			echo "[CHECK] ════════════════════════════════════════════════════"
			echo "[CHECK] ✓ ALL SYSTEMS READY — robot is fully operational"
			echo "[CHECK] ════════════════════════════════════════════════════"
			touch "${READY_FLAG_FILE}" 2>/dev/null || true
			SYSTEM_READY=1
			return 0
		fi

		sleep "$interval_sec"
		elapsed=$((elapsed + interval_sec))
	done

	# ── Timeout — report what is still missing ────────────────────────────────
	echo "[CHECK] ════════════════════════════════════════════════════"
	echo "[WARN]  Readiness timeout after ${timeout_sec}s — MISSING:"
	for t in "${REQUIRED_TOPICS[@]}"; do
		[[ "${topic_ok[$t]}" -eq 0 ]] && echo "[WARN]    topic  $t"
	done
	for n in "${REQUIRED_NODES[@]}"; do
		[[ "${node_ok[$n]}" -eq 0 ]] && echo "[WARN]    node   $n"
	done
	for tf in "${REQUIRED_TF[@]}"; do
		[[ "${tf_ok[$tf]}" -eq 0 ]] && echo "[WARN]    tf     $tf"
	done
	echo "[CHECK] ════════════════════════════════════════════════════"
	SYSTEM_READY=0
	return 0
}

# Run health check in background so launch remains foreground and interruptible

SMALL_TEST_MODE="${SMALL_TEST_MODE:-false}"
echo "[MODE] small_test_mode=${SMALL_TEST_MODE}"

# ENV_MODE: set to 'dynamic' for environments with chairs/containers (slower speed, wider safety margins)
#   Usage: ENV_MODE=dynamic ./START_ROBOT.sh
#   Default: static (open room, normal speed)
ENV_MODE="${ENV_MODE:-static}"
if [[ "$ENV_MODE" != "static" && "$ENV_MODE" != "dynamic" ]]; then
        echo "[ERROR] ENV_MODE must be 'static' or 'dynamic' (got: '$ENV_MODE')"
        exit 1
fi
echo "[MODE] env=${ENV_MODE}"

# Default OFF here: exploration_coordinator handles auto initial pose itself.
# Set AUTO_INITIAL_POSE=true only for explicit one-shot script-level (0,0,0) publish.
AUTO_INITIAL_POSE="${AUTO_INITIAL_POSE:-false}"
echo "[MODE] auto_initial_pose=${AUTO_INITIAL_POSE}"

SYSTEM_READY=0
READY_FLAG_FILE="/tmp/fea_slam_ready_$$"

# Run health check in background so launch remains foreground and interruptible

SMALL_TEST_MODE="${SMALL_TEST_MODE:-false}"
echo "[MODE] small_test_mode=${SMALL_TEST_MODE}"

# ENV_MODE: set to 'dynamic' for environments with chairs/containers (slower speed, wider safety margins)
#   Usage: ENV_MODE=dynamic ./START_ROBOT.sh
#   Default: static (open room, normal speed)
ENV_MODE="${ENV_MODE:-static}"
if [[ "$ENV_MODE" != "static" && "$ENV_MODE" != "dynamic" ]]; then
        echo "[ERROR] ENV_MODE must be 'static' or 'dynamic' (got: '$ENV_MODE')"
        exit 1
fi
echo "[MODE] env=${ENV_MODE}"

# Run launch in a dedicated session/process-group so Ctrl+C handler can reliably terminate it
setsid ros2 launch fea_slam robot_full.launch.py slam:=true exploration:=true rviz:=true map_odom_fallback:=false nav2_lifecycle_override:=false small_test_mode:=${SMALL_TEST_MODE} env:=${ENV_MODE} &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d '[:space:]' || true)"

# Run the readiness check in the background; it sets SYSTEM_READY=1 when done
post_launch_healthcheck &
HEALTHCHECK_PID=$!


# --- Wait for all systems to be ready before publishing initial pose ---
echo "[POSE] Waiting for system readiness check to complete..."
wait_ready_timeout=100
wait_ready_elapsed=0
while [[ "$wait_ready_elapsed" -lt "$wait_ready_timeout" ]]; do
	if [[ "${stop_requested:-0}" -eq 1 ]]; then
		echo "[STOP] Interrupted while waiting for system ready"
		exit 130
	fi
	# Check if the ready flag file was created by the healthcheck subprocess
	if [[ -f "${READY_FLAG_FILE}" ]]; then
		SYSTEM_READY=1
		echo "[POSE] System ready — proceeding to initial pose publication"
		break
	fi
	# Also check if healthcheck subprocess has exited (covers timeout/error paths)
	if ! kill -0 "$HEALTHCHECK_PID" 2>/dev/null; then
		[[ -f "${READY_FLAG_FILE}" ]] && SYSTEM_READY=1
		echo "[POSE] Readiness check finished (SYSTEM_READY=${SYSTEM_READY})"
		break
	fi
	sleep 1
	wait_ready_elapsed=$((wait_ready_elapsed + 1))
done
if [[ "$wait_ready_elapsed" -ge "$wait_ready_timeout" ]]; then
	echo "[POSE][WARN] Timed out waiting for system ready — attempting pose publish anyway"
fi

if [[ "${AUTO_INITIAL_POSE}" == "true" ]]; then
	echo "[POSE][WARN] Script-level /initialpose publish is disabled to avoid mid-run map re-anchoring."
	echo "[POSE][STATUS] initialized=deferred reason=handled_by_exploration_coordinator"
	AUTO_INITIAL_POSE="false"
fi

if [[ "${AUTO_INITIAL_POSE}" == "true" ]]; then
	# --- Wait for /initialpose topic to exist, then for subscriber, then publish initial pose ---
	echo "[POSE] AUTO_INITIAL_POSE=true: Waiting for /initialpose topic to be created..."
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
else
	echo "[POSE] AUTO_INITIAL_POSE=false: script-level pose publish disabled (exploration_coordinator auto-init handles initial pose)."
fi

wait $LAUNCH_PID || true
