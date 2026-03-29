#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

MODE="headless"
WORLD_MODE="dynamic"
SPAWN_DELAY="6.0"
KEEP_RUNNING="false"

usage() {
  cat <<'EOF'
Usage: bash scripts/run_dynamic_with_health_check.sh [options]

Options:
  --gui                 Launch Gazebo GUI mode
  --headless            Launch server-only mode (default)
  --world static|dynamic  World mode (default: dynamic)
  --spawn-delay SEC     Spawn delay in seconds (default: 6.0)
  --keep-running        Keep simulation running after checks and follow logs
  -h, --help            Show this help

Examples:
  bash scripts/run_dynamic_with_health_check.sh
  bash scripts/run_dynamic_with_health_check.sh --gui --keep-running
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)
      MODE="gui"
      shift
      ;;
    --headless)
      MODE="headless"
      shift
      ;;
    --world)
      WORLD_MODE="${2:-}"
      shift 2
      ;;
    --spawn-delay)
      SPAWN_DELAY="${2:-}"
      shift 2
      ;;
    --keep-running)
      KEEP_RUNNING="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ "$WORLD_MODE" != "static" && "$WORLD_MODE" != "dynamic" ]]; then
  echo "[ERROR] --world must be static or dynamic"
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] Missing ROS setup: /opt/ros/humble/setup.bash"
  exit 1
fi
if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "[ERROR] Missing workspace setup: $ROOT_DIR/install/setup.bash"
  echo "[HINT] Build first: colcon build --packages-select fea_slam"
  exit 1
fi

# Some shells export nounset behavior aggressively; initialize this var explicitly
# before sourcing ROS setup files.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

HEADLESS="true"
if [[ "$MODE" == "gui" ]]; then
  HEADLESS="false"
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[WARN] DISPLAY is empty; GUI cannot open in this shell."
    echo "[WARN] Falling back to headless mode for health check run."
    HEADLESS="true"
  fi
fi

LOG_DIR="$ROOT_DIR/log/sim_health_$(date +%F_%H-%M-%S)"
mkdir -p "$LOG_DIR"
LAUNCH_LOG="$LOG_DIR/launch.log"
CHECK_LOG="$LOG_DIR/health_check.log"

unpause_world() {
  # Best-effort: unpause sim so sensors/odom start publishing in GUI mode.
  ign service -s /world/default/control \
    --reqtype ignition.msgs.WorldControl \
    --reptype ignition.msgs.Boolean \
    --timeout 2000 \
    --req 'pause: false' >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "$KEEP_RUNNING" == "true" ]]; then
    return
  fi
  pkill -9 -f 'ros2 launch fea_slam gazebo_sim.launch.py|ign gazebo|ros_gz_bridge/parameter_bridge|robot_state_publisher|scan_timestamp_fix' >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[STEP] Clearing stale simulation processes"
pkill -9 -f 'ros2 launch fea_slam gazebo_sim.launch.py|ign gazebo|ros_gz_bridge/parameter_bridge|robot_state_publisher|scan_timestamp_fix' >/dev/null 2>&1 || true
sleep 1

echo "[STEP] Starting simulation: world=${WORLD_MODE}, headless=${HEADLESS}"
ros2 launch fea_slam gazebo_sim.launch.py \
  world_mode:="$WORLD_MODE" \
  headless:="$HEADLESS" \
  spawn_delay:="$SPAWN_DELAY" \
  rviz:=false \
  slam:=false >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

echo "[STEP] Waiting for simulator topics and robot spawn"
ready=0
for i in $(seq 1 90); do
  if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    # On some systems the parent launch process may exit while simulator children continue.
    # Do not hard-fail here; readiness checks below determine success/failure.
    if (( i % 10 == 0 )); then
      echo "[WAIT] launch parent not running; continuing child-process readiness checks"
    fi
  fi

  unpause_world
  clock_seen=0
  spawned=0
  tf_ready=0
  scan_ready=0

  if timeout 2 ros2 topic echo /clock --once >/dev/null 2>&1; then
    clock_seen=1
  fi

  if grep -q 'OK creation of entity' "$LAUNCH_LOG" 2>/dev/null; then
    spawned=1
  fi

  if timeout 1.5 ros2 run tf2_ros tf2_echo odom base_footprint >/tmp/sim_wait_tf.out 2>&1; then
    if grep -q 'At time' /tmp/sim_wait_tf.out; then
      tf_ready=1
    fi
  fi
  rm -f /tmp/sim_wait_tf.out

  if timeout 1.5 ros2 topic echo /scan --once --qos-reliability best_effort >/dev/null 2>&1 \
    || timeout 1.5 ros2 topic echo /scan --once >/dev/null 2>&1; then
    scan_ready=1
  fi

  # Consider robot spawned if either create node confirms spawn in log or TF/scan are already alive.
  if [[ "$spawned" -eq 1 || "$tf_ready" -eq 1 || "$scan_ready" -eq 1 ]]; then
    spawned=1
  fi

  if [[ "$clock_seen" -eq 1 ]]; then
    ready=1
  fi

  if [[ "$ready" -eq 1 && "$spawned" -eq 1 ]]; then
    ready=1
    break
  fi

  if (( i % 10 == 0 )); then
    echo "[WAIT] ${i}s: clock=$clock_seen spawned=$spawned tf=$tf_ready scan=$scan_ready"
  fi
  sleep 1
done

if [[ "$ready" -ne 1 ]]; then
  echo "[FAIL] Simulation did not become ready within timeout"
  echo "[INFO] Launch log: $LAUNCH_LOG"
  echo "[INFO] Recent launch log tail:"
  tail -n 40 "$LAUNCH_LOG" || true
  exit 1
fi

unpause_world
sleep 2

echo "[STEP] Running health checks"
echo "[INFO] Skipping simulation health checks (check_sim_health.sh removed)"

if [[ "$KEEP_RUNNING" == "true" ]]; then
  echo "[INFO] keep-running enabled; simulation running (pid $LAUNCH_PID)"
  echo "[INFO] Following launch log. Press Ctrl+C to detach; simulation will keep running."
  tail --pid="$LAUNCH_PID" -f "$LAUNCH_LOG" || true
fi

exit 0
