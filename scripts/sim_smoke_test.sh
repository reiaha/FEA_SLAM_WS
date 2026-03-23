#!/usr/bin/env bash
set -euo pipefail

# Software-in-the-loop smoke test for FEA_SLAM using rosbag playback.
# Usage:
#   scripts/sim_smoke_test.sh /absolute/or/relative/path/to/bag_dir
# Example:
#   scripts/sim_smoke_test.sh bags/smoke_2026-03-19_18-55-00

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${1:-}"

if [[ -z "${BAG_DIR}" ]]; then
  echo "Usage: $0 <bag_dir>"
  echo "Example: $0 bags/smoke_2026-03-19_18-55-00"
  exit 2
fi

if [[ ! -d "${ROOT_DIR}/${BAG_DIR}" && ! -d "${BAG_DIR}" ]]; then
  echo "[FAIL] Bag directory not found: ${BAG_DIR}"
  exit 2
fi

if [[ -d "${ROOT_DIR}/${BAG_DIR}" ]]; then
  BAG_PATH="${ROOT_DIR}/${BAG_DIR}"
else
  BAG_PATH="${BAG_DIR}"
fi

LOG_DIR="${ROOT_DIR}/log/smoke_test_$(date +%F_%H-%M-%S)"
mkdir -p "${LOG_DIR}"
LAUNCH_LOG="${LOG_DIR}/launch.log"
PLAY_LOG="${LOG_DIR}/bag_play.log"
METRICS_LOG="${LOG_DIR}/metrics.log"

cleanup() {
  set +e
  if [[ -n "${PLAY_PID:-}" ]]; then
    kill "${PLAY_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
  fi
  pkill -f "robot_full.launch.py use_sim_time:=true" 2>/dev/null || true
  pkill -f "ros2 bag play ${BAG_PATH}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "${ROOT_DIR}"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"

echo "[INFO] Starting robot_full in sim replay mode..."
ros2 launch fea_slam robot_full.launch.py \
  use_sim_time:=true \
  lidar:=false \
  arduino_bridge:=false \
  slam:=true \
  rviz:=false >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

sleep 8

echo "[INFO] Playing bag: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock >"${PLAY_LOG}" 2>&1 &
PLAY_PID=$!

# Wait until bag playback exits.
wait "${PLAY_PID}"

# Give nodes a moment to flush logs.
sleep 2

# Capture rate snapshots (best-effort)
{
  echo "=== TOPIC RATES ==="
  timeout 6 ros2 topic hz /scan 2>&1 | tail -n 6 || true
  timeout 6 ros2 topic hz /odom 2>&1 | tail -n 6 || true
  timeout 6 ros2 topic hz /map 2>&1 | tail -n 6 || true
} >"${METRICS_LOG}" 2>&1

# Failure signatures to catch regressions.
SIG_EKF="Failed to meet update rate"
SIG_IMU="IMU sign mismatch"
SIG_TF="Lookup would require extrapolation"
SIG_SCAN_STALE="Scan stale"

fail_count=0

check_sig() {
  local sig="$1"
  local label="$2"
  if grep -q "$sig" "${LAUNCH_LOG}"; then
    echo "[FAIL] ${label}: found '${sig}'"
    fail_count=$((fail_count + 1))
  else
    echo "[PASS] ${label}: no '${sig}'"
  fi
}

echo ""
echo "========== SMOKE TEST SUMMARY =========="
check_sig "${SIG_EKF}" "EKF timing"
check_sig "${SIG_IMU}" "IMU sign agreement"
check_sig "${SIG_TF}" "TF timestamp health"
check_sig "${SIG_SCAN_STALE}" "Scan freshness"

echo "[INFO] Logs: ${LOG_DIR}"
echo "[INFO] Launch log: ${LAUNCH_LOG}"
echo "[INFO] Bag play log: ${PLAY_LOG}"
echo "[INFO] Metrics log: ${METRICS_LOG}"

if [[ ${fail_count} -eq 0 ]]; then
  echo "[PASS] Smoke test passed"
  exit 0
else
  echo "[FAIL] Smoke test failed (${fail_count} issue(s))"
  exit 1
fi
