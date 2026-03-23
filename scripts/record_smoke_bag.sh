#!/usr/bin/env bash
set -euo pipefail

# Record a smoke-test rosbag for replay testing.
# Usage:
#   scripts/record_smoke_bag.sh                # runs until Ctrl+C
#   scripts/record_smoke_bag.sh 90             # records for 90 seconds

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DURATION_SEC="${1:-}"

cd "${ROOT_DIR}"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"

mkdir -p bags
BAG_NAME="bags/smoke_$(date +%F_%H-%M-%S)"

echo "[INFO] Recording bag to: ${BAG_NAME}"
echo "[INFO] Topics: /scan /tf /tf_static /odom /imu/data_raw /cmd_vel /cmd_vel_nav /map /initialpose"

if [[ -n "${DURATION_SEC}" ]]; then
  echo "[INFO] Duration: ${DURATION_SEC}s"
  timeout "${DURATION_SEC}" ros2 bag record -o "${BAG_NAME}" \
    /scan /tf /tf_static /odom /imu/data_raw /cmd_vel /cmd_vel_nav /map /initialpose || true
else
  echo "[INFO] Press Ctrl+C to stop recording"
  ros2 bag record -o "${BAG_NAME}" \
    /scan /tf /tf_static /odom /imu/data_raw /cmd_vel /cmd_vel_nav /map /initialpose
fi

echo "[INFO] Done. Bag folder: ${BAG_NAME}"
if [[ -f "${BAG_NAME}/metadata.yaml" ]]; then
  echo "[INFO] metadata.yaml found"
fi
