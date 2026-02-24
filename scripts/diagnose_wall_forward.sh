#!/usr/bin/env bash

set -u

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

print_header() {
  echo
  echo "============================================================"
  echo "$1"
  echo "============================================================"
}

pass() {
  echo "✅ PASS: $1"
  PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
  echo "❌ FAIL: $1"
  FAIL_COUNT=$((FAIL_COUNT + 1))
}

warn() {
  echo "⚠️  WARN: $1"
  WARN_COUNT=$((WARN_COUNT + 1))
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "❌ Missing command: $1"
    echo "   Make sure ROS 2 is sourced, e.g."
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
  fi
}

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "$topic"
}

get_hz() {
  local topic="$1"
  local output
  output=$(timeout 6s ros2 topic hz "$topic" 2>&1 || true)
  echo "$output" | sed -nE 's/.*average rate: ([0-9.]+).*/\1/p' | tail -n1
}

check_topic_hz() {
  local topic="$1"
  local label="$2"
  if ! topic_exists "$topic"; then
    fail "$label topic missing: $topic"
    return
  fi
  local hz
  hz=$(get_hz "$topic")
  if [[ -n "$hz" ]]; then
    pass "$label is publishing on $topic at ~${hz} Hz"
  else
    fail "$label topic exists but no measurable rate on $topic"
  fi
}

print_header "ROS2 Wall-Forward One-Go Diagnosis"
echo "Workspace: $PWD"
echo "Time: $(date)"

require_cmd ros2
require_cmd timeout
require_cmd awk

print_header "1) Costmap Publishing Check"
check_topic_hz "/local_costmap/costmap" "Local costmap"
check_topic_hz "/global_costmap/costmap" "Global costmap"

print_header "2) LaserScan Path Check (/scan_raw -> /scan)"
check_topic_hz "/scan_raw" "Raw scan"
check_topic_hz "/scan" "Filtered scan"

if topic_exists "/scan_raw" && topic_exists "/scan"; then
  raw_hz=$(get_hz "/scan_raw")
  fixed_hz=$(get_hz "/scan")
  if [[ -n "$raw_hz" && -n "$fixed_hz" ]]; then
    ratio=$(awk -v a="$fixed_hz" -v b="$raw_hz" 'BEGIN { if (b > 0) printf "%.2f", a/b; else print "0.00" }')
    if awk -v r="$ratio" 'BEGIN { exit !(r >= 0.70 && r <= 1.30) }'; then
      pass "Scan relay healthy: /scan roughly tracks /scan_raw (ratio=${ratio})"
    else
      warn "Scan relay rate mismatch: /scan_raw=${raw_hz}Hz, /scan=${fixed_hz}Hz (ratio=${ratio})"
    fi
  fi
fi

print_header "3) TF Consistency Check"
tf_map_odom_out=$(timeout 5s ros2 run tf2_ros tf2_echo map base_footprint 2>&1 || true)
if echo "$tf_map_odom_out" | grep -q "Translation"; then
  pass "TF map -> base_footprint available"
else
  fail "TF map -> base_footprint unavailable or unstable"
  echo "---- tf2_echo output (map->base_footprint) ----"
  echo "$tf_map_odom_out" | tail -n 8
fi

tf_odom_out=$(timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 || true)
if echo "$tf_odom_out" | grep -q "Translation"; then
  pass "TF odom -> base_footprint available"
else
  fail "TF odom -> base_footprint unavailable or unstable"
  echo "---- tf2_echo output (odom->base_footprint) ----"
  echo "$tf_odom_out" | tail -n 8
fi

print_header "4) Safety Layer Trigger Check"
if topic_exists "/safety_stop"; then
  safety_out=$(timeout 6s ros2 topic echo /safety_stop --once 2>&1 || true)
  if echo "$safety_out" | grep -q "data:"; then
    safety_value=$(echo "$safety_out" | sed -nE 's/.*data: *([0-9.]+).*/\1/p' | tail -n1)
    pass "Safety topic is active (/safety_stop data=${safety_value:-unknown})"
    echo "   During wall test, value should drop near obstacle distance, not stay at sentinel/clear."
  else
    fail "No message received on /safety_stop"
  fi
else
  fail "Safety topic missing: /safety_stop"
fi

print_header "5) Detection Angle & Trigger Config"
front_angle_out=$(ros2 param get /arduino_motor_bridge front_obstacle_half_angle_deg 2>&1 || true)
if echo "$front_angle_out" | grep -q "Double value\|Integer value"; then
  front_angle=$(echo "$front_angle_out" | awk '{print $NF}')
  if awk -v a="$front_angle" 'BEGIN { exit !(a >= 60.0) }'; then
    pass "Front detection half-angle is wide enough (${front_angle} deg)"
  else
    warn "Front detection half-angle is narrow (${front_angle} deg). Recommend >= 60.0"
  fi
else
  fail "Could not read parameter: /arduino_motor_bridge front_obstacle_half_angle_deg"
fi

trigger_out=$(ros2 param get /arduino_motor_bridge safety_stop_trigger_distance 2>&1 || true)
if echo "$trigger_out" | grep -q "Double value\|Integer value"; then
  trigger_dist=$(echo "$trigger_out" | awk '{print $NF}')
  if awk -v d="$trigger_dist" 'BEGIN { exit !(d >= 0.12) }'; then
    pass "Ultrasonic trigger distance is proactive (${trigger_dist} m)"
  else
    warn "Ultrasonic trigger distance is very tight (${trigger_dist} m). Recommend >= 0.12 m"
  fi
else
  fail "Could not read parameter: /arduino_motor_bridge safety_stop_trigger_distance"
fi

print_header "Summary"
echo "✅ Passed : $PASS_COUNT"
echo "⚠️  Warnings: $WARN_COUNT"
echo "❌ Failed : $FAIL_COUNT"

if [[ $FAIL_COUNT -gt 0 ]]; then
  echo
  echo "Result: NOT READY - Fix failed checks first."
  exit 2
fi

echo
echo "Result: READY FOR WALL TEST"
echo "Next: Slowly drive toward a wall and watch /safety_stop + local costmap update in RViz."
exit 0
