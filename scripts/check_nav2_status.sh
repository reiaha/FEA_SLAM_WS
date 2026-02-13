#!/usr/bin/env bash
set -u

section() {
  echo ""
  echo "== $1 =="
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found in PATH. Source your ROS setup first."
  echo "Example: source /opt/ros/humble/setup.bash"
  exit 1
fi

section "ROS Env"
echo "ROS_DISTRO=${ROS_DISTRO-}"
echo "ROS_VERSION=${ROS_VERSION-}"

action_topics=$(ros2 action list 2>/dev/null | grep -E 'navigate_to_pose|navigate_through_poses' || true)
section "Nav2 Actions"
if [[ -z "$action_topics" ]]; then
  echo "No Nav2 actions found."
else
  echo "$action_topics"
fi

section "Nav2 Nodes"
nav2_nodes=$(ros2 node list 2>/dev/null | grep -E 'nav2|planner|controller|bt|behavior|smoother|costmap|lifecycle' || true)
if [[ -z "$nav2_nodes" ]]; then
  echo "No Nav2-related nodes found."
else
  echo "$nav2_nodes"
fi

section "Lifecycle States"
for node in /bt_navigator /planner_server /controller_server /behavior_server /smoother_server /waypoint_follower /velocity_smoother /lifecycle_manager_navigation; do
  if ros2 node list 2>/dev/null | grep -qx "$node"; then
    echo "$node:" 
    ros2 lifecycle list "$node" 2>/dev/null || true
  else
    echo "$node: not running"
  fi
  echo ""
done

section "TF (once)"
ros2 topic echo /tf --once 2>/dev/null || echo "No /tf data."

section "TF Static (once)"
ros2 topic echo /tf_static --once 2>/dev/null || echo "No /tf_static data."

section "Costmaps (once)"
for topic in /global_costmap/costmap /local_costmap/costmap; do
  echo "$topic:" 
  ros2 topic echo "$topic" --once 2>/dev/null || echo "No data on $topic"
  echo ""
done

section "Costmap Topics"
ros2 topic list 2>/dev/null | grep costmap || echo "No costmap topics."
