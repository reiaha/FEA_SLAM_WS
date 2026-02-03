#!/bin/bash
# Quick diagnostics for exploration issues

echo "=== Checking Robot Movement ==="
echo "Monitoring /cmd_vel for 3 seconds..."
timeout 3 ros2 topic echo /cmd_vel --once

echo ""
echo "=== Checking Frontier Detection ==="
ros2 topic echo /frontiers --once | head -20

echo ""
echo "=== Checking Robot Position ==="
ros2 topic echo /robot_pose --once | head -10

echo ""
echo "=== Checking Obstacle Distance ==="
ros2 topic echo /front_obstacle_distance --once
