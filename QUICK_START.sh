#!/bin/bash
# Quick-Start Commands for FEA-SLAM Frontier Exploration
# Copy and paste these commands into your terminal

echo "FEA-SLAM Frontier Exploration Quick-Start"
echo "=========================================="
echo ""
echo "Usage: Copy any command below into your terminal"
echo ""

# Setup
echo "## SETUP (Run once per session)"
echo "cd /home/pi/FEA_SLAM_WS && source install/setup.bash"
echo ""

# Basic Exploration
echo "## START BASIC EXPLORATION"
echo "ros2 launch fea_slam robot_full.launch.py exploration:=true"
echo ""

# Manual Mode
echo "## START MANUAL MODE (no auto exploration)"
echo "ros2 launch fea_slam robot_full.launch.py exploration:=false"
echo ""

# Fast Exploration (closest frontier, 5 min limit)
echo "## FAST EXPLORATION (closest, 5 min limit)"
echo "# Edit robot_full.launch.py:"
echo "# - frontier_selection_method: 'closest'"
echo "# - max_exploration_time: 300.0"
echo "ros2 launch fea_slam robot_full.launch.py exploration:=true"
echo ""

# Thorough Exploration (gain, 20 min limit)
echo "## THOROUGH EXPLORATION (info gain, 20 min limit)"
echo "# Edit robot_full.launch.py:"
echo "# - frontier_selection_method: 'gain'"
echo "# - max_exploration_time: 1200.0"
echo "ros2 launch fea_slam robot_full.launch.py exploration:=true"
echo ""

# Individual Node Startup
echo "## START NODES INDIVIDUALLY"
echo "Terminal 1: ros2 launch fea_slam robot_full.launch.py exploration:=false"
echo "Terminal 2: ros2 run localization frontier_detector"
echo "Terminal 3: ros2 run localization exploration_coordinator"
echo ""

# Monitoring
echo "## MONITOR FRONTIER EXPLORATION"
echo "# In separate terminals:"
echo "ros2 topic echo /frontiers --rate=1           # See frontier locations"
echo "ros2 topic echo /current_frontier_goal --rate=1  # See current goal"
echo "ros2 topic hz /map                            # Check map update rate"
echo ""

# Manual Teleop
echo "## MANUAL ROBOT CONTROL"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "# Use arrow keys to move"
echo ""

# Save Map
echo "## SAVE FINAL MAP"
echo "ros2 run nav2_map_server map_saver_cli -f ~/final_map"
echo ""

# Debugging
echo "## DEBUG COMMANDS"
echo "ros2 node list                                # List all nodes"
echo "ros2 topic list                               # List all topics"
echo "ros2 topic hz /scan                           # Check LiDAR rate"
echo "ros2 param list                               # List all parameters"
echo ""

# Cleanup
echo "## STOP EVERYTHING"
echo "# Press Ctrl+C in all terminals"
echo "pkill -f ros2                                 # Kill all ROS2"
echo ""

# Build
echo "## REBUILD PACKAGES"
echo "cd /home/pi/FEA_SLAM_WS && colcon build --packages-select localization fea_slam"
echo ""

# View Logs
echo "## VIEW LOGS"
echo "tail -f ~/.ros/log/latest/*/stdout.log        # Real-time log"
echo ""

# System Check
echo "## VERIFY SETUP"
echo "# Check Arduino"
echo "ls -la /dev/ttyACM0"
echo "# Check LiDAR"
echo "ls -la /dev/ttyUSB0"
echo "# Check ROS2 version"
echo "ros2 --version"
echo ""

echo "=========================================="
echo "See SYSTEM_GUIDE.md for detailed info"
echo "See FRONTIER_EXPLORATION_GUIDE.md for algorithm details"
