#!/bin/bash
# FEA-SLAM Robot Startup Script
# This script starts the complete autonomous exploration system

cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fea_slam robot_full.launch.py slam:=true exploration:=true map_odom_fallback:=true 
