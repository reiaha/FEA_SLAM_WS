#!/bin/bash
# Save Map as PGM/YAML (ROS standard format)
# This creates a static map file that can be used with Nav2 localization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         SAVE MAP AS PGM/YAML (Nav2 Format)                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if a map name was provided
if [ -z "$1" ]; then
    # Generate timestamp-based name if no name provided
    MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
    echo "No map name provided. Using: $MAP_NAME"
else
    MAP_NAME="$1"
fi

# Create saved_maps directory if it doesn't exist
SAVE_DIR="/home/pi/FEA_SLAM_WS/saved_maps"
mkdir -p "$SAVE_DIR"

# Full path for the map
MAP_PATH="$SAVE_DIR/$MAP_NAME"

echo ""
echo "Saving map to: $MAP_PATH.pgm/.yaml"
echo ""

# Source the workspace
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

# Use map_saver to save current /map topic as PGM/YAML
echo "📡 Saving /map topic to PGM/YAML format..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10000

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Map saved successfully in Nav2 format!"
    echo ""
    echo "Files created:"
    ls -lh "$MAP_PATH"* 2>/dev/null
    echo ""
    echo "Map location: $SAVE_DIR/"
    echo ""
    echo "To use this map for localization (AMCL), run:"
    echo "  ros2 launch fea_slam robot_full.launch.py slam:=false map:=$MAP_PATH.yaml"
else
    echo ""
    echo "❌ Failed to save map. Make sure:"
    echo "  - SLAM Toolbox is running and publishing to /map"
    echo "  - The map topic has data"
    echo ""
    echo "Check map topic with:"
    echo "  ros2 topic echo /map --once"
fi

echo ""
