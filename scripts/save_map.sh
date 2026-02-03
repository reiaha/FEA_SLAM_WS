#!/bin/bash
# Save SLAM Toolbox Map
# Run this while the robot is still running to save the current map

echo "╔════════════════════════════════════════════════════════════╗"
echo "║              SLAM TOOLBOX MAP SAVER                        ║"
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

# Full path for the map (without extension - SLAM Toolbox adds .data and .posegraph)
MAP_PATH="$SAVE_DIR/$MAP_NAME"

echo ""
echo "Saving map to: $MAP_PATH"
echo ""

# Source the workspace
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

# Call SLAM Toolbox serialize_map service
echo "📡 Calling SLAM Toolbox serialize_map service..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_PATH'}"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Map saved successfully!"
    echo ""
    echo "Files created:"
    ls -lh "$MAP_PATH"* 2>/dev/null
    echo ""
    echo "Map location: $SAVE_DIR/"
    echo "Map name: $MAP_NAME"
    echo ""
    echo "To load this map later, use:"
    echo "  ros2 launch fea_slam robot_full.launch.py slam:=true map:=$MAP_PATH.yaml"
else
    echo ""
    echo "❌ Failed to save map. Make sure:"
    echo "  - SLAM Toolbox is running"
    echo "  - Robot has created a map"
    echo "  - /slam_toolbox/serialize_map service is available"
    echo ""
    echo "Check available services with:"
    echo "  ros2 service list | grep slam_toolbox"
fi

echo ""
