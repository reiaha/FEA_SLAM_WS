# Saved Maps Directory

This directory stores maps created by the FEA-SLAM robot.

## Usage

### Save a new map:
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Save with custom name
ros2 run nav2_map_server map_saver_cli -f saved_maps/my_map_name

# Save with timestamp
MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f saved_maps/$MAP_NAME
```

### Map files:
Each saved map creates two files:
- `.pgm` - The occupancy grid image (grayscale)
- `.yaml` - Map metadata (resolution, origin, thresholds)

### View saved maps:
```bash
ls -lh saved_maps/
```

### Load a map for navigation:
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=saved_maps/my_map_name.yaml
```

## Map Naming Convention

Recommended format: `[location]_[date]_[version].pgm`

Examples:
- `living_room_20260121.pgm`
- `hallway_v2.pgm`
- `full_house_20260121_1430.pgm`
