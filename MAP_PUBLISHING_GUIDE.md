# FEA-SLAM Map Publishing and RViz Configuration

**Hardware Operation Only - No Gazebo Simulation**

This document covers map publishing configuration for the real hardware FEA-SLAM robot. All operations are performed on physical robot hardware without simulation support.

## Summary of Changes

### 1. Fixed Frame Updated to "map"
- **RViz Configuration**: Changed global fixed frame from `base_footprint` to `map`
- **File**: `config/view_robot.rviz`
- **Effect**: All visualizations are now relative to the global map frame instead of the robot's base frame

### 2. Single RViz Instance Guarantee
- **Updated Files**: 
  - `launch/display.launch.py`
  - `launch/robot_full.launch.py`
- **Changes**:
  - Added `emulate_tty=True` for proper terminal handling
  - Added TF remappings to prevent duplicate subscriptions
  - Single RViz node with proper configuration

### 3. Map Publishing Configuration
- **New File**: `config/slam_toolbox.yaml`
- **New File**: `launch/slam_toolbox.launch.py`
- **Features**:
  - Configured SLAM Toolbox 2 for continuous map publishing
  - Publishes `/map` topic (occupancy grid)
  - Publishes `map -> base_footprint` transformation
  - Map update interval: 5 seconds
  - Loop closure enabled

---

## Map Publishing Details

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid map from SLAM |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | Incremental map updates |

### Published Transformations

| From | To | Description |
|------|-----|-------------|
| `map` | `base_footprint` | Robot's position in global map frame |
| `base_footprint` | `base_link` | Robot center frame |
| `base_link` | All sensors | Sensor positions (lidar_link, ultrasonic_link, etc.) |

### Full Transform Tree

```
map
└── base_footprint
    └── base_link
        ├── middle_plate
        │   └── ultrasonic_link
        ├── lower_plate
        ├── upper_plate
        │   └── lidar_link
        ├── wheel_left
        ├── wheel_right
        ├── caster_front
        └── caster_rear
```

---

## RViz Configuration

### Global Settings
- **Fixed Frame**: `map` (global reference frame)
- **Frame Rate**: 30 Hz
- **Background**: Dark gray (48; 48; 48)

### Display Plugins

1. **Grid** - XY plane grid relative to map frame
2. **RobotModel** - 3D visualization of robot structure
3. **TF** - Transform frame visualization
4. **LaserScan** - Live LiDAR scan points
5. **Map** - Occupancy grid from SLAM Toolbox

### Launch Configuration

```yaml
Visualization:
  Fixed Frame: map  # Global reference
  Displays:
    - Grid (XY plane)
    - RobotModel (robot structure)
    - TF (transform tree)
    - LaserScan (LiDAR points)
    - Map (SLAM occupancy grid)
```

---

## Launching the System

### Full System with Map Publishing and Single RViz

```bash
# Build the package
cd ~/FEA_SLAM_WS
colcon build --packages-select fea_slam
source install/setup.bash

# Launch complete system
ros2 launch fea_slam robot_full.launch.py
```

### What Gets Launched

1. **Robot State Publisher**
   - Publishes transform tree
   - Robot model from URDF

2. **Joint State Publisher**
   - Publishes joint states for visualization

3. **YDLiDAR Driver** (if enabled)
   - Publishes `/scan` topic with laser data

4. **SLAM Toolbox**
   - Processes laser scans
   - Publishes `/map` (occupancy grid)
   - Publishes `map -> base_footprint` transform
   - Detects loop closures

5. **RViz** (single instance)
   - Displays everything
   - Fixed frame set to `map`
   - Shows robot, scans, and map together

---

## Verifying Map Publishing

### Check Map Topic

```bash
# List all active topics
ros2 topic list | grep map

# Echo map topic (subscribe and display)
ros2 topic echo /map

# Check map publishing rate
ros2 topic hz /map
```

### Check Transform Tree

```bash
# View full transform tree
ros2 run tf2_tools view_frames
cat frames.pdf  # Open to visualize

# Check specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# Check all transforms
ros2 run tf2_ros tf2_echo map lidar_link
```

### Monitor in RViz

- Map display should show building map as robot moves
- Robot model should move relative to map
- Grid should remain fixed (map frame)
- TF tree should show all connections

---

## Troubleshooting

### Map Not Publishing

1. **Check SLAM status**:
   ```bash
   ros2 topic list | grep -E "(scan|map)"
   ```

2. **Verify LiDAR data**:
   ```bash
   ros2 topic echo /scan --once
   ```

3. **Check SLAM node**:
   ```bash
   ros2 node list | grep slam
   ```

### RViz Showing Wrong Coordinates

1. **Verify fixed frame in RViz**:
   - Should be `map`
   - Check: Display panel → Fixed Frame

2. **Check transform availability**:
   ```bash
   ros2 run tf2_ros tf2_echo map base_footprint
   ```

### Multiple RViz Windows Opening

1. **Check for lingering processes**:
   ```bash
   pkill -f rviz2
   ```

2. **Verify single instance launch**:
   - `emulate_tty=True` should prevent duplicates

3. **Clear old configurations**:
   ```bash
   rm ~/.ros/rviz2/* (if using custom dir)
   ```

---

## Key Files

| File | Purpose |
|------|---------|
| `config/view_robot.rviz` | RViz display config (Fixed Frame: map) |
| `config/slam_toolbox.yaml` | SLAM Toolbox parameters |
| `launch/slam_toolbox.launch.py` | SLAM Toolbox launcher |
| `launch/display.launch.py` | Visualization only (single RViz) |
| `launch/robot_full.launch.py` | Full system with SLAM (single RViz) |
| `description/robot_core.xacro` | Robot URDF with base_footprint |

---

## Performance Notes

- **Map Update Rate**: 5 seconds (configurable in slam_toolbox.yaml)
- **Transform Publish Rate**: 50 Hz (0.02s)
- **RViz Rendering**: 30 Hz
- **Loop Closure**: Enabled for map consistency

---

## Next Steps

1. **Start Robot**:
   ```bash
   ros2 launch fea_slam robot_full.launch.py
   ```

2. **Drive Robot** - Move robot around to build map
   - Use manual control or autonomous navigation

3. **Monitor Map**:
   - Watch RViz for map building
   - Check `/map` topic publishing

4. **Close Loops**:
   - Return to previously visited areas
   - SLAM will detect loop closure and optimize

5. **Save Map** (optional):
   - SLAM Toolbox will save to disk automatically
   - Or use `ros2 service call /save_map nav2_msgs/SaveMap`
