# FEA-SLAM Data Flow and RViz Visualization

## System Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     FEA-SLAM Data Pipeline                      │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────────┐
│   YDLiDAR X3 Hardware    │
│  (360° Laser Scanner)    │
└──────────┬───────────────┘
           │
           ├──→ /scan (sensor_msgs/LaserScan)
           │    └─ [x, y, range, intensity data]
           │
┌──────────┴───────────────────────────────────────────────────────┐
│                    SLAM Toolbox Processing                        │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ Input: /scan                                                 │ │
│  │  - Continuous laser scans from LiDAR                        │ │
│  │  - 360° field of view                                       │ │
│  │  - Range: 0.27-12 meters                                    │ │
│  │                                                              │ │
│  │ Processing:                                                  │ │
│  │  - Scan-to-map matching                                     │ │
│  │  - Occupancy grid generation                                │ │
│  │  - Graph-based SLAM optimization                            │ │
│  │  - Loop closure detection                                   │ │
│  │                                                              │ │
│  │ Outputs:                                                     │ │
│  │  - /map (nav_msgs/OccupancyGrid)                           │ │
│  │  - /map_updates (map_msgs/OccupancyGridUpdate)             │ │
│  │  - map → base_footprint (Transform)                         │ │
│  │  - base_footprint → base_link (Transform)                   │ │
│  │  - base_link → all sensors (Transforms)                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────┬──────────────────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
     /map        /map_updates      /tf /tf_static
     (Grid)      (Incremental)     (Transforms)
        │              │              │
        └──────────────┼──────────────┘
                       │
        ┌──────────────┴──────────────────────────┐
        │                                         │
        ▼                                         ▼
   ┌──────────────┐                    ┌─────────────────────┐
   │  /robot_     │                    │  RViz Visualization │
   │  description │                    │  (visualization_pkg)│
   │  (URDF)      │                    │                     │
   └──────────────┘                    │ Displays:           │
        │                              │  • Map (/map)       │
        │                              │  • LaserScan (/scan)│
        │                              │  • TF Tree (/tf)    │
        │                              │  • Robot Model      │
        │                              │                     │
        └──────────────────┬───────────┤ Fixed Frame: map    │
                           │           └─────────────────────┘
                           │
                    ┌──────┴────────┐
                    │               │
                    ▼               ▼
              Final RViz      Map Visualization
              One Instance    (Occupancy Grid)
```

---

## Topic Structure

### Input Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | YDLiDAR driver | 360° laser scan points |

### SLAM Processing

| Input | Processing | Output |
|-------|-----------|--------|
| `/scan` | Scan matching | `/map` |
|  | Graph optimization | `/map_updates` |
|  | Transform computation | `/tf`, `/tf_static` |

### Output Topics

| Topic | Type | Publisher | Contains |
|-------|------|-----------|----------|
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Occupancy grid from scans |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | SLAM Toolbox | Incremental map updates |
| `/robot_description` | String (URDF) | robot_state_publisher | Robot model definition |

### Transform Topics (TF)

| Source | Target | Publisher | Update Rate |
|--------|--------|-----------|-------------|
| `map` | `base_footprint` | SLAM Toolbox | 5 Hz |
| `base_footprint` | `base_link` | robot_state_publisher | 50 Hz |
| `base_link` | Sensors | robot_state_publisher | 50 Hz |

---

## RViz Display Configuration

### visualization_pkg RViz Displays

**File**: `/home/pi/FEA_SLAM_WS/src/visualization_pkg/config/visualization.rviz`

#### 1. Grid Display
- **Type**: `rviz_default_plugins/Grid`
- **Frame**: map (fixed frame)
- **Size**: 10x10 cells
- **Color**: Light gray (160, 160, 164)
- **Purpose**: Ground reference plane

#### 2. Map Display ✅ (NEWLY ADDED)
- **Type**: `rviz_default_plugins/Map`
- **Topic**: `/map` (Primary) and `/map_updates` (Incremental)
- **Data**: Occupancy grid from SLAM
- **Color Scheme**: Standard map (white=free, gray=unknown, black=occupied)
- **Update Rate**: 5 Hz
- **Purpose**: Shows the built map in real-time

#### 3. LaserScan Display
- **Type**: `rviz_default_plugins/LaserScan`
- **Topic**: `/scan`
- **Color Transformer**: Intensity
- **Style**: Flat Squares (3 pixels)
- **Purpose**: Visualizes current LiDAR data points

#### 4. TF Display ✅ (ENABLED)
- **Type**: `rviz_default_plugins/TF`
- **Status**: Enabled
- **Show Arrows**: True
- **Show Axes**: True
- **Show Names**: True
- **Purpose**: Displays coordinate frame tree and transformations

#### 5. RobotModel Display
- **Type**: `rviz_default_plugins/RobotModel`
- **Description Source**: Topic (`/robot_description`)
- **Purpose**: Shows robot structure and position

### Global RViz Settings

```yaml
Fixed Frame: map          # Global reference frame
Frame Rate: 30 Hz         # Rendering frequency
Background: Dark gray     # Display background
```

---

## Data Verification Checklist

### Topics Published
- [ ] `/scan` - LiDAR data from YDLiDAR driver
- [ ] `/map` - Occupancy grid from SLAM Toolbox
- [ ] `/map_updates` - Incremental map updates
- [ ] `/robot_description` - Robot URDF from robot_state_publisher
- [ ] `/tf` - Transform frames being published
- [ ] `/tf_static` - Static transforms

### Transforms Published
- [ ] `map` frame exists
- [ ] `base_footprint` exists and linked to map
- [ ] `base_link` exists and linked to base_footprint
- [ ] `lidar_link` exists (for sensor data)
- [ ] All sensor links visible in TF tree

### RViz Displays Active
- [ ] Grid visible (reference plane)
- [ ] Map display showing occupancy grid ✅
- [ ] LaserScan points visible ✅
- [ ] TF tree showing all frames ✅
- [ ] Robot model positioned correctly ✅

---

## Debugging Data Flow

### Check if Topics are Publishing

```bash
# List all active topics
ros2 topic list

# Check map topic
ros2 topic echo /map --once

# Check scan topic
ros2 topic echo /scan --once

# Check publishing rates
ros2 topic hz /map
ros2 topic hz /scan
ros2 topic hz /tf
```

### Check Transform Tree

```bash
# View all transforms
ros2 run tf2_tools view_frames
cat frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo map base_footprint

# List all frames
ros2 run tf2_ros tf2_echo --help
```

### Verify RViz Connections

```bash
# Check if RViz can see /map topic
# In RViz: Add → Map → /map

# Check if RViz can see /scan topic
# In RViz: Add → LaserScan → /scan

# Check if RViz can see transforms
# In RViz: Add → TF → (should show tree)
```

---

## Common Issues and Solutions

### Map Not Displaying

**Problem**: Map display is empty or not updating

**Causes**:
1. `/map` topic not publishing
2. Fixed frame not set to `map`
3. SLAM Toolbox not initialized

**Solutions**:
```bash
# Check if /map is publishing
ros2 topic hz /map

# Check if SLAM is running
ros2 node list | grep slam

# Verify fixed frame in RViz
# Global Options → Fixed Frame = "map"
```

### LaserScan Not Showing

**Problem**: No scan points visible in RViz

**Causes**:
1. YDLiDAR not connected
2. `/scan` topic not publishing
3. Decay time too low

**Solutions**:
```bash
# Check LiDAR connection
ls -l /dev/ttyUSB*

# Check scan topic
ros2 topic hz /scan

# Adjust decay time in RViz
# LaserScan → Decay Time = 0 (for all points visible)
```

### Transform Errors

**Problem**: "Unknown frame" errors in RViz

**Causes**:
1. Transform not published yet
2. Fixed frame not initialized
3. SLAM still initializing

**Solutions**:
```bash
# Check available frames
ros2 run tf2_ros tf2_echo map base_footprint

# Wait for SLAM initialization (20-30 seconds)

# Check tf tree
ros2 run tf2_tools view_frames
```

---

## Correct Data Flow Summary

✅ **YDLiDAR** → `/scan` topic
✅ **SLAM Toolbox** processes `/scan` → `/map` + Transforms
✅ **robot_state_publisher** publishes `/robot_description` + base transforms
✅ **RViz (visualization_pkg)** subscribes to:
   - `/map` (Map display)
   - `/scan` (LaserScan display)
   - `/robot_description` (RobotModel)
   - `/tf` (TF display)

✅ **Final RViz Window** opens from visualization_pkg
✅ **All data flows correctly** with proper frame transformations

