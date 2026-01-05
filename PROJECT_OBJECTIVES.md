# FEA-SLAM Project Summary

**Hardware Operation Only - No Gazebo Simulation**

This project is designed exclusively for real hardware robot operation. All development, testing, and evaluation is performed on the physical FEA-SLAM robot platform. There is no Gazebo simulation support.

## Project Name
**FEA-SLAM: Frontier-based Exploration and SLAM for Autonomous Robot Navigation in GPS-Denied Environments**

## Research Objectives

### 1. Design and Implement SLAM-Based Navigation System
- Develop a robust localization framework capable of autonomous localization and mapping
- Enable navigation in GPS-denied environments
- Achieve real-time position estimation and environmental reconstruction

### 2. Integrate Frontier-Based Exploration Algorithm
- Implement autonomous exploration strategies
- Optimize area coverage in unfamiliar environments
- Enhance real-time decision-making capabilities
- Enable adaptive exploration based on discovered frontiers

### 3. Evaluate System Performance
- Assess localization accuracy across varying environments
- Evaluate mapping reliability and map quality
- Measure exploration efficiency metrics
- Test robustness in dynamic environmental conditions

## Technology Stack

### SLAM Framework
- **SLAM Toolbox 2**: Advanced graph-based SLAM with loop closure detection
- Features:
  - Real-time pose graph optimization
  - Loop closure detection and correction
  - Lifelong mapping capabilities
  - Serialization of maps and poses

### Robot Platform
- **Hardware**: Stack-based mobile platform with differential drive
- **Sensors**:
  - YDLiDAR: 360° laser scanning for SLAM perception
  - Ultrasonic sensor: Front-facing obstacle detection
  - Wheel encoders: Odometry for motion estimation
  - Dual drive motors: Differential steering control

### ROS 2 Integration
- robot_state_publisher: Transform publishing
- joint_state_publisher: Joint state management
- rviz2: Real-time visualization and debugging
- Navigation 2 (nav2): Path planning and autonomous navigation (planned)

## System Architecture

```
┌──────────────────────────────────────────────┐
│    FEA-SLAM Autonomous Navigation System     │
├──────────────────────────────────────────────┤
│                                              │
│  ┌─────────────────┐  ┌──────────────────┐  │
│  │  SLAM Toolbox   │  │   Exploration    │  │
│  │  - Mapping      │  │   - Frontiers    │  │
│  │  - Localization │  │   - Coverage     │  │
│  │  - Loop Closure │  │   - Path Plan    │  │
│  └─────────────────┘  └──────────────────┘  │
│         │                      │             │
│  ┌──────┴──────────────────────┴─────────┐  │
│  │      Sensor Fusion & Odometry         │  │
│  │  - Scan matching                      │  │
│  │  - Feature extraction                 │  │
│  │  - Graph optimization                 │  │
│  └──────┬──────────────────────┬─────────┘  │
│         │                      │             │
│  ┌──────┴──────┐      ┌────────┴──────────┐ │
│  │  YDLiDAR    │      │  Ultrasonic +     │ │
│  │  Laser Scan │      │  Wheel Encoders   │ │
│  └─────────────┘      └───────────────────┘ │
│                                              │
│  ┌──────────────────────────────────────┐   │
│  │  Hardware: Motors, Sensors, Actuators│   │
│  └──────────────────────────────────────┘   │
│                                              │
└──────────────────────────────────────────────┘
```

## Launch Files

### 1. `display.launch.py`
**Purpose**: Robot visualization and manual control testing
- Launches robot_state_publisher
- Launches joint_state_publisher_gui for manual joint control
- Launches RViz2 with pre-configured visualization
- **Use Case**: Desktop testing, robot description verification

### 2. `robot_full.launch.py`
**Purpose**: Complete FEA-SLAM system deployment
- Launches all core components
- Integrates YDLiDAR driver
- Activates SLAM Toolbox for mapping and localization
- Enables frontier-based exploration
- Provides real-time RViz visualization
- **Use Case**: Hardware operation, autonomous mapping and exploration

### 3. `rsp.launch.py`
**Purpose**: Minimal robot state publisher
- Lightweight launch for integration with external systems
- **Use Case**: Integration with external navigation or planning systems

## Key Features

### Real-Time SLAM
- Continuous mapping and localization
- Loop closure detection for map consistency
- Pose graph optimization for accuracy
- Sub-second pose updates for control

### Autonomous Exploration
- Frontier-based exploration strategy
- Real-time frontier detection from scan data
- Intelligent goal selection for coverage
- Adaptive exploration in dynamic environments

### Sensor Integration
- YDLiDAR: 360° 2D scanning for SLAM perception
- Ultrasonic: Obstacle detection for safety
- Wheel encoders: Odometry for motion estimation
- Multi-sensor fusion for robust localization

### Visualization & Debugging
- RViz2 with pre-configured displays
- Robot model visualization
- Live scan visualization
- Occupancy grid and costmap displays
- Transform tree (TF) visualization
- Real-time map updates

## Performance Evaluation Metrics

### Localization Accuracy
- Absolute position error in mapped environments
- Drift accumulation over time and distance
- Loop closure correction effectiveness
- Repeatability in revisited areas

### Mapping Reliability
- Map consistency score
- Feature preservation quality
- Scan matching success rate
- Map update latency

### Exploration Efficiency
- Frontier detection rate (Hz)
- Coverage ratio (%)
- Path length vs. area covered ratio
- Time to full coverage
- Decision-making responsiveness

### System Robustness
- Performance in varying lighting conditions
- Behavior with dynamic obstacles
- Sensor failure resilience
- Long-duration mapping stability

## Build and Deployment

### Build
```bash
cd ~/FEA_SLAM_WS
colcon build --packages-select fea_slam
source install/setup.bash
```

### Run Full System
```bash
ros2 launch fea_slam robot_full.launch.py
```

### Run Visualization Only
```bash
ros2 launch fea_slam display.launch.py
```

## File Structure
```
src/fea_slam/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata with FEA-SLAM objectives
├── README.md                         # Main project documentation
├── config/
│   ├── view_robot.rviz              # RViz visualization config
│   └── empty.yaml                   # Custom configuration template
├── description/
│   ├── robot.urdf.xacro             # Main URDF file
│   ├── robot_core.xacro             # Core robot structure (base_footprint included)
│   ├── materials.xacro              # Material definitions
│   └── meshes/                      # 3D mesh files
├── launch/
│   ├── rsp.launch.py                # Robot state publisher
│   ├── display.launch.py            # Visualization + manual control
│   ├── robot_full.launch.py         # Full FEA-SLAM system
│   └── README.md                    # Launch file documentation
└── worlds/                          # Environment definitions
```

## Dependencies

### Core ROS 2 Packages
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- rviz2
- xacro
- tf2 / tf2_ros

### SLAM & Navigation
- slam_toolbox (Required)
- nav2_bringup (Planned)
- nav2_core (Planned)

### Hardware Drivers
- ydlidar_ros2_driver (Required for LiDAR)

## Research Contributions

1. **SLAM Framework Integration**: Comprehensive integration of SLAM Toolbox 2 with custom robot platform
2. **Frontier-Based Exploration**: Implementation of boundary-based exploration for autonomous coverage
3. **Hardware Implementation**: Full stack from hardware integration to high-level autonomy
4. **Performance Analysis**: Systematic evaluation of localization, mapping, and exploration metrics
5. **GPS-Denied Navigation**: Demonstrated autonomous operation without external positioning

## Future Work

- **3D SLAM**: Extension to 3D mapping for multi-floor environments
- **Dynamic Environments**: Enhanced robustness for moving obstacles
- **Multi-Robot SLAM**: Coordination and map sharing between multiple robots
- **Semantic Understanding**: Integration of object detection and semantic segmentation
- **Real-time Optimization**: Frontier selection optimization using learning-based approaches
- **Integration with Nav2**: Full autonomous navigation stack integration

## References & Standards

- SLAM Toolbox: "Lifelong Mapping in Dynamic Environments"
- Frontier-Based Exploration: Classical and modern approaches
- ROS 2: Humble and later distributions
- IEEE Standards for Robot Navigation and SLAM

## Contact & Maintainer

**Rhea Khim Gaudiano**
- Email: rheakhimgaudiano00@gmail.com
- Repository: https://github.com/rhea-khim/FEA_SLAM_WS
- Project Type: Academic Research - Autonomous Navigation & SLAM
