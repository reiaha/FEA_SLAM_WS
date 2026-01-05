# FEA-SLAM: Hardware-Only Project

## Important Note

**This project is designed for REAL HARDWARE OPERATION ONLY.**

There is **NO Gazebo simulation** support in this project. All development, testing, and evaluation is performed on the physical FEA-SLAM robot platform.

---

## Why Hardware-Only?

1. **Research Focus**: The FEA-SLAM framework is designed to validate real-world performance in GPS-denied environments
2. **Sensor Integration**: Direct hardware sensor feedback (LiDAR, ultrasonic, encoders) is essential for SLAM validation
3. **Real-World Constraints**: Hardware operation reveals true system limitations and capabilities
4. **Frontier Exploration**: Meaningful frontier-based exploration requires actual environmental navigation

---

## Hardware Platform

### Physical Robot
- **Base**: Stack-based mobile platform with differential drive
- **Actuation**: Dual DC motors with wheel encoders
- **Power**: Battery-powered system
- **Weight**: Compact form factor for indoor navigation

### Sensors
- **LiDAR**: YDLiDAR X3 (360° 2D laser scanning)
- **Ultrasonic**: HC-SR04 proximity sensor (front-facing)
- **Encoders**: Wheel encoders for odometry
- **IMU**: Gyroscope data from wheel-based odometry

### Compute
- **Onboard PC**: Raspberry Pi 4 (or equivalent)
- **OS**: Ubuntu 20.04 / 22.04 with ROS 2
- **Communication**: WiFi (robot to development machine)

---

## Supported Workflows

### Workflow 1: Remote Development & Hardware Control
```
Development Machine (with ROS 2)
         ↓
    ROS 2 Network
         ↓
    Physical Robot
   (LiDAR, Motors, etc.)
```

### Workflow 2: Bag File Analysis
```
Record Hardware Data
        ↓
   Rosbag File
        ↓
Use with use_sim_time=true
   for Offline Analysis
```

**Note**: use_sim_time is for **recorded data playback only**, not Gazebo simulation.

---

## What You Cannot Do

❌ Simulate robot movement
❌ Test without physical hardware
❌ Use Gazebo environments
❌ Run virtual sensor data
❌ Test without actual LiDAR scans

---

## What You Can Do

✅ Control physical robot via ROS 2
✅ Visualize real sensor data in RViz
✅ Run SLAM Toolbox on live LiDAR data
✅ Implement frontier-based exploration
✅ Record and playback real data with rosbag
✅ Test SLAM in real environments
✅ Evaluate map quality on actual maps
✅ Measure real robot navigation accuracy

---

## Getting Started with Hardware

### 1. Setup Hardware Robot
```bash
# On robot (Raspberry Pi):
ros2 launch fea_slam robot_full.launch.py
```

### 2. Connect Development Machine
```bash
# On development machine:
export ROS_DOMAIN_ID=0
export ROS_MASTER_URI=http://<robot-ip>:11311
ros2 topic list  # Should see /scan, /map, etc.
```

### 3. View in RViz (Development Machine)
```bash
# Opens single RViz instance with map frame
ros2 launch fea_slam display.launch.py
```

### 4. Monitor SLAM Mapping
```bash
# Check map publishing
ros2 topic echo /map

# Check transform tree
ros2 run tf2_ros tf2_echo map base_footprint

# Record rosbag for offline analysis
ros2 bag record /scan /map /tf
```

---

## Hardware Requirements Checklist

- [ ] Raspberry Pi 4 (4GB RAM minimum)
- [ ] YDLiDAR X3 or compatible 2D LiDAR
- [ ] HC-SR04 ultrasonic sensor
- [ ] DC motor drivers (L298N or equivalent)
- [ ] Dual DC motors with encoders
- [ ] LiPo battery (7.4V recommended)
- [ ] USB WiFi adapter (if not integrated)
- [ ] Ubuntu 20.04 / 22.04 with ROS 2
- [ ] Development machine with ROS 2

---

## Performance on Hardware

### Typical Performance Metrics

| Metric | Value |
|--------|-------|
| SLAM Update Rate | 20-30 Hz |
| Map Update Rate | 5 Hz |
| Max Navigation Speed | 0.5 m/s |
| Max Turn Rate | 1.5 rad/s |
| LiDAR Range | 0.27 - 12 m |
| Mapping Accuracy | ±5-10 cm |
| Loop Closure Detection | After ~50m travel |

### Environmental Factors
- **Lighting**: Works with varying light (LiDAR is light-independent)
- **Surfaces**: Works on flat indoor floors (carpet, tile, wood)
- **Obstacles**: Detects static and moving objects
- **Noise**: Handles reflections and glass surfaces

---

## Troubleshooting Hardware Issues

### LiDAR Not Publishing
```bash
# Check connection
ls -l /dev/ttyUSB*

# Check driver
ros2 node list | grep lidar

# Test serial connection
picocom /dev/ttyUSB0 -b 115200
```

### Motor Control Issues
```bash
# Test GPIO pins
gpio -v
gpio readall

# Check motor driver connections
# Verify power supply voltage
```

### Odometry Drift
- Ensure wheel encoders are functioning
- Check for wheel slippage
- Verify SLAM is detecting loop closures
- Recalibrate encoder counts

### Map Quality Issues
- Move robot slowly for accurate scans
- Ensure LiDAR is horizontal
- Clean LiDAR lens
- Check for reflective surfaces

---

## Recording Data for Offline Analysis

### Record Hardware Data
```bash
# On robot or over network
ros2 bag record /scan /tf /tf_static /odom
```

### Playback with Offline Processing
```bash
# On development machine with use_sim_time=true
ros2 launch fea_slam robot_full.launch.py use_sim_time:=true lidar:=false

# In another terminal
ros2 bag play recorded_data.db3 --clock
```

---

## Hardware Deployment Checklist

Before running on physical robot:

1. **Safety**
   - [ ] Robot on flat, clear surface
   - [ ] No obstacles in movement area
   - [ ] Emergency stop accessible
   - [ ] Battery charged

2. **Connectivity**
   - [ ] Robot connected to WiFi
   - [ ] Robot IP address known
   - [ ] Ping successful from development machine

3. **Hardware**
   - [ ] LiDAR spinning (blue ring visible)
   - [ ] Motors respond to commands
   - [ ] Encoders reading movement
   - [ ] Sensors publishing data

4. **Software**
   - [ ] ROS 2 running on robot
   - [ ] SLAM Toolbox started
   - [ ] Map being published
   - [ ] RViz showing data

---

## Key Files for Hardware Operation

| File | Purpose |
|------|---------|
| `robot_full.launch.py` | Start all hardware systems |
| `slam_toolbox.launch.py` | SLAM with map publishing |
| `slam_toolbox.yaml` | SLAM configuration |
| `view_robot.rviz` | RViz visualization (fixed frame: map) |
| `robot_core.xacro` | Hardware robot description |

---

## Future Hardware Enhancements

- 3D LiDAR for multi-floor mapping
- IMU integration for pitch/roll stabilization
- Better encoder calibration
- Motor feedback control
- Battery management system
- Wireless charging dock

---

## Important Reminders

🔴 **This is not a simulation project**
- All code runs on real hardware
- All testing is on physical robots
- No virtual environments
- No Gazebo support

🔴 **Hardware-only operation**
- Requires physical robot platform
- Requires ROS 2 on Raspberry Pi
- Requires LiDAR and motor drivers
- Requires WiFi connectivity

🟢 **Advantages of Hardware-Only**
- Real-world validation
- Actual SLAM performance
- True frontier exploration
- Genuine robot navigation
- Publishable research results

---

## Support & Troubleshooting

For hardware issues:
1. Check ROS 2 topics are publishing
2. Verify hardware connections
3. Test sensors independently
4. Review ROS 2 logs
5. Check transform tree completeness

For software issues:
1. Review launch file parameters
2. Check RViz fixed frame (should be "map")
3. Verify SLAM Toolbox is running
4. Monitor topic publishing rates
5. Check for ROS 2 domain ID mismatches

---

**Hardware Operation Only - No Simulation Support**

This project brings frontier-based exploration and SLAM to real physical robots in GPS-denied environments.
