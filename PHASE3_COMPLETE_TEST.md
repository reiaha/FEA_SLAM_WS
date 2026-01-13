# 🎯 Phase 3 Complete Test Guide - Ubuntu Terminal

**Date:** January 13, 2026  
**Test Type:** Manual Mapping with Local Map Saving  
**Terminal:** Ubuntu Terminal (NOT VSCode)

---

## 📋 Overview

Phase 3 is the **System Integration Test** where you will:
1. ✅ Launch the full system with all nodes
2. ✅ Manually drive the robot to build a map
3. ✅ Monitor all components working together
4. ✅ **Save the map locally** for future use

**Expected Duration:** 15-20 minutes

---

## 🔧 Pre-Test Checklist

Before starting, verify all hardware and software:

### Hardware Check
```bash
# 1. Check Arduino connection
ls -la /dev/ttyACM0
# Expected: crw-rw---- 1 root dialout ... /dev/ttyACM0

# 2. Check LiDAR connection
ls -la /dev/ttyUSB0
# Expected: crw-rw---- 1 root dialout ... /dev/ttyUSB0

# 3. Fix permissions if needed
sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0
```

### Physical Checks
- [ ] Robot on flat, stable surface
- [ ] All wheels spin freely by hand
- [ ] Arduino LED lights are on
- [ ] YDLiDAR motor spinning
- [ ] Battery fully charged
- [ ] All cables securely connected
- [ ] 2m radius around robot is clear
- [ ] Smooth, flat floor (no stairs/ramps)

### Software Setup
```bash
# 1. Navigate to workspace
cd /home/pi/FEA_SLAM_WS

# 2. Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. Verify packages are available
ros2 pkg list | grep -E 'fea_slam|localization'
# Should show: fea_slam, localization

# 4. Create directory for saving maps
mkdir -p ~/maps
```

---

## 🚀 Phase 3: Step-by-Step Test Procedure

### TERMINAL 1: Launch the Full System

```bash
# Navigate to workspace
cd /home/pi/FEA_SLAM_WS

# Source setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch full system (exploration disabled for manual testing)
ros2 launch fea_slam robot_full.launch.py exploration:=false
```

**What You Should See:**
```
[INFO] [launch]: All log files can be found below /home/pi/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [xxx]
[INFO] [joint_state_publisher-2]: process started with pid [xxx]
[INFO] [arduino_motor_bridge-3]: process started with pid [xxx]
[INFO] [ydlidar_ros2_driver_node-4]: process started with pid [xxx]
[INFO] [sync_slam_toolbox_node-5]: process started with pid [xxx]
[INFO] [rviz2-6]: process started with pid [xxx]

[arduino_motor_bridge]: Arduino connected on /dev/ttyACM0
[ydlidar_ros2_driver_node]: Lidar successfully connected [/dev/ttyUSB0:115200]
[ydlidar_ros2_driver_node]: Model: S2PRO
[ydlidar_ros2_driver_node]: Now lidar is scanning...
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
```

**Wait 10-15 seconds** for:
- ✅ All nodes to start
- ✅ RViz window to open
- ✅ "Now lidar is scanning..." message

**If RViz Opens:** You should see:
- Gray floor grid (coordinate system)
- Robot model (wheels, base, sensors)
- Colored axes (red/green/blue = X/Y/Z)
- White/red LiDAR scan dots
- Empty map panel on left

---

### TERMINAL 2: Verify System Status

Open a **NEW Ubuntu Terminal** window:

```bash
# Source the environment
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check all nodes are running
ros2 node list
```

**Expected Output:**
```
/arduino_motor_bridge
/joint_state_publisher
/robot_state_publisher
/sync_slam_toolbox_node
/ydlidar_ros2_driver_node
/rviz2
```

**If all nodes are present:** ✅ System is ready!

**If nodes are missing:** ❌ Go back to Terminal 1 and check for errors

```bash
# Verify topics are publishing
ros2 topic list
```

**Expected Topics:**
```
/cmd_vel
/imu/data_raw
/joint_states
/map
/map_metadata
/robot_description
/scan
/tf
/tf_static
/ultrasonic
```

```bash
# Check LiDAR publishing rate
ros2 topic hz /scan
# Expected: average rate: 7.000 Hz (wait 5 seconds for measurement)

# Check IMU publishing rate
ros2 topic hz /imu/data_raw
# Expected: average rate: 50.000 Hz
```

✅ **All checks pass?** Continue to manual driving!

---

### TERMINAL 3: Manual Teleoperation (Keyboard Control)

Open a **NEW Ubuntu Terminal** window:

```bash
# Source environment
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Install teleop keyboard if not already installed
sudo apt install ros-humble-teleop-twist-keyboard -y

# Start keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
```
Reading from keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i = Move forward
k = Stop
j = Turn left (rotate in place)
l = Turn right (rotate in place)
u = Forward + turn left
o = Forward + turn right
m = Backward + turn left
, = Backward
. = Backward + turn right

q/z = increase/decrease max speeds by 10%
w/x = increase/decrease only linear speed by 10%
e/c = increase/decrease only angular speed by 10%

CTRL-C to quit
```

**Important:** Keep this terminal window **FOCUSED** when driving! Keypresses only work when the terminal is active.

---

### 🗺️ Mapping Instructions

Now drive the robot to build a map of your room:

#### Strategy: Square Pattern

**Drive the robot in a large square:**

1. **Forward (3 meters)** 
   - Press and hold `i` for ~5 seconds
   - Watch robot move forward
   - Press `k` to stop

2. **Turn Left (90°)**
   - Press `j` briefly (1-2 seconds)
   - Robot rotates left
   - Press `k` to stop

3. **Forward (3 meters)**
   - Press `i` for ~5 seconds
   - Press `k` to stop

4. **Turn Left (90°)**
   - Press `j` briefly
   - Press `k` to stop

5. **Forward (3 meters)**
   - Press `i` for ~5 seconds
   - Press `k` to stop

6. **Turn Left (90°)**
   - Press `j` briefly
   - Press `k` to stop

7. **Forward (3 meters)**
   - Return to approximately starting position
   - Press `k` to stop

**Total driving time:** 3-5 minutes

**Tips:**
- 🐌 **Drive slowly** - gives SLAM time to process
- 🛑 **Stop at corners** - better for map quality
- 👀 **Watch RViz** - see map building in real-time
- 🔄 **Overlap paths** - improves map accuracy

---

### 📊 Watch Map Build in RViz

As you drive, observe RViz window:

**Timeline:**
- **0-30 seconds:** May see just some lines (walls)
- **1 minute:** Wall outlines starting to form
- **2 minutes:** Room outline clearly visible
- **3 minutes:** Detailed room map with furniture
- **5 minutes:** Complete, high-quality map

**What You Should See:**
- ✅ **White areas:** Free space (robot can move here)
- ✅ **Black/dark areas:** Obstacles/walls
- ✅ **Gray areas:** Unknown (not yet scanned)
- ✅ **Red/green dots:** Live LiDAR scan
- ✅ **Robot model:** Moving as you drive

**Map Quality Indicators:**
- ✅ Walls are continuous (not broken)
- ✅ Room shape is recognizable
- ✅ Straight walls appear straight
- ✅ Map is stable (not jumping around)

---

### 💾 Save Map Locally (IMPORTANT!)

Once you have a good map (after 3-5 minutes of driving):

#### TERMINAL 4: Save the Map

Open a **NEW Ubuntu Terminal** window:

```bash
# Source environment
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Install map saver if not already installed
sudo apt install ros-humble-nav2-map-server -y

# Create maps directory
mkdir -p ~/maps

# Save the map with timestamp
MAP_NAME="phase3_test_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f ~/maps/$MAP_NAME

# Or save with a simple name
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room_map
```

**Expected Output:**
```
[INFO] [map_saver]: Waiting for map to be available...
[INFO] [map_saver]: Received map
[INFO] [map_saver]: Map saved to ~/maps/my_room_map.pgm and ~/maps/my_room_map.yaml
```

**Files Created:**
- `~/maps/my_room_map.yaml` - Map metadata (origin, resolution, etc.)
- `~/maps/my_room_map.pgm` - Map image (grayscale image file)

#### Verify Map Files

```bash
# List saved maps
ls -lh ~/maps/

# Should show:
# my_room_map.yaml  (~200 bytes)
# my_room_map.pgm   (~50-500 KB depending on map size)

# View map metadata
cat ~/maps/my_room_map.yaml
```

**Expected YAML content:**
```yaml
image: my_room_map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

#### View Map Image (Optional)

```bash
# Install image viewer if needed
sudo apt install eog -y

# Open map image
eog ~/maps/my_room_map.pgm &
```

You should see your room map as a black-and-white image!

---

## ✅ Success Criteria

Your Phase 3 test is **SUCCESSFUL** if:

- [x] ✅ All nodes launched without errors
- [x] ✅ RViz displays robot model and grid
- [x] ✅ Robot responds to keyboard controls
- [x] ✅ LiDAR data visible in RViz (red/green dots)
- [x] ✅ Map starts appearing after 30 seconds
- [x] ✅ Walls clearly visible after 2 minutes
- [x] ✅ Map is coherent (not jumping around)
- [x] ✅ At least 50% of room is mapped
- [x] ✅ Map saved successfully to `~/maps/`
- [x] ✅ Both .yaml and .pgm files created
- [x] ✅ Map image viewable and recognizable

---

## 🐛 Troubleshooting

### Problem: RViz Shows Black Screen

**Solution:**
```bash
# In RViz menu:
# 1. Click Displays → Global Options → Fixed Frame
# 2. Change to: map or base_footprint
# 3. Click Views → Reset View
```

### Problem: Robot Not Moving

**Check teleop terminal:**
- Is the terminal window focused? (click on it)
- Are you pressing the correct keys? (i/j/k/l)

**Test manually:**
```bash
# In Terminal 2:
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
# Robot should move forward briefly
```

### Problem: No Map Appearing

**Check SLAM is running:**
```bash
# Terminal 2:
ros2 topic hz /map
# Should show 1-5 Hz

# If no output, restart system (Ctrl+C in Terminal 1, relaunch)
```

### Problem: Map Jumping/Unstable

**Solutions:**
- Drive more slowly
- Stop at corners before turning
- Check IMU drift (gyro values should be near 0)
- Drive in areas with more features (walls, furniture)

### Problem: Map Saving Failed

**Check if map server is installed:**
```bash
sudo apt install ros-humble-nav2-map-server -y
```

**Check if /map topic exists:**
```bash
ros2 topic list | grep map
# Should show: /map, /map_metadata
```

**Try saving again with full path:**
```bash
ros2 run nav2_map_server map_saver_cli -f /home/pi/maps/test_map
```

---

## 📈 Performance Monitoring (Terminal 2)

While driving, monitor system health:

```bash
# Watch all topic rates
ros2 topic hz /scan           # Should be ~7 Hz
ros2 topic hz /imu/data_raw   # Should be ~50 Hz
ros2 topic hz /map            # Should be 1-5 Hz
ros2 topic hz /cmd_vel        # Should match your driving

# Check latest scan data
ros2 topic echo /scan | head -50

# Check IMU stability
ros2 topic echo /imu/data_raw --once

# Check map size
ros2 topic echo /map_metadata --once
```

---

## 🎬 When Test is Complete

### 1. Save the Map (if not done already)

```bash
# Terminal 4:
ros2 run nav2_map_server map_saver_cli -f ~/maps/phase3_final
```

### 2. Document Test Results

```bash
# Create test log
cat > ~/maps/phase3_test_log.txt << EOF
═══════════════════════════════════════════════════════════
PHASE 3 TEST RESULTS
═══════════════════════════════════════════════════════════
Date: $(date)
Tester: $(whoami)

TEST RESULTS:
✅ System launched successfully
✅ Robot responded to controls  
✅ Map built successfully
✅ Map saved to: ~/maps/

PERFORMANCE:
LiDAR Rate: _____ Hz (expected: 7 Hz)
IMU Rate: _____ Hz (expected: 50 Hz)
Map Coverage: _____% (expected: >50%)
Driving Time: _____ minutes

ISSUES ENCOUNTERED:
[List any problems here]

NOTES:
[Additional observations]
═══════════════════════════════════════════════════════════
EOF

# View log
cat ~/maps/phase3_test_log.txt
```

### 3. Stop All Terminals

```bash
# Terminal 3 (teleop): Press Ctrl+C
# Terminal 2 (monitoring): Press Ctrl+C
# Terminal 1 (main launch): Press Ctrl+C

# Wait 5 seconds for graceful shutdown
```

### 4. Backup Maps

```bash
# Copy maps to timestamped backup
cp -r ~/maps ~/maps_backup_$(date +%Y%m%d_%H%M%S)

# List all saved maps
ls -lh ~/maps/
```

---

## 📊 What You Should Have After Phase 3

### Files Created:
```
~/maps/
├── my_room_map.pgm          # Map image
├── my_room_map.yaml         # Map metadata
├── phase3_final.pgm         # Final map
├── phase3_final.yaml        # Final map metadata
└── phase3_test_log.txt      # Test results
```

### Terminal Logs:
- System launch output (Terminal 1)
- Node status checks (Terminal 2)  
- Teleoperation session (Terminal 3)
- Map saving output (Terminal 4)

### Knowledge Gained:
- ✅ How to launch the full system
- ✅ How to manually control the robot
- ✅ How to build a map with SLAM
- ✅ How to save maps locally
- ✅ How to monitor system health
- ✅ Troubleshooting common issues

---

## 🎯 Next Steps

After completing Phase 3 successfully:

### Phase 4: Autonomous Frontier Exploration

Once you're ready to test autonomous exploration:

```bash
# Launch with exploration enabled
ros2 launch fea_slam robot_full.launch.py exploration:=true

# Robot will automatically explore the room!
# Watch in RViz as it:
# - Detects frontiers (green spheres)
# - Navigates to unexplored areas
# - Builds complete map autonomously
```

**Before Phase 4:**
- Ensure robot starts in **middle** of room (not corner)
- Clear 3m radius around robot
- Have emergency stop ready (Ctrl+C)
- Expected duration: 5-15 minutes depending on room size

### Map Reuse

Your saved map can be used for:
- Navigation with Nav2
- Path planning
- Localization (robot finding itself on map)
- Comparison with future maps

```bash
# View saved map
eog ~/maps/my_room_map.pgm

# Load map in RViz (for navigation)
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/pi/maps/my_room_map.yaml
```

---

## 📝 Quick Reference

### Terminal Setup (All Terminals Need This)
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Essential Commands
```bash
# Launch system
ros2 launch fea_slam robot_full.launch.py exploration:=false

# Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Check nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor rates
ros2 topic hz /scan
```

### Keyboard Controls (Teleop)
```
i = forward     j = left      l = right
k = stop        , = backward
```

---

## 🚨 Emergency Procedures

### Robot Moving Uncontrollably
```bash
# Immediately in any terminal:
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once

# Or press Ctrl+C in Terminal 1 to stop everything
```

### System Frozen/Stuck
```bash
# Kill all ROS2 processes
pkill -9 -f ros2

# Reset and restart
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py exploration:=false
```

---

**Good luck with your Phase 3 testing! 🚀**

**Remember:** Take your time, drive slowly, and save your maps!

---

**Last Updated:** January 13, 2026  
**Status:** Ready for Testing  
**Expected Duration:** 15-20 minutes
