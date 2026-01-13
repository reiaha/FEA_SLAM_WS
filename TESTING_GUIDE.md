# 🧪 FEA-SLAM Robot - Complete Testing Guide

**Date:** January 7, 2026  
**System:** FEA-SLAM Robot with YDLiDAR S2PRO + Arduino + Motors  
**Status:** Ready for Hardware Testing

---

## 📋 Table of Contents

1. [Pre-Flight Checks](#phase-1-pre-flight-checks)
2. [Component Testing](#phase-2-component-testing)
3. [System Integration](#phase-3-system-integration-test)
4. [Autonomous Exploration](#phase-4-autonomous-frontier-exploration)
5. [Verification Tests](#phase-5-verification-tests)
6. [Troubleshooting](#troubleshooting-guide)
7. [Testing Checklist](#full-testing-checklist)

---

## PHASE 1: Pre-Flight Checks

**Time Required:** 5 minutes

### Step 1: Verify Hardware Connections
```bash
# Check Arduino connection
ls -la /dev/ttyACM0

# Expected output: crw-rw---- 1 root dialout ... /dev/ttyACM0
# If NOT found: Arduino not connected or wrong port
```

### Step 2: Verify LiDAR Connection
```bash
# Check YDLiDAR connection
ls -la /dev/ttyUSB0

# Expected output: crw-rw---- 1 root dialout ... /dev/ttyUSB0
# If NOT found: YDLiDAR not connected or wrong port
```

### Step 3: Visual Hardware Check
- [ ] Robot is on flat, stable surface
- [ ] All wheels spin freely by hand
- [ ] Arduino LED lights are on (red/green)
- [ ] YDLiDAR motor spins (visual check)
- [ ] Battery fully charged (visual indicator or voltage test)
- [ ] All cables securely connected
- [ ] No loose parts or damaged wires

### Step 4: Clear Testing Area
- [ ] 2m radius around robot is clear
- [ ] No obstacles, furniture, or people
- [ ] Good lighting for visibility
- [ ] Smooth, flat floor (no stairs/ramps)
- [ ] Safe to let robot move in any direction

### Step 5: Source ROS2 Setup
```bash
source /opt/ros/humble/setup.bash
source /home/pi/FEA_SLAM_WS/install/setup.bash

# Verify setup
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep FEA_SLAM_WS
ros2 pkg list | grep -E 'fea_slam|localization'
# Note: ROS_PACKAGE_PATH can be empty on ROS2; AMENT_PREFIX_PATH is the one to check
```

### Step 6: Verify Build Status
```bash
# Check if packages compiled successfully
colcon build --packages-select localization fea_slam 2>&1 | tail -5

# Expected: "Summary: 2 packages finished [X.XXs]"
```

### Step 7: Note Current Time
```bash
date
# Record this - helps track test duration later
```

**✅ Proceed to Phase 2 when all checks pass**

---

## PHASE 2: Component Testing

**Time Required:** 30 minutes

### TEST 1: Arduino Serial Communication

#### Setup
```bash
# Open terminal
ls -l /dev/ttyACM*

# Start serial monitor (sudo avoids dialout permission issues)
sudo screen /dev/ttyACM0 115200

# If screen immediately terminates, quick fallback to confirm data:
# sudo cat /dev/ttyACM0 | head
```

#### What You'll See
```
Streaming sensor data every 20ms:
1.2,0.5,9.8,-2.8,-1.4,0.1,45,0,0
0.9,0.3,9.8,-2.7,-1.4,0.2,45,0,0
1.1,0.4,9.8,-2.8,-1.5,0.1,46,0,0

Format: ax,ay,az,gx,gy,gz,distance,motorA,motorB
```

#### Expected Values
- **ax, ay, az**: Accelerometer (m/s²) - varies with movement
- **az when stationary**: Should be close to 9.8 (gravity)
- **gx, gy, gz**: Gyroscope (deg/s) - should be near 0 (calibrated)
  - Expected: gx ≈ -2.8, gy ≈ -1.4, gz ≈ -1.1
- **distance**: Ultrasonic (cm) - changes as you move objects near sensor
- **motorA, motorB**: Motor PWM values (0 when not moving)
- **Update Rate**: New line every ~20ms (50 Hz)

#### Success Criteria
```
✅ Data streaming continuously
✅ Values update every 20ms
✅ az close to 9.8 (±0.5)
✅ gx, gy, gz close to 0 (±1.0)
✅ distance changes when you move hand near sensor
✅ motorA, motorB show 0 (no movement yet)
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No output | Arduino not connected | Check USB cable, verify `/dev/ttyACM0` exists |
| Garbage characters | Wrong baud rate | Verify 115200 in Arduino IDE settings |
| Values static | IMU not responding | Check I2C connections (A4/A5), reset Arduino |
| High gyro drift | Calibration wrong | Update offsets in `scripts/arduino_mpu6050.ino` |
| Distance always 0 | Ultrasonic not working | Check pins 8/9, verify sensor power |

#### Exit Serial Monitor
```
# Press Ctrl+A, then type: :quit
```

---

### TEST 2: LiDAR Connection and Data

#### Terminal 1: Launch LiDAR Driver
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# Expected output:
# [INFO] YDLiDAR ROS2 Driver...
# [INFO] Open port /dev/ttyUSB0 success!
# [INFO] start scan...
```

#### Terminal 2: Check Topic Publishing Rate
```bash
ros2 topic hz /scan

# Expected: ~7.0 Hz
# Output should show: average rate: 7.000 Hz
```

#### Terminal 3: View Scan Data
```bash
ros2 topic echo /scan | head -30

# Expected output (first few lines):
# header:
#   seq: 1234
#   stamp:
#     sec: 1705000000
#     nsec: 123456789
#   frame_id: laser_frame
# angle_min: 0.0
# angle_max: 6.28...
# angle_increment: 0.0174...
# ranges: [12.0, 11.5, 11.2, ..., 0.5, 0.4]  (400+ values)
```

#### Success Criteria
```
✅ LiDAR launches without errors
✅ /scan topic publishing at ~7 Hz
✅ Scan contains 400+ range measurements
✅ Range values between 0.1m and 12m
✅ Data updates continuously
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Port open failed | Wrong port or driver issue | Check `/dev/ttyUSB0`, restart YDLiDAR |
| Low frequency (<5Hz) | Baud rate wrong | Set to 115200 in `ydlidar.yaml` |
| LiDAR not spinning | Power issue | Check USB power, verify connections |
| Huge ranges (>20m) | Sensor malfunction | Check LiDAR lens for dust, restart |
| No ranges/zeros | TF frame mismatch | Check laser_frame TF is published |

#### Stop LiDAR
```bash
# Press Ctrl+C in Terminal 1
```

---

### TEST 3: Motor Control (BE CAREFUL!)

#### Terminal 1: Launch System
```bash
# IMPORTANT: exploration:=false (no autonomous movement yet!)
ros2 launch fea_slam robot_full.launch.py exploration:=false

# Wait 5 seconds for all nodes to start
```

#### Check System Started
```bash
# Terminal 2: Verify nodes
ros2 node list

# Should show:
# /robot_state_publisher
# /joint_state_publisher
# /arduino_motor_bridge
# /ydlidar_ros2_driver_node
# /sync_slam_toolbox_node
# /rviz2
```

#### Terminal 3: Send Small Test Command
```bash
# TEST 1: Move forward slowly (SAFE)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once

# Watch robot - should move forward very slowly for ~1 second
```

#### Expected Behavior
- ✅ Both wheels rotate forward
- ✅ Robot moves slowly (~0.1 m/s)
- ✅ Movement stops after 1 second (command expires)
- ✅ No jerky or erratic motion

#### More Movement Tests
```bash
# TEST 2: Turn left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# TEST 3: Turn right
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: -0.5}}" --once

# TEST 4: Move backward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1}}" --once

# TEST 5: Diagonal motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}" --once
```

#### Success Criteria
```
✅ All motors respond to /cmd_vel
✅ Robot moves smoothly without jerking
✅ Turns are balanced (both wheels move evenly)
✅ Movement matches velocity direction
✅ Robot stops after command expires
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No movement | Arduino not connected | Verify `/dev/ttyACM0`, check serial cable |
| One wheel faster | Motor speed imbalance | Adjust `wheel_base` or motor calibration |
| Jerky movement | Software/timing issue | Reduce `max_speed` parameter in launch file |
| Opposite direction | Motor wiring reversed | Swap wires on motor connector |
| Very slow response | Serial latency | Check USB power, reduce command frequency |

#### Stop All Movement
```bash
# Send zero velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once

# Or press Ctrl+C to stop system
```

---

### TEST 4: IMU Data Quality

#### Terminal 1: System Running (from TEST 3)
```bash
# Should still be running from previous test
ros2 launch fea_slam robot_full.launch.py exploration:=false
```

#### Terminal 2: Monitor IMU Data
```bash
# Subscribe to IMU data
ros2 topic echo /imu/data_raw

# You'll see (update every ~200ms with --rate=5):
# linear_acceleration:
#   x: 0.024
#   y: -0.018
#   z: 9.824
# angular_velocity:
#   x: -0.024  (should be near -2.8 offset value)
#   y: -0.019  (should be near -1.4 offset value)
#   z: 0.008   (should be near -1.1 offset value)
```

#### Keep Robot Stationary
```bash
# Don't move robot for this test
# Let IMU readings stabilize for 30 seconds
```

#### Check Stability
```bash
# Watch gyro values for 1 minute
# They should vary ±0.5 deg/s max
# Any larger variation = calibration issue
```

#### Success Criteria
```
✅ IMU publishes at 50Hz
✅ Angular velocity (gyro) near 0 (±0.5 deg/s)
✅ Acceleration z-axis near 9.8 m/s² (gravity)
✅ Values stable and not drifting
✅ No NaN or inf values
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Gyro drifts | Calibration offset wrong | Update in `scripts/arduino_mpu6050.ino`: gx=-2.8, gy=-1.4, gz=-1.1 |
| High noise | Vibration from motors | Isolate IMU with rubber standoffs |
| No data | I2C connection | Check Arduino pins A4/A5 for MPU-6050 |
| Values frozen | I2C lockup | Restart Arduino |

---

## PHASE 3: System Integration Test

**Time Required:** 15-20 minutes

### TEST 5: Manual Mapping

#### Objective
Build an initial map by manually driving the robot around the room.

#### Terminal 1: Launch Full System
```bash
# Make sure to disable autonomous exploration!
ros2 launch fea_slam robot_full.launch.py exploration:=false

# Wait 10 seconds for RViz to open
# You should see RViz window with robot model
```

#### Terminal 2: Start Manual Teleoperation
```bash
# Install if needed:
sudo apt install ros-humble-teleop-twist-keyboard

# Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Controls:
# i = forward       u = forward+turn left    o = forward+turn right
# k = stop          ,/. = backward+turn left/right
# j = turn left     l = turn right
# < = go faster     > = go slower
# q/z = increase/decrease max angular velocity
```

#### RViz Expected Display
```
You should see:
✅ Gray floor grid (coordinate system)
✅ Robot model (wheels, base, sensors)
✅ Colored axes (red/green/blue = X/Y/Z)
✅ LiDAR data as red/green dots
✅ Empty map on left side
```

#### Mapping Instructions
```
1. Position robot in center of room
2. Drive forward 2-3 meters
3. Stop and turn 90° left
4. Drive 2-3 meters
5. Stop and turn 90° left
6. Drive 2-3 meters
7. Stop and turn 90° left
8. Drive 2-3 meters
9. Stop and turn 90° left
10. Return to approximate starting position

Total time: 3-5 minutes of driving
```

#### Watch the Map Build
```
RViz Map Display:
- First 30 seconds: May see just lines (walls)
- After 1 minute: Clearer wall outline
- After 2 minutes: Room outline visible
- After 3 minutes: Good room map with details
- After 5 minutes: Complete room map
```

#### Success Criteria
```
✅ RViz displays robot and grid
✅ Robot moves smoothly when you press keys
✅ Map starts appearing (gray/white areas)
✅ Walls clearly visible after 2 minutes
✅ Map is coherent (not jumping around)
✅ At least 50% of room mapped
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| RViz shows nothing | Wrong fixed frame | Change RViz fixed frame to `base_footprint` |
| Black screen | Map not loading | Ensure SLAM Toolbox launched |
| Robot not moving | Teleop not working | Verify teleop window is focused, try moving mouse there |
| Map jumping | Localization issue | Move more slowly, check IMU drift |
| Map is inverted | TF frame issue | Check base_footprint→base_link orientation |

#### Stop When Done
```bash
# Press Ctrl+C in teleop terminal
```

---

## PHASE 4: Autonomous Frontier Exploration

**Time Required:** 5-15 minutes (depending on room size)

### TEST 6: Enable Frontier Exploration

#### Important Setup
```
BEFORE YOU START:
✅ Robot positioned in MIDDLE of room (not corner)
✅ Area around robot is clear
✅ You are ready to monitor robot movement
✅ Have emergency stop plan (Ctrl+C)
```

#### Terminal 1: Launch with Exploration ENABLED
```bash
# This is the main test!
ros2 launch fea_slam robot_full.launch.py exploration:=true

# Wait 15 seconds for initialization
# You should see output like:

# [robot_state_publisher-1]: process started with pid [XXXX]
# [joint_state_publisher-2]: process started with pid [XXXX]
# [arduino_motor_bridge-3]: process started with pid [XXXX]
# [ydlidar_ros2_driver_node-4]: process started with pid [XXXX]
# [sync_slam_toolbox_node-5]: process started with pid [XXXX]
# [frontier_detector-6]: Frontier Detector initialized
# [exploration_coordinator-7]: Exploration Coordinator initialized
```

#### Watch Terminal Output
```bash
# You should see frontier detection:
[frontier_detector-6] Found 3 frontier clusters
[exploration_coordinator-7] 📍 Selected closest frontier at distance: 1.45m
[exploration_coordinator-7] 🚀 Sending goal to frontier: (1.23, 0.89)
[exploration_coordinator-7] ✨ Goal accepted, robot navigating to frontier...

# After 10-30 seconds:
[exploration_coordinator-7] ✅ Successfully reached frontier!
[exploration_coordinator-7] 📍 Selected closest frontier at distance: 2.10m
```

#### Watch RViz Display
```
You should see:
✅ Green spheres appear (frontier markers)
✅ Each sphere at an unknown area boundary
✅ Robot moves toward nearest sphere
✅ As robot moves, new scans appear (white dots)
✅ Map expands showing newly explored areas
✅ New frontiers appear in expanded areas
```

#### Exploration Progress
```
Timeline of autonomous exploration:

0-2 min:  Initial frontiers detected, robot starts moving
2-5 min:  Robot reaching first frontiers, map expanding
5-10 min: Multiple frontiers explored, map growing
10+ min:  Exploring deeper areas, may reach completion

Watch for:
- Robot consistently moving toward green spheres
- Spheres disappearing as robot reaches them
- Map continuously expanding
- New green spheres appearing in expanded areas
```

#### Success Criteria
```
✅ Terminal shows frontier detection (Found X clusters)
✅ Robot moves autonomously without user input
✅ RViz shows green frontier spheres
✅ Robot reaches at least 3 frontiers
✅ Map expands as robot explores
✅ Terminal shows "Successfully reached frontier!" messages
✅ Eventually shows: "⭐ Exploration Complete!" OR timeout
```

#### Expected Completion Time
```
Small room (5×5m):      2-3 minutes
Medium room (10×10m):   8-12 minutes
Large room (15×15m):    15-20 minutes
Very large (20×20m):    25-40 minutes
```

#### Failure Troubleshooting
| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| No frontiers detected | SLAM not mapping | Drive manually first to build map, then restart |
| Robot doesn't move | Nav2 not installed | Run: `sudo apt install ros-humble-nav2-bringup` |
| "Goal failed" messages | Obstacle in path | Manually move robot, clear path |
| Robot stuck in corner | Local minima (dead end) | Manually move robot out, restart exploration |
| Very slow exploration | frontier_threshold too high | Lower to 40 in launch parameters |
| Too many false frontiers | Clutter creating noise | Increase frontier_threshold to 60 |

#### Monitor During Exploration

**Terminal 2: Watch Frontiers**
```bash
ros2 topic echo /frontiers

# You'll see MarkerArray with sphere data:
# markers:
#   - id: 0
#     pose:
#       position:
#         x: 2.34
#         y: 1.56
#         z: 0.0
#     scale:
#       x: 0.15  # size = importance
```

**Terminal 3: Watch Current Goal**
```bash
ros2 topic echo /current_frontier_goal

# Shows which frontier robot is heading toward:
# point:
#   x: 2.34
#   y: 1.56
#   z: 0.0
```

**Terminal 4: Check Map Update Rate**
```bash
ros2 topic hz /map

# Should show 1-5 Hz (updates as SLAM processes)
```

#### Stop Exploration
```bash
# Option 1: Let it finish naturally
# Wait for: "⭐ Exploration Complete!"

# Option 2: Early stop
# Press Ctrl+C in Terminal 1
```

---

## PHASE 5: Verification Tests

**Time Required:** Varies (run while exploration active)

### TEST 7: Frontier Detection Accuracy

#### What to Verify
```bash
# Monitor frontiers during exploration
ros2 topic echo /frontiers

# For each frontier sphere:
# ✅ Position is at boundary of known/unknown map
# ✅ Should NOT be inside walls
# ✅ Should NOT be inside explored area
# ✅ Sphere size = cluster size (more important = bigger)
```

#### Visual Check in RViz
```
Compare green spheres to:
✅ Sphere positions match wall/obstacle edges
✅ No spheres in the middle of explored areas
✅ Larger spheres are more important targets
✅ Spheres disappear as frontier is explored
```

---

### TEST 8: Navigation Success Rate

#### Measurement Method
```bash
# Count successful frontier reaches during exploration
# Watch terminal output and count:

✅ = Successfully reached frontier!  (SUCCESS)
❌ = Failed to reach frontier (FAILURE)

Record:
- Total goals attempted
- Total successfully reached
- Total failed
- Success rate = (successful / total) × 100%
```

#### Expected Success Rate
```
Typical results:
- Cluttered room: 70-80% success
- Open room: 90%+ success
- Very tight spaces: 50-70% success
```

#### If Low Success Rate
```
Troubleshooting:
- Check for obstacles in planned paths
- Verify wheel traction (slipping?)
- Check ultrasonic sensor for false readings
- Adjust navigation parameters in launch file
```

---

### TEST 9: Map Coverage Percentage

#### Calculation Method
```bash
# After exploration (or let it run 10 minutes):

ros2 topic echo /map | head -1000 > /tmp/map_data.txt

# Count map cells:
# - Unknown cells (< 50): to explore
# - Free cells (0-50): explored clear areas
# - Occupied cells (> 50): walls/obstacles

# Coverage = (free cells / total cells) × 100%
```

#### Expected Coverage
```
Small empty room:      95%+ coverage
Room with furniture:   80-90% coverage
Complex layout:        70-85% coverage
With dead-ends:        60-75% coverage
```

---

### TEST 10: Exploration Completion Time

#### Recording
```bash
# Note start time when launching with exploration:=true
START_TIME=$(date +%s)

# Note end time when you see:
# "⭐ Exploration Complete!"
END_TIME=$(date +%s)

# Calculate duration:
DURATION=$((END_TIME - START_TIME))
echo "Exploration took $DURATION seconds (or $((DURATION/60)) minutes)"
```

#### Comparison to Expected
```
Room Size    | Expected Time | Your Time | Status
5×5m        | 2-3 min      | ?????     | ✅/⚠️
10×10m      | 8-12 min     | ?????     | ✅/⚠️
15×15m      | 15-20 min    | ?????     | ✅/⚠️
20×20m      | 25-40 min    | ?????     | ✅/⚠️
```

---

## Troubleshooting Guide

### General Issues

**Problem: RViz shows nothing**
```
Solutions:
1. Check RViz fixed frame = base_footprint
   - RViz menu → Displays → Global Options → Fixed Frame
2. Enable all display layers
   - RViz left panel: check boxes for Map, TF, RobotModel, LaserScan
3. Reset RViz view
   - RViz menu → View → Reset View
4. Check SLAM is running
   - ros2 node list | grep slam
```

**Problem: No /map topic**
```
Solutions:
1. Verify SLAM Toolbox launched
   - ros2 topic list | grep map
2. Drive robot manually for 2 minutes to build map
3. Check SLAM log for errors
   - ros2 node info /sync_slam_toolbox_node
```

**Problem: Robot doesn't respond to commands**
```
Solutions:
1. Check Arduino connection
   - screen /dev/ttyACM0 115200
2. Verify /cmd_vel is being published
   - ros2 topic hz /cmd_vel
3. Check battery power
   - Measure voltage with multimeter
4. Reset Arduino
   - Unplug USB for 5 seconds, reconnect
```

### Frontier Exploration Issues

**Problem: No frontiers detected**
```
Solutions:
1. Manual map building first
   - Drive robot with teleop for 2-3 minutes
2. Lower frontier_threshold
   - Change from 50 to 40 in robot_full.launch.py
3. Check /map is updating
   - ros2 topic hz /map (should be 1-5 Hz)
```

**Problem: Robot doesn't move to frontiers**
```
Solutions:
1. Install Nav2
   - sudo apt install ros-humble-nav2-bringup
2. Check navigation goal is being sent
   - ros2 topic echo /current_frontier_goal
3. Verify /cmd_vel being published
   - ros2 topic hz /cmd_vel
```

**Problem: Exploration takes too long**
```
Solutions:
1. Change selection strategy
   - Set frontier_selection_method: 'closest' (default)
2. Increase frontier_threshold
   - Set to 60 to ignore small uncertain areas
3. Lower min_frontier_size
   - Set to 3 to accept small frontiers
```

**Problem: Robot stuck in corner**
```
Solutions (during exploration):
1. Manually push robot out
2. Press Ctrl+C to stop exploration
3. Move robot to center of room
4. Restart exploration
```

---

## Full Testing Checklist

Print this section and check off as you complete each test:

### Pre-Flight Checks (PHASE 1)
```
☐ Hardware connected (/dev/ttyACM0, /dev/ttyUSB0 visible)
☐ Robot powered on and battery charged
☐ 2m radius clear of obstacles
☐ Smooth flat testing surface
☐ Good lighting for visibility
☐ ROS2 setup sourced (source install/setup.bash)
☐ Packages built successfully (colcon build output shows 2 packages)
☐ Current date/time recorded for test log
```

### Component Tests (PHASE 2)

**TEST 1: Arduino Serial Data**
```
☐ screen /dev/ttyACM0 115200 shows continuous data
☐ Data updates every ~20ms
☐ az value near 9.8 when robot stationary
☐ gx, gy, gz values near 0 (within ±1.0)
☐ distance value changes when you move hand near sensor
☐ motorA, motorB show 0 (motors not moving yet)
☐ Successfully exited screen (Ctrl+A, :quit)
```

**TEST 2: LiDAR Data**
```
☐ ros2 launch ydlidar_ros2_driver ydlidar_launch.py starts without errors
☐ ros2 topic hz /scan shows ~7.0 Hz publishing rate
☐ ros2 topic echo /scan shows 400+ range values
☐ Range values between 0.1m and 12m
☐ Data continuous (no long gaps)
☐ Successfully stopped (Ctrl+C)
```

**TEST 3: Motor Control**
```
☐ ros2 launch fea_slam robot_full.launch.py exploration:=false starts
☐ ros2 node list shows all nodes running
☐ Motor test 1 (forward): robot moves forward
☐ Motor test 2 (turn left): robot turns left
☐ Motor test 3 (turn right): robot turns right
☐ Motor test 4 (backward): robot moves backward
☐ Motor test 5 (diagonal): robot moves diagonally
☐ Movement is smooth (not jerky)
☐ Both wheels move evenly
```

**TEST 4: IMU Quality**
```
☐ ros2 topic echo /imu/data_raw --rate=5 shows data
☐ Angular velocity values near 0 (±0.5 deg/s)
☐ Z-axis acceleration near 9.8 m/s² (gravity)
☐ Values stable during 1 minute observation
☐ No NaN or inf values in output
☐ Publishing rate confirmed 50Hz
```

### System Integration (PHASE 3)

**TEST 5: Manual Mapping**
```
☐ ros2 launch fea_slam robot_full.launch.py exploration:=false starts
☐ RViz window opens showing robot model
☐ RViz shows grid, TF axes, robot parts
☐ ros2 run teleop_twist_keyboard teleop_twist_keyboard starts
☐ Robot responds to keyboard controls
☐ Drove robot in 4-side square pattern
☐ Map appeared in RViz after 1 minute
☐ Wall outlines visible after 2 minutes
☐ Room clearly mapped after 3-5 minutes
☐ Map appears stable (not jumping around)
☐ At least 50% of room visible in map
```

### Autonomous Exploration (PHASE 4)

**TEST 6: Frontier Exploration**
```
☐ Robot positioned in MIDDLE of room (not corner)
☐ Clear area around robot (2m minimum)
☐ ros2 launch fea_slam robot_full.launch.py exploration:=true starts
☐ Wait 15 seconds for all nodes to initialize
☐ Terminal shows "Frontier Detector initialized"
☐ Terminal shows "Exploration Coordinator initialized"
☐ Terminal shows "Found X frontier clusters"
☐ Terminal shows "Selected closest frontier" message
☐ RViz shows green sphere markers (frontiers)
☐ Robot begins moving autonomously (no keyboard input)
☐ Watch terminal for: "Successfully reached frontier!" message
☐ Robot moves to new frontier, repeats process
☐ At least 3 frontiers explored
☐ Exploration completes OR reaches time limit
☐ Terminal shows final status message
☐ Date and duration recorded for test log
```

### Verification Tests (PHASE 5)

**TEST 7: Frontier Detection**
```
☐ Opened ros2 topic echo /frontiers during exploration
☐ Verified frontier positions at map boundaries
☐ Confirmed no false positives (spheres not in explored areas)
☐ Sphere sizes corresponded to frontier importance
☐ Spheres disappeared as robot explored them
```

**TEST 8: Navigation Success**
```
☐ Counted successful frontier reaches: _____/_____ = ____% success
☐ Recorded any failed navigation attempts
☐ Checked for obstacles in robot's path
☐ Success rate acceptable (>70%)
```

**TEST 9: Map Coverage**
```
☐ Measured/estimated final map coverage: _____% 
☐ Compared to expected coverage for room size
☐ Walls clearly visible and accurate
☐ No major gaps in coverage (except dead-ends)
```

**TEST 10: Completion Time**
```
☐ Recorded start time: ___________
☐ Recorded completion time: ___________
☐ Total duration: _____ minutes
☐ Compared to expected for room size
☐ Time acceptable/reasonable: ✅/⚠️
```

---

## Test Log Template

Record your test results here:

```
═══════════════════════════════════════════════════════════
FEA-SLAM ROBOT TEST LOG
═══════════════════════════════════════════════════════════

Date: ________________
Tester Name: ________________
Location: ________________
Robot ID: ________________

ENVIRONMENT
───────────────────────────────────────────────────────────
Room Size: ________________
Room Layout: ✅ Empty  ✅ Furniture  ✅ Obstacles
Surface: ✅ Flat  ✅ Carpet  ✅ Uneven
Lighting: ✅ Good  ✅ Dim  ✅ Dark

PHASE 1: PRE-FLIGHT
───────────────────────────────────────────────────────────
Arduino (/dev/ttyACM0): ✅ Found / ❌ Not found
LiDAR (/dev/ttyUSB0): ✅ Found / ❌ Not found
Battery: ✅ Full / ⚠️ Partial / ❌ Low
Area cleared: ✅ Yes / ❌ No
Packages built: ✅ Yes / ❌ No
Result: ✅ PASSED / ❌ FAILED

PHASE 2: COMPONENT TESTS
───────────────────────────────────────────────────────────
Test 1 - Arduino Serial:
  Data streaming: ✅ Yes / ❌ No
  Update rate: ✅ 50Hz / ❌ Other: ___
  Sensor values normal: ✅ Yes / ❌ No
  Result: ✅ PASSED / ❌ FAILED

Test 2 - LiDAR:
  Publishing to /scan: ✅ Yes / ❌ No
  Frequency: _____ Hz (expected: 7 Hz)
  Range count: _____ points (expected: 400+)
  Result: ✅ PASSED / ❌ FAILED

Test 3 - Motors:
  Forward movement: ✅ Yes / ❌ No
  Turning: ✅ Yes / ❌ No
  Smoothness: ✅ Good / ⚠️ Jerky / ❌ Not moving
  Result: ✅ PASSED / ❌ FAILED

Test 4 - IMU:
  Publishing data: ✅ Yes / ❌ No
  Gyro stability: ✅ Stable / ⚠️ Drifting
  Z-axis gravity: ✅ ~9.8 / ❌ Wrong
  Result: ✅ PASSED / ❌ FAILED

Overall Phase 2 Result: ✅ PASSED / ❌ FAILED

PHASE 3: SYSTEM INTEGRATION
───────────────────────────────────────────────────────────
Test 5 - Manual Mapping:
  System launched: ✅ Yes / ❌ No
  RViz displays: ✅ Yes / ❌ No
  Teleop working: ✅ Yes / ❌ No
  Map started building: ✅ Yes / ❌ No
  Final map coverage: ✅ >50% / ⚠️ 25-50% / ❌ <25%
  Result: ✅ PASSED / ❌ FAILED

Overall Phase 3 Result: ✅ PASSED / ❌ FAILED

PHASE 4: AUTONOMOUS EXPLORATION
───────────────────────────────────────────────────────────
Test 6 - Frontier Exploration:
  System launched: ✅ Yes / ❌ No
  Frontiers detected: ✅ Yes / ❌ No
  Green spheres visible: ✅ Yes / ❌ No
  Robot moved autonomously: ✅ Yes / ❌ No
  Frontiers explored: _____ (expected: 3+)
  Exploration completed: ✅ Yes / ⏱️ Timeout / ❌ Error
  Duration: _____ minutes (expected: see table)
  Result: ✅ PASSED / ❌ FAILED

Overall Phase 4 Result: ✅ PASSED / ❌ FAILED

PHASE 5: VERIFICATION
───────────────────────────────────────────────────────────
Test 7 - Frontier Accuracy: ✅ PASSED / ❌ FAILED
Test 8 - Navigation Success: _____% (expected: >70%)
Test 9 - Map Coverage: _____% (expected: see table)
Test 10 - Completion Time: _____ min (expected: see table)

Overall Phase 5 Result: ✅ PASSED / ❌ FAILED

═══════════════════════════════════════════════════════════
FINAL RESULTS
═══════════════════════════════════════════════════════════

Overall Test Result: ✅ ALL PASSED / ⚠️ PARTIAL / ❌ FAILED

Issues Encountered:
────────────────────────────────────────────────────────────
1. ________________
2. ________________
3. ________________

Solutions Applied:
────────────────────────────────────────────────────────────
1. ________________
2. ________________
3. ________________

Recommendations for Next Steps:
────────────────────────────────────────────────────────────
1. ________________
2. ________________
3. ________________

Notes:
────────────────────────────────────────────────────────────
[Any additional observations or comments]

Tester Signature: ________________________  Date: ________
```

---

## Success Criteria Summary

### Minimum Success Criteria
```
✅ All Phase 1 pre-flight checks pass
✅ All Phase 2 component tests show correct values
✅ Phase 3 system launches and builds map
✅ Phase 4 frontier exploration begins and completes
```

### Excellent Success
```
✅ All of above PLUS:
✅ Map coverage >90%
✅ Navigation success >85%
✅ Exploration completes within expected time
✅ Zero critical errors in logs
```

### Issues to Investigate
```
⚠️ Navigation success <70%
⚠️ Map coverage <50%
⚠️ Exploration takes 2x expected time
⚠️ Frequent timeout errors
⚠️ High sensor noise/drift
```

---

## Next Steps After Testing

1. **Document Results**
   - Complete the test log above
   - Take photos/videos of exploration
   - Save terminal output logs

2. **Analyze Performance**
   - Compare actual to expected times
   - Measure map quality
   - Identify any systematic issues

3. **Fine-Tuning**
   - Adjust parameters based on results
   - Try different frontier selection methods
   - Optimize for your specific environment

4. **Report Generation**
   - Create report with test results
   - Include photos and performance graphs
   - Document any lessons learned

---

**Last Updated:** January 7, 2026
**Status:** Ready for Real Robot Testing
**Expected Test Duration:** 1-2 hours for complete test suite

Good luck with your testing! 🚀

