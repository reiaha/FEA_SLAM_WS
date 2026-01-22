# 🚀 Phase 4: Autonomous Frontier Exploration - Start Guide

**Date:** January 21, 2026  
**Test Type:** Autonomous Frontier-Based Exploration  
**Terminal:** Ubuntu Terminal (NOT VSCode)  
**Status:** Ready to Launch

---

## 🎯 Phase 4 Overview

Phase 4 is **Autonomous Frontier Exploration** where the robot will:
1. ✅ Automatically detect unexplored areas (frontiers)
2. ✅ Select and navigate to frontiers autonomously
3. ✅ Build a complete map without manual control
4. ✅ Complete exploration when no more frontiers exist

**Expected Duration:** 5-20 minutes (depends on room size)

---

## ⚠️ IMPORTANT: Prerequisites

Before starting Phase 4, you MUST have completed:
- ✅ **Phase 1:** Pre-flight checks (hardware connected)
- ✅ **Phase 2:** Component testing (sensors working)
- ✅ **Phase 3:** Manual mapping (system integration verified)

**If you haven't completed Phase 3, do it first!**

---

## 🔧 Pre-Phase 4 Checklist

### 1. Hardware Verification
```bash
# Check Arduino connection
ls -la /dev/ttyACM0
# Expected: crw-rw---- 1 root dialout

# Check LiDAR connection
ls -la /dev/ttyUSB0
# Expected: crw-rw---- 1 root dialout

# Fix permissions if needed
sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0
```

### 2. Physical Setup
- [ ] Robot positioned in **MIDDLE** of room (NOT in corner!)
- [ ] Clear 2m radius around robot
- [ ] Battery fully charged
- [ ] All cables secure
- [ ] Flat, smooth floor
- [ ] Good lighting
- [ ] Ready to monitor robot (can reach emergency stop)

### 3. Software Setup
```bash
# Navigate to workspace
cd /home/pi/FEA_SLAM_WS

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify packages
ros2 pkg list | grep -E 'fea_slam|localization'
# Should show: fea_slam, localization
```

### 4. Install Nav2 (Required for Autonomous Navigation)
```bash
# Install if not already installed
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# Verify installation
ros2 pkg list | grep nav2
# Should show multiple nav2_* packages
```

---

## 🚀 Launch Phase 4: Autonomous Exploration

### TERMINAL 1: Launch Full System with Exploration ENABLED

```bash
# Navigate to workspace
cd /home/pi/FEA_SLAM_WS

# Source setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with exploration ENABLED (THIS IS PHASE 4!)
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

**⏱️ Wait 15-20 seconds** for system initialization...

### Expected Terminal Output

```
[INFO] [launch]: All log files can be found below /home/pi/.ros/log/...
[INFO] [robot_state_publisher-1]: process started with pid [xxx]
[INFO] [joint_state_publisher-2]: process started with pid [xxx]
[INFO] [arduino_motor_bridge-3]: process started with pid [xxx]
[INFO] [ydlidar_ros2_driver_node-4]: process started with pid [xxx]
[INFO] [sync_slam_toolbox_node-5]: process started with pid [xxx]
[INFO] [frontier_detector-6]: process started with pid [xxx]
[INFO] [exploration_coordinator-7]: process started with pid [xxx]
[INFO] [rviz2-8]: process started with pid [xxx]

[arduino_motor_bridge]: Arduino connected on /dev/ttyACM0
[ydlidar_ros2_driver_node]: Lidar successfully connected
[ydlidar_ros2_driver_node]: Now lidar is scanning...
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver

[frontier_detector-6]: 🔍 Frontier Detector initialized
[exploration_coordinator-7]: 🧭 Exploration Coordinator initialized
[exploration_coordinator-7]: Waiting for map data...
```

**✅ RViz Window Opens:** You should see:
- Robot model in center
- Gray grid floor
- LiDAR scan (white/red dots)
- **GREEN SPHERES** = Frontiers (unexplored areas)

---

## 👀 What to Watch For

### Terminal Messages (Good Signs)
```
[frontier_detector-6] 🔍 Found 5 frontier clusters
[exploration_coordinator-7] 📍 Selected closest frontier at distance: 1.45m
[exploration_coordinator-7] 🚀 Sending goal to frontier: (1.23, 0.89)
[exploration_coordinator-7] ✨ Goal accepted, robot navigating to frontier...

# After 10-30 seconds:
[exploration_coordinator-7] ✅ Successfully reached frontier!
[frontier_detector-6] 🔍 Found 7 frontier clusters
[exploration_coordinator-7] 📍 Selected closest frontier at distance: 2.10m
```

### RViz Display (Good Signs)
- ✅ **Green spheres** appear at map boundaries
- ✅ Robot moves toward nearest sphere
- ✅ White laser scan dots moving/updating
- ✅ Map expands as robot moves
- ✅ New green spheres appear in expanded areas
- ✅ Old spheres disappear as robot reaches them

### Robot Behavior (Good Signs)
- ✅ Robot starts moving within 30 seconds
- ✅ Moves smoothly toward frontiers
- ✅ Stops briefly at frontiers, then moves to next
- ✅ No erratic movements or spinning

---

## 🔍 Monitoring the Exploration (Optional)

Open **NEW Ubuntu Terminal** windows for monitoring:

### TERMINAL 2: Watch Frontier Detection
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Monitor frontiers
ros2 topic echo /frontiers

# You'll see MarkerArray with frontier positions:
# markers:
#   - id: 0
#     pose:
#       position:
#         x: 2.34
#         y: 1.56
#         z: 0.0
```

### TERMINAL 3: Watch Current Goal
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Monitor current navigation goal
ros2 topic echo /current_frontier_goal

# Shows which frontier robot is heading to:
# point:
#   x: 2.34
#   y: 1.56
#   z: 0.0
```

### TERMINAL 4: Check Map Updates
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check map publishing rate
ros2 topic hz /map

# Should show: average rate: 1-5 Hz
```

### TERMINAL 5: Monitor Robot Velocity
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Watch velocity commands
ros2 topic echo /cmd_vel

# Should show linear and angular velocities while moving
```

---

## ⏱️ Exploration Timeline

### What to Expect Over Time

**0-30 seconds:**
- System initializes
- SLAM builds initial map from current position
- Frontier detector analyzes map
- First frontiers detected

**0-2 minutes:**
- Robot starts moving to first frontier
- Green spheres visible in RViz
- Map expands as robot moves
- New frontiers appear

**2-5 minutes:**
- Robot explores multiple frontiers
- Map grows significantly
- Walls and obstacles become clear
- Exploration accelerates

**5-10 minutes:**
- Deeper exploration of room
- Most major areas mapped
- Fewer frontiers remaining
- Robot may navigate longer distances

**10-20 minutes:**
- Fine-tuning exploration
- Small unexplored gaps
- Approaching completion
- Frontier count decreasing

**Completion:**
```
[exploration_coordinator-7] ⭐ Exploration Complete!
[exploration_coordinator-7] No more frontiers to explore
[exploration_coordinator-7] Total frontiers explored: 47
[exploration_coordinator-7] Exploration duration: 782.3 seconds
```

---

## ✅ Success Criteria

Your Phase 4 test is **SUCCESSFUL** if:

1. ✅ Robot moves autonomously without keyboard input
2. ✅ Green frontier spheres appear in RViz
3. ✅ Robot reaches at least 3 frontiers
4. ✅ Terminal shows "Successfully reached frontier!" messages
5. ✅ Map expands continuously
6. ✅ Robot completes exploration OR reaches time limit (10 minutes default)
7. ✅ At least 70% of room is mapped

---

## ⏹️ Stopping Exploration

### Option 1: Let It Complete Naturally
```
Wait for terminal message:
[exploration_coordinator-7] ⭐ Exploration Complete!

This is the best option - let the algorithm finish!
```

### Option 2: Manual Stop (Emergency)
```bash
# In Terminal 1 where you launched, press:
Ctrl+C

# Robot will stop immediately
# All nodes will shut down
```

### Option 3: Time Limit (Automatic)
```
Default: Exploration stops after 600 seconds (10 minutes)

You'll see:
[exploration_coordinator-7] ⏰ Time limit reached (600s)
[exploration_coordinator-7] Stopping exploration
```

---

## 💾 Save the Map (After Completion)

After exploration completes or you stop it:

### TERMINAL (NEW):
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash

# Create maps directory if needed
mkdir -p ~/maps

# Save the map with timestamp
MAP_NAME="autonomous_exploration_$(date +%Y%m%d_%H%M%S)"
ros2 run nav2_map_server map_saver_cli -f ~/maps/$MAP_NAME

# Expected output:
# Saving map to ~/maps/autonomous_exploration_20260121_143022.pgm
# Map saved successfully!
```

### Verify Map Files
```bash
ls -lh ~/maps/

# Should show two files:
# autonomous_exploration_20260121_143022.pgm (image)
# autonomous_exploration_20260121_143022.yaml (metadata)
```

---

## 🔧 Troubleshooting Phase 4

### Problem: No frontiers detected
**Solution:**
```bash
# Robot may need initial map. Do manual drive first:
# 1. Stop current launch (Ctrl+C)
# 2. Launch with exploration:=false
# 3. Drive manually for 2-3 minutes to build initial map
# 4. Stop and relaunch with exploration:=true
```

### Problem: Robot doesn't move
**Possible Causes:**
1. Nav2 not installed → Run: `sudo apt install ros-humble-nav2-bringup`
2. No map data yet → Wait 30 seconds for SLAM to initialize
3. No frontiers → Map may be complete already
4. Arduino disconnected → Check `/dev/ttyACM0`

**Check:**
```bash
# Verify Nav2 installed
ros2 pkg list | grep nav2

# Verify /cmd_vel publishing
ros2 topic hz /cmd_vel
```

### Problem: Robot stuck in corner
**Solution:**
```bash
# Manual intervention:
# 1. Press Ctrl+C to stop
# 2. Manually move robot to center of room
# 3. Relaunch system
```

### Problem: "Goal failed" messages repeatedly
**Possible Causes:**
1. Obstacle in path
2. Map quality issues
3. Navigation parameters need tuning

**Solution:**
```bash
# Try changing selection method:
ros2 launch fea_slam robot_full.launch.py \
  exploration:=true \
  frontier_selection_method:=gain
```

### Problem: Too many false frontiers
**Solution:**
```bash
# Increase threshold (reduces sensitivity):
ros2 launch fea_slam robot_full.launch.py \
  exploration:=true \
  frontier_threshold:=60
```

### Problem: Very slow exploration
**Solution:**
```bash
# Lower threshold (more aggressive):
ros2 launch fea_slam robot_full.launch.py \
  exploration:=true \
  frontier_threshold:=40
```

---

## 📊 Expected Results

### Small Room (5m × 5m)
- **Time:** 2-3 minutes
- **Frontiers:** 10-20
- **Coverage:** 90-95%

### Medium Room (10m × 10m)
- **Time:** 8-12 minutes
- **Frontiers:** 30-50
- **Coverage:** 80-90%

### Large Room (15m × 15m)
- **Time:** 15-20 minutes
- **Frontiers:** 50-80
- **Coverage:** 70-85%

### Complex Layout (with furniture)
- **Time:** 15-30 minutes
- **Frontiers:** 60-100+
- **Coverage:** 60-75%

---

## 📝 Test Log Template

Record your Phase 4 results:

```
═══════════════════════════════════════════════════════════
PHASE 4: AUTONOMOUS FRONTIER EXPLORATION TEST LOG
═══════════════════════════════════════════════════════════

Date: ________________
Time Started: ________________
Room Size: ________________
Room Layout: [ ] Empty [ ] Furnished [ ] Complex

RESULTS:
─────────────────────────────────────────────────────────
Start Time: ________________
End Time: ________________
Total Duration: _______ minutes

Frontiers Detected: _______
Frontiers Explored: _______
Success Rate: _______% (explored/detected)

Map Coverage: _______% 

System Performance:
- Robot moved autonomously: [ ] Yes [ ] No
- Green spheres visible: [ ] Yes [ ] No
- Map expanded continuously: [ ] Yes [ ] No
- Exploration completed: [ ] Yes [ ] Timeout [ ] Manual stop

Issues Encountered:
_____________________________________________
_____________________________________________

Overall Result: [ ] SUCCESS [ ] PARTIAL [ ] FAILED

Map Saved To: _________________________________

Notes:
_____________________________________________
_____________________________________________
_____________________________________________
```

---

## ✨ Next Steps After Phase 4

Once Phase 4 is complete:

1. **Review the map** - Check coverage and quality
2. **Analyze performance** - Review logs and terminal output
3. **Document results** - Fill out test log
4. **Save data** - Keep map files for future use
5. **Proceed to Phase 5** - Verification tests and fine-tuning

---

## 🎓 Understanding the Technology

### What is Frontier-Based Exploration?
Frontiers are boundaries between:
- **Known free space** (explored, clear areas)
- **Unknown space** (not yet explored)

The robot:
1. Detects these boundaries on the map
2. Selects the most promising frontier
3. Navigates to it using Nav2
4. Repeats until no frontiers remain

### Selection Strategies

**Closest (Default):**
- Greedy approach
- Selects nearest frontier
- Fast but may not be optimal
- Good for small/medium rooms

**Gain-Based:**
- Considers exploration value
- Balances distance vs. information gain
- Slower but more efficient
- Good for large/complex spaces

---

## 🎯 You're Ready!

Everything is configured and ready for Phase 4.

**To start:**
```bash
cd /home/pi/FEA_SLAM_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

**Then watch the magic happen!** 🤖✨

Good luck with your autonomous exploration test!
