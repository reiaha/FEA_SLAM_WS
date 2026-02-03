# FEA-SLAM v2 - Quick Start Guide (8-Phase Architecture)

## What's New

**Complete implementation of all 8 phases from your FEA-SLAM thesis:**

```
Phase 1: System Initialization Phase
Phase 2: Continuous SLAM and Localization Loop (EKF sensor fusion)
Phase 3: Frontier Detection Logic (BFS clustering)
Phase 4: Global Path Planning Logic (A* search)
Phase 5: Local Motion Planning and Control (DWA via Nav2)
Phase 6: Dynamic Obstacle Handling Logic
Phase 7: Failure Recovery Logic
Phase 8: Mission Completion Logic
```

---

## Installation

### Step 1: Update Code

```bash
cd ~/FEA_SLAM_WS
rm -rf build install log
colcon build --packages-select localization fea_slam
source install/setup.bash
```

### Step 2: Verify Installation

```bash
# Check if v2 node is available
ros2 run localization exploration_coordinator_v2 --ros-args --help
```

You should see output listing parameters like:
- `max_exploration_time`
- `frontier_selection_method` 
- `nav2_init_delay`
- `localization_confidence_threshold`
- etc.

---

## Launch Configurations

### Option 1: Manual Testing (Recommended for First Run)

**Terminal 1: Core System (no exploration yet)**
```bash
cd ~/FEA_SLAM_WS
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py \
  exploration:=false \
  rviz:=true \
  slam:=true \
  lidar:=true \
  arduino_bridge:=true
```

This starts:
- ✅ YDLiDAR driver
- ✅ Nav2 navigation stack
- ✅ SLAM Toolbox (autonomous mapping)
- ✅ Arduino motor bridge (movement)
- ✅ RViz visualization

**Terminal 2: Test Robot Movement**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 5
```

Robot should move forward slowly. Check:
- `/scan` topic in RViz (red dots show LiDAR)
- `/map` topic in RViz (gray grid shows known space)
- `/tf` transforms (verify base_link → map)

**Manually move robot around for 30-60 seconds to build initial map.**

### Option 2: Full Autonomous Exploration

Once you've verified core system works:

```bash
cd ~/FEA_SLAM_WS
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py \
  exploration:=true \
  rviz:=true
```

This also starts:
- ✅ Phase 1: Frontier Detector
- ✅ Phase 2: Ultrasonic Explorer (bootstrap SLAM, ~2 min)
- ✅ Phase 3: Exploration Coordinator v2 (all 8 phases)

---

## System Flow

### Phase 1: System Initialization (0-30 seconds)

```
[Console Output]
==============================================================
FEA-SLAM Exploration Coordinator v2 Initialized
Phase 1: System Initialization
Selection method: astar
Coverage threshold: 85%
==============================================================
⏳ Phase 1: Waiting for Nav2... (28s remaining)
⏳ Phase 1: Waiting for Nav2... (23s remaining)
⏳ Phase 1: Waiting for Nav2... (18s remaining)
...
✅ Phase 1 Complete: System Initialized
```

**What's happening:**
- Waiting for Nav2 action server to be ready
- LiDAR collecting scans
- SLAM building initial map
- IMU providing motion estimates

### Phase 2: SLAM & Localization Loop (Continuous Background)

```
[Console Output - every 2 seconds]
[exploration_coordinator_v2]: Updated robot pose: x=0.12 y=-0.05 θ=1.23
[exploration_coordinator_v2]: SLAM confidence: 0.78 (good)
```

**What's happening:**
- EKF sensor fusion combines LiDAR + IMU
- Pose estimate updated from TF (map → base_link)
- Localization confidence calculated from map coverage
- If confidence < 0.7 → Phase 7 (Recovery) triggered

### Phase 3: Frontier Detection (Every 2 seconds)

```
[Console Output - when exploration starts]
[frontier_detector]: BFS clustering on occupancy grid
[frontier_detector]: Detected frontier cluster: size=47 cells at (1.23, 2.45)
[frontier_detector]: Detected frontier cluster: size=32 cells at (0.87, 3.12)
[exploration_coordinator_v2]: 🎯 Detected 2 frontier clusters
```

**What's happening:**
- Occupancy grid analyzed for unknown cells
- BFS clustering groups adjacent frontiers
- Each frontier scored by distance + information gain
- Best frontier selected for navigation

### Phase 4: Global Path Planning (A*) - ~100ms

```
[Console Output]
[exploration_coordinator_v2]: Computing A* path...
[exploration_coordinator_v2]: ✅ A* path found: 28 cells
[exploration_coordinator_v2]: 📍 Sending goal to frontier #1
```

**What's happening:**
- A* search on occupancy grid
- Path validation checks
- Goal pose computed from frontier centroid
- Frontier marked as "visited" before sending goal

**[RViz Visualization]**
- Red arrow appears showing frontier goal
- Path displayed as line on map

### Phase 5: Local Motion Control (DWA) - Real-time

```
[Console Output]
[exploration_coordinator_v2]: Goal accepted by Nav2
[exploration_coordinator_v2]: Goal reached!
```

**What's happening:**
- Nav2 controller executes DWA
- Robot follows computed path
- Real-time collision avoidance
- Motor commands sent to Arduino at 10Hz

**[Robot Behavior]**
- Robot drives toward frontier
- Avoids obstacles detected by LiDAR
- Adjusts path smoothly

### Phase 6: Obstacle Handling (Reactive)

```
[Console Output - if blocked]
🚧 Persistent obstacle detected - triggering global replanning
```

**What's happening:**
- If robot blocked > 5 cycles
- Cancels current goal
- Selects different frontier
- Returns to Phase 3

### Phase 7: Recovery (If Localization Fails)

```
[Console Output - if SLAM confidence drops]
⚠️  SLAM confidence low: 0.45 < 0.70
🔄 Attempting recovery (attempt 1)...
[Robot rotates in place for 2 seconds]
✅ Localization recovered!
```

**What's happening:**
- Detects poor SLAM state
- Attempts to re-acquire features
- Up to 3 attempts before aborting
- Saves map and terminates if recovery fails

### Phase 8: Mission Completion

```
[Console Output - when all frontiers explored]
ℹ️  No frontiers detected - checking for completion

🎉 MISSION COMPLETE!
   Coverage: 87.5%
   Frontiers visited: 12
   Recovery attempts: 0
🗺️  Saving final map to: ~/FEA_SLAM_WS/saved_maps/exploration_2026-01-30_143022/
✅ Map saved
```

**What's happening:**
- All reachable frontiers explored
- Coverage exceeds threshold (85%)
- Final map saved with timestamp
- Robot stops
- Mission complete log printed

---

## Configuration Parameters

Edit parameters in `/home/pi/FEA_SLAM_WS/src/fea_slam/launch/robot_full.launch.py`:

```python
exploration_coordinator_node = Node(
    ...
    parameters=[{
        # ===== MISSION CONTROL =====
        'max_exploration_time': 3600.0,             # 1 hour max mission time
        
        # ===== FRONTIER SELECTION =====
        'frontier_selection_method': 'astar',       # Options:
                                                    #  'bfs' - greedy distance
                                                    #  'astar' - distance + size
                                                    #  'gain' - maximize size
        
        # ===== INITIALIZATION =====
        'nav2_init_delay': 30.0,                    # Wait 30s for Nav2 startup
        
        # ===== SLAM HEALTH =====
        'localization_confidence_threshold': 0.7,   # Minimum SLAM confidence
        
        # ===== FRONTIER SCORING =====
        'frontier_score_weight_distance': 0.4,      # Prefer closer frontiers (0-1)
        'frontier_score_weight_gain': 0.6,          # Prefer larger frontiers (0-1)
        
        # ===== OBSTACLE HANDLING =====
        'obstacle_persistence_threshold': 5,        # Replan if blocked 5+ cycles
        
        # ===== COMPLETION =====
        'coverage_threshold': 0.85                  # Mission complete at 85% coverage
    }],
)
```

### Tuning Guide

**If robot revisits areas:**
- ↑ Increase `frontier_score_weight_distance` to 0.6
- ↓ Decrease `frontier_score_weight_gain` to 0.4

**If robot gets stuck:**
- ↓ Decrease `obstacle_persistence_threshold` from 5 to 3
- ↑ Increase `nav2_init_delay` from 30 to 45

**If SLAM loses localization:**
- ↓ Lower `localization_confidence_threshold` from 0.7 to 0.5
- Increase robot movement speed to get more LiDAR features

**For faster exploration:**
- Change `frontier_selection_method` to 'bfs' (greedy, faster)
- Increase weights: distance=0.5, gain=0.5

**For more complete maps:**
- Lower `coverage_threshold` from 0.85 to 0.80
- Increase `max_exploration_time` from 3600 to 7200 (2 hours)

---

## Monitoring System Health

### Check Phase Transitions

```bash
# Terminal 1: Watch console output
ros2 launch fea_slam robot_full.launch.py exploration:=true | grep -E "Phase|🎯|🗺️|✅|❌"
```

Expected sequence:
```
Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6 → (repeat 3-6) → Phase 8
```

### Check Key Topics

```bash
# Terminal 2: Monitor robot pose
ros2 topic echo /tf

# Terminal 3: Monitor map
ros2 topic echo /map --rate 0.5

# Terminal 4: Monitor frontiers
ros2 topic echo /frontiers

# Terminal 5: Monitor navigation
ros2 topic echo /navigate_to_pose/_action/status
```

### Check SLAM Health

```bash
# Check map metadata
ros2 topic echo /map/metadata

# Check SLAM status
ros2 topic list | grep slam
```

---

## Troubleshooting

### Problem: Phase 1 timeout (waiting for Nav2 > 60 seconds)

**Cause:** Nav2 not initializing properly

**Solution:**
```bash
# Kill all nodes
pkill -f ros2

# Check Nav2 logs
ros2 launch fea_slam robot_full.launch.py exploration:=false | grep -i nav2

# Rebuild if needed
cd ~/FEA_SLAM_WS && colcon build --packages-select nav2_bringup
```

### Problem: No frontiers detected

**Cause:** Threshold too high or map not building

**Solution:**
```bash
# Check map in RViz - should see gray grid
ros2 launch fea_slam robot_full.launch.py exploration:=false rviz:=true

# Manually move robot around - watch map build
# Once map is visible, start exploration with:
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

### Problem: Robot stuck in Phase 6 (obstacle handling) repeatedly

**Cause:** Frontier unreachable (blocked permanently)

**Solution:**
- Decrease `obstacle_persistence_threshold` from 5 to 3
- Robot will try different frontier sooner

### Problem: Recovery fails (Phase 7 aborts mission)

**Cause:** SLAM too degraded to recover

**Solution:**
```bash
# Rotate robot manually to re-acquire features
# Wait for confidence to recover
# Restart exploration manually:
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

### Problem: Map not saving at mission end

**Cause:** SaveMap service unavailable

**Solution:**
```bash
# Check service is running
ros2 service list | grep save_map

# If missing, start SLAM launcher:
ros2 launch slam_toolbox online_async.launch.py
```

---

## Testing Phases Individually

### Test Phase 1 (Initialization) Only

```bash
timeout 35 ros2 launch fea_slam robot_full.launch.py \
  exploration:=true rviz:=false
```

Watch console for "Phase 1 Complete"

### Test Phase 2 (SLAM) Only

```bash
ros2 launch fea_slam robot_full.launch.py \
  exploration:=false rviz:=true

# Manually move robot with:
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" -r 5

# Watch /map update in RViz
```

### Test Phase 3 (Frontier Detection) Only

```bash
# Start basic system
ros2 launch fea_slam robot_full.launch.py exploration:=false rviz:=true

# In another terminal, record a bag of the robot moving
ros2 bag record /scan /map /tf /tf_static -o my_exploration_bag

# Later, replay and test frontier detection:
ros2 bag play my_exploration_bag
ros2 run localization frontier_detector
```

### Test Phase 4 (A* Planning) Only

```bash
# Create a simple test script that:
# 1. Loads occupancy grid from map
# 2. Tests A* from (0,0) to (5,5)
# 3. Prints path

# See exploration_coordinator_v2.py method: compute_astar_path()
```

---

## Next Steps

1. **Run Phase 1 test** - verify Nav2 initializes
2. **Run Phase 2 test** - verify SLAM builds map
3. **Run Phase 3 test** - verify frontier detection
4. **Run full exploration** - all phases together
5. **Monitor mission completion** - robot stops when frontiers exhausted
6. **Save and analyze final maps** - in ~/FEA_SLAM_WS/saved_maps/

---

## Performance Targets

| Phase | Target Time | Status |
|-------|-------------|--------|
| Phase 1 | 30 seconds | ⏳ Waiting for Nav2 |
| Phase 2 | Background | ⚙️ Continuous |
| Phase 3 | 2 Hz (500ms) | 🎯 Detecting frontiers |
| Phase 4 | 100 ms | 📍 Planning path |
| Phase 5 | Real-time | 🚀 Following path |
| Phase 6 | <100 ms | 🚧 Obstacle check |
| Phase 7 | 2-5 seconds | 🔄 Recovery attempt |
| Phase 8 | 5 seconds | 🎉 Completion |

**Total mission time:** 20-60 minutes (depends on environment size)

---

## Getting Help

Check logs with:
```bash
# Full coordinator logs
ros2 launch fea_slam robot_full.launch.py exploration:=true 2>&1 | tee mission.log

# View log later:
grep "Phase" mission.log
grep "🎯" mission.log
grep "❌" mission.log
```

All phase transitions are logged with `[exploration_coordinator_v2]` prefix.

---

**System ready for autonomous FEA-SLAM exploration!** 🚀
