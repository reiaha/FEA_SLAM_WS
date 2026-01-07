# FEA-SLAM Robot Full System Guide

## What to Expect When Running `robot_full.launch.py`

---

## Startup Sequence (10-15 seconds)

### Terminal Output:
```bash
[INFO] [launch]: All log files can be found below /home/pi/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [xxx]
[INFO] [joint_state_publisher-2]: process started with pid [xxx]
[INFO] [arduino_motor_bridge-3]: process started with pid [xxx]
[INFO] [ydlidar_ros2_driver_node-4]: process started with pid [xxx]
[INFO] [sync_slam_toolbox_node-5]: process started with pid [xxx]
[INFO] [rviz2-6]: process started with pid [xxx]
```

### Component Initialization:
- **Robot State Publisher** (2s): Loads URDF, publishes robot model
- **Arduino Motor Bridge** (3s): Connects to `/dev/ttyACM0`, starts reading sensors
- **YDLiDAR** (4s): Connects to LiDAR, starts scanning at 7Hz
- **SLAM Toolbox** (2s): Initializes mapping system, waits for scans
- **RViz** (5s): Opens visualization window

### RViz Window Opens:
You'll see a 3D window with:
- ✅ **Grid** - Gray floor grid
- ✅ **Robot Model** - Your robot (wheels, plates, sensors)
- ✅ **TF Frames** - Coordinate axes (red/green/blue arrows)
- ⏳ **Map** - Empty at first, builds as robot moves
- ⏳ **LaserScan** - White/colored dots from LiDAR

---

## System Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                        HARDWARE LAYER                        │
├─────────────────────────────────────────────────────────────┤
│  Arduino (MPU6050 + Motors + Ultrasonic) ←→ /dev/ttyACM0   │
│  YDLiDAR S2PRO ←→ /dev/ttyUSB0                              │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 NODES LAYER                        │
├─────────────────────────────────────────────────────────────┤
│ 1. arduino_motor_bridge:                                    │
│    • Reads: Arduino serial data (50Hz)                      │
│    • Publishes: /imu/data_raw, /ultrasonic                  │
│    • Subscribes: /cmd_vel                                   │
│    • Sends: Motor commands to Arduino                       │
│                                                              │
│ 2. ydlidar_ros2_driver_node:                                │
│    • Reads: LiDAR serial data                               │
│    • Publishes: /scan                                       │
│    • Publishes TF: lidar_link → laser_frame                 │
│                                                              │
│ 3. robot_state_publisher:                                   │
│    • Reads: /robot_description (URDF)                       │
│    • Publishes TF: base_footprint → base_link → sensors     │
│                                                              │
│ 4. sync_slam_toolbox_node:                                  │
│    • Subscribes: /scan                                      │
│    • Publishes: /map, /map_metadata                         │
│    • Publishes TF: map → base_footprint                     │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                    VISUALIZATION LAYER                       │
├─────────────────────────────────────────────────────────────┤
│  RViz2: Displays robot model, map, laser scans, TF tree     │
└─────────────────────────────────────────────────────────────┘
```

---

## Expected Behavior

### When Everything Works:

**1. Terminal shows:**
```
[arduino_motor_bridge]: Arduino connected on /dev/ttyACM0
[ydlidar_ros2_driver_node]: Lidar successfully connected [/dev/ttyUSB0:115200]
[ydlidar_ros2_driver_node]: Model: S2PRO
[ydlidar_ros2_driver_node]: Now lidar is scanning...
[slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
```

**2. RViz displays:**
- Robot model in center (wheels, plates, LiDAR visible)
- Green/blue TF axes showing coordinate frames
- White laser scan points around robot (obstacles/walls)
- Map starts empty, fills in as you move robot

**3. Robot responds:**
- You can send `/cmd_vel` commands
- Motors respond immediately
- IMU data streams continuously

---

## Common Errors & Solutions

### Error 1: Arduino Not Connected

**Error Message:**
```
[ERROR] [arduino_motor_bridge]: Failed to open /dev/ttyACM0: [Errno 2] No such file or directory
```

**Diagnosis:**
```bash
ls -l /dev/ttyACM*
# OR
ls -l /dev/ttyUSB*
```

**Solutions:**
1. **Arduino unplugged:** Plug in Arduino USB cable
2. **Wrong port:** Change to `/dev/ttyUSB0` if Arduino appears there
3. **Permission denied:** 
   ```bash
   sudo chmod 666 /dev/ttyACM0
   # OR permanently add user to dialout group
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```
4. **Arduino not programmed:** Upload `arduino_mpu6050.ino` first using Arduino IDE

---

### Error 2: LiDAR Timeout/Trembling

**Error Message:**
```
[ERROR] [ydlidar_ros2_driver_node]: Timeout count: 1, 2, 3...
```
OR motor physically trembles/stutters when spinning

**Diagnosis:**
```bash
# Check if LiDAR is detected
ls -l /dev/ttyUSB*
# Should show: /dev/ttyUSB0

# Check current baudrate setting
cat src/ydlidar_ros2_driver/params/ydlidar.yaml | grep baudrate
```

**Solutions:**
1. **Wrong baudrate:** Should be `115200` for S2PRO model
   - Edit: `src/ydlidar_ros2_driver/params/ydlidar.yaml`
   - Set: `baudrate: 115200`
   - Rebuild: `colcon build --packages-select ydlidar_ros2_driver`

2. **USB power insufficient:** Use powered USB hub or external power

3. **Frequency too high causing trembling:** 
   - Edit: `src/ydlidar_ros2_driver/params/ydlidar.yaml`
   - Change: `frequency: 7.0` (try lower: 5.0 or 6.0)
   - Rebuild package

4. **Motor physically trembling:** Check mechanical mounting and bearings

---

### Error 3: RViz Black Screen / Nothing Displays

**Error Message:**
```
[WARN] [robot_state_publisher]: Waiting for robot_description...
```
OR RViz opens but shows nothing

**Diagnosis:**
```bash
# Check if robot description is published
ros2 topic echo /robot_description --once

# Check TF tree
ros2 run tf2_ros tf2_echo map base_footprint
```

**Solutions:**
1. **URDF not loading:** 
   ```bash
   cd /home/pi/FEA_SLAM_WS
   colcon build --packages-select fea_slam
   source install/setup.bash
   ```

2. **Fixed Frame wrong in RViz:**
   - Open RViz
   - Left panel → Global Options → Fixed Frame
   - Change to: `base_footprint` or `map`

3. **Displays disabled:**
   - In RViz left panel (Displays)
   - Check boxes next to: Grid, RobotModel, TF, Map, LaserScan

4. **TF tree broken:** 
   ```bash
   ros2 node list
   # Should show all nodes: robot_state_publisher, ydlidar_ros2_driver_node, slam_toolbox
   ```

---

### Error 4: SLAM Dropping Messages

**Error Message:**
```
[INFO] [slam_toolbox]: Message Filter dropping message: frame 'laser_frame' at time... for reason 'discarding message because the queue is full'
```

**Diagnosis:**
```bash
# Check TF chain exists
ros2 run tf2_ros view_frames
# This generates frames.pdf showing the TF tree
```

**Solutions:**
1. **TF not published:** Missing `lidar_link → laser_frame` transform
   - Check YDLiDAR launch file publishes static TF
   
2. **Time synchronization issue:** SLAM can't find transform at scan timestamp
   - Usually resolves itself after a few seconds

3. **Usually not critical:** Messages eventually get through, map still builds

4. **If persistent (keeps dropping all messages):**
   - Stop: `Ctrl+C`
   - Restart: `ros2 launch fea_slam robot_full.launch.py`

---

### Error 5: Motors Don't Move

**Symptoms:**
- No error messages
- Send `/cmd_vel` but robot doesn't respond
- Arduino connected, no motor movement

**Diagnosis:**
```bash
# Test if Arduino is receiving commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Check Arduino serial output (Ctrl+A then K to exit)
screen /dev/ttyACM0 115200
# Should see: ACK:MOTOR:40,40 (or similar acknowledgment)
```

**Solutions:**
1. **Arduino not programmed with motor control:**
   - Upload the updated `arduino_mpu6050.ino` (with motor control code)

2. **Wrong serial port in launch file:**
   - Arduino bridge might be trying to connect to wrong device
   - Check: `src/fea_slam/launch/robot_full.launch.py`
   - Verify: `'serial_port': '/dev/ttyACM0'`

3. **Motor wiring incorrect:**
   - Verify TB6612FND connections:
     - Motor A: IN1=Pin2, IN2=Pin3, PWM=Pin5
     - Motor B: IN1=Pin4, IN2=Pin7, PWM=Pin6

4. **No external power to motors:**
   - TB6612FND needs external power (not USB)
   - Connect battery/power supply to VM pin

5. **PWM pins don't match wiring:**
   - Check your pin diagram matches Arduino code
   - Verify pins in `scripts/arduino_mpu6050.ino`

---

## Quick Debugging Commands

```bash
# 1. Check all nodes running
ros2 node list

# 2. Check all topics
ros2 topic list

# 3. Check topic data rates
ros2 topic hz /scan
ros2 topic hz /imu/data_raw

# 4. Check topic data (one message)
ros2 topic echo /cmd_vel --once
ros2 topic echo /imu/data_raw --once

# 5. Visualize TF tree (creates frames.pdf)
ros2 run tf2_tools view_frames

# 6. Monitor Arduino serial directly (Ctrl+A then K to exit)
screen /dev/ttyACM0 115200

# 7. Check device permissions
ls -l /dev/ttyACM0 /dev/ttyUSB0

# 8. Kill everything if system is stuck
pkill -f "ros2 launch"

# 9. Check ROS2 daemon
ros2 daemon status
ros2 daemon stop    # If needed
ros2 daemon start

# 10. Check available serial devices
ls -l /dev/tty*
```

---

## Expected Performance Metrics

| Component | Rate | Status Check Command |
|-----------|------|---------------------|
| Arduino IMU | 50Hz | `ros2 topic hz /imu/data_raw` |
| Ultrasonic | 50Hz | `ros2 topic hz /ultrasonic` |
| LiDAR Scan | 7Hz | `ros2 topic hz /scan` |
| SLAM Map Updates | 1-5Hz | Watch RViz map display |
| Motor Commands | On demand | `ros2 topic echo /cmd_vel` |
| RViz Frame Rate | 30Hz | Should be smooth visually |

---

## First Time Launch Checklist

Before running `robot_full.launch.py`, ensure:

- [ ] **Arduino programmed** with `scripts/arduino_mpu6050.ino`
- [ ] **Arduino connected** via USB (check `ls /dev/ttyACM*`)
- [ ] **LiDAR connected** via USB (check `ls /dev/ttyUSB*`)
- [ ] **Permissions set** for serial devices:
  ```bash
  sudo chmod 666 /dev/ttyACM0 /dev/ttyUSB0
  ```
  OR permanently:
  ```bash
  sudo usermod -a -G dialout $USER
  # Then logout and login
  ```
- [ ] **Workspace built**:
  ```bash
  cd /home/pi/FEA_SLAM_WS
  colcon build
  ```
- [ ] **Setup sourced** (in same terminal):
  ```bash
  source install/setup.bash
  ```
- [ ] **External power connected** to motor driver (if using motors)
- [ ] **Battery charged** (if robot is mobile)

---

## Running the Full System

**Step 1: Open Terminal**
```bash
cd /home/pi/FEA_SLAM_WS
source install/setup.bash
```

**Step 2: Launch the System**
```bash
ros2 launch fea_slam robot_full.launch.py
```

**Step 3: Wait for Initialization (10-15 seconds)**
- Watch terminal output
- Wait for "Now lidar is scanning..." message
- RViz should open automatically

**Step 4: Verify Everything Works**
```bash
# In a NEW terminal:
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

# Check all nodes are running
ros2 node list
# Expected output:
#   /arduino_motor_bridge
#   /joint_state_publisher
#   /map_saver
#   /robot_state_publisher
#   /slam_toolbox
#   /static_tf_pub_laser
#   /ydlidar_ros2_driver_node

# Check topics are publishing
ros2 topic list
# Expected topics:
#   /cmd_vel
#   /imu/data_raw
#   /map
#   /robot_description
#   /scan
#   /ultrasonic
```

**Step 5: Test Motor Control**
```bash
# Install teleop keyboard if not already installed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop (in new terminal with setup.bash sourced)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Use keyboard to control:
#   i = forward
#   k = stop
#   j = turn left
#   l = turn right
#   u/o = forward+turn
#   , = backward
```

OR send direct commands:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Turn right
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: -0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## RViz Usage

### Adjust View:
- **Rotate:** Left-click and drag
- **Pan:** Middle-click (or Shift+Left-click) and drag
- **Zoom:** Scroll wheel

### Enable/Disable Displays:
- Left panel → Displays
- Check/uncheck boxes to show/hide:
  - Grid
  - RobotModel
  - TF (transform axes)
  - Map
  - LaserScan

### Save Map (when mapping is done):
```bash
# In new terminal:
ros2 run nav2_map_server map_saver_cli -f ~/my_map
# This saves:
#   ~/my_map.yaml
#   ~/my_map.pgm
```

---

## Arduino Serial Commands Reference

From ROS2 to Arduino:

| Command | Format | Example | Action |
|---------|--------|---------|--------|
| Both motors | `MOTOR:speedA,speedB\n` | `MOTOR:150,150\n` | Set individual speeds |
| Forward | `FWD:speed\n` | `FWD:150\n` | Both motors same speed |
| Backward | `BWD:speed\n` | `BWD:100\n` | Both motors reverse |
| Turn Left | `LEFT:speed\n` | `LEFT:120\n` | Right motor on, left off |
| Turn Right | `RIGHT:speed\n` | `RIGHT:120\n` | Left motor on, right off |
| Stop | `STOP\n` | `STOP\n` | Stop all motors |

Speed range: `-255` to `255`
- Positive = forward
- Negative = backward
- 0 = stop

From Arduino to ROS2 (CSV format, 50Hz):
```
ax,ay,az,gx,gy,gz,distance,motorA,motorB
0.0234,-0.0156,0.9876,-2.8,1.4,-1.1,25.5,100,100
```

Fields:
- `ax, ay, az`: Acceleration in g (±2g range)
- `gx, gy, gz`: Gyroscope in deg/s (±250°/s range)
- `distance`: Ultrasonic distance in cm
- `motorA, motorB`: Current motor speeds (-255 to 255)

---

## Hardware Pin Connections (Reference)

### Arduino Uno Connections:

**MPU-6050 IMU:**
- SDA → A4 (Blue wire)
- SCL → A5 (Violet wire)
- VCC → 5V
- GND → GND

**HC-SR04 Ultrasonic:**
- TRIG → Pin 8
- ECHO → Pin 9
- VCC → 5V (Yellow)
- GND → GND

**TB6612FND Motor Driver:**
- Motor A (Left):
  - IN1 → Pin 2 (Green)
  - IN2 → Pin 3 (White)
  - PWM → Pin 5 (Violet)
- Motor B (Right):
  - IN1 → Pin 4
  - IN2 → Pin 7 (Orange)
  - PWM → Pin 6 (Brown)
- PWMA → Pin 6 (PWM A control)
- PWMB → Pin ? (PWM B control)
- VM → External Power (Battery +)
- VCC → 5V (Arduino)
- GND → GND (Common ground)

---

## System Architecture Summary

### Software Stack:
```
┌──────────────────────────────────────────┐
│            User Interface                 │
│  (Teleop Keyboard / RViz / Nav2)         │
└──────────────────────────────────────────┘
                   ↓ /cmd_vel
┌──────────────────────────────────────────┐
│         ROS2 Navigation Layer             │
│   (SLAM Toolbox, Arduino Motor Bridge)   │
└──────────────────────────────────────────┘
        ↓ /map              ↓ Serial
┌──────────────────────────────────────────┐
│          Hardware Abstraction             │
│  (YDLiDAR Driver, Serial Communication)  │
└──────────────────────────────────────────┘
        ↓ USB               ↓ USB
┌──────────────────────────────────────────┐
│            Physical Hardware              │
│    (LiDAR S2PRO, Arduino, Motors)        │
└──────────────────────────────────────────┘
```

### Data Flow:
1. **Sensing:** LiDAR + IMU + Ultrasonic → ROS2 Topics
2. **Mapping:** /scan → SLAM Toolbox → /map
3. **Visualization:** /map + /robot_description → RViz
4. **Control:** Teleop/Nav2 → /cmd_vel → Arduino → Motors
5. **Feedback:** Arduino → IMU data → ROS2 (closed loop)

---

## Troubleshooting Tips

### System Won't Start:
1. Check all USB cables connected
2. Verify permissions on `/dev/tty*` devices
3. Source `install/setup.bash` before launching
4. Kill any old processes: `pkill -f "ros2 launch"`

### Robot Moves Erratically:
1. Check motor wiring polarity
2. Verify wheel_base parameter (should be ~0.18m)
3. Calibrate motor speeds if one side faster
4. Check for loose connections

### Map Not Building:
1. Verify /scan topic publishing: `ros2 topic hz /scan`
2. Check TF tree complete: `ros2 run tf2_tools view_frames`
3. Move robot slowly for better mapping
4. Ensure sufficient lighting (LiDAR needs visibility)

### RViz Performance Issues:
1. Reduce LaserScan decay time
2. Disable unused displays
3. Lower map resolution in SLAM config
4. Close other applications

---

## Contact & Support

For issues specific to:
- **YDLiDAR:** Check official documentation
- **SLAM Toolbox:** GitHub issues page
- **ROS2 Humble:** ROS Answers forum
- **Hardware wiring:** Verify with multimeter

---

## Autonomous Frontier-Based Exploration (NEW!)

### What is Frontier Exploration?

The FEA-SLAM robot can now **autonomously explore** unknown environments by:
1. **Detecting frontiers** - Finding edges between known and unknown map areas
2. **Selecting targets** - Choosing which frontier to explore next
3. **Planning paths** - Using Nav2 to navigate to selected frontiers
4. **Building maps** - SLAM Toolbox continuously refines the map
5. **Stopping automatically** - When all exploreable areas are discovered

### Launching with Exploration Enabled

```bash
cd /home/pi/FEA_SLAM_WS
source install/setup.bash

# Launch with frontier exploration
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

### What You'll See

#### Terminal Output:
```
[INFO] [frontier_detector-X]: Frontier Detector initialized
[INFO] [exploration_coordinator-Y]: Exploration Coordinator initialized
[INFO] [exploration_coordinator-Y]: Selection method: closest
[frontier_detector-X] Found 5 frontier clusters
[exploration_coordinator-Y] 📍 Selected closest frontier at distance: 2.34m
[exploration_coordinator-Y] 🚀 Sending goal to frontier: (2.14, 1.56)
[exploration_coordinator-Y] ✨ Goal accepted, robot navigating to frontier...
[exploration_coordinator-Y] ✅ Successfully reached frontier!
[exploration_coordinator-Y] 📍 Selected closest frontier at distance: 1.89m
```

#### RViz Display:
- **Green Spheres** - Frontier candidates (size = frontier size)
- **Growing Map** - More area revealed as robot explores
- **Robot Path** - Trail shown in navigation visualization
- **Status Update** - Terminal shows exploration progress

### How Frontier Exploration Works

#### 1. **Frontier Detection** (`frontier_detector` node)
- Analyzes `/map` topic (occupancy grid from SLAM)
- Finds cells at boundary between **known** and **unknown** areas
- Groups adjacent frontier cells into clusters
- Publishes frontier clusters as RViz markers

**Parameters:**
```yaml
min_frontier_size: 5              # Minimum cells in cluster
min_distance_to_frontier: 0.3     # Minimum 0.3m from known area
frontier_threshold: 50            # Cells > 50 are "unknown"
```

#### 2. **Frontier Selection** (`exploration_coordinator` node)
Chooses next frontier using one of two strategies:

**Strategy: `closest` (default)**
- Greedy approach: always go to nearest frontier
- Fast exploration, completes quickly
- May miss larger unexplored regions
- **Best for:** Small to medium environments

**Strategy: `gain` (information gain)**
- Smart approach: prioritizes larger unknown areas
- Explores more thoroughly
- Takes longer but more complete
- **Best for:** Large or complex environments

Change strategy in `robot_full.launch.py`:
```python
'frontier_selection_method': 'closest'  # or 'gain'
```

#### 3. **Navigation to Frontier** 
- Creates `/navigate_to_pose` action goal
- Uses Nav2 (Nav2 must be configured with navigation plugins)
- Handles obstacle avoidance and local planning
- **Note:** If Nav2 not available, goal still published but robot won't move autonomously

#### 4. **Completion Detection**
- Exploration ends when:
  - ✅ **No more frontiers** - All exploreable areas discovered
  - ⏱️ **Time limit exceeded** - After `max_exploration_time` seconds (default: 600s = 10 min)
  - ❌ **User interrupt** - Ctrl+C stops exploration

### Frontier Exploration Data Flow

```
SLAM Toolbox (publishes /map)
          ↓
   Frontier Detector (analyzes map)
          ↓
    frontier_detector publishes /frontiers (MarkerArray)
          ↓
  Exploration Coordinator (selects goal)
          ↓
 Nav2 navigate_to_pose action
          ↓
   Robot moves to frontier
          ↓
   SLAM updates map
          ↓
   [Loop back to step 1]
```

### Behavior Examples

#### Scenario 1: Simple Room Exploration
```
Initial State:        After Exploration:
┌─────────────┐      ┌─────────────┐
│ ?????????? │ →    │ ███░░░░░░░░ │  (mostly explored)
│ ?????????? │       │ ███░░░░░░░░ │
│ R======== │       │ R███████████ │
└─────────────┘      └─────────────┘

R = Robot, ? = Unknown, ░ = Frontier, ███ = Explored
```

#### Scenario 2: Exploration Sequence (3 frontiers)
1. **Start:** Robot at origin, map empty
   - Frontier detected at (2m, 0m)
   - Status: `Navigating to frontier #1`

2. **After 1st frontier:** Map grows
   - New frontier detected at (-1m, 2m)
   - Status: `Successfully reached frontier! Navigating to frontier #2`

3. **After 2nd frontier:** More area revealed
   - New frontier detected at (2m, -2m)
   - Status: `Navigating to frontier #3`

4. **After 3rd frontier:** All explored
   - Status: `⭐ Exploration Complete! No more frontiers detected.`

### Performance Tuning

**To speed up exploration:**
```python
'min_frontier_size': 3,           # Lower threshold (accept small frontiers)
'frontier_selection_method': 'closest',  # Greedy strategy
'max_exploration_time': 300.0     # 5 minute limit
```

**To explore more thoroughly:**
```python
'min_frontier_size': 10,          # Higher threshold (avoid noise)
'frontier_selection_method': 'gain',  # Information gain strategy
'max_exploration_time': 1200.0    # 20 minute limit
```

**To handle narrow spaces:**
```python
'min_distance_to_frontier': 0.2   # Closer to walls
'frontier_threshold': 40          # More sensitive to unknown
```

### Troubleshooting Frontier Exploration

**Problem: "No frontiers found" but map seems incomplete**
- Solution: Check `frontier_threshold` - may be too high (70+)
- Try: `frontier_threshold: 40` to detect more unknown areas
- Verify SLAM is running and updating `/map`

**Problem: Robot not moving to frontiers**
- Solution: Nav2 may not be available
- Check: `ros2 node list | grep nav`
- Alternative: Use `teleop_twist_keyboard` to manually navigate
- Install Nav2: `sudo apt install ros-humble-nav2-*`

**Problem: Exploration takes too long**
- Solution: Change selection method to `closest`
- Or lower `min_frontier_size` to explore all areas (even small ones)
- Check for obstacles blocking robot path

**Problem: Robot stuck in loop exploring same area**
- Solution: Disable exploration, manually move robot, restart
- Or increase `min_distance_to_frontier` to avoid noise

**Problem: Memory usage high during long exploration**
- Solution: Exploration itself doesn't use much memory
- Issue is likely SLAM Toolbox with large map
- Try: `ros2 service call /finish_first_rotation std_srvs/Empty`
- Or restart launch with fresh map

### Frontier Exploration Advantages

✅ **Fully Autonomous** - No user input needed
✅ **Efficient** - Chooses logical next targets
✅ **Complete** - Explores until all areas mapped
✅ **Adaptive** - Adjusts to environment shape
✅ **Recoverable** - Continues even if goals fail
✅ **Visible** - Shows progress in RViz

### Current Limitations & Future Work

⚠️ **Nav2 Integration**: Requires Nav2 for autonomous navigation (can be added)
⚠️ **Local Minima**: May get stuck in dead-end corridors (can add escape logic)
⚠️ **Time-Based**: No energy awareness (assumes unlimited battery)
⚠️ **Single Frontier**: Explores one target at a time (could parallelize)

---

**Workspace Location:** `/home/pi/FEA_SLAM_WS`

**Key Files:**
- Launch: `src/fea_slam/launch/robot_full.launch.py`
- Arduino: `scripts/arduino_mpu6050.ino`
- URDF: `src/fea_slam/description/robot_core.xacro`
- LiDAR Config: `src/ydlidar_ros2_driver/params/ydlidar.yaml`

---

## Quick Reference: Most Common Commands

```bash
# Start system (manual mode)
cd /home/pi/FEA_SLAM_WS
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py

# Start with AUTONOMOUS FRONTIER EXPLORATION
ros2 launch fea_slam robot_full.launch.py exploration:=true

# Start with exploration disabled initially, enable later
ros2 launch fea_slam robot_full.launch.py exploration:=false
# Then in another terminal:
ros2 run localization frontier_detector
ros2 run localization exploration_coordinator

# Stop system
Ctrl+C

# Check nodes running
ros2 node list

# Check all topics
ros2 topic list

# Monitor frontier detection
ros2 topic echo /frontiers --rate=1

# Monitor exploration goal
ros2 topic echo /current_frontier_goal

# Manually control robot (requires teleop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then use arrow keys to move

# Save final map after exploration
ros2 run nav2_map_server map_saver_cli -f ~/final_map

# Monitor Arduino serial (debug sensor values)
screen /dev/ttyACM0 115200

# View complete TF tree
ros2 run tf2_tools view_frames

# Kill all ROS2 processes
pkill -f ros2
```

---

**Last Updated:** January 7, 2026 (Frontier Exploration Added)
**System:** FEA-SLAM Robot with Arduino + YDLiDAR S2PRO + Autonomous Frontier Exploration
**ROS2 Version:** Humble
**Status:** ✅ COMPLETE - Full Autonomous Exploration System Ready
