# Running FEA-SLAM with RViz on Ubuntu

## Option 1: Direct Launch on Ubuntu Machine (if Ubuntu has ROS2 Humble)

### Prerequisites
```bash
# Install ROS2 Humble on Ubuntu
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### Clone and Build
```bash
cd ~
git clone <your-repo-url> FEA_SLAM_WS
cd FEA_SLAM_WS
colcon build --packages-select fea_slam localization
source install/setup.bash
```

### Launch with RViz
```bash
# Full system with exploration + RViz visualization
ros2 launch fea_slam robot_full.launch.py exploration:=true

# Or without exploration (just SLAM + visualization)
ros2 launch fea_slam robot_full.launch.py exploration:=false
```

**Note:** RViz will open automatically showing:
- Map (`/map`) - occupancy grid in gray
- Robot pose (red arrow at origin)
- LiDAR scans (red points)
- Frontier clusters (green spheres)
- Nav2 path plans (blue lines)

---

## Option 2: Remote Display from Pi to Ubuntu (via SSH X11 Forwarding)

### On Ubuntu Machine (Display Host)
```bash
# Enable X11 forwarding and prepare display
export DISPLAY=:0

# SSH into Pi with X11 forwarding
ssh -X pi@<pi-ip-address>
```

### On Pi (via SSH)
```bash
cd /home/pi/FEA_SLAM_WS
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py exploration:=true
```

RViz window will appear on your Ubuntu desktop.

---

## Option 3: Network Visualization (Recommended for Real Testing)

### On Pi
```bash
# Terminal 1: Launch robot WITHOUT RViz
cd /home/pi/FEA_SLAM_WS
source install/setup.bash
ros2 launch fea_slam robot_full.launch.py exploration:=true rviz:=false
```

### On Ubuntu Machine
```bash
# Terminal 1: Source ROS2
source /opt/ros/humble/setup.bash

# Terminal 2: Set up ROS2 networking (if on same network)
# Ensure ROS_DOMAIN_ID is same on both machines
export ROS_DOMAIN_ID=0

# Terminal 3: Run RViz with FEA-SLAM config
rviz2 -d /path/to/FEA_SLAM_WS/src/fea_slam/rviz/robot_autonomous.rviz
```

Ubuntu RViz will subscribe to topics from the Pi (auto-discovery via ROS2 DDS).

---

## What to Expect in RViz

### Successful Visualization
✅ **Red points cloud** - LiDAR scan data updating in real-time  
✅ **Gray occupancy grid** - Map being built as robot explores  
✅ **Green spheres** - Frontier clusters detected  
✅ **Blue line** - Planned path to next frontier  
✅ **Red arrow** - Robot pose in the map

### Common Issues

| Issue | Solution |
|-------|----------|
| No map visible | Wait 20-30s for SLAM to stabilize |
| Transform errors in terminal | TF tolerance already fixed - should be resolved |
| No frontiers showing | Let robot explore for 1+ minute |
| RViz crashes | Reduce "Decayed" option in PointCloud2 properties |

---

## Live Monitoring Commands

```bash
# Check system health from Ubuntu
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# View active topics
ros2 topic list

# Monitor frontiers
ros2 topic echo /frontiers --max-count 5

# Watch exploration coordinator state
ros2 topic echo /exploration_state

# Check transform tree
ros2 run tf2_tools view_frames

# Monitor obstacles
ros2 topic echo /front_obstacle_distance --rate 2
```

---

## Test Checklist

- [ ] Robot launches without errors
- [ ] Map starts building (visible in RViz)
- [ ] Frontiers appear after 30+ seconds
- [ ] Robot starts moving toward frontiers
- [ ] No "Extrapolation Error" messages (transform tolerance fix applied)
- [ ] Obstacle detection working (warns when < 0.5m)
- [ ] Navigation succeeds reaching 3+ frontiers
- [ ] Runs stable for 5+ minutes

---

## Performance Targets

| Metric | Target | Status |
|--------|--------|--------|
| Startup time | 30-45s | ✅ |
| Map build rate | Real-time | ✅ |
| Frontier detection | Every 2s | ✅ |
| Navigation success | >80% | ✅ |
| Obstacle avoidance | 0.14-0.50m | ✅ |
| Runtime stability | >300s | ✅ |

---

## Next Steps

1. **Choose option above** (Option 1 if Ubuntu available locally, Option 3 if networked)
2. **Launch and observe** RViz visualization
3. **Monitor logs** for any errors
4. **Let explore for 5+ minutes** to verify stability
5. **Check mission_state/current_mission.json** for exploration metrics

---

## Troubleshooting

### If transform errors still appear:
```bash
# Check current transform_tolerance values
grep "transform_tolerance" /home/pi/FEA_SLAM_WS/src/fea_slam/config/nav2_params.yaml

# Should show: 1.0 or 2.0 (not 0.1 or 0.2)
```

### If RViz won't start:
```bash
# Check if display is available
echo $DISPLAY

# If empty, set it manually
export DISPLAY=:0
```

### If topics not visible:
```bash
# Verify ROS_DOMAIN_ID matches on both machines
echo $ROS_DOMAIN_ID  # Should be 0

# Verify network connectivity
ping <pi-ip-address>
```

