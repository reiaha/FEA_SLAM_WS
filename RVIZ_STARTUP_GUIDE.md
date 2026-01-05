# RViz Startup Guide - Black Screen Troubleshooting

## What to Expect on Startup

When you run `ros2 launch fea_slam robot_full.launch.py`, RViz should show:

### Initial State (First 2-3 seconds):
- **Black screen** is NORMAL initially
- Robot model appears once `/robot_description` is published
- TF tree builds as transforms are established
- LaserScan (if LIDAR enabled) appears once data arrives
- Map (if SLAM enabled) appears after SLAM initializes (usually 10-30 seconds)

## Diagnosis Steps

### 1. Check if RViz Opened
```bash
ps aux | grep rviz
```
Should show rviz2 process running.

### 2. Check Topics Being Published
```bash
ros2 topic list
```
Should see:
- `/robot_description` (always present)
- `/scan` (if LIDAR enabled)
- `/map` (if SLAM enabled, after initialization)

### 3. Check if Topics Have Data
```bash
# Robot description (should return data)
ros2 topic echo /robot_description | head -10

# Laser scan (should show point data)
ros2 topic echo /scan | head -20

# Map (may be empty initially while SLAM initializes)
ros2 topic echo /map | head -20
```

### 4. Check Transform Tree
```bash
ros2 run tf2_tools view_frames
```
Should show `base_footprint` → `base_link` → sensors

## RViz Configuration Issues

If RViz is open but still completely black:

### Option A: Reset RViz to Defaults
1. In RViz top menu: **Panels → Displays**
2. Click the X on each display to clear
3. Click **Add** (bottom-left of Displays panel)
4. Add these displays in order:
   - Grid
   - RobotModel (set Topic: /robot_description)
   - LaserScan (set Topic: /scan)
   - Map (set Topic: /map)
   - TF

### Option B: Reload RViz Config
```bash
pkill rviz2
```
Then manually launch visualization_pkg:
```bash
ros2 launch visualization_pkg visualization_launch.py
```

### Option C: Use Different Fixed Frame
If displays still don't appear:
1. In RViz **Global Options**, change **Fixed Frame** from `base_footprint` to:
   - Try: `base_link`
   - Try: `odom` (if SLAM is running)
   - Try: `map` (if SLAM is running)

## Expected Behavior Timeline

```
Time  Event
---------------------------------------------
0s    RViz opens (black screen)
0.5s  Robot model appears (from /robot_description)
1-2s  TF tree is built (visible in TF display)
2-5s  LaserScan points appear (if LIDAR enabled)
10-30s Map occupancy grid appears (if SLAM enabled)
```

## Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| Completely black screen | Fixed frame doesn't exist | Change Fixed Frame in Global Options |
| Robot model appears but no scan points | LIDAR not publishing | Check `ros2 topic list \| grep scan` |
| Map never appears | SLAM not initialized | Wait 30 seconds, or check SLAM logs |
| Displays disabled | Config error | Add displays manually via Add button |
| High latency/lag | Too much data | Reduce LaserScan display size or Map resolution |

## Logs to Check

```bash
# Check SLAM initialization
tail -50 install/fea_slam/log/*latest*/stdout

# Check RViz errors
tail -50 install/visualization_pkg/log/*latest*/stdout

# Check ROS node status
ros2 node list
```

## Next Steps

1. **On first run:** Wait 30 seconds for system to fully initialize
2. **If still black:** Run steps 1-4 above to diagnose
3. **If RobotModel appears but no scan:** Check LIDAR connections
4. **If Map doesn't appear:** SLAM may still be initializing or loop closure needs data

---

**Last Updated:** 2026-01-05  
**Fixed Frame:** base_footprint (changes to map when SLAM publishes map transform)
