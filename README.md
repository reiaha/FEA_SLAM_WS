# FEA-SLAM Framework

Frontier Exploration Autonomous SLAM (FEA-SLAM) is a localization and exploration framework for autonomous mobile robots operating in GPS-denied environments. The system combines SLAM, frontier-based exploration, and safety-aware motion control for real-world indoor deployment.

## 1.2 Statement of the Problem

In the field of autonomous robotics, effective navigation in GPS-denied environments presents a significant challenge [16], [82]. Traditional navigation systems rely heavily on GPS data, which is unavailable or unreliable in certain scenarios such as indoor settings, underground locations, or densely built urban areas [6], [29]. This necessitates the development of robust localization frameworks that can operate independently of GPS [13], [27].

High-precision sensors required for accurate localization and mapping are often prohibitively expensive, limiting the accessibility and scalability of advanced robotic systems, particularly for small-scale or budget-constrained applications [44], [89]. Additionally, robots must navigate and adapt to constantly changing environments. Dynamic obstacles, varying terrain, and unpredictable human activities pose significant challenges to the reliability and efficiency of navigation algorithms [52], [53].

Furthermore, Simultaneous Localization and Mapping (SLAM) systems are predominantly trained and tested in static environments, creating a gap in performance when these systems are deployed in real-world, dynamic settings [43], [55]. This leads to potential inaccuracies and failures in localization and mapping [59], [62]. Addressing these issues is crucial for advancing the capabilities of autonomous robots, enabling them to perform complex tasks in diverse and challenging environments.

The proposed framework integrates frontier-based exploration algorithms with SLAM techniques to enhance robustness and adaptability of robot navigation systems in GPS-denied environments.

## 1.3 Objectives

The objective of this study is to develop a localization framework for autonomous robot navigation in GPS-denied environments by integrating frontier-based exploration with SLAM, referred to as the FEA-SLAM framework. Specifically, this research aims to:

1. Design and implement a SLAM-based navigation system capable of autonomous localization and mapping in GPS-denied environments.
2. Integrate a frontier-based exploration algorithm to optimize autonomous area coverage and enhance real-time decision-making in unknown areas.
3. Evaluate the proposed FEA-SLAM framework in terms of localization accuracy, mapping reliability, and exploration efficiency in static and dynamic environments.

## Important System Parts

1. Sensor Layer
- YDLiDAR for 2D range sensing.
- IMU and wheel/bridge odometry for state estimation support.
- Optional ultrasonic safety layer for close-range emergency stop and backup behavior.

2. Perception and Localization
- SLAM Toolbox for map building and map-to-odom transform generation.
- robot_localization EKF for fused odometry/IMU state estimation.
- Scan timestamp correction pipeline to reduce TF timing drop issues.

3. Navigation and Exploration
- Nav2 stack for planning, control, recovery, and lifecycle-managed navigation.
- Frontier detector for unknown-boundary target generation.
- Exploration coordinator for goal selection, retries, blacklist handling, completion logic, and map/path persistence.

4. Motion and Safety
- Arduino motor bridge for low-level wheel command execution.
- LiDAR-based obstacle gating and escape behavior.
- Optional ultrasonic backup safety behavior, toggled at launch.

5. Logging, Metrics, and Outputs
- Session outputs under saved_maps with timestamped folders.
- CSV summaries including explored area and robot path.
- Optional post-processing script for area-rate metrics in m^2/s.

## Software Used

Core runtime and middleware:
- Ubuntu Linux
- ROS 2 Humble
- Fast DDS

Navigation and mapping:
- Nav2 (navigation2)
- SLAM Toolbox
- robot_localization
- tf2 and lifecycle nodes

Sensors and visualization:
- YDLiDAR ROS 2 driver
- RViz2

Project packages:
- fea_slam
- localization
- visualization_pkg
- ydlidar_ros2_driver

Languages and tools:
- Python 3
- C++ (ROS 2 and driver components)
- colcon build system
- Bash startup and automation scripts

## Basic Run Command

Start the full system:

```bash
./START_ROBOT.sh
```

Run with ultrasonic explicitly disabled:

```bash
./START_ROBOT.sh use_ultrasonic:=false
```

Run with ultrasonic explicitly enabled:

```bash
./START_ROBOT.sh use_ultrasonic:=true
```

## Notes for Evaluation

- Static and dynamic environment evaluation is supported through launch/environment profiles.
- Exploration performance can be assessed from saved CSV outputs and map completion logs.
- Safety behavior can be demonstrated by toggling ultrasonic usage and comparing motion response near close obstacles.

## Expected Results and KPIs

Use the following metrics for experiment reporting and panel demonstrations.

1. Localization Accuracy
- Pose consistency in RViz and TF (no prolonged pose freeze/stale warnings).
- Stable odom->base_footprint and map->odom transforms during motion.
- Low path tracking error from coordinator logs (cross-track error and heading error).

2. Mapping Reliability
- Continuous map growth without SLAM collapse.
- Map completion increase over time in coordinator logs.
- Low frequency of dropped scans due to TF timing mismatch.

3. Exploration Efficiency
- Explored area growth over time from explore_area_*.csv.
- Mapping rate (m^2/s) from area delta per elapsed second.
- Frontier utilization efficiency: accepted goals vs rejected/blacklisted goals.

4. Navigation Robustness (Static vs Dynamic)
- Successful goal execution rate in static and dynamic scenes.
- Recovery behavior count (replans, retries, costmap clears) before mission completion.
- Completion criteria satisfaction under both environment modes.

5. Safety Performance
- Reaction time to close obstacles (LiDAR and optional ultrasonic).
- Number of emergency stops and successful escape maneuvers.
- Absence of collisions during autonomous operation.

### Recommended KPI Table Format

For each run, report:
- Run ID, environment mode, ultrasonic mode (on/off), and duration.
- Final map completion (%), explored area (m^2), and average mapping rate (m^2/s).
- Goal success rate (%), planner failure count, safety stop count.
- Notes on major failures (TF stale, scan drops, rejected goals).
