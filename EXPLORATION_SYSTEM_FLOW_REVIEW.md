# FEA_SLAM Exploration System: Full Flow and Parameter Review

## System Flow Overview

- **Initialization:**
  - System starts in `Phase.INIT`.
  - All parameters are declared and read from a defaults map using `declare_parameter` and `get_parameter`.
  - Parameters control everything from obstacle thresholds to topic names and recovery logic.

- **Main Loop and State Machine:**
  - The main loop manages state transitions using the `Phase` enum:
    - `INIT`: Waits for initial pose, costmap, and system readiness.
    - `EXPLORE`: Selects and sends navigation goals to frontiers.
    - `OBSTACLE`: Handles detected obstacles, triggers recovery or backup.
    - `RESCAN`: Performs additional scans if stuck or blocked.
    - `RECOVERY`: Executes recovery behaviors (costmap clearing, backup, etc).
    - `DONE`: Stops the robot and saves results when exploration is complete.

- **Frontier Handling:**
  - Frontiers are filtered and selected based on map data, costmap, and various thresholds (e.g., minimum distance, occupancy, unknown neighbors).
  - Parameters control how frontiers are validated, pruned, and chosen.

- **Obstacle and Recovery:**
  - Obstacle detection (LiDAR, ultrasonic) triggers backup, costmap clearing, and recovery logic.
  - Parameters control thresholds, timing, and which sensors are used.

- **Completion:**
  - Exploration ends when no valid frontiers remain and completion criteria (coverage, goals, movement) are satisfied.

---

## State Transitions

- Controlled by the `Phase` enum:
  - `INIT`, `EXPLORE`, `OBSTACLE`, `RESCAN`, `RECOVERY`, `DONE`
- Transitions are triggered by events such as:
  - Initial pose received
  - Frontiers detected or exhausted
  - Obstacles detected/cleared
  - Recovery success/failure
  - Completion criteria met

---

## Key Parameters (examples)

- **Navigation and Obstacle:**
  - `nav2_timeout`, `obstacle_distance`, `backup_speed`, `backup_time`
  - `use_lidar_obstacle`, `strict_obstacle_handling`, `use_ultrasonic_backup`
- **Frontier and Map:**
  - `frontier_goal_offset`, `min_frontier_distance`, `frontier_lidar_range_m`
  - `costmap_free_threshold`, `require_costmap`, `costmap_wait_timeout`
- **Completion and Recovery:**
  - `coverage_complete_percent`, `min_goals_for_complete`, `post_abort_goal_cooldown_sec`, `goal_watchdog_timeout`
- **Other:**
  - `auto_save_on_complete`, `map_save_dir`, `path_record_min_dist`
- **All parameters are declared in a single defaults dictionary at startup.**

---

## Parameter Declaration and Usage

- Parameters are declared in the constructor using a `defaults` dictionary:

```python
self._declare_params(defaults)
for name, default in defaults.items():
    setattr(self, name, self._read_param(name, default))
```

- Used throughout the system for:
  - Thresholds (distances, timeouts, counts)
  - Feature toggles (enable/disable behaviors)
  - Topic names and file paths
  - Recovery and completion logic

---

## Example: Phase Enum

```python
from enum import Enum
class Phase(Enum):
    INIT = 1
    EXPLORE = 2
    OBSTACLE = 3
    RESCAN = 4
    RECOVERY = 5
    DONE = 6
```

---

## Summary

- The FEA_SLAM exploration system is highly parameterized and state-driven.
- All behaviors, thresholds, and logic are controlled by parameters declared at startup.
- The main loop and state machine ensure robust, flexible exploration, obstacle handling, and recovery.

For a full list of parameters and their default values, see the `defaults` dictionary in `exploration_coordinator.py`.
