# FEA_SLAM_WS: Tuning Guide for Different Environments

## 1. Large Area (e.g., >5m x 5m)
- **front_stop_distance**: Can be increased (e.g., 0.20–0.30) for more cautious stopping.
- **front_obstacle_half_angle_deg**: 45–60° is typical; increase if you want wider detection.
- **rear_stop_distance**: Can be increased (e.g., 0.20–0.30) for more rear safety.
- **scan_stale_timeout**: Increase if you expect occasional scan delays (e.g., 2.0s).
- **max_linear_speed_mps**: You may increase for faster movement in open spaces (e.g., 0.4–0.5).

## 2. Glossy Tiles (High Reflectivity)
- **LIDAR/Ultrasonic Min/Max Range**: Increase min range (e.g., 0.08) to ignore false close readings.
- **any_obstacle_min_hits / front_block_min_hits**: Increase to filter out noise (e.g., 8–12).
- **safety_stop_confirm_count**: Increase to avoid false positives (e.g., 3–5).

## 3. Noisy/Cluttered Area
- **any_obstacle_stop_distance**: Lower to avoid overreacting to small objects (e.g., 0.10–0.15).
- **any_obstacle_min_hits**: Increase to require more consistent detection (e.g., 10–15).
- **velocity_deadband / angular_deadband**: Increase to suppress small, noisy commands (e.g., 0.05/0.10).

## 4. General Robustness
- **rear_backup_min_distance**: Lower if you want to allow closer backup (e.g., 0.10–0.15).
- **allow_backward_when_rear_blocked**: Set to True for more flexible escape.
- **enable_safety_override**: Keep True for emergency stops.

## 5. Troubleshooting
- If robot shakes/oscillates: Lower front_obstacle_half_angle_deg and/or increase min_hits.
- If robot stops too far from obstacles: Lower front_stop_distance.
- If robot doesn't stop soon enough: Increase front_stop_distance.
- If robot gets stuck: Lower rear_stop_distance and rear_backup_min_distance.

---

## Example Parameter Set for Large, Glossy, Noisy Area

```
front_stop_distance: 0.22
front_obstacle_half_angle_deg: 45.0
rear_stop_distance: 0.20
rear_backup_min_distance: 0.12
any_obstacle_stop_distance: 0.14
any_obstacle_min_hits: 12
front_block_min_hits: 8
safety_stop_confirm_count: 4
ultrasonic_min_range: 0.08
scan_stale_timeout: 2.0
velocity_deadband: 0.05
angular_deadband: 0.10
allow_backward_when_rear_blocked: True
enable_safety_override: True
max_linear_speed_mps: 0.4
```

Adjust these values as needed for your environment. Always test after tuning!
