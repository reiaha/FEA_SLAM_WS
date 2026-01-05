# IMU Integration Guide - MPU6050

## Hardware Setup

**IMU Sensor:** MPU6050 (6-axis accelerometer + gyroscope)  
**Connection:** Arduino via Serial (115200 baud)  
**Port:** `/dev/ttyACM0`  
**Power:** 3.3V or 5V (check your MPU6050 module)

### Wiring
```
MPU6050 → Arduino
-----------------
VCC     → 5V (or 3.3V depending on module)
GND     → GND
SDA     → A4 (or dedicated SDA pin)
SCL     → A5 (or dedicated SCL pin)
```

Arduino → Raspberry Pi: USB connection

## Software Architecture

### Data Flow
```
MPU6050 → Arduino (I2C) → Serial (115200) → Raspberry Pi
                                             ↓
                            ROS2 sensor_fusion node (EKF)
                                             ↓
                        /imu/data_raw topic (sensor_msgs/Imu)
                                             ↓
                             SLAM Toolbox / Navigation
```

### Node: sensor_fusion (imu_lidar_ekf)

**Package:** `localization`  
**File:** `src/localization/localization/sensor_fusion.py`

**Publishers:**
- `/imu/data_raw` (sensor_msgs/Imu) - Raw IMU measurements
- `/odom` (nav_msgs/Odometry) - Filtered odometry from EKF
- `/scan_fused` (sensor_msgs/LaserScan) - Time-synced LiDAR scans

**Subscribers:**
- `/scan` (sensor_msgs/LaserScan) - Raw LiDAR data

**TF Broadcasts:**
- `odom → base_link` (from EKF state estimation)
- `base_link → laser_frame` (static transform)

### Extended Kalman Filter (EKF)

**State Vector:** `[x, y, theta, vx, vy]`
- Position: (x, y) in meters
- Orientation: theta in radians
- Velocity: (vx, vy) in m/s

**Prediction Step:** Uses IMU accelerations and gyro for dead reckoning  
**Update Step:** Uses LiDAR odometry (placeholder - needs implementation)

## Arduino Code Requirements

Your Arduino must send comma-separated values at 115200 baud:

```arduino
// Expected format:
// ax, ay, az, gx, gy, gz
// Example line:
0.05, -0.02, 1.00, 0.12, -0.34, 0.01
```

**Units from Arduino:**
- Acceleration: in `g` (gravity units, where 1g = 9.80665 m/s²)
- Gyroscope: in `deg/s` (degrees per second)

**Conversion (done by ROS node):**
- Acceleration: multiplied by 9.80665 → m/s²
- Gyroscope: multiplied by π/180 → rad/s

### Sample Arduino Code Template

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  Serial.println("MPU6050 Ready");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert raw values to g and deg/s
  float ax_g = ax / 16384.0;  // for ±2g range
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_d = gx / 131.0;    // for ±250 deg/s range
  float gy_d = gy / 131.0;
  float gz_d = gz / 131.0;
  
  // Send as CSV
  Serial.print(ax_g, 4); Serial.print(", ");
  Serial.print(ay_g, 4); Serial.print(", ");
  Serial.print(az_g, 4); Serial.print(", ");
  Serial.print(gx_d, 4); Serial.print(", ");
  Serial.print(gy_d, 4); Serial.print(", ");
  Serial.println(gz_d, 4);
  
  delay(20);  // 50 Hz
}
```

## Data Validation

### Expected Values (Robot Stationary)

**Accelerometer:**
- ax ≈ 0.0 g
- ay ≈ 0.0 g
- az ≈ 1.0 g (gravity on Z-axis if mounted flat)

**Gyroscope:**
- gx ≈ 0.0 deg/s
- gy ≈ 0.0 deg/s
- gz ≈ 0.0 deg/s

### Acceptable Noise Levels

- Acceleration noise: ±0.02 g (±0.2 m/s²)
- Gyroscope noise: ±1.0 deg/s (±0.017 rad/s)

### Red Flags (Bad IMU Data)

❌ **Constant zeros:** IMU not connected or not initialized  
❌ **Random large spikes:** Electromagnetic interference or loose connection  
❌ **Drifting gyro bias:** Needs calibration  
❌ **az not near ±1.0g:** IMU mounted at wrong angle or defective  
❌ **NaN or inf values:** Serial parsing error or buffer overflow

## Diagnostics

### Quick Test
```bash
# Run the diagnostic script
./scripts/check_imu_data.sh
```

### Manual Checks

**1. Check Arduino connection:**
```bash
ls -l /dev/ttyACM*
# Should show /dev/ttyACM0 (or ttyACM1, etc.)
```

**2. Check permissions:**
```bash
groups
# Should include 'dialout' group
# If not: sudo usermod -aG dialout $USER
```

**3. Read raw serial data:**
```bash
cat /dev/ttyACM0
# Should show comma-separated numbers streaming
```

**4. Check ROS topic:**
```bash
ros2 topic list | grep imu
# Should show /imu/data_raw

ros2 topic echo /imu/data_raw
# Should show sensor_msgs/Imu messages
```

**5. Check publish rate:**
```bash
ros2 topic hz /imu/data_raw
# Expected: ~50 Hz
```

**6. Check data values:**
```bash
ros2 topic echo /imu/data_raw --once
# Verify reasonable values within expected ranges
```

## Launching Sensor Fusion

### Standalone
```bash
ros2 run localization sensor_fusion
```

### With Launch File
```bash
ros2 launch localization sensor_fusion_launch.py
```

### Integrated with Full System
```bash
# Currently sensor_fusion is optional in robot_full.launch.py
# It will attempt to include it if the package is available
ros2 launch fea_slam robot_full.launch.py
```

## IMU Calibration

### Why Calibrate?
- Removes zero-offset bias from gyroscope
- Corrects accelerometer scale factors
- Improves EKF accuracy

### Calibration Procedure

1. **Place robot on flat, level surface**
2. **Keep robot completely still for 30 seconds**
3. **Record average values:**
   ```bash
   ros2 topic echo /imu/data_raw | head -50
   ```
4. **Calculate offsets:**
   - Gyro offsets: gx_avg, gy_avg, gz_avg (should be ~0)
   - Accel offsets: ax_avg, ay_avg (should be ~0), az_avg (should be ~1g)

5. **Apply offsets in Arduino code:**
   ```cpp
   // Subtract calibration offsets
   float ax_g = (ax / 16384.0) - ax_offset;
   float ay_g = (ay / 16384.0) - ay_offset;
   float az_g = (az / 16384.0) - az_offset;
   float gx_d = (gx / 131.0) - gx_offset;
   float gy_d = (gy / 131.0) - gy_offset;
   float gz_d = (gz / 131.0) - gz_offset;
   ```

## EKF Tuning

### Process Noise (`Q` matrix in sensor_fusion.py)

Located at lines 58-63:
```python
q_pos = 1e-3      # Position uncertainty growth
q_theta = 1e-4    # Heading uncertainty growth
q_vel = 1e-2      # Velocity uncertainty growth
self.Q = np.diag([q_pos, q_pos, q_theta, q_vel, q_vel])
```

**Increase if:** Robot position drifts from true path  
**Decrease if:** Position estimate is too noisy

### Measurement Noise (`R` matrix)

Located at lines 65-66:
```python
r_pos = 0.05  # LiDAR position measurement variance (meters)
self.R = np.diag([r_pos, r_pos])
```

**Increase if:** LiDAR data is noisy or unreliable  
**Decrease if:** LiDAR data is accurate and filter is too slow to respond

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No /imu/data_raw topic | Node not running | Launch sensor_fusion node |
| "Failed to open /dev/ttyACM0" | Arduino not connected | Check USB connection |
| Permission denied | User not in dialout group | `sudo usermod -aG dialout $USER` then logout/login |
| Invalid data format | Arduino code incorrect | Verify CSV format with 6 values |
| High gyro drift | IMU not calibrated | Run calibration procedure |
| Noisy acceleration | Vibration or EMI | Add damping or move IMU away from motors |
| EKF diverges | Tuning parameters wrong | Adjust Q and R matrices |
| No data after 2 seconds | Arduino reset delay | Check Arduino initialization messages |

## Integration with SLAM

The IMU data enhances SLAM performance by:

1. **Predicting motion between scans** - reduces scan matching errors
2. **Providing orientation estimate** - helps with loop closure detection
3. **Filtering sensor noise** - EKF fuses IMU + LiDAR for robust odometry
4. **Enabling faster updates** - IMU at 50Hz vs LiDAR at 10Hz

### Current Status

✅ IMU reading from Arduino serial  
✅ Publishing `/imu/data_raw` topic  
✅ EKF prediction using IMU accelerations  
✅ TF broadcasting (odom → base_link)  
⚠️  LiDAR odometry measurement update is **placeholder** (needs implementation)

The `compute_lidar_pose()` function (line 87-97) currently returns `None`, meaning only IMU prediction is used without LiDAR correction. For best results, implement scan matching or use a dedicated LiDAR odometry package.

## Next Steps

1. ✅ Verify Arduino connection: `./scripts/check_imu_data.sh`
2. ✅ Check data quality: `ros2 topic echo /imu/data_raw`
3. ⚠️  Calibrate IMU if needed (follow procedure above)
4. ⚠️  Implement LiDAR odometry in `compute_lidar_pose()`
5. ⚠️  Tune EKF parameters (Q, R matrices)
6. ⚠️  Test with real robot motion and validate odometry accuracy

---

**Last Updated:** 2026-01-05  
**IMU Model:** MPU6050  
**Sample Rate:** 50 Hz  
**Communication:** Serial 115200 baud
