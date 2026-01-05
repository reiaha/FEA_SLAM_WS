# IMU Data Analysis Report
**Generated:** 2026-01-05  
**Robot:** FEA-SLAM Platform  
**IMU:** MPU6050 via Arduino

## Connection Status
✅ **Arduino Connected:** /dev/ttyACM0  
✅ **Serial Port Accessible:** User in dialout group  
✅ **Baud Rate:** 115200  

## Data Format Issues Detected

### Problem 1: Corrupted Values
Some data points have malformed floats:
```
❌ 10.06,-0.05,-0.03,-0.0.01  (extra period: -0.0.01)
```

### Problem 2: Truncated Lines
Some lines have fewer than 6 values:
```
❌ 15,-0.04,-0.02,-0.03  (only 4 values)
```

### Problem 3: Inconsistent Formatting
Expected format: `ax, ay, az, gx, gy, gz` (6 comma-separated values)

**Sample Good Data:**
```
✓ 0.89,-0.01,9.75,-0.06,-0.02,-0.01
✓ 1.08,-0.02,11.09,-0.03,-0.03,-0.03
```

**Sample Bad Data:**
```
✗ 0.86,0.07,10.06,-0.05,-0.03,-0.0.01  (corrupted last value)
✗ 15,-0.04,-0.02,-0.03  (missing 2 values)
```

## Data Quality Assessment

### Accelerometer (in g units)
- **ax range:** 0.86 to 1.70 g
- **ay range:** -0.34 to 0.09 g
- **az range:** 9.70 to 11.09 g ⚠️ **SHOULD BE ~1.0g**

### Gyroscope (in deg/s)
- **gx range:** -0.21 to -0.03 deg/s
- **gy range:** -0.03 to -0.02 deg/s
- **gz range:** -0.03 to 0.00 deg/s

## Critical Issues

### 🔴 Issue 1: Accelerometer Z-axis Too High
**Expected:** az ≈ 1.0 g (9.8 m/s²)  
**Actual:** az ≈ 10.0 g (98 m/s²)

**Possible Causes:**
1. Wrong scale factor in Arduino code (should be 16384 for ±2g)
2. IMU configured for wrong sensitivity range
3. Unit conversion error (raw value not divided correctly)

**Fix:** Check Arduino code line that converts az:
```cpp
// Should be:
float az_g = az_raw / 16384.0;  // for ±2g range

// NOT:
float az_g = az_raw / 1638.4;   // wrong scale
```

### 🔴 Issue 2: Data Corruption
**Impact:** ROS node will reject ~10-20% of messages

**Root Cause:** 
- Serial buffer overflow
- Arduino Print() function formatting errors
- Timing issues between measurements

**Fix:** Use the provided `arduino_mpu6050.ino` code with proper formatting

### 🟡 Issue 3: No ROS Topic Active
**Status:** sensor_fusion node NOT running  
**Impact:** IMU data not integrated into SLAM

**Fix:**
```bash
ros2 launch localization sensor_fusion_launch.py
```

## Recommended Actions

### Immediate (Required):
1. **Re-flash Arduino** with corrected code:
   - Use `/home/pi/FEA_SLAM_WS/scripts/arduino_mpu6050.ino`
   - Verify scale factors (16384 for accel, 131 for gyro)
   - Add proper decimal formatting

2. **Verify data format** after re-flash:
   ```bash
   timeout 5 cat /dev/ttyACM0 | head -20
   ```
   All lines should have exactly 6 comma-separated values

3. **Launch sensor_fusion node:**
   ```bash
   ros2 launch localization sensor_fusion_launch.py
   ```

### Short-term (Recommended):
4. **Calibrate IMU** (after fixing Arduino code):
   - Place robot on level surface
   - Record 30 seconds of data
   - Calculate average offsets
   - Update calibration values in Arduino code

5. **Verify ROS integration:**
   ```bash
   ros2 topic echo /imu/data_raw --once
   ros2 topic hz /imu/data_raw
   ```
   Should see 50 Hz with clean data

### Long-term (Optional):
6. **Implement LiDAR odometry** in sensor_fusion.py:
   - Replace placeholder `compute_lidar_pose()` function
   - Use scan matching or ICP algorithm
   - Enables EKF measurement update step

7. **Tune EKF parameters:**
   - Adjust process noise (Q matrix) after testing
   - Adjust measurement noise (R matrix) based on sensor accuracy

## Expected Behavior After Fix

### Stationary Robot:
```
Accelerometer:
  ax:  0.00 ± 0.02 g
  ay:  0.00 ± 0.02 g
  az:  1.00 ± 0.02 g

Gyroscope:
  gx:  0.00 ± 1.0 deg/s
  gy:  0.00 ± 1.0 deg/s
  gz:  0.00 ± 1.0 deg/s
```

### During Motion:
- Acceleration changes reflect robot movement
- Gyroscope gz shows rotation rate (turns)
- EKF fuses data for smooth odometry

## Testing Procedure

1. **Fix Arduino code and re-upload**
2. **Verify raw serial data:**
   ```bash
   timeout 5 cat /dev/ttyACM0 | head -10
   ```
   ✓ All lines should have 6 values  
   ✓ az should be close to 1.0g  
   ✓ No formatting errors

3. **Launch sensor fusion:**
   ```bash
   ros2 launch localization sensor_fusion_launch.py
   ```

4. **Check ROS topic:**
   ```bash
   ros2 topic echo /imu/data_raw
   ```
   ✓ Messages publishing at ~50 Hz  
   ✓ linear_acceleration.z ≈ 9.8 m/s²  
   ✓ No warning messages in terminal

5. **Test with full system:**
   ```bash
   ros2 launch fea_slam robot_full.launch.py
   ```
   ✓ SLAM receives IMU data  
   ✓ Odometry published on /odom  
   ✓ TF tree includes odom → base_link

## Summary

| Component | Status | Action Required |
|-----------|--------|-----------------|
| Arduino Connection | ✅ OK | None |
| Serial Permissions | ✅ OK | None |
| Data Format | ❌ FAIL | Re-flash Arduino |
| Scale Factors | ❌ FAIL | Fix in Arduino code |
| ROS Integration | ⚠️ NOT RUNNING | Launch sensor_fusion |
| Calibration | ⚠️ NEEDED | After fixing format |

**Next Step:** Upload corrected Arduino code from `scripts/arduino_mpu6050.ino`

---

For detailed Arduino code and calibration instructions, see [IMU_INTEGRATION.md](IMU_INTEGRATION.md)
