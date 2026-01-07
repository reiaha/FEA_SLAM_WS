# Quick Fix for IMU Drift Issue

## The Problem
Your EKF shows the robot "moving" 34 meters in 7 seconds while stationary!

**Root cause:** Arduino code has wrong accelerometer scale factor
- Reading: az ≈ 10 m/s² (should be ~9.8 m/s²)
- EKF interprets this as constant 10 m/s² acceleration
- Position error compounds: x = 0.5 * a * t² 
- After 7 seconds: 0.5 * 10 * 49 = 245m error potential

## The Solution

### Step 1: Upload Corrected Arduino Code
Upload `/home/pi/FEA_SLAM_WS/scripts/arduino_mpu6050.ino` to your Arduino

**Key fixes in the new code:**
```cpp
const float ACCEL_SCALE = 16384.0;  // Correct for ±2g range
const float GYRO_SCALE = 131.0;     // Correct for ±250 deg/s

// Proper formatting with fixed decimal places
Serial.print(ax_g, 4);  // 4 decimal places
Serial.print(",");
// ... etc
```

### Step 2: Verify Fixed Data
After uploading, check the serial output:
```bash
timeout 5 cat /dev/ttyACM0 | head -10
```

**Expected output (all lines should have 6 values):**
```
0.02,-0.01,1.00,-0.05,-0.02,0.01
-0.01,0.03,0.99,-0.04,-0.03,-0.01
0.00,0.02,1.01,-0.05,-0.02,0.00
```

**Key checks:**
- ✓ All lines have exactly 6 comma-separated values
- ✓ az (3rd value) should be close to 1.0 (±0.1)
- ✓ No truncated or malformed numbers

### Step 3: Restart sensor_fusion
```bash
ros2 launch localization sensor_fusion_launch.py
```

### Step 4: Verify No Drift
With corrected data, the EKF position should stay near zero for a stationary robot:
```
EKF | x: 0.000 y: 0.000 theta: 0.000
EKF | x: 0.001 y: 0.000 theta: 0.000
EKF | x: 0.001 y: 0.001 theta: 0.000
```

## Why This Happened

Your current Arduino code likely has:
```cpp
// WRONG - causes 10x error:
float az_g = az_raw / 1638.4;  // Missing a zero!

// CORRECT:
float az_g = az_raw / 16384.0;  // For ±2g range
```

The MPU6050 in ±2g mode outputs:
- 16384 LSB per g (least significant bits)
- So 1g (gravity) = 16384 in raw reading
- Dividing by 1638.4 gives you 10g instead of 1g!

## After Fix: Expected Behavior

**Stationary robot:**
- ax ≈ 0.0g, ay ≈ 0.0g, az ≈ 1.0g
- gx, gy, gz ≈ 0 deg/s
- EKF position drift < 0.01 m/s

**Moving robot:**
- Acceleration changes reflect actual motion
- EKF tracks position accurately
- No unrealistic position jumps

## If Still Drifting After Fix

If small drift persists even with correct data:

1. **Calibrate IMU** - Follow procedure in IMU_INTEGRATION.md
2. **Tune EKF Q matrix** - Reduce process noise if too sensitive
3. **Add LiDAR measurement update** - Implement compute_lidar_pose() function

---

**Critical action:** Re-flash Arduino NOW with scripts/arduino_mpu6050.ino
