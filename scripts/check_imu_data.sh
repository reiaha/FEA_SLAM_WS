#!/bin/bash
# IMU Data Diagnostics Script
# Checks if MPU6050 IMU is connected and publishing valid data

echo "=========================================="
echo "IMU Data Diagnostics"
echo "=========================================="
echo ""

# Check if Arduino is connected
echo "1. Checking Arduino Connection..."
if [ -e /dev/ttyACM0 ]; then
    echo "✓ Arduino detected at /dev/ttyACM0"
    ls -l /dev/ttyACM0
else
    echo "✗ Arduino NOT found at /dev/ttyACM0"
    echo "Available serial devices:"
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No serial devices found"
fi
echo ""

# Check permissions
echo "2. Checking Serial Port Permissions..."
if groups | grep -q dialout; then
    echo "✓ User is in 'dialout' group"
else
    echo "✗ User NOT in 'dialout' group"
    echo "  Run: sudo usermod -aG dialout $USER"
    echo "  Then logout and login again"
fi
echo ""

# Check if sensor_fusion node is running
echo "3. Checking if sensor_fusion node is running..."
if ros2 node list 2>/dev/null | grep -q imu_lidar_ekf; then
    echo "✓ sensor_fusion (imu_lidar_ekf) node is running"
else
    echo "✗ sensor_fusion node NOT running"
    echo "  Start with: ros2 run localization sensor_fusion"
fi
echo ""

# Check IMU topic
echo "4. Checking /imu/data_raw topic..."
if ros2 topic list 2>/dev/null | grep -q '/imu/data_raw'; then
    echo "✓ /imu/data_raw topic exists"
    
    echo ""
    echo "5. Sampling IMU data (5 seconds)..."
    timeout 5 ros2 topic echo /imu/data_raw --once 2>/dev/null || echo "✗ No data received on /imu/data_raw"
else
    echo "✗ /imu/data_raw topic NOT found"
fi
echo ""

# Check topic frequency
echo "6. Checking IMU publish rate..."
if ros2 topic list 2>/dev/null | grep -q '/imu/data_raw'; then
    echo "Measuring frequency (10 seconds)..."
    ros2 topic hz /imu/data_raw --window 10 &
    HZ_PID=$!
    sleep 10
    kill $HZ_PID 2>/dev/null
else
    echo "✗ Cannot measure frequency - topic doesn't exist"
fi
echo ""

# Check for Arduino serial output directly
echo "7. Checking raw Arduino serial output..."
if [ -e /dev/ttyACM0 ]; then
    echo "Reading 5 lines from /dev/ttyACM0..."
    timeout 3 head -5 /dev/ttyACM0 2>/dev/null || echo "✗ No data from serial port"
else
    echo "✗ /dev/ttyACM0 not available"
fi
echo ""

# Summary and recommendations
echo "=========================================="
echo "IMU Configuration Summary"
echo "=========================================="
echo ""
echo "Expected MPU6050 Data Format:"
echo "  ax, ay, az, gx, gy, gz"
echo "  Example: 0.05, -0.02, 1.00, 0.12, -0.34, 0.01"
echo ""
echo "Valid Ranges:"
echo "  Acceleration: -2g to +2g (-19.6 to +19.6 m/s²)"
echo "  Gyroscope: -250 to +250 deg/s (-4.36 to +4.36 rad/s)"
echo ""
echo "Common Issues:"
echo "  1. Arduino not connected → Check USB connection"
echo "  2. No /imu/data_raw topic → Start sensor_fusion node"
echo "  3. Permission denied → Add user to dialout group"
echo "  4. Invalid data format → Check Arduino code sends CSV"
echo "  5. High noise → Calibrate MPU6050 or add filtering"
echo ""
echo "To launch sensor_fusion:"
echo "  ros2 launch localization sensor_fusion_launch.py"
echo ""
echo "=========================================="
