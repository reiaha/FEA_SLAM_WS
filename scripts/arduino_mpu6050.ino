/*
 * MPU6050 IMU Reader for FEA-SLAM Robot
 * 
 * Reads accelerometer and gyroscope data from MPU6050
 * and sends it via serial in CSV format to ROS2
 * 
 * Hardware: Arduino + MPU6050 IMU
 * Baud Rate: 115200
 * Output Format: ax, ay, az, gx, gy, gz
 * Units: g (acceleration), deg/s (gyroscope)
 */

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Calibration offsets (set these after calibration)
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;
float gx_offset = 0.0;
float gy_offset = 0.0;
float gz_offset = 0.0;

// Conversion factors
const float ACCEL_SCALE = 16384.0;  // For ±2g range
const float GYRO_SCALE = 131.0;     // For ±250 deg/s range

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Test connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1) {
      delay(1000);
    }
  }
  
  // Configure MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250 deg/s
  
  // Optional: Enable low-pass filter
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  // 20 Hz bandwidth
  
  Serial.println("MPU6050 Ready");
  delay(100);
}

void loop() {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  
  // Read raw IMU data
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
  
  // Convert to g and deg/s
  float ax_g = (ax_raw / ACCEL_SCALE) - ax_offset;
  float ay_g = (ay_raw / ACCEL_SCALE) - ay_offset;
  float az_g = (az_raw / ACCEL_SCALE) - az_offset;
  
  float gx_d = (gx_raw / GYRO_SCALE) - gx_offset;
  float gy_d = (gy_raw / GYRO_SCALE) - gy_offset;
  float gz_d = (gz_raw / GYRO_SCALE) - gz_offset;
  
  // Send as CSV with fixed precision
  // IMPORTANT: Use fixed field width for consistent parsing
  Serial.print(ax_g, 4);  // 4 decimal places
  Serial.print(",");
  Serial.print(ay_g, 4);
  Serial.print(",");
  Serial.print(az_g, 4);
  Serial.print(",");
  Serial.print(gx_d, 4);
  Serial.print(",");
  Serial.print(gy_d, 4);
  Serial.print(",");
  Serial.println(gz_d, 4);  // println adds newline
  
  delay(20);  // 50 Hz update rate
}

/*
 * CALIBRATION PROCEDURE:
 * 
 * 1. Upload this code to Arduino
 * 2. Place robot on flat, level surface
 * 3. Keep completely still for 30 seconds
 * 4. Open Serial Monitor and record average values
 * 5. Set the offset variables above:
 *    - ax_offset, ay_offset should make ax≈0, ay≈0
 *    - az_offset should make az≈1.0
 *    - gx_offset, gy_offset, gz_offset should make all≈0
 * 6. Re-upload and verify
 * 
 * Example:
 * If average az = 0.98, set az_offset = -0.02
 * If average gx = 0.15, set gx_offset = 0.15
 */
