/*
  Arduino Motor Bridge for FEA-SLAM Robot (ROS2 Compatible)
  
  This code provides a MOTOR BRIDGE for ROS2 control:
  - Receives motor commands via serial (MOTOR:speedA,speedB)
  - Receives servo commands via serial (SERVO:angle)
  - Publishes IMU data via CSV to ROS2
  - Publishes ultrasonic readings via CSV to ROS2
  
  ALL AUTONOMOUS LOGIC IS IN ROS2 (frontier_detector, exploration_coordinator, ultrasonic_explorer)
  Arduino ONLY handles: Motors, Servo, Sensor Data Publishing
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// ============= MPU-6050 IMU Configuration =============
Adafruit_MPU6050 mpu;
bool imu_initialized = false;

// Calibrated offsets (adjust based on your hardware calibration)
float gx_offset = -2.8;
float gy_offset = -1.4;
float gz_offset = -1.1;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// ============= Ultrasonic Sensor Configuration =============
const int TRIG_PIN = A0;
const int ECHO_PIN = A1;

// ============= Servo Configuration =============
const int SERVO_PIN = 10;
Servo ultrasonic_servo;
int servo_angle = 80;  // Current angle (0-180)
int servo_target_angle = 80;  // Target angle for slow movement
unsigned long last_servo_move = 0;
const int SERVO_MOVE_DELAY = 20;  // milliseconds between 1-degree steps
const bool SERVO_INVERT = false;  // Set to true if servo turns opposite direction

// Sweep configuration for ultrasonic scanning
const int SWEEP_MIN_ANGLE = 90;   // Start sweep at center
const int SWEEP_MAX_ANGLE = 180;  // End sweep to the right
const int SWEEP_STEP = 10;        // Step size in degrees

// ============= Motor Control Configuration =============
// Motor A (Left)
const int MOTOR_A_IN1 = 4;
const int MOTOR_A_IN2 = 5;
const int MOTOR_A_PWM = 6;

// Motor B (Right)
const int MOTOR_B_IN1 = 8;
const int MOTOR_B_IN2 = 7;
const int MOTOR_B_PWM = 11;

// Standby pin
const int MOTOR_STBY = 3;

// Current motor speeds
int motorA_speed = 0;
int motorB_speed = 0;

// ============= Serial Communication =============
String inputBuffer = "";
const char COMMAND_DELIMITER = '\n';

// ============= Debug / Quiet Mode =============
// When QUIET_MODE is true, all non-IMU Serial logs are suppressed.
const bool QUIET_MODE = true;
#define DPRINT(x)    do { if (!QUIET_MODE) Serial.print(x); } while(0)
#define DPRINTLN(x)  do { if (!QUIET_MODE) Serial.println(x); } while(0)
// Two-argument variants for base/format prints (e.g., HEX)
#define DPRINT2(x, y)    do { if (!QUIET_MODE) Serial.print((x), (y)); } while(0)
#define DPRINTLN2(x, y)  do { if (!QUIET_MODE) Serial.println((x), (y)); } while(0)

// ============= IMU Filter State =============
bool imu_filter_initialized = false;
float fax = 0, fay = 0, faz = 0;   // filtered accel
float fgx = 0, fgy = 0, fgz = 0;   // filtered gyro
const float FILTER_ALPHA = 0.2f;   // EMA factor (0-1); lower = smoother

int mapServoAngle(int logical_angle) {
  int clamped = constrain(logical_angle, 0, 180);
  return SERVO_INVERT ? (180 - clamped) : clamped;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize Motor Control Pins FIRST (before MPU attempt)
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);  
  
  // Initialize Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize Servo
  ultrasonic_servo.attach(SERVO_PIN);
  ultrasonic_servo.write(mapServoAngle(90));  // Start at 90 degrees (center)
  
  // Stop motors at startup
  stopMotors();
  
  // Try to Initialize MPU-6050 (non-blocking - continue if fails)
  // I2C Device Scanner (diagnostic - won't block motor operation)
  DPRINTLN("\n[DIAGNOSTIC] I2C Device Scan:");
  int i2c_found = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    if (error == 0) {
      DPRINT("  Device at 0x");
      DPRINT2(i, HEX);
      DPRINTLN(" (found)");
      i2c_found++;
    }
    delayMicroseconds(10);  // Small delay to prevent I2C bus issues
  }
  if (i2c_found == 0) {
    DPRINTLN("  [WARNING] No I2C devices found!");
  }
  DPRINTLN("");
  
  DPRINT("Initializing MPU6050...");
  int mpu_retries = 10;
  while (mpu_retries > 0 && !imu_initialized) {
    if (mpu.begin()) {
      // Configure MPU-6050
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      imu_initialized = true;
      DPRINTLN(" OK");
    } else {
      mpu_retries--;
      DPRINT(".");
      delay(500);
    }
  }
  
  if (!imu_initialized) {
    DPRINTLN(" FAILED - Motors still operational");
  }
  
  delay(100);
  DPRINTLN("IMU-Motor-Ultrasonic System Ready");
}

void loop() {
  // Check for incoming serial commands (non-blocking)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == COMMAND_DELIMITER) {
      // Process complete command
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
  
  // --- Reference Logic: Sweep, Find Closest, Decide ---
  float minDistance = 999;
  int obstacleAngle = 90;
  float maxDistance = 0;
  int clearestAngle = SWEEP_MIN_ANGLE;
  // Sweep left to right, find the closest obstacle and the clearest (farthest) direction
  for (int angle = SWEEP_MIN_ANGLE; angle <= SWEEP_MAX_ANGLE; angle += SWEEP_STEP) {
    ultrasonic_servo.write(mapServoAngle(angle));
    delay(100); // Allow servo to move and echo to settle
    float distance = readUltrasonic();
    DPRINT("Angle: "); DPRINT(angle);
    DPRINT("  Distance: "); DPRINTLN(distance);
    if (distance > 0 && distance < minDistance) {
      minDistance = distance;
      obstacleAngle = angle;
    }
    if (distance > maxDistance && distance < 400 && distance > 0) {
      maxDistance = distance;
      clearestAngle = angle;
    }
  }

  // Obstacle avoidance logic
  if (minDistance < 40) {
    DPRINT("Obstacle detected at angle: ");
    DPRINT(obstacleAngle);
    DPRINT(" (distance: ");
    DPRINT(minDistance);
    DPRINTLN(" cm), avoiding obstacle.");
    ultrasonic_servo.write(mapServoAngle(obstacleAngle));
    delay(200);
    // Decide avoidance direction: if obstacle is left, turn right; if right, turn left; if center, move backward
    if (obstacleAngle < 120) {
      DPRINTLN("Obstacle left, turning right to avoid.");
      turnRight(150);
      delay(500);
      stopMotors();
    } else if (obstacleAngle > 150) {
      DPRINTLN("Obstacle right, turning left to avoid.");
      turnLeft(150);
      delay(500);
      stopMotors();
    } else {
      DPRINTLN("Obstacle center, moving backward.");
      moveBackward(150);
      delay(600);
      stopMotors();
    }
  } else {
    // No obstacle close: drive forward continuously
    DPRINTLN("Path clear, moving forward.");
    moveForward(150);
    // Optionally keep a short delay to let forward motion happen
    delay(100);
  }

  // Return servo to center smoothly
  int returnAngle = (minDistance < 40) ? obstacleAngle : clearestAngle;
  for (int a = returnAngle; a != 90; a += (returnAngle > 90 ? -1 : 1)) {
    ultrasonic_servo.write(mapServoAngle(a));
    delay(10);
  }
  ultrasonic_servo.write(mapServoAngle(90));
  delay(50);

  // ============= IMU CSV Publish (filtered) =============
  if (imu_initialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply offsets
    float ax = a.acceleration.x + ax_offset;
    float ay = a.acceleration.y + ay_offset;
    float az = a.acceleration.z + az_offset;
    float gx = g.gyro.x + gx_offset;
    float gy = g.gyro.y + gy_offset;
    float gz = g.gyro.z + gz_offset;

    // Initialize filter on first sample
    if (!imu_filter_initialized) {
      fax = ax; fay = ay; faz = az;
      fgx = gx; fgy = gy; fgz = gz;
      imu_filter_initialized = true;
    } else {
      // Exponential Moving Average
      fax = FILTER_ALPHA * ax + (1.0f - FILTER_ALPHA) * fax;
      fay = FILTER_ALPHA * ay + (1.0f - FILTER_ALPHA) * fay;
      faz = FILTER_ALPHA * az + (1.0f - FILTER_ALPHA) * faz;
      fgx = FILTER_ALPHA * gx + (1.0f - FILTER_ALPHA) * fgx;
      fgy = FILTER_ALPHA * gy + (1.0f - FILTER_ALPHA) * fgy;
      fgz = FILTER_ALPHA * gz + (1.0f - FILTER_ALPHA) * fgz;
    }

    // Publish ONLY IMU filtered data as CSV to Raspberry Pi
    // Format: fax,fay,faz,fgx,fgy,fgz
    Serial.print(fax, 6); Serial.print(",");
    Serial.print(fay, 6); Serial.print(",");
    Serial.print(faz, 6); Serial.print(",");
    Serial.print(fgx, 6); Serial.print(",");
    Serial.print(fgy, 6); Serial.print(",");
    Serial.println(fgz, 6);
  }
}

// Read ultrasonic distance in cm
float readUltrasonic() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo duration
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Convert to distance (cm)
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (duration * 0.0343) / 2
  float distance = (duration * 0.0343) / 2.0;
  
  // Debug: Log distance reading issues
  if (duration == 0) {
    DPRINTLN("[DEBUG] Ultrasonic: No echo received (timeout)");
  } else if (distance > 400) {
    DPRINTLN("[DEBUG] Ultrasonic: Out of range (>400cm)");
  }
  
  return distance;
}

// Motor control functions
void setMotorA(int speed) {
  motorA_speed = constrain(speed, -255, 255);
  if (motorA_speed > 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else if (motorA_speed < 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  analogWrite(MOTOR_A_PWM, abs(motorA_speed));
}

void setMotorB(int speed) {
  motorB_speed = constrain(speed, -255, 255);
  if (motorB_speed > 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else if (motorB_speed < 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
  analogWrite(MOTOR_B_PWM, abs(motorB_speed));
}

void setMotors(int speedA, int speedB) {
  setMotorA(speedA);
  setMotorB(speedB);
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
}

void moveForward(int speed) {
  setMotors(speed, speed);
}

void moveBackward(int speed) {
  setMotors(-speed, -speed);
}

void turnLeft(int speed) {
  setMotors(0, speed);
}

void turnRight(int speed) {
  setMotors(speed, 0);
}

// Helper function to update servo position (non-blocking)
void updateServo() {
  unsigned long now = millis();
  if (now - last_servo_move >= SERVO_MOVE_DELAY) {
    if (servo_angle < servo_target_angle) {
      servo_angle++;
      ultrasonic_servo.write(mapServoAngle(servo_angle));
      last_servo_move = now;
    } else if (servo_angle > servo_target_angle) {
      servo_angle--;
      ultrasonic_servo.write(mapServoAngle(servo_angle));
      last_servo_move = now;
    }
  }
}

// Wall-follow helpers (DEPRECATED - use ROS2 instead)
// These functions are kept for reference but NOT used in normal operation

// Reference-based: Perform a sweep and find closest object (DEPRECATED)
// DO NOT USE - causes blocking delays that freeze ROS2 control

void processCommand(String cmd) {
  cmd.trim();
  
  // Motor command: MOTOR:speedA,speedB
  // Example: MOTOR:100,-50
  if (cmd.startsWith("MOTOR:")) {
    String speedStr = cmd.substring(6);
    int commaPos = speedStr.indexOf(',');
    if (commaPos > 0) {
      int speedA = speedStr.substring(0, commaPos).toInt();
      int speedB = speedStr.substring(commaPos + 1).toInt();
      setMotors(speedA, speedB);
      DPRINT("ACK:MOTOR:");
      DPRINT(speedA);
      DPRINT(",");
      DPRINTLN(speedB);
    } else {
      DPRINTLN("ERR:MOTOR:Invalid format");
    }
  }
  
  // Stop command: STOP
  else if (cmd == "STOP") {
    stopMotors();
    DPRINTLN("ACK:STOP");
  }
  
  // Servo command: SERVO:angle (0-180)
  // Example: SERVO:45
  else if (cmd.startsWith("SERVO:")) {
    int angle = cmd.substring(6).toInt();
    angle = constrain(angle, 0, 180);
    servo_target_angle = angle;
    DPRINT("ACK:SERVO:");
    DPRINTLN(angle);
  }
  
  // Forward movement: FWD:speed
  else if (cmd.startsWith("FWD:")) {
    int speed = cmd.substring(4).toInt();
    setMotors(speed, speed);
    DPRINT("ACK:FWD:");
    DPRINTLN(speed);
  }
  
  // Backward movement: BWD:speed
  else if (cmd.startsWith("BWD:")) {
    int speed = cmd.substring(4).toInt();
    setMotors(-speed, -speed);
    DPRINT("ACK:BWD:");
    DPRINTLN(speed);
  }
  
  // Turn left: LEFT:speed
  else if (cmd.startsWith("LEFT:")) {
    int speed = cmd.substring(5).toInt();
    setMotors(0, speed);
    DPRINT("ACK:LEFT:");
    DPRINTLN(speed);
  }
  
  // Turn right: RIGHT:speed
  else if (cmd.startsWith("RIGHT:")) {
    int speed = cmd.substring(6).toInt();
    setMotors(speed, 0);
    DPRINT("ACK:RIGHT:");
    DPRINTLN(speed);
  }
  
  // Unknown command
  else {
    DPRINT("ERR:Unknown command: ");
    DPRINTLN(cmd);
  }
}
