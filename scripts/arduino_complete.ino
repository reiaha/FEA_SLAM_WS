/*
  Complete Arduino Code for FEA-SLAM Robota
  
  Sensors:
  - MPU-6050 IMU (I2C: SDA=A4, SCL=A5)
  - HC-SR04 Ultrasonic (TRIG=8, ECHO=9)
  
  Motor Control (TB6612FND):
  - Motor A (Left): IN1=2, IN2=3, PWM=5
  - Motor B (Right): IN1=4, IN2=7, PWM=6
  
  Data Format (CSV):
  ax,ay,az,gx,gy,gz,distance,motorA,motorB
  
  Library Required: Adafruit MPU6050
  Install via: Sketch -> Include Library -> Manage Libraries -> "Adafruit MPU6050"
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU-6050 Configuration
Adafruit_MPU6050 mpu;

// Calibrated offsets (near 0)
float gx_offset = -2.8;
float gy_offset = -1.4;
float gz_offset = -1.1;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// Ultrasonic Sensor Pins
const int TRIG_PIN = A0;
const int ECHO_PIN = A1;

// Motor Control Pins
// Motor A (Left)
const int MOTOR_A_IN1 = 4;   // AIN1
const int MOTOR_A_IN2 = 5;   // AIN2
const int MOTOR_A_PWM = 6;   // PWMA

// Motor B (Right)
const int MOTOR_B_IN1 = 8;   // BIN1 (CORRECTED)
const int MOTOR_B_IN2 = 7;   // BIN2 (CORRECTED)
const int MOTOR_B_PWM = 11;  // PWMB

// TB6612FND Standby Pin (CRITICAL - must be HIGH to enable driver)
const int MOTOR_STBY = 3;

// Motor speed variables
int motorA_speed = 0;
int motorB_speed = 0;

// Serial communication buffer
String inputBuffer = "";
const char COMMAND_DELIMITER = '\n';

// Global flag to track if IMU is working
bool imu_initialized = false;

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
  digitalWrite(MOTOR_STBY, HIGH);  // Enable motor driver immediately
  
  // Initialize Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Stop motors at startup
  stopMotors();
  
  // Try to Initialize MPU-6050 (non-blocking - continue if fails)
  Serial.print("Initializing MPU6050...");
  int mpu_retries = 3;
  while (mpu_retries > 0 && !imu_initialized) {
    if (mpu.begin()) {
      // Configure MPU-6050
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      imu_initialized = true;
      Serial.println(" OK");
    } else {
      mpu_retries--;
      Serial.print(".");
      delay(500);
    }
  }
  
  if (!imu_initialized) {
    Serial.println(" FAILED - Motors still operational");
  }
  
  delay(100);
  Serial.println("IMU-Motor-Ultrasonic System Ready");
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
  
  // Read IMU only if initialized
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  
  if (imu_initialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Apply calibration offsets
    // Acceleration in m/s²
    ax = a.acceleration.x + ax_offset;
    ay = a.acceleration.y + ay_offset;
    az = a.acceleration.z + az_offset;
    
    // Gyro in deg/s (converted from rad/s to deg/s)
    gx = (g.gyro.x * 57.2958) + gx_offset;
    gy = (g.gyro.y * 57.2958) + gy_offset;
    gz = (g.gyro.z * 57.2958) + gz_offset;
  }
  
  // Read Ultrasonic
  float distance = readUltrasonic();
  
  // Send data as CSV
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.print(distance, 2); Serial.print(",");
  Serial.print(motorA_speed); Serial.print(",");
  Serial.println(motorB_speed);
  
  delay(20);  // ~50Hz
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
  
  return distance;
}

// Motor control functions
void setMotorA(int speed) {
  motorA_speed = constrain(speed, -255, 255);
  if (motorA_speed > 0) {
    // Forward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (motorA_speed < 0) {
    // Backward
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  analogWrite(MOTOR_A_PWM, abs(motorA_speed));
}

void setMotorB(int speed) {
  motorB_speed = constrain(speed, -255, 255);
  if (motorB_speed > 0) {
    // Forward
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else if (motorB_speed < 0) {
    // Backward
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else {
    // Stop
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

// Process incoming serial commands from ROS2
void processCommand(String cmd) {
  cmd.trim();  // Remove whitespace
  
  // Command format: MOTOR:speedA,speedB
  // Example: "MOTOR:100,-50" - Motor A forward at 100, Motor B backward at 50
  
  if (cmd.startsWith("MOTOR:")) {
    // Extract motor speeds
    String speedStr = cmd.substring(6);  // Remove "MOTOR:"
    
    // Find comma separator
    int commaPos = speedStr.indexOf(',');
    if (commaPos > 0) {
      int speedA = speedStr.substring(0, commaPos).toInt();
      int speedB = speedStr.substring(commaPos + 1).toInt();
      
      setMotors(speedA, speedB);
      
      // Send acknowledgment
      Serial.print("ACK:MOTOR:");
      Serial.print(speedA); Serial.print(",");
      Serial.println(speedB);
    }
  }
  // Command format: STOP
  else if (cmd == "STOP") {
    stopMotors();
    Serial.println("ACK:STOP");
  }
  // Command format: FWD:speed
  else if (cmd.startsWith("FWD:")) {
    int speed = cmd.substring(4).toInt();
    moveForward(speed);
    Serial.print("ACK:FWD:"); Serial.println(speed);
  }
  // Command format: BWD:speed
  else if (cmd.startsWith("BWD:")) {
    int speed = cmd.substring(4).toInt();
    moveBackward(speed);
    Serial.print("ACK:BWD:"); Serial.println(speed);
  }
  // Command format: LEFT:speed
  else if (cmd.startsWith("LEFT:")) {
    int speed = cmd.substring(5).toInt();
    turnLeft(speed);
    Serial.print("ACK:LEFT:"); Serial.println(speed);
  }
  // Command format: RIGHT:speed
  else if (cmd.startsWith("RIGHT:")) {
    int speed = cmd.substring(6).toInt();
    turnRight(speed);
    Serial.print("ACK:RIGHT:"); Serial.println(speed);
  }
}
