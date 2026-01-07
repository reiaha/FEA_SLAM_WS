/*
  Complete Arduino Code for FEA-SLAM Robot
  
  Sensors:
  - MPU-6050 IMU (I2C: SDA=A4, SCL=A5)
  - HC-SR04 Ultrasonic (TRIG=8, ECHO=9)
  
  Motor Control (TB6612FND):
  - Motor A (Left): IN1=2, IN2=3, PWM=5
  - Motor B (Right): IN1=4, IN2=7, PWM=6
  
  Data Format (CSV):
  ax,ay,az,gx,gy,gz,distance,motorA,motorB
*/

#include <Wire.h>
#include <MPU6050.h>

// MPU-6050 Configuration
MPU6050 mpu;
const float ACCEL_SCALE = 16384.0;  // For ±2g
const float GYRO_SCALE = 131.0;     // For ±250 deg/s

// Calibrated offsets (near 0)
float gx_offset = -2.8;
float gy_offset = -1.4;
float gz_offset = -1.1;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// Ultrasonic Sensor Pins
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

// Motor Control Pins
// Motor A (Left)
const int MOTOR_A_IN1 = 2;
const int MOTOR_A_IN2 = 3;
const int MOTOR_A_PWM = 5;

// Motor B (Right)
const int MOTOR_B_IN1 = 4;
const int MOTOR_B_IN2 = 7;
const int MOTOR_B_PWM = 6;

// Motor speed variables
int motorA_speed = 0;
int motorB_speed = 0;

// Serial communication buffer
String inputBuffer = "";
const char COMMAND_DELIMITER = '\n';

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize MPU-6050
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("MPU6050 init failed!");
    while(1);
  }
  
  // Configure MPU-6050
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);
  mpu.setRate(49);  // ~50Hz
  
  delay(100);
  
  // Initialize Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize Motor Control Pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Stop motors at startup
  stopMotors();
  
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
  
  // Read IMU
  Vector rawAccel = mpu.getRawAcceleration();
  Vector rawGyro = mpu.getRawRotation();
  
  // Convert to m/s² and deg/s with calibration offsets
  float ax = (rawAccel.XAxis / ACCEL_SCALE) + ax_offset;
  float ay = (rawAccel.YAxis / ACCEL_SCALE) + ay_offset;
  float az = (rawAccel.ZAxis / ACCEL_SCALE) + az_offset;
  
  float gx = (rawGyro.XAxis / GYRO_SCALE) + gx_offset;
  float gy = (rawGyro.YAxis / GYRO_SCALE) + gy_offset;
  float gz = (rawGyro.ZAxis / GYRO_SCALE) + gz_offset;
  
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
