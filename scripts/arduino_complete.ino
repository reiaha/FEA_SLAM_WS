#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>


#define MIN_SPEED       0
#define MAX_SPEED       255
#define FORWARD_SPEED   210
#define BACKWARD_SPEED  200
#define TURN_SPEED      220

#define SAFETY_STOP_CM      10.0  // Emergency stop at 10cm (ultrasonic only at close range)
#define SAFETY_HIT_COUNT    3     // Require 3 consecutive hits to trigger
#define SAFETY_CLEAR_COUNT  3     // Require 3 consecutive clears to reset

Adafruit_MPU6050 mpu;

// Calibrated offsets
float gx_offset = -2.8;
float gy_offset = -1.4;
float gz_offset = -1.1;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// Servo + Ultrasonic Pins
const int SERVO_PIN = 10;
const int TRIG_PIN = A0;
const int ECHO_PIN = A1;

// Servo positions (degrees)
#define SERVO_LEFT   110
#define SERVO_CENTER 150
#define SERVO_RIGHT  170
Servo scan_servo;
int servo_angle = SERVO_CENTER;

// Motor Control Pins
const int MOTOR_A_IN1 = 5;   
const int MOTOR_A_IN2 = 4;  
const int MOTOR_A_PWM = 6;   
const int MOTOR_B_IN1 = 7;  
const int MOTOR_B_IN2 = 8;   
const int MOTOR_B_PWM = 11; 
const int MOTOR_STBY = 3;

// Motor speed variables
int motorA_speed = 0;
int motorB_speed = 0;

// Motors enabled by default (Python sends START for confirmation)
bool motors_enabled = true;

// Safety stop state (emergency only)
bool safety_stop_active = false;
int safety_hit_streak = 0;
int safety_clear_streak = 0;

// Serial buffer
String inputBuffer = "";
const char COMMAND_DELIMITER = '\n';

// IMU status
bool imu_initialized = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo setup
  scan_servo.attach(SERVO_PIN);
  scan_servo.write(servo_angle);

  stopMotors();

  // Initialize MPU6050
  Serial.print("Initializing MPU6050...");
  int retries = 3;
  while (retries > 0 && !imu_initialized) {
    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      imu_initialized = true;
      Serial.println(" OK");
    } else {
      retries--;
      Serial.print(".");
      delay(500);
    }
  }
  if (!imu_initialized) Serial.println(" FAILED - Motors still operational");

  Serial.println("IMU-Motor-Ultrasonic System Ready");
}

void loop() {
  // PRIORITY 1: Handle incoming commands (check multiple times)
  for (int i = 0; i < 3; i++) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == COMMAND_DELIMITER) {
        processCommand(inputBuffer);
        inputBuffer = "";
      } else if (c != '\r') {  // Ignore carriage return
        inputBuffer += c;
      }
    }
  }

  // PRIORITY 2: Read sensors
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (imu_initialized) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x + ax_offset;
    ay = a.acceleration.y + ay_offset;
    az = a.acceleration.z + az_offset;

    gx = (g.gyro.x * 57.2958) + gx_offset;
    gy = (g.gyro.y * 57.2958) + gy_offset;
    gz = (g.gyro.z * 57.2958) + gz_offset;
  }

  // Ultrasonic
  float distance = readUltrasonic();

  // PRIORITY 3: Safety stop: emergency signal only (do not stop motors here)
  if (distance > 0 && distance < SAFETY_STOP_CM) {
    safety_hit_streak++;
    safety_clear_streak = 0;
    if (!safety_stop_active && safety_hit_streak >= SAFETY_HIT_COUNT) {
      safety_stop_active = true;
      Serial.print("SAFETY_STOP:1,");
      Serial.println(distance, 2);
      Serial.flush();  // Ensure safety stop is sent immediately
    }
  } else {
    safety_clear_streak++;
    safety_hit_streak = 0;
    if (safety_stop_active && safety_clear_streak >= SAFETY_CLEAR_COUNT) {
      safety_stop_active = false;
      Serial.println("SAFETY_STOP:0");
      Serial.flush();
    }
  }

  // PRIORITY 4: CSV output for Raspberry Pi
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.print(distance, 2); Serial.print(",");
  Serial.print(motorA_speed); Serial.print(",");
  Serial.println(motorB_speed);
  Serial.flush();  // Ensure data is sent

  delay(20); // ~50Hz
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}


void setMotorA(int speed) {
  motorA_speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (motorA_speed > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else if (motorA_speed < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
  }
  analogWrite(MOTOR_A_PWM, abs(motorA_speed));
}

void setMotorB(int speed) {
  motorB_speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (motorB_speed > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else if (motorB_speed < 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
  }
  analogWrite(MOTOR_B_PWM, abs(motorB_speed));
}

void setMotors(int speedA, int speedB) {
  if (!motors_enabled) {
    Serial.println("IGNORED: Motors disabled (send START)");
    return;
  }
  
  setMotorA(speedA);
  setMotorB(speedB);
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
  Serial.println("Motors stopped.");
}

void moveForward(int speed) { 
  setMotors(speed, speed); 
  Serial.print("Moving forward at speed "); Serial.println(speed);
}
void moveBackward(int speed) { 
  setMotors(-speed, -speed); 
  Serial.print("Moving backward at speed "); Serial.println(speed);
}
void turnLeft(int speed) { 
  setMotors(0, speed); 
  Serial.print("Turning left at speed "); Serial.println(speed);
}
void turnRight(int speed) { 
  setMotors(speed, 0); 
  Serial.print("Turning right at speed "); Serial.println(speed);
}


void processCommand(String cmd) {
  cmd.trim();
  
  // Remove any carriage returns
  cmd.replace("\r", "");
  cmd.replace("\n", "");

  if (cmd == "START") {
    motors_enabled = true;
    Serial.println("✅ ACK:START - Motors ENABLED - Safety stop ACTIVE");
    Serial.flush();
    return;
  }
  if (cmd == "STOP") {
    motors_enabled = false;
    stopMotors();
    Serial.println("ACK:STOP");
    Serial.flush();
    return;
  }
  if (cmd.startsWith("SERVO:")) {
    String arg = cmd.substring(6);
    arg.trim();
    if (arg == "LEFT") {
      servo_angle = SERVO_LEFT;
    } else if (arg == "CENTER") {
      servo_angle = SERVO_CENTER;
    } else if (arg == "RIGHT") {
      servo_angle = SERVO_RIGHT;
    } else {
      int angle = arg.toInt();
      servo_angle = constrain(angle, 0, 180);
    }
    scan_servo.write(servo_angle);
    Serial.print("ACK:SERVO:");
    Serial.println(servo_angle);
    Serial.flush();
    return;
  }
  
  // Check if motors are enabled for motor commands
  if (!motors_enabled && cmd.startsWith("MOTOR:")) {
    Serial.println("IGNORED: Motors disabled (send START)");
    Serial.flush();
    return;
  }

  if (cmd.startsWith("MOTOR:")) {
    String speedStr = cmd.substring(6);
    int commaPos = speedStr.indexOf(',');
    if (commaPos > 0) {
      int speedA = speedStr.substring(0, commaPos).toInt();
      int speedB = speedStr.substring(commaPos + 1).toInt();
      setMotors(speedA, speedB);
      Serial.print("ACK:MOTOR:");
      Serial.print(speedA); Serial.print(",");
      Serial.println(speedB);
      Serial.flush();
    }
  } else if (cmd.startsWith("FWD:")) {
    moveForward(FORWARD_SPEED);
    Serial.print("ACK:FWD:"); Serial.println(FORWARD_SPEED);
    Serial.flush();
  } else if (cmd.startsWith("BWD:")) {
    moveBackward(BACKWARD_SPEED);
    Serial.print("ACK:BWD:"); Serial.println(BACKWARD_SPEED);
    Serial.flush();
  } else if (cmd.startsWith("LEFT:")) {
    turnLeft(TURN_SPEED);
    Serial.print("ACK:LEFT:"); Serial.println(TURN_SPEED);
    Serial.flush();
  } else if (cmd.startsWith("RIGHT:")) {
    turnRight(TURN_SPEED);
    Serial.print("ACK:RIGHT:"); Serial.println(TURN_SPEED);
    Serial.flush();
  }
}
