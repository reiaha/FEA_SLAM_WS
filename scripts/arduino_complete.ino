#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>


#define MIN_SPEED       0
#define MAX_SPEED       255
#define FORWARD_SPEED   210
#define BACKWARD_SPEED  200
#define TURN_SPEED      220

#define OBSTACLE_CM         30.0
#define OBSTACLE_CLEAR_CM   35.0
#define AVOID_BACKUP_MS     800
#define AVOID_TURN_MS       900

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

// Safety: motors disabled until START command
bool motors_enabled = false;

// Local autonomy (obstacle avoidance on Arduino). ALWAYS ENABLED for safety!
// ROS2 commands still work - Arduino checks obstacles BEFORE applying motor speeds
bool local_autonomy_enabled = true;

// Obstacle avoidance state
enum AvoidState { AVOID_NONE, AVOID_BACKUP, AVOID_TURN };
AvoidState avoid_state = AVOID_NONE;
unsigned long avoid_start_ms = 0;
int turn_direction = 1; // 1 = left, -1 = right

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
  // Handle incoming commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == COMMAND_DELIMITER) {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // IMU data
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

  // Obstacle avoidance override (prevents corner revisits)
  if (local_autonomy_enabled) {
    handleObstacle(distance);
  }

  // CSV output for Raspberry Pi
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.print(distance, 2); Serial.print(",");
  Serial.print(motorA_speed); Serial.print(",");
  Serial.println(motorB_speed);

  delay(20); // ~50Hz
}

void handleObstacle(float distance_cm) {
  if (!motors_enabled) {
    avoid_state = AVOID_NONE;
    return;
  }

  unsigned long now = millis();

  if (avoid_state == AVOID_NONE) {
    if (distance_cm > 0 && distance_cm < OBSTACLE_CM) {
      avoid_state = AVOID_BACKUP;
      avoid_start_ms = now;
      turn_direction = (now / 1000) % 2 == 0 ? 1 : -1; // alternate turns
      Serial.print("🚨 OBSTACLE DETECTED at ");
      Serial.print(distance_cm);
      Serial.println("cm - STARTING BACKUP");
    }
  }

  if (avoid_state == AVOID_BACKUP) {
    moveBackward(BACKWARD_SPEED);
    if (now - avoid_start_ms >= AVOID_BACKUP_MS) {
      avoid_state = AVOID_TURN;
      avoid_start_ms = now;
      Serial.print("⏱️  TURNING ");
      Serial.println(turn_direction > 0 ? "LEFT" : "RIGHT");
    }
    return;
  }

  if (avoid_state == AVOID_TURN) {
    if (turn_direction > 0) {
      turnLeft(TURN_SPEED);
    } else {
      turnRight(TURN_SPEED);
    }

    if (now - avoid_start_ms >= AVOID_TURN_MS) {
      if (distance_cm > OBSTACLE_CLEAR_CM) {
        avoid_state = AVOID_NONE;
        Serial.print("✅ OBSTACLE CLEARED (distance=");
        Serial.print(distance_cm);
        Serial.println("cm)");
      } else {
        avoid_state = AVOID_BACKUP;
        avoid_start_ms = now;
        Serial.println("⚠️  Still blocked - BACKUP again");
      }
    }
    return;
  }
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
  
  // If obstacle avoidance is active, obstacle avoidance has priority
  // But we still acknowledge the ROS2 command so Nav2 stays aware
  if (local_autonomy_enabled && avoid_state != AVOID_NONE) {
    Serial.print("OVERRIDE: Obstacle avoidance active (state=");
    Serial.print(avoid_state); Serial.println(")");
    return;  // Don't apply ROS2 speeds, let handleObstacle() control motors
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

  if (cmd == "START") {
    motors_enabled = true;
    Serial.println("✅ ACK:START - Motors ENABLED - Obstacle avoidance ACTIVE");
    return;
  }
  if (cmd == "STOP") {
    motors_enabled = false;
    avoid_state = AVOID_NONE;
    stopMotors();
    Serial.println("ACK:STOP");
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
    return;
  }
  if (cmd == "AUTO:ON") {
    local_autonomy_enabled = true;
    Serial.println("ACK:AUTO:ON");
    return;
  }
  if (cmd == "AUTO:OFF") {
    local_autonomy_enabled = false;
    Serial.println("ACK:AUTO:OFF");
    return;
  }
  if (!motors_enabled) {
    Serial.println("IGNORED: Motors disabled (send START)");
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
    }
  } else if (cmd.startsWith("FWD:")) {
    moveForward(FORWARD_SPEED);
    Serial.print("ACK:FWD:"); Serial.println(FORWARD_SPEED);
  } else if (cmd.startsWith("BWD:")) {
    moveBackward(BACKWARD_SPEED);
    Serial.print("ACK:BWD:"); Serial.println(BACKWARD_SPEED);
  } else if (cmd.startsWith("LEFT:")) {
    turnLeft(TURN_SPEED);
    Serial.print("ACK:LEFT:"); Serial.println(TURN_SPEED);
  } else if (cmd.startsWith("RIGHT:")) {
    turnRight(TURN_SPEED);
    Serial.print("ACK:RIGHT:"); Serial.println(TURN_SPEED);
  }
}
