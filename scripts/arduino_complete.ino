#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define MIN_SPEED       0
#define MAX_SPEED       255
#define FORWARD_SPEED   210
#define BACKWARD_SPEED  200
#define TURN_SPEED      220

#define SAFETY_STOP_CM      12.0
#define SAFETY_HIT_COUNT    2     
#define SAFETY_CLEAR_COUNT  3    

Adafruit_MPU6050 mpu;

// Calibrated offsets
float gx_offset = -2.8;
float gy_offset = -1.4;
float gz_offset = 0.96;  // recalibrated: raw bias was -0.96 deg/s → offset +0.96 nulls it (measured 2026-03-10)
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// Ultrasonic Pins
const int TRIG_PIN = A0;
const int ECHO_PIN = A1;

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

  // Timer2 (pin 11 = MOTOR_B_PWM): fast PWM, prescaler=1 → 62.5kHz
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  stopMotors();

  // Initialize I2C and MPU6050
  Wire.begin();  // Explicit init: SDA=A4, SCL=A5 on Uno
  delay(100);    // Let I2C bus settle
  initMPU6050();

  Serial.println("IMU-Motor-Ultrasonic System Ready");
}

// Try to initialize MPU6050 — attempts address 0x68 first, then 0x69 (AD0 high)
void initMPU6050() {
  imu_initialized = false;
  Serial.print("Initializing MPU6050...");

  uint8_t addresses[] = {0x68, 0x69};
  for (int a = 0; a < 2 && !imu_initialized; a++) {
    for (int retry = 0; retry < 3 && !imu_initialized; retry++) {
      if (mpu.begin(addresses[a])) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        imu_initialized = true;
        Serial.print(" OK at 0x");
        Serial.println(addresses[a], HEX);
      } else {
        Serial.print(".");
        delay(500);
      }
    }
  }
  if (!imu_initialized) {
    Serial.println(" FAILED - send REINIT_IMU to retry, or check SDA/SCL wiring");
  }
}

// Scan I2C bus and print found addresses
void i2cScan() {
  Serial.println("I2C_SCAN:start");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C_SCAN:found:0x");
      Serial.println(addr, HEX);
      found++;
    }
    delay(5);
  }
  if (found == 0) Serial.println("I2C_SCAN:none_found");
  Serial.println("I2C_SCAN:done");
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
  // Always record the commanded speeds so the CSV feedback is accurate.
  motorA_speed = constrain(speedA, -MAX_SPEED, MAX_SPEED);
  motorB_speed = constrain(speedB, -MAX_SPEED, MAX_SPEED);

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
  } else if (cmd == "REINIT_IMU") {
    // Retry MPU6050 init without power cycling — useful when wiring was loose on boot
    Wire.begin();
    delay(100);
    initMPU6050();
    Serial.print("ACK:REINIT_IMU:");
    Serial.println(imu_initialized ? "OK" : "FAILED");
    Serial.flush();
  } else if (cmd == "I2C_SCAN") {
    // Scan I2C bus and report device addresses — helps diagnose MPU6050 wiring
    i2cScan();
    Serial.flush();
  } else if (cmd.startsWith("AUTO:")) {
    // AUTO:ON / AUTO:OFF — sent by ROS bridge to enable/disable onboard avoidance.
    // This firmware delegates all avoidance to ROS; just acknowledge.
    Serial.print("ACK:"); Serial.println(cmd);
    Serial.flush();
  }
}
