/*
  MOTOR TEST ONLY - Simplified Arduino Code
  No sensors, just motor control for debugging
  
  Motor Control (TB6612FND):
  - Motor A (Left): AIN1=4, AIN2=5, PWMA=6
  - Motor B (Right): BIN1=7, BIN2=8, PWNB=11
  - STBY = 3
*/

// Motor Control Pins (CORRECTED)
const int MOTOR_A_IN1 = 4;   // AIN1
const int MOTOR_A_IN2 = 5;   // AIN2
const int MOTOR_A_PWM = 6;   // PWMA

const int MOTOR_B_IN1 = 8;   // BIN1 (CORRECTED - was 7)
const int MOTOR_B_IN2 = 7;   // BIN2 (CORRECTED - was 8)
const int MOTOR_B_PWM = 11;  // PWMB

const int MOTOR_STBY = 3;    // CRITICAL - must connect to TB6612 STBY pin

// Motor speed variables
int motorA_speed = 0;
int motorB_speed = 0;

// Serial command buffer
String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize Motor Control Pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Enable TB6612FND motor driver
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);  // HIGH = motors enabled
  
  Serial.println("=== MOTOR TEST ONLY ===");
  Serial.println("Pin Configuration:");
  Serial.println("  Motor A: AIN1=4, AIN2=5, PWMA=6");
  Serial.println("  Motor B: BIN1=8, BIN2=7, PWMB=11");
  Serial.println("  STBY=3 (set HIGH)");
  Serial.println("");
  Serial.println("Automatic test starting in 3 seconds...");
  delay(3000);
  
  // Run automatic test sequence
  runAutoTest();
  
  Serial.println("");
  Serial.println("=== TEST COMPLETE ===");
  Serial.println("Send commands: MOTOR:speedA,speedB");
}

void runAutoTest() {
  Serial.println("\n[TEST 1] Motor A Forward (PWM=150)");
  setMotorA(150);
  delay(2000);
  stopMotors();
  Serial.println("  Motor A stopped");
  delay(1000);
  
  Serial.println("\n[TEST 2] Motor B Forward (PWM=150)");
  setMotorB(150);
  delay(2000);
  stopMotors();
  Serial.println("  Motor B stopped");
  delay(1000);
  
  Serial.println("\n[TEST 3] Both Forward (PWM=150)");
  setMotors(150, 150);
  delay(2000);
  stopMotors();
  Serial.println("  Both stopped");
  delay(1000);
  
  Serial.println("\n[TEST 4] Motor A Reverse (PWM=-150)");
  setMotorA(-150);
  delay(2000);
  stopMotors();
  Serial.println("  Motor A stopped");
  delay(1000);
  
  Serial.println("\n[TEST 5] Motor B Reverse (PWM=-150)");
  setMotorB(-150);
  delay(2000);
  stopMotors();
  Serial.println("  Motor B stopped");
  delay(1000);
}

void loop() {
  // Check for serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
  
  // Print motor status every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.print("Status: A=");
    Serial.print(motorA_speed);
    Serial.print(" B=");
    Serial.println(motorB_speed);
    lastPrint = millis();
  }
}

// Motor control functions
void setMotorA(int speed) {
  motorA_speed = constrain(speed, -255, 255);
  
  if (motorA_speed > 0) {
    // Forward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_PWM, abs(motorA_speed));
    Serial.print("  Motor A: FWD, PWM=");
    Serial.println(abs(motorA_speed));
  } 
  else if (motorA_speed < 0) {
    // Backward
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_PWM, abs(motorA_speed));
    Serial.print("  Motor A: REV, PWM=");
    Serial.println(abs(motorA_speed));
  } 
  else {
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_PWM, 0);
    Serial.println("  Motor A: STOP");
  }
}

void setMotorB(int speed) {
  motorB_speed = constrain(speed, -255, 255);
  
  if (motorB_speed > 0) {
    // Forward
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_PWM, abs(motorB_speed));
    Serial.print("  Motor B: FWD, PWM=");
    Serial.println(abs(motorB_speed));
  } 
  else if (motorB_speed < 0) {
    // Backward
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, abs(motorB_speed));
    Serial.print("  Motor B: REV, PWM=");
    Serial.println(abs(motorB_speed));
  } 
  else {
    // Stop
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_PWM, 0);
    Serial.println("  Motor B: STOP");
  }
}

void setMotors(int speedA, int speedB) {
  Serial.println("  Setting both motors:");
  setMotorA(speedA);
  setMotorB(speedB);
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
}

// Process serial commands
void processCommand(String cmd) {
  cmd.trim();
  
  Serial.print("Command received: ");
  Serial.println(cmd);
  
  if (cmd.startsWith("MOTOR:")) {
    String speedStr = cmd.substring(6);
    int commaPos = speedStr.indexOf(',');
    
    if (commaPos > 0) {
      int speedA = speedStr.substring(0, commaPos).toInt();
      int speedB = speedStr.substring(commaPos + 1).toInt();
      
      setMotors(speedA, speedB);
      Serial.print("ACK:MOTOR:");
      Serial.print(speedA);
      Serial.print(",");
      Serial.println(speedB);
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
    Serial.println("ACK:STOP");
  }
}
