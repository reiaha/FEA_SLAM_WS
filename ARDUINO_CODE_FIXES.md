# ✅ Arduino Code Review & Fixes

**Status**: ✅ **CLEANED & FIXED**

---

## **Problems Found & Fixed**

### **❌ Problem 1: Blocking Main Loop**
**Issue**: The main loop was executing a full servo sweep (90→180°) every cycle with 100ms delays per angle, totaling ~1 second per loop. This froze ROS2 motor control.

```cpp
// OLD (BROKEN): Blocking sweep in loop
for (int angle = SWEEP_MIN_ANGLE; angle <= SWEEP_MAX_ANGLE; angle += SWEEP_STEP) {
  ultrasonic_servo.write(mapServoAngle(angle));
  delay(100);  // <-- BLOCKING! Freezes ROS2 for ~1 second
  float distance = readUltrasonic();
}
```

**Fix**: Replaced with **non-blocking** publish-only loop:
```cpp
// NEW (CORRECT): Non-blocking sensor publishing
void loop() {
  // Handle ROS2 commands (non-blocking)
  while (Serial.available() > 0) { ... }
  
  // Update servo smoothly (1° per 20ms) - NON-BLOCKING
  updateServo();
  
  // Publish IMU + ultrasonic (once per 20ms) - NO BLOCKING
  publishSensorData();
  
  delay(20);  // ~50 Hz publish rate
}
```

---

### **❌ Problem 2: Conflicting Control Modes**
**Issue**: Arduino code tried to be both:
1. A ROS2 motor bridge (receiving commands)
2. An autonomous obstacle avoider (running own logic)

Result: Robot received motor commands from ROS2 but couldn't follow them because Arduino was busy sweeping.

**Fix**: Arduino is now **DUMB MOTOR BRIDGE ONLY**:
- ✅ Receives: `MOTOR:speedA,speedB`, `SERVO:angle`
- ✅ Publishes: IMU + ultrasonic data (CSV)
- ❌ Removed: All autonomous sweep, wall-follow, navigation logic
- ✅ All autonomy happens in ROS2 (where it belongs)

---

### **❌ Problem 3: Redundant Code**
**Issue**: IMU/ultrasonic data was being sent **twice** per loop cycle.

**Fix**: Single `publishSensorData()` function called once per loop.

---

### **❌ Problem 4: Unused Blocking Functions**
**Issue**: Functions like `performSweep()`, `servoRangeTest()`, etc. used `delay()` and blocking loops, breaking ROS2 integration.

**Fix**: Removed/deprecated these functions. Arduino should **never block**.

---

## **Current Arduino Responsibilities**

```
┌─────────────────────────────────┐
│    Arduino Motor Bridge (CLEAN)  │
├─────────────────────────────────┤
│                                 │
│  ✅ Receive ROS2 Commands:      │
│     - MOTOR:speedA,speedB       │
│     - SERVO:angle (0-180)       │
│     - FWD/BWD/LEFT/RIGHT        │
│     - STOP                       │
│                                 │
│  ✅ Publish Sensor Data (CSV):  │
│     - IMU (accel/gyro)          │
│     - Ultrasonic distance       │
│     - Motor speeds              │
│     @ 50 Hz (~20ms)             │
│                                 │
│  ✅ Hardware Control:           │
│     - Motor A/B speed PWM       │
│     - Servo position (smooth)   │
│                                 │
│  ❌ NO BLOCKING DELAYS          │
│  ❌ NO AUTONOMOUS LOGIC         │
│  ❌ NO SERVO SWEEPS IN LOOP     │
│                                 │
└─────────────────────────────────┘
```

---

## **Non-Blocking Servo Movement**

Servo now moves smoothly **without blocking** the main loop:

```cpp
const int SERVO_MOVE_DELAY = 20;  // milliseconds per 1-degree step

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
```

**How it works**:
- Every 20ms: servo moves 1 degree closer to target
- 0°→160°: Takes 160 × 20ms = **3.2 seconds**
- ROS2 commands are processed every 20ms (never blocked)

---

## **CSV Data Format**

Arduino publishes sensor data as CSV (one line per 20ms):

```
ax, ay, az, gx, gy, gz, distance, motorA, motorB
0.15, 0.02, 9.81, -2.5, -0.8, 1.2, 25.3, 100, 100
```

Columns:
- `ax, ay, az`: Accel (m/s²)
- `gx, gy, gz`: Gyro (°/s)
- `distance`: Ultrasonic (cm)
- `motorA, motorB`: Current PWM speeds (-255 to 255)

ROS2 `arduino_motor_bridge.py` parses this and publishes to topics.

---

## **Command Format**

All commands end with `\n` (newline):

```
MOTOR:100,-50\n     → Motor A forward 100, Motor B backward 50
SERVO:45\n          → Move servo to 45° (smoothly, non-blocking)
STOP\n              → Stop all motors
FWD:150\n           → Move forward at PWM 150
BWD:100\n           → Move backward at PWM 100
LEFT:120\n          → Turn left at PWM 120
RIGHT:120\n         → Turn right at PWM 120
```

---

## **What Was Removed**

| Removed | Why |
|---------|-----|
| `performSweep()` | Blocked for 1+ seconds |
| `autonomousNavigationCycle()` | Blocking delays |
| `servoRangeTest()` | Not needed in motor bridge |
| `wallFollowStep()` | Belongs in ROS2 |
| `findClearestDirection()` | Belongs in ROS2 |
| Blocking servo loops in main() | Froze ROS2 control |
| `autonomous_mode` flag | Not used here |
| All `delay()` in critical paths | Blocking = bad |

---

## **Testing the Arduino**

### **1. Upload Code**
```bash
# In Arduino IDE:
# - Select correct board (Arduino Uno/Nano)
# - Select correct COM port (/dev/ttyACM0)
# - Click Upload
```

### **2. Test Serial Commands**
```bash
# Terminal 1: Monitor Arduino
screen /dev/ttyACM0 115200

# Terminal 2: Send commands
python3
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(1)

# Test motor
ser.write(b"MOTOR:100,100\n")
print(ser.readline())  # Should print: ACK:MOTOR:100,100

# Test servo
ser.write(b"SERVO:90\n")
print(ser.readline())  # Should print: ACK:SERVO:90

# Test stop
ser.write(b"STOP\n")
print(ser.readline())  # Should print: ACK:STOP
```

### **3. Test with ROS2**
```bash
cd /home/pi/FEA_SLAM_WS
colcon build --packages-select localization fea_slam
source install/setup.bash

# Start the system
ros2 launch fea_slam robot_full.launch.py exploration:=false

# In another terminal, test motor control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Press 'i' to move forward
# Robot should move smoothly without servo interference
```

---

## **Backup**

Original code backed up at:
```
/home/pi/FEA_SLAM_WS/scripts/arduino_complete.ino.backup
```

---

## **Summary**

✅ **Before**: Arduino tried to do autonomous navigation → Blocked ROS2 → Broken system  
✅ **After**: Arduino is clean motor bridge → Non-blocking → Works with ROS2  

**Result**: ROS2 now has full control of motors and servo with 20ms response time.

