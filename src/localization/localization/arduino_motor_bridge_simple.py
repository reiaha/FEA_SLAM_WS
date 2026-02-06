#!/usr/bin/env python3
"""
Arduino Motor Bridge - SIMPLIFIED VERSION
Core job: Convert ROS2 cmd_vel commands to Arduino PWM motor commands
~150 lines, no extra features
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from tf2_ros import TransformBroadcaster
import serial
import time
import threading
import atexit

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('max_speed', 220)
        self.declare_parameter('min_pwm', 90)
        self.declare_parameter('publish_odom', True)  # Enable odom/TF by default
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('enable_stuck_recovery', True)
        self.declare_parameter('stuck_speed_threshold', 5.0)  # Arduino speed units
        self.declare_parameter('stuck_time', 0.6)             # seconds
        self.declare_parameter('stuck_backup_pwm', 120)       # PWM
        self.declare_parameter('stuck_backup_time', 0.8)      # seconds
        self.declare_parameter('safety_stop_backup_time', 2.0)  # seconds
        self.declare_parameter('safety_stop_backup_pwm', 120)   # PWM
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.publish_odom = self.get_parameter('publish_odom').value
        self.odom_rate = float(self.get_parameter('odom_rate').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.enable_stuck_recovery = self.get_parameter('enable_stuck_recovery').value
        self.stuck_speed_threshold = float(self.get_parameter('stuck_speed_threshold').value)
        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.stuck_backup_pwm = int(self.get_parameter('stuck_backup_pwm').value)
        self.stuck_backup_time = float(self.get_parameter('stuck_backup_time').value)
        self.safety_stop_backup_time = float(self.get_parameter('safety_stop_backup_time').value)
        self.safety_stop_backup_pwm = int(self.get_parameter('safety_stop_backup_pwm').value)
        # Override window to prioritize /cmd_vel over /cmd_vel_nav (seconds)
        self.cmd_vel_override_duration = 0.2
        self.cmd_vel_override_until = 0.0
        
        # Try to open serial port
        self.ser = None
        for port in [serial_port, '/dev/ttyACM0', '/dev/ttyUSB0']:
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(2)  # Wait for Arduino reset
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f'✅ Arduino connected on {port}')
                break
            except:
                pass
        
        self.serial_disabled = False
        if self.ser is None:
            self.serial_disabled = True
            self.get_logger().error('❌ Failed to connect to Arduino - running without serial (odom only)')
        
        # Enable motors
        if not self.serial_disabled:
            try:
                self.ser.write(b'START\n')
                time.sleep(0.1)
                self.get_logger().info('✅ Motors ENABLED')
                
                # Disable Arduino's local obstacle avoidance (ROS2 handles it)
                self.ser.write(b'AUTO:OFF\n')
                time.sleep(0.1)
                self.get_logger().info('✅ Arduino local avoidance DISABLED')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Arduino: {e}')
        
        # Subscribers (prefer /cmd_vel when active)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        
        # Servo control
        self.create_subscription(Int32, '/servo_angle', self.servo_cb, 10)
        self.create_subscription(Float32, '/servo_command', self.servo_cmd_cb, 10)  # For Float32 servo angles
        
        # Ultrasonic distance publisher
        self.ultrasonic_pub = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        self.safety_stop_pub = self.create_publisher(Float32, '/safety_stop', 10)  # Direct safety stop signal

        # Simple odom publisher (integrate cmd_vel)
        self.odom_pub = None
        self.tf_broadcaster = None
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 106)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = 0.0
            self.last_time = self.get_clock().now()
            self.create_timer(1.0 / max(1.0, self.odom_rate), self._publish_odom)
        
        # Start serial reader thread to parse Arduino data
        self.serial_reading = True
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

        # Stuck detection state (uses Arduino motor feedback)
        self.last_cmd_pwm_left = 0
        self.last_cmd_pwm_right = 0
        self.last_cmd_time = 0.0
        self.last_motor_speed_left = None
        self.last_motor_speed_right = None
        self.last_motor_speed_time = 0.0
        self.stuck_start_time = None
        self.stuck_recovery_active = False
        self.stuck_recovery_end_time = 0.0
        self.safety_override_until = 0.0
        self.safety_override_active = False

        if self.enable_stuck_recovery:
            self.create_timer(0.1, self._stuck_recovery_loop)

        # Safety stop override loop (ultrasonic emergency)
        self.create_timer(0.05, self._safety_override_loop)
        
        self.get_logger().info('🚀 Arduino Motor Bridge initialized')

        # Ensure motors stop on shutdown
        atexit.register(self._shutdown_motors)
    
    def cmd_vel_cb(self, msg: Twist):
        """Convert cmd_vel (linear, angular) to motor PWM commands"""
        # Safety override: ignore normal commands while backing up
        if self.safety_override_active:
            return

        if self.stuck_recovery_active:
            return

        # Any /cmd_vel message takes priority for a short window
        self.cmd_vel_override_until = time.time() + self.cmd_vel_override_duration
        
        linear = msg.linear.x      # m/s
        angular = msg.angular.z    # rad/s

        # Store for odom integration
        if self.publish_odom:
            self.last_cmd_linear = linear
            self.last_cmd_angular = angular
        
        # Differential drive kinematics
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # Convert velocity (m/s) to PWM (-255 to +255)
        # max_speed is PWM value (default 220), corresponding to ~0.5 m/s
        # So: pwm = velocity * (max_speed / 0.5)
        pwm_left = int(v_left * (self.max_speed / 0.5))
        pwm_right = int(v_right * (self.max_speed / 0.5))
        
        # Enforce minimum PWM when moving (avoid stall)
        MIN_PWM = self.min_pwm
        if pwm_left != 0:
            sign = 1 if pwm_left > 0 else -1
            pwm_left = sign * max(MIN_PWM, abs(pwm_left))
        if pwm_right != 0:
            sign = 1 if pwm_right > 0 else -1
            pwm_right = sign * max(MIN_PWM, abs(pwm_right))
        
        # Clamp to valid PWM range
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))

        self.last_cmd_pwm_left = pwm_left
        self.last_cmd_pwm_right = pwm_right
        self.last_cmd_time = time.time()
        
        # Send to Arduino
        # Send to Arduino (if available)
        self._send_motor_pwm(pwm_left, pwm_right)

    def cmd_vel_nav_cb(self, msg: Twist):
        """Handle cmd_vel_nav only when no override is active"""
        if time.time() < self.cmd_vel_override_until:
            return  # Ignore nav commands during override window
        self.cmd_vel_cb(msg)
    
    def servo_cb(self, msg: Int32):
        """Send servo angle (0-180 degrees) to Arduino"""
        if self.ser is None:
            return
        
        angle = max(0, min(180, msg.data))
        try:
            cmd = f"SERVO:{angle}\n"
            self.ser.write(cmd.encode())
            self.get_logger().debug(f'Servo: {angle}°')
        except Exception as e:
            self.get_logger().error(f'Serial write error (servo): {e}')

    def servo_cmd_cb(self, msg: Float32):
        """Send servo angle from Float32 command (for sweeping)"""
        if self.ser is None:
            return
        
        angle = int(max(0, min(180, msg.data)))
        try:
            cmd = f"SERVO:{angle}\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f'🔄 Servo sweep: {angle}°')
        except Exception as e:
            self.get_logger().error(f'Serial write error (servo): {e}')

    def _publish_odom(self):
        if not self.publish_odom:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:  # Skip if too long (initialization or clock jump)
            self.last_time = now
            return

        v = self.last_cmd_linear
        w = self.last_cmd_angular

        # Integrate pose
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # Use slightly backdated timestamp to avoid "future" TF errors
        # Publish at current time minus 10ms to ensure laser scans arrive "after"
        stamp = now - rclpy.duration.Duration(seconds=0.01)

        # Publish TF first (it's the source of truth for pose)
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Then publish odom message
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        self.last_time = now
    
    def _get_action_name(self, left, right):
        """Convert PWM values to human-readable action"""
        if left == 0 and right == 0:
            return '⏸️  STOPPED'
        elif left > 0 and right > 0:
            return '⬆️  FORWARD'
        elif left < 0 and right < 0:
            return '⬇️  BACKWARD'
        elif left > 0 and right == 0:
            return '⤴️  PIVOT LEFT'
        elif left == 0 and right > 0:
            return '⤵️  PIVOT RIGHT'
        elif left < right:
            return '↖️  FORWARD + SLIGHT LEFT'
        elif right < left:
            return '↗️  FORWARD + SLIGHT RIGHT'
        elif abs(left - right) > 50:
            return '↺  TURN'
        return '↔️  MIXED'

    def _send_motor_pwm(self, pwm_left, pwm_right, action_override=None, log_level='info'):
        try:
            if self.ser is None:
                return
            cmd = f"MOTOR:{pwm_left},{pwm_right}\n"
            self.ser.write(cmd.encode())

            action = action_override or self._get_action_name(pwm_left, pwm_right)
            msg = f'🚀 {action} | MOTOR:{pwm_left},{pwm_right}'
            if log_level == 'warn':
                self.get_logger().warn(msg)
            elif log_level == 'error':
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _stuck_recovery_loop(self):
        if self.ser is None:
            return

        now = time.time()

        if self.stuck_recovery_active:
            if now >= self.stuck_recovery_end_time:
                self.stuck_recovery_active = False
                self.stuck_start_time = None
                self._send_motor_pwm(0, 0, action_override='⏸️  STOPPED')
            else:
                self._send_motor_pwm(
                    -abs(self.stuck_backup_pwm),
                    -abs(self.stuck_backup_pwm),
                    action_override='⬇️  BACKWARD (STUCK RECOVERY)',
                    log_level='warn'
                )
            return

        if self.last_motor_speed_left is None or self.last_motor_speed_right is None:
            return
        if (now - self.last_motor_speed_time) > 0.5:
            return

        cmd_active = (abs(self.last_cmd_pwm_left) >= 120 or abs(self.last_cmd_pwm_right) >= 120)
        speeds_low = (abs(self.last_motor_speed_left) <= self.stuck_speed_threshold and
                      abs(self.last_motor_speed_right) <= self.stuck_speed_threshold)

        if cmd_active and speeds_low:
            if self.stuck_start_time is None:
                self.stuck_start_time = now
            elif (now - self.stuck_start_time) >= self.stuck_time:
                self.stuck_recovery_active = True
                self.stuck_recovery_end_time = now + self.stuck_backup_time
                self.get_logger().warn(
                    f"⚠️ Motors appear stuck (cmd PWM {self.last_cmd_pwm_left},{self.last_cmd_pwm_right} | "
                    f"speed {self.last_motor_speed_left},{self.last_motor_speed_right}). Backing up."
                )
        else:
            self.stuck_start_time = None

    def _safety_override_loop(self):
        if self.ser is None:
            return

        now = time.time()
        if now < self.safety_override_until:
            self._send_motor_pwm(
                -abs(self.safety_stop_backup_pwm),
                -abs(self.safety_stop_backup_pwm),
                action_override='⬇️  BACKWARD (SAFETY OVERRIDE)',
                log_level='warn'
            )
            return

        if self.safety_override_active:
            self.safety_override_active = False
            self._send_motor_pwm(0, 0, action_override='⏸️  STOPPED')
    
    def _serial_reader(self):
        """Background thread to read and parse Arduino CSV data"""
        if self.ser is None:
            return
        
        while self.serial_reading and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Parse SAFETY_STOP format: SAFETY_STOP:1,distance_cm or SAFETY_STOP:0
                    if line.startswith('SAFETY_STOP'):
                        self.get_logger().warn(f'🚨 Arduino: {line}')
                        try:
                            if ',' in line:
                                # Format: SAFETY_STOP:1,16.24 - OBSTACLE DETECTED
                                parts = line.split(',')
                                if len(parts) >= 2:
                                    distance_cm = float(parts[1])
                                    distance_m = distance_cm / 100.0  # Convert cm to meters
                                    
                                    # Publish to both topics
                                    msg = Float32()
                                    msg.data = distance_m
                                    self.ultrasonic_pub.publish(msg)
                                    self.safety_stop_pub.publish(msg)  # Explicit safety stop with distance
                                    self.get_logger().error(f'🚨 SAFETY_STOP! Published /safety_stop: {distance_cm}cm ({distance_m:.4f}m)')

                                    # Activate safety override backup (local motor control)
                                    now = time.time()
                                    self.safety_override_until = max(self.safety_override_until, now + self.safety_stop_backup_time)
                                    if not self.safety_override_active:
                                        self.safety_override_active = True
                                        self.get_logger().warn(
                                            f'🚨 SAFETY OVERRIDE: backing up for {self.safety_stop_backup_time:.1f}s'
                                        )
                            else:
                                # Format: SAFETY_STOP:0 - OBSTACLE CLEARED
                                msg = Float32()
                                msg.data = 999.0  # Sentinel value: no obstacle
                                self.safety_stop_pub.publish(msg)
                                self.get_logger().warn(f'✅ SAFETY_STOP:0 - Obstacle cleared')
                        except (ValueError, IndexError) as e:
                            self.get_logger().error(f'Failed to parse SAFETY_STOP: {line} - {e}')
                    # Parse CSV data: ax,ay,az,gx,gy,gz,distance,motorA_speed,motorB_speed
                    elif line and ',' in line and not line.startswith('ACK'):
                        parts = line.split(',')
                        if len(parts) >= 7:
                            try:
                                distance_raw = float(parts[6])  # Index 6 is distance in cm
                                # Arduino sends in cm, convert to meters
                                if distance_raw > 1.0:
                                    distance_m = distance_raw / 100.0
                                else:
                                    distance_m = distance_raw
                                
                                msg = Float32()
                                msg.data = distance_m
                                self.ultrasonic_pub.publish(msg)
                            except (ValueError, IndexError):
                                pass  # Skip malformed lines

                            # Optional motor speed feedback (indices 7,8)
                            if len(parts) >= 9:
                                try:
                                    self.last_motor_speed_left = float(parts[7])
                                    self.last_motor_speed_right = float(parts[8])
                                    self.last_motor_speed_time = time.time()
                                except (ValueError, IndexError):
                                    pass
                    elif line.startswith('ACK'):
                        self.get_logger().debug(f'✓ {line}')
                
                time.sleep(0.01)  # Don't busy-wait
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.1)

    def _shutdown_motors(self):
        """Send a STOP command to the Arduino on shutdown."""
        try:
            if self.ser is not None:
                self.ser.write(b"MOTOR:0,0\n")
                self.ser.write(b"STOP\n")
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorBridge()
    try:
        rclpy.spin(node)
    finally:
        node._shutdown_motors()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
