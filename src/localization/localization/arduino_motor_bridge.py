#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int32
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import numpy as np

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('max_speed', 255)    # INCREASED from 200 to 255 (max PWM)
        # Motion shaping
        self.declare_parameter('velocity_deadband', 0.015)  #Lower for more sensitivity; higher for more stability
        self.declare_parameter('min_pwm', 120)  # INCREASED to 120 for heavier robot - ensures motors always have enough torque
        self.declare_parameter('pwm_slew_rate', 255)  # INSTANT response - no ramping delay for heavy robot
        self.declare_parameter('recovery_speed_threshold', 0.35)  # m/s - skip slew limit only for recovery/unstuck moves
        self.declare_parameter('ultrasonic_zero_means_no_echo', True)
        self.declare_parameter('use_cmd_vel_for_odom', True)  # Enable temporarily until real encoders added
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.ultrasonic_zero_means_no_echo = self.get_parameter('ultrasonic_zero_means_no_echo').value
        self.use_cmd_vel_for_odom = self.get_parameter('use_cmd_vel_for_odom').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.pwm_slew_rate = int(self.get_parameter('pwm_slew_rate').value)
        self.recovery_speed_threshold = float(self.get_parameter('recovery_speed_threshold').value)
        self.last_cmd_vel_log_time = 0.0
        self.last_no_serial_log_time = 0.0
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_left = 0
        self.last_cmd_right = 0
        
        # Subscriber for motor commands (accept both cmd_vel and cmd_vel_nav)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.cmd_vel_callback, 10)

        # Subscriber for servo commands (angle in degrees)
        self.servo_sub = self.create_subscription(
            Int32, 'servo_angle', self.servo_angle_callback, 10)
        
        # Create timer to publish odometry periodically
        self.odom_timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        
        # Serial connection
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.reconnect_backoff = 0.1  # Start with 100ms, exponential backoff
        self.last_reconnect_attempt = 0  # Timestamp of last attempt
        try:
            ports_to_try = []
            for port in [serial_port, '/dev/ttyACM0', '/dev/ttyUSB0']:
                if port and port not in ports_to_try:
                    ports_to_try.append(port)

            self.ser = None
            for port in ports_to_try:
                try:
                    self.ser = serial.Serial(port, baud_rate, timeout=1)
                    self.serial_port = port
                    time.sleep(2)  # Allow Arduino reset
                    # Flush any residual data from startup/reset
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    self.get_logger().info(f'Arduino connected on {port}')
                    break
                except serial.SerialException as e:
                    self.get_logger().warn(f'Failed to open {port}: {e}')

            if self.ser is None:
                raise serial.SerialException(f'No usable serial port found (tried: {ports_to_try})')
            
            # Auto-enable motors for autonomous operation
            self.ser.write(b'START\n')
            time.sleep(0.1)
            self.get_logger().info('✅ Motors ENABLED - Robot ready for autonomous operation')
            
            # Disable Arduino's local obstacle avoidance (let ROS2 handle it)
            self.ser.write(b'AUTO:OFF\n')
            time.sleep(0.1)
            self.get_logger().info('✅ Arduino local avoidance DISABLED - ROS2 has full control')
            
            self.reconnect_backoff = 0.1  # Reset backoff on success
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None
            
        # Timer for reading sensor data
        self.timer = self.create_timer(0.02, self.read_sensors)  # 50Hz
        
        self.get_logger().info('Arduino Motor Bridge initialized')
    
    def try_reconnect(self):
        """Attempt to reconnect to Arduino if connection is lost (with backoff)"""
        if self.ser is None:
            now = time.time()
            # Only try to reconnect if backoff delay has passed
            if now - self.last_reconnect_attempt < self.reconnect_backoff:
                return
            
            self.last_reconnect_attempt = now
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                time.sleep(0.5)  # Give Arduino time to reset
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f'✅ Reconnected to Arduino on {self.serial_port}')
                # Re-enable motors
                self.ser.write(b'START\n')
                time.sleep(0.1)
                self.get_logger().info('✅ Motors re-enabled')
                self.reconnect_backoff = 0.1  # Reset backoff on success
            except Exception as e:
                # Exponential backoff: 0.1s → 0.2s → 0.4s → 0.8s → cap at 2s
                self.reconnect_backoff = min(2.0, self.reconnect_backoff * 2)
                self.get_logger().debug(f'Reconnection failed (retry in {self.reconnect_backoff:.2f}s): {e}')
                self.ser = None
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel (linear.x, angular.z) to differential drive motor speeds
        """
        if not self.ser:
            now = time.time()
            if now - self.last_no_serial_log_time >= 1.0:
                self.get_logger().error('cmd_vel received but serial not connected to Arduino')
                self.last_no_serial_log_time = now
            return
        
        linear = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s
        
        # Track velocity for odometry integration (optional)
        if self.use_cmd_vel_for_odom:
            self.last_linear = linear
            self.last_angular = angular
        
        # Differential drive kinematics
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # Apply deadband on velocities
        if abs(v_left) < self.velocity_deadband:
            speed_left = 0
        else:
            speed_left = int(v_left * (self.max_speed / 0.5))
        if abs(v_right) < self.velocity_deadband:
            speed_right = 0
        else:
            speed_right = int(v_right * (self.max_speed / 0.5))

        # Enforce minimum PWM once moving
        if speed_left != 0:
            sign_l = 1 if speed_left > 0 else -1
            speed_left = sign_l * max(self.min_pwm, abs(speed_left))
        if speed_right != 0:
            sign_r = 1 if speed_right > 0 else -1
            speed_right = sign_r * max(self.min_pwm, abs(speed_right))

        # Constrain to valid range
        speed_left = max(-255, min(255, speed_left))
        speed_right = max(-255, min(255, speed_right))
        
        # Send command to Arduino
        # Slew-limit to reduce jerkiness (skip during recovery/high-power commands)
        if abs(linear) < self.recovery_speed_threshold:
            speed_left = self.slew_limit(self.last_cmd_left, speed_left)
            speed_right = self.slew_limit(self.last_cmd_right, speed_right)

        # Re-enforce minimum PWM AFTER slew limiting to avoid weak commands
        if speed_left != 0:
            sign_l = 1 if speed_left > 0 else -1
            speed_left = sign_l * max(self.min_pwm, abs(speed_left))
        if speed_right != 0:
            sign_r = 1 if speed_right > 0 else -1
            speed_right = sign_r * max(self.min_pwm, abs(speed_right))

        # Final constrain
        speed_left = max(-255, min(255, speed_left))
        speed_right = max(-255, min(255, speed_right))
        self.last_cmd_left = speed_left
        self.last_cmd_right = speed_right

        now = time.time()
        if now - self.last_cmd_vel_log_time >= 1.0:
            self.get_logger().info(
                f'cmd_vel: linear={linear:.2f} m/s angular={angular:.2f} rad/s -> PWM L={speed_left} R={speed_right}'
            )
            self.last_cmd_vel_log_time = now

        command = f"MOTOR:{speed_left},{speed_right}\n"
        try:
            self.ser.write(command.encode())
            
            # Decode and display human-readable action
            action = self.decode_motor_action(speed_left, speed_right)
            self.get_logger().info(f'🚀 {action} | MOTOR:{speed_left},{speed_right}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def servo_angle_callback(self, msg: Int32):
        """Send servo angle command to Arduino (0-180)"""
        if not self.ser:
            return

        angle = max(0, min(180, int(msg.data)))
        command = f"SERVO:{angle}\n"
        try:
            self.ser.write(command.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write error (servo): {e}')

    def slew_limit(self, current, target):
        """Limit PWM change per cycle to smooth motion"""
        delta = target - current
        if abs(delta) <= self.pwm_slew_rate:
            return target
        return current + self.pwm_slew_rate * (1 if delta > 0 else -1)
    
    def decode_motor_action(self, left, right):
        """Convert motor speeds to human-readable action"""
        if left == 0 and right == 0:
            return "⏸️  STOPPED"
        
        # Both wheels moving forward
        if left > 0 and right > 0:
            if abs(left - right) < 20:  # Similar speeds
                return "⬆️  FORWARD"
            elif left > right:
                return "↗️  FORWARD + SLIGHT RIGHT"
            else:
                return "↖️  FORWARD + SLIGHT LEFT"
        
        # Both wheels moving backward
        if left < 0 and right < 0:
            if abs(left - right) < 20:
                return "⬇️  BACKWARD"
            elif abs(left) > abs(right):
                return "↙️  BACKWARD + SLIGHT RIGHT"
            else:
                return "↘️  BACKWARD + SLIGHT LEFT"
        
        # Opposite directions = turning in place
        if left > 0 and right < 0:
            return "↻  TURN RIGHT (rotate)"
        if left < 0 and right > 0:
            return "↺  TURN LEFT (rotate)"
        
        # One wheel stopped, other moving
        if left == 0 and right > 0:
            return "⤴️  PIVOT LEFT"
        if left == 0 and right < 0:
            return "⤵️  PIVOT LEFT (back)"
        if right == 0 and left > 0:
            return "⤴️  PIVOT RIGHT"
        if right == 0 and left < 0:
            return "⤵️  PIVOT RIGHT (back)"
        
        return "❓ UNKNOWN"
    
    def read_sensors(self):
        """Read sensor data from Arduino"""
        if not self.ser:
            # Try to reconnect if disconnected
            self.try_reconnect()
            return
        
        try:
            # Check if data is available with error handling
            if not self.ser.in_waiting:
                return
        except (OSError, serial.SerialException) as e:
            # Serial port error - likely disconnected or I/O issue
            self.get_logger().warn(f'Serial port error during check: {e}')
            # Don't crash, just disconnect and try reconnecting later
            try:
                if self.ser:
                    self.ser.close()
            except:
                pass
            self.ser = None
            return
        
        try:
            # Read one line (up to newline)
            line = self.ser.readline()
            if not line:
                return
            
            # Decode and strip whitespace
            line = line.decode('utf-8', errors='ignore').strip()
            
            if not line:
                return
            
            # Skip acknowledgment and debug messages
            if line.startswith('ACK:'):
                self.get_logger().debug(f'Arduino ACK: {line}')
                return
            
            if line.startswith('[DEBUG]'):
                self.get_logger().debug(f'Arduino debug: {line}')
                return
            
            # Parse CSV: ax,ay,az,gx,gy,gz,distance,motorA,motorB
            parts = line.split(',')
            if len(parts) != 9:
                self.get_logger().debug(f'Invalid CSV (expected 9 fields, got {len(parts)}): {line[:50]}')
                return
            
            # Convert to floats
            ax, ay, az, gx, gy, gz, distance, motorA, motorB = map(float, parts)
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Acceleration (convert g to m/s²)
            imu_msg.linear_acceleration.x = ax * 9.80665
            imu_msg.linear_acceleration.y = ay * 9.80665
            imu_msg.linear_acceleration.z = az * 9.80665
            
            # Angular velocity (convert deg/s to rad/s)
            imu_msg.angular_velocity.x = gx * math.pi / 180.0
            imu_msg.angular_velocity.y = gy * math.pi / 180.0
            imu_msg.angular_velocity.z = gz * math.pi / 180.0
            
            self.imu_pub.publish(imu_msg)
            
            # Publish Ultrasonic data
            ultrasonic_msg = Range()
            ultrasonic_msg.header = Header()
            ultrasonic_msg.header.stamp = self.get_clock().now().to_msg()
            ultrasonic_msg.header.frame_id = 'ultrasonic_link'
            ultrasonic_msg.radiation_type = Range.ULTRASOUND
            ultrasonic_msg.field_of_view = 0.26  # ~15 degrees
            ultrasonic_msg.min_range = 0.02  
            ultrasonic_msg.max_range = 4.0  
            # Convert cm to meters
            dist_m = distance / 100.0

            # Optionally treat zero as "no echo" (publish max range)
            if self.ultrasonic_zero_means_no_echo and dist_m <= 0.0:
                ultrasonic_msg.range = ultrasonic_msg.max_range
                # Log occasionally to avoid spam
                self.get_logger().debug('Ultrasonic reported 0.0m; mapping to max_range as no-echo')
            else:
                # Clamp to sensor bounds
                if dist_m < ultrasonic_msg.min_range:
                    self.get_logger().debug(f'Ultrasonic distance {dist_m:.3f}m below min; clamping')
                if dist_m > ultrasonic_msg.max_range:
                    self.get_logger().debug(f'Ultrasonic distance {dist_m:.3f}m above max; clamping')
                ultrasonic_msg.range = max(ultrasonic_msg.min_range, min(ultrasonic_msg.max_range, dist_m))
            
            self.ultrasonic_pub.publish(ultrasonic_msg)
            
            # Log successful parse
            self.get_logger().debug(f'Sensor data: ax={ax:.2f} distance={distance:.1f}cm motors=[{motorA:.0f},{motorB:.0f}]')
                
        except ValueError as e:
            self.get_logger().debug(f'Parse error (malformed data): {e}')
        except Exception as e:
            self.get_logger().warn(f'Read error: {e}')
    
    def publish_odom(self):
        """Publish odometry by integrating velocity commands"""
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0 or dt > 0.1:
            dt = 0.02
        self.last_odom_time = now
        
        # Integrate velocity to update pose
        if self.use_cmd_vel_for_odom:
            linear = self.last_linear
            angular = self.last_angular
        else:
            linear = 0.0
            angular = 0.0
        
        if abs(linear) > 0.001 or abs(angular) > 0.001:
            # Update heading
            self.odom_theta += angular * dt
            
            # Update position (body frame to world frame)
            self.odom_x += linear * math.cos(self.odom_theta) * dt
            self.odom_y += linear * math.sin(self.odom_theta) * dt
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        qz = math.sin(self.odom_theta / 2.0)
        qw = math.cos(self.odom_theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Velocity
        odom.twist.twist.linear.x = self.last_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.last_angular
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF odom->base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        try:
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Failed to broadcast TF: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on exit
        if node.ser:
            try:
                node.ser.write(b"STOP\n")
                node.ser.close()
            except Exception as e:
                node.get_logger().warn(f'Error closing serial: {e}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
