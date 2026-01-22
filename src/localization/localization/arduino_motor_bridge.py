#!/usr/bin/env python3
"""
Arduino Motor Bridge Node
Handles bidirectional communication with Arduino:
- Receives cmd_vel from ROS2 navigation/teleop
- Sends motor commands to Arduino
- Publishes sensor data (IMU, ultrasonic) to ROS2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import numpy as np

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)  # Distance between wheels (m)
        self.declare_parameter('max_speed', 200)     # Max motor PWM speed
        # Motion shaping
        self.declare_parameter('velocity_deadband', 0.03)  # m/s below which we command 0
        self.declare_parameter('min_pwm', 70)               # minimum |PWM| when moving
        # If true, interpret a distance reading of 0 from the Arduino ultrasonic
        # as "no echo" and publish max_range instead of 0.0 meters.
        self.declare_parameter('ultrasonic_zero_means_no_echo', True)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.ultrasonic_zero_means_no_echo = self.get_parameter('ultrasonic_zero_means_no_echo').value
        self.velocity_deadband = self.get_parameter('velocity_deadband').value
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        
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
        
        # Subscriber for motor commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Create timer to publish odometry periodically
        self.odom_timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        
        # Serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino reset
            # Flush any residual data from startup/reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Arduino connected on {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {serial_port}: {e}')
            self.ser = None
            
        # Timer for reading sensor data
        self.timer = self.create_timer(0.02, self.read_sensors)  # 50Hz
        
        # Publish initial TF immediately so SLAM has it available at startup
        self.publish_odom()
        
        self.get_logger().info('Arduino Motor Bridge initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel (linear.x, angular.z) to differential drive motor speeds
        """
        if not self.ser:
            return
        
        linear = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s
        
        # Track velocity for odometry integration
        self.last_linear = linear
        self.last_angular = angular
        
        # Differential drive kinematics
        # v_left = linear - (angular * wheel_base / 2)
        # v_right = linear + (angular * wheel_base / 2)
        
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
        command = f"MOTOR:{speed_left},{speed_right}\n"
        try:
            self.ser.write(command.encode())
            self.get_logger().info(f'🚀 Sent motor command: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_sensors(self):
        """Read sensor data from Arduino"""
        if not self.ser:
            return
        
        try:
            # Check if data is available with error handling
            if not self.ser.in_waiting:
                return
        except (OSError, serial.SerialException) as e:
            # Serial port error - likely disconnected or I/O issue
            self.get_logger().warn(f'Serial port error during check: {e}')
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
            ultrasonic_msg.min_range = 0.02  # 2 cm
            ultrasonic_msg.max_range = 4.0   # 4 meters
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
            
            # Log successful parse (debug level to avoid spam)
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
        linear = self.last_linear
        angular = self.last_angular
        
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
        odom.child_frame_id = 'base_link'
        
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
        
        # Broadcast TF odom->base_link
        # Use current time for TF to ensure it's available for SLAM lookups
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

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
