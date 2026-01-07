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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Header
import serial
import time
import math

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)  # Distance between wheels (m)
        self.declare_parameter('max_speed', 200)     # Max motor PWM speed
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic', 10)
        
        # Subscriber for motor commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino reset
            self.get_logger().info(f'Arduino connected on {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {serial_port}: {e}')
            self.ser = None
            
        # Timer for reading sensor data
        self.timer = self.create_timer(0.02, self.read_sensors)  # 50Hz
        
        self.get_logger().info('Arduino Motor Bridge initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel (linear.x, angular.z) to differential drive motor speeds
        """
        if not self.ser:
            return
        
        linear = msg.linear.x   # m/s
        angular = msg.angular.z  # rad/s
        
        # Differential drive kinematics
        # v_left = linear - (angular * wheel_base / 2)
        # v_right = linear + (angular * wheel_base / 2)
        
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # Convert to motor speeds (-255 to 255)
        # Assume max speed of 0.5 m/s maps to 200 PWM
        speed_left = int(v_left * (self.max_speed / 0.5))
        speed_right = int(v_right * (self.max_speed / 0.5))
        
        # Constrain to valid range
        speed_left = max(-255, min(255, speed_left))
        speed_right = max(-255, min(255, speed_right))
        
        # Send command to Arduino
        command = f"MOTOR:{speed_left},{speed_right}\n"
        try:
            self.ser.write(command.encode())
            self.get_logger().debug(f'Sent: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def read_sensors(self):
        """Read sensor data from Arduino"""
        if not self.ser or not self.ser.in_waiting:
            return
        
        try:
            line = self.ser.readline().decode('utf-8').strip()
            
            # Skip acknowledgment messages
            if line.startswith('ACK:'):
                self.get_logger().debug(f'Arduino: {line}')
                return
            
            # Parse CSV: ax,ay,az,gx,gy,gz,distance,motorA,motorB
            parts = line.split(',')
            if len(parts) == 9:
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
                ultrasonic_msg.range = distance / 100.0  # Convert cm to m
                
                self.ultrasonic_pub.publish(ultrasonic_msg)
                
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')

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
            node.ser.write(b"STOP\n")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
