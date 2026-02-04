#!/usr/bin/env python3
"""
Arduino Motor Bridge - SIMPLIFIED VERSION
Core job: Convert ROS2 cmd_vel commands to Arduino PWM motor commands
~150 lines, no extra features
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial
import time

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('max_speed', 220)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        
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
        
        if self.ser is None:
            self.get_logger().error('❌ Failed to connect to Arduino')
            return
        
        # Enable motors
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
        
        # Subscribers (accept both /cmd_vel and /cmd_vel_nav)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_cb, 10)
        
        # Servo control
        self.create_subscription(Int32, '/servo_angle', self.servo_cb, 10)
        
        self.get_logger().info('🚀 Arduino Motor Bridge initialized')
    
    def cmd_vel_cb(self, msg: Twist):
        """Convert cmd_vel (linear, angular) to motor PWM commands"""
        if self.ser is None:
            return
        
        linear = msg.linear.x      # m/s
        angular = msg.angular.z    # rad/s
        
        # Differential drive kinematics
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # Convert velocity (m/s) to PWM (-220 to +220)
        # Assuming 0.5 m/s = max_speed
        pwm_left = int(v_left * (self.max_speed / 0.5))
        pwm_right = int(v_right * (self.max_speed / 0.5))
        
        # Enforce minimum PWM when moving (avoid stall)
        MIN_PWM = 120
        if pwm_left != 0:
            sign = 1 if pwm_left > 0 else -1
            pwm_left = sign * max(MIN_PWM, abs(pwm_left))
        if pwm_right != 0:
            sign = 1 if pwm_right > 0 else -1
            pwm_right = sign * max(MIN_PWM, abs(pwm_right))
        
        # Clamp to valid PWM range
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))
        
        # Send to Arduino
        try:
            cmd = f"MOTOR:{pwm_left},{pwm_right}\n"
            self.ser.write(cmd.encode())
            
            # Log action
            action = self._get_action_name(pwm_left, pwm_right)
            self.get_logger().info(f'🚀 {action} | MOTOR:{pwm_left},{pwm_right}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
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

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
