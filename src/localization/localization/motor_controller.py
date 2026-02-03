#!/usr/bin/env python3
"""
Motor Controller - Serial bridge between ROS and Arduino
Sends motor commands to Arduino via serial port
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import serial
import time
import threading


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Serial connection
        self.ser = None
        self.motors_enabled = False
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.get_logger().info(f'✅ Connected to Arduino on {self.serial_port}')
            
            # Send START command
            self.send_command('START')
            self.motors_enabled = True
            self.get_logger().info('✅ Motors enabled')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to connect to Arduino: {e}')
            self.ser = None
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        
        # Timer to monitor Arduino
        self.create_timer(1.0, self.check_motor_status)
    
    def send_command(self, cmd):
        """Send command to Arduino"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open')
            return False
        
        try:
            self.ser.write((cmd + '\n').encode())
            self.get_logger().debug(f'📡 Sent: {cmd}')
            
            # Read response
            response = self.ser.readline().decode().strip()
            if response:
                self.get_logger().debug(f'📥 Response: {response}')
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Serial error: {e}')
            return False
    
    def check_motor_status(self):
        """Periodically check motor status"""
        if self.ser and self.ser.is_open:
            try:
                # Read any buffered data from Arduino
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode().strip()
                    if line:
                        self.get_logger().debug(f'Arduino: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
    
    def forward(self, speed=210):
        """Move forward"""
        return self.send_command('FWD:')
    
    def backward(self, speed=180):
        """Move backward"""
        return self.send_command('BWD:')
    
    def turn_left(self, speed=180):
        """Turn left"""
        return self.send_command('LEFT:')
    
    def turn_right(self, speed=180):
        """Turn right"""
        return self.send_command('RIGHT:')
    
    def stop(self):
        """Stop motors"""
        return self.send_command('STOP')
    
    def __del__(self):
        """Cleanup on exit"""
        if self.ser:
            self.stop()
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Shutting down motor controller')
        motor_controller.stop()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
