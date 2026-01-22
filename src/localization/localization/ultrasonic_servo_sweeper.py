#!/usr/bin/env python3
"""
Ultrasonic Servo Sweeper Node
Sweeps the servo-mounted ultrasonic sensor to detect obstacles at different angles.
Publishes Range messages for each angle measurement.
Works alongside LiDAR for redundant obstacle detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import serial
import time
import math

class UltrasonicServoSweeper(Node):
    def __init__(self):
        super().__init__('ultrasonic_servo_sweeper')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sweep_angles', [0, 20, 40, 60, 80, 100, 120, 140, 160, 140, 120, 100, 80, 60, 40, 20])  # Servo 0-160° sweep
        self.declare_parameter('sweep_delay', 0.8)  # Seconds between angle changes (slower)
        self.declare_parameter('max_range', 0.30)  # Max ultrasonic range in meters (30cm threshold)
        self.declare_parameter('fov', 15.0)  # Field of view in degrees
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.sweep_angles = self.get_parameter('sweep_angles').value
        self.sweep_delay = self.get_parameter('sweep_delay').value
        self.max_range = self.get_parameter('max_range').value
        self.fov = self.get_parameter('fov').value
        
        # Publisher for ultrasonic readings (one per angle)
        self.range_pub = self.create_publisher(Range, 'ultrasonic_sweep', 10)
        
        # Serial connection
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Allow Arduino reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Arduino connected on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.serial_port}: {e}')
            self.ser = None
        
        # Sweep state
        self.current_angle_idx = 0
        self.frame_id = 'ultrasonic_servo'
        
        # Timer for sweep control
        self.create_timer(self.sweep_delay, self.perform_sweep_step)
        
        self.get_logger().info(f'Ultrasonic Servo Sweeper initialized. Angles: {self.sweep_angles}')
    
    def perform_sweep_step(self):
        """
        Move servo to next angle and read ultrasonic distance.
        """
        if not self.ser:
            return
        
        # Get next angle
        angle = self.sweep_angles[self.current_angle_idx]
        
        # Command servo to angle
        try:
            cmd = f"SERVO:{angle}\n"
            self.ser.write(cmd.encode())
            # Wait briefly for servo to move
            time.sleep(0.1)
            
            # Read ultrasonic distance
            distance_cm = self.read_ultrasonic_from_arduino()
            
            if distance_cm is not None:
                # Convert cm to meters
                distance_m = distance_cm / 100.0
                # Clamp to max range
                distance_m = min(distance_m, self.max_range)
                
                # Publish Range message
                range_msg = Range()
                range_msg.header = Header()
                range_msg.header.stamp = self.get_clock().now()
                range_msg.header.frame_id = f'{self.frame_id}_angle_{angle}'
                
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = math.radians(self.fov)
                range_msg.min_range = 0.02  # 2 cm minimum
                range_msg.max_range = self.max_range
                range_msg.range = distance_m
                
                self.range_pub.publish(range_msg)
                
                self.get_logger().debug(
                    f'Servo {angle}°: {distance_cm:.1f} cm ({distance_m:.2f} m)'
                )
        except Exception as e:
            self.get_logger().error(f'Error during sweep: {e}')
        
        # Move to next angle
        self.current_angle_idx = (self.current_angle_idx + 1) % len(self.sweep_angles)
    
    def read_ultrasonic_from_arduino(self):
        """
        Extract ultrasonic distance from the continuous Arduino CSV output.
        CSV format: ax,ay,az,gx,gy,gz,distance,motorA,motorB
        Returns distance in cm, or None if read fails.
        """
        try:
            # Read one complete line
            line = self.ser.readline().decode('utf-8').strip()
            
            if not line:
                return None
            
            # Parse CSV
            parts = line.split(',')
            if len(parts) >= 7:
                try:
                    distance_cm = float(parts[6])
                    # Validate: ultrasonic should be 0-400cm
                    if 0 <= distance_cm <= 400:
                        return distance_cm
                except ValueError:
                    pass
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicServoSweeper()
    rclpy.spin(node)
    
    if node.ser:
        node.ser.close()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
