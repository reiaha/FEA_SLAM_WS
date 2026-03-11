#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BootstrapMover(Node):
    def __init__(self):
        super().__init__('bootstrap_mover')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Bootstrap Mover initialized')
        
    def move_forward(self, duration=3.0, speed=0.1):
        """Move forward for specified duration"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds...')
        
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Movement complete - stopped')
    
    def rotate(self, duration=2.0, angular_speed=0.3):
        """Rotate for specified duration"""
        self.get_logger().info(f'Rotating at {angular_speed} rad/s for {duration} seconds...')
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Rotation complete - stopped')

def main():rhea

    rclpy.init()
    node = BootstrapMover()
    
    time.sleep(2)  # Wait for connections
    
    # Perform bootstrap movements
    node.get_logger().info('=== Starting Bootstrap Movement Sequence ===')
    
    # Move forward to get some initial map data
    node.move_forward(duration=2.0, speed=0.15)
    time.sleep(1)
    
    # Rotate to scan surroundings
    node.rotate(duration=3.0, angular_speed=0.4)
    time.sleep(1)
    
    # Move forward again
    node.move_forward(duration=2.0, speed=0.15)
    time.sleep(1)
    
    node.get_logger().info('=== Bootstrap Complete - Robot should now have initial map ===')
    node.get_logger().info('Check RViz for map data, then autonomous exploration should begin!')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
