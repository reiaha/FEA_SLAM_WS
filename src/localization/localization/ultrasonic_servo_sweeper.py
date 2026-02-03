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
from std_msgs.msg import Header, Int32
import math

class UltrasonicServoSweeper(Node):
    def __init__(self):
        super().__init__('ultrasonic_servo_sweeper')
        
        # Parameters
        self.declare_parameter('sweep_angles', [110, 150, 170, 150])  # Servo sweep (left/center/right)
        self.declare_parameter('sweep_delay', 0.8)  # Seconds between angle changes (slower)
        self.declare_parameter('max_range', 0.30)  # Max ultrasonic range in meters (30cm threshold)
        self.declare_parameter('fov', 15.0)  # Field of view in degrees

        self.sweep_angles = self.get_parameter('sweep_angles').value
        self.sweep_delay = self.get_parameter('sweep_delay').value
        self.max_range = self.get_parameter('max_range').value
        self.fov = self.get_parameter('fov').value
        
        # Publisher for ultrasonic readings (one per angle)
        self.range_pub = self.create_publisher(Range, 'ultrasonic_sweep', 10)

        # Publisher for servo angle commands
        self.servo_pub = self.create_publisher(Int32, 'servo_angle', 10)

        # Subscriber for ultrasonic readings from Arduino
        self.ultrasonic_sub = self.create_subscription(
            Range, 'ultrasonic', self.ultrasonic_callback, 10)
        
        # Sweep state
        self.current_angle_idx = 0
        self.frame_id = 'ultrasonic_servo'
        self.pending_angle = None
        self.last_range = None
        
        # Timer for sweep control
        self.create_timer(self.sweep_delay, self.perform_sweep_step)
        
        self.get_logger().info(f'Ultrasonic Servo Sweeper initialized. Angles: {self.sweep_angles}')
    
    def perform_sweep_step(self):
        """
        Move servo to next angle and read ultrasonic distance.
        """
        # Get next angle
        angle = self.sweep_angles[self.current_angle_idx]

        # Command servo to angle via Arduino bridge
        angle_msg = Int32()
        angle_msg.data = int(angle)
        self.servo_pub.publish(angle_msg)
        self.pending_angle = int(angle)
        
        # Move to next angle
        self.current_angle_idx = (self.current_angle_idx + 1) % len(self.sweep_angles)

    def ultrasonic_callback(self, msg: Range):
        """Receive ultrasonic readings and publish with current servo angle."""
        self.last_range = msg.range

        if self.pending_angle is None:
            return

        # Publish Range message tagged with the servo angle
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = f'{self.frame_id}_angle_{self.pending_angle}'

        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = math.radians(self.fov)
        range_msg.min_range = 0.02  # 2 cm minimum
        range_msg.max_range = self.max_range
        range_msg.range = min(self.last_range, self.max_range)

        self.range_pub.publish(range_msg)
        self.pending_angle = None


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicServoSweeper()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
