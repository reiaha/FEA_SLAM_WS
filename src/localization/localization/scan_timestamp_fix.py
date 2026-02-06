#!/usr/bin/env python3
"""
Scan Timestamp Fix

Republishes LaserScan with header.stamp set to node clock time.
This prevents TF message filter drops due to stale scan timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanTimestampFix(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fix')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self._scan_cb, qos)

        self.get_logger().info(
            f"🔧 Scan timestamp fix: {self.input_topic} -> {self.output_topic}"
        )

    def _scan_cb(self, msg: LaserScan):
        fixed = LaserScan()
        fixed.header = msg.header
        fixed.header.stamp = self.get_clock().now().to_msg()

        fixed.angle_min = msg.angle_min
        fixed.angle_max = msg.angle_max
        fixed.angle_increment = msg.angle_increment
        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max
        fixed.ranges = list(msg.ranges)
        fixed.intensities = list(msg.intensities)

        self.pub.publish(fixed)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFix()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
