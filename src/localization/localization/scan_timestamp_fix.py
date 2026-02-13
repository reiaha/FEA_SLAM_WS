#!/usr/bin/env python3
"""
Scan Timestamp Fix

Republishes LaserScan with header.stamp set to node clock time.
This prevents TF message filter drops due to stale scan timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanTimestampFix(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fix')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Message counters for diagnostics
        self.input_count = 0
        self.output_count = 0
        self.last_input_time = self.get_clock().now()
        self.last_output_time = self.get_clock().now()

        # Match sensor_data QoS to avoid incompatible reliability with the LiDAR driver.
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"🔧 Scan timestamp fix: {self.input_topic} -> {self.output_topic} (QoS: sensor_data)"
        )

    def _scan_cb(self, msg: LaserScan):
        """Callback to republish scan with corrected timestamp"""
        self.input_count += 1
        self.last_input_time = self.get_clock().now()
        
        try:
            fixed = LaserScan()
            # Use current clock time instead of original timestamp
            fixed.header.stamp = self.get_clock().now().to_msg()
            fixed.header.frame_id = msg.header.frame_id

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
            self.output_count += 1
            self.last_output_time = self.get_clock().now()
            
            # Log periodically (every 100 messages)
            if self.output_count % 100 == 0:
                elapsed = (self.get_clock().now() - self.last_input_time).nanoseconds / 1e9
                self.get_logger().debug(
                    f"📊 Scan fix stats: in={self.input_count}, out={self.output_count}, "
                    f"proc_time={elapsed*1000:.1f}ms, ranges={len(msg.ranges)}"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing scan: {e}")

    def _timer_cb(self):
        """Periodic diagnostics"""
        now = self.get_clock().now()
        input_delta = (now - self.last_input_time).nanoseconds / 1e9
        output_delta = (now - self.last_output_time).nanoseconds / 1e9
        
        if input_delta > 1.0:
            if self.input_count == 0:
                self.get_logger().warn("⚠️ No input scans received yet on input_topic")
            else:
                self.get_logger().warn(f"⚠️ No input scans for {input_delta:.1f}s (received {self.input_count})")
        
        if output_delta > 1.0 and self.output_count > 0 and self.input_count > self.output_count:
            self.get_logger().warn(f"⚠️ Output stalled: in={self.input_count}, out={self.output_count}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFix()
    
    # Add diagnostics timer
    timer = node.create_timer(5.0, node._timer_cb)
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
