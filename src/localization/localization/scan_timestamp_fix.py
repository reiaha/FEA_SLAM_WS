#!/usr/bin/env python3
"""
Scan Timestamp Fix

Republishes LaserScan with header.stamp set to node clock time.
This prevents TF message filter drops due to stale scan timestamps.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanTimestampFix(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fix')

        # Force ROS time usage for synchronization
        self.use_clock = True
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('expected_scan_size', -1)
        self.declare_parameter('auto_lock_scan_size', True)
        self.declare_parameter('size_mismatch_log_interval', 10.0)
        self.declare_parameter('timestamp_offset_sec', 0.03)
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.expected_scan_size = int(self.get_parameter('expected_scan_size').value)
        self.auto_lock_scan_size = bool(self.get_parameter('auto_lock_scan_size').value)
        self.size_mismatch_log_interval = float(self.get_parameter('size_mismatch_log_interval').value)
        self.timestamp_offset_sec = float(self.get_parameter('timestamp_offset_sec').value)
        self.scan_size_ref = self.expected_scan_size if self.expected_scan_size > 0 else None
        self.last_size_log_time = 0.0
        self.size_mismatch_count = 0
        self.last_output_stamp_ns = 0

        # Message counters for diagnostics
        self.input_count = 0
        self.output_count = 0
        self.last_input_time = self.get_clock().now()
        self.last_output_time = self.get_clock().now()

        # Subscribe with sensor_data QoS to match the LiDAR driver (often best-effort).
        # Publish with reliable QoS so Nav2 costmaps (reliable subscribers) receive scans.
        reliable_qos = QoSProfile(depth=1)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(LaserScan, self.output_topic, reliable_qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"🔧 Scan timestamp fix: {self.input_topic} -> {self.output_topic} (pub: reliable, sub: sensor_data)"
        )
        self.get_logger().info("[TimeSync] use_sim_time set to False. Using system (real) time.")

    def _scan_cb(self, msg: LaserScan):
        """Callback to republish scan with corrected timestamp and time diagnostics"""
        self.input_count += 1
        self.last_input_time = self.get_clock().now()

        # Diagnostics: print incoming and outgoing timestamps
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds / 1e9
        if abs(now_time - msg_time) > 0.1:
            self.get_logger().warn(f"[TimeSync] Incoming scan timestamp {msg_time:.3f} is {now_time - msg_time:.3f}s different from node time {now_time:.3f}")

        try:
            fixed = LaserScan()
            # Use current clock time with a small positive offset and enforce monotonicity.
            # This avoids occasional "earlier than transform cache" drops under scheduling jitter.
            now_ns = self.get_clock().now().nanoseconds
            stamp_ns = now_ns + int(max(0.0, self.timestamp_offset_sec) * 1e9)
            if stamp_ns <= self.last_output_stamp_ns:
                stamp_ns = self.last_output_stamp_ns + 1
            self.last_output_stamp_ns = stamp_ns
            fixed.header.stamp.sec = stamp_ns // 1_000_000_000
            fixed.header.stamp.nanosec = stamp_ns % 1_000_000_000
            fixed.header.frame_id = msg.header.frame_id

            fixed.angle_min = msg.angle_min
            fixed.angle_max = msg.angle_max
            fixed.angle_increment = msg.angle_increment
            fixed.time_increment = msg.time_increment
            fixed.scan_time = msg.scan_time
            fixed.range_min = msg.range_min
            fixed.range_max = msg.range_max

            ranges = list(msg.ranges)
            intensities = list(msg.intensities)

            if self.scan_size_ref is None and self.auto_lock_scan_size and len(ranges) > 0:
                self.scan_size_ref = len(ranges)
                self.get_logger().info(f"🔒 Auto-locked scan size to {self.scan_size_ref}")

            if self.scan_size_ref is not None and len(ranges) != self.scan_size_ref:
                self.size_mismatch_count += 1
                if len(ranges) > self.scan_size_ref:
                    ranges = ranges[:self.scan_size_ref]
                    if intensities:
                        intensities = intensities[:self.scan_size_ref]
                else:
                    pad_count = self.scan_size_ref - len(ranges)
                    ranges.extend([float('inf')] * pad_count)
                    if intensities:
                        intensities.extend([0.0] * pad_count)

                now_sec = self.get_clock().now().nanoseconds / 1e9
                if (now_sec - self.last_size_log_time) >= max(1.0, self.size_mismatch_log_interval):
                    self.last_size_log_time = now_sec
                    self.get_logger().info(
                        f"🔧 Normalized scan size {len(msg.ranges)} -> {self.scan_size_ref} "
                        f"(mismatches={self.size_mismatch_count})"
                    )

            # Keep angle metadata consistent with output range length.
            # slam_toolbox derives expected beam count from angle_min/max/increment,
            # so range resizing must update angle_max to match.
            if len(ranges) > 0:
                fixed.angle_max = fixed.angle_min + (len(ranges) - 1) * fixed.angle_increment

            fixed.ranges = ranges
            fixed.intensities = intensities

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
                self.get_logger().info("\u2139\ufe0f No input scans received yet on input_topic")
            else:
                self.get_logger().info(f"\u2139\ufe0f No input scans for {input_delta:.1f}s (received {self.input_count})")
        if output_delta > 1.0 and self.output_count > 0 and self.input_count > self.output_count:
            self.get_logger().info(f"\u2139\ufe0f Output stalled: in={self.input_count}, out={self.output_count}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFix()
    
    # Add diagnostics timer
    timer = node.create_timer(5.0, node._timer_cb)
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
