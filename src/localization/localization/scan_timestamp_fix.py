#!/usr/bin/env python3
"""
Scan Timestamp Fix

Republishes LaserScan with corrected timestamp.
Uses sensor stamp + configurable offset when valid, and drops stale delayed scans
to prevent TF message filter drops and map smearing from mis-timed scans.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ScanTimestampFix(Node):
    def __init__(self):
        super().__init__('scan_timestamp_fix')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('expected_scan_size', -1)
        self.declare_parameter('auto_lock_scan_size', True)
        self.declare_parameter('size_mismatch_log_interval', 10.0)
        self.declare_parameter('timestamp_offset_sec', 0.02)
        self.declare_parameter('input_queue_depth', 1)
        self.declare_parameter('max_input_age_sec', 0.70)
        self.declare_parameter('rebase_stale_to_now', False)
        self.declare_parameter('force_stamp_now', False)
        self.declare_parameter('range_filter_enabled', True)
        self.declare_parameter('range_filter_window', 3)                        
        self.declare_parameter('range_filter_max_deviation', 0.20)          
        self.declare_parameter('tilt_gating_enabled', True)
        self.declare_parameter('tilt_gate_threshold_deg', 15.0)                               
        self.declare_parameter('tilt_throttle_threshold_deg', 10.0)                              
        self.declare_parameter('tilt_throttle_ratio', 3)                                                     
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.expected_scan_size = int(self.get_parameter('expected_scan_size').value)
        self.auto_lock_scan_size = bool(self.get_parameter('auto_lock_scan_size').value)
        self.size_mismatch_log_interval = float(self.get_parameter('size_mismatch_log_interval').value)
        self.timestamp_offset_sec = float(self.get_parameter('timestamp_offset_sec').value)
        self.max_input_age_sec = float(self.get_parameter('max_input_age_sec').value)
        self.input_queue_depth = max(1, int(self.get_parameter('input_queue_depth').value))
        self.rebase_stale_to_now = bool(self.get_parameter('rebase_stale_to_now').value)
        self.force_stamp_now = bool(self.get_parameter('force_stamp_now').value)
        self.range_filter_enabled = bool(self.get_parameter('range_filter_enabled').value)
        self.range_filter_window = int(self.get_parameter('range_filter_window').value)
        self.range_filter_max_deviation = float(self.get_parameter('range_filter_max_deviation').value)
        self.tilt_gating_enabled = bool(self.get_parameter('tilt_gating_enabled').value)
        self.tilt_gate_threshold_deg = float(self.get_parameter('tilt_gate_threshold_deg').value)
        self.tilt_throttle_threshold_deg = float(self.get_parameter('tilt_throttle_threshold_deg').value)
        self.tilt_throttle_ratio = int(self.get_parameter('tilt_throttle_ratio').value)
        self.tilt_roll_deg = 0.0
        self.tilt_pitch_deg = 0.0
        self.scan_throttle_counter = 0
        self.scan_size_ref = self.expected_scan_size if self.expected_scan_size > 0 else None
        self.last_size_log_time = 0.0
        self.size_mismatch_count = 0
        self.last_output_stamp_ns = 0
        self.last_stale_drop_log_time = 0.0
        self.stale_drop_count = 0
        self.stale_rebase_count = 0
        self._last_timesync_warn_time = 0.0                                     

        self.input_count = 0
        self.output_count = 0
        self.last_input_time = self.get_clock().now()
        self.last_output_time = self.get_clock().now()

        input_qos = QoSProfile(depth=self.input_queue_depth)
        input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        input_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=1)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(LaserScan, self.output_topic, reliable_qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self._scan_cb, input_qos)
        
        self.tilt_roll_sub = self.create_subscription(Float32, '/tilt_roll_deg', self._tilt_roll_cb, 10)
        self.tilt_pitch_sub = self.create_subscription(Float32, '/tilt_pitch_deg', self._tilt_pitch_cb, 10)

        self.get_logger().info(
            f"🔧 Scan timestamp fix: {self.input_topic} -> {self.output_topic} "
            f"(pub: reliable depth=1, sub: best_effort depth={self.input_queue_depth})"
        )
        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.get_logger().info(f"[TimeSync] use_sim_time={use_sim_time}")
        self.get_logger().info(
            f"max_input_age_sec={self.max_input_age_sec:.3f}, "
            f"timestamp_offset_sec={self.timestamp_offset_sec:.3f}, "
            f"rebase_stale_to_now={self.rebase_stale_to_now}, "
            f"force_stamp_now={self.force_stamp_now}"
        )
        if self.tilt_gating_enabled:
            self.get_logger().info(
                f"[TiltGating] Enabled: gate_threshold={self.tilt_gate_threshold_deg}°, "
                f"throttle_threshold={self.tilt_throttle_threshold_deg}°, throttle_ratio={self.tilt_throttle_ratio}"
            )

    def _tilt_roll_cb(self, msg):
        """Update latest roll angle"""
        self.tilt_roll_deg = msg.data

    def _tilt_pitch_cb(self, msg):
        """Update latest pitch angle"""
        self.tilt_pitch_deg = msg.data

    def _scan_cb(self, msg: LaserScan):
        """Callback to republish scan with corrected timestamp and time diagnostics"""
        self.input_count += 1
        self.last_input_time = self.get_clock().now()
        self._last_frame_id = msg.header.frame_id

        if self.tilt_gating_enabled:
            max_tilt = max(abs(self.tilt_roll_deg), abs(self.tilt_pitch_deg))
            
            if max_tilt > self.tilt_gate_threshold_deg:
                return
            
            if max_tilt > self.tilt_throttle_threshold_deg:
                self.scan_throttle_counter += 1
                if (self.scan_throttle_counter % self.tilt_throttle_ratio) != 0:
                    return

        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds / 1e9
        if abs(now_time - msg_time) > 0.25:
            if (now_time - self._last_timesync_warn_time) >= 5.0:
                self._last_timesync_warn_time = now_time
                self.get_logger().warn(f"[TimeSync] Incoming scan timestamp {msg_time:.3f} is {now_time - msg_time:.3f}s different from node time {now_time:.3f}")

        try:
            fixed = LaserScan()
            now_ns = self.get_clock().now().nanoseconds
            msg_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            input_age_sec = (now_ns - msg_stamp_ns) / 1e9 if msg_stamp_ns > 0 else 0.0

            if msg_stamp_ns > 0 and input_age_sec > self.max_input_age_sec:
                now_sec = now_ns / 1e9
                if self.rebase_stale_to_now:
                    self.stale_rebase_count += 1
                    if (now_sec - self.last_stale_drop_log_time) >= 1.0:
                        self.last_stale_drop_log_time = now_sec
                        self.get_logger().warn(
                            f"[TimeSync] Rebasing stale scan age={input_age_sec:.3f}s "
                            f"(limit={self.max_input_age_sec:.3f}s, rebased={self.stale_rebase_count})"
                        )
                    msg_stamp_ns = 0
                else:
                    self.stale_drop_count += 1
                    if (now_sec - self.last_stale_drop_log_time) >= 1.0:
                        self.last_stale_drop_log_time = now_sec
                        self.get_logger().warn(
                            f"[TimeSync] Dropping stale scan age={input_age_sec:.3f}s "
                            f"(limit={self.max_input_age_sec:.3f}s, drops={self.stale_drop_count})"
                        )
                    return

            if self.force_stamp_now:
                # When forcing stamp to now, do not apply additional offset.
                # Adding offset here can future-date scans and destabilize TF lookups.
                stamp_ns = now_ns
            elif msg_stamp_ns > 0:
                stamp_ns = msg_stamp_ns + int(max(0.0, self.timestamp_offset_sec) * 1e9)
            else:
                stamp_ns = now_ns + int(max(0.0, self.timestamp_offset_sec) * 1e9)

            min_stamp_ns = now_ns - int(0.01 * 1e9)
            if stamp_ns < min_stamp_ns:
                stamp_ns = min_stamp_ns

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

            if self.range_filter_enabled and len(ranges) > 2 * self.range_filter_window:
                w = self.range_filter_window
                n = len(ranges)
                filtered = ranges[:]
                for i in range(n):
                    r = ranges[i]
                    if not (msg.range_min <= r <= msg.range_max):
                        continue
                    neighbours = [
                        ranges[(i + d) % n]
                        for d in range(-w, w + 1) if d != 0
                        if msg.range_min <= ranges[(i + d) % n] <= msg.range_max
                    ]
                    if len(neighbours) < w:                                         
                        continue
                    neighbours.sort()
                    median = neighbours[len(neighbours) // 2]
                    if abs(r - median) > self.range_filter_max_deviation:
                        filtered[i] = float('inf')
                ranges = filtered

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

            if len(ranges) > 0:
                fixed.angle_max = fixed.angle_min + (len(ranges) - 1) * fixed.angle_increment

            fixed.ranges = ranges
            fixed.intensities = intensities

            self.pub.publish(fixed)
            self.output_count += 1
            self.last_output_time = self.get_clock().now()
                
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
    
    node.create_timer(5.0, node._timer_cb)
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
