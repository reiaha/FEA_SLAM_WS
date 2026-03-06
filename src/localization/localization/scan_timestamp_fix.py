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
        self.declare_parameter('debug_timestamps', False)
        self.declare_parameter('debug_log_interval_sec', 1.0)
        # Spatial outlier filter: replace isolated spikes with inf
        self.declare_parameter('range_filter_enabled', True)
        self.declare_parameter('range_filter_window', 3)       # beams each side
        self.declare_parameter('range_filter_max_deviation', 0.20)  # metres
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.expected_scan_size = int(self.get_parameter('expected_scan_size').value)
        self.auto_lock_scan_size = bool(self.get_parameter('auto_lock_scan_size').value)
        self.size_mismatch_log_interval = float(self.get_parameter('size_mismatch_log_interval').value)
        self.timestamp_offset_sec = float(self.get_parameter('timestamp_offset_sec').value)
        self.max_input_age_sec = float(self.get_parameter('max_input_age_sec').value)
        self.input_queue_depth = max(1, int(self.get_parameter('input_queue_depth').value))
        self.rebase_stale_to_now = bool(self.get_parameter('rebase_stale_to_now').value)
        self.debug_timestamps = bool(self.get_parameter('debug_timestamps').value)
        self.debug_log_interval_sec = float(self.get_parameter('debug_log_interval_sec').value)
        self.range_filter_enabled = bool(self.get_parameter('range_filter_enabled').value)
        self.range_filter_window = int(self.get_parameter('range_filter_window').value)
        self.range_filter_max_deviation = float(self.get_parameter('range_filter_max_deviation').value)
        self.scan_size_ref = self.expected_scan_size if self.expected_scan_size > 0 else None
        self.last_size_log_time = 0.0
        self.size_mismatch_count = 0
        self.last_output_stamp_ns = 0
        self.last_stale_drop_log_time = 0.0
        self.stale_drop_count = 0
        self.stale_rebase_count = 0
        self.last_debug_log_time = 0.0

        # Message counters for diagnostics
        self.input_count = 0
        self.output_count = 0
        self.last_input_time = self.get_clock().now()
        self.last_output_time = self.get_clock().now()

        # Subscribe with best-effort depth=1 to avoid callback backlog and stale scan queueing.
        # Publish with reliable QoS so Nav2 costmaps (reliable subscribers) receive scans.
        input_qos = QoSProfile(depth=self.input_queue_depth)
        input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        input_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=1)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(LaserScan, self.output_topic, reliable_qos)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self._scan_cb, input_qos)

        self.get_logger().info(
            f"🔧 Scan timestamp fix: {self.input_topic} -> {self.output_topic} "
            f"(pub: reliable depth=1, sub: best_effort depth={self.input_queue_depth})"
        )
        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.get_logger().info(f"[TimeSync] use_sim_time={use_sim_time}")
        self.get_logger().info(
            f"[TimeSync] debug_timestamps={self.debug_timestamps}, "
            f"debug_log_interval_sec={self.debug_log_interval_sec:.2f}, "
            f"max_input_age_sec={self.max_input_age_sec:.3f}, "
            f"timestamp_offset_sec={self.timestamp_offset_sec:.3f}, "
            f"rebase_stale_to_now={self.rebase_stale_to_now}"
        )

    def _log_timing_debug(self, now_ns: int, msg_stamp_ns: int, stamp_ns: int | None, dropped: bool, reason: str):
        if not self.debug_timestamps:
            return
        now_sec = now_ns / 1e9
        if (now_sec - self.last_debug_log_time) < max(0.1, self.debug_log_interval_sec):
            return
        self.last_debug_log_time = now_sec

        msg_sec = msg_stamp_ns / 1e9 if msg_stamp_ns > 0 else 0.0
        input_age_sec = (now_ns - msg_stamp_ns) / 1e9 if msg_stamp_ns > 0 else 0.0
        line = (
            f"[TimeSync][DEBUG] frame={self.input_topic} src_frame={getattr(self, '_last_frame_id', 'unknown')} "
            f"msg_t={msg_sec:.6f} now_t={now_sec:.6f} age={input_age_sec:.3f}s "
            f"limit={self.max_input_age_sec:.3f}s offset={self.timestamp_offset_sec:.3f}s "
            f"dropped={int(dropped)} reason={reason}"
        )
        if stamp_ns is not None:
            out_sec = stamp_ns / 1e9
            out_skew_sec = out_sec - now_sec
            line += f" out_t={out_sec:.6f} out_minus_now={out_skew_sec:.3f}s"
        self.get_logger().info(line)

    def _scan_cb(self, msg: LaserScan):
        """Callback to republish scan with corrected timestamp and time diagnostics"""
        self.input_count += 1
        self.last_input_time = self.get_clock().now()
        self._last_frame_id = msg.header.frame_id

        # Diagnostics: print incoming and outgoing timestamps
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_time = self.get_clock().now().nanoseconds / 1e9
        if abs(now_time - msg_time) > 0.1:
            self.get_logger().warn(f"[TimeSync] Incoming scan timestamp {msg_time:.3f} is {now_time - msg_time:.3f}s different from node time {now_time:.3f}")

        try:
            fixed = LaserScan()
            # Prefer sensor timestamp + offset to preserve temporal alignment with robot pose.
            # Drop stale delayed scans to avoid map smearing and TF message-filter drops.
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
                    self._log_timing_debug(now_ns, msg_stamp_ns, None, True, 'stale_input')
                    return

            if msg_stamp_ns > 0:
                stamp_ns = msg_stamp_ns + int(max(0.0, self.timestamp_offset_sec) * 1e9)
            else:
                stamp_ns = now_ns + int(max(0.0, self.timestamp_offset_sec) * 1e9)

            # Ensure output stamp is not in the past relative to processing time.
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

            # Spatial outlier filter: kill isolated spikes before publishing
            if self.range_filter_enabled and len(ranges) > 2 * self.range_filter_window:
                w = self.range_filter_window
                n = len(ranges)
                filtered = ranges[:]
                for i in range(n):
                    r = ranges[i]
                    if not (msg.range_min <= r <= msg.range_max):
                        continue
                    # Collect valid neighbours (wrap-around for 360 scans)
                    neighbours = [
                        ranges[(i + d) % n]
                        for d in range(-w, w + 1) if d != 0
                        if msg.range_min <= ranges[(i + d) % n] <= msg.range_max
                    ]
                    if len(neighbours) < w:  # too few valid neighbours — keep as-is
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
            self._log_timing_debug(now_ns, msg_stamp_ns, stamp_ns, False, 'published')
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
