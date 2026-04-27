#!/usr/bin/env python3
"""
Publish a fallback map->odom transform if SLAM is not providing it.
Stops publishing once a valid map->odom transform is detected.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer


class MapOdomFallback(Node):
    def __init__(self):
        super().__init__('map_odom_fallback')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('stop_when_map_tf', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_fresh_timeout', 2.0)
        self.declare_parameter('publish_rate', 10.0)      
        self.declare_parameter('activation_grace_sec', 8.0)
        self.declare_parameter('missing_tf_timeout_sec', 1.5)
        self.declare_parameter('disable_after_first_slam_tf', True)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.stop_when_map_tf = self.get_parameter('stop_when_map_tf').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_fresh_timeout = float(self.get_parameter('map_fresh_timeout').value)
        self.publish_rate = self.get_parameter('publish_rate').value
        self.activation_grace_sec = float(self.get_parameter('activation_grace_sec').value)
        self.missing_tf_timeout_sec = float(self.get_parameter('missing_tf_timeout_sec').value)
        self.disable_after_first_slam_tf = bool(self.get_parameter('disable_after_first_slam_tf').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.fallback_active = False
        self.tf_publish_interval = 1.0 / self.publish_rate
        self.startup_time = self.get_clock().now()
        self.last_slam_tf_time = None
        self.saw_slam_tf_once = False
        
        self.checks = 0
        self.fallback_activations = 0

        self.create_timer(self.tf_publish_interval, self._timer_cb)

    def _timer_cb(self):
        """Check for SLAM transform and publish fallback if needed"""
        self.checks += 1
        now = self.get_clock().now()

        uptime_sec = (now - self.startup_time).nanoseconds / 1e9
        if uptime_sec < self.activation_grace_sec:
            return

        slam_available = False
        try:
            timeout = Duration(seconds=0.05)
            if self.tf_buffer.can_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=timeout,
            ):
                slam_available = True
        except Exception:
            pass

        if slam_available:
            self.last_slam_tf_time = now
            self.saw_slam_tf_once = True
            if self.fallback_active:
                self.get_logger().info("map->odom from SLAM detected; disabling fallback TF")
                self.fallback_active = False
            return

        if self.disable_after_first_slam_tf and self.saw_slam_tf_once:
            return

        if self.last_slam_tf_time is not None:
            missing_for_sec = (now - self.last_slam_tf_time).nanoseconds / 1e9
            if missing_for_sec < self.missing_tf_timeout_sec:
                return

        if self.stop_when_map_tf:
            pass

        if not self.fallback_active:
            self.fallback_active = True
            self.fallback_activations += 1
            self.get_logger().warn(
                f"⚠️ SLAM map->odom not available ({self.checks} checks, {self.fallback_activations} activations); publishing fallback TF"
            )

        msg = TransformStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MapOdomFallback()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
