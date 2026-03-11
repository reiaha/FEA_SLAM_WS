#!/usr/bin/env python3
"""
Publish a fallback map->odom transform if SLAM is not providing it.
Stops publishing once a valid map->odom transform is detected.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster, TransformListener, Buffer


class MapOdomFallback(Node):
    def __init__(self):
        super().__init__('map_odom_fallback')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('stop_when_map_tf', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_fresh_timeout', 2.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.stop_when_map_tf = self.get_parameter('stop_when_map_tf').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_fresh_timeout = float(self.get_parameter('map_fresh_timeout').value)
        self.publish_rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.fallback_active = False
        self.last_check_time = self.get_clock().now()
        self.check_interval = 1.0  # Check every second
        self.tf_publish_interval = 1.0 / self.publish_rate
        self.last_publish_time = self.get_clock().now()
        self.last_map_time = None
        
        # Statistics
        self.checks = 0
        self.fallback_activations = 0

        # Map subscription to confirm SLAM is publishing
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)

        # Timer for checking and publishing
        self.create_timer(self.tf_publish_interval, self._timer_cb)

    def _map_cb(self, msg: OccupancyGrid):
        self.last_map_time = self.get_clock().now()

    def _timer_cb(self):
        """Check for SLAM transform and publish fallback if needed"""
        self.checks += 1
        now = self.get_clock().now()
        
        # Check if SLAM is providing a fresh map and map->odom transform
        slam_available = False
        map_fresh = False
        if self.last_map_time is not None:
            map_age = (now - self.last_map_time).nanoseconds / 1e9
            map_fresh = map_age <= self.map_fresh_timeout
        try:
            timeout = Duration(seconds=0.05)
            if self.tf_buffer.can_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=timeout,
            ):
                slam_available = True
        except Exception as e:
            self.get_logger().debug(f"TF check: {e}")

        if slam_available and map_fresh and self.stop_when_map_tf:
            if self.fallback_active:
                self.get_logger().info("✅ map->odom from SLAM detected; disabling fallback TF")
                self.fallback_active = False
            return

        # SLAM not providing transform, check if we should publish fallback
        if self.stop_when_map_tf:
            # Already handled above - SLAM not available, so publish fallback
            pass

        # Publish fallback transform
        if not self.fallback_active:
            self.fallback_active = True
            self.fallback_activations += 1
            self.get_logger().warn(
                f"⚠️ SLAM map->odom not available ({self.checks} checks, {self.fallback_activations} activations); publishing fallback TF"
            )

        # Publish the fallback transform with current time
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

        # Log periodically (every 100 publish cycles)
        if self.checks % 100 == 0:
            self.get_logger().debug(
                f"📊 Fallback stats: checks={self.checks}, active={self.fallback_active}, "
                f"activations={self.fallback_activations}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomFallback()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
