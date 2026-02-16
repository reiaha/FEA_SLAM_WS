#!/usr/bin/env python3
"""
LiDAR-Based Explorer Node
Uses 360-degree LiDAR scans for obstacle detection and 180-degree aware navigation.
Provides autonomous exploration to bootstrap SLAM mapping.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class LidarExplorer(Node):
    def __init__(self):
        super().__init__('lidar_explorer')
        
        # Parameters
        self.declare_parameter('obstacle_distance', 0.25)  # Reduced from 0.35m
        self.declare_parameter('safe_distance', 0.35)     # Reduced from 0.45m
        self.declare_parameter('forward_speed', 0.15)     
        self.declare_parameter('turn_speed', 0.4)         
        self.declare_parameter('exploration_timeout', 300.0)  
        self.declare_parameter('scan_topic', '/scan')
        
        self.obstacle_dist = self.get_parameter('obstacle_distance').value
        self.safe_dist = self.get_parameter('safe_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.timeout = self.get_parameter('exploration_timeout').value
        self.scan_topic = self.get_parameter('scan_topic').value
        
        # Subscribers
        # Match LiDAR publisher QoS (Best Effort, sensor profile)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publish obstacle warnings for autonomous exploration
        from std_msgs.msg import Bool, Float32
        self.obstacle_warning_pub = self.create_publisher(Bool, '/obstacle_warning', 10)
        self.front_distance_pub = self.create_publisher(Float32, '/front_obstacle_distance', 10)
        
        # Nav2 action client (to check if Nav2 is ready)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # State
        self.front_distance = float('inf')      # distance straight ahead (0-90°)
        self.left_distance = float('inf')       # distance to left (90-180°)
        self.right_distance = float('inf')      # distance to right (-90 to 0°)
        self.map_received = False
        self.exploring = False  # ⚠️ Start as FALSE - don't move until Nav2 confirmed ready
        self.nav2_ready = False  # Track if Nav2 is ready to take over
        self.nav2_init_delay = 30.0  # Match exploration_coordinator - wait 30s for Nav2
        self.init_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.turn_direction = 0  # 0=forward, 1=turn left, -1=turn right
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # Nav2 readiness check timer (slower, every 2 seconds)
        self.nav2_check_timer = self.create_timer(2.0, self.check_nav2_ready)
        
        self.get_logger().info('🤖 LiDAR Explorer started - 180° obstacle detection active')
        self.get_logger().info(f'   Scan topic: {self.scan_topic}')
        self.get_logger().info(f'   Obstacle threshold: {self.obstacle_dist}m, Safe distance: {self.safe_dist}m')
    
    def scan_callback(self, msg: LaserScan):
        """Analyze 180-degree LiDAR scan for obstacles"""
        if not msg.ranges:
            return
        
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        # LiDAR 0° is forward, positive angles go counter-clockwise
        # -90° (270°) is right, +90° is left
        
        # Front sector: -30° to +30° (forward)
        front_start = int((num_ranges * (270 + 30)) / 360)  # -30° in array
        front_end = int((num_ranges * 30) / 360)             # +30° in array
        front_ranges = self.get_safe_ranges(ranges, front_start, front_end)
        self.front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # Left sector: +30° to +150° (left side)
        left_start = int((num_ranges * 30) / 360)
        left_end = int((num_ranges * 150) / 360)
        left_ranges = self.get_safe_ranges(ranges, left_start, left_end)
        self.left_distance = min(left_ranges) if left_ranges else float('inf')
        
        # Right sector: -150° to -30° (right side)
        right_start = int((num_ranges * (360 - 150)) / 360)  # -150°
        right_end = int((num_ranges * (360 - 30)) / 360)     # -30°
        right_ranges = self.get_safe_ranges(ranges, right_start, right_end)
        self.right_distance = min(right_ranges) if right_ranges else float('inf')
    
    def get_safe_ranges(self, ranges, start, end):
        """Get valid range values (filter out inf and 0)"""
        safe = []
        # Handle wraparound
        if start <= end:
            for i in range(start, end):
                if i < len(ranges) and 0 < ranges[i] < float('inf'):
                    safe.append(ranges[i])
        else:
            for i in range(start, len(ranges)):
                if 0 < ranges[i] < float('inf'):
                    safe.append(ranges[i])
            for i in range(0, end):
                if 0 < ranges[i] < float('inf'):
                    safe.append(ranges[i])
        return safe
    
    def map_callback(self, msg: OccupancyGrid):
        """Check if map has been received (SLAM is working)"""
        if not self.map_received:
            # Check if map has actual data (not just empty)
            data_array = list(msg.data)
            known_cells = sum(1 for cell in data_array if cell >= 0 and cell <= 100)
            
            if known_cells > 100:  # At least 100 known cells means SLAM is working
                self.map_received = True
                self.get_logger().info('✅ Map received! SLAM is working. Continuing exploration...')
    
    def control_loop(self):
        """Main control loop for LiDAR-based 180° exploration"""
        # Wait for Nav2 to be fully operational (it starts FIRST now)
        if not self.nav2_ready:
            elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9
            if elapsed < self.nav2_init_delay:
                remaining = self.nav2_init_delay - elapsed
                if int(elapsed) % 3 == 0 and remaining > 1:
                    self.get_logger().info(f'⏳ LiDAR Explorer waiting for Nav2... ({remaining:.0f}s remaining)')
                return
            else:
                self.get_logger().info('✅ Nav2 is operational. Starting LiDAR exploration...')
                if self.nav_client.wait_for_server(timeout_sec=2.0):
                    self.nav2_ready = True
                    self.start_time = self.get_clock().now()
                else:
                    self.get_logger().warn('⚠️  Nav2 server not available yet, retrying...')
                    return
        
        # Publish obstacle detection data even when Nav2 is in control
        from std_msgs.msg import Bool, Float32
        obstacle_msg = Bool()
        obstacle_msg.data = self.front_distance < self.obstacle_dist
        self.obstacle_warning_pub.publish(obstacle_msg)
        
        distance_msg = Float32()
        distance_msg.data = float(self.front_distance)
        self.front_distance_pub.publish(distance_msg)
        
        # If Nav2 took over, don't publish cmd_vel
        if not self.exploring:
            return
        
        # Check timeout (only stop when time limit reached, keep moving entire time)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.get_logger().info('⏱️ Exploration timeout reached. Stopping LiDAR explorer.')
            self.stop_robot()
            self.exploring = False
            return
        
        # Keep exploring - don't stop even if map is built
        # Nav2 exploration_coordinator will take over cmd_vel when ready
        
        # 180-degree obstacle avoidance logic
        cmd = Twist()
        
        # Decision logic based on three sectors
        if self.front_distance < self.obstacle_dist:
            # Front blocked - decide which way to turn
            if self.left_distance > self.right_distance:
                self.get_logger().info(f'🚧 Front blocked ({self.front_distance:.2f}m) - turning LEFT')
                cmd.angular.z = self.turn_speed
            else:
                self.get_logger().info(f'🚧 Front blocked ({self.front_distance:.2f}m) - turning RIGHT')
                cmd.angular.z = -self.turn_speed
        elif self.front_distance < self.safe_dist:
            # Approaching obstacle - slow down
            speed_factor = (self.front_distance - self.obstacle_dist) / (self.safe_dist - self.obstacle_dist)
            cmd.linear.x = self.forward_speed * speed_factor
            self.get_logger().debug(f'⚠️ Approaching: front={self.front_distance:.2f}m, left={self.left_distance:.2f}m, right={self.right_distance:.2f}m')
        else:
            # Path clear - move forward, steer around side obstacles if needed
            cmd.linear.x = self.forward_speed
            
            # Gentle steering to avoid side obstacles
            if self.left_distance < self.safe_dist and self.right_distance > self.safe_dist:
                cmd.angular.z = -0.2  # Gentle turn right
                self.get_logger().debug(f'🔄 Steering right: left={self.left_distance:.2f}m')
            elif self.right_distance < self.safe_dist and self.left_distance > self.safe_dist:
                cmd.angular.z = 0.2   # Gentle turn left
                self.get_logger().debug(f'🔄 Steering left: right={self.right_distance:.2f}m')
            else:
                self.get_logger().debug(f'✅ Clear path: front={self.front_distance:.2f}m, left={self.left_distance:.2f}m, right={self.right_distance:.2f}m')
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('🛑 Robot stopped')
    
    def check_nav2_ready(self):
        """Check if Nav2 navigate_to_pose action server is ready"""
        if self.nav2_ready:
            return  # Already marked ready
        
        if self.nav_client.server_is_ready():
            self.nav2_ready = True
            self.exploring = False  # Stop publishing cmd_vel
            self.stop_robot()
            self.get_logger().info('✅ Nav2 /navigate_to_pose is READY! Handing off cmd_vel control to frontier exploration.')
            self.get_logger().info('   lidar_explorer will continue monitoring obstacles in background.')
            # Note: Keep running for continuous obstacle detection, just stop publishing cmd_vel
        else:
            self.get_logger().debug('⏳ Waiting for Nav2 /navigate_to_pose action server...')

def main(args=None):
    rclpy.init(args=args)
    node = LidarExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
