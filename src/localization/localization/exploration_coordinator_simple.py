#!/usr/bin/env python3
"""
FEA-SLAM Exploration Coordinator - SIMPLIFIED VERSION
Core logic only: 5 phases, ~400 lines

What it does:
1. Wait for frontiers from frontier_detector
2. Pick best frontier and send to Nav2
3. Monitor obstacles - if too close, backup and rescan
4. Repeat until exploration complete
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
from action_msgs.msg import GoalStatus
from tf2_ros import TransformListener, Buffer
from enum import Enum
import math
import time

class Phase(Enum):
    INIT = 1
    EXPLORE = 2           # Pick frontier, plan, move
    OBSTACLE = 3          # Handle collision
    RESCAN = 4            # Check if path clear
    DONE = 5              # Exploration complete

class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator_v2')
        
        # Parameters
        self.declare_parameter('nav2_timeout', 30.0)
        self.declare_parameter('obstacle_distance', 0.35)      # meters
        self.declare_parameter('backup_speed', -0.5)           # m/s
        self.declare_parameter('backup_time', 2.0)             # seconds
        
        self.nav2_timeout = self.get_parameter('nav2_timeout').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_time = self.get_parameter('backup_time').value
        self.backup_iterations = int(self.backup_time / 0.05)  # 40 at 20Hz
        
        # State
        self.current_phase = Phase.INIT
        self.phase_start_time = time.time()
        self.robot_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.current_frontiers = []
        self.obstacle_detected = False
        self.obstacle_distance_m = float('inf')
        self.nav2_ready = False
        self.goal_handle = None
        self.no_frontier_cycles = 0
        
        # TF listener for pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        
        # Subscribers
        self.create_subscription(MarkerArray, '/frontiers', self.frontiers_cb, 10)
        self.create_subscription(Float32, '/obstacle_distance', self.obstacle_distance_cb, 10)
        
        # Timer for main loop
        self.create_timer(0.5, self.main_loop)  # 2 Hz
        
        self.get_logger().info("🚀 Exploration Coordinator Started (Simplified)")
    
    def frontiers_cb(self, msg: MarkerArray):
        """Store frontier positions"""
        self.current_frontiers = []
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.current_frontiers.append((x, y))
    
    def obstacle_distance_cb(self, msg: Float32):
        """Store obstacle distance from ultrasonic"""
        self.obstacle_distance_m = msg.data
        if self.obstacle_distance_m < self.obstacle_distance:
            self.obstacle_detected = True
            self.get_logger().error(f"🚨 OBSTACLE at {self.obstacle_distance_m:.2f}m!")
    
    def update_pose(self):
        """Get robot pose from TF"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            self.robot_pose = (x, y, 0.0)
        except Exception:
            pass  # TF not ready yet
    
    def pick_best_frontier(self):
        """Pick closest unexplored frontier"""
        if not self.current_frontiers:
            return None
        
        self.update_pose()
        robot_x, robot_y, _ = self.robot_pose
        
        # Score = 1/distance (prefer closer)
        best = None
        best_score = -999
        for fx, fy in self.current_frontiers:
            dist = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            score = 1.0 / (dist + 0.1)  # +0.1 to avoid division by zero
            if score > best_score:
                best_score = score
                best = (fx, fy)
        
        return best
    
    def send_goal_to_nav2(self, goal_x, goal_y):
        """Send goal to Nav2 navigate_to_pose"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 server not ready")
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_cb)
        
        return True
    
    def goal_response_cb(self, future):
        """Handle Nav2 goal response"""
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("✅ Goal accepted by Nav2")
            self.nav2_ready = True
        else:
            self.get_logger().warn("❌ Goal rejected by Nav2")
    
    def emergency_backup(self):
        """Move backward at full speed for backup_time seconds"""
        self.get_logger().error(f"⬅️ BACKING UP FOR {self.backup_time}s!")
        
        for i in range(self.backup_iterations):
            # Send stop commands to stop Nav2
            stop_msg = Twist()
            self.cmd_vel_nav_pub.publish(stop_msg)
            
            # Send backup command
            backup_msg = Twist()
            backup_msg.linear.x = self.backup_speed
            self.cmd_vel_pub.publish(backup_msg)
            self.cmd_vel_nav_pub.publish(backup_msg)
            
            self.get_logger().error(f"  ⬅️ {i+1}/{self.backup_iterations}")
            time.sleep(0.05)
        
        # Stop motors
        stop_msg = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
            self.cmd_vel_nav_pub.publish(stop_msg)
            time.sleep(0.05)
        
        self.get_logger().error("✅ Backup complete")
        self.obstacle_detected = False
    
    def rotate_to_scan(self):
        """Rotate in place to scan for clear direction"""
        self.get_logger().info("🔄 Rotating to scan...")
        
        # Rotate left for ~45 degrees (1.5 rad)
        rotate_msg = Twist()
        rotate_msg.angular.z = 1.0  # rad/s
        
        for _ in range(100):  # ~5 seconds
            self.cmd_vel_pub.publish(rotate_msg)
            if not self.obstacle_detected:
                break
            time.sleep(0.05)
        
        # Stop
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("✅ Scan complete")
    
    def main_loop(self):
        """Main exploration state machine"""
        
        # ==== PHASE 1: INITIALIZATION ====
        if self.current_phase == Phase.INIT:
            self.get_logger().info("⏳ Waiting for Nav2 and frontiers...")
            
            # Wait for Nav2 to be ready
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                if self.current_frontiers:
                    self.get_logger().info("✅ System ready! Starting exploration")
                    self.current_phase = Phase.EXPLORE
                    self.phase_start_time = time.time()
        
        # ==== PHASE 2: EXPLORE ====
        elif self.current_phase == Phase.EXPLORE:
            # Check if obstacles detected
            if self.obstacle_detected:
                self.get_logger().error("💥 Obstacle collision detected!")
                self.current_phase = Phase.OBSTACLE
                return
            
            # Pick best frontier
            goal = self.pick_best_frontier()
            if goal is None:
                self.no_frontier_cycles += 1
                if self.no_frontier_cycles > 10:
                    self.get_logger().info("🎉 Exploration complete! No more frontiers.")
                    self.current_phase = Phase.DONE
                return
            
            self.no_frontier_cycles = 0
            goal_x, goal_y = goal
            self.get_logger().info(f"🎯 Exploring frontier at ({goal_x:.2f}, {goal_y:.2f})")
            self.send_goal_to_nav2(goal_x, goal_y)
        
        # ==== PHASE 3: OBSTACLE HANDLING ====
        elif self.current_phase == Phase.OBSTACLE:
            self.emergency_backup()
            self.current_phase = Phase.RESCAN
            self.phase_start_time = time.time()
        
        # ==== PHASE 4: RESCAN ====
        elif self.current_phase == Phase.RESCAN:
            elapsed = time.time() - self.phase_start_time
            
            if elapsed < 1.5:  # Rotate for 1.5 seconds
                self.rotate_to_scan()
            else:
                # Path should be clear now, resume exploration
                if self.current_frontiers:
                    self.get_logger().info("✅ Path clear - resuming exploration")
                    self.current_phase = Phase.EXPLORE
                else:
                    self.current_phase = Phase.DONE
        
        # ==== PHASE 5: DONE ====
        elif self.current_phase == Phase.DONE:
            self.get_logger().info("🏁 Mission complete!")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
