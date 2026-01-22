#!/usr/bin/env python3
"""
Exploration Coordinator Node for FEA-SLAM Robot
Manages autonomous frontier-based exploration using Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray
import math

class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator')
        
        # Parameters
        self.declare_parameter('max_exploration_time', 600.0)  # 10 minutes
        self.declare_parameter('frontier_selection_method', 'closest')  # or 'gain'
        
        self.max_time = self.get_parameter('max_exploration_time').value
        self.selection_method = self.get_parameter('frontier_selection_method').value
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.frontiers_sub = self.create_subscription(
            MarkerArray, 'frontiers', self.frontiers_callback, 10)
        
        # Publishers
        self.current_goal_pub = self.create_publisher(PointStamped, 'current_frontier_goal', 10)
        
        # State
        self.current_frontiers = []
        self.exploring = False
        self.start_time = None
        self.goal_handle = None
        
        # Timer for exploration logic
        self.timer = self.create_timer(2.0, self.exploration_loop)
        
        self.get_logger().info('Exploration Coordinator initialized')
        self.get_logger().info(f'Selection method: {self.selection_method}')
    
    def frontiers_callback(self, msg: MarkerArray):
        """Receive frontier detections"""
        self.current_frontiers = msg.markers
        
        if len(msg.markers) == 0:
            self.get_logger().info('⭐ Exploration Complete! No more frontiers detected.')
            self.exploring = False
    
    def exploration_loop(self):
        """Main exploration control loop"""
        if not self.current_frontiers:
            self.get_logger().debug('No frontiers available yet')
            return
        
        # Check time limit
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > self.max_time:
            self.get_logger().warn(f'⏱️ Max exploration time ({self.max_time}s) exceeded!')
            self.exploring = False
            return
        
        # Check if current goal is reached
        if self.goal_handle is not None:
            if self.goal_handle.done():
                result = self.goal_handle.result()
                if result:
                    self.get_logger().info('✅ Frontier reached! Selecting next frontier...')
                    self.goal_handle = None
        
        # Send new goal if not currently navigating
        if self.goal_handle is None and len(self.current_frontiers) > 0:
            self.get_logger().info(f'🎯 Selecting frontier from {len(self.current_frontiers)} candidates...')
            selected_frontier = self.select_frontier()
            if selected_frontier:
                self.send_goal_to_frontier(selected_frontier)
            else:
                self.get_logger().warn('❌ Failed to select frontier')
    
    def select_frontier(self):
        """Select best frontier using configured method"""
        if not self.current_frontiers:
            return None
        
        if self.selection_method == 'closest':
            return self.select_closest_frontier()
        elif self.selection_method == 'gain':
            return self.select_max_gain_frontier()
        else:
            return self.current_frontiers[0]
    
    def select_closest_frontier(self):
        """Select closest frontier (greedy approach)"""
        # Assume robot is at origin in most recent map frame
        robot_x, robot_y = 0.0, 0.0
        
        min_distance = float('inf')
        closest_frontier = None
        
        for frontier in self.current_frontiers:
            fx = frontier.pose.position.x
            fy = frontier.pose.position.y
            
            distance = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_frontier = frontier
        
        if closest_frontier:
            self.get_logger().info(f'📍 Selected closest frontier at distance: {min_distance:.2f}m')
        
        return closest_frontier
    
    def select_max_gain_frontier(self):
        """Select frontier with maximum information gain (size-weighted)"""
        max_gain = 0
        best_frontier = None
        
        for frontier in self.current_frontiers:
            # Gain proportional to frontier size
            gain = frontier.scale.x  # Size used as proxy for information gain
            
            if gain > max_gain:
                max_gain = gain
                best_frontier = frontier
        
        if best_frontier:
            self.get_logger().info(f'🎯 Selected max-gain frontier with gain: {max_gain:.3f}')
        
        return best_frontier
    
    def send_goal_to_frontier(self, frontier_marker):
        """Send navigation goal to frontier"""
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frontier_marker.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = frontier_marker.pose.position.x
        goal_pose.pose.position.y = frontier_marker.pose.position.y
        goal_pose.pose.position.z = 0.0
        
        # Default orientation (facing frontier direction)
        goal_pose.pose.orientation.w = 1.0
        
        # Create Nav2 action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send goal asynchronously
        self.get_logger().info(f'🔍 Checking for nav server... (timeout 2.0s)')
        if self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f'🚀 Sending goal to frontier: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
            
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
            
            self.exploring = True
        else:
            self.get_logger().error('❌ Navigation server NOT available! Nav2 may not be running.')
    
    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().info('❌ Goal rejected by navigation server')
            return
        
        self.get_logger().info('✨ Goal accepted, robot navigating to frontier...')
        
        # Get result when done
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        
        if result and result.result:
            self.get_logger().info('✅ Successfully reached frontier!')
        else:
            self.get_logger().warn('⚠️ Failed to reach frontier (obstacle or timeout)')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
