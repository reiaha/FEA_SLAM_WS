#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray
import math
import os
from datetime import datetime
import time

class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator')
        
        # Parameters
        self.declare_parameter('max_exploration_time', 3600.0)  # 60 minutes - stops only when no frontiers
        self.declare_parameter('frontier_selection_method', 'closest')  # or 'gain'
        # Wait for Nav2 stack to initialize (ultrasonic_explorer moves robot during this time)
        self.declare_parameter('nav2_init_delay', 60.0)  # 60s for Nav2 initialization
        
        self.max_time = self.get_parameter('max_exploration_time').value
        self.selection_method = self.get_parameter('frontier_selection_method').value
        self.nav2_init_delay = self.get_parameter('nav2_init_delay').value
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Map saver service client
        self.map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')
        
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
        self.nav2_ready = False
        self.init_time = self.get_clock().now()
        
        # Timer for exploration logic
        self.timer = self.create_timer(2.0, self.exploration_loop)
        
        self.get_logger().info('Exploration Coordinator initialized')
        self.get_logger().info(f'Selection method: {self.selection_method}')
        self.get_logger().info(f'Waiting {self.nav2_init_delay}s for Nav2 to initialize...')
    
    def frontiers_callback(self, msg: MarkerArray):
        """Receive frontier detections"""
        self.current_frontiers = msg.markers
        
        if len(msg.markers) == 0:
            self.get_logger().info('Exploration Complete! No more frontiers detected.')
            self.exploring = False            # Auto-save map when exploration ends
            self.save_map_auto()
    
    def exploration_loop(self):
        """Main exploration control loop"""
        # Wait for Nav2 to initialize on first run
        if not self.nav2_ready:
            elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9
            if elapsed < self.nav2_init_delay:
                if elapsed % 2 < 0.1:  # Log every 2 seconds
                    remaining = self.nav2_init_delay - elapsed
                    self.get_logger().info(f'Waiting for Nav2... ({remaining:.1f}s remaining)')
                return
            else:
                self.get_logger().info('Nav2 init delay complete, checking server availability...')
                if self.nav_client.wait_for_server(timeout_sec=5.0):
                    self.nav2_ready = True
                    self.get_logger().info('Navigation server is READY!')
                else:
                    self.get_logger().error('Navigation server still not available after init delay!')
                    return
        
        if not self.current_frontiers:
            self.get_logger().debug('No frontiers available yet')
            return
        
        # Check time limit
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_time > self.max_time:
            self.get_logger().warn(f'Max exploration time ({self.max_time}s) exceeded!')
            self.exploring = False
            return
        
        # Check if current goal is reached
        if self.goal_handle is not None:
            if self.goal_handle.done():
                result = self.goal_handle.result()
                if result:
                    self.get_logger().info('Frontier reached! Selecting next frontier...')
                    self.goal_handle = None
        
        # Send new goal if not currently navigating
        if self.goal_handle is None and len(self.current_frontiers) > 0:
            self.get_logger().info(f'Selecting frontier from {len(self.current_frontiers)} candidates...')
            selected_frontier = self.select_frontier()
            if selected_frontier:
                self.send_goal_to_frontier(selected_frontier)
            else:
                self.get_logger().warn('Failed to select frontier')
    
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
            self.get_logger().info(f'Selected closest frontier at distance: {min_distance:.2f}m')
        
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
            self.get_logger().info(f'Selected max-gain frontier with gain: {max_gain:.3f}')
        
        return best_frontier
    
    def send_goal_to_frontier(self, frontier_marker):
        """Send navigation goal to frontier"""
        # Ensure Nav2 is ready
        if not self.nav2_ready:
            self.get_logger().warn('Nav2 not ready yet, skipping goal send')
            return
        
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
        self.get_logger().info(f'Sending goal to frontier: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by navigation server')
            return
        
        self.get_logger().info('Goal accepted, robot navigating to frontier...')
        
        # Get result when done
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()
        
        if result and result.result:
            self.get_logger().info('Successfully reached frontier!')
        else:
            self.get_logger().warn('Failed to reach frontier (obstacle or timeout)')    
    def save_map_auto(self):
        """Auto-save map with timestamped filename when exploration completes"""
        # Create timestamped filename: exploration_2026-01-23_143045.pgm
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        map_name = f'exploration_{timestamp}'
        
        # Create maps directory if it doesn't exist
        maps_dir = os.path.expanduser('~/maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        map_path = os.path.join(maps_dir, map_name)
        
        # Create and send save request
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = map_path
        
        if self.map_saver_client.service_is_ready():
            self.get_logger().info(f'Saving map: {map_path}')
            future = self.map_saver_client.call_async(request)
            future.add_done_callback(self.save_map_callback)
        else:
            self.get_logger().warn('Map saver service not available. Map not saved.')
    
    def save_map_callback(self, future):
        """Handle map save response"""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Map saved successfully!')
            else:
                self.get_logger().error(f'Failed to save map: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
