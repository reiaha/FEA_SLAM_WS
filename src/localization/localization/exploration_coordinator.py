#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap, ManageLifecycleNodes
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
        self.declare_parameter('frontier_selection_method', 'astar')  # 'bfs', 'astar', or 'gain'
        # Nav2 takes time to fully initialize all components
        self.declare_parameter('nav2_init_delay', 15.0)  # 15s for Nav2 initialization (was 30s)
        
        self.max_time = self.get_parameter('max_exploration_time').value
        self.selection_method = self.get_parameter('frontier_selection_method').value
        self.nav2_init_delay = self.get_parameter('nav2_init_delay').value
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Lifecycle manager client (to activate Nav2 if needed)
        self.lifecycle_activate_client = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        
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
        
        # Track visited frontiers to avoid revisiting
        self.visited_frontiers = []  # List of (x, y) positions
        self.min_frontier_distance = 0.8  # Increased from 0.5 to 0.8m
        self.robot_position = (0.0, 0.0)  # Track robot position
        
        # Timer for exploration logic
        self.timer = self.create_timer(2.0, self.exploration_loop)
        
        self.get_logger().info('Exploration Coordinator initialized')
        self.get_logger().info(f'Selection method: {self.selection_method}')
        self.get_logger().info(f'Waiting {self.nav2_init_delay}s for Nav2 to initialize...')
    
    def frontiers_callback(self, msg: MarkerArray):
        """Receive frontier detections"""
        if len(msg.markers) > 0 and len(self.current_frontiers) == 0:
            self.get_logger().info(f'🎯 Received first batch of {len(msg.markers)} frontiers')
        
        self.current_frontiers = msg.markers
        
        if len(msg.markers) == 0 and len(self.visited_frontiers) > 0:
            # Only end exploration if we've actually visited some frontiers
            self.get_logger().info('✅ Exploration Complete! No more frontiers detected.')
            self.get_logger().info(f'   Total frontiers visited: {len(self.visited_frontiers)}')
            if self.exploring:  # Only save once
                self.exploring = False
                # Auto-save map when exploration ends
                self.save_map_auto()
    
    def exploration_loop(self):
        """Main exploration control loop"""
        # Wait for Nav2 to initialize on first run
        if not self.nav2_ready:
            elapsed = (self.get_clock().now() - self.init_time).nanoseconds / 1e9
            if elapsed < self.nav2_init_delay:
                remaining = self.nav2_init_delay - elapsed
                if int(elapsed) % 3 == 0 and remaining > 1:  # Log every 3 seconds
                    self.get_logger().info(f'⏳ Waiting for Nav2... ({remaining:.0f}s remaining)')
                return
            else:
                self.get_logger().info('Nav2 init delay complete, checking server availability...')
                # Try multiple times to confirm server is ready
                for attempt in range(3):
                    if self.nav_client.wait_for_server(timeout_sec=2.0):
                        self.nav2_ready = True
                        self.get_logger().info('✅ Navigation server is READY! Starting frontier exploration.')
                        self.start_time = self.get_clock().now()
                        return
                    elif attempt < 2:
                        self.get_logger().warn(f'⚠️  Nav2 not ready (attempt {attempt+1}/3), retrying...')
                
                # If still not ready, give it more time
                self.get_logger().warn('⚠️  Navigation server still initializing. Waiting 10s more...')
                self.nav2_init_delay += 10.0
                return
        
        if not self.current_frontiers:
            self.get_logger().debug('No frontiers available yet - waiting for frontier_detector')
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
        
        if self.selection_method == 'bfs':
            return self.select_bfs_frontier()
        elif self.selection_method == 'astar':
            return self.select_astar_frontier()
        elif self.selection_method == 'gain':
            return self.select_max_gain_frontier()
        else:
            return self.current_frontiers[0]
    
    def select_bfs_frontier(self):
        """Select closest frontier using BFS approach (pure distance, no heuristic)"""
        robot_x, robot_y = self.robot_position
        
        min_distance = float('inf')
        closest_frontier = None
        
        for frontier in self.current_frontiers:
            fx = frontier.pose.position.x
            fy = frontier.pose.position.y
            
            # Check if this frontier is too close to a visited one
            if self.is_frontier_visited(fx, fy):
                self.get_logger().debug(f'Skipping visited frontier at ({fx:.2f}, {fy:.2f})')
                continue
            
            # Pure Euclidean distance (BFS style - no heuristic)
            distance = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            
            # Skip frontiers too close (noise)
            if distance < 0.3:
                continue
            
            if distance < min_distance:
                min_distance = distance
                closest_frontier = frontier
        
        if closest_frontier:
            self.get_logger().info(f'🔵 BFS selected frontier at distance: {min_distance:.2f}m')
        else:
            self.get_logger().warn('No unvisited frontiers available!')
        
        return closest_frontier
    
    def select_astar_frontier(self):
        """Select frontier using A* cost estimation (g + h)"""
        robot_x, robot_y = self.robot_position
        
        min_cost = float('inf')
        best_frontier = None
        
        for frontier in self.current_frontiers:
            fx = frontier.pose.position.x
            fy = frontier.pose.position.y
            
            # Check if this frontier is too close to a visited one
            if self.is_frontier_visited(fx, fy):
                self.get_logger().debug(f'Skipping visited frontier at ({fx:.2f}, {fy:.2f})')
                continue
            
            # Skip frontiers that are too close to current position (likely noise)
            euclidean_dist = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            if euclidean_dist < 0.3:
                continue
            
            # A* cost function: f(n) = g(n) + h(n)
            # g(n) = actual path cost from start to node (straight-line distance)
            # h(n) = heuristic estimated cost to goal (frontier gain/size)
            g_cost = euclidean_dist
            
            # Heuristic: reward larger frontiers (more information gain)
            # Negative because we want to minimize total cost but maximize gain
            h_cost = -frontier.scale.x * 0.1
            
            # Total A* cost (lower is better)
            f_cost = g_cost + h_cost
            
            if f_cost < min_cost:
                min_cost = f_cost
                best_frontier = frontier
        
        if best_frontier:
            dist = math.sqrt((best_frontier.pose.position.x - robot_x)**2 + 
                           (best_frontier.pose.position.y - robot_y)**2)
            self.get_logger().info(f'⭐ A* selected frontier with f_cost: {min_cost:.2f} '
                                   f'(distance: {dist:.2f}m, size: {best_frontier.scale.x:.2f})')
        else:
            self.get_logger().warn('No unvisited frontiers available!')
        
        return best_frontier
    
    def select_max_gain_frontier(self):
        """Select frontier with maximum information gain (size-weighted) that hasn't been visited"""
        max_gain = 0
        best_frontier = None
        
        for frontier in self.current_frontiers:
            fx = frontier.pose.position.x
            fy = frontier.pose.position.y
            
            # Check if this frontier is too close to a visited one
            if self.is_frontier_visited(fx, fy):
                self.get_logger().debug(f'Skipping visited frontier at ({fx:.2f}, {fy:.2f})')
                continue
            
            # Gain proportional to frontier size
            gain = frontier.scale.x  # Size used as proxy for information gain
            
            if gain > max_gain:
                max_gain = gain
                best_frontier = frontier
        
        if best_frontier:
            self.get_logger().info(f'Selected max-gain unvisited frontier with gain: {max_gain:.3f}')
        else:
            self.get_logger().warn('No unvisited frontiers available!')
        
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
        
        # Mark this frontier as visited BEFORE sending goal
        # This prevents selecting it again even if navigation fails
        self.visited_frontiers.append((goal_pose.pose.position.x, goal_pose.pose.position.y))
        self.get_logger().info(f'Marked frontier as visited. Total visited: {len(self.visited_frontiers)}')
        
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
            self.get_logger().info('✅ Successfully reached frontier!')
        else:
            self.get_logger().warn('⚠️  Failed to reach frontier (obstacle or timeout)')
        
        # Clear goal handle so we can select next frontier
        self.goal_handle = None
    
    def is_frontier_visited(self, x, y):
        """Check if a frontier position is too close to a visited one"""
        for visited_x, visited_y in self.visited_frontiers:
            distance = math.sqrt((x - visited_x)**2 + (y - visited_y)**2)
            if distance < self.min_frontier_distance:
                return True
        return False    
    
    def save_map_auto(self):
        """Auto-save map with timestamped filename when exploration completes"""
        self.get_logger().info('🗺️  AUTO-SAVING MAP - No frontiers remaining!')
        
        # Create timestamped filename: exploration_2026-01-23_143045
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        map_name = f'exploration_{timestamp}'
        
        # Create saved_maps directory if it doesn't exist (using workspace directory)
        workspace_dir = os.path.expanduser('~/FEA_SLAM_WS')
        maps_dir = os.path.join(workspace_dir, 'saved_maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        map_path = os.path.join(maps_dir, map_name)
        
        self.get_logger().info(f'   Map will be saved to: {map_path}')
        
        # Wait for map_saver service (give it a few seconds)
        if not self.map_saver_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('⚠️  Map saver service not available after 5s - using fallback shell script')
            self.save_map_fallback(map_name)
            return
        
        # Create and send save request
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = map_path
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        self.get_logger().info('   Calling map_saver service...')
        future = self.map_saver_client.call_async(request)
        future.add_done_callback(self.save_map_callback)
    
    def save_map_fallback(self, map_name):
        """Fallback: Use SLAM Toolbox's serialize_map service"""
        import subprocess
        
        workspace_dir = os.path.expanduser('~/FEA_SLAM_WS')
        script_path = os.path.join(workspace_dir, 'scripts', 'save_map_pgm.sh')
        maps_dir = os.path.join(workspace_dir, 'saved_maps')
        
        try:
            # Run the save_map_pgm.sh script with custom name
            cmd = f'bash {script_path}'
            self.get_logger().info(f'   Running fallback script: {cmd}')
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ Map saved successfully via fallback script!')
                self.get_logger().info(f'   Check {maps_dir}/ for the map files')
            else:
                self.get_logger().error(f'❌ Fallback script failed: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'❌ Fallback save error: {e}')
    
    def save_map_callback(self, future):
        """Handle map save response"""
        try:
            result = future.result()
            if result and hasattr(result, 'success') and result.success:
                self.get_logger().info('✅ Map saved successfully via map_saver service!')
            else:
                error_msg = result.message if hasattr(result, 'message') else 'Unknown error'
                self.get_logger().error(f'❌ Failed to save map: {error_msg}')
        except Exception as e:
            self.get_logger().error(f'❌ Map save error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
