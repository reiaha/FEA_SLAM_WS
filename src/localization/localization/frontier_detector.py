#!/usr/bin/env python3
"""
Frontier Detection Node for FEA-SLAM Robot
Analyzes occupancy grid map and publishes frontier candidate locations
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        
        # Parameters
        self.declare_parameter('min_frontier_size', 3)  # Reduced from 5 to catch smaller frontiers
        self.declare_parameter('min_distance_to_frontier', 0.3)
        self.declare_parameter('frontier_threshold', 15)  # Unknown cell threshold (lower = more sensitive)
        self.declare_parameter('free_space_threshold', 20)  # Increased from default for clearer boundaries
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_distance = self.get_parameter('min_distance_to_frontier').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.free_space_threshold = self.get_parameter('free_space_threshold').value
        
        # Performance: throttle processing to avoid slowing down RViz
        self.last_frontier_time = 0.0
        self.frontier_throttle_rate = 2.0  # Process at most every 0.5 seconds
        
        # Publishers
        self.frontiers_pub = self.create_publisher(MarkerArray, 'frontiers', 10)
        self.frontier_points_pub = self.create_publisher(PointStamped, 'frontier_points', 10)
        
        # Subscriber - use TRANSIENT_LOCAL + RELIABLE to match slam_toolbox map publisher
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            ))
        
        self.map_data = None
        self.get_logger().info('Frontier Detector initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        """Analyze map for frontiers"""
        import time
        current_time = time.time()
        if current_time - self.last_frontier_time < 1.0 / self.frontier_throttle_rate:
            return  # Skip if called too frequently
        self.last_frontier_time = current_time
        
        self.map_data = msg
        
        # Get map dimensions
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        
        # Convert to 2D array
        map_array = np.array(msg.data).reshape((height, width))
        
        # Find frontier cells (boundaries between explored and unexplored)
        frontier_cells = self.find_frontiers(map_array)
        
        if len(frontier_cells) == 0:
            self.get_logger().info('No frontiers found - map fully explored!')
            return
        
        # Cluster frontier cells
        frontier_clusters = self.cluster_frontiers(frontier_cells, map_array)
        
        # Filter and publish frontiers
        valid_frontiers = []
        for i, cluster in enumerate(frontier_clusters):
            self.get_logger().debug(f'Cluster {i}: size={len(cluster)}, min_required={self.min_frontier_size}')
            if len(cluster) >= self.min_frontier_size:
                # Calculate centroid
                cluster_array = np.array(cluster)
                centroid_cell = np.mean(cluster_array, axis=0).astype(int)
                
                # Convert to world coordinates
                world_x = origin.x + (centroid_cell[1] * resolution)
                world_y = origin.y + (centroid_cell[0] * resolution)
                
                valid_frontiers.append({
                    'x': world_x,
                    'y': world_y,
                    'size': len(cluster),
                    'cell': centroid_cell
                })
        
        self.get_logger().info(f'Found {len(frontier_clusters)} frontier clusters, {len(valid_frontiers)} are valid (size >= {self.min_frontier_size})')
        
        # Publish frontiers as markers for RViz
        marker_array = MarkerArray()
        for i, frontier in enumerate(valid_frontiers):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = msg.header.frame_id
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = frontier['x']
            marker.pose.position.y = frontier['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Size proportional to frontier size
            size = min(0.3, 0.1 + (frontier['size'] * 0.01))
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.1
            
            # Color: green for frontiers
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 2  # Markers expire after 2 seconds
            
            marker_array.markers.append(marker)
        
        self.frontiers_pub.publish(marker_array)
        self.get_logger().info(f'✅ Published MarkerArray with {len(marker_array.markers)} markers to /frontiers')
        if len(marker_array.markers) > 0:
            self.get_logger().info(f'   First marker at: ({marker_array.markers[0].pose.position.x:.2f}, {marker_array.markers[0].pose.position.y:.2f})')
        else:
            self.get_logger().warn('⚠️  MarkerArray is EMPTY - no markers to publish!')
    
    def find_frontiers(self, map_array):
        """Find frontier cells (edge between explored and unexplored)"""
        height, width = map_array.shape
        frontier_cells = []
        
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cell = map_array[y, x]
                
                # Only consider FREE cells (low cost) - use parameter for threshold
                if cell >= 0 and cell < self.free_space_threshold:  # Free space only
                    # Check 4-connectivity neighbors for speed
                    neighbors = [
                        map_array[y-1, x],
                        map_array[y+1, x],
                        map_array[y, x-1],
                        map_array[y, x+1]
                    ]
                    
                    # Count unknown neighbors
                    unknown_count = sum(1 for n in neighbors if n == -1 or n >= self.frontier_threshold)
                    
                    # If has unknown neighbors, it's a frontier boundary
                    if unknown_count > 0:
                        frontier_cells.append((y, x))
        
        return frontier_cells
    
    def cluster_frontiers(self, frontier_cells, map_array):
        """Cluster frontier cells using A* pathfinding-based connectivity"""
        if not frontier_cells:
            return []
        
        visited = set()
        clusters = []
        
        for cell in frontier_cells:
            if cell not in visited:
                # A*-based region growing for more intelligent clustering
                cluster = self._astar_cluster_region(cell, frontier_cells, map_array, visited)
                
                if cluster:
                    clusters.append(cluster)
        
        return clusters
    
    def _astar_cluster_region(self, start_cell, all_frontier_cells, map_array, visited):
        """Use A* to cluster frontier regions by path cost"""
        import heapq
        
        cluster = []
        # Priority queue: (f_score, g_score, cell)
        open_set = [(0, 0, start_cell)]
        g_scores = {start_cell: 0}
        
        while open_set:
            f_score, g_score, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            
            visited.add(current)
            cluster.append(current)
            
            y, x = current
            
            # Explore 8-connected neighbors
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                ny, nx = y + dy, x + dx
                neighbor = (ny, nx)
                
                # Check bounds and if it's a frontier cell
                if (0 <= ny < map_array.shape[0] and 
                    0 <= nx < map_array.shape[1] and
                    neighbor in all_frontier_cells and
                    neighbor not in visited):
                    
                    # Calculate movement cost (diagonal costs more)
                    move_cost = 1.414 if abs(dy) + abs(dx) == 2 else 1.0
                    
                    # Add terrain cost based on occupancy
                    cell_cost = abs(map_array[ny, nx]) / 100.0 if map_array[ny, nx] >= 0 else 0.5
                    
                    tentative_g = g_score + move_cost + cell_cost
                    
                    if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                        g_scores[neighbor] = tentative_g
                        
                        # Heuristic: Manhattan distance to start (keep cluster compact)
                        h_score = abs(ny - start_cell[0]) + abs(nx - start_cell[1])
                        f_score = tentative_g + h_score
                        
                        heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        
        return cluster

def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
