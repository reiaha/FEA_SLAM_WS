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
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('min_distance_to_frontier', 0.3)
        self.declare_parameter('frontier_threshold', 50)  # Unknown cell threshold
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_distance = self.get_parameter('min_distance_to_frontier').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        
        # Publishers
        self.frontiers_pub = self.create_publisher(MarkerArray, 'frontiers', 10)
        self.frontier_points_pub = self.create_publisher(PointStamped, 'frontier_points', 10)
        
        # Subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
            ))
        
        self.map_data = None
        self.get_logger().info('Frontier Detector initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        """Analyze map for frontiers"""
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
        for cluster in frontier_clusters:
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
        
        self.get_logger().info(f'Found {len(valid_frontiers)} frontier clusters')
        
        # Publish frontiers as markers for RViz
        self.publish_frontier_markers(valid_frontiers, msg.header.frame_id)
    
    def find_frontiers(self, map_array):
        """Find frontier cells (edge between explored and unexplored)"""
        height, width = map_array.shape
        frontier_cells = []
        
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cell = map_array[y, x]
                
                # Skip if cell is known (occupied or free)
                if cell >= 0 and cell < self.frontier_threshold:
                    # Check if adjacent to unknown cell
                    neighbors = [
                        map_array[y-1, x],
                        map_array[y+1, x],
                        map_array[y, x-1],
                        map_array[y, x+1],
                        map_array[y-1, x-1],
                        map_array[y-1, x+1],
                        map_array[y+1, x-1],
                        map_array[y+1, x+1]
                    ]
                    
                    # If any neighbor is unknown (>= frontier_threshold), this is a frontier
                    if any(n >= self.frontier_threshold or n == -1 for n in neighbors):
                        frontier_cells.append((y, x))
        
        return frontier_cells
    
    def cluster_frontiers(self, frontier_cells, map_array):
        """Cluster frontier cells using connected components"""
        if not frontier_cells:
            return []
        
        visited = set()
        clusters = []
        
        for cell in frontier_cells:
            if cell not in visited:
                # BFS to find connected component
                cluster = []
                queue = [cell]
                
                while queue:
                    y, x = queue.pop(0)
                    if (y, x) in visited:
                        continue
                    
                    visited.add((y, x))
                    cluster.append((y, x))
                    
                    # Check neighbors
                    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1),
                                   (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < map_array.shape[0] and 0 <= nx < map_array.shape[1]:
                            if (ny, nx) not in visited and (ny, nx) in frontier_cells:
                                queue.append((ny, nx))
                
                if cluster:
                    clusters.append(cluster)
        
        return clusters
    
    def publish_frontier_markers(self, frontiers, frame_id):
        """Publish frontiers as RViz markers"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = frame_id
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

def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
