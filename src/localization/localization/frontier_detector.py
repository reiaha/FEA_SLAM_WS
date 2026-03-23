#!/usr/bin/env python3
"""
Frontier Detection Node for FEA-SLAM Robot
Analyzes occupancy grid map and publishes frontier candidate locations
"""

import rclpy
from rclpy.node import Node
import numpy as np
import heapq
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        
        self.declare_parameter('min_frontier_size', 2)
        self.declare_parameter('min_distance_to_frontier', 0.1)
        self.declare_parameter('frontier_threshold', 20)
        self.declare_parameter('free_space_threshold', 35)
        self.declare_parameter('min_unknown_neighbors', 1)
        self.declare_parameter('diagnostics', True)
        self.declare_parameter('diagnostics_log_interval', 2.0)
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.min_distance = self.get_parameter('min_distance_to_frontier').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.free_space_threshold = self.get_parameter('free_space_threshold').value
        self.min_unknown_neighbors = int(self.get_parameter('min_unknown_neighbors').value)
        self.diagnostics = bool(self.get_parameter('diagnostics').value)
        self.diagnostics_log_interval = float(self.get_parameter('diagnostics_log_interval').value)
        
        self.last_frontier_time = 0.0
        self.frontier_throttle_rate = 1.0                                   
        self.last_diag_log_time = 0.0
        
        self.frontiers_pub = self.create_publisher(MarkerArray, 'frontiers', 10)
        self.frontier_points_pub = self.create_publisher(PointStamped, 'frontier_points', 10)
        
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
        current_time = time.time()
        if current_time - self.last_frontier_time < 1.0 / self.frontier_throttle_rate:
            return                                 
        self.last_frontier_time = current_time
        
        self.map_data = msg
        
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position
        
        try:
            map_array = np.array(msg.data).reshape((height, width))
        except ValueError as e:
            self.get_logger().error(f'Failed to reshape map data: {e}')
            return

        if self.diagnostics and (current_time - self.last_diag_log_time) >= self.diagnostics_log_interval:
            self.last_diag_log_time = current_time
            known_cells = int(np.count_nonzero(map_array >= 0))
            unknown_cells = int(np.count_nonzero(map_array == -1))
            free_cells = int(np.count_nonzero((map_array >= 0) & (map_array < self.free_space_threshold)))
            self.get_logger().info(
                f"🧭 Frontier map stats: known={known_cells}, unknown={unknown_cells}, free={free_cells}, "
                f"resolution={resolution:.3f}, size={width}x{height}"
            )
        
        frontier_cells = self.find_frontiers(map_array)
        
        if len(frontier_cells) == 0:
            marker_array = MarkerArray()
            self.frontiers_pub.publish(marker_array)
            self.get_logger().info('No frontiers found in current map update')
            return
        
        frontier_clusters = self.cluster_frontiers(frontier_cells, map_array)
        
        valid_frontiers = []
        for i, cluster in enumerate(frontier_clusters):
            if len(cluster) >= self.min_frontier_size:
                cluster_array = np.array(cluster)
                centroid_cell = np.mean(cluster_array, axis=0).astype(int)
                
                centroid_cell[0] = np.clip(centroid_cell[0], 0, height - 1)
                centroid_cell[1] = np.clip(centroid_cell[1], 0, width - 1)
                
                world_x = origin.x + (centroid_cell[1] * resolution)
                world_y = origin.y + (centroid_cell[0] * resolution)
                
                valid_frontiers.append({
                    'x': world_x,
                    'y': world_y,
                    'size': len(cluster),
                    'cell': centroid_cell
                })
        
        self.get_logger().info(
            f'Found {len(frontier_cells)} frontier cells, {len(frontier_clusters)} clusters, '
            f'{len(valid_frontiers)} valid (size >= {self.min_frontier_size})'
        )
        
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
            
            size = min(0.3, 0.1 + (frontier['size'] * 0.01))
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.1
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 2                                  
            
            marker_array.markers.append(marker)
        
        MAX_DISPLAYED_FRONTIERS = 30
        if len(marker_array.markers) > MAX_DISPLAYED_FRONTIERS:
            marker_array.markers.sort(key=lambda m: m.scale.x, reverse=True)
            marker_array.markers = marker_array.markers[:MAX_DISPLAYED_FRONTIERS]
            for idx, m in enumerate(marker_array.markers):
                m.id = idx
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
                
                if cell >= 0 and cell < self.free_space_threshold:
                    neighbors = [
                        map_array[y-1, x],
                        map_array[y+1, x],
                        map_array[y, x-1],
                        map_array[y, x+1],
                        map_array[y-1, x-1],
                        map_array[y-1, x+1],
                        map_array[y+1, x-1],
                        map_array[y+1, x+1],
                    ]
                    
                    unknown_count = sum(1 for n in neighbors if n == -1)
                    
                    if unknown_count >= max(1, self.min_unknown_neighbors):
                        frontier_cells.append((y, x))
        
        return frontier_cells
    
    def cluster_frontiers(self, frontier_cells, map_array):
        """Cluster frontier cells using A* pathfinding-based connectivity"""
        if not frontier_cells:
            return []

        frontier_set = set(frontier_cells)
        
        visited = set()
        clusters = []
        
        for cell in frontier_cells:
            if cell not in visited:
                try:
                    cluster = self._astar_cluster_region(cell, frontier_set, map_array, visited)
                    if cluster:
                        clusters.append(cluster)
                except Exception as e:
                    self.get_logger().warn(f'Error clustering cell {cell}: {e}')
                    if cell not in visited:
                        visited.add(cell)
                        clusters.append([cell])
        
        return clusters
    
    def _astar_cluster_region(self, start_cell, all_frontier_cells, map_array, visited):
        """Use A* to cluster frontier regions by path cost"""
        cluster = []
        open_set = [(0, 0, start_cell)]
        g_scores = {start_cell: 0}
        
        height, width = map_array.shape
        
        while open_set:
            f_score, g_score, current = heapq.heappop(open_set)
            
            if current in visited:
                continue
            
            visited.add(current)
            cluster.append(current)
            
            y, x = current
            
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                ny, nx = y + dy, x + dx
                neighbor = (ny, nx)
                
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue                                
                
                if neighbor not in all_frontier_cells or neighbor in visited:
                    continue
                
                move_cost = 1.414 if abs(dy) + abs(dx) == 2 else 1.0
                
                try:
                    cell_value = map_array[ny, nx]
                    cell_cost = abs(cell_value) / 100.0 if cell_value >= 0 else 0.5
                except IndexError:
                    cell_cost = 0.5                               
                
                tentative_g = g_score + move_cost + cell_cost
                
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    
                    h_score = abs(ny - start_cell[0]) + abs(nx - start_cell[1])
                    f_score = tentative_g + h_score
                    
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        
        return cluster

def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Frontier detector error: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

