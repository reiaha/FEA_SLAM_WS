#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.get_logger().info('=== Map Visualizer Node Starting ===')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info('Subscribed to /map topic')
        
        self.marker_pub = self.create_publisher(Marker, '/map_marker', 10)
        self.get_logger().info('Publishing markers on /map_marker topic')
        
        self.map_info = None
        self.data = None
        self.get_logger().info('Map Visualizer initialized successfully')

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().debug('Map callback triggered')
        self.map_info = msg.info
        self.data = msg.data
        
        occupied_cells = sum(1 for val in self.data if val > 0)
        total_cells = len(self.data)
        occupancy_pct = (occupied_cells / total_cells * 100) if total_cells > 0 else 0
        
        self.get_logger().info(
            f"Map received: {self.map_info.width}x{self.map_info.height} cells, "
            f"resolution={self.map_info.resolution:.3f}m, "
            f"occupied={occupied_cells}/{total_cells} ({occupancy_pct:.1f}%)"
        )
        self.publish_markers()

    def publish_markers(self):
        if not self.data or not self.map_info:
            self.get_logger().warn('Cannot publish markers: map data not available')
            return
        
        self.get_logger().debug('Creating marker visualization...')
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "map_cells"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.map_info.resolution * 0.8
        marker.scale.y = self.map_info.resolution * 0.8
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        width = self.map_info.width
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        for idx, value in enumerate(self.data):
            if value > 0:
                x = (idx % width) * resolution + origin_x + resolution/2
                y = (idx // width) * resolution + origin_y + resolution/2
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.05
                marker.points.append(p)
        
        num_points = len(marker.points)
        self.get_logger().info(f'Publishing {num_points} marker points to /map_marker')
        self.marker_pub.publish(marker)
        self.get_logger().debug('Marker published successfully')

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    node.get_logger().info('Map Visualizer spinning - waiting for map data...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.get_logger().info('Destroying node...')
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Map Visualizer shutdown complete')

if __name__ == '__main__':
    main()
