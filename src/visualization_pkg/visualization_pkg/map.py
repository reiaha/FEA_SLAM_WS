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
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/map_marker', 10)
        self.map_info = None
        self.data = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.data = msg.data
        self.get_logger().info(f"Map received: width={self.map_info.width}, height={self.map_info.height}, resolution={self.map_info.resolution}")
        self.publish_markers()

    def publish_markers(self):
        if not self.data or not self.map_info:
            return
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
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
