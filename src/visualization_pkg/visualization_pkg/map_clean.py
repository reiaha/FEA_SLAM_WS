#!/usr/bin/env python3
"""
map_clean_node: republishes /map as /map_display with unknown cells (-1)
replaced by free (0) so RViz shows only the explored area without the grey
pre-allocation border. All navigation nodes still use the real /map.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapCleanNode(Node):
    def __init__(self):
        super().__init__('map_clean_node')

        transient_local_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            OccupancyGrid, '/map', self.cb, transient_local_qos)
        self.pub = self.create_publisher(
            OccupancyGrid, '/map_display', transient_local_qos)

    def cb(self, msg: OccupancyGrid):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        raw = bytes(msg.data)
        out.data = bytes(0 if b == 255 else b for b in raw)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapCleanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
