#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveTester(Node):
    def __init__(self):
        super().__init__('move_tester')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def send(self, lin_x: float, ang_z: float, duration_s: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.get_logger().info(f"cmd_vel: lin_x={lin_x:.3f} m/s, ang_z={ang_z:.3f} rad/s for {duration_s:.2f}s")
        t_end = time.time() + duration_s
        rate = self.create_rate(20)  # 20 Hz
        while rclpy.ok() and time.time() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()

    def stop(self, duration_s: float = 0.5):
        self.send(0.0, 0.0, duration_s)


def main():
    rclpy.init()
    node = MoveTester()
    try:
        node.get_logger().info("Starting movement sequence: forward → stop → rotate → stop → reverse → stop")
        # Gentle speeds; adjust if needed
        node.send(lin_x=0.05, ang_z=0.0, duration_s=2.0)   # forward ~2s
        node.stop(1.0)
        node.send(lin_x=0.0, ang_z=0.3, duration_s=2.0)    # rotate left ~2s
        node.stop(1.0)
        node.send(lin_x=-0.05, ang_z=0.0, duration_s=1.5)  # reverse ~1.5s
        node.stop(1.0)
        node.get_logger().info("Movement sequence complete.")
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
