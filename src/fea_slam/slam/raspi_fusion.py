import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import serial
import time
import math

class IMULidarFusion(Node):
    def __init__(self):
        super().__init__('imu_lidar_fusion')

        # ROS 2 publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan_fused', 10)

        # Subscribe to existing LiDAR topic
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.latest_scan = None

        # Serial connection to Arduino MPU6050
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)

        # MPU6050 scale factors
        self.accel_scale = 9.80665 / 16384.0
        self.gyro_scale = math.pi / (180 * 131)

        # Timer to read MPU6050
        self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def lidar_callback(self, msg):
        # Save the latest LiDAR scan
        self.latest_scan = msg

    def timer_callback(self):
        # Read IMU data from Arduino
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = map(int, line.split(','))

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                # Accelerometer in m/s²
                imu_msg.linear_acceleration.x = ax_raw * self.accel_scale
                imu_msg.linear_acceleration.y = ay_raw * self.accel_scale
                imu_msg.linear_acceleration.z = az_raw * self.accel_scale

                # Gyroscope in rad/s
                imu_msg.angular_velocity.x = gx_raw * self.gyro_scale
                imu_msg.angular_velocity.y = gy_raw * self.gyro_scale
                imu_msg.angular_velocity.z = gz_raw * self.gyro_scale

                # Orientation unknown
                imu_msg.orientation_covariance[0] = -1

                self.imu_pub.publish(imu_msg)
                self.get_logger().info(f"Published IMU data | Acc: ({imu_msg.linear_acceleration.x:.2f}, "
                                       f"{imu_msg.linear_acceleration.y:.2f}, {imu_msg.linear_acceleration.z:.2f})")

                # If a LiDAR scan is available, publish it as a "fused" topic
                if self.latest_scan:
                    fused_scan = self.latest_scan
                    fused_scan.header.stamp = imu_msg.header.stamp  # synchronize timestamp
                    self.lidar_pub.publish(fused_scan)

            except ValueError:
                self.get_logger().warn(f"Invalid IMU data: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = IMULidarFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
