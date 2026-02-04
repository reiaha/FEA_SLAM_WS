#!/usr/bin/env python3
"""
Sensor Fusion - SIMPLIFIED VERSION
Just reads IMU from Arduino and publishes it.
slam_toolbox handles all localization. ~80 lines vs 296.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time
import math
import re

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_lidar_ekf')  # Keep same name for launch compatibility
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Connect to Arduino
        self.ser = None
        for port in ['/dev/ttyACM0', '/dev/ttyUSB0']:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # Wait for Arduino reset
                self.get_logger().info(f'✅ Arduino IMU connected on {port}')
                break
            except:
                pass
        
        if self.ser is None:
            self.get_logger().error('❌ Failed to connect to Arduino for IMU')
            return
        
        # IMU unit conversions
        self.accel_g_to_ms2 = 9.80665      # g to m/s²
        self.gyro_deg_to_rad = math.pi / 180.0  # deg/s to rad/s
        
        # Timer to read sensor (50 Hz)
        self.create_timer(0.02, self.read_and_publish_imu)
        
        self.get_logger().info('🚀 IMU Publisher initialized (simplified - no EKF)')
    
    def read_and_publish_imu(self):
        """Read IMU from Arduino serial and publish"""
        if not self.ser or self.ser.in_waiting == 0:
            return
        
        try:
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return
        
        if not raw:
            return
        
        # Extract numbers from Arduino output (ax, ay, az, gx, gy, gz)
        numeric = re.findall(r"[-+]?[0-9]*\.?[0-9]+", raw)
        if len(numeric) < 6:
            return
        
        try:
            ax_g, ay_g, az_g, gx_deg, gy_deg, gz_deg = map(float, numeric[:6])
        except:
            return
        
        # Convert to SI units
        ax = ax_g * self.accel_g_to_ms2
        ay = ay_g * self.accel_g_to_ms2
        az = az_g * self.accel_g_to_ms2
        gx = gx_deg * self.gyro_deg_to_rad
        gy = gy_deg * self.gyro_deg_to_rad
        gz = gz_deg * self.gyro_deg_to_rad
        
        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Linear acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # Angular velocity
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Orientation unknown (no magnetometer)
        imu_msg.orientation_covariance[0] = -1.0
        
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
