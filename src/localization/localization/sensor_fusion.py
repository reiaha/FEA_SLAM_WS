#!/usr/bin/env python3
"""
sensor_fusion_ekf.py

ROS2 node (rclpy) that reads IMU from Arduino serial and LiDAR scans,
runs an Extended Kalman Filter (EKF) with state [x, y, theta, vx, vy],
publishes imu/data_raw, scan_fused, odom, and broadcasts TFs.

Note: compute_lidar_pose() is a placeholder — replace with real LiDAR odometry / scan-matching.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import math
import numpy as np
import re
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class IMULidarEKF(Node):
    def __init__(self):
        super().__init__('imu_lidar_ekf')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan_fused', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to raw LiDAR scans (BEST_EFFORT typical for many LiDARs)
        lidar_qos = QoSProfile(depth=10,
                               reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, 'scan', self.lidar_callback, lidar_qos)
        self.latest_scan = None

        # Serial to Arduino (MPU6050)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)  # allow Arduino reset
            self.get_logger().info('Arduino connected on /dev/ttyACM0')
        except serial.SerialException:
            self.get_logger().error('Failed to open /dev/ttyACM0')
            self.ser = None

        # EKF state: [x, y, theta, vx, vy]
        self.state = np.zeros(5, dtype=float)
        # Covariance
        self.P = np.eye(5) * 0.01

        # Process noise (tune these)
        q_pos = 1e-3
        q_theta = 1e-4
        q_vel = 1e-2
        self.Q = np.diag([q_pos, q_pos, q_theta, q_vel, q_vel])

        # Measurement noise for LiDAR position (tune these)
        r_pos = 0.05  # variance in meters
        self.R = np.diag([r_pos, r_pos])

        # IMU scaling
        self.accel_g_to_ms2 = 9.80665  # convert g to m/s^2
        self.gyro_deg_to_rad = math.pi / 180.0

        # Timing
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        # Timer (50 Hz safe; will check serial timestamps)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.get_logger().info('IMU-LiDAR EKF node initialized.')

    def lidar_callback(self, msg: LaserScan):
        # Store latest scan; will be fused when IMU message processed
        self.latest_scan = msg

    def compute_lidar_pose(self, scan: LaserScan):
        """
        Placeholder function to extract a 2D pose (x, y, theta) from a LaserScan.
        Replace this with a real LiDAR odometry or scan-matching algorithm
        (for example, using scanmatchers, ICP, or a dedicated LiDAR odom node).

        For now, this function returns None or a simple incremental pose if you prefer.
        """
        # Placeholder: no real LiDAR odometry available.
        # Return None to skip measurement update.
        return None

    def timer_callback(self):
        # Read serial line if available
        if not self.ser:
            return

        try:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not raw:
            return

        # Extract numeric fields only (filter out debug text from Arduino)
        # Expect at least 6 numbers: ax_g, ay_g, az_g, gx_deg, gy_deg, gz_deg
        numeric = re.findall(r"[-+]?[0-9]*\.?[0-9]+", raw)
        if len(numeric) < 6:
            self.get_logger().warn(f"Invalid IMU data (not enough numbers): {raw}")
            return

        try:
            ax_g, ay_g, az_g, gx_d, gy_d, gz_d = map(float, numeric[:6])
        except Exception:
            self.get_logger().warn(f"Invalid IMU data (parse error): {raw}")
            return

        # Convert to SI units
        ax = ax_g * self.accel_g_to_ms2
        ay = ay_g * self.accel_g_to_ms2
        az = az_g * self.accel_g_to_ms2
        gx = gx_d * self.gyro_deg_to_rad
        gy = gy_d * self.gyro_deg_to_rad
        gz = gz_d * self.gyro_deg_to_rad  # angular rate around Z (rad/s) assumed gz for yaw

        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.orientation_covariance[0] = -1  # orientation unknown
        self.imu_pub.publish(imu_msg)

        # EKF prediction step using IMU
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time if self.last_time is not None else 0.02
        # clamp dt to reasonable bounds
        if dt <= 0 or dt > 0.5:
            dt = 0.02
        self.last_time = now

        # Unpack state
        x, y, theta, vx, vy = self.state

        # Accelerations are in the sensor/body frame: rotate to world frame for prediction
        a_world_x = math.cos(theta) * ax - math.sin(theta) * ay
        a_world_y = math.sin(theta) * ax + math.cos(theta) * ay
        # Integrate velocities and positions (constant acceleration model)
        x_pred = x + vx * dt + 0.5 * a_world_x * dt**2
        y_pred = y + vy * dt + 0.5 * a_world_y * dt**2
        theta_pred = theta + gz * dt  # assume gz is yaw rate
        vx_pred = vx + a_world_x * dt
        vy_pred = vy + a_world_y * dt

        x_pred = float(x_pred); y_pred = float(y_pred); theta_pred = float(theta_pred)
        vx_pred = float(vx_pred); vy_pred = float(vy_pred)

        # Build state transition Jacobian F (5x5)
        F = np.eye(5)
        # Partial derivatives for x,y wrt vx,vy
        F[0, 3] = dt
        F[1, 4] = dt
        # Partial derivatives due to theta dependence of a_world terms
        # a_world_x = cos(theta)*ax - sin(theta)*ay
        daxdtheta = -math.sin(theta) * ax - math.cos(theta) * ay
        daydtheta =  math.cos(theta) * ax - math.sin(theta) * ay
        F[0, 2] = 0.5 * daxdtheta * dt**2
        F[1, 2] = 0.5 * daydtheta * dt**2
        F[3, 2] = daxdtheta * dt
        F[4, 2] = daydtheta * dt
        # theta derivative wrt theta is 1 (already identity)
        # velocities derivatives wrt velocities remain 1

        # Predict covariance
        P_pred = F @ self.P @ F.T + self.Q

        # Store predictions
        self.state = np.array([x_pred, y_pred, theta_pred, vx_pred, vy_pred], dtype=float)
        self.P = P_pred

        # If LiDAR pose measurement available, perform EKF update
        z = None
        if self.latest_scan is not None:
            lidar_pose = self.compute_lidar_pose(self.latest_scan)
            if lidar_pose is not None:
                # Expect lidar_pose = [x_l, y_l, theta_l] or at least x,y
                z = np.array([lidar_pose[0], lidar_pose[1]], dtype=float)

        if z is not None:
            # Measurement matrix H: maps state to measurement z = [x, y]
            H = np.zeros((2, 5))
            H[0, 0] = 1.0
            H[1, 1] = 1.0

            # Innovation
            y_err = z - H @ self.state
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.state = self.state + K @ y_err
            self.P = (np.eye(5) - K @ H) @ self.P

            # Optionally clear latest_scan after using it
            # self.latest_scan = None

        # Publish fused LiDAR scan (time-synced)
        if self.latest_scan is not None:
            fused = self.latest_scan
            fused.header.stamp = imu_msg.header.stamp
            fused.header.frame_id = 'laser_frame'
            self.lidar_pub.publish(fused)

        # Publish odometry (filtered state)
        odom = Odometry()
        odom.header.stamp = imu_msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = 0.0
        # orientation quaternion from yaw theta
        qz = math.sin(self.state[2] / 2.0)
        qw = math.cos(self.state[2] / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # publish velocities as twist if desired
        odom.twist.twist.linear.x = float(self.state[3])
        odom.twist.twist.linear.y = float(self.state[4])
        odom.twist.twist.angular.z = gz

        self.odom_pub.publish(odom)

        # Broadcast TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = imu_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.state[0])
        t.transform.translation.y = float(self.state[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Broadcast TF base_link -> laser_frame (static offset)
        t2 = TransformStamped()
        t2.header.stamp = imu_msg.header.stamp
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser_frame'
        t2.transform.translation.x = 0.1
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.1
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)

        # Log filtered pose at low frequency
        if int(time.time()) % 2 == 0:
            self.get_logger().info(
                f"EKF | x: {self.state[0]:.3f} y: {self.state[1]:.3f} theta: {self.state[2]:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = IMULidarEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
