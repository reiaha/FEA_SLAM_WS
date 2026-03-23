#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32


class TiltSafetyController(Node):
    def __init__(self):
        super().__init__('tilt_safety_controller')

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('slowdown_deg', 10.0)
        self.declare_parameter('stop_deg', 15.0)
        self.declare_parameter('emergency_deg', 25.0)
        self.declare_parameter('slowdown_exit_deg', 8.0)
        self.declare_parameter('stop_exit_deg', 13.0)
        self.declare_parameter('emergency_exit_deg', 22.0)
        self.declare_parameter('slowdown_speed_scale', 0.5)
        self.declare_parameter('accel_lpf_alpha', 0.3)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('accel_norm_min_ms2', 7.0)
        self.declare_parameter('accel_norm_max_ms2', 12.5)
        self.declare_parameter('imu_invalid_hold_sec', 0.8)
        self.declare_parameter('stop_trip_count', 2)
        self.declare_parameter('emergency_trip_count', 3)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.slowdown_deg = float(self.get_parameter('slowdown_deg').value)
        self.stop_deg = float(self.get_parameter('stop_deg').value)
        self.emergency_deg = float(self.get_parameter('emergency_deg').value)
        self.slowdown_exit_deg = float(self.get_parameter('slowdown_exit_deg').value)
        self.stop_exit_deg = float(self.get_parameter('stop_exit_deg').value)
        self.emergency_exit_deg = float(self.get_parameter('emergency_exit_deg').value)
        self.slowdown_speed_scale = float(self.get_parameter('slowdown_speed_scale').value)
        self.accel_lpf_alpha = float(self.get_parameter('accel_lpf_alpha').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.accel_norm_min_ms2 = float(self.get_parameter('accel_norm_min_ms2').value)
        self.accel_norm_max_ms2 = float(self.get_parameter('accel_norm_max_ms2').value)
        self.imu_invalid_hold_sec = float(self.get_parameter('imu_invalid_hold_sec').value)
        self.stop_trip_count = max(1, int(self.get_parameter('stop_trip_count').value))
        self.emergency_trip_count = max(1, int(self.get_parameter('emergency_trip_count').value))

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.speed_scale = 1.0
        self.tilt_hazard = False
        self.tilt_emergency = False
        self.state = 'NORMAL'
        self.last_state = None

        self.ax_f = None
        self.ay_f = None
        self.az_f = None
        self.last_valid_imu_time = 0.0
        self.stop_counter = 0
        self.emergency_counter = 0
        self.last_invalid_imu_log_time = 0.0

        self.create_subscription(Imu, imu_topic, self.imu_cb, 20)

        self.roll_pub = self.create_publisher(Float32, '/tilt_roll_deg', 10)
        self.pitch_pub = self.create_publisher(Float32, '/tilt_pitch_deg', 10)
        self.speed_scale_pub = self.create_publisher(Float32, '/tilt_speed_scale', 10)
        self.hazard_pub = self.create_publisher(Bool, '/tilt_hazard', 10)
        self.emergency_pub = self.create_publisher(Bool, '/tilt_emergency', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.create_timer(1.0 / publish_rate_hz, self.publish_state)

        self.get_logger().info(
            f'Tilt safety online (slowdown={self.slowdown_deg:.1f}deg, '
            f'stop={self.stop_deg:.1f}deg, emergency={self.emergency_deg:.1f}deg)'
        )

    def imu_cb(self, msg: Imu):
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        accel_norm = math.sqrt((ax * ax) + (ay * ay) + (az * az))
        if not (self.accel_norm_min_ms2 <= accel_norm <= self.accel_norm_max_ms2):
            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self.last_invalid_imu_log_time) >= 1.0:
                self.last_invalid_imu_log_time = now
                self.get_logger().warn(
                    f'Ignoring invalid IMU accel norm={accel_norm:.2f} m/s^2 '
                    f'(range {self.accel_norm_min_ms2:.1f}-{self.accel_norm_max_ms2:.1f})'
                )
            return

        self.last_valid_imu_time = self.get_clock().now().nanoseconds / 1e9

        if self.ax_f is None:
            self.ax_f, self.ay_f, self.az_f = ax, ay, az
        else:
            a = min(1.0, max(0.0, self.accel_lpf_alpha))
            self.ax_f = (a * ax) + ((1.0 - a) * self.ax_f)
            self.ay_f = (a * ay) + ((1.0 - a) * self.ay_f)
            self.az_f = (a * az) + ((1.0 - a) * self.az_f)

        self.roll_deg = math.degrees(math.atan2(self.ay_f, self.az_f))
        self.pitch_deg = math.degrees(
            math.atan2(-self.ax_f, math.sqrt((self.ay_f * self.ay_f) + (self.az_f * self.az_f)))
        )

        max_abs_tilt = max(abs(self.roll_deg), abs(self.pitch_deg))
        candidate_state = self.state

        if self.state == 'EMERGENCY':
            if max_abs_tilt <= self.emergency_exit_deg:
                candidate_state = 'STOP' if max_abs_tilt > self.stop_deg else (
                    'SLOWDOWN' if max_abs_tilt > self.slowdown_deg else 'NORMAL'
                )
        elif self.state == 'STOP':
            if max_abs_tilt > self.emergency_deg:
                candidate_state = 'EMERGENCY'
            elif max_abs_tilt <= self.stop_exit_deg:
                candidate_state = 'SLOWDOWN' if max_abs_tilt > self.slowdown_deg else 'NORMAL'
        elif self.state == 'SLOWDOWN':
            if max_abs_tilt > self.emergency_deg:
                candidate_state = 'EMERGENCY'
            elif max_abs_tilt > self.stop_deg:
                candidate_state = 'STOP'
            elif max_abs_tilt <= self.slowdown_exit_deg:
                candidate_state = 'NORMAL'
        else:          
            if max_abs_tilt > self.emergency_deg:
                candidate_state = 'EMERGENCY'
            elif max_abs_tilt > self.stop_deg:
                candidate_state = 'STOP'
            elif max_abs_tilt > self.slowdown_deg:
                candidate_state = 'SLOWDOWN'

        if candidate_state == 'EMERGENCY':
            self.emergency_counter += 1
            self.stop_counter = 0
            if self.state != 'EMERGENCY' and self.emergency_counter < self.emergency_trip_count:
                candidate_state = self.state
        elif candidate_state == 'STOP':
            self.stop_counter += 1
            self.emergency_counter = 0
            if self.state not in ('STOP', 'EMERGENCY') and self.stop_counter < self.stop_trip_count:
                candidate_state = self.state
        else:
            self.stop_counter = 0
            self.emergency_counter = 0

        self.state = candidate_state

        if self.state == 'EMERGENCY':
            self.speed_scale = 0.0
            self.tilt_hazard = True
            self.tilt_emergency = True
        elif self.state == 'STOP':
            self.speed_scale = 0.0
            self.tilt_hazard = True
            self.tilt_emergency = False
        elif self.state == 'SLOWDOWN':
            self.speed_scale = min(1.0, max(0.0, self.slowdown_speed_scale))
            self.tilt_hazard = False
            self.tilt_emergency = False
        else:
            self.speed_scale = 1.0
            self.tilt_hazard = False
            self.tilt_emergency = False

        if self.state != self.last_state:
            self.last_state = self.state
            self.get_logger().warn(
                f'Tilt state={self.state} roll={self.roll_deg:.2f}deg pitch={self.pitch_deg:.2f}deg '
                f'scale={self.speed_scale:.2f}'
            )

    def publish_state(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_valid_imu_time > 0.0 and (now - self.last_valid_imu_time) > self.imu_invalid_hold_sec:
            self.state = 'NORMAL'
            self.speed_scale = 1.0
            self.tilt_hazard = False
            self.tilt_emergency = False

        roll_msg = Float32()
        pitch_msg = Float32()
        scale_msg = Float32()
        hazard_msg = Bool()
        emergency_msg = Bool()

        roll_msg.data = float(self.roll_deg)
        pitch_msg.data = float(self.pitch_deg)
        scale_msg.data = float(self.speed_scale)
        hazard_msg.data = bool(self.tilt_hazard)
        emergency_msg.data = bool(self.tilt_emergency)

        self.roll_pub.publish(roll_msg)
        self.pitch_pub.publish(pitch_msg)
        self.speed_scale_pub.publish(scale_msg)
        self.hazard_pub.publish(hazard_msg)
        self.emergency_pub.publish(emergency_msg)

        if self.tilt_hazard or self.tilt_emergency:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.cmd_vel_nav_pub.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = TiltSafetyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'context is not valid' not in str(e):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
