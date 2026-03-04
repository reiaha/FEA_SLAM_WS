#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Range, LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.srv import GetState
import serial
import time
import threading
import atexit
import glob
from threading import Lock

class ArduinoMotorBridge(Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('max_speed', 220)
        self.declare_parameter('max_speed_forward', 167)
        self.declare_parameter('max_speed_backward', 180)
        self.declare_parameter('max_linear_speed_mps', 0.35)
        self.declare_parameter('max_angular_speed_radps', 0.8)
        self.declare_parameter('fixed_pwm_forward', 150)
        self.declare_parameter('fixed_pwm_backward', 120)
        self.declare_parameter('fixed_pwm_turn', 120)
        self.declare_parameter('auto_backup_speed_mps', 0.10)
        self.declare_parameter('min_pwm', 90)
        self.declare_parameter('min_pwm_forward', 105)
        self.declare_parameter('min_pwm_backward', 100)
        self.declare_parameter('min_pwm_turn', 95)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('pwm_change_threshold', 8)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')  # Changed from base_link to match Nav2
        self.declare_parameter('enable_stuck_recovery', True)
        self.declare_parameter('stuck_speed_threshold', 5.0)
        self.declare_parameter('stuck_time', 0.6)
        self.declare_parameter('stuck_backup_pwm', 120)
        self.declare_parameter('stuck_backup_time', 0.8)
        self.declare_parameter('safety_stop_backup_time', 2.0)
        self.declare_parameter('safety_stop_backup_pwm', 120)
        self.declare_parameter('safety_stop_trigger_distance', 0.08)
        self.declare_parameter('safety_stop_confirm_count', 2)
        self.declare_parameter('safety_stop_clear_confirm_count', 2)
        self.declare_parameter('safety_stop_front_latch_time', 0.8)
        self.declare_parameter('enable_safety_override', True)
        self.declare_parameter('ultrasonic_frame', 'base_footprint')
        self.declare_parameter('ultrasonic_min_range', 0.02)
        self.declare_parameter('ultrasonic_max_range', 0.50)
        self.declare_parameter('ultrasonic_fov', 0.52)
        self.declare_parameter('rear_stop_distance', 0.40)
        self.declare_parameter('rear_backup_min_distance', 0.30)
        self.declare_parameter('rear_stop_hold_time', 0.6)
        self.declare_parameter('rear_block_min_hits', 3)
        self.declare_parameter('front_stop_distance', 0.80)
        self.declare_parameter('front_stop_hold_time', 0.6)
        self.declare_parameter('any_obstacle_stop_distance', 0.20)
        self.declare_parameter('any_obstacle_stop_hold_time', 0.5)
        self.declare_parameter('front_block_min_hits', 4)
        self.declare_parameter('any_obstacle_min_hits', 6)
        self.declare_parameter('scan_stale_timeout', 1.2)
        self.declare_parameter('flip_guard_time', 0.15)
        self.declare_parameter('wall_debug_log_interval', 1.0)
        self.declare_parameter('swap_lidar_front_back', False)
        self.declare_parameter('escape_turn_speed', 0.6)
        self.declare_parameter('escape_turn_period', 2.0)
        self.declare_parameter('escape_turn_toggle_interval', 1.2)
        self.declare_parameter('turn_angular_scale', 0.75)
        self.declare_parameter('turn_pwm_limit', 125)
        self.declare_parameter('front_obstacle_backup_speed_mps', 0.12)
        self.declare_parameter('front_obstacle_backup_turn_scale', 0.35)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_obstacle_half_angle_deg', 50.0)
        self.declare_parameter('rear_obstacle_half_angle_deg', 30.0)
        self.declare_parameter('force_backward_on_zero_cmd', False)
        self.declare_parameter('zero_cmd_forward_pwm', 110)
        self.declare_parameter('allow_backward_when_rear_blocked', False)
        self.declare_parameter('require_nav2_active', True)
        self.declare_parameter('nav2_state_check_interval', 1.0)
        self.declare_parameter('serial_reconnect_interval', 1.0)
        self.declare_parameter('serial_max_error_streak', 5)
        self.declare_parameter('serial_error_log_interval', 2.0)
        self.declare_parameter('motor_log_interval', 0.25)
        self.declare_parameter('odom_feedback_gate_enabled', True)
        self.declare_parameter('odom_stationary_speed_threshold', 5.0)
        self.declare_parameter('odom_freeze_pose_when_stationary', True)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = int(self.get_parameter('max_speed').value)
        self.max_speed_forward = int(self.get_parameter('max_speed_forward').value)
        self.max_speed_backward = int(self.get_parameter('max_speed_backward').value)
        self.max_linear_speed_mps = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular_speed_radps = float(self.get_parameter('max_angular_speed_radps').value)
        self.fixed_pwm_forward = int(self.get_parameter('fixed_pwm_forward').value)
        self.fixed_pwm_backward = int(self.get_parameter('fixed_pwm_backward').value)
        self.fixed_pwm_turn = int(self.get_parameter('fixed_pwm_turn').value)
        self.auto_backup_speed_mps = float(self.get_parameter('auto_backup_speed_mps').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.min_pwm_forward = int(self.get_parameter('min_pwm_forward').value)
        self.min_pwm_backward = int(self.get_parameter('min_pwm_backward').value)
        self.min_pwm_turn = int(self.get_parameter('min_pwm_turn').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.pwm_change_threshold = int(self.get_parameter('pwm_change_threshold').value)
        self.publish_odom = self.get_parameter('publish_odom').value

        # IMU orientation state
        self.imu_yaw = None
        self.imu_quat = None
        self.imu_last_stamp = None

        # Subscribe to IMU
        from sensor_msgs.msg import Imu
        self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_cb,
            10
        )

        self.odom_rate = float(self.get_parameter('odom_rate').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.enable_stuck_recovery = self.get_parameter('enable_stuck_recovery').value
        self.stuck_speed_threshold = float(self.get_parameter('stuck_speed_threshold').value)
        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.stuck_backup_pwm = int(self.get_parameter('stuck_backup_pwm').value)
        self.stuck_backup_time = float(self.get_parameter('stuck_backup_time').value)
        self.safety_stop_backup_time = float(self.get_parameter('safety_stop_backup_time').value)
        self.safety_stop_backup_pwm = int(self.get_parameter('safety_stop_backup_pwm').value)
        self.safety_stop_trigger_distance = float(self.get_parameter('safety_stop_trigger_distance').value)
        self.safety_stop_confirm_count = int(self.get_parameter('safety_stop_confirm_count').value)
        self.safety_stop_clear_confirm_count = int(self.get_parameter('safety_stop_clear_confirm_count').value)
        self.safety_stop_front_latch_time = float(self.get_parameter('safety_stop_front_latch_time').value)
        self.enable_safety_override = self.get_parameter('enable_safety_override').value
        self.ultrasonic_frame = self.get_parameter('ultrasonic_frame').value
        self.ultrasonic_min_range = float(self.get_parameter('ultrasonic_min_range').value)
        self.ultrasonic_max_range = float(self.get_parameter('ultrasonic_max_range').value)
        self.ultrasonic_fov = float(self.get_parameter('ultrasonic_fov').value)
        self.rear_stop_distance = float(self.get_parameter('rear_stop_distance').value)
        self.rear_backup_min_distance = float(self.get_parameter('rear_backup_min_distance').value)
        self.rear_stop_hold_time = float(self.get_parameter('rear_stop_hold_time').value)
        self.rear_block_min_hits = int(self.get_parameter('rear_block_min_hits').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_stop_hold_time = float(self.get_parameter('front_stop_hold_time').value)
        self.any_obstacle_stop_distance = float(self.get_parameter('any_obstacle_stop_distance').value)
        self.any_obstacle_stop_hold_time = float(self.get_parameter('any_obstacle_stop_hold_time').value)
        self.front_block_min_hits = int(self.get_parameter('front_block_min_hits').value)
        self.any_obstacle_min_hits = int(self.get_parameter('any_obstacle_min_hits').value)
        self.scan_stale_timeout = float(self.get_parameter('scan_stale_timeout').value)
        self.flip_guard_time = float(self.get_parameter('flip_guard_time').value)
        self.wall_debug_log_interval = float(self.get_parameter('wall_debug_log_interval').value)
        self.swap_lidar_front_back = bool(self.get_parameter('swap_lidar_front_back').value)
        self.escape_turn_speed = float(self.get_parameter('escape_turn_speed').value)
        self.escape_turn_period = float(self.get_parameter('escape_turn_period').value)
        self.escape_turn_toggle_interval = float(self.get_parameter('escape_turn_toggle_interval').value)
        self.turn_angular_scale = float(self.get_parameter('turn_angular_scale').value)
        self.turn_pwm_limit = int(self.get_parameter('turn_pwm_limit').value)
        self.front_obstacle_backup_speed_mps = float(self.get_parameter('front_obstacle_backup_speed_mps').value)
        self.front_obstacle_backup_turn_scale = float(self.get_parameter('front_obstacle_backup_turn_scale').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.front_obstacle_half_angle = math.radians(float(self.get_parameter('front_obstacle_half_angle_deg').value))
        self.rear_obstacle_half_angle = math.radians(float(self.get_parameter('rear_obstacle_half_angle_deg').value))
        self.force_backward_on_zero_cmd = bool(self.get_parameter('force_backward_on_zero_cmd').value)
        self.zero_cmd_forward_pwm = int(self.get_parameter('zero_cmd_forward_pwm').value)
        self.allow_backward_when_rear_blocked = bool(self.get_parameter('allow_backward_when_rear_blocked').value)
        self.require_nav2_active = bool(self.get_parameter('require_nav2_active').value)
        self.nav2_state_check_interval = float(self.get_parameter('nav2_state_check_interval').value)
        self.serial_reconnect_interval = float(self.get_parameter('serial_reconnect_interval').value)
        self.serial_max_error_streak = int(self.get_parameter('serial_max_error_streak').value)
        self.serial_error_log_interval = float(self.get_parameter('serial_error_log_interval').value)
        self.motor_log_interval = float(self.get_parameter('motor_log_interval').value)
        self.odom_feedback_gate_enabled = bool(self.get_parameter('odom_feedback_gate_enabled').value)
        self.odom_stationary_speed_threshold = float(self.get_parameter('odom_stationary_speed_threshold').value)
        self.odom_freeze_pose_when_stationary = bool(self.get_parameter('odom_freeze_pose_when_stationary').value)
        self.cmd_vel_override_duration = 0.2
        self.cmd_vel_override_until = 0.0

        self.get_logger().info(
            f"⚙️ PWM profile loaded: forward={self.fixed_pwm_forward}, backward={self.fixed_pwm_backward}, turn={self.fixed_pwm_turn}"
        )

        self.nav2_ready = not self.require_nav2_active
        self.nav2_state_log_interval = 2.0
        self.last_nav2_state_log_time = 0.0
        self.last_nav2_ready = self.nav2_ready
        self.nav2_state_response_timeout = 1.0
        self.nav2_clients = {
            'controller_server': self.create_client(GetState, '/controller_server/get_state'),
            'planner_server': self.create_client(GetState, '/planner_server/get_state'),
            'bt_navigator': self.create_client(GetState, '/bt_navigator/get_state')
        }
        self.nav2_state_futures = {}
        self.nav2_state_cache = {}
        self.nav2_state_update_time = {}
        if self.require_nav2_active:
            check_interval = max(0.5, self.nav2_state_check_interval)
            self.create_timer(check_interval, self._check_nav2_state)
            self.get_logger().info(f"🧭 Nav2 state checker enabled (interval: {check_interval}s)")
        else:
            self.get_logger().info("🧭 Nav2 state checking disabled")

        # Try to open serial port
        self.ser = None
        # IMPORTANT: avoid cross-family fallback (e.g., grabbing LiDAR /dev/ttyUSB* when Arduino expected on /dev/ttyACM*)
        if serial_port.startswith('/dev/ttyACM'):
            ports_to_try = [serial_port] + sorted(glob.glob('/dev/ttyACM*'))
        elif serial_port.startswith('/dev/ttyUSB'):
            ports_to_try = [serial_port, '/dev/ttyUSB0', '/dev/ttyUSB1']
        else:
            # For custom paths (/dev/arduino), keep explicit path first, then any ACM devices
            ports_to_try = [serial_port] + sorted(glob.glob('/dev/ttyACM*'))
        self.ports_to_try = list(dict.fromkeys(ports_to_try))
        self.serial_error_streak = 0
        self.last_serial_error_log_time = 0.0
        self.last_serial_reconnect_time = 0.0
        self.last_motor_log_time = 0.0
        self.last_motor_log_cmd = None
        for port in ports_to_try:
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(1.0)  # Wait for Arduino reset (reduced from 2s)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f'✅ Arduino connected on {port}')
                break
            except Exception as e:
                self.get_logger().debug(f'Failed to connect to {port}: {e}')
                pass

        self.serial_disabled = False
        if self.ser is None:
            self.serial_disabled = True
            self.get_logger().error('❌ Failed to connect to Arduino - running without serial (odom only)')
        else:
            self.get_logger().info(f'✅ Serial port open: {self.ser.port}, baudrate: {self.ser.baudrate}')

        # Enable motors
        if not self.serial_disabled:
            try:
                bytes_written = self.ser.write(b'START\n')
                self.get_logger().info(f'📤 Sent START command: {bytes_written} bytes')
                time.sleep(0.1)
                self.get_logger().info('✅ Motors ENABLED')

                # Disable Arduino's local obstacle avoidance (ROS2 handles it)
                bytes_written = self.ser.write(b'AUTO:OFF\n')
                self.get_logger().info(f'📤 Sent AUTO:OFF command: {bytes_written} bytes')
                time.sleep(0.1)
                self.get_logger().info('✅ Arduino local avoidance DISABLED')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Arduino: {e}')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.ultrasonic_pub = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        self.ultrasonic_range_pub = self.create_publisher(Range, '/ultrasonic_range', 10)
        self.safety_stop_pub = self.create_publisher(Float32, '/safety_stop', 10)

        self.odom_pub = None
        self.tf_broadcaster = None
        self.odom_fallback_active = False  # Track if using fallback odometry
        # Always initialize shutdown and serial locks, regardless of odom publishing
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False
        self._serial_lock = threading.Lock()  # Thread safety for serial access

        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 106)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = 0.0
            self.last_time = self.get_clock().now()
            self.create_timer(1.0 / max(1.0, self.odom_rate), self._publish_odom)
            self.get_logger().info('✅ Odometry publisher initialized')

        # Start serial reader thread to parse Arduino data
        self.serial_reading = True
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

        # Stuck detection state (uses Arduino motor feedback)
        self.last_cmd_pwm_left = 0
        self.last_cmd_pwm_right = 0
        self.last_cmd_time = 0.0
        self.last_motor_speed_left = None
        self.last_motor_speed_right = None
        self.last_motor_speed_time = 0.0
        self.stuck_start_time = None
        self.stuck_recovery_active = False
        self.stuck_recovery_end_time = 0.0
        self.safety_override_until = 0.0
        self.safety_override_active = False
        self.rear_blocked_until = 0.0
        self.last_rear_distance = float('inf')
        self.front_blocked_until = 0.0
        self.last_front_distance = float('inf')
        self.any_obstacle_blocked_until = 0.0
        self.last_any_obstacle_distance = float('inf')
        self.last_rear_block_log_time = 0.0
        self.rear_block_log_interval = 1.0
        self.last_front_block_log_time = 0.0
        self.last_scan_time = 0.0
        self.last_nav2_block_log_time = 0.0
        self.nav2_block_log_interval = 2.0
        self.last_cmd_sign = 0
        self.last_dir_change_time = 0.0
        self.last_flip_guard_log_time = 0.0
        self.flip_guard_log_interval = 1.0
        self.last_scan_stale_log_time = 0.0
        self.scan_stale_log_interval = 1.0
        self.last_wall_debug_log_time = 0.0
        self.escape_turn_dir = 1.0
        self.last_escape_turn_toggle_time = 0.0
        self.safety_stop_obstacle_hits = 0
        self.safety_stop_clear_hits = 0
        self.last_safety_stop_brake_time = 0.0
        self.last_odom_feedback_gate_log_time = 0.0
        self.odom_feedback_gate_log_interval = 1.0

        if self.enable_stuck_recovery:
            self.create_timer(0.1, self._stuck_recovery_loop)

        # Safety stop override loop (ultrasonic emergency)
        self.create_timer(0.05, self._safety_override_loop)

        self.get_logger().info('🚀 Arduino Motor Bridge initialized')

        # Ensure motors stop on shutdown
        atexit.register(self._shutdown_motors)

    def _open_serial_connection(self):
        """Try to open Arduino serial on allowed ports and initialize controller state."""
        for port in self.ports_to_try:
            try:
                self.ser = serial.Serial(port, self.get_parameter('baud_rate').value, timeout=1)
                time.sleep(0.5)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().warn(f'🔌 Reconnected Arduino serial on {port}')
                try:
                    self.ser.write(b'START\n')
                    time.sleep(0.05)
                    self.ser.write(b'AUTO:OFF\n')
                    self.ser.write(b'MOTOR:0,0\n')
                except Exception as init_err:
                    self.get_logger().warn(f'⚠️ Reconnect init command failed: {init_err}')
                self.serial_disabled = False
                self.serial_error_streak = 0
                return True
            except Exception:
                continue
        self.ser = None
        self.serial_disabled = True
        return False

    def imu_cb(self, msg):
        # Extract yaw from quaternion
        import math
        q = msg.orientation
        # Normalize quaternion to avoid drift from invalid IMU data
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-6:
            return
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_quat = q
        self.imu_last_stamp = msg.header.stamp
    
    def cmd_vel_cb(self, msg: Twist):
        self._handle_cmd_vel(msg, source='cmd_vel')

    def _handle_cmd_vel(self, msg: Twist, source: str):
        if self.require_nav2_active and not self.nav2_ready:
            now = time.time()
            if now - self.last_nav2_block_log_time >= self.nav2_block_log_interval:
                self.last_nav2_block_log_time = now
                self.get_logger().warn(f"🛑 Nav2 not active; suppressing {source}")
            return
        # Safety override: ignore normal commands while backing up
        if self.enable_safety_override and self.safety_override_active:
            return

        if self.stuck_recovery_active:
            return

        # Any /cmd_vel message takes priority for a short window
        self.cmd_vel_override_until = time.time() + self.cmd_vel_override_duration
        
        linear = msg.linear.x      # m/s
        angular = msg.angular.z    # rad/s

        # Explicit speed limits for motor safety/smoothness
        linear = max(-self.max_linear_speed_mps, min(self.max_linear_speed_mps, linear))
        angular = max(-self.max_angular_speed_radps, min(self.max_angular_speed_radps, angular))

        # Deadband to suppress tiny oscillating commands that cause motor chatter/noise
        if abs(linear) < self.velocity_deadband:
            linear = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0
        elif linear == 0.0:
            angular *= self.turn_angular_scale

        now = time.time()
        scan_stale = False
        scan_age = now - self.last_scan_time if self.last_scan_time > 0.0 else float('inf')
        if self.last_scan_time > 0.0 and scan_age > self.scan_stale_timeout:
            scan_stale = True
            linear = 0.0
            angular = 0.0
            if now - self.last_scan_stale_log_time >= self.scan_stale_log_interval:
                self.last_scan_stale_log_time = now
                self.get_logger().warn(
                    f"🛑 Scan stale ({scan_age:.2f}s); suppressing {source}"
                )

        # When a front obstacle is latched, allow command sign changes so
        # obstacle escape (backing/turning) is not blocked by flip guard.
        front_escape_active = (not scan_stale) and self._front_blocked()

        if linear != 0.0:
            sign = 1 if linear > 0.0 else -1
            if self.last_cmd_sign != 0 and sign != self.last_cmd_sign:
                if ((now - self.last_dir_change_time) < self.flip_guard_time) and (not front_escape_active):
                    linear = 0.0
                    # Keep commanded turn when present so obstacle escape rotations
                    # are not suppressed by linear direction flip protection.
                    if abs(angular) < self.angular_deadband:
                        angular = 0.0
                    if now - self.last_flip_guard_log_time >= self.flip_guard_log_interval:
                        self.last_flip_guard_log_time = now
                        self.get_logger().warn(
                            f"🛑 Direction flip guard active; suppressing linear {source}"
                        )
            if sign != self.last_cmd_sign:
                self.last_dir_change_time = now
                self.last_cmd_sign = sign

        if (not scan_stale) and self.force_backward_on_zero_cmd and linear == 0.0 and angular == 0.0:
            linear = -abs(self.auto_backup_speed_mps)
            angular = 0.0

        auto_backup = False
        can_front_backup = (
            (not scan_stale) and
            (not self._rear_blocked()) and
            math.isfinite(self.last_rear_distance) and
            (self.last_rear_distance > self.rear_backup_min_distance)
        )
        if (not scan_stale) and self._front_blocked() and linear > 0.0:
            if self._rear_blocked():
                # Front and rear blocked: allow in-place turning instead of backing up.
                linear = 0.0
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"🌀 Front+rear blocked (front {self.last_front_distance:.2f}m, "
                        f"rear {self.last_rear_distance:.2f}m); allowing turn only"
                    )
            elif abs(angular) > self.angular_deadband and can_front_backup:
                # Back away from front obstacle while preserving turn direction.
                linear = -abs(self.front_obstacle_backup_speed_mps)
                angular *= self.front_obstacle_backup_turn_scale
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"⬇️ Front blocked at {self.last_front_distance:.2f}m; backing while turning"
                    )
            else:
                # Default escape: back away if rear is clear, else rotate in place.
                angular = self._select_escape_turn(angular)
                if can_front_backup:
                    linear = -abs(self.front_obstacle_backup_speed_mps)
                    angular *= self.front_obstacle_backup_turn_scale
                    auto_backup = True
                else:
                    linear = 0.0
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"⬇️ Front blocked at {self.last_front_distance:.2f}m; {'backing off' if can_front_backup else 'rotate-only'}"
                    )

        # Any-angle near-field safety gate: stop forward if anything is dangerously close,
        # even when front/rear sector classification is imperfect.
        if linear > 0.0 and self._any_obstacle_blocked():
            angular = self._select_escape_turn(angular)
            if can_front_backup:
                linear = -abs(self.front_obstacle_backup_speed_mps)
                angular *= self.front_obstacle_backup_turn_scale
            else:
                linear = 0.0
            if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                self.last_front_block_log_time = now
                self.get_logger().warn(
                    f"🛑 Forward blocked by near obstacle at {self.last_any_obstacle_distance:.2f}m (any-angle gate, {'backup' if can_front_backup else 'rotate'})"
                )

        # Hard safety gate: if front obstacle is latched, never allow forward command.
        if linear > 0.0 and self._front_blocked():
            angular = self._select_escape_turn(angular)
            if can_front_backup:
                linear = -abs(self.front_obstacle_backup_speed_mps)
                angular *= self.front_obstacle_backup_turn_scale
            else:
                linear = 0.0
            if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                self.last_front_block_log_time = now
                self.get_logger().warn(
                    f"🛑 Forward blocked by latched front obstacle at {self.last_front_distance:.2f}m ({'backup' if can_front_backup else 'rotate'})"
                )

        if linear < 0.0 and (now - self.last_wall_debug_log_time) >= self.wall_debug_log_interval:
            self.last_wall_debug_log_time = now
            self.get_logger().info(
                "🧱 Wall check: "
                f"front={self.last_front_distance:.2f}m blocked={self._front_blocked()}, "
                f"rear={self.last_rear_distance:.2f}m blocked={self._rear_blocked()}"
            )

        rear_known = math.isfinite(self.last_rear_distance)
        rear_scan_recent = (self.last_scan_time > 0.0 and (now - self.last_scan_time) <= self.scan_stale_timeout)
        if linear < 0.0 and (scan_stale or not rear_scan_recent or not rear_known):
            linear = 0.0
            if self._front_blocked():
                angular = self._select_escape_turn(angular)
            else:
                angular = 0.0
            if now - self.last_rear_block_log_time >= self.rear_block_log_interval:
                self.last_rear_block_log_time = now
                self.get_logger().warn(
                    "🛑 Backward suppressed: rear clearance unknown or scan stale"
                )

        if not self.allow_backward_when_rear_blocked and linear < 0.0 and self._rear_blocked():
            linear = 0.0
            if self._front_blocked():
                angular = self._select_escape_turn(angular)
            else:
                angular = 0.0
            if now - self.last_rear_block_log_time >= self.rear_block_log_interval:
                self.last_rear_block_log_time = now
                self.get_logger().warn(
                    f"🛑 Rear blocked at {self.last_rear_distance:.2f}m; suppressing backward cmd_vel"
                )

        if linear < 0.0 and self.last_rear_distance <= self.rear_backup_min_distance:
            linear = 0.0
            angular = 0.0
            if now - self.last_rear_block_log_time >= self.rear_block_log_interval:
                self.last_rear_block_log_time = now
                self.get_logger().warn(
                    f"🛑 Rear distance {self.last_rear_distance:.2f}m <= backup min "
                    f"{self.rear_backup_min_distance:.2f}m; suppressing backward cmd_vel"
                )

        # Store for odom integration
        if self.publish_odom:
            self.last_cmd_linear = linear
            self.last_cmd_angular = angular
        
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        scale_left = self.max_speed_forward if v_left >= 0.0 else self.max_speed_backward
        scale_right = self.max_speed_forward if v_right >= 0.0 else self.max_speed_backward
        pwm_left = int(v_left * (scale_left / 0.5))
        pwm_right = int(v_right * (scale_right / 0.5))
        
        # Enforce minimum PWM by motion mode to avoid stall/buzz while keeping control.
        if linear > self.velocity_deadband:
            MIN_PWM = self.min_pwm_forward
        elif linear < -self.velocity_deadband:
            MIN_PWM = self.min_pwm_backward
        elif abs(angular) > self.angular_deadband:
            MIN_PWM = self.min_pwm_turn
        else:
            MIN_PWM = self.min_pwm
        if pwm_left != 0:
            sign = 1 if pwm_left > 0 else -1
            pwm_left = sign * max(MIN_PWM, abs(pwm_left))
        if pwm_right != 0:
            sign = 1 if pwm_right > 0 else -1
            pwm_right = sign * max(MIN_PWM, abs(pwm_right))
        
        # Clamp to valid PWM range
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))

        # Fixed-output motor profiles for predictable behavior and logging.
        fwd_pwm = max(0, min(255, abs(self.fixed_pwm_forward)))
        back_pwm = max(0, min(255, abs(self.fixed_pwm_backward)))
        turn_pwm = max(0, min(255, abs(self.fixed_pwm_turn)))
        if linear > self.velocity_deadband and abs(angular) <= self.angular_deadband:
            pwm_left = fwd_pwm
            pwm_right = fwd_pwm
        elif linear < -self.velocity_deadband and abs(angular) <= self.angular_deadband:
            pwm_left = -back_pwm
            pwm_right = -back_pwm
        elif abs(linear) <= self.velocity_deadband and abs(angular) > self.angular_deadband:
            if angular > 0.0:
                pwm_left = -turn_pwm
                pwm_right = turn_pwm
            else:
                pwm_left = turn_pwm
                pwm_right = -turn_pwm
        elif linear > self.velocity_deadband and abs(angular) > self.angular_deadband:
            # FORCE: Both motors get same PWM for forward motion, regardless of angular
            pwm_left = fwd_pwm
            pwm_right = fwd_pwm
        elif linear < -self.velocity_deadband and abs(angular) > self.angular_deadband:
            if angular > 0.0:
                pwm_left = -turn_pwm
                pwm_right = -back_pwm
            else:
                pwm_left = -back_pwm
                pwm_right = -turn_pwm

        # Keep in-place left/right turns controlled and symmetric.
        if linear == 0.0 and abs(angular) > self.angular_deadband:
            turn_limit = max(self.min_pwm, self.turn_pwm_limit)
            pwm_left = max(-turn_limit, min(turn_limit, pwm_left))
            pwm_right = max(-turn_limit, min(turn_limit, pwm_right))

        # Suppress tiny PWM dithering around same command to reduce motor noise
        if (
            abs(pwm_left - self.last_cmd_pwm_left) < self.pwm_change_threshold and
            abs(pwm_right - self.last_cmd_pwm_right) < self.pwm_change_threshold
        ):
            pwm_left = self.last_cmd_pwm_left
            pwm_right = self.last_cmd_pwm_right

        self.last_cmd_pwm_left = pwm_left
        self.last_cmd_pwm_right = pwm_right
        self.last_cmd_time = time.time()
        
        self._send_motor_pwm(pwm_left, pwm_right, source=source)

    def cmd_vel_nav_cb(self, msg: Twist):
        """Handle cmd_vel_nav only when no override is active"""
        if time.time() < self.cmd_vel_override_until:
            return  # Ignore nav commands during override window
        self._handle_cmd_vel(msg, source='cmd_vel_nav')

    def scan_cb(self, msg: LaserScan):
        now = time.time()
        self.last_scan_time = now
        min_rear = float('inf')
        min_front = float('inf')
        min_any = float('inf')
        front_hit_count = 0
        rear_hit_count = 0
        any_hit_count = 0
        angle = msg.angle_min
        for distance in msg.ranges:
            if distance < msg.range_min or distance > msg.range_max:
                angle += msg.angle_increment
                continue
            if distance < min_any:
                min_any = distance
            if distance <= self.any_obstacle_stop_distance:
                any_hit_count += 1
            in_front_zone = (-self.front_obstacle_half_angle <= angle <= self.front_obstacle_half_angle)
            in_rear_zone = (abs(abs(angle) - math.pi) <= self.rear_obstacle_half_angle)

            if in_front_zone:
                if distance < min_front:
                    min_front = distance
                if distance <= self.front_stop_distance:
                    front_hit_count += 1
            if in_rear_zone:
                if distance < min_rear:
                    min_rear = distance
                if distance <= self.rear_stop_distance:
                    rear_hit_count += 1
            angle += msg.angle_increment

        if min_front < float('inf'):
            self.last_front_distance = min_front
            if min_front <= self.front_stop_distance and front_hit_count >= max(1, self.front_block_min_hits):
                self.front_blocked_until = now + self.front_stop_hold_time

        if min_rear < float('inf'):
            self.last_rear_distance = min_rear
            if min_rear <= self.rear_stop_distance and rear_hit_count >= max(1, self.rear_block_min_hits):
                self.rear_blocked_until = now + self.rear_stop_hold_time

        if min_any < float('inf'):
            self.last_any_obstacle_distance = min_any
            if min_any <= self.any_obstacle_stop_distance and any_hit_count >= max(1, self.any_obstacle_min_hits):
                self.any_obstacle_blocked_until = now + self.any_obstacle_stop_hold_time

        if self.swap_lidar_front_back:
            self.last_front_distance, self.last_rear_distance = self.last_rear_distance, self.last_front_distance
            self.front_blocked_until, self.rear_blocked_until = self.rear_blocked_until, self.front_blocked_until

    def _rear_blocked(self) -> bool:
        return time.time() < self.rear_blocked_until

    def _front_blocked(self) -> bool:
        return time.time() < self.front_blocked_until

    def _any_obstacle_blocked(self) -> bool:
        return time.time() < self.any_obstacle_blocked_until

    def _select_escape_turn(self, commanded_angular: float) -> float:
        if abs(commanded_angular) > self.angular_deadband:
            return self.escape_turn_speed if commanded_angular > 0.0 else -self.escape_turn_speed

        now = time.time()
        if (now - self.last_escape_turn_toggle_time) >= self.escape_turn_toggle_interval:
            self.escape_turn_dir *= -1.0
            self.last_escape_turn_toggle_time = now
        return self.escape_turn_speed * self.escape_turn_dir

    def _check_nav2_state(self):
        if not self.require_nav2_active:
            self.nav2_ready = True
            return

        now = time.time()
        state_map = {
            0: 'UNKNOWN',
            1: 'UNCONFIGURED',
            2: 'INACTIVE',
            3: 'ACTIVE',
            4: 'FINALIZED'
        }

        # Fire async requests without blocking the executor.
        for name, client in self.nav2_clients.items():
            if not client.service_is_ready():
                continue
            if self.nav2_state_futures.get(name) is None:
                try:
                    future = client.call_async(GetState.Request())
                    self.nav2_state_futures[name] = future
                    future.add_done_callback(lambda f, n=name: self._handle_nav2_state_result(n, f))
                except Exception as e:
                    self.get_logger().error(f"🧭 Error sending {name} state request: {e}")

        all_active = True
        active_count = 0
        status_lines = []

        for name in self.nav2_clients.keys():
            if not self.nav2_clients[name].service_is_ready():
                all_active = False
                status_lines.append(f"{name}: service_not_ready")
                continue

            if name not in self.nav2_state_cache:
                all_active = False
                status_lines.append(f"{name}: pending")
                continue

            last_update = self.nav2_state_update_time.get(name, 0.0)
            if now - last_update > self.nav2_state_response_timeout:
                all_active = False
                status_lines.append(f"{name}: timeout")
                continue

            state_id = self.nav2_state_cache.get(name)
            if state_id == 3:
                active_count += 1
            else:
                all_active = False
                status_lines.append(f"{name}: {state_map.get(state_id, str(state_id))}")

        self.nav2_ready = all_active

        if self.nav2_ready != self.last_nav2_ready:
            self.last_nav2_ready = self.nav2_ready
            state = 'ACTIVE' if self.nav2_ready else 'INACTIVE'
            self.get_logger().info(f"🧭 Nav2 state changed: {state} ({active_count}/3 nodes active)")

        if not self.nav2_ready:
            if now - self.last_nav2_state_log_time >= self.nav2_state_log_interval:
                self.last_nav2_state_log_time = now
                if status_lines:
                    status = '; '.join(status_lines)
                    self.get_logger().warn(f"🧭 Nav2 not ready: {status}")
                else:
                    self.get_logger().warn(f"🧭 Nav2 check in progress... ({active_count}/3 active)")

    def _handle_nav2_state_result(self, name: str, future):
        try:
            response = future.result()
            self.nav2_state_cache[name] = response.current_state.id
            self.nav2_state_update_time[name] = time.time()
        except Exception as e:
            self.get_logger().error(f"🧭 Error getting {name} state: {e}")
            self.nav2_state_cache[name] = None
            self.nav2_state_update_time[name] = time.time()
        finally:
            self.nav2_state_futures[name] = None

    def _publish_odom(self):

        # DEBUG: Confirm timer is firing
        self.get_logger().debug('🟢 _publish_odom timer called')
        if not self.publish_odom:
            self.get_logger().debug('🔴 publish_odom is False, skipping')
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:  # Skip if too long (initialization or clock jump)
            self.last_time = now
            return

        v_cmd = self.last_cmd_linear
        w_cmd = self.last_cmd_angular
        v = v_cmd
        w = w_cmd
        freeze_pose_update = False

        if self.odom_feedback_gate_enabled:
            feedback_recent = (time.time() - self.last_motor_speed_time) <= 0.5
            if feedback_recent and self.last_motor_speed_left is not None and self.last_motor_speed_right is not None:
                motors_stationary = (
                    abs(self.last_motor_speed_left) <= self.odom_stationary_speed_threshold and
                    abs(self.last_motor_speed_right) <= self.odom_stationary_speed_threshold
                )
                cmd_demands_linear_motion = abs(v_cmd) > self.velocity_deadband
                cmd_demands_rotation = abs(w_cmd) > self.angular_deadband

                # Freeze pose only when linear motion is commanded but feedback says stationary.
                # Do not freeze on pure rotation commands, to preserve heading updates for SLAM.
                if motors_stationary and cmd_demands_linear_motion and not cmd_demands_rotation:
                    v = 0.0
                    w = 0.0
                    freeze_pose_update = self.odom_freeze_pose_when_stationary
                    now_wall = time.time()
                    if (now_wall - self.last_odom_feedback_gate_log_time) >= self.odom_feedback_gate_log_interval:
                        self.last_odom_feedback_gate_log_time = now_wall
                        self.get_logger().warn(
                            "🧭 Odom gate: blocked linear motion detected; suppressing cmd-based odom integration"
                        )
        # Use IMU yaw only while moving; when stationary-gated, freeze pose to avoid IMU jitter smear.
        if not freeze_pose_update:
            if self.imu_yaw is not None:
                dx = v * math.cos(self.imu_yaw) * dt
                dy = v * math.sin(self.imu_yaw) * dt
                self.x += dx
                self.y += dy
                self.yaw = self.imu_yaw
            else:
                self.x += v * math.cos(self.yaw) * dt
                self.y += v * math.sin(self.yaw) * dt
                self.yaw += w * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Use slightly backdated timestamp to avoid "future" TF errors
        stamp = now - rclpy.duration.Duration(seconds=0.002)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # Use IMU quaternion only when not stationary-gated to avoid orientation jitter at standstill.
        if self.imu_quat is not None and not freeze_pose_update:
            t.transform.rotation = self.imu_quat
        else:
            t.transform.rotation.z = math.sin(self.yaw / 2.0)
            t.transform.rotation.w = math.cos(self.yaw / 2.0)

        # DEBUG: Show TF being published
        self.get_logger().debug(f'Publishing TF: {t.header.frame_id} -> {t.child_frame_id} at ({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, {self.yaw:.3f})')
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        if self.imu_quat is not None and not freeze_pose_update:
            odom.pose.pose.orientation = self.imu_quat
        else:
            odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # DEBUG: Show odometry being published
        self.get_logger().debug(f'Publishing Odometry: ({odom.pose.pose.position.x:.3f}, {odom.pose.pose.position.y:.3f}, {self.yaw:.3f}) v={v:.3f} w={self.last_cmd_angular:.3f}')
        self.odom_pub.publish(odom)

        # Track if we're using fallback (no serial feedback received)
        if self.serial_disabled and not self.odom_fallback_active:
            self.odom_fallback_active = True
            self.get_logger().warn('⚠️ Using FALLBACK odometry (no serial feedback from Arduino)')
        elif not self.serial_disabled and self.odom_fallback_active:
            self.odom_fallback_active = False
            self.get_logger().info('✅ Serial communication restored')

        # DIAGNOSTIC: keep at debug to avoid flooding logs and starving controller loops
        if abs(v) > 0.001 or abs(self.last_cmd_angular) > 0.001:
            self.get_logger().debug(f'📍 TF published: odom->base_footprint at ({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f}) v={v:.3f} w={w:.3f}')

        self.last_time = now
    
    def _get_action_name(self, left, right):
        if left == 0 and right == 0:
            return '⏸️  STOPPED'
        elif left > 0 and right > 0:
            return '⬆️  FORWARD'
        elif left < 0 and right < 0:
            return '⬇️  BACKWARD'
        elif left < 0 and right > 0:
            return '↺  TURN LEFT'
        elif left > 0 and right < 0:
            return '↻  TURN RIGHT'
        elif left > 0 and right == 0:
            return '⤴️  PIVOT LEFT'
        elif left == 0 and right > 0:
            return '⤵️  PIVOT RIGHT'
        elif left < right:
            return '↖️  FORWARD + SLIGHT LEFT'
        elif right < left:
            return '↗️  FORWARD + SLIGHT RIGHT'
        elif abs(left - right) > 50:
            return '↺  TURN'
        return '↔️  MIXED'

    def _send_motor_pwm(self, pwm_left, pwm_right, action_override=None, log_level='info', source='unknown'):
        try:
            if self.ser is None:
                self.get_logger().debug('🚫 Serial disabled - motor command ignored')
                return
            cmd = f"MOTOR:{pwm_left},{pwm_right}\n"
            with self._serial_lock:
                try:
                    bytes_written = self.ser.write(cmd.encode())
                    if bytes_written == 0:
                        self.get_logger().error(f'⚠️ Serial write returned 0 bytes for: {cmd.strip()}')
                    self.ser.flush()
                except (OSError, serial.SerialException) as se:
                    self.get_logger().error(f'❌ Serial write exception: {se}')
                    self.serial_disabled = True
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    return

            action = action_override or self._get_action_name(pwm_left, pwm_right)
            msg = f'🚀 {action} | MOTOR:{pwm_left},{pwm_right} | SRC:{source}'
            now = time.time()
            cmd_key = (pwm_left, pwm_right, action)
            should_log_info = (now - self.last_motor_log_time) >= self.motor_log_interval
            if log_level == 'warn':
                self.get_logger().warn(msg)
            elif log_level == 'error':
                self.get_logger().error(msg)
            else:
                if should_log_info:
                    self.get_logger().info(msg)
                    self.last_motor_log_time = now
                    self.last_motor_log_cmd = cmd_key
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _stuck_recovery_loop(self):
        if self.ser is None:
            return

        now = time.time()

        if self.stuck_recovery_active:
            if now >= self.stuck_recovery_end_time:
                self.stuck_recovery_active = False
                self.stuck_start_time = None
                if self.last_rear_distance <= self.rear_backup_min_distance:
                    self._send_motor_pwm(
                        0,
                        0,
                        action_override='🛑 STOPPED (REAR LIMIT)',
                        log_level='warn'
                    )
                else:
                    self._send_motor_pwm(
                        -abs(self.stuck_backup_pwm),
                        -abs(self.stuck_backup_pwm),
                        action_override='⬇️  BACKWARD (STUCK EXIT)',
                        log_level='warn'
                    )
            else:
                if self.last_rear_distance <= self.rear_backup_min_distance:
                    self.stuck_recovery_active = False
                    self.stuck_start_time = None
                    self._send_motor_pwm(
                        0,
                        0,
                        action_override='🛑 STOPPED (REAR LIMIT)',
                        log_level='warn'
                    )
                else:
                    self._send_motor_pwm(
                        -abs(self.stuck_backup_pwm),
                        -abs(self.stuck_backup_pwm),
                        action_override='⬇️  BACKWARD (STUCK RECOVERY)',
                        log_level='warn'
                    )
            return

        if self.last_motor_speed_left is None or self.last_motor_speed_right is None:
            return
        if (now - self.last_motor_speed_time) > 0.5:
            return

        cmd_active = (abs(self.last_cmd_pwm_left) >= 120 or abs(self.last_cmd_pwm_right) >= 120)
        speeds_low = (abs(self.last_motor_speed_left) <= self.stuck_speed_threshold and
                      abs(self.last_motor_speed_right) <= self.stuck_speed_threshold)

        if cmd_active and speeds_low:
            if self.stuck_start_time is None:
                self.stuck_start_time = now
            elif (now - self.stuck_start_time) >= self.stuck_time:
                self.stuck_recovery_active = True
                self.stuck_recovery_end_time = now + self.stuck_backup_time
                self.get_logger().warn(
                    f"⚠️ Motors appear stuck (cmd PWM {self.last_cmd_pwm_left},{self.last_cmd_pwm_right} | "
                    f"speed {self.last_motor_speed_left},{self.last_motor_speed_right}). Backing up."
                )
        else:
            self.stuck_start_time = None

    def _safety_override_loop(self):
        if self.ser is None:
            return

        if self.require_nav2_active and not self.nav2_ready:
            self.safety_override_active = False
            self.safety_override_until = 0.0
            self._send_motor_pwm(
                0,
                0,
                action_override='🛑 STOPPED (NAV2 INACTIVE)',
                log_level='warn'
            )
            return

        if not self.enable_safety_override:
            return

        now = time.time()
        if now < self.safety_override_until:
            if self.last_rear_distance <= self.rear_backup_min_distance:
                self.safety_override_active = False
                self.safety_override_until = 0.0
                self._send_motor_pwm(
                    0,
                    0,
                    action_override='🛑 STOPPED (REAR LIMIT)',
                    log_level='warn'
                )
            else:
                self._send_motor_pwm(
                    -abs(self.safety_stop_backup_pwm),
                    -abs(self.safety_stop_backup_pwm),
                    action_override='⬇️  BACKWARD (SAFETY OVERRIDE)',
                    log_level='warn'
                )
            return

        if self.safety_override_active:
            self.safety_override_active = False
            if self.last_rear_distance <= self.rear_backup_min_distance:
                self._send_motor_pwm(
                    0,
                    0,
                    action_override='🛑 STOPPED (REAR LIMIT)',
                    log_level='warn'
                )
            else:
                self._send_motor_pwm(
                    -abs(self.safety_stop_backup_pwm),
                    -abs(self.safety_stop_backup_pwm),
                    action_override='⬇️  BACKWARD (SAFETY EXIT)'
                )
    
    def _serial_reader(self):
        """Background thread to read and parse Arduino CSV data"""
        while self.serial_reading:
            try:
                if self.ser is None or self.serial_disabled:
                    now = time.time()
                    if (now - self.last_serial_reconnect_time) >= self.serial_reconnect_interval:
                        self.last_serial_reconnect_time = now
                        if self._open_serial_connection():
                            self.get_logger().warn('🔌 Arduino serial reconnected after startup failure')
                        elif (now - self.last_serial_error_log_time) >= self.serial_error_log_interval:
                            self.last_serial_error_log_time = now
                            self.get_logger().warn('⏳ Arduino serial not available yet; retrying...')
                    time.sleep(0.1)
                    continue

                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Parse SAFETY_STOP format: SAFETY_STOP:1,distance_cm or SAFETY_STOP:0
                    if line.startswith('SAFETY_STOP'):
                        self.get_logger().warn(f'🚨 Arduino: {line}')
                        try:
                            if ',' in line:
                                # Format: SAFETY_STOP:1,16.24 - OBSTACLE DETECTED
                                parts = line.split(',')
                                if len(parts) >= 2:
                                    distance_cm = float(parts[1])
                                    distance_m = distance_cm / 100.0  # Convert cm to meters
                                    
                                    # Publish to both topics
                                    msg = Float32()
                                    msg.data = distance_m
                                    self.ultrasonic_pub.publish(msg)
                                    self._publish_ultrasonic_range(distance_m)
                                    self.safety_stop_pub.publish(msg)  # Explicit safety stop with distance
                                    if distance_m <= self.safety_stop_trigger_distance:
                                        self.safety_stop_obstacle_hits += 1
                                        self.safety_stop_clear_hits = 0
                                        if self.safety_stop_obstacle_hits >= self.safety_stop_confirm_count:
                                            now = time.time()
                                            # Treat ultrasonic emergency as FRONT stop latch.
                                            self.front_blocked_until = max(
                                                self.front_blocked_until,
                                                now + self.safety_stop_front_latch_time
                                            )
                                            self.last_front_distance = min(self.last_front_distance, distance_m)
                                            self.get_logger().error(
                                                f'🚨 SAFETY_STOP! Published /safety_stop: {distance_cm}cm ({distance_m:.4f}m)'
                                            )

                                            # Immediate stop command so we don't wait for next cmd_vel cycle.
                                            if (now - self.last_safety_stop_brake_time) >= 0.2:
                                                self.last_safety_stop_brake_time = now
                                                self.last_cmd_pwm_left = 0
                                                self.last_cmd_pwm_right = 0
                                                self.last_cmd_time = now
                                                self._send_motor_pwm(
                                                    0,
                                                    0,
                                                    action_override='🛑 STOPPED (ULTRASONIC FRONT)',
                                                    log_level='warn',
                                                    source='safety_stop'
                                                )

                                            # Activate safety override backup (local motor control)
                                            if self.enable_safety_override:
                                                self.safety_override_until = max(
                                                    self.safety_override_until,
                                                    now + self.safety_stop_backup_time
                                                )
                                                if not self.safety_override_active:
                                                    self.safety_override_active = True
                                                    self.get_logger().warn(
                                                        f'🚨 SAFETY OVERRIDE: backing up for {self.safety_stop_backup_time:.1f}s'
                                                    )
                                    else:
                                        # Ignore noisy/too-far safety spikes
                                        self.safety_stop_obstacle_hits = 0
                                        self.safety_stop_clear_hits += 1
                            else:
                                # Format: SAFETY_STOP:0 - OBSTACLE CLEARED
                                msg = Float32()
                                msg.data = 999.0  # Sentinel value: no obstacle
                                self.safety_stop_pub.publish(msg)
                                self.safety_stop_clear_hits += 1
                                self.safety_stop_obstacle_hits = 0
                                if self.safety_stop_clear_hits >= self.safety_stop_clear_confirm_count:
                                    self.get_logger().warn(f'✅ SAFETY_STOP:0 - Obstacle cleared')
                        except (ValueError, IndexError) as e:
                            self.get_logger().error(f'Failed to parse SAFETY_STOP: {line} - {e}')
                    # Parse CSV data: ax,ay,az,gx,gy,gz,distance,motorA_speed,motorB_speed
                    elif line and ',' in line and not line.startswith('ACK'):
                        parts = line.split(',')
                        if len(parts) >= 7:
                            try:
                                distance_raw = float(parts[6])  # Index 6 is distance in cm
                                # Arduino sends in cm, convert to meters
                                if distance_raw > 1.0:
                                    distance_m = distance_raw / 100.0
                                else:
                                    distance_m = distance_raw
                                
                                msg = Float32()
                                msg.data = distance_m
                                self.ultrasonic_pub.publish(msg)
                                self._publish_ultrasonic_range(distance_m)
                            except (ValueError, IndexError):
                                pass  # Skip malformed lines

                            # Optional motor speed feedback (indices 7,8)
                            if len(parts) >= 9:
                                try:
                                    self.last_motor_speed_left = float(parts[7])
                                    self.last_motor_speed_right = float(parts[8])
                                    self.last_motor_speed_time = time.time()
                                except (ValueError, IndexError):
                                    pass
                    elif line.startswith('ACK'):
                        self.get_logger().debug(f'✓ {line}')
                self.serial_error_streak = 0
                
                time.sleep(0.01)  # Don't busy-wait
            except Exception as e:
                self.serial_error_streak += 1
                now = time.time()
                if (now - self.last_serial_error_log_time) >= self.serial_error_log_interval:
                    self.last_serial_error_log_time = now
                    self.get_logger().error(f'Serial read error ({self.serial_error_streak}): {e}')

                if self.serial_error_streak >= self.serial_max_error_streak and (now - self.last_serial_reconnect_time) >= self.serial_reconnect_interval:
                    self.last_serial_reconnect_time = now
                    try:
                        if self.ser is not None:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    if not self._open_serial_connection():
                        if (now - self.last_serial_error_log_time) >= self.serial_error_log_interval:
                            self.last_serial_error_log_time = now
                            self.get_logger().error('❌ Serial reconnect failed; will retry')
                time.sleep(0.1)

    def _shutdown_motors(self):
        """Send a STOP command to the Arduino on shutdown."""
        with self._shutdown_lock:
            if self._shutdown_done:
                return
            self._shutdown_done = True

            self.serial_reading = False
            if self.serial_thread.is_alive():
                self.serial_thread.join(timeout=0.2)

            try:
                if self.ser is not None:
                    # Send a few stop commands to ensure Arduino receives it
                    for _ in range(3):
                        self.ser.write(b"MOTOR:0,0\n")
                        self.ser.write(b"STOP\n")
                        time.sleep(0.05)
                    self.ser.close()
            except Exception:
                pass

    def _publish_ultrasonic_range(self, distance_m: float):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.ultrasonic_frame
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.ultrasonic_fov
        range_msg.min_range = self.ultrasonic_min_range
        range_msg.max_range = self.ultrasonic_max_range
        range_msg.range = max(self.ultrasonic_min_range, min(self.ultrasonic_max_range, distance_m))
        self.ultrasonic_range_pub.publish(range_msg)

    def destroy_node(self):
        self._shutdown_motors()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorBridge()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'context is not valid' not in str(e):
            node.get_logger().error(f'Arduino motor bridge error: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
