#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Range, LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.srv import GetState
import serial
import time
import threading
import atexit
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
        self.declare_parameter('min_pwm', 90)
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
        self.declare_parameter('enable_safety_override', True)
        self.declare_parameter('ultrasonic_frame', 'base_footprint')
        self.declare_parameter('ultrasonic_min_range', 0.02)
        self.declare_parameter('ultrasonic_max_range', 0.50)
        self.declare_parameter('ultrasonic_fov', 0.52)
        self.declare_parameter('rear_stop_distance', 0.40)
        self.declare_parameter('rear_backup_min_distance', 0.30)
        self.declare_parameter('rear_stop_hold_time', 0.6)
        self.declare_parameter('front_stop_distance', 0.35)
        self.declare_parameter('front_stop_hold_time', 0.5)
        self.declare_parameter('scan_stale_timeout', 1.2)
        self.declare_parameter('flip_guard_time', 0.15)
        self.declare_parameter('wall_debug_log_interval', 1.0)
        self.declare_parameter('swap_lidar_front_back', False)
        self.declare_parameter('escape_turn_speed', 0.6)
        self.declare_parameter('escape_turn_period', 2.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('force_backward_on_zero_cmd', False)
        self.declare_parameter('zero_cmd_forward_pwm', 110)
        self.declare_parameter('allow_backward_when_rear_blocked', False)
        self.declare_parameter('require_nav2_active', True)
        self.declare_parameter('nav2_state_check_interval', 1.0)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = int(self.get_parameter('max_speed').value)
        self.max_speed_forward = int(self.get_parameter('max_speed_forward').value)
        self.max_speed_backward = int(self.get_parameter('max_speed_backward').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
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
        self.enable_safety_override = self.get_parameter('enable_safety_override').value
        self.ultrasonic_frame = self.get_parameter('ultrasonic_frame').value
        self.ultrasonic_min_range = float(self.get_parameter('ultrasonic_min_range').value)
        self.ultrasonic_max_range = float(self.get_parameter('ultrasonic_max_range').value)
        self.ultrasonic_fov = float(self.get_parameter('ultrasonic_fov').value)
        self.rear_stop_distance = float(self.get_parameter('rear_stop_distance').value)
        self.rear_backup_min_distance = float(self.get_parameter('rear_backup_min_distance').value)
        self.rear_stop_hold_time = float(self.get_parameter('rear_stop_hold_time').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_stop_hold_time = float(self.get_parameter('front_stop_hold_time').value)
        self.scan_stale_timeout = float(self.get_parameter('scan_stale_timeout').value)
        self.flip_guard_time = float(self.get_parameter('flip_guard_time').value)
        self.wall_debug_log_interval = float(self.get_parameter('wall_debug_log_interval').value)
        self.swap_lidar_front_back = bool(self.get_parameter('swap_lidar_front_back').value)
        self.escape_turn_speed = float(self.get_parameter('escape_turn_speed').value)
        self.escape_turn_period = float(self.get_parameter('escape_turn_period').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.force_backward_on_zero_cmd = bool(self.get_parameter('force_backward_on_zero_cmd').value)
        self.zero_cmd_forward_pwm = int(self.get_parameter('zero_cmd_forward_pwm').value)
        self.allow_backward_when_rear_blocked = bool(self.get_parameter('allow_backward_when_rear_blocked').value)
        self.require_nav2_active = bool(self.get_parameter('require_nav2_active').value)
        self.nav2_state_check_interval = float(self.get_parameter('nav2_state_check_interval').value)
        self.cmd_vel_override_duration = 0.2
        self.cmd_vel_override_until = 0.0

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
        ports_to_try = [serial_port, '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
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
                self.get_logger().error(f'Failed to initialize Arduino: {e}', exc_info=True)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, '/servo_angle', self.servo_cb, 10)
        self.create_subscription(Float32, '/servo_command', self.servo_cmd_cb, 10)

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

        if self.enable_stuck_recovery:
            self.create_timer(0.1, self._stuck_recovery_loop)

        # Safety stop override loop (ultrasonic emergency)
        self.create_timer(0.05, self._safety_override_loop)

        self.get_logger().info('🚀 Arduino Motor Bridge initialized')

        # Ensure motors stop on shutdown
        atexit.register(self._shutdown_motors)

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

        now = time.time()
        if self.last_scan_time > 0.0 and (now - self.last_scan_time) > self.scan_stale_timeout:
            linear = 0.0
            angular = 0.0
            if now - self.last_scan_stale_log_time >= self.scan_stale_log_interval:
                self.last_scan_stale_log_time = now
                self.get_logger().warn(
                    f"🛑 Scan stale ({now - self.last_scan_time:.2f}s); suppressing {source}"
                )

        if linear != 0.0:
            sign = 1 if linear > 0.0 else -1
            if self.last_cmd_sign != 0 and sign != self.last_cmd_sign:
                if (now - self.last_dir_change_time) < self.flip_guard_time:
                    linear = 0.0
                    angular = 0.0
                    if now - self.last_flip_guard_log_time >= self.flip_guard_log_interval:
                        self.last_flip_guard_log_time = now
                        self.get_logger().warn(
                            f"🛑 Direction flip guard active; suppressing {source}"
                        )
            if sign != self.last_cmd_sign:
                self.last_dir_change_time = now
                self.last_cmd_sign = sign

        if linear == 0.0 and angular == 0.0 and self._front_blocked() and not self._rear_blocked():
            linear = -(abs(self.safety_stop_backup_pwm) * 0.5 / max(1.0, self.max_speed_backward))
            angular = 0.0
        elif self.force_backward_on_zero_cmd and linear == 0.0 and angular == 0.0:
            linear = -(abs(self.safety_stop_backup_pwm) * 0.5 / max(1.0, self.max_speed_backward))
            angular = 0.0

        auto_backup = False
        if self._front_blocked() and (linear > 0.0 or abs(angular) > 0.0):
            if self._rear_blocked():
                # Front and rear blocked: allow in-place turning instead of backing up.
                linear = 0.0
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"🌀 Front+rear blocked (front {self.last_front_distance:.2f}m, "
                        f"rear {self.last_rear_distance:.2f}m); allowing turn only"
                    )
            else:
                # Auto-reverse immediately when front is blocked
                backup_v = -(abs(self.safety_stop_backup_pwm) * 0.5 / max(1.0, self.max_speed_backward))
                linear = backup_v
                angular = 0.0
                auto_backup = True
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"⬇️ Front blocked at {self.last_front_distance:.2f}m; backing up immediately"
                    )

        if linear < 0.0 and (now - self.last_wall_debug_log_time) >= self.wall_debug_log_interval:
            self.last_wall_debug_log_time = now
            self.get_logger().info(
                "🧱 Wall check: "
                f"front={self.last_front_distance:.2f}m blocked={self._front_blocked()}, "
                f"rear={self.last_rear_distance:.2f}m blocked={self._rear_blocked()}"
            )

        if not self.allow_backward_when_rear_blocked and linear < 0.0 and self._rear_blocked():
            linear = 0.0
            if self._front_blocked():
                period = max(0.5, self.escape_turn_period)
                phase = int(time.time() / period) % 2
                angular = self.escape_turn_speed if phase == 0 else -self.escape_turn_speed
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
        
        # Enforce minimum PWM when moving (avoid stall)
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
        self.last_scan_time = time.time()
        min_rear = float('inf')
        min_front = float('inf')
        angle = msg.angle_min
        for distance in msg.ranges:
            if distance < msg.range_min or distance > msg.range_max:
                angle += msg.angle_increment
                continue
            if -math.pi / 2 <= angle <= math.pi / 2:
                if distance < min_front:
                    min_front = distance
            if angle >= math.pi / 2 or angle <= -math.pi / 2:
                if distance < min_rear:
                    min_rear = distance
            angle += msg.angle_increment

        if min_front < float('inf'):
            self.last_front_distance = min_front
            if min_front <= self.front_stop_distance:
                self.front_blocked_until = time.time() + self.front_stop_hold_time

        if min_rear < float('inf'):
            self.last_rear_distance = min_rear
            if min_rear <= self.rear_stop_distance:
                self.rear_blocked_until = time.time() + self.rear_stop_hold_time

        if self.swap_lidar_front_back:
            self.last_front_distance, self.last_rear_distance = self.last_rear_distance, self.last_front_distance
            self.front_blocked_until, self.rear_blocked_until = self.rear_blocked_until, self.front_blocked_until

    def _rear_blocked(self) -> bool:
        return time.time() < self.rear_blocked_until

    def _front_blocked(self) -> bool:
        return time.time() < self.front_blocked_until

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
    
    def servo_cb(self, msg: Int32):
        """Send servo angle (0-180 degrees) to Arduino"""
        if self.ser is None:
            return

        angle = max(0, min(180, msg.data))
        try:
            cmd = f"SERVO:{angle}\n"
            with self._serial_lock:
                self.ser.write(cmd.encode())
            self.get_logger().debug(f'Servo: {angle}°')
        except Exception as e:
            self.get_logger().error(f'Serial write error (servo): {e}')

    def servo_cmd_cb(self, msg: Float32):
        if self.ser is None:
            return
        angle = int(max(0, min(180, msg.data)))
        try:
            cmd = f"SERVO:{angle}\n"
            with self._serial_lock:
                self.ser.write(cmd.encode())
            self.get_logger().info(f'🔄 Servo sweep: {angle}°')
        except Exception as e:
            self.get_logger().error(f'Serial write error (servo): {e}')

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

        v = self.last_cmd_linear
        # Use IMU yaw if available, else integrate
        if self.imu_yaw is not None:
            # Integrate x, y using IMU yaw
            dx = v * math.cos(self.imu_yaw) * dt
            dy = v * math.sin(self.imu_yaw) * dt
            self.x += dx
            self.y += dy
            self.yaw = self.imu_yaw
        else:
            w = self.last_cmd_angular
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
        # Use IMU quaternion if available, else synthesize from yaw
        if self.imu_quat is not None:
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
        if self.imu_quat is not None:
            odom.pose.pose.orientation = self.imu_quat
        else:
            odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = self.last_cmd_angular

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

        # DIAGNOSTIC: Log TF publishing status periodically
        if abs(v) > 0.001 or abs(self.last_cmd_angular) > 0.001:
            self.get_logger().info(f'📍 TF published: odom->base_footprint at ({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f}) v={v:.3f} w={self.last_cmd_angular:.3f}')

        self.last_time = now
    
    def _get_action_name(self, left, right):
        if left == 0 and right == 0:
            return '⏸️  STOPPED'
        elif left > 0 and right > 0:
            return '⬆️  FORWARD'
        elif left < 0 and right < 0:
            return '⬇️  BACKWARD'
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
                    self.get_logger().error(f'❌ Serial write exception: {se}', exc_info=True)
                    return

            action = action_override or self._get_action_name(pwm_left, pwm_right)
            msg = f'🚀 {action} | MOTOR:{pwm_left},{pwm_right} | SRC:{source}'
            if log_level == 'warn':
                self.get_logger().warn(msg)
            elif log_level == 'error':
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}', exc_info=True)

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
        if self.ser is None:
            return
        
        while self.serial_reading:
            try:
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
                                    self.get_logger().error(f'🚨 SAFETY_STOP! Published /safety_stop: {distance_cm}cm ({distance_m:.4f}m)')

                                    # Activate safety override backup (local motor control)
                                    if self.enable_safety_override:
                                        now = time.time()
                                        self.safety_override_until = max(self.safety_override_until, now + self.safety_stop_backup_time)
                                        if not self.safety_override_active:
                                            self.safety_override_active = True
                                            self.get_logger().warn(
                                                f'🚨 SAFETY OVERRIDE: backing up for {self.safety_stop_backup_time:.1f}s'
                                            )
                            else:
                                # Format: SAFETY_STOP:0 - OBSTACLE CLEARED
                                msg = Float32()
                                msg.data = 999.0  # Sentinel value: no obstacle
                                self.safety_stop_pub.publish(msg)
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
                
                time.sleep(0.01)  # Don't busy-wait
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
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

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorBridge()
    try:
        rclpy.spin(node)
    finally:
        node._shutdown_motors()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
