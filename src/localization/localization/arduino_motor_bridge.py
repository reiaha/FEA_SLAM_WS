#!/usr/bin/env python3

import atexit
import glob
import math
import threading
import time

import rclpy
import rclpy.time
import serial
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster
from .arduino_motor_bridge_runtime_mixin import ArduinoMotorBridgeRuntimeMixin

class ArduinoMotorBridge(ArduinoMotorBridgeRuntimeMixin, Node):
    def __init__(self):
        super().__init__('arduino_motor_bridge')
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.18)
        self.declare_parameter('max_speed_forward', 158)
        self.declare_parameter('max_speed_backward', 149)
        self.declare_parameter('max_linear_speed_mps', 0.347)
        self.declare_parameter('max_angular_speed_radps', 0.792)
        self.declare_parameter('fixed_pwm_forward', 149)
        self.declare_parameter('fixed_pwm_backward', 119)
        self.declare_parameter('fixed_pwm_turn', 149)
        self.declare_parameter('auto_backup_speed_mps', 0.099)
        self.declare_parameter('min_pwm', 100)
        self.declare_parameter('min_pwm_forward', 120)
        self.declare_parameter('min_pwm_backward', 115)
        self.declare_parameter('min_pwm_turn', 120)
        self.declare_parameter('velocity_deadband', 0.03)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('pwm_change_threshold', 8)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)                                                            
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')                                        
        self.declare_parameter('enable_stuck_recovery', True)
        self.declare_parameter('stuck_speed_threshold', 5.0)
        self.declare_parameter('stuck_time', 0.6)
        self.declare_parameter('stuck_backup_pwm', 120)
        self.declare_parameter('stuck_backup_time', 0.8)
        self.declare_parameter('safety_stop_backup_time', 2.0)
        self.declare_parameter('safety_stop_backup_pwm', 119)
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
        self.declare_parameter('scan_min_range', 0.27)                                                                
        self.declare_parameter('front_stop_distance', 0.50)
        self.declare_parameter('front_stop_hold_time', 0.6)
        self.declare_parameter('any_obstacle_stop_distance', 0.20)
        self.declare_parameter('any_obstacle_stop_hold_time', 0.5)
        self.declare_parameter('front_block_min_hits', 1)
        self.declare_parameter('any_obstacle_min_hits', 2)
        self.declare_parameter('scan_stale_timeout', 1.2)
        self.declare_parameter('flip_guard_time', 0.15)
        self.declare_parameter('swap_lidar_front_back', False)
        self.declare_parameter('escape_turn_speed', 0.693)
        self.declare_parameter('escape_turn_toggle_interval', 1.2)
        self.declare_parameter('osc_escape_turn_dur', 3.0)                                                                        
        self.declare_parameter('direction_flip_window_sec', 12.0)
        self.declare_parameter('direction_flip_trip_count', 6)
        self.declare_parameter('direction_flip_escape_turn_dur', 1.2)
        self.declare_parameter('turn_angular_scale', 0.75)
        self.declare_parameter('turn_smoothing_enabled', True)
        self.declare_parameter('turn_slew_rate_radps2', 3.0)                                         
        self.declare_parameter('invert_turn_direction', False)
        self.declare_parameter('turn_pwm_limit', 134)
        self.declare_parameter('front_obstacle_backup_speed_mps', 0.119)
        self.declare_parameter('front_obstacle_backup_turn_scale', 0.35)
        self.declare_parameter('front_blocked_max_backup_sec', 0.9)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_obstacle_half_angle_deg', 50.0)
        self.declare_parameter('rear_obstacle_half_angle_deg', 30.0)
        self.declare_parameter('force_backward_on_zero_cmd', False)
        self.declare_parameter('allow_backward_when_rear_blocked', False)
        self.declare_parameter('require_nav2_active', True)
        self.declare_parameter('require_active_goal', True)                                         
        self.declare_parameter('prefer_nav_cmd_with_active_goal', True)                                                            
        self.declare_parameter('nav2_state_check_interval', 1.0)
        self.declare_parameter('nav2_state_response_timeout', 2.5)
        self.declare_parameter('nav2_inactive_confirm_sec', 6.0)
        self.declare_parameter('nav2_allow_goal_override', False)
        self.declare_parameter('nav2_min_active_nodes', 2)
        self.declare_parameter('nav2_tolerate_bt_pending', True)
        self.declare_parameter('nav2_uncertain_grace_sec', 10.0)
        self.declare_parameter('nav2_inactive_stop_repeat_sec', 0.5)
        self.declare_parameter('nav_cmd_timeout_sec', 1.2)
        self.declare_parameter('nav_cmd_log_interval', 2.0)
        self.declare_parameter('motor_cmd_log_interval', 1.5)
        self.declare_parameter('serial_reconnect_interval', 1.0)
        self.declare_parameter('serial_max_error_streak', 5)
        self.declare_parameter('serial_error_log_interval', 2.0)
        self.declare_parameter('odom_feedback_gate_enabled', True)
        self.declare_parameter('odom_stationary_speed_threshold', 5.0)
        self.declare_parameter('odom_freeze_pose_when_stationary', True)
        self.declare_parameter('odom_linear_scale', 1.0)
        self.declare_parameter('odom_angular_scale', 1.0)
        self.declare_parameter('use_imu_yaw_in_odom', True)                                                      
        self.declare_parameter('imu_publish', True)                                                                  
        self.declare_parameter('imu_rotated_180', True)                                                                                                    
        self.declare_parameter('imu_gyro_scale', 0.017453)                                                                    
        self.declare_parameter('imu_accel_scale', 1.0)                                                             
        self.declare_parameter('imu_sign_check_enabled', True)
        self.declare_parameter('imu_sign_check_min_cmd_radps', 0.25)
        self.declare_parameter('imu_sign_mismatch_trip_count', 5)
        self.declare_parameter('imu_sign_disable_sec', 8.0)
        self.declare_parameter('enable_tilt_safety', True)
        self.declare_parameter('tilt_speed_scale_topic', '/tilt_speed_scale')
        self.declare_parameter('tilt_hazard_topic', '/tilt_hazard')
        self.declare_parameter('tilt_emergency_topic', '/tilt_emergency')
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
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
        self.turn_pwm_floor = max(100, self.min_pwm_turn)

        # Keep in-place turning above the configured turn deadzone.
        if self.fixed_pwm_turn < self.min_pwm_turn:
            self.get_logger().warn(
                f"⚠️ fixed_pwm_turn ({self.fixed_pwm_turn}) < min_pwm_turn ({self.min_pwm_turn}); "
                f"clamping turn PWM to {self.min_pwm_turn}"
            )
            self.fixed_pwm_turn = self.min_pwm_turn
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.pwm_change_threshold = int(self.get_parameter('pwm_change_threshold').value)
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.imu_yaw = None
        self.imu_quat = None
        self.imu_last_stamp = None
        self.imu_angular_vel_z = None                                                  
        self.imu_hardware_dead = False                                                      
        self._imu_zero_streak = 0                                       

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
        self.scan_min_range = float(self.get_parameter('scan_min_range').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_stop_hold_time = float(self.get_parameter('front_stop_hold_time').value)
        self.any_obstacle_stop_distance = float(self.get_parameter('any_obstacle_stop_distance').value)
        self.any_obstacle_stop_hold_time = float(self.get_parameter('any_obstacle_stop_hold_time').value)
        self.front_block_min_hits = int(self.get_parameter('front_block_min_hits').value)
        self.any_obstacle_min_hits = int(self.get_parameter('any_obstacle_min_hits').value)
        self.scan_stale_timeout = float(self.get_parameter('scan_stale_timeout').value)
        self.flip_guard_time = float(self.get_parameter('flip_guard_time').value)
        self.swap_lidar_front_back = bool(self.get_parameter('swap_lidar_front_back').value)
        self.escape_turn_speed = float(self.get_parameter('escape_turn_speed').value)
        self.escape_turn_toggle_interval = float(self.get_parameter('escape_turn_toggle_interval').value)
        _osc_turn_dur_param = float(self.get_parameter('osc_escape_turn_dur').value)
        _dir_flip_window_sec_param = max(4.0, float(self.get_parameter('direction_flip_window_sec').value))
        _dir_flip_trip_count_param = max(3, int(self.get_parameter('direction_flip_trip_count').value))
        _dir_flip_escape_turn_dur_param = max(0.5, float(self.get_parameter('direction_flip_escape_turn_dur').value))
        self.turn_angular_scale = float(self.get_parameter('turn_angular_scale').value)
        self.turn_smoothing_enabled = bool(self.get_parameter('turn_smoothing_enabled').value)
        self.turn_slew_rate_radps2 = float(self.get_parameter('turn_slew_rate_radps2').value)
        self.invert_turn_direction = bool(self.get_parameter('invert_turn_direction').value)
        self.turn_pwm_limit = int(self.get_parameter('turn_pwm_limit').value)
        self.front_obstacle_backup_speed_mps = float(self.get_parameter('front_obstacle_backup_speed_mps').value)
        self.front_obstacle_backup_turn_scale = float(self.get_parameter('front_obstacle_backup_turn_scale').value)
        self.front_blocked_max_backup_sec = max(0.0, float(self.get_parameter('front_blocked_max_backup_sec').value))
        self.scan_topic = self.get_parameter('scan_topic').value
        self.front_obstacle_half_angle = math.radians(float(self.get_parameter('front_obstacle_half_angle_deg').value))
        self.rear_obstacle_half_angle = math.radians(float(self.get_parameter('rear_obstacle_half_angle_deg').value))
        self.force_backward_on_zero_cmd = bool(self.get_parameter('force_backward_on_zero_cmd').value)
        self.allow_backward_when_rear_blocked = bool(self.get_parameter('allow_backward_when_rear_blocked').value)
        self.require_nav2_active = bool(self.get_parameter('require_nav2_active').value)
        self.require_active_goal = bool(self.get_parameter('require_active_goal').value)
        self.prefer_nav_cmd_with_active_goal = bool(self.get_parameter('prefer_nav_cmd_with_active_goal').value)
        self.nav2_state_check_interval = float(self.get_parameter('nav2_state_check_interval').value)
        self.nav2_state_response_timeout = float(self.get_parameter('nav2_state_response_timeout').value)
        self.nav2_inactive_confirm_sec = float(self.get_parameter('nav2_inactive_confirm_sec').value)
        self.nav2_allow_goal_override = bool(self.get_parameter('nav2_allow_goal_override').value)
        self.nav2_min_active_nodes = int(self.get_parameter('nav2_min_active_nodes').value)
        self.nav2_tolerate_bt_pending = bool(self.get_parameter('nav2_tolerate_bt_pending').value)
        self.nav2_uncertain_grace_sec = float(self.get_parameter('nav2_uncertain_grace_sec').value)
        self.nav2_inactive_stop_repeat_sec = float(self.get_parameter('nav2_inactive_stop_repeat_sec').value)
        self.nav_cmd_timeout_sec = max(0.2, float(self.get_parameter('nav_cmd_timeout_sec').value))
        self.nav_cmd_log_interval = max(0.5, float(self.get_parameter('nav_cmd_log_interval').value))
        self.motor_cmd_log_interval = max(0.2, float(self.get_parameter('motor_cmd_log_interval').value))
        self.serial_reconnect_interval = float(self.get_parameter('serial_reconnect_interval').value)
        self.serial_max_error_streak = int(self.get_parameter('serial_max_error_streak').value)
        self.serial_error_log_interval = float(self.get_parameter('serial_error_log_interval').value)
        self.odom_feedback_gate_enabled = bool(self.get_parameter('odom_feedback_gate_enabled').value)
        self.use_imu_yaw_in_odom = bool(self.get_parameter('use_imu_yaw_in_odom').value)
        self.imu_publish = bool(self.get_parameter('imu_publish').value)
        self.imu_rotated_180 = bool(self.get_parameter('imu_rotated_180').value)
        self.imu_gyro_scale = float(self.get_parameter('imu_gyro_scale').value)
        self.imu_accel_scale = float(self.get_parameter('imu_accel_scale').value)
        self.imu_sign_check_enabled = bool(self.get_parameter('imu_sign_check_enabled').value)
        self.imu_sign_check_min_cmd_radps = float(self.get_parameter('imu_sign_check_min_cmd_radps').value)
        self.imu_sign_mismatch_trip_count = int(self.get_parameter('imu_sign_mismatch_trip_count').value)
        self.imu_sign_disable_sec = float(self.get_parameter('imu_sign_disable_sec').value)
        self.enable_tilt_safety = bool(self.get_parameter('enable_tilt_safety').value)
        self.tilt_speed_scale_topic = str(self.get_parameter('tilt_speed_scale_topic').value)
        self.tilt_hazard_topic = str(self.get_parameter('tilt_hazard_topic').value)
        self.tilt_emergency_topic = str(self.get_parameter('tilt_emergency_topic').value)
        self.odom_stationary_speed_threshold = float(self.get_parameter('odom_stationary_speed_threshold').value)
        self.odom_freeze_pose_when_stationary = bool(self.get_parameter('odom_freeze_pose_when_stationary').value)
        self.odom_linear_scale = float(self.get_parameter('odom_linear_scale').value)
        self.odom_angular_scale = float(self.get_parameter('odom_angular_scale').value)

        self.tilt_speed_scale = 1.0
        self.tilt_hazard_active = False
        self.tilt_emergency_active = False
        self.last_tilt_limit_log_time = 0.0
        self.tilt_limit_log_interval = 1.0

        self.cmd_vel_override_duration = 0.2
        self.cmd_vel_override_until = 0.0
        self.last_turn_cmd_time = time.time()
        self.smoothed_angular_cmd = 0.0

        self.get_logger().info(
            f"⚙️ PWM profile loaded: forward={self.fixed_pwm_forward}, backward={self.fixed_pwm_backward}, turn={self.fixed_pwm_turn}"
        )
        self.get_logger().info(
            f"⚙️ Turn PWM guards: min_turn={self.min_pwm_turn}, turn_limit={self.turn_pwm_limit}, turn_floor={self.turn_pwm_floor}"
        )

        self.nav2_ready = not self.require_nav2_active
        self.has_active_goal = False
        self.last_goal_active_at = 0.0
        self.goal_cleared_at = 0.0                                              
        self.goal_cleared_grace = 1.5                                                                 
        self.last_goal_block_log_time = 0.0
        self.goal_block_log_interval = 2.0
        self.nav2_state_log_interval = 2.0
        self.last_nav2_state_log_time = 0.0
        self.last_nav2_ready = self.nav2_ready
        self.nav2_not_ready_since = 0.0
        self.nav2_explicitly_inactive = False
        self.last_nav2_ready_true_at = time.time()
        self.last_nav2_inactive_stop_time = 0.0
        self.last_nav_cmd_time = 0.0
        self.last_nav_cmd_stale_log_time = 0.0
        self.nav2_clients = {
            'controller_server': self.create_client(GetState, '/controller_server/get_state'),
            'planner_server': self.create_client(GetState, '/planner_server/get_state'),
            'bt_navigator': self.create_client(GetState, '/bt_navigator/get_state')
        }
        self.nav2_state_futures = {}
        self.nav2_state_request_time = {}
        self.nav2_state_cache = {}
        self.nav2_state_update_time = {}
        if self.require_nav2_active:
            check_interval = max(0.5, self.nav2_state_check_interval)
            self.create_timer(check_interval, self._check_nav2_state)
            self.get_logger().info(f"🧭 Nav2 state checker enabled (interval: {check_interval}s)")
        else:
            self.get_logger().info("🧭 Nav2 state checking disabled")

        self.ser = None
        if serial_port.startswith('/dev/ttyACM'):
            ports_to_try = [serial_port] + sorted(glob.glob('/dev/ttyACM*'))
        elif serial_port.startswith('/dev/ttyUSB'):
            ports_to_try = [serial_port, '/dev/ttyUSB0', '/dev/ttyUSB1']
        else:
            ports_to_try = [serial_port] + sorted(glob.glob('/dev/ttyACM*'))
        self.ports_to_try = list(dict.fromkeys(ports_to_try))
        self.serial_error_streak = 0
        self.last_serial_error_log_time = 0.0
        self.last_serial_reconnect_time = 0.0
        self.last_motor_log_time = 0.0
        self.last_motor_log_signature = None
        for port in ports_to_try:
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(1.0)                                            
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.get_logger().info(f'✅ Arduino connected on {port}')
                break
            except Exception:
                pass

        self.serial_disabled = False
        if self.ser is None:
            self.serial_disabled = True
            self.get_logger().error('❌ Failed to connect to Arduino - running without serial (odom only)')
        else:
            self.get_logger().info(f'✅ Serial port open: {self.ser.port}, baudrate: {self.ser.baudrate}')

        if not self.serial_disabled:
            try:
                bytes_written = self.ser.write(b'START\n')
                self.get_logger().info(f'📤 Sent START command: {bytes_written} bytes')
                time.sleep(0.1)
                self.get_logger().info('✅ Motors ENABLED')

                bytes_written = self.ser.write(b'AUTO:OFF\n')
                self.get_logger().info(f'📤 Sent AUTO:OFF command: {bytes_written} bytes')
                time.sleep(0.1)
                self.get_logger().info('✅ Arduino local avoidance DISABLED')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize Arduino: {e}')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        if self.enable_tilt_safety:
            self.create_subscription(Float32, self.tilt_speed_scale_topic, self._tilt_speed_scale_cb, 10)
            self.create_subscription(Bool, self.tilt_hazard_topic, self._tilt_hazard_cb, 10)
            self.create_subscription(Bool, self.tilt_emergency_topic, self._tilt_emergency_cb, 10)
            self.get_logger().info(
                f"🛡️ Tilt safety enabled (scale={self.tilt_speed_scale_topic}, "
                f"hazard={self.tilt_hazard_topic}, emergency={self.tilt_emergency_topic})"
            )

        from action_msgs.msg import GoalStatusArray
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav2_goal_status_cb,
            10
        )

        self.ultrasonic_pub = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        self.ultrasonic_range_pub = self.create_publisher(Range, '/ultrasonic_range', 10)
        self.safety_stop_pub = self.create_publisher(Float32, '/safety_stop', 10)
        self.x_rel_pub = self.create_publisher(Float32, '/x_rel', 10)
        self.y_rel_pub = self.create_publisher(Float32, '/y_rel', 10)
        self._Imu = Imu
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10) if self.imu_publish else None

        self.odom_pub = None
        self.tf_broadcaster = None
        self.odom_fallback_active = False                                    
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False
        self._serial_lock = threading.Lock()                                   

        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 106)
            if self.publish_tf:
                self.tf_broadcaster = TransformBroadcaster(self)
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.x_origin = 0.0
            self.y_origin = 0.0
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = 0.0
            self.last_time = self.get_clock().now()
            self.create_timer(1.0 / max(1.0, self.odom_rate), self._publish_odom)
            self.get_logger().info('✅ Odometry publisher initialized')

        self.serial_reading = True
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

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
        self.front_blocked_backup_until = 0.0
        self.spin_only_start = 0.0                                                              
        self.spin_escape_timeout = 4.0                                                         
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
        self.escape_turn_dir = 1.0
        self.last_escape_turn_toggle_time = 0.0
        self.safety_stop_obstacle_hits = 0
        self.safety_stop_clear_hits = 0
        self.last_safety_stop_brake_time = 0.0
        self.last_odom_feedback_gate_log_time = 0.0
        self.odom_feedback_gate_log_interval = 1.0
        self.last_imu_sign_mismatch_log_time = 0.0
        self.imu_sign_mismatch_streak = 0
        self.imu_yaw_override_disabled_until = 0.0

        from collections import deque
        self._motion_state_history = deque()                          
        self._osc_window_sec   = 20.0                                         
        self._osc_trip_count   = 8                                                        
        self._dir_flip_history = deque()
        self._dir_flip_window_sec = _dir_flip_window_sec_param
        self._dir_flip_trip_count = _dir_flip_trip_count_param
        self._dir_flip_escape_turn_dur = _dir_flip_escape_turn_dur_param
        self._osc_escape_until = 0.0                                              
        self._osc_escape_backup_dur = 0.8
        self._osc_escape_turn_dur   = _osc_turn_dur_param                              
        self._osc_escape_active     = False
        self._osc_escape_step       = 0                     
        self._osc_escape_step_start = 0.0
        self._osc_escape_turn_dir   = 1.0

        if self.enable_stuck_recovery:
            self.create_timer(0.1, self._stuck_recovery_loop)
        self.create_timer(0.1, self._osc_escape_step_loop)

        self.create_timer(0.05, self._safety_override_loop)

        self.get_logger().info('🚀 Arduino Motor Bridge initialized')

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
        self.imu_angular_vel_z = msg.angular_velocity.z
        self.imu_last_stamp = msg.header.stamp                                            
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-6:
            return
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_quat = q
        self.imu_last_stamp = msg.header.stamp
    
    def _nav2_goal_status_cb(self, msg):
        from action_msgs.msg import GoalStatus
        now = time.time()
        was_active = self.has_active_goal
        has_active_from_status = any(
            gs.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            for gs in msg.status_list
        )

        if has_active_from_status:
            self.has_active_goal = True
            self.last_goal_active_at = now
        else:
            # Debounce transient status-list dropouts under CPU spikes.
            if (now - self.last_goal_active_at) <= self.goal_cleared_grace:
                self.has_active_goal = True
            else:
                self.has_active_goal = False

        if self.has_active_goal != was_active:
            state = 'ACTIVE' if self.has_active_goal else 'NONE'
            self.get_logger().info(f'🎯 Nav2 goal state changed: {state}')
            if not self.has_active_goal:
                self.goal_cleared_at = now

    def cmd_vel_cb(self, msg: Twist):
        if self.prefer_nav_cmd_with_active_goal and self.has_active_goal and self.nav2_ready:
            linear = float(msg.linear.x)
            allow_recovery = linear < -self.velocity_deadband
            if not allow_recovery:
                now = time.time()
                if now - self.last_goal_block_log_time >= self.goal_block_log_interval:
                    self.last_goal_block_log_time = now
                    self.get_logger().warn('🧭 Active Nav2 goal: suppressing /cmd_vel to keep motor-goal sync')
                return
        self._handle_cmd_vel(msg, source='cmd_vel')

    def _tilt_speed_scale_cb(self, msg: Float32):
        self.tilt_speed_scale = max(0.0, min(1.0, float(msg.data)))

    def _tilt_hazard_cb(self, msg: Bool):
        self.tilt_hazard_active = bool(msg.data)

    def _tilt_emergency_cb(self, msg: Bool):
        self.tilt_emergency_active = bool(msg.data)

    def _handle_cmd_vel(self, msg: Twist, source: str):
        # ...existing code...
        self.cmd_vel_override_until = time.time() + self.cmd_vel_override_duration
        linear = msg.linear.x
        angular = msg.angular.z
        # ...existing code...
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
        # HARD SAFETY: Log backward attempts only at error level (fast path - no detailed logging)
        if self.require_nav2_active and not self.nav2_ready:
            now = time.time()
            if now - self.last_nav2_block_log_time >= self.nav2_block_log_interval:
                self.last_nav2_block_log_time = now
                self.get_logger().warn(f"🛑 Nav2 not active; suppressing {source}")
            return
        if self.require_active_goal and source == 'cmd_vel_nav' and not self.has_active_goal:
            now = time.time()
            if (now - self.goal_cleared_at) < self.goal_cleared_grace:
                pass                                                   
            else:
                if now - self.last_goal_block_log_time >= self.goal_block_log_interval:
                    self.last_goal_block_log_time = now
                    self.get_logger().warn(f"🛑 No active Nav2 goal; suppressing {source}")
                return
        if self.enable_safety_override and self.safety_override_active:
            return

        if self.stuck_recovery_active:
            return

        if self._osc_escape_active:
            return

        self.cmd_vel_override_until = time.time() + self.cmd_vel_override_duration
        
        linear = msg.linear.x           
        angular = msg.angular.z           

        if self.enable_tilt_safety:
            now_tilt = time.time()
            if self.tilt_emergency_active or self.tilt_hazard_active:
                linear = 0.0
                angular = 0.0
                if (now_tilt - self.last_tilt_limit_log_time) >= self.tilt_limit_log_interval:
                    self.last_tilt_limit_log_time = now_tilt
                    state = 'EMERGENCY' if self.tilt_emergency_active else 'HAZARD'
                    self.get_logger().warn(f"🛑 Tilt {state}: forcing motor stop")
            elif self.tilt_speed_scale < 0.999:
                linear *= self.tilt_speed_scale
                angular *= self.tilt_speed_scale
                if (now_tilt - self.last_tilt_limit_log_time) >= self.tilt_limit_log_interval:
                    self.last_tilt_limit_log_time = now_tilt
                    self.get_logger().warn(
                        f"⚠️ Tilt slowdown active: scaling cmd_vel by {self.tilt_speed_scale:.2f}"
                    )

        linear = max(-self.max_linear_speed_mps, min(self.max_linear_speed_mps, linear))
        angular = max(-self.max_angular_speed_radps, min(self.max_angular_speed_radps, angular))

        if abs(linear) < self.velocity_deadband:
            linear = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0
        elif linear == 0.0:
            angular *= self.turn_angular_scale

        if self.invert_turn_direction:
            angular = -angular

        if self.turn_smoothing_enabled:
            now_slew = time.time()
            dt = max(1e-3, now_slew - self.last_turn_cmd_time)
            self.last_turn_cmd_time = now_slew
            max_delta = max(0.0, self.turn_slew_rate_radps2) * dt
            delta = angular - self.smoothed_angular_cmd
            if abs(delta) > max_delta:
                angular = self.smoothed_angular_cmd + math.copysign(max_delta, delta)
            self.smoothed_angular_cmd = angular
        else:
            self.smoothed_angular_cmd = angular

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

        front_escape_active = (not scan_stale) and self._front_blocked()

        if linear != 0.0:
            sign = 1 if linear > 0.0 else -1
            if self.last_cmd_sign != 0 and sign != self.last_cmd_sign:
                if ((now - self.last_dir_change_time) < self.flip_guard_time) and (not front_escape_active):
                    linear = 0.0
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

        can_front_backup = (
            (not scan_stale) and
            (not self._rear_blocked()) and
            math.isfinite(self.last_rear_distance) and
            (self.last_rear_distance > self.rear_backup_min_distance)
        )

        front_blocked  = (not scan_stale) and self._front_blocked()
        backup_window_open = now <= self.front_blocked_backup_until
        spin_timeout_exceeded = (
            self.spin_only_start > 0.0 and
            (now - self.spin_only_start) >= self.spin_escape_timeout
        )

        if linear > 0.0 and front_blocked and self._rear_blocked():
            linear = 0.0
            if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                self.last_front_block_log_time = now
                self.get_logger().warn(
                    f"🌀 Front+rear blocked (front {self.last_front_distance:.2f}m, "
                    f"rear {self.last_rear_distance:.2f}m); allowing turn only"
                )

        elif linear > 0.0 and front_blocked and abs(angular) > self.angular_deadband and can_front_backup and backup_window_open:
            linear = -abs(self.front_obstacle_backup_speed_mps)
            angular *= self.front_obstacle_backup_turn_scale
            self.spin_only_start = 0.0
            if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                self.last_front_block_log_time = now
                self.get_logger().warn(
                    f"⬇️ Front blocked at {self.last_front_distance:.2f}m; backing while turning"
                )

        elif linear > 0.0 and (front_blocked or spin_timeout_exceeded):
            angular = self._select_escape_turn(angular)
            if backup_window_open and (can_front_backup or spin_timeout_exceeded):
                linear = -abs(self.front_obstacle_backup_speed_mps)
                angular *= self.front_obstacle_backup_turn_scale
                if spin_timeout_exceeded:
                    self.spin_only_start = 0.0
                    self.get_logger().warn(
                        f"🔀 Spin-escape: forced backup after {self.spin_escape_timeout:.0f}s of rotation"
                    )
            else:
                linear = 0.0
                if self.spin_only_start == 0.0:
                    self.spin_only_start = now
                if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                    self.last_front_block_log_time = now
                    self.get_logger().warn(
                        f"🧭 Front blocked persists beyond backup window ({self.front_blocked_max_backup_sec:.2f}s); rotate-only"
                    )
            dist_str = (f"{self.last_front_distance:.2f}m" if front_blocked
                        else f"{self.last_any_obstacle_distance:.2f}m")
            if now - self.last_front_block_log_time >= self.rear_block_log_interval:
                self.last_front_block_log_time = now
                self.get_logger().warn(
                    f"⬇️ Obstacle at {dist_str}; "
                    f"{'backing off' if (backup_window_open and (can_front_backup or spin_timeout_exceeded)) else 'rotate-only'}"
                )

        else:
            self.spin_only_start = 0.0

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

        # HARD SAFETY: Never allow forward if front is too close, never allow backward if rear is too close
        # Only allow rotation if both are blocked
        if linear > 0.0:
            if not math.isfinite(self.last_front_distance) or self.last_front_distance <= self.front_stop_distance:
                linear = 0.0
                if abs(angular) < self.angular_deadband:
                    angular = 0.0
                if (not math.isfinite(self.last_rear_distance) or self.last_rear_distance <= self.rear_backup_min_distance):
                    # Both front and rear blocked: only allow rotation
                    if abs(angular) < self.angular_deadband:
                        angular = self.turn_pwm_limit / 255.0  # force a small turn
                    self.get_logger().error(
                        f"🛑 HARD SAFETY: Both front ({self.last_front_distance:.2f}m) and rear ({self.last_rear_distance:.2f}m) blocked; allowing only rotation."
                    )
                else:
                    self.get_logger().error(
                        f"🛑 HARD SAFETY: Front distance {self.last_front_distance:.2f}m <= stop {self.front_stop_distance:.2f}m; suppressing ALL forward cmd_vel (source: {source})"
                    )

        # HARD SAFETY: Never allow backward motion if rear is too close, regardless of command source
        if linear < 0.0:
            if not math.isfinite(self.last_rear_distance) or self.last_rear_distance <= self.rear_backup_min_distance:
                linear = 0.0
                angular = 0.0
                if now - self.last_rear_block_log_time >= self.rear_block_log_interval:
                    self.last_rear_block_log_time = now
                    self.get_logger().error(
                        f"🛑 HARD SAFETY: Rear distance {self.last_rear_distance:.2f}m <= backup min "
                        f"{self.rear_backup_min_distance:.2f}m; suppressing ALL backward cmd_vel (source: {source})"
                    )

        if self.publish_odom:
            self.last_cmd_linear = linear
            self.last_cmd_angular = angular
        
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        scale_left = self.max_speed_forward if v_left >= 0.0 else self.max_speed_backward
        scale_right = self.max_speed_forward if v_right >= 0.0 else self.max_speed_backward
        pwm_left = int(v_left * (scale_left / 0.5))
        pwm_right = int(v_right * (scale_right / 0.5))
        
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
        
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))

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
            target_turn_pwm = max(self.turn_pwm_floor, abs(turn_pwm))
            if angular > 0.0:
                pwm_left = -target_turn_pwm
                pwm_right = target_turn_pwm
            else:
                pwm_left = target_turn_pwm
                pwm_right = -target_turn_pwm
        elif linear > self.velocity_deadband and abs(angular) > self.angular_deadband:
            turn_fraction = min(abs(angular) / self.max_angular_speed_radps, 1.0)
            inner_pwm = max(self.min_pwm_forward, int(fwd_pwm * (1.0 - turn_fraction)))
            if angular > 0.0:                                                
                pwm_left = inner_pwm
                pwm_right = fwd_pwm
            else:                                                             
                pwm_left = fwd_pwm
                pwm_right = inner_pwm
        elif linear < -self.velocity_deadband and abs(angular) > self.angular_deadband:
            if angular > 0.0:
                pwm_left = -turn_pwm
                pwm_right = -back_pwm
            else:
                pwm_left = -back_pwm
                pwm_right = -turn_pwm

        if linear == 0.0 and abs(angular) > self.angular_deadband:
            # Keep commanded in-place turn above turn deadzone even when turn limit is set too low.
            turn_limit = max(self.min_pwm_turn, self.min_pwm, self.turn_pwm_limit, self.turn_pwm_floor)
            pwm_left = max(-turn_limit, min(turn_limit, pwm_left))
            pwm_right = max(-turn_limit, min(turn_limit, pwm_right))

        if (
            abs(pwm_left - self.last_cmd_pwm_left) < self.pwm_change_threshold and
            abs(pwm_right - self.last_cmd_pwm_right) < self.pwm_change_threshold
        ):
            pwm_left = self.last_cmd_pwm_left
            pwm_right = self.last_cmd_pwm_right

        self.last_cmd_pwm_left = pwm_left
        self.last_cmd_pwm_right = pwm_right
        self.last_cmd_time = time.time()

        if (not self.imu_hardware_dead and
                self.imu_angular_vel_z is not None and
                abs(angular) > 0.3 and
                abs(linear) <= self.velocity_deadband and
                not self._front_blocked() and
                abs(self.imu_angular_vel_z) > 0.08 and
                (angular > 0) != (self.imu_angular_vel_z > 0)):
            if (now - self.last_imu_sign_mismatch_log_time) >= 5.0:
                self.last_imu_sign_mismatch_log_time = now
                self.get_logger().warn(
                    f"⚠️ IMU sign mismatch: Nav2 cmd angular={angular:.2f} rad/s but "
                    f"IMU gz={self.imu_angular_vel_z:.3f} rad/s (opposite sign). "
                    f"Robot heading drifts opposite to command → endless pivot loop. "
                    f"Fix: flip imu_rotated_180 (True <-> False) in launch file and retest pivot sign."
                )

        self._send_motor_pwm(pwm_left, pwm_right, source=source)
        self._track_motion_oscillation(pwm_left, pwm_right)

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
