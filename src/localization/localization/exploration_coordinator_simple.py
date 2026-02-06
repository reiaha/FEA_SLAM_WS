#!/usr/bin/env python3
"""
FEA-SLAM Exploration Coordinator - SIMPLIFIED VERSION
Core logic only: 5 phases, ~400 lines

What it does:
1. Wait for frontiers from frontier_detector
2. Pick best frontier and send to Nav2
3. Monitor obstacles - if too close, backup and rescan
4. Repeat until exploration complete
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Float32
from action_msgs.msg import GoalStatus
from tf2_ros import TransformListener, Buffer
from enum import Enum
import math
import time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

class Phase(Enum):
    INIT = 1
    EXPLORE = 2           # Pick frontier, plan, move
    OBSTACLE = 3          # Handle collision
    RESCAN = 4            # Check if path clear
    RECOVERY = 5          # Stuck recovery - rotate in place
    DONE = 6              # Exploration complete

class ExplorationCoordinator(Node):
    def __init__(self):
        super().__init__('exploration_coordinator_v2')
        
        # Parameters
        self.declare_parameter('nav2_timeout', 30.0)
        self.declare_parameter('obstacle_distance', 0.35)      # meters (35cm - earlier stop)
        self.declare_parameter('backup_speed', -0.3)           # m/s (gentler backward)
        self.declare_parameter('backup_time', 2.0)             # seconds (2 seconds duration)
        self.declare_parameter('use_lidar_obstacle', True)
        self.declare_parameter('use_ultrasonic_backup', True)
        self.declare_parameter('lidar_obstacle_distance', 0.45)   # meters (earlier stop)
        self.declare_parameter('ultrasonic_backup_distance', 0.10)  # meters
        self.declare_parameter('lidar_stale_timeout', 0.5)       # seconds
        self.declare_parameter('rear_obstacle_threshold', 0.25)   # meters (rear safety)
        self.declare_parameter('rear_obstacle_hold_time', 0.8)    # seconds (latch rear obstacle)
        self.declare_parameter('frontier_goal_offset', 0.35)   # meters (pull goal into free space)
        self.declare_parameter('min_frontier_distance', 0.8)   # meters (avoid very close goals)
        self.declare_parameter('blacklist_duration', 30.0)     # seconds (avoid failed goals)
        self.declare_parameter('blacklist_radius', 0.4)        # meters (treat nearby goals as same)
        self.declare_parameter('avoid_revisit', True)          # skip goals near previously reached ones
        self.declare_parameter('visited_goal_radius', 0.6)     # meters (radius to treat as visited)
        self.declare_parameter('recent_goal_radius', 0.8)      # meters (avoid recently attempted goals)
        self.declare_parameter('recent_goal_hold_time', 90.0)  # seconds (cooldown for attempted goals)
        self.declare_parameter('startup_scan_time', 2.0)       # seconds to gather initial LiDAR/frontiers
        self.declare_parameter('nav2_handles_obstacles', True) # let Nav2 handle obstacle avoidance
        self.declare_parameter('use_servo_scan', False)        # disable servo sweep for testing
        self.declare_parameter('startup_clear_hold_time', 1.0) # seconds path must be clear before explore
        self.declare_parameter('base_frame', 'base_footprint') # TF base frame
        self.declare_parameter('scan_raw_topic', '/scan_raw')  # raw scan topic (timestamp fix source)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('frontier_lidar_fallback_timeout', 2.0)
        self.declare_parameter('costmap_free_threshold', 50)
        self.declare_parameter('use_costmap_goal_filter', True)
        self.declare_parameter('pose_movement_threshold', 0.05)
        self.declare_parameter('require_costmap', True)
        self.declare_parameter('require_nav2_active', True)
        
        self.nav2_timeout = self.get_parameter('nav2_timeout').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_time = self.get_parameter('backup_time').value
        self.backup_iterations = int(self.backup_time / 0.05)  # 40 at 20Hz
        self.use_lidar_obstacle = self.get_parameter('use_lidar_obstacle').value
        self.use_ultrasonic_backup = self.get_parameter('use_ultrasonic_backup').value
        self.lidar_obstacle_distance = self.get_parameter('lidar_obstacle_distance').value
        self.ultrasonic_backup_distance = self.get_parameter('ultrasonic_backup_distance').value
        self.lidar_stale_timeout = self.get_parameter('lidar_stale_timeout').value
        self.rear_obstacle_threshold = self.get_parameter('rear_obstacle_threshold').value
        self.rear_obstacle_hold_time = self.get_parameter('rear_obstacle_hold_time').value
        self.frontier_goal_offset = self.get_parameter('frontier_goal_offset').value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').value
        self.blacklist_duration = self.get_parameter('blacklist_duration').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.avoid_revisit = self.get_parameter('avoid_revisit').value
        self.visited_goal_radius = self.get_parameter('visited_goal_radius').value
        self.recent_goal_radius = float(self.get_parameter('recent_goal_radius').value)
        self.recent_goal_hold_time = float(self.get_parameter('recent_goal_hold_time').value)
        self.startup_scan_time = self.get_parameter('startup_scan_time').value
        self.startup_clear_hold_time = self.get_parameter('startup_clear_hold_time').value
        self.nav2_handles_obstacles = self.get_parameter('nav2_handles_obstacles').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_servo_scan = self.get_parameter('use_servo_scan').value
        self.scan_raw_topic = self.get_parameter('scan_raw_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.frontier_lidar_fallback_timeout = float(self.get_parameter('frontier_lidar_fallback_timeout').value)
        self.costmap_free_threshold = int(self.get_parameter('costmap_free_threshold').value)
        self.use_costmap_goal_filter = self.get_parameter('use_costmap_goal_filter').value
        self.pose_movement_threshold = float(self.get_parameter('pose_movement_threshold').value)
        self.require_costmap = self.get_parameter('require_costmap').value
        self.require_nav2_active = self.get_parameter('require_nav2_active').value
        
        # State
        self.current_phase = Phase.INIT
        self.phase_start_time = None  # Will be set when we detect frontiers
        self.robot_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.current_frontiers = []
        self.obstacle_detected = False
        self.obstacle_distance_m = float('inf')
        self.last_obstacle_time = 0.0
        self.obstacle_hold_time = 1.0  # seconds to latch detection
        self.last_scan_time = 0.0
        self.rescan_done = False
        self.ultrasonic_emergency_until = 0.0
        self.nav2_ready = False
        self.nav2_activation_time = 15.0  # Wait 15 seconds for Nav2 to fully activate
        self.startup_scan_end_time = None
        self.first_lidar_time = None
        self.first_frontier_time = None
        self.first_pose_time = None
        self.last_frontier_time = 0.0
        self.last_startup_status_log = 0.0
        self.startup_status_log_interval = 2.0
        self.startup_diag_interval = 2.0
        self.last_startup_diag_log = 0.0
        self.goal_handle = None
        self.goal_in_progress = False  # Flag: prevents sending goals while waiting for callback
        self.no_frontier_cycles = 0
        self.last_goal_time = 0.0  # Track when we last sent a goal
        self.goal_cooldown = 3.0   # Wait 3 seconds between goal attempts
        
        # Stuck detection and recovery
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3  # After 3 failed goals, trigger recovery
        self.last_successful_goal_pos = (0.0, 0.0)
        self.stuck_recovery_in_progress = False
        self.recovery_rotation_duration = 3.0  # Rotate for 3 seconds
        self.recovery_start_time = 0.0
        self.last_goal_target = None
        self.blacklisted_goals = {}  # (x, y) -> expiry_time
        self.visited_goals = []      # list of (x, y) reached successfully
        self.recent_goals = []       # list of (x, y, expiry_time) attempted recently
        
        # Backward scan state (non-blocking)
        self.scan_in_progress = False
        self.scan_step = 0
        self.scan_angle_index = 0
        self.use_servo_scan = self.use_servo_scan
        self.scan_angles = [135, 150, 165]  # Short sweep: LEFT → CENTER → RIGHT
        self.scan_steps_per_angle = 10  # ~0.5s per angle at 20Hz (~1.5s total)
        self.scan_total_time = 0.0
        # Rate limiting for EXPLORE phase (avoid spamming at 20Hz)
        self.last_explore_check = 0.0
        self.explore_check_interval = 0.5  # Check for new frontiers every 0.5s
        
        # Phase state logging (every 5 seconds)
        self.last_phase_log_time = 0.0
        self.phase_log_interval = 5.0  # Log phase state every 5 seconds
        
        # TF listener for pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_valid = False
        self.last_pose = None
        self.last_pose_change_time = time.time()
        self.pose_change_threshold = 0.02  # meters
        self.pose_stale_timeout = 2.0      # seconds since last TF update
        self.pose_stale = False
        self.last_pose_stale_log_time = 0.0
        self.pose_stale_log_interval = 5.0
        self.last_tf_stamp = None
        self.origin_warn_interval = 5.0
        self.last_origin_warn_time = 0.0
        self.startup_time = time.time()
        self.last_frontier_skip_reason = None
        self.last_tf_check_log_time = 0.0
        self.tf_check_log_interval = 5.0
        self.costmap = None
        self.local_costmap = None

        # Nav2 lifecycle state clients
        self.bt_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.controller_state_client = self.create_client(GetState, '/controller_server/get_state')
        self.planner_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.last_odom_time = 0.0
        self.odom_stale_timeout = 1.0  # seconds
        self.last_odom_stale_log_time = 0.0
        self.odom_stale_log_interval = 5.0
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.servo_cmd_pub = self.create_publisher(Float32, '/servo_command', 10)  # For servo sweep angles
        
        # Subscribers
        self.create_subscription(MarkerArray, '/frontiers', self.frontiers_cb, 10)
        distance_sub = self.create_subscription(Float32, '/safety_stop', self.obstacle_distance_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        # Also listen to raw scan to mark lidar freshness in case /scan is delayed
        self.create_subscription(LaserScan, self.scan_raw_topic, self.scan_raw_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_cb, 10)
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.local_costmap_cb, 10)
        self.get_logger().error("✅ SUBSCRIBED to /safety_stop (Arduino safety stop signals)")
        
        # Rear obstacle detection from LiDAR
        self.rear_obstacle_detected = False
        self.last_rear_obstacle_time = 0.0
        
        # Timer for main loop - 20Hz for smooth backward motion
        self.create_timer(0.05, self.main_loop)  # 20 Hz for smooth servo + motion control
        
        self.get_logger().info("🚀 Exploration Coordinator Started (Simplified)")
    
    def frontiers_cb(self, msg: MarkerArray):
        """Store frontier positions"""
        self.current_frontiers = []
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.current_frontiers.append((x, y))

        # If frontiers exist, LiDAR/map is flowing; use as fallback for lidar freshness
        if self.current_frontiers:
            self.last_frontier_time = time.time()
            self.last_scan_time = time.time()
    
    def obstacle_distance_cb(self, msg: Float32):
        """Handle Arduino safety stop signals"""
        if not self.use_ultrasonic_backup:
            return

        distance = msg.data
        now = time.time()

        # Emergency ultrasonic backup at very close range (10cm)
        if distance < 999.0 and distance <= self.ultrasonic_backup_distance:
            self.ultrasonic_emergency_until = now + self.backup_time
            self.obstacle_distance_m = distance
            self.last_obstacle_time = now
            self.get_logger().error(f"🚨 Ultrasonic EMERGENCY! {distance:.3f}m - backing up {self.backup_time:.1f}s")
            return

        # If LiDAR is healthy, ignore ultrasonic (backup only)
        if (now - self.last_scan_time) <= self.lidar_stale_timeout:
            return
        
        # Only react to very close obstacles (backup)
        if distance < 999.0 and distance > self.ultrasonic_backup_distance:
            return
        
        # 999.0 = sentinel value for SAFETY_STOP:0 (obstacle cleared)
        if distance >= 999.0:
            # Latch detection briefly so main loop can react
            if self.obstacle_detected and (now - self.last_obstacle_time) < self.obstacle_hold_time:
                return
            if self.current_phase == Phase.OBSTACLE:
                return
            self.get_logger().info(f"✅ Obstacle cleared (SAFETY_STOP:0)")
            self.obstacle_detected = False
            return
        
        # Otherwise it's a SAFETY_STOP:1 with actual distance (backup only if LiDAR stale)
        self.obstacle_distance_m = distance
        self.last_obstacle_time = now
        self.get_logger().error(f"🚨🚨🚨 SAFETY_STOP TRIGGERED! Obstacle at {distance:.3f}m!")
        self.obstacle_detected = True
    
    def scan_cb(self, msg: LaserScan):
        """Monitor LiDAR for front and rear obstacles"""
        self.last_scan_time = time.time()
        # Check rear 180 degrees (90° left to 90° right from rear = robot's back half)
        # LiDAR angle 0 = front, π/2 = left, -π/2 = right, ±π = rear
        
        # Rear detection zone: 135° to 225° (±45° from rear)
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        rear_obstacle = False
        front_obstacle = False
        min_front_distance = float('inf')
        min_rear_distance = float('inf')
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            # Normalize angle to [-π, π]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Check rear 90° cone (135° to 225° = -π to -3π/4 and 3π/4 to π)
            in_rear_zone = (angle >= 2.35 or angle <= -2.35)  # ±135° to ±180°
            # Check front 60° cone (-30° to +30°)
            in_front_zone = (-0.52 <= angle <= 0.52)
            
            if in_rear_zone:
                if distance < min_rear_distance:
                    min_rear_distance = distance
                if distance < self.rear_obstacle_threshold:
                    rear_obstacle = True

            if self.use_lidar_obstacle and in_front_zone:
                if distance < min_front_distance:
                    min_front_distance = distance
                if distance < self.lidar_obstacle_distance:
                    front_obstacle = True
        
        # Latch rear obstacle detection to stop backing reliably
        if rear_obstacle:
            self.last_rear_obstacle_time = time.time()
            if not self.rear_obstacle_detected:
                self.get_logger().warn(
                    f"⚠️ Rear obstacle detected at {min_rear_distance:.2f}m (threshold: {self.rear_obstacle_threshold:.2f}m)"
                )
            self.rear_obstacle_detected = True
        else:
            if (time.time() - self.last_rear_obstacle_time) > self.rear_obstacle_hold_time:
                self.rear_obstacle_detected = False

        # Front obstacle triggers obstacle handling when using LiDAR
        if self.use_lidar_obstacle and front_obstacle:
            self.obstacle_distance_m = min_front_distance
            self.last_obstacle_time = time.time()
            if not self.obstacle_detected:
                self.get_logger().error(
                    f"🚨 LiDAR obstacle at {min_front_distance:.2f}m (threshold: {self.lidar_obstacle_distance:.2f}m)"
                )
            self.obstacle_detected = True
        elif self.use_lidar_obstacle:
            # Clear obstacle after hold time when front is clear
            if self.obstacle_detected and (time.time() - self.last_obstacle_time) >= self.obstacle_hold_time:
                self.obstacle_detected = False

    def scan_raw_cb(self, msg: LaserScan):
        """Track raw scan timing for startup readiness"""
        self.last_scan_time = time.time()

    def odom_cb(self, msg: Odometry):
        self.last_odom_time = time.time()

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def local_costmap_cb(self, msg: OccupancyGrid):
        self.local_costmap = msg

    def _odom_recent(self) -> bool:
        now = time.time()
        if (now - self.last_odom_time) <= self.odom_stale_timeout:
            return True

        # Fallback: check odom->base TF freshness if /odom topic is missing
        try:
            tf = self.tf_buffer.lookup_transform('odom', self.base_frame, rclpy.time.Time())
            tf_age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            return tf_age <= self.odom_stale_timeout
        except Exception:
            return False

    def _goal_in_free_space(self, x, y):
        if not self.use_costmap_goal_filter or self.costmap is None:
            return True

        info = self.costmap.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False

        index = my * info.width + mx
        value = self.costmap.data[index]

        # Treat unknown or high-cost as not free
        if value < 0:
            return False
        if value >= self.costmap_free_threshold:
            return False

        return True
    
    def update_pose(self):
        """Get robot pose from TF"""
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            old_pose = self.robot_pose
            self.robot_pose = (x, y, 0.0)
            self.pose_valid = True
            self._update_pose_stale_state(x, y, tf.header.stamp)
            
            # Warn if pose remains near origin while odom is updating (throttled)
            if abs(x) < 0.01 and abs(y) < 0.01:
                now = time.time()
                odom_recent = (now - self.last_odom_time) <= self.odom_stale_timeout
                origin_stale = (now - self.last_pose_change_time) >= 5.0
                if odom_recent and origin_stale and (now - self.last_origin_warn_time) >= self.origin_warn_interval:
                    self.last_origin_warn_time = now
                    self.get_logger().warn("⚠️ Robot still at origin (0,0) - odom/TF may not be updating")
        except Exception as e:
            # Fallback to base_link if base_footprint is missing
            try:
                fallback_frame = 'base_link' if self.base_frame != 'base_link' else 'base_footprint'
                tf = self.tf_buffer.lookup_transform('map', fallback_frame, rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                self.robot_pose = (x, y, 0.0)
                self.pose_valid = True
                self._update_pose_stale_state(x, y, tf.header.stamp)
            except Exception as e2:
                self.pose_valid = False
                # Log TF errors to debug localization issues
                self.get_logger().error(f"❌ TF lookup failed (map->{self.base_frame}): {str(e)}")
                self.get_logger().error(f"❌ TF lookup failed (map->{fallback_frame}): {str(e2)}")
                self.get_logger().error(f"   Current pose stuck at: {self.robot_pose}")
                pass  # TF not ready yet

    def _update_pose_stale_state(self, x, y, stamp):
        now = time.time()
        self.last_tf_stamp = stamp
        tf_age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
        self.pose_stale = tf_age > self.pose_stale_timeout
        if self.pose_stale and (now - self.last_pose_stale_log_time) > self.pose_stale_log_interval:
            self.last_pose_stale_log_time = now
            self.get_logger().warn(
                f"⚠️ TF data stale (age {tf_age:.2f}s). Waiting for odom/TF to update."
            )

        if self.last_pose is None:
            self.last_pose = (x, y)
            self.last_pose_change_time = now
            return

        last_x, last_y = self.last_pose
        dist = math.hypot(x - last_x, y - last_y)
        if dist >= self.pose_change_threshold:
            self.last_pose = (x, y)
            self.last_pose_change_time = now
            return

    def _log_startup_diagnostics(self):
        now = time.time()
        if now - self.last_startup_diag_log < self.startup_diag_interval:
            return
        self.last_startup_diag_log = now

        now = time.time()
        frontiers_ready = len(self.current_frontiers) > 0
        lidar_ready = frontiers_ready or ((now - self.last_scan_time) <= self.lidar_stale_timeout) or (
            (now - self.last_frontier_time) <= self.frontier_lidar_fallback_timeout
        )
        frontiers_ready = len(self.current_frontiers) > 0
        pose_ready = self.pose_valid and not self.pose_stale
        pose_moved = math.hypot(self.robot_pose[0], self.robot_pose[1]) >= self.pose_movement_threshold
        odom_recent = self._odom_recent()
        nav2_server_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        nav2_active = self._nav2_active(require_active=True) or nav2_server_ready
        can_map_odom = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        can_odom_base = self.tf_buffer.can_transform('odom', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        clear_for_hold = (time.time() - self.last_obstacle_time) >= self.startup_clear_hold_time
        costmap_ready = (self.costmap is not None) or (self.local_costmap is not None)

        self.get_logger().info(
            "🔎 Startup status: "
            f"lidar={lidar_ready}, frontiers={frontiers_ready}, pose={pose_ready}, pose_moved={pose_moved}, "
            f"odom_recent={odom_recent}, nav2_server={nav2_server_ready}, nav2_active={nav2_active}, "
            f"map->odom={can_map_odom}, odom->{self.base_frame}={can_odom_base}, costmap={costmap_ready}, "
            f"clear={clear_for_hold}"
        )
    
    def pick_best_frontier(self):
        """Pick closest unexplored frontier (with minimum distance threshold)"""
        if not self.current_frontiers:
            self.get_logger().warn("❌ No frontiers available in list")
            return None
        
        self.update_pose()
        if not self.pose_valid:
            self.get_logger().warn("⚠️ No valid TF pose yet - skipping frontier selection")
            self.last_frontier_skip_reason = "pose_invalid"
            return None
        if self.pose_stale:
            self.get_logger().warn("⚠️ TF appears stale - skipping frontier selection")
            self.last_frontier_skip_reason = "pose_stale"
            return None
        robot_x, robot_y, _ = self.robot_pose
        
        self.get_logger().info(f"📍 Robot pose: ({robot_x:.2f}, {robot_y:.2f})")
        
        MIN_FRONTIER_DISTANCE = self.min_frontier_distance
        now = time.time()

        # Prune expired blacklisted goals
        if self.blacklisted_goals:
            expired = [k for k, v in self.blacklisted_goals.items() if v <= now]
            for k in expired:
                self.blacklisted_goals.pop(k, None)

        # Prune expired recent goals
        if self.recent_goals:
            self.recent_goals = [g for g in self.recent_goals if g[2] > now]
        
        # Score = 1/distance (prefer closer, but not too close)
        best = None
        best_score = -999
        valid_frontiers = 0
        for fx, fy in self.current_frontiers:
            dist = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            
            # Skip frontiers that are too close (likely inside robot footprint or planning deadzone)
            if dist < MIN_FRONTIER_DISTANCE:
                continue
            
            # Pull goal inward from frontier so it's inside known free space
            if self.frontier_goal_offset > 0.0:
                if dist <= (self.frontier_goal_offset + 0.05):
                    continue
                ux = (fx - robot_x) / dist
                uy = (fy - robot_y) / dist
                goal_x = fx - (ux * self.frontier_goal_offset)
                goal_y = fy - (uy * self.frontier_goal_offset)
            else:
                goal_x = fx
                goal_y = fy

            # Skip goals that are in occupied/unknown space in global costmap
            if not self._goal_in_free_space(goal_x, goal_y):
                continue

            # Skip goals near recently failed attempts
            if self.blacklisted_goals:
                skip_goal = False
                for (bx, by), _expiry in self.blacklisted_goals.items():
                    if math.hypot(goal_x - bx, goal_y - by) <= self.blacklist_radius:
                        skip_goal = True
                        break
                if skip_goal:
                    continue

            # Skip goals near previously reached targets (avoid revisits)
            if self.avoid_revisit and self.visited_goals:
                too_close = False
                for (vx, vy) in self.visited_goals:
                    if math.hypot(goal_x - vx, goal_y - vy) <= self.visited_goal_radius:
                        too_close = True
                        break
                if too_close:
                    continue

            # Skip goals near recently attempted targets (cooldown)
            if self.recent_goals:
                too_close_recent = False
                for (rx, ry, _expiry) in self.recent_goals:
                    if math.hypot(goal_x - rx, goal_y - ry) <= self.recent_goal_radius:
                        too_close_recent = True
                        break
                if too_close_recent:
                    continue

            valid_frontiers += 1
            dist_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
            score = 1.0 / (dist_goal + 0.1)  # +0.1 to avoid division by zero
            if score > best_score:
                best_score = score
                best = {
                    'frontier': (fx, fy),
                    'goal': (goal_x, goal_y),
                    'dist': dist_goal
                }
        
        if best is None:
            self.get_logger().warn(f"⚠️ No valid frontiers! Total: {len(self.current_frontiers)}, Valid (>{MIN_FRONTIER_DISTANCE}m): 0")
            self.last_frontier_skip_reason = "no_valid_frontiers"
        else:
            fx, fy = best['frontier']
            gx, gy = best['goal']
            dist_to_best = best['dist']
            if self.frontier_goal_offset > 0.0:
                self.get_logger().info(
                    f"✅ Selected frontier ({fx:.2f}, {fy:.2f}) -> goal ({gx:.2f}, {gy:.2f}), distance: {dist_to_best:.2f}m"
                )
            else:
                self.get_logger().info(f"✅ Selected frontier at ({fx:.2f}, {fy:.2f}), distance: {dist_to_best:.2f}m")
            self.last_frontier_skip_reason = None
        
        return best
    
    def send_goal_to_nav2(self, goal_x, goal_y):
        """Send goal to Nav2 navigate_to_pose"""
        # Require recent odom and TF before sending goals
        now = time.time()
        if not self._odom_recent():
            self.get_logger().warn("⚠️ Skipping goal send: /odom/odom TF not updating")
            return False
        if self.pose_stale:
            self.get_logger().warn("⚠️ Skipping goal send: TF is stale")
            return False
        if self.require_costmap and self.costmap is None and self.local_costmap is None:
            self.get_logger().warn("⚠️ Skipping goal send: costmap not ready")
            return False
        nav2_server_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        if self.require_nav2_active and not (self._nav2_active(require_active=True) or nav2_server_ready):
            self.get_logger().warn("⚠️ Skipping goal send: Nav2 not active")
            return False

        # Verify required TF chains exist (map->odom->base_footprint)
        can_map_base = self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.1))
        can_map_odom = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.1))
        if not can_map_base or not can_map_odom:
            if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                self.last_tf_check_log_time = now
                self.get_logger().warn(
                    f"⚠️ Missing TF: map->base_footprint={can_map_base}, map->odom={can_map_odom}. Goal send skipped."
                )
            return False

        # Check Nav2 lifecycle states (informational)
        self._nav2_active()

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 server not ready")
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.last_goal_target = (goal_x, goal_y)
        # Track recently attempted goals to avoid revisits
        self.recent_goals.append((goal_x, goal_y, time.time() + self.recent_goal_hold_time))
        
        self.goal_in_progress = True  # Set flag IMMEDIATELY to prevent duplicate sends
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_cb)
        
        return True

    def _nav2_active(self):
        return self._nav2_active(require_active=False)

    def _nav2_active(self, require_active: bool = False):
        now = time.time()
        if not self.bt_state_client.service_is_ready():
            if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                self.last_tf_check_log_time = now
                self.get_logger().warn("⚠️ bt_navigator lifecycle service not ready")
            return False if require_active else True

        bt_state = self._get_lifecycle_state(self.bt_state_client)
        controller_state = self._get_lifecycle_state(self.controller_state_client)
        planner_state = self._get_lifecycle_state(self.planner_state_client)

        active = (bt_state == 'active' and controller_state == 'active' and planner_state == 'active')
        if not active:
            if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                self.last_tf_check_log_time = now
                self.get_logger().warn(
                    f"⚠️ Nav2 not active: bt={bt_state}, controller={controller_state}, planner={planner_state}"
                )
        return active if require_active else True

    def _get_lifecycle_state(self, client):
        try:
            req = GetState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            if future.result() is None:
                return 'unknown'
            return future.result().current_state.label
        except Exception:
            return 'error'
    
    def goal_response_cb(self, future):
        """Handle Nav2 goal response"""
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("✅ Goal accepted by Nav2")
            self.nav2_ready = True
            # Request result to know when goal completes
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_cb)
        else:
            self.get_logger().warn("❌ Goal rejected by Nav2")
            self.goal_handle = None  # Clear so we can send a new goal
            self.goal_in_progress = False  # Clear flag on rejection
    
    def goal_result_cb(self, future):
        """Handle Nav2 goal completion"""
        result = future.result()
        self.goal_handle = None  # Clear goal handle to allow sending new goals
        self.goal_in_progress = False  # Clear flag to allow new goals
        
        # Update pose to know where we are
        self.update_pose()
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"✅ Reached frontier goal! Robot at ({robot_x:.2f}, {robot_y:.2f})")
            self.consecutive_failures = 0  # Reset failure counter on success
            self.last_successful_goal_pos = (robot_x, robot_y)
            if self.avoid_revisit:
                self.visited_goals.append((robot_x, robot_y))
        elif result.status == 5:  # ABORTED
            self.consecutive_failures += 1
            self.get_logger().error(f"❌ Goal ABORTED! Failure #{self.consecutive_failures}/{self.max_consecutive_failures} at pos ({robot_x:.2f}, {robot_y:.2f})")
            self.get_logger().error(f"   Likely cause: No valid path found by Nav2 planner")

            # Blacklist this goal to avoid immediately retrying the same spot
            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)
            
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().error(f"🚨 STUCK DETECTED! {self.consecutive_failures} consecutive failures - entering RECOVERY mode")
                self.current_phase = Phase.RECOVERY
                self.stuck_recovery_in_progress = True
                self.recovery_start_time = time.time()
        elif result.status == 6:  # CANCELED
            self.get_logger().warn(f"⚠️ Navigation canceled at ({robot_x:.2f}, {robot_y:.2f})")
        else:
            self.get_logger().warn(f"⚠️ Navigation ended with status: {result.status} at ({robot_x:.2f}, {robot_y:.2f})")

    def _blacklist_goal(self, goal_xy):
        now = time.time()
        self.blacklisted_goals[goal_xy] = now + self.blacklist_duration
        self.get_logger().warn(
            f"⚠️ Blacklisting failed goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) for {self.blacklist_duration:.0f}s"
        )
    
    def emergency_backup(self):
        """Obstacle detected - move backward while scanning for clear path"""
        self.get_logger().error(f"🚨 OBSTACLE at {self.obstacle_distance_m:.3f}m - BACKING UP + SCANNING!")
        
        # Cancel Nav2 goal
        if self.goal_handle is not None:
            self.get_logger().error("🛑 Cancelling Nav2 goal")
            try:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested"))
            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
            self.goal_handle = None
            self.goal_in_progress = False
        
        # Stop nav2 commands
        stop_msg = Twist()
        self.cmd_vel_nav_pub.publish(stop_msg)
    
    def start_backward_scan(self):
        """Initialize backward scan - NON-BLOCKING"""
        self.get_logger().info("🔍 Starting BACKWARD + SERVO SCAN...")
        self.scan_in_progress = True
        self.scan_step = 0
        self.scan_angle_index = 0
        self.obstacle_detected = False  # Clear obstacle flag to allow scan to proceed
        # Compute total scan time for RESCAN timing logic
        self.scan_total_time = len(self.scan_angles) * self.scan_steps_per_angle * 0.05
        
    def execute_scan_step(self):
        """Execute one step of backward scan - called from main loop (NON-BLOCKING)"""
        if not self.scan_in_progress:
            return False  # Scan not active
        
        # SAFETY: Check for rear obstacles before moving backward
        if self.rear_obstacle_detected:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.scan_in_progress = False
            self.get_logger().error("🛑 REAR OBSTACLE DETECTED! Stopping backward motion.")
            return False  # Abort scan due to rear obstacle
        
        # Check if we've finished all angles
        if self.scan_angle_index >= len(self.scan_angles):
            # Scan complete - stop motion
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.scan_in_progress = False
            self.get_logger().info("✅ Backward scan complete")
            return False  # Scan finished
        
        # Start new angle
        if self.scan_step == 0 and self.use_servo_scan:
            angle = self.scan_angles[self.scan_angle_index]
            servo_msg = Float32()
            servo_msg.data = float(angle)
            self.servo_cmd_pub.publish(servo_msg)
            self.get_logger().info(f"  ➜ Servo: {angle}° (backing up)")
        
        # Move backward continuously
        backup_msg = Twist()
        backup_msg.linear.x = -0.06  # Short, gentle backward
        self.cmd_vel_pub.publish(backup_msg)
        
        self.scan_step += 1
        
        # Check if angle duration complete
        if self.scan_step >= self.scan_steps_per_angle:
            self.scan_step = 0
            self.scan_angle_index += 1
        
        return True  # Scan still in progress
    
    def main_loop(self):
        """Main exploration state machine"""
        # Ultrasonic emergency backup (2s) at very close range
        if time.time() < self.ultrasonic_emergency_until:
            if self.rear_obstacle_detected:
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                return
            backup_msg = Twist()
            backup_msg.linear.x = self.backup_speed
            self.cmd_vel_pub.publish(backup_msg)
            return
        # Global safety stop handling: custom obstacle handling only when enabled
        if (not self.nav2_handles_obstacles and self.obstacle_detected and
                self.current_phase not in (Phase.OBSTACLE, Phase.RESCAN)):
            self.get_logger().error(
                f"💥 SAFETY_STOP detected ({self.obstacle_distance_m:.3f}m) - forcing OBSTACLE phase"
            )
            self.current_phase = Phase.OBSTACLE
            self.phase_start_time = time.time()
            return

        # Periodic phase state logging (every 5 seconds)
        now = time.time()
        if now - self.last_phase_log_time >= self.phase_log_interval:
            self.last_phase_log_time = now
            phase_name = self.current_phase.name
            frontier_count = len(self.current_frontiers)
            self.get_logger().info(f"📊 Phase: {phase_name} | Frontiers: {frontier_count} | Failures: {self.consecutive_failures}/{self.max_consecutive_failures}")
            # Periodic odom freshness check
            if not self._odom_recent():
                if now - self.last_odom_stale_log_time > self.odom_stale_log_interval:
                    self.last_odom_stale_log_time = now
                    self.get_logger().warn("⚠️ No /odom or odom TF updates in the last 1s. Check Arduino bridge and TF.")
        
        # ==== PHASE 1: INITIALIZATION ====
        if self.current_phase == Phase.INIT:
            # Throttled startup status logging
            now = time.time()
            if now - self.last_startup_status_log >= self.startup_status_log_interval:
                self.last_startup_status_log = now
                self.get_logger().info("⏳ Waiting for LiDAR, frontiers, pose, and Nav2...")
            # Detailed startup diagnostics
            self._log_startup_diagnostics()
            
            # Start timer when we first detect frontiers
            if self.current_frontiers and self.phase_start_time is None:
                self.phase_start_time = time.time()
                self.get_logger().info(f"✅ Frontiers detected! Waiting {self.nav2_activation_time}s for Nav2 lifecycle activation...")
                self.first_frontier_time = self.phase_start_time
            
            # Start initial scan window once LiDAR is available
            now = time.time()
            frontiers_ready = len(self.current_frontiers) > 0
            lidar_ready = frontiers_ready or ((now - self.last_scan_time) <= self.lidar_stale_timeout) or (
                (now - self.last_frontier_time) <= self.frontier_lidar_fallback_timeout
            )
            if self.startup_scan_end_time is None and lidar_ready:
                self.startup_scan_end_time = time.time() + self.startup_scan_time
                self.get_logger().info(f"🔍 Startup scan window: {self.startup_scan_time:.1f}s")
                self.first_lidar_time = time.time()

            # Update pose during init to verify localization
            self.update_pose()
            if self.pose_valid and not self.pose_stale and self.first_pose_time is None:
                self.first_pose_time = time.time()
            
            # Wait for Nav2 to be ready AND fully activated
            if self.nav_client.wait_for_server(timeout_sec=1.0) and self.phase_start_time is not None:
                init_time_elapsed = time.time() - self.phase_start_time
                scan_ready = (self.startup_scan_end_time is not None and time.time() >= self.startup_scan_end_time)
                now = time.time()
                frontiers_ready = len(self.current_frontiers) > 0
                lidar_ready = frontiers_ready or ((now - self.last_scan_time) <= self.lidar_stale_timeout) or (
                    (now - self.last_frontier_time) <= self.frontier_lidar_fallback_timeout
                )
                pose_ready = self.pose_valid and not self.pose_stale
                pose_moved = math.hypot(self.robot_pose[0], self.robot_pose[1]) >= self.pose_movement_threshold
                costmap_ready = (self.costmap is not None or self.local_costmap is not None) or not self.require_costmap
                nav2_active = (self._nav2_active(require_active=True) or self.nav_client.wait_for_server(timeout_sec=0.1)) or not self.require_nav2_active
                clear_for_hold = (time.time() - self.last_obstacle_time) >= self.startup_clear_hold_time
                if init_time_elapsed >= self.nav2_activation_time and scan_ready and lidar_ready and frontiers_ready and pose_ready and costmap_ready and nav2_active and clear_for_hold and not self.obstacle_detected:
                    self.get_logger().info("✅ Startup scan complete, pose valid, path clear. Starting exploration")
                    self.current_phase = Phase.EXPLORE
                    self.phase_start_time = time.time()
        
        # ==== PHASE 2: EXPLORE ====
        elif self.current_phase == Phase.EXPLORE:
            # Check if obstacles detected - TRIGGER IMMEDIATE AVOIDANCE (only in EXPLORE phase)
            if not self.nav2_handles_obstacles and self.obstacle_detected:
                self.get_logger().error(f"💥💥💥 OBSTACLE at {self.obstacle_distance_m:.3f}m - OBSTACLE PHASE!")
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = time.time()
                return
            
            # Rate limit frontier checks to avoid spamming at 20Hz
            now = time.time()
            if now - self.last_explore_check < self.explore_check_interval:
                return  # Skip this cycle
            self.last_explore_check = now
            
            # Check if we have an active goal or one in progress - wait for it
            if self.goal_handle is not None or self.goal_in_progress:
                return
            
            # Check cooldown - don't spam goals
            time_since_last_goal = now - self.last_goal_time
            if time_since_last_goal < self.goal_cooldown:
                # Still in cooldown period, wait
                return
            
            # Pick best frontier
            goal_info = self.pick_best_frontier()
            if goal_info is None:
                if self.last_frontier_skip_reason in ("pose_invalid", "pose_stale"):
                    return

                self.no_frontier_cycles += 1
                if self.no_frontier_cycles > 10:
                    total_frontiers = len(self.current_frontiers)
                    self.get_logger().info(f"🎉 EXPLORATION COMPLETE! No valid frontiers remaining.")
                    self.get_logger().info(f"   Final frontier count: {total_frontiers}")
                    self.get_logger().info(f"   All accessible areas have been explored!")
                    self.current_phase = Phase.DONE
                else:
                    self.update_pose()
                    robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
                    self.get_logger().warn(f"⚠️ No valid frontiers! (cycle {self.no_frontier_cycles}/10) Robot at ({robot_x:.2f}, {robot_y:.2f})")
                return
            
            self.no_frontier_cycles = 0
            goal_x, goal_y = goal_info['goal']
            frontier_x, frontier_y = goal_info['frontier']
            total_frontiers = len(self.current_frontiers)
            self.update_pose()
            robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
            distance_to_goal = goal_info['dist']
            self.get_logger().info(
                f"🎯 Sending goal: ({goal_x:.2f}, {goal_y:.2f}) from frontier ({frontier_x:.2f}, {frontier_y:.2f}) | Distance: {distance_to_goal:.2f}m | {total_frontiers} frontiers"
            )
            self.last_goal_time = now  # Record goal send time
            self.send_goal_to_nav2(goal_x, goal_y)
        
        # ==== PHASE 3: OBSTACLE HANDLING ====
        elif self.current_phase == Phase.OBSTACLE:
            self.get_logger().error("📍 OBSTACLE PHASE: Cancel goal and prepare for RESCAN")
            self.emergency_backup()
            # Switch to RESCAN phase (will execute on next loop)
            self.rescan_done = False  # Reset flag so scan will execute
            self.current_phase = Phase.RESCAN
            self.phase_start_time = time.time()
        
        # ==== PHASE 4: RESCAN ====
        elif self.current_phase == Phase.RESCAN:
            elapsed = time.time() - self.phase_start_time
            
            # Start scan on first entry
            if not self.rescan_done:
                self.get_logger().info("📍 RESCAN: Executing backward scan now...")
                self.start_backward_scan()
                self.rescan_done = True
            
            # Execute scan steps continuously (NON-BLOCKING)
            scan_active = self.execute_scan_step()
            
            # If scan is done, check if path is clear
            if not scan_active:
                # Allow 1 second after scan before checking clearance
                if elapsed < (self.scan_total_time + 0.5):  # Scan duration + hold time
                    return
                
                # Check if path is now clear
                clear_for_hold = (time.time() - self.last_obstacle_time) >= self.obstacle_hold_time
                if not self.obstacle_detected and clear_for_hold:
                    self.get_logger().info("✅ Path clear after rescan! Looking for new frontier...")
                    
                    # Pick new frontier to explore
                    new_frontier = self.pick_best_frontier()
                    if new_frontier:
                        fx, fy = new_frontier['frontier']
                        gx, gy = new_frontier['goal']
                        self.get_logger().info(
                            f"✅ Resuming exploration at frontier ({fx:.2f}, {fy:.2f}) -> goal ({gx:.2f}, {gy:.2f})"
                        )
                        self.current_phase = Phase.EXPLORE
                    else:
                        self.get_logger().info("🎉 No frontiers available - exploration complete")
                        self.current_phase = Phase.DONE
                elif elapsed > 8.0:
                    # Still blocked after 8 seconds, retry
                    self.get_logger().error("⚠️ RESCAN: Path STILL blocked after 8s - retrying backup")
                    self.current_phase = Phase.OBSTACLE
                    self.rescan_done = False
        
        # ==== PHASE 5: RECOVERY (Stuck in corner) ====
        elif self.current_phase == Phase.RECOVERY:
            elapsed = time.time() - self.recovery_start_time
            
            if elapsed < self.recovery_rotation_duration:
                # Rotate in place to scan surroundings
                rotate_msg = Twist()
                rotate_msg.angular.z = 0.5  # Rotate at 0.5 rad/s
                self.cmd_vel_pub.publish(rotate_msg)
                if int(elapsed * 2) % 2 == 0:  # Log every 0.5 seconds
                    self.get_logger().info(f"🔄 RECOVERY: Rotating in place ({elapsed:.1f}s/{self.recovery_rotation_duration}s)")
            else:
                # Stop rotation
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                self.get_logger().info("✅ RECOVERY complete - resetting failure counter and picking new frontier")
                self.consecutive_failures = 0  # Reset after recovery
                self.stuck_recovery_in_progress = False
                self.current_phase = Phase.EXPLORE
                self.phase_start_time = time.time()
        
        # ==== PHASE 6: DONE ====
        elif self.current_phase == Phase.DONE:
            self.get_logger().info("🏁 Mission complete!")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
