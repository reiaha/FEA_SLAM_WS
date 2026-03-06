from geometry_msgs.msg import Twist
#!/usr/bin/env python3

import rclpy
import os
import os
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
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
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy

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
        self.initial_pose_set = False
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)

    def initialpose_cb(self, msg):
        self.initial_pose_set = True
        self.get_logger().info("✅ Initial pose received. Exploration can begin.")
    def __init__(self):
        super().__init__('exploration_coordinator_v2')
        self.initial_pose_set = False
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.max_total_cells = 0  # Track the largest map size seen
        # Parameters
        self.declare_parameter('nav2_timeout', 12.0)
        self.declare_parameter('obstacle_distance', 0.35)      # meters (35cm - earlier stop)
        self.declare_parameter('backup_speed', -0.3)           # m/s (gentler backward)
        self.declare_parameter('backup_time', 2.0)             # seconds (2 seconds duration)
        self.declare_parameter('use_lidar_obstacle', True)
        self.declare_parameter('strict_obstacle_handling', True)
        self.declare_parameter('use_ultrasonic_backup', True)
        self.declare_parameter('scan_min_range', 0.27)           # m: ignore closer readings (filters chassis self-hits)
        self.declare_parameter('lidar_obstacle_distance', 0.45)   # meters (wider trigger distance for earlier stop)
        self.declare_parameter('ultrasonic_backup_distance', 0.15)  # meters
        self.declare_parameter('ultrasonic_confirm_count', 2)     # consecutive hits to confirm obstacle
        self.declare_parameter('ultrasonic_clear_confirm_count', 2)  # consecutive clears to reset obstacle
        self.declare_parameter('ultrasonic_min_valid_distance', 0.02)  # reject invalid tiny spikes
        self.declare_parameter('ultrasonic_max_jump', 0.35)       # meters, reject sudden jump spikes
        self.declare_parameter('ultrasonic_jump_window', 0.20)    # seconds for jump check window
        self.declare_parameter('lidar_motion_window', 0.8)        # seconds
        self.declare_parameter('lidar_motion_distance_delta', 0.08)  # meters
        self.declare_parameter('lidar_obstacle_class_log_interval', 2.0)  # seconds
        self.declare_parameter('lidar_stale_timeout', 2.0)       # seconds
        self.declare_parameter('lidar_backup_on_obstacle', True)
        self.declare_parameter('swap_lidar_front_back', False)
        self.declare_parameter('rear_obstacle_threshold', 0.25)   # meters (rear safety)
        self.declare_parameter('rear_obstacle_hold_time', 0.8)    # seconds (latch rear obstacle)
        self.declare_parameter('front_obstacle_half_angle_deg', 40.0)
        self.declare_parameter('rear_obstacle_half_angle_deg', 30.0)
        self.declare_parameter('front_obstacle_min_hits', 2)    # consecutive hits to confirm obstacle
        self.declare_parameter('rear_obstacle_min_hits', 2)
        self.declare_parameter('rear_emergency_distance', 0.25)   # meters (hard stop if closer)
        self.declare_parameter('front_emergency_rotate_distance', 0.30)  # meters (rotate instead of backup when very close in front)
        self.declare_parameter('frontier_goal_offset', 0.35)   # meters (pull goal into free space)
        self.declare_parameter('min_frontier_distance', 0.8)   # meters (avoid very close goals)
        self.declare_parameter('relaxed_frontier_after_cycles', 4)
        self.declare_parameter('relaxed_min_frontier_distance', 0.15)
        self.declare_parameter('relaxed_frontier_goal_offset', 0.05)
        self.declare_parameter('relaxed_use_costmap_filter', False)
        self.declare_parameter('blacklist_duration', 30.0)     # seconds (avoid failed goals)
        self.declare_parameter('blacklist_radius', 0.4)        # meters (treat nearby goals as same)
        self.declare_parameter('avoid_revisit', True)          # skip goals near previously reached ones
        self.declare_parameter('strict_no_revisit', True)      # avoid revisiting already-attempted regions
        self.declare_parameter('force_frontier_goal', True)    # always target detected frontier point
        self.declare_parameter('visited_goal_radius', 0.6)     # meters (radius to treat as visited)
        self.declare_parameter('known_frontier_avoid_radius', 1.2)  # meters (avoid known explored frontier areas)
        self.declare_parameter('recent_goal_radius', 0.8)      # meters (avoid recently attempted goals)
        self.declare_parameter('recent_goal_hold_time', 90.0)  # seconds (cooldown for attempted goals)
        self.declare_parameter('nav2_handles_obstacles', True) # let Nav2 handle obstacle avoidance
        self.declare_parameter('base_frame', 'base_footprint') # TF base frame
        self.declare_parameter('scan_topic', '/scan')    # scan topic used for obstacles
        self.declare_parameter('scan_raw_topic', '/scan_raw')  # raw scan topic (timestamp fix source)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('costmap_raw_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('local_costmap_raw_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('frontier_lidar_fallback_timeout', 2.0)
        self.declare_parameter('costmap_free_threshold', 90)
        self.declare_parameter('use_costmap_goal_filter', True)
        self.declare_parameter('pose_movement_threshold', 0.05)
        self.declare_parameter('require_costmap', True)
        self.declare_parameter('costmap_wait_timeout', 10.0)
        self.declare_parameter('require_nav2_active', False)
        self.declare_parameter('nav2_state_timeout_sec', 0.5)
        self.declare_parameter('nav2_active_grace_sec', 8.0)
        self.declare_parameter('nav2_active_fallback_on_action', True)
        self.declare_parameter('enable_simple_exploration', True)  # Enable simple exploration when Nav2 fails
        self.declare_parameter('stop_when_no_frontiers', True)
        self.declare_parameter('no_frontier_recovery_cycles', 2)
        self.declare_parameter('no_frontier_complete_cycles', 3)
        self.declare_parameter('corner_recovery_time', 1.5)
        self.declare_parameter('corner_recovery_turn_speed', 0.4)
        self.declare_parameter('coverage_complete_percent', 90.0)
        self.declare_parameter('zero_frontier_complete_percent', 80.0)
        self.declare_parameter('min_goals_for_complete', 3)
        self.declare_parameter('small_test_mode', False)
        self.declare_parameter('complete_on_zero_frontiers', False)
        self.declare_parameter('auto_save_on_complete', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_save_dir', '/home/pi/FEA_SLAM_WS/saved_maps')
        self.declare_parameter('map_save_name', 'auto_explore_map')
        self.declare_parameter('mapped_area_file', 'auto_explore_area.csv')
        self.declare_parameter('robot_path_file', 'auto_explore_path.csv')
        self.declare_parameter('path_record_min_dist', 0.2)  # metres between recorded waypoints
        self.declare_parameter('replan_on_frontier_update', True)
        self.declare_parameter('replan_interval', 6.0)
        self.declare_parameter('replan_goal_change_distance', 1.0)
        self.declare_parameter('replan_min_improvement', 0.3)
        self.declare_parameter('always_replan_on_frontier_update', False)
        self.declare_parameter('replan_hard_min_interval', 8.0)
        self.declare_parameter('replan_cancel_cooldown', 1.0)
        self.declare_parameter('cancel_active_goal_for_replan', False)
        self.declare_parameter('goal_commit_before_replan_cancel', 10.0)
        self.declare_parameter('allow_costmapless_replan_candidates', False)
        self.declare_parameter('bootstrap_fallback_min_distance', 0.75)
        self.declare_parameter('avoid_return_radius', 1.0)
        self.declare_parameter('avoid_last_goal_radius', 0.8)
        self.declare_parameter('frontier_pick_farthest', True)
        self.declare_parameter('frontier_selection_method', 'astar')
        self.declare_parameter('astar_max_candidates', 30)
        self.declare_parameter('astar_max_expansions', 12000)
        self.declare_parameter('clear_costmap_on_obstacle', True)
        self.declare_parameter('costmap_clear_cooldown', 2.0)
        self.declare_parameter('frontier_debug', True)
        self.declare_parameter('frontier_debug_log_interval', 2.0)
        self.declare_parameter('startup_clear_hold_time', 0.0)
        self.declare_parameter('startup_frontier_timeout', 20.0)
        self.declare_parameter('auto_publish_initial_pose', True)
        self.declare_parameter('auto_publish_initial_pose_timeout', 15.0)
        
        self.nav2_timeout = self.get_parameter('nav2_timeout').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.backup_speed = self.get_parameter('backup_speed').value
        self.backup_time = self.get_parameter('backup_time').value
        self.backup_iterations = int(self.backup_time / 0.05)  # 40 at 20Hz
        self.use_lidar_obstacle = self.get_parameter('use_lidar_obstacle').value
        self.strict_obstacle_handling = bool(self.get_parameter('strict_obstacle_handling').value)
        self.use_ultrasonic_backup = self.get_parameter('use_ultrasonic_backup').value
        self.scan_min_range = float(self.get_parameter('scan_min_range').value)
        self.lidar_obstacle_distance = self.get_parameter('lidar_obstacle_distance').value
        self.ultrasonic_backup_distance = self.get_parameter('ultrasonic_backup_distance').value
        self.ultrasonic_confirm_count = int(self.get_parameter('ultrasonic_confirm_count').value)
        self.ultrasonic_clear_confirm_count = int(self.get_parameter('ultrasonic_clear_confirm_count').value)
        self.ultrasonic_min_valid_distance = float(self.get_parameter('ultrasonic_min_valid_distance').value)
        self.ultrasonic_max_jump = float(self.get_parameter('ultrasonic_max_jump').value)
        self.ultrasonic_jump_window = float(self.get_parameter('ultrasonic_jump_window').value)
        self.lidar_motion_window = float(self.get_parameter('lidar_motion_window').value)
        self.lidar_motion_distance_delta = float(self.get_parameter('lidar_motion_distance_delta').value)
        self.lidar_obstacle_class_log_interval = float(self.get_parameter('lidar_obstacle_class_log_interval').value)
        self.lidar_stale_timeout = self.get_parameter('lidar_stale_timeout').value
        self.lidar_backup_on_obstacle = self.get_parameter('lidar_backup_on_obstacle').value
        self.swap_lidar_front_back = bool(self.get_parameter('swap_lidar_front_back').value)
        self.rear_obstacle_threshold = self.get_parameter('rear_obstacle_threshold').value
        self.rear_obstacle_hold_time = self.get_parameter('rear_obstacle_hold_time').value
        self.front_obstacle_half_angle = math.radians(float(self.get_parameter('front_obstacle_half_angle_deg').value))
        self.rear_obstacle_half_angle = math.radians(float(self.get_parameter('rear_obstacle_half_angle_deg').value))
        self.front_obstacle_min_hits = int(self.get_parameter('front_obstacle_min_hits').value)
        self.rear_obstacle_min_hits = int(self.get_parameter('rear_obstacle_min_hits').value)
        self.rear_emergency_distance = self.get_parameter('rear_emergency_distance').value
        self.front_emergency_rotate_distance = float(self.get_parameter('front_emergency_rotate_distance').value)
        self.frontier_goal_offset = self.get_parameter('frontier_goal_offset').value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').value
        self.relaxed_frontier_after_cycles = int(self.get_parameter('relaxed_frontier_after_cycles').value)
        self.relaxed_min_frontier_distance = float(self.get_parameter('relaxed_min_frontier_distance').value)
        self.relaxed_frontier_goal_offset = float(self.get_parameter('relaxed_frontier_goal_offset').value)
        self.relaxed_use_costmap_filter = bool(self.get_parameter('relaxed_use_costmap_filter').value)
        self.blacklist_duration = self.get_parameter('blacklist_duration').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.avoid_revisit = self.get_parameter('avoid_revisit').value
        self.strict_no_revisit = bool(self.get_parameter('strict_no_revisit').value)
        self.force_frontier_goal = bool(self.get_parameter('force_frontier_goal').value)
        self.visited_goal_radius = self.get_parameter('visited_goal_radius').value
        self.known_frontier_avoid_radius = float(self.get_parameter('known_frontier_avoid_radius').value)
        self.recent_goal_radius = float(self.get_parameter('recent_goal_radius').value)
        self.recent_goal_hold_time = float(self.get_parameter('recent_goal_hold_time').value)
        self.nav2_handles_obstacles = self.get_parameter('nav2_handles_obstacles').value
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_raw_topic = self.get_parameter('scan_raw_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.costmap_raw_topic = self.get_parameter('costmap_raw_topic').value
        self.local_costmap_raw_topic = self.get_parameter('local_costmap_raw_topic').value
        self.frontier_lidar_fallback_timeout = float(self.get_parameter('frontier_lidar_fallback_timeout').value)
        self.costmap_free_threshold = int(self.get_parameter('costmap_free_threshold').value)
        self.use_costmap_goal_filter = self.get_parameter('use_costmap_goal_filter').value
        self.pose_movement_threshold = 0.0  # Allow exploration from origin
        self.require_costmap = self.get_parameter('require_costmap').value
        self.costmap_wait_timeout = float(self.get_parameter('costmap_wait_timeout').value)
        self.require_nav2_active = self.get_parameter('require_nav2_active').value  # Properly use the parameter
        self.nav2_state_timeout_sec = float(self.get_parameter('nav2_state_timeout_sec').value)
        self.nav2_active_grace_sec = float(self.get_parameter('nav2_active_grace_sec').value)
        self.nav2_active_fallback_on_action = bool(self.get_parameter('nav2_active_fallback_on_action').value)
        self.stop_when_no_frontiers = self.get_parameter('stop_when_no_frontiers').value
        self.no_frontier_recovery_cycles = int(self.get_parameter('no_frontier_recovery_cycles').value)
        self.no_frontier_complete_cycles = int(self.get_parameter('no_frontier_complete_cycles').value)
        self.corner_recovery_time = float(self.get_parameter('corner_recovery_time').value)
        self.corner_recovery_turn_speed = float(self.get_parameter('corner_recovery_turn_speed').value)
        self.coverage_complete_percent = float(self.get_parameter('coverage_complete_percent').value)
        self.zero_frontier_complete_percent = float(self.get_parameter('zero_frontier_complete_percent').value)
        self.min_goals_for_complete = int(self.get_parameter('min_goals_for_complete').value)
        self.small_test_mode = bool(self.get_parameter('small_test_mode').value)
        self.complete_on_zero_frontiers = bool(self.get_parameter('complete_on_zero_frontiers').value)
        self.enable_simple_exploration = self.get_parameter('enable_simple_exploration').value
        self.auto_save_on_complete = self.get_parameter('auto_save_on_complete').value
        self.map_topic = self.get_parameter('map_topic').value
        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.map_save_name = self.get_parameter('map_save_name').value
        self.mapped_area_file = self.get_parameter('mapped_area_file').value
        self.robot_path_file = self.get_parameter('robot_path_file').value
        self._path_record_min_dist_val = float(self.get_parameter('path_record_min_dist').value)
        self.replan_on_frontier_update = self.get_parameter('replan_on_frontier_update').value
        self.replan_interval = float(self.get_parameter('replan_interval').value)
        self.replan_goal_change_distance = float(self.get_parameter('replan_goal_change_distance').value)
        self.replan_min_improvement = float(self.get_parameter('replan_min_improvement').value)
        self.always_replan_on_frontier_update = bool(self.get_parameter('always_replan_on_frontier_update').value)
        self.replan_hard_min_interval = float(self.get_parameter('replan_hard_min_interval').value)
        self.replan_cancel_cooldown = float(self.get_parameter('replan_cancel_cooldown').value)
        self.cancel_active_goal_for_replan = bool(self.get_parameter('cancel_active_goal_for_replan').value)
        self.goal_commit_before_replan_cancel = float(self.get_parameter('goal_commit_before_replan_cancel').value)
        self.allow_costmapless_replan_candidates = bool(self.get_parameter('allow_costmapless_replan_candidates').value)
        self.bootstrap_fallback_min_distance = float(self.get_parameter('bootstrap_fallback_min_distance').value)
        self.avoid_return_radius = float(self.get_parameter('avoid_return_radius').value)
        self.avoid_last_goal_radius = float(self.get_parameter('avoid_last_goal_radius').value)
        self.frontier_pick_farthest = bool(self.get_parameter('frontier_pick_farthest').value)
        self.frontier_selection_method = self.get_parameter('frontier_selection_method').value
        self.astar_max_candidates = int(self.get_parameter('astar_max_candidates').value)
        self.astar_max_expansions = int(self.get_parameter('astar_max_expansions').value)
        self.clear_costmap_on_obstacle = bool(self.get_parameter('clear_costmap_on_obstacle').value)
        self.costmap_clear_cooldown = float(self.get_parameter('costmap_clear_cooldown').value)
        self.frontier_debug = bool(self.get_parameter('frontier_debug').value)
        self.frontier_debug_log_interval = float(self.get_parameter('frontier_debug_log_interval').value)
        self.startup_clear_hold_time = float(self.get_parameter('startup_clear_hold_time').value)
        self.startup_frontier_timeout = float(self.get_parameter('startup_frontier_timeout').value)
        self.auto_publish_initial_pose = bool(self.get_parameter('auto_publish_initial_pose').value)
        self.auto_publish_initial_pose_timeout = float(self.get_parameter('auto_publish_initial_pose_timeout').value)
        self.costmap_waited_out = False
        if self.strict_no_revisit:
            self.avoid_revisit = True
        
        # State
        self.startup_time = time.time()
        self.init_start_time = self.startup_time
        self.nav2_activation_time = 5.0
        self.current_phase = Phase.INIT
        self.phase_start_time = None  # Will be set when we detect frontiers
        self.robot_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.home_pose = None               # recorded when exploration starts
        self.current_frontiers = []
        self.obstacle_detected = False
        self.obstacle_distance_m = float('inf')
        self.last_obstacle_time = 0.0
        self.obstacle_hold_time = 1.0  # seconds to latch detection
        self.ultrasonic_obstacle_hits = 0
        self.ultrasonic_clear_hits = 0
        self.last_ultrasonic_distance = None
        self.last_ultrasonic_time = 0.0
        self.last_ultrasonic_filter_log_time = 0.0
        self.ultrasonic_filter_log_interval = 2.0
        self.front_obstacle_detected = False
        self.last_front_obstacle_time = 0.0
        self.last_lidar_front_distance = None
        self.last_lidar_front_time = 0.0
        self.current_lidar_obstacle_type = "unknown"
        self.last_lidar_obstacle_class_log_time = 0.0
        self.last_scan_time = 0.0
        self.rescan_done = False
        self.ultrasonic_emergency_until = 0.0
        self.nav2_ready = False
        self.startup_scan_end_time = None
        self.first_lidar_time = None
        self.first_frontier_time = None
        self.first_pose_time = None
        self.startup_frontier_timeout_warned = False
        self.auto_initial_pose_published = False
        self.last_auto_pose_log_time = 0.0
        self.last_frontier_time = 0.0
        self.last_startup_status_log = 0.0
        self.startup_status_log_interval = 2.0
        self.startup_diag_interval = 2.0
        self.last_startup_diag_log = 0.0
        self.goal_handle = None
        self.goal_in_progress = False  # Flag: prevents sending goals while waiting for callback
        self.no_frontier_cycles = 0
        self.corner_recovery_until = 0.0
        self.corner_recovery_mode = None
        self.last_goal_time = 0.0  # Track when we last sent a goal
        # Lethal-space escape: back up when same position fails N times
        self._lethal_fail_pos = None            # (x, y) of last planning failure
        self._lethal_fail_count = 0              # consecutive failures from same position
        # Static-obstacle deep escape
        self.static_stuck_start = 0.0          # wall-clock when STATIC first appeared
        self.static_stuck_escape_active = False
        self.static_stuck_escape_step = 0      # 0=backup, 1=turn
        self.static_stuck_escape_step_start = 0.0
        self.static_stuck_escape_backup_dur = 1.5   # seconds to reverse
        self.static_stuck_escape_turn_dur = 2.5     # seconds to rotate after backup
        self.static_stuck_escape_turn_dir = 1.0
        self.static_stuck_escape_attempts = 0       # alternates direction each time
        self.static_stuck_trigger_time = 8.0        # trigger after this many seconds stuck
        self.last_costmap_clear_time = 0.0
        self.goal_cooldown = 0.5   # Reduced from 3s for faster exploration
        self.ever_had_frontiers = False
        self.ever_sent_goal = False
        self.last_done_log_time = 0.0
        self.done_log_interval = 5.0
        self.simple_exploration_active = False  # Flag for simple exploration mode
        self.simple_exploration_start_time = 0.0
        self.simple_exploration_duration = 10.0  # 10 seconds of simple exploration
        self.simple_exploration_turn_time = 2.0  # Turn for 2 seconds
        self.simple_exploration_move_time = 3.0  # Move for 3 seconds
        self.recovery_turn_dir = 1.0
        self.map_save_requested = False
        self.complete_marker_sent = False
        self._exploration_complete_lock = __import__('threading').Lock()
        self._exploration_complete_handled = False
        self.mapped_area_series = []
        self.last_mapped_area_m2 = 0.0
        self.last_percent_known = 0.0
        self.robot_path_series = []          # list of (elapsed_s, x, y)
        self.last_recorded_path_x = None    # last recorded x for distance check
        self.last_recorded_path_y = None    # last recorded y for distance check
        
        # EXPLORATION TRACKING - ensure bot actually explores area
        self.goals_reached = 0              # Track how many goals actually reached
        self.last_frontier_check_time = 0.0 # Track when we last checked frontiers
        self.frontier_stable_count = 0       # Count consecutive times frontiers unchanged
        self.min_frontier_stable_time = 5.0  # Need 5 seconds of stable frontiers before declaring complete
        self.exploration_start_time = 0.0   # When exploration actually started
        self.nav2_fully_active = False       # Track if Nav2 lifecycle is active
        self.last_known_frontier_count = 0   # Track frontier count for stability check
        self.obstacle_blocking_start = 0.0  # When obstacle blocking started
        self.max_obstacle_block_time = 10.0  # Max time to wait for obstacle to clear (10s)
        
        # Stuck detection and recovery
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3  # After 3 failed goals, trigger recovery
        self.last_successful_goal_pos = (0.0, 0.0)
        self.stuck_recovery_in_progress = False
        self.recovery_rotation_duration = 3.0  # Rotate for 3 seconds
        self.recovery_start_time = 0.0
        self.last_goal_target = None
        self.blacklisted_goals = {}  # (x, y) -> expiry_time

        if self.small_test_mode:
            self.coverage_complete_percent = min(self.coverage_complete_percent, 75.0)
            self.min_goals_for_complete = min(self.min_goals_for_complete, 1)
            self.no_frontier_complete_cycles = min(self.no_frontier_complete_cycles, 3)
            self.complete_on_zero_frontiers = True
            self.get_logger().warn(
                "🧪 SMALL TEST MODE enabled: faster completion criteria for 1m validation"
            )

        self.get_logger().info(
            f"🎯 Completion criteria: coverage>={self.coverage_complete_percent:.1f}% "
            f"and goals>={self.min_goals_for_complete}, "
            f"no_frontier_cycles={self.no_frontier_complete_cycles}, "
            f"complete_on_zero_frontiers={self.complete_on_zero_frontiers}"
        )
        self.visited_goals = []      # list of (x, y) reached successfully
        self.recent_goals = []       # list of (x, y, expiry_time) attempted recently
        self.strict_avoid_goals = [] # list of (x, y) attempted/reached (no expiry)
        self.attempted_frontiers = []  # list of (x, y) frontier targets (no expiry)
        
        # Backward scan state (non-blocking)
        self.scan_in_progress = False
        self.scan_step = 0
        self.scan_segment_index = 0
        self.scan_segments = 3  # keep short backward scan duration without servo
        self.scan_steps_per_angle = 10  # ~0.5s per angle at 20Hz (~1.5s total)
        self.scan_total_time = 0.0
        # Rate limiting for EXPLORE phase (avoid spamming at 20Hz)
        self.last_explore_check = 0.0
        self.explore_check_interval = 0.5  # Check for new frontiers every 0.5s
        
        # Phase state logging (every 5 seconds)
        self.last_phase_log_time = 0.0
        self.phase_log_interval = 5.0  # Log phase state every 5 seconds
        self.last_init_gate_log_time = 0.0
        self.init_gate_log_interval = 2.0
        
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
        self.last_frontier_skip_reason = None
        self.last_frontier_relax_log_time = 0.0
        self.frontier_relax_log_interval = 5.0
        self.last_frontier_debug_log_time = 0.0
        self.last_tf_check_log_time = 0.0
        self.tf_check_log_interval = 5.0
        self.costmap = None
        self.local_costmap = None
        self.costmap_raw_received = False
        self.local_costmap_raw_received = False

        # Nav2 lifecycle state clients
        self.bt_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.controller_state_client = self.create_client(GetState, '/controller_server/get_state')
        self.planner_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.last_odom_time = 0.0
        self.odom_stale_timeout = 1.0  # seconds
        self.last_odom_stale_log_time = 0.0
        self.odom_stale_log_interval = 5.0
        self.last_frontier_update_time = 0.0
        self.last_replan_time = 0.0
        self.pending_replan_goal = None
        self.last_goal_cancel_time = 0.0
        self.replan_cancel_pending = False
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.mapped_area_pub = self.create_publisher(Float32, '/mapped_area_m2', 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_marker_pub = self.create_publisher(Marker, '/exploration_status', status_qos)

        self.map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        # Subscribers
        self.create_subscription(MarkerArray, '/frontiers', self.frontiers_cb, 10)
        distance_sub = self.create_subscription(Float32, '/safety_stop', self.obstacle_distance_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        # Also listen to raw scan to mark lidar freshness in case /scan is delayed
        self.create_subscription(LaserScan, self.scan_raw_topic, self.scan_raw_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = DurabilityPolicy.VOLATILE
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_raw_qos = qos_profile_sensor_data
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, map_qos)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_cb, costmap_qos)
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.local_costmap_cb, costmap_qos)
        self.create_subscription(Costmap, self.costmap_raw_topic, self.costmap_raw_cb, costmap_raw_qos)
        self.create_subscription(Costmap, self.local_costmap_raw_topic, self.local_costmap_raw_cb, costmap_raw_qos)
        self.get_logger().info(
            f"🧭 Costmap subscriptions: global={self.costmap_topic}, local={self.local_costmap_topic}, "
            f"global_raw={self.costmap_raw_topic}, local_raw={self.local_costmap_raw_topic}"
        )
        self.get_logger().info("✅ SUBSCRIBED to /safety_stop (Arduino safety stop signals)")
        
        # Rear obstacle detection from LiDAR
        self.rear_obstacle_detected = False
        self.last_rear_obstacle_time = 0.0
        self.last_rear_distance = float('inf')
        self.lidar_backup_until = 0.0
        
        # Timer for main loop - 20Hz for smooth backward motion
        self.create_timer(0.05, self.main_loop)  # 20 Hz control loop
        
        self.get_logger().info("🚀 Exploration Coordinator Started (Simplified)")

    def destroy_node(self):
        """Save map synchronously on any shutdown (Ctrl+C or natural exit), but only if not already saved."""
        try:
            if not getattr(self, 'map_save_requested', False):
                self._save_map_sync()
        except Exception as e:
            try:
                self.get_logger().error(f'❌ Map save on shutdown failed: {e}')
            except Exception:
                pass
        super().destroy_node()

    def _save_map_sync(self):
        import struct
        import zlib
        import datetime
        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            return
        map_save_dir = getattr(self, 'map_save_dir', '/home/pi/FEA_SLAM_WS/saved_maps')
        ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        map_save_name = f'map_{ts}'
        os.makedirs(map_save_dir, exist_ok=True)
        map_name = os.path.join(map_save_dir, map_save_name)
        info = slam_map.info
        width, height = info.width, info.height
        resolution = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = slam_map.data
        pixels = bytearray(width * height)
        for row in range(height):
            for col in range(width):
                idx = (height - 1 - row) * width + col
                v = data[idx]
                if v < 0:
                    pixels[row * width + col] = 205
                elif v == 0:
                    pixels[row * width + col] = 254
                else:
                    pixels[row * width + col] = max(0, 255 - int(v * 2.55))
        # PGM
        with open(map_name + '.pgm', 'wb') as f:
            f.write(f'P5\n# CREATOR: fea_slam\n{width} {height}\n255\n'.encode('ascii'))
            f.write(bytes(pixels))
        # YAML
        with open(map_name + '.yaml', 'w') as f:
            f.write(f'image: {map_save_name}.pgm\n')
            f.write(f'resolution: {resolution}\n')
            f.write(f'origin: [{ox:.6f}, {oy:.6f}, 0.000000]\n')
            f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        # PNG
        def _chunk(tag, data):
            c = zlib.crc32(tag + data) & 0xFFFFFFFF
            return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', c)
        ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
        raw = b''.join(b'\x00' + bytes(pixels[r * width:(r + 1) * width]) for r in range(height))
        png_bytes = (b'\x89PNG\r\n\x1a\n'
                     + _chunk(b'IHDR', ihdr)
                     + _chunk(b'IDAT', zlib.compress(raw, 6))
                     + _chunk(b'IEND', b''))
        with open(map_name + '.png', 'wb') as f:
            f.write(png_bytes)
        try:
            self.get_logger().info(
                f'💾 Shutdown map saved: {map_name}.pgm + .png ({width}x{height})')
        except Exception:
            pass

    def frontiers_cb(self, msg: MarkerArray):
        """Store frontier positions"""
        self.current_frontiers = []
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.current_frontiers.append((x, y))

        self.last_frontier_update_time = time.time()

        # If frontiers exist, LiDAR/map is flowing; use as fallback for lidar freshness
        if self.current_frontiers:
            self.ever_had_frontiers = True
            self.last_frontier_time = time.time()
            self.last_scan_time = time.time()
    
    def obstacle_distance_cb(self, msg: Float32):
        """Handle Arduino safety stop signals"""
        if self.nav2_handles_obstacles and not self.strict_obstacle_handling:
            return
        if not self.use_ultrasonic_backup:
            return

        distance = msg.data
        now = time.time()

        # 999.0 = sentinel value for SAFETY_STOP:0 (obstacle cleared)
        is_clear_signal = distance >= 999.0

        # Reject invalid non-sentinel ultrasonic values
        if not is_clear_signal and distance < self.ultrasonic_min_valid_distance:
            if (now - self.last_ultrasonic_filter_log_time) >= self.ultrasonic_filter_log_interval:
                self.last_ultrasonic_filter_log_time = now
                self.get_logger().info(
                    f"ℹ️ Ignoring ultrasonic spike: {distance:.3f}m < min_valid {self.ultrasonic_min_valid_distance:.3f}m"
                )
            return

        # Reject sudden distance jumps within a short window (noise/spike suppression)
        if (
            not is_clear_signal and
            self.last_ultrasonic_distance is not None and
            (now - self.last_ultrasonic_time) <= self.ultrasonic_jump_window and
            abs(distance - self.last_ultrasonic_distance) > self.ultrasonic_max_jump
        ):
            if (now - self.last_ultrasonic_filter_log_time) >= self.ultrasonic_filter_log_interval:
                self.last_ultrasonic_filter_log_time = now
                self.get_logger().info(
                    f"ℹ️ Ignoring ultrasonic jump: {self.last_ultrasonic_distance:.3f}m -> {distance:.3f}m"
                )
            return

        # Update last valid sample
        if not is_clear_signal:
            self.last_ultrasonic_distance = distance
            self.last_ultrasonic_time = now

        # Emergency ultrasonic backup at very close range (15cm default)
        if distance < 999.0 and distance <= self.ultrasonic_backup_distance:
            self.ultrasonic_obstacle_hits += 1
            self.ultrasonic_clear_hits = 0
            if self.ultrasonic_obstacle_hits < self.ultrasonic_confirm_count:
                return
            self.ultrasonic_emergency_until = now + self.backup_time
            self.obstacle_distance_m = distance
            self.last_obstacle_time = now
            self.obstacle_detected = True
            self.front_obstacle_detected = True
            self.last_front_obstacle_time = now
            self.current_lidar_obstacle_type = "ultrasonic"
            self.get_logger().error(
                f"🚨 Ultrasonic EMERGENCY! {distance:.3f}m (<= {self.ultrasonic_backup_distance:.2f}m) - backing up {self.backup_time:.1f}s"
            )
            # Immediate stop — react now, don't wait for main_loop tick
            _stop = Twist()
            self.cmd_vel_pub.publish(_stop)
            self.cmd_vel_nav_pub.publish(_stop)
            try:
                if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None
                    self.goal_in_progress = False
            except Exception:
                pass
            if self.current_phase not in (Phase.OBSTACLE, Phase.RESCAN, Phase.DONE):
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = now
            return

        # If LiDAR is healthy, ignore ultrasonic (backup only)
        if (now - self.last_scan_time) <= self.lidar_stale_timeout:
            return
        
        # Only react to very close obstacles (backup)
        if distance < 999.0 and distance > self.ultrasonic_backup_distance:
            self.ultrasonic_obstacle_hits = 0
            return
        
        if distance >= 999.0:
            self.ultrasonic_clear_hits += 1
            self.ultrasonic_obstacle_hits = 0
            if self.ultrasonic_clear_hits < self.ultrasonic_clear_confirm_count:
                return
            # Latch detection briefly so main loop can react
            if self.obstacle_detected and (now - self.last_obstacle_time) < self.obstacle_hold_time:
                return
            if self.current_phase == Phase.OBSTACLE:
                return
            self.get_logger().info(f"✅ Obstacle cleared (SAFETY_STOP:0)")
            self.obstacle_detected = False
            return
        
        # Otherwise it's a SAFETY_STOP:1 with actual distance (backup only if LiDAR stale)
        self.ultrasonic_obstacle_hits += 1
        self.ultrasonic_clear_hits = 0
        if self.ultrasonic_obstacle_hits < self.ultrasonic_confirm_count:
            return
        self.obstacle_distance_m = distance
        self.last_obstacle_time = now
        self.get_logger().error(f"🚨🚨🚨 SAFETY_STOP TRIGGERED! Obstacle at {distance:.3f}m!")
        self.obstacle_detected = True
    
    def scan_cb(self, msg: LaserScan):
        """Monitor LiDAR for front and rear obstacles"""
        self.last_scan_time = time.time()
        # Check rear 180 degrees (90° left to 90° right from rear = robot's back half)
        # LiDAR angle 0 = front, π/2 = left, -π/2 = right, ±π = rear
        
        # Rear detection zone: 90° to 270° (wide cone to catch walls behind)
        num_readings = len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        rear_obstacle = False
        front_obstacle = False
        min_front_distance = float('inf')
        min_rear_distance = float('inf')
        front_hit_count = 0
        rear_hit_count = 0
        for i, distance in enumerate(msg.ranges):
            if distance <= max(msg.range_min, self.scan_min_range) or distance > msg.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            # Normalize angle to [-π, π]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            # Rear cone centered at ±pi; excludes side sectors
            in_rear_zone = (abs(abs(angle) - math.pi) <= self.rear_obstacle_half_angle)
            # Front cone centered at 0 rad
            in_front_zone = (-self.front_obstacle_half_angle <= angle <= self.front_obstacle_half_angle)
            
            if in_rear_zone:
                if distance < min_rear_distance:
                    min_rear_distance = distance
                if distance < self.rear_obstacle_threshold:
                    rear_hit_count += 1

            if self.use_lidar_obstacle and in_front_zone:
                if distance < min_front_distance:
                    min_front_distance = distance
                if distance < self.lidar_obstacle_distance:
                    front_hit_count += 1

        if self.swap_lidar_front_back:
            min_front_distance, min_rear_distance = min_rear_distance, min_front_distance
            front_hit_count, rear_hit_count = rear_hit_count, front_hit_count

        front_obstacle = front_hit_count >= max(1, self.front_obstacle_min_hits)
        rear_obstacle = rear_hit_count >= max(1, self.rear_obstacle_min_hits)
        
        # Latch rear obstacle detection to stop backing reliably
        if min_rear_distance < float('inf'):
            self.last_rear_distance = min_rear_distance

        if rear_obstacle or (min_rear_distance <= self.rear_emergency_distance):
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
            now = time.time()
            obstacle_type = "static"
            if (
                self.last_lidar_front_distance is not None and
                (now - self.last_lidar_front_time) > 0.0 and
                (now - self.last_lidar_front_time) <= self.lidar_motion_window
            ):
                delta = abs(min_front_distance - self.last_lidar_front_distance)
                if delta >= self.lidar_motion_distance_delta:
                    obstacle_type = "dynamic"
            self.current_lidar_obstacle_type = obstacle_type
            self.last_lidar_front_distance = min_front_distance
            self.last_lidar_front_time = now

            self.obstacle_distance_m = min_front_distance
            self.last_obstacle_time = now
            self.last_front_obstacle_time = now
            self.front_obstacle_detected = True
            if not self.obstacle_detected:
                self.get_logger().error(
                    f"🚨 LiDAR obstacle at {min_front_distance:.2f}m (threshold: {self.lidar_obstacle_distance:.2f}m)"
                )
                # Immediate stop on first detection — don't wait for main_loop
                _stop = Twist()
                self.cmd_vel_pub.publish(_stop)
                self.cmd_vel_nav_pub.publish(_stop)
                try:
                    if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                        self.goal_handle = None
                        self.goal_in_progress = False
                except Exception:
                    pass
            if (now - self.last_lidar_obstacle_class_log_time) >= self.lidar_obstacle_class_log_interval:
                self.last_lidar_obstacle_class_log_time = now
                self.get_logger().warn(
                    f"🧠 LiDAR obstacle classified as {obstacle_type.upper()} (front={min_front_distance:.2f}m)"
                )
            self.obstacle_detected = True
            if self.lidar_backup_on_obstacle:
                self.lidar_backup_until = now + self.backup_time
            # Track how long a STATIC obstacle has been present (for deep escape)
            if obstacle_type == "static":
                if self.static_stuck_start == 0.0:
                    self.static_stuck_start = now
            else:
                # Dynamic obstacle → reset stuck timer
                self.static_stuck_start = 0.0
                self.static_stuck_escape_active = False
        elif self.use_lidar_obstacle:
            if min_front_distance < float('inf'):
                self.last_lidar_front_distance = min_front_distance
                self.last_lidar_front_time = time.time()
            # Clear obstacle after hold time when front is clear
            if self.obstacle_detected and (time.time() - self.last_obstacle_time) >= self.obstacle_hold_time:
                self.obstacle_detected = False
            if self.front_obstacle_detected and (time.time() - self.last_front_obstacle_time) >= self.obstacle_hold_time:
                self.front_obstacle_detected = False
            # Front cleared → reset static stuck state
            self.static_stuck_start = 0.0
            self.static_stuck_escape_active = False

    def scan_raw_cb(self, msg: LaserScan):
        """Track raw scan timing for startup readiness"""
        self.last_scan_time = time.time()

    def odom_cb(self, msg: Odometry):
        self.last_odom_time = time.time()

    def costmap_cb(self, msg: OccupancyGrid):
        if self.costmap is None:
            self.get_logger().info("✅ Received global costmap (OccupancyGrid)")
        self.costmap = msg

    def local_costmap_cb(self, msg: OccupancyGrid):
        if self.local_costmap is None:
            self.get_logger().info("✅ Received local costmap (OccupancyGrid)")
        self.local_costmap = msg

    def map_cb(self, msg: OccupancyGrid):
        data = msg.data
        if not data or self.last_percent_known is None:
            return
        # Only count valid known cells (0-100) as explored
        known_cells = 0
        total_cells = len(data)
        for val in data:
            if 0 <= val <= 100:
                known_cells += 1
        # Track the maximum map size ever seen for a stable metric
        if total_cells > self.max_total_cells:
            self.max_total_cells = total_cells
        resolution = msg.info.resolution
        area_m2 = known_cells * (resolution * resolution)
        now = time.time()
        elapsed = now - self.startup_time
        # Standard percentage (may drop if map expands)
        percent_known = (known_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
        # Stable percentage (uses max map size ever seen)
        stable_percent_known = (known_cells / self.max_total_cells) * 100.0 if self.max_total_cells > 0 else 0.0
        self.last_mapped_area_m2 = area_m2
        self.last_percent_known = percent_known
        self.last_stable_percent_known = stable_percent_known
        self.last_slam_map = msg  # store for map saving
        self.mapped_area_series.append((elapsed, area_m2, percent_known, known_cells, total_cells, stable_percent_known))
        # Log both metrics for debugging
        self.get_logger().info(f"[DEBUG] Map completion: {percent_known:.2f}% (current map size), {stable_percent_known:.2f}% (stable, max map size)")

        msg_area = Float32()
        msg_area.data = area_m2
        self.mapped_area_pub.publish(msg_area)

    def costmap_raw_cb(self, msg: Costmap):
        if not self.costmap_raw_received:
            self.get_logger().info("✅ Received global costmap_raw (Costmap)")
        self.costmap_raw_received = True

    def local_costmap_raw_cb(self, msg: Costmap):
        if not self.local_costmap_raw_received:
            self.get_logger().info("✅ Received local costmap_raw (Costmap)")
        self.local_costmap_raw_received = True

    def _odom_recent(self) -> bool:
        """Check if odometry data is recent, using TF transforms as fallback"""
        now = time.time()
        timeout = Duration(seconds=0.05)  # 50ms timeout
        
        # Check if /odom topic is recent
        if (now - self.last_odom_time) <= self.odom_stale_timeout:
            return True

        # Fallback: check odom->base TF freshness if /odom topic is missing
        # This is critical - use TF transforms when Arduino serial is not working
        try:
            # Check if odom->base_footprint transform is available and recent
            if not self.tf_buffer.can_transform('odom', self.base_frame, rclpy.time.Time(), timeout=timeout):
                return False
            tf = self.tf_buffer.lookup_transform('odom', self.base_frame, rclpy.time.Time(), timeout=timeout)
            tf_age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if tf_age <= self.odom_stale_timeout:
                return True
        except Exception:
            pass
        
        # Also check if map->odom transform exists (SLAM is providing it)
        try:
            if self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=timeout):
                # SLAM is providing map->odom, so odometry is effectively available
                return True
        except Exception:
            pass
            
        return False

    def _goal_in_free_space(self, x, y, use_costmap_filter: bool = True):
        if not use_costmap_filter or self.costmap is None:
            return True

        if hasattr(self.costmap, 'info'):
            info = self.costmap.info
            origin_x = info.origin.position.x
            origin_y = info.origin.position.y
            resolution = info.resolution
            width = info.width
            height = info.height
            data = self.costmap.data
        elif hasattr(self.costmap, 'metadata'):
            meta = self.costmap.metadata
            origin_x = meta.origin.position.x
            origin_y = meta.origin.position.y
            resolution = meta.resolution
            width = meta.size_x
            height = meta.size_y
            data = self.costmap.data
        else:
            return True

        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False

        index = my * width + mx
        value = data[index]

        # Treat unknown or high-cost as not free
        if value < 0:
            return False
        if value >= self.costmap_free_threshold:
            return False

        return True
    
    def update_pose(self):
        """Get robot pose from TF with proper timeout handling"""
        # Use rclpy.time.Time() with timeout parameter for proper TF lookup
        # This fixes the "extrapolation into the past" errors
        try:
            # Use time=0 (latest available) with explicit timeout to avoid timing issues
            now = self.get_clock().now()
            timeout = Duration(seconds=0.1)  # 100ms timeout
            
            # Check if transform is available first
            if not self.tf_buffer.can_transform('map', self.base_frame, rclpy.time.Time(), timeout=timeout):
                self.pose_valid = False
                self.get_logger().debug("TF transform not yet available, will retry...")
                return
                
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time(), timeout=timeout)
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            old_pose = self.robot_pose
            self.robot_pose = (x, y, 0.0)
            self.pose_valid = True
            self._update_pose_stale_state(x, y, tf.header.stamp)
            self._record_path_point(x, y)
            
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
                
                # Check if fallback transform is available
                if not self.tf_buffer.can_transform('map', fallback_frame, rclpy.time.Time(), timeout=timeout):
                    self.pose_valid = False
                    return
                    
                tf = self.tf_buffer.lookup_transform('map', fallback_frame, rclpy.time.Time(), timeout=timeout)
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                self.robot_pose = (x, y, 0.0)
                self.pose_valid = True
                self._update_pose_stale_state(x, y, tf.header.stamp)
                self._record_path_point(x, y)
            except Exception as e2:
                self.pose_valid = False
                # Log TF errors to debug localization issues (throttled to avoid spam)
                now = time.time()
                if (now - self.last_origin_warn_time) >= self.origin_warn_interval:
                    self.last_origin_warn_time = now
                    self.get_logger().debug(f"TF lookup pending: {str(e)[:50]}...")
                # Don't log error on every cycle - TF takes time to initialize

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
        nav2_services_ready = self._nav2_services_ready()
        nav2_lifecycle_active = self._nav2_active(require_active=self.require_nav2_active, services_ready=nav2_services_ready)
        nav2_ready = nav2_server_ready
        can_map_odom = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        can_odom_base = self.tf_buffer.can_transform('odom', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        clear_for_hold = (time.time() - self.last_obstacle_time) >= self.startup_clear_hold_time
        costmap_ready = (
            (self.costmap is not None) or
            (self.local_costmap is not None) or
            self.costmap_raw_received or
            self.local_costmap_raw_received
        )
        if self.require_costmap and not costmap_ready and self.init_start_time is not None:
            if self.costmap_wait_timeout > 0.0 and (now - self.init_start_time) >= self.costmap_wait_timeout:
                if not self.costmap_waited_out:
                    self.costmap_waited_out = True
                    self.get_logger().warn(
                        "⚠️ Costmap not received in time; proceeding without startup costmap gate."
                    )
        costmap_gate = costmap_ready or (self.require_costmap and self.costmap_waited_out)

        self.get_logger().info(
            "🔎 Startup status: "
            f"lidar={lidar_ready}, frontiers={frontiers_ready}, pose={pose_ready}, pose_moved={pose_moved}, "
            f"odom_recent={odom_recent}, nav2_server={nav2_server_ready}, nav2_ready={nav2_ready}, "
            f"map->odom={can_map_odom}, odom->{self.base_frame}={can_odom_base}, "
            f"costmap={costmap_ready}, costmap_gate={costmap_gate}, "
            f"clear={clear_for_hold}"
        )

    def _get_costmap_info(self):
        if self.costmap is None:
            return None

        if hasattr(self.costmap, 'info'):
            info = self.costmap.info
            origin_x = info.origin.position.x
            origin_y = info.origin.position.y
            resolution = info.resolution
            width = info.width
            height = info.height
            data = self.costmap.data
        elif hasattr(self.costmap, 'metadata'):
            meta = self.costmap.metadata
            origin_x = meta.origin.position.x
            origin_y = meta.origin.position.y
            resolution = meta.resolution
            width = meta.size_x
            height = meta.size_y
            data = self.costmap.data
        else:
            return None

        return origin_x, origin_y, resolution, width, height, data

    def _world_to_costmap(self, x, y, costmap_info):
        origin_x, origin_y, resolution, width, height, _ = costmap_info
        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None
        return (mx, my)

    def _is_costmap_free(self, mx, my, costmap_info):
        _, _, _, width, _, data = costmap_info
        index = my * width + mx
        value = data[index]
        if value < 0:
            return False
        return value < self.costmap_free_threshold

    def _astar_path_length(self, start_xy, goal_xy):
        costmap_info = self._get_costmap_info()
        if costmap_info is None:
            return None

        start = self._world_to_costmap(start_xy[0], start_xy[1], costmap_info)
        goal = self._world_to_costmap(goal_xy[0], goal_xy[1], costmap_info)
        if start is None or goal is None:
            return None

        if not self._is_costmap_free(start[0], start[1], costmap_info):
            # Log why A* failed for this candidate
            self.get_logger().debug(f"A* skipped: start position ({start_xy[0]:.2f}, {start_xy[1]:.2f}) not in costmap free space")
            return None
        if not self._is_costmap_free(goal[0], goal[1], costmap_info):
            # Log why A* failed for this candidate
            self.get_logger().debug(f"A* skipped: goal position ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) not in costmap free space")
            return None

        import heapq
        sx, sy = start
        gx, gy = goal
        open_set = [(0.0, 0.0, sx, sy)]
        g_scores = {(sx, sy): 0.0}
        expansions = 0

        while open_set and expansions < self.astar_max_expansions:
            _, g, x, y = heapq.heappop(open_set)
            expansions += 1
            if (x, y) == (gx, gy):
                _, _, resolution, _, _, _ = costmap_info
                path_length = g * resolution
                self.get_logger().debug(f"A* success: ({start_xy[0]:.2f}, {start_xy[1]:.2f}) → ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) = {path_length:.2f}m in {expansions} expansions")
                return path_length

            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if not self._is_costmap_free(nx, ny, costmap_info):
                    continue
                ng = g + 1.0
                if (nx, ny) not in g_scores or ng < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = ng
                    h = abs(gx - nx) + abs(gy - ny)
                    f = ng + h
                    heapq.heappush(open_set, (f, ng, nx, ny))

        # Exceeded max expansions without finding path
        self.get_logger().debug(f"A* failed: ({start_xy[0]:.2f}, {start_xy[1]:.2f}) → ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) exceeded {self.astar_max_expansions} expansions")
        return None
    
    def pick_best_frontier(self):
        if not self.current_frontiers:
            self.last_frontier_skip_reason = "no_frontiers"
            return None

        self.update_pose()
        
        # Check if pose is valid for navigation
        if not self.pose_valid:
            # Pose not available - skip this cycle
            self.last_frontier_skip_reason = "pose_invalid"
            return None
            
        # LENIENT: Allow navigation even with stale pose (TF takes time to update)
        # Only block if pose is completely invalid (not just stale)
        if self.pose_stale:
            # Log occasionally but still allow navigation
            now = time.time()
            if (now - self.last_origin_warn_time) >= self.origin_warn_interval:
                self.last_origin_warn_time = now
                self.get_logger().warn(
                    f"⚠️ Using stale pose (age > {self.pose_stale_timeout}s). Navigation may be imprecise."
                )
            # Continue with navigation - stale is better than nothing

        robot_x, robot_y, _ = self.robot_pose
        now = time.time()

        if self.blacklisted_goals:
            expired = [k for k, v in self.blacklisted_goals.items() if v <= now]
            for k in expired:
                self.blacklisted_goals.pop(k, None)

        if self.recent_goals:
            self.recent_goals = [g for g in self.recent_goals if g[2] > now]

        def select_frontier(min_dist, goal_offset, use_costmap_filter):
            best = None
            best_dist = -float('inf') if self.frontier_pick_farthest else float('inf')
            candidates = []

            rejection_counts = {
                'too_close': 0,
                'strict_revisit': 0,
                'strict_attempted': 0,
                'costmap_or_free_space': 0,
                'blacklisted': 0,
                'visited_goal': 0,
                'near_last_success': 0,
                'near_last_goal': 0,
                'recent_goal': 0,
            }

            def build_candidates(costmap_filter: bool):
                built = []
                for fx, fy in self.current_frontiers:
                    dist = math.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
                    if dist < min_dist:
                        rejection_counts['too_close'] += 1
                        continue

                    if self.force_frontier_goal:
                        goal_x = fx
                        goal_y = fy
                    elif goal_offset > 0.0:
                        if dist <= (goal_offset + 0.05):
                            continue
                        ux = (fx - robot_x) / dist
                        uy = (fy - robot_y) / dist
                        goal_x = fx - (ux * goal_offset)
                        goal_y = fy - (uy * goal_offset)
                    else:
                        goal_x = fx
                        goal_y = fy

                    if self.strict_no_revisit and self.strict_avoid_goals:
                        strict_radius = max(
                            self.visited_goal_radius,
                            self.recent_goal_radius,
                            self.avoid_return_radius,
                            self.avoid_last_goal_radius,
                            self.known_frontier_avoid_radius,
                        )
                        if any(math.hypot(goal_x - sx, goal_y - sy) <= strict_radius
                               for sx, sy in self.strict_avoid_goals):
                            rejection_counts['strict_revisit'] += 1
                            continue
                    if self.strict_no_revisit and self.attempted_frontiers:
                        strict_radius = max(
                            self.visited_goal_radius,
                            self.recent_goal_radius,
                            self.avoid_return_radius,
                            self.avoid_last_goal_radius,
                            self.known_frontier_avoid_radius,
                        )
                        if any(math.hypot(fx - sx, fy - sy) <= strict_radius
                               for sx, sy in self.attempted_frontiers):
                            rejection_counts['strict_attempted'] += 1
                            continue

                    if not self._goal_in_free_space(goal_x, goal_y, costmap_filter):
                        rejection_counts['costmap_or_free_space'] += 1
                        continue

                    if self.blacklisted_goals:
                        skip = any(math.hypot(goal_x - bx, goal_y - by) <= self.blacklist_radius
                                  for (bx, by), _ in self.blacklisted_goals.items())
                        if skip:
                            rejection_counts['blacklisted'] += 1
                            continue

                    if self.avoid_revisit and self.visited_goals:
                        skip = any(math.hypot(goal_x - vx, goal_y - vy) <= self.visited_goal_radius
                                  for vx, vy in self.visited_goals)
                        if skip:
                            rejection_counts['visited_goal'] += 1
                            continue

                    if self.avoid_revisit and self.last_successful_goal_pos is not None:
                        if math.hypot(goal_x - self.last_successful_goal_pos[0], goal_y - self.last_successful_goal_pos[1]) <= self.avoid_return_radius:
                            rejection_counts['near_last_success'] += 1
                            continue

                    if self.avoid_revisit and self.last_goal_target is not None:
                        if math.hypot(goal_x - self.last_goal_target[0], goal_y - self.last_goal_target[1]) <= self.avoid_last_goal_radius:
                            rejection_counts['near_last_goal'] += 1
                            continue

                    if self.recent_goals:
                        skip = any(math.hypot(goal_x - rx, goal_y - ry) <= self.recent_goal_radius
                                  for rx, ry, _ in self.recent_goals)
                        if skip:
                            rejection_counts['recent_goal'] += 1
                            continue

                    built.append({
                        'frontier': (fx, fy),
                        'goal': (goal_x, goal_y),
                        'dist': dist,
                        'costmap_filtered': costmap_filter
                    })
                return built

            candidates = build_candidates(use_costmap_filter)
            if not candidates and use_costmap_filter and self.allow_costmapless_replan_candidates:
                candidates = build_candidates(False)
                if candidates:
                    now = time.time()
                    if (now - self.last_frontier_relax_log_time) >= self.frontier_relax_log_interval:
                        self.last_frontier_relax_log_time = now
                        self.get_logger().warn("No candidates after costmap filter; retrying without costmap filter.")

            if not candidates:
                if self.frontier_debug:
                    now_dbg = time.time()
                    if (now_dbg - self.last_frontier_debug_log_time) >= self.frontier_debug_log_interval:
                        self.last_frontier_debug_log_time = now_dbg
                        self.get_logger().warn(
                            "🧪 Frontier filter diagnostics: "
                            f"raw={len(self.current_frontiers)}, min_dist={min_dist:.2f}, "
                            f"reject={rejection_counts}"
                        )
                return None

            if self.frontier_selection_method == 'astar':
                candidates.sort(key=lambda c: c['dist'])
                candidates = candidates[:max(1, self.astar_max_candidates)]
                best_cost = None
                astar_found = False
                astar_attempts = 0
                for candidate in candidates:
                    astar_attempts += 1
                    path_cost = self._astar_path_length((robot_x, robot_y), candidate['goal'])
                    if path_cost is None:
                        continue
                    astar_found = True
                    if best_cost is None:
                        best_cost = path_cost
                        best = candidate
                        best['dist'] = path_cost
                        continue
                    if self.frontier_pick_farthest:
                        if path_cost > best_cost:
                            best_cost = path_cost
                            best = candidate
                            best['dist'] = path_cost
                    else:
                        if path_cost < best_cost:
                            best_cost = path_cost
                            best = candidate
                            best['dist'] = path_cost
                if astar_found:
                    self.get_logger().info(f"✅ A* selected best frontier from {astar_attempts} candidates (path cost: {best_cost:.2f}m)")
                    return best
                now = time.time()
                if (now - self.last_frontier_relax_log_time) >= self.frontier_relax_log_interval:
                    self.last_frontier_relax_log_time = now
                    self.get_logger().warn(f"⚠️ A* failed for all {astar_attempts} candidates (check debug logs); falling back to distance selection.")

            for candidate in candidates:
                dist = candidate['dist']
                if self.frontier_pick_farthest:
                    if dist <= best_dist:
                        continue
                else:
                    if dist >= best_dist:
                        continue
                best_dist = dist
                best = candidate

            return best

        best_frontier = select_frontier(
            self.min_frontier_distance,
            self.frontier_goal_offset,
            self.use_costmap_goal_filter
        )

        if best_frontier is None and self.no_frontier_cycles >= self.relaxed_frontier_after_cycles:
            best_frontier = select_frontier(
                self.relaxed_min_frontier_distance,
                self.relaxed_frontier_goal_offset,
                self.relaxed_use_costmap_filter
            )
            if best_frontier:
                self.get_logger().info("⚠️ Relaxed frontier filter enabled for this goal")

        # Bootstrap fallback: if frontiers exist but all strict filters reject them,
        # allow selecting the nearest raw frontier for the first few goals so
        # exploration can actually leave origin.
        if best_frontier is None and self.current_frontiers and self.goals_reached < self.min_goals_for_complete:
            fallback = None
            fallback_dist = float('inf')
            for fx, fy in self.current_frontiers:
                dist = math.hypot(fx - robot_x, fy - robot_y)
                # Avoid only trivially-close points at startup.
                if dist < max(self.bootstrap_fallback_min_distance, self.relaxed_min_frontier_distance):
                    continue
                if not self._goal_in_free_space(fx, fy, self.use_costmap_goal_filter):
                    continue
                if dist < fallback_dist:
                    fallback_dist = dist
                    fallback = {
                        'frontier': (fx, fy),
                        'goal': (fx, fy),
                        'dist': dist,
                        'costmap_filtered': False
                    }
            if fallback is not None:
                best_frontier = fallback
                now = time.time()
                if (now - self.last_frontier_relax_log_time) >= self.frontier_relax_log_interval:
                    self.last_frontier_relax_log_time = now
                    self.get_logger().warn(
                        "⚠️ Using bootstrap frontier fallback (strict filters rejected all candidates)."
                    )

        if best_frontier:
            fx, fy = best_frontier['frontier']
            gx, gy = best_frontier['goal']
            self.get_logger().info(f"🎯 Nearest frontier: ({fx:.2f}, {fy:.2f}) -> goal ({gx:.2f}, {gy:.2f}), dist: {best_frontier['dist']:.2f}m")
            return best_frontier

        self.last_frontier_skip_reason = "no_valid_frontiers"
        return None

    def _refresh_pending_goal_from_frontiers(self, reason: str):
        if not self.replan_on_frontier_update:
            return
        if self.last_frontier_update_time <= self.last_replan_time:
            return
        now = time.time()
        if (now - self.last_replan_time) < self.replan_hard_min_interval:
            return
        if not self.always_replan_on_frontier_update and (now - self.last_replan_time) < self.replan_interval:
            return
        self.update_pose()
        if not self.pose_valid:
            return

        goal_info = self.pick_best_frontier()
        if goal_info is None:
            return

        best_goal_x, best_goal_y = goal_info['goal']
        if self.last_goal_target is not None:
            current_goal_x, current_goal_y = self.last_goal_target
            goal_delta = math.hypot(best_goal_x - current_goal_x, best_goal_y - current_goal_y)
            if goal_delta < max(0.05, self.replan_goal_change_distance):
                return

        self.pending_replan_goal = (best_goal_x, best_goal_y)
        self.last_replan_time = now
        self.get_logger().info(f"🔁 Replan pending ({reason}): new goal ({best_goal_x:.2f}, {best_goal_y:.2f})")
    
    def send_goal_to_nav2(self, goal_x, goal_y, use_costmap_filter: bool | None = None, frontier_xy=None):
        """Send goal to Nav2 navigate_to_pose with proper TF timeout handling"""
        now = time.time()
        timeout = Duration(seconds=0.1)  # 100ms timeout for TF operations
        if use_costmap_filter is None:
            use_costmap_filter = self.use_costmap_goal_filter
        
        # Check TF directly instead of requiring odom
        try:
            can_map_base = self.tf_buffer.can_transform('map', self.base_frame, rclpy.time.Time(), timeout=timeout)
            if not can_map_base:
                self.get_logger().warn("⚠️ Skipping goal send: map->base TF not available")
                return False
        except Exception as e:
            self.get_logger().warn(f"⚠️ Skipping goal send: TF check failed: {e}")
            return False

        # Double-check goal is in free space (costmap may have updated)
        if not self._goal_in_free_space(goal_x, goal_y, use_costmap_filter):
            self.get_logger().warn(f"⚠️ Goal ({goal_x:.2f}, {goal_y:.2f}) no longer in free space - skipping")
            self._blacklist_goal((goal_x, goal_y))
            return False

        nav2_server_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        nav2_services_ready = self._nav2_services_ready()
        nav2_lifecycle_active = self._nav2_active(require_active=self.require_nav2_active, services_ready=nav2_services_ready)
        nav2_ready = nav2_server_ready and (nav2_lifecycle_active if self.require_nav2_active else True)
        if self.require_nav2_active and not nav2_ready:
            self.get_logger().warn("⚠️ Skipping goal send: Nav2 not active")
            return False

        # Verify required TF chains exist (map->odom->base_footprint)
        # Use explicit timeout to avoid extrapolation errors
        try:
            can_map_base = self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=timeout)
            can_map_odom = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=timeout)
        except Exception as tf_err:
            if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                self.last_tf_check_log_time = now
                self.get_logger().warn(f"⚠️ TF availability check failed: {tf_err}. Goal send skipped.")
            return False
            
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
        
        # Get current robot position for logging
        self.update_pose()
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        if frontier_xy is not None:
            fx, fy = frontier_xy
            heading = math.atan2(fy - robot_y, fx - robot_x)
            heading_deg = math.degrees(heading)
            self.get_logger().info(
                f"🚀 Sending Nav2 goal: Robot ({robot_x:.2f}, {robot_y:.2f}) → Frontier ({fx:.2f}, {fy:.2f}) → Goal ({goal_x:.2f}, {goal_y:.2f}) | Heading: {heading_deg:.1f}°"
            )
        else:
            heading = math.atan2(goal_y - robot_y, goal_x - robot_x)
            heading_deg = math.degrees(heading)
            self.get_logger().info(
                f"🚀 Sending Nav2 goal: Robot ({robot_x:.2f}, {robot_y:.2f}) → Goal ({goal_x:.2f}, {goal_y:.2f}) | Heading: {heading_deg:.1f}°"
            )
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_cb)

        return True

    def _nav2_services_ready(self) -> bool:
        now = time.time()
        missing = []
        if not self.bt_state_client.service_is_ready():
            missing.append('bt_navigator')
        if not self.controller_state_client.service_is_ready():
            missing.append('controller_server')
        if not self.planner_state_client.service_is_ready():
            missing.append('planner_server')
        if missing and (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
            self.last_tf_check_log_time = now
            self.get_logger().warn(f"⚠️ Nav2 lifecycle services not ready: {', '.join(missing)}")
        return len(missing) == 0

    def _nav2_active(self, require_active: bool = False, services_ready: bool | None = None):
        now = time.time()
        if services_ready is None:
            services_ready = self._nav2_services_ready()
        if not services_ready:
            if require_active and self.nav2_active_fallback_on_action:
                return self.nav_client.wait_for_server(timeout_sec=0.05)
            return False

        bt_state = self._get_lifecycle_state(self.bt_state_client)
        controller_state = self._get_lifecycle_state(self.controller_state_client)
        planner_state = self._get_lifecycle_state(self.planner_state_client)

        active = (bt_state == 'active' and controller_state == 'active' and planner_state == 'active')
        unknown_states = (bt_state == 'unknown' and controller_state == 'unknown' and planner_state == 'unknown')
        if require_active and (not active) and unknown_states and self.nav2_active_fallback_on_action:
            if (now - self.startup_time) >= self.nav2_active_grace_sec:
                if self.nav_client.wait_for_server(timeout_sec=0.05):
                    if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                        self.last_tf_check_log_time = now
                        self.get_logger().warn(
                            "⚠️ Nav2 lifecycle states unknown; using action-server readiness fallback"
                        )
                    return True
        if require_active and not active:
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
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.nav2_state_timeout_sec)
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
        
        # If we intentionally canceled due to frontier replan, don't count this as failure.
        if self.replan_cancel_pending and (time.time() - self.last_goal_cancel_time) < 2.0:
            if result.status in (5, 6):
                status_name = 'ABORTED' if result.status == 5 else 'CANCELED'
                self.get_logger().info(
                    f"🔁 Ignoring {status_name} result from intentional replan cancel at ({robot_x:.2f}, {robot_y:.2f})"
                )
                self.replan_cancel_pending = False
                self.last_goal_time = 0.0
                return
        self.replan_cancel_pending = False

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"✅ Reached frontier goal! Robot at ({robot_x:.2f}, {robot_y:.2f})")
            self.consecutive_failures = 0  # Reset failure counter on success
            self.last_successful_goal_pos = (robot_x, robot_y)
            self.goals_reached += 1  # Track successful goals for completion check
            self.get_logger().info(f"📊 Goals reached: {self.goals_reached}/{self.min_goals_for_complete}")
            if self.avoid_revisit:
                self.visited_goals.append((robot_x, robot_y))
            if self.strict_no_revisit and self.last_goal_target is not None:
                self.strict_avoid_goals.append(self.last_goal_target)
        elif result.status == 5:  # ABORTED
            self.consecutive_failures += 1
            self.get_logger().error(f"❌ Goal ABORTED! Failure #{self.consecutive_failures}/{self.max_consecutive_failures} at pos ({robot_x:.2f}, {robot_y:.2f})")
            self.get_logger().error(f"   Likely cause: No valid path found by Nav2 planner")

            # Blacklist this goal to avoid immediately retrying the same spot
            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)
                if self.strict_no_revisit:
                    self.strict_avoid_goals.append(self.last_goal_target)
            
            # Check if obstacle is currently detected
            if self.obstacle_detected:
                # Obstacle present - enter OBSTACLE phase to handle it
                self.get_logger().warn(f"🚧 Goal aborted AND obstacle detected at {self.obstacle_distance_m:.3f}m - entering OBSTACLE handling")
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = time.time()
            else:
                # No immediate obstacle - try a different frontier immediately
                self.get_logger().warn(f"🔄 Goal aborted but path clear - will try new frontier immediately")
                self.last_goal_time = 0.0  # Reset to allow immediate new goal
            
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().error(f"🚨 STUCK DETECTED! {self.consecutive_failures} consecutive failures - entering RECOVERY mode")
                self.current_phase = Phase.RECOVERY
                self.stuck_recovery_in_progress = True
                self.recovery_start_time = time.time()
            else:
                # Clear global costmap immediately so the next plan doesn't hit lethal space
                self._clear_costmaps('aborted_planning_failure')
                self._maybe_lethal_escape(robot_x, robot_y)
        elif result.status == 6:  # CANCELED
            self.get_logger().warn(f"⚠️ Navigation canceled at ({robot_x:.2f}, {robot_y:.2f})")
            if self.last_goal_target is not None and self.strict_no_revisit:
                self.strict_avoid_goals.append(self.last_goal_target)
            # Check if obstacle caused the cancellation
            if self.obstacle_detected:
                self.get_logger().warn(f"🚧 Goal canceled with obstacle present - will wait for obstacle handling")
                # Don't immediately retry - let obstacle handling run
            else:
                # No obstacle and not an intentional replan cancel → treat as a planning failure.
                # bt_navigator sends CANCELED (not ABORTED) when SmacPlanner exhausts iterations,
                # so if we don't count this the failure counter stays 0 forever and we loop endlessly.
                self.consecutive_failures += 1
                self.get_logger().warn(
                    f"🔄 Goal canceled (no obstacle) — treating as planning failure "
                    f"#{self.consecutive_failures}/{self.max_consecutive_failures}"
                )
                # Blacklist this goal so we don't immediately re-send the same unreachable frontier
                if self.last_goal_target is not None:
                    self._blacklist_goal(self.last_goal_target)
                self.last_goal_time = 0.0

                if self.consecutive_failures >= self.max_consecutive_failures:
                    self.get_logger().error(
                        f"🚨 STUCK DETECTED! {self.consecutive_failures} consecutive planning failures — entering RECOVERY mode"
                    )
                    self.current_phase = Phase.RECOVERY
                    self.stuck_recovery_in_progress = True
                    self.recovery_start_time = time.time()
                else:
                    # Clear global costmap immediately so the next plan doesn't hit lethal space
                    self._clear_costmaps('canceled_planning_failure')
                    self._maybe_lethal_escape(robot_x, robot_y)
        else:
            self.get_logger().warn(f"⚠️ Navigation ended with status: {result.status} at ({robot_x:.2f}, {robot_y:.2f})")

    def _blacklist_goal(self, goal_xy):
        now = time.time()
        self.blacklisted_goals[goal_xy] = now + self.blacklist_duration
        self.get_logger().warn(
            f"⚠️ Blacklisting failed goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) for {self.blacklist_duration:.0f}s"
        )

    def _clear_costmaps(self, reason: str):
        if not self.clear_costmap_on_obstacle:
            return
        now = time.time()
        if (now - self.last_costmap_clear_time) < self.costmap_clear_cooldown:
            return
        self.last_costmap_clear_time = now

        req = ClearEntireCostmap.Request()
        if self.clear_local_costmap_client.service_is_ready():
            try:
                self.clear_local_costmap_client.call_async(req)
                self.get_logger().warn(f"🧹 Requested local costmap clear ({reason})")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Local costmap clear failed ({reason}): {e}")
        # NOTE: global costmap is NOT cleared here — nav2 handles lethal-start via
        # footprint_clearing_enabled: true in nav2_params.yaml (global_costmap section).
        # Clearing the global costmap would wipe the static_layer and cause 2s repopulate lag.

    def _maybe_lethal_escape(self, robot_x: float, robot_y: float):
        """Track same-position failures; fire a physical backup escape when stuck in lethal space."""
        if self._lethal_fail_pos is not None:
            dx = robot_x - self._lethal_fail_pos[0]
            dy = robot_y - self._lethal_fail_pos[1]
            if math.sqrt(dx * dx + dy * dy) < 0.25:
                self._lethal_fail_count += 1
            else:
                self._lethal_fail_count = 1
        else:
            self._lethal_fail_count = 1
        self._lethal_fail_pos = (robot_x, robot_y)
        if self._lethal_fail_count >= 2:
            self._fire_lethal_escape()

    def _fire_lethal_escape(self):
        """Physically back up the robot to move out of lethal space on the global costmap."""
        import threading
        self.get_logger().warn(
            f"🏃 Lethal-space escape: {self._lethal_fail_count} consecutive failures "
            f"at same position ({self._lethal_fail_pos[0]:.2f}, {self._lethal_fail_pos[1]:.2f}) "
            f"— backing up to escape lethal costmap cell"
        )
        self._lethal_fail_count = 0
        self._lethal_fail_pos = None
        # Block goal sending for 4 s while we physically move
        self.last_goal_time = time.time() + 4.0

        def _backup_thread():
            try:
                msg = Twist()
                msg.linear.x = -0.15
                end_t = time.time() + 2.0
                while time.time() < end_t:
                    self.cmd_vel_pub.publish(msg)
                    time.sleep(0.1)
                stop = Twist()
                self.cmd_vel_pub.publish(stop)
                self.get_logger().info("✅ Lethal-space backup complete")
            except Exception as exc:
                self.get_logger().warn(f"⚠️ Lethal escape thread error: {exc}")

        threading.Thread(target=_backup_thread, daemon=True).start()

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
        self.get_logger().info("🔍 Starting BACKWARD SCAN...")
        self.scan_in_progress = True
        self.scan_step = 0
        self.scan_segment_index = 0
        self.obstacle_detected = False  # Clear obstacle flag to allow scan to proceed
        # Compute total scan time for RESCAN timing logic
        self.scan_total_time = self.scan_segments * self.scan_steps_per_angle * 0.05
        
    def execute_scan_step(self):
        """Execute one step of backward scan - called from main loop (NON-BLOCKING)"""
        if not self.scan_in_progress:
            return False  # Scan not active
        
        # SAFETY: If rear is too close, stop backing up; otherwise rotate to clear the front.
        if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
            self.scan_in_progress = False
            self.get_logger().error(
                "🛑 Rear obstacle too close; stopping backward scan and holding position."
            )
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            return False
        elif self.rear_obstacle_detected:
            # Turn in place to search for a clear direction without backing into the rear obstacle.
            rotate_msg = Twist()
            rotate_msg.angular.z = 0.4
            self.cmd_vel_pub.publish(rotate_msg)
            self.scan_step += 1
            if self.scan_step >= self.scan_steps_per_angle:
                self.scan_step = 0
                self.scan_segment_index += 1
            return True
        
        # Check if we've finished all scan segments
        if self.scan_segment_index >= self.scan_segments:
            # Scan complete - stop motion
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.scan_in_progress = False
            self.get_logger().info("✅ Backward scan complete")
            return False  # Scan finished
        
        # Move backward only when front is not critically close; otherwise rotate to clear front.
        if self.front_obstacle_detected and self.obstacle_distance_m <= self.front_emergency_rotate_distance:
            rotate_msg = Twist()
            rotate_msg.angular.z = self.corner_recovery_turn_speed
            self.cmd_vel_pub.publish(rotate_msg)
        else:
            backup_msg = Twist()
            backup_msg.linear.x = -0.06  # Short, gentle backward
            self.cmd_vel_pub.publish(backup_msg)
        
        self.scan_step += 1
        
        # Check if angle duration complete
        if self.scan_step >= self.scan_steps_per_angle:
            self.scan_step = 0
            self.scan_segment_index += 1
        
        return True  # Scan still in progress

    def _maybe_auto_publish_initial_pose(self, init_time_elapsed: float):
        if self.initial_pose_set or self.auto_initial_pose_published:
            return
        if not self.auto_publish_initial_pose:
            return
        if init_time_elapsed < self.auto_publish_initial_pose_timeout:
            return

        self.update_pose()
        if not self.pose_valid:
            now = time.time()
            if (now - self.last_auto_pose_log_time) >= 2.0:
                self.last_auto_pose_log_time = now
                self.get_logger().warn("⚠️ Auto initial-pose publish deferred: pose not valid yet")
            return

        x, y, theta = self.robot_pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(theta * 0.5)
        pose_msg.pose.pose.orientation.w = math.cos(theta * 0.5)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.068
        pose_msg.pose.covariance = covariance

        self.initialpose_pub.publish(pose_msg)
        self.initial_pose_set = True
        self.auto_initial_pose_published = True
        self.get_logger().warn(
            f"⚠️ Auto-published initial pose after {init_time_elapsed:.1f}s at x={x:.2f}, y={y:.2f}, yaw={math.degrees(theta):.1f}°"
        )
    
    def main_loop(self):
        """Main exploration state machine"""
        # When exploration is done, stop all motion and do nothing
        if self.current_phase == Phase.DONE:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            return

        # ── Universal coverage gate ────────────────────────────────────────────
        # Regardless of frontiers or phase, once coverage >= 80% we are done.
        if (self.current_phase not in (Phase.INIT,) and
                self.last_percent_known >= self.zero_frontier_complete_percent):
            self.get_logger().info(
                f"🎉 EXPLORATION COMPLETE! Coverage {self.last_percent_known:.1f}% "
                f">= {self.zero_frontier_complete_percent:.1f}% threshold."
            )
            try:
                if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                    self.goal_handle = None
                    self.goal_in_progress = False
            except Exception:
                pass
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.current_phase = Phase.DONE
            self._on_exploration_complete()
            return

        if self.strict_obstacle_handling or not self.nav2_handles_obstacles:
            # Ultrasonic emergency backup (2s) at very close range
            if time.time() < self.ultrasonic_emergency_until:
                if self.rear_obstacle_detected or (
                    self.front_obstacle_detected and
                    self.obstacle_distance_m <= self.front_emergency_rotate_distance
                ):
                    rotate_msg = Twist()
                    rotate_msg.angular.z = self.corner_recovery_turn_speed
                    self.cmd_vel_pub.publish(rotate_msg)
                    return
                backup_msg = Twist()
                backup_msg.linear.x = self.backup_speed
                self.cmd_vel_pub.publish(backup_msg)
                return
            # LiDAR-triggered backup
            if time.time() < self.lidar_backup_until:
                if not self.front_obstacle_detected:
                    self.lidar_backup_until = 0.0
                    return
                if self.rear_obstacle_detected or self.obstacle_distance_m <= self.front_emergency_rotate_distance:
                    rotate_msg = Twist()
                    rotate_msg.angular.z = self.corner_recovery_turn_speed
                    self.cmd_vel_pub.publish(rotate_msg)
                    return
                backup_msg = Twist()
                backup_msg.linear.x = self.backup_speed
                self.cmd_vel_pub.publish(backup_msg)
                return
        # Global safety stop handling: custom obstacle handling only when enabled
        if ((self.strict_obstacle_handling or not self.nav2_handles_obstacles) and self.obstacle_detected and
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
            percent_complete = self.last_percent_known if hasattr(self, 'last_percent_known') else 0.0
            self.get_logger().info(
                f"📊 Phase: {phase_name} | Frontiers: {frontier_count} | Failures: {self.consecutive_failures}/{self.max_consecutive_failures} | Map completion: {percent_complete:.2f}%"
            )
            # Always print map completion percentage for debugging
            self.get_logger().info(f"[DEBUG] Map completion percentage: {percent_complete:.2f}%")
            # Periodic odom freshness check
            if not self._odom_recent():
                if now - self.last_odom_stale_log_time > self.odom_stale_log_interval:
                    self.last_odom_stale_log_time = now
                    self.get_logger().warn("⚠️ No /odom or odom TF updates in the last 1s. Check Arduino bridge and TF.")
        
        # ==== PHASE 1: INITIALIZATION ====
        if self.current_phase == Phase.INIT:
            # Throttled startup status logging
            # Update pose during init to verify localization
            self.update_pose()

            init_time_elapsed = time.time() - self.startup_time
            self._maybe_auto_publish_initial_pose(init_time_elapsed)
            frontiers_ready = len(self.current_frontiers) > 0
            scan_ready = (time.time() - self.last_scan_time) <= self.lidar_stale_timeout
            lidar_ready = frontiers_ready or scan_ready or (
                (time.time() - self.last_frontier_time) <= self.frontier_lidar_fallback_timeout
            )
            pose_ready = self.pose_valid and not self.pose_stale

            nav2_server_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
            nav2_services_ready = self._nav2_services_ready()
            nav2_lifecycle_active = self._nav2_active(
                require_active=self.require_nav2_active,
                services_ready=nav2_services_ready
            )
            nav2_ready = nav2_server_ready and (nav2_lifecycle_active if self.require_nav2_active else True)

            try:
                can_map_odom = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.05))
                can_odom_base = self.tf_buffer.can_transform('odom', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
                tf_ready = can_map_odom and can_odom_base
            except Exception:
                tf_ready = False

            costmap_ready = (
                (self.costmap is not None) or
                (self.local_costmap is not None) or
                self.costmap_raw_received or
                self.local_costmap_raw_received
            )
            if self.require_costmap and not costmap_ready and self.init_start_time is not None:
                if self.costmap_wait_timeout > 0.0 and (time.time() - self.init_start_time) >= self.costmap_wait_timeout:
                    if not self.costmap_waited_out:
                        self.costmap_waited_out = True
                        self.get_logger().warn(
                            "⚠️ Costmap not received in time; proceeding without startup costmap gate."
                        )
            costmap_gate = costmap_ready or (self.require_costmap and self.costmap_waited_out)

            clear_for_hold = (time.time() - self.last_obstacle_time) >= self.startup_clear_hold_time
            obstacle_blocking = self.obstacle_detected and (self.strict_obstacle_handling or not self.nav2_handles_obstacles)

            # Handle obstacle blocking with timeout - proceed after max_obstacle_block_time
            if obstacle_blocking:
                if self.obstacle_blocking_start == 0.0:
                    self.obstacle_blocking_start = time.time()
                    self.get_logger().warn(f"⚠️ Obstacle detected at startup - waiting up to {self.max_obstacle_block_time}s for clearance")
                elif (time.time() - self.obstacle_blocking_start) >= self.max_obstacle_block_time:
                    # Timeout reached - proceed despite obstacle
                    self.get_logger().warn(f"⚠️ Obstacle timeout reached ({self.max_obstacle_block_time}s) - proceeding anyway")
                    obstacle_blocking = False
            else:
                self.obstacle_blocking_start = 0.0

            nav2_acceptable = nav2_lifecycle_active if self.require_nav2_active else nav2_ready
            odom_ready = self._odom_recent()

            frontier_timeout_reached = (
                self.startup_frontier_timeout > 0.0 and
                init_time_elapsed >= self.startup_frontier_timeout
            )
            lidar_gate = lidar_ready or frontier_timeout_reached
            frontiers_gate = frontiers_ready or frontier_timeout_reached
            if frontier_timeout_reached and (not frontiers_ready) and (not self.startup_frontier_timeout_warned):
                self.startup_frontier_timeout_warned = True
                self.get_logger().warn(
                    f"⚠️ No frontiers after {self.startup_frontier_timeout:.1f}s; starting explore mode anyway"
                )

            # During INIT, only require pose_valid (TF exists), not pose_ready
            # (pose_ready also requires non-stale TF, but SLAM TF is always stale
            # at startup until it processes the first scans — a chicken-and-egg).
            init_pose_ok = self.pose_valid
            gate_ok = (
                init_time_elapsed >= self.nav2_activation_time and
                lidar_gate and init_pose_ok and
                costmap_gate and tf_ready and nav2_acceptable and odom_ready and clear_for_hold and
                not obstacle_blocking
            )

            if not gate_ok and (time.time() - self.last_init_gate_log_time) >= self.init_gate_log_interval:
                self.last_init_gate_log_time = time.time()
                self.get_logger().info(
                    "⛳ INIT gate: "
                    f"elapsed={init_time_elapsed:.1f}/{self.nav2_activation_time:.1f}, "
                    f"scan_ready={scan_ready}, lidar_ready={lidar_ready}, lidar_gate={lidar_gate}, frontiers={frontiers_ready}, frontiers_gate={frontiers_gate}, "
                    f"pose_valid={self.pose_valid}, pose_stale={self.pose_stale}, costmap={costmap_ready}, costmap_gate={costmap_gate}, "
                    f"tf_ready={tf_ready}, nav2_server={nav2_server_ready}, nav2_ready={nav2_ready}, "
                    f"odom_ready={odom_ready}, "
                    f"clear={clear_for_hold}, obstacle={self.obstacle_detected}, blocking={obstacle_blocking}"
                )
            if gate_ok:
                if not self.initial_pose_set:
                    self.get_logger().warn("Waiting for initial pose to be set via RViz 2D Pose Estimate before starting exploration.")
                    return
                self.exploration_start_time = time.time()  # Mark when exploration actually starts
                self.update_pose()
                self.home_pose = self.robot_pose  # remember start for return-home
                self.get_logger().info(
                    f"🏠 Home pose recorded: ({self.home_pose[0]:.2f}, {self.home_pose[1]:.2f})")
                self.get_logger().info("✅ Startup complete! Starting exploration")
                self.current_phase = Phase.EXPLORE
                self.phase_start_time = time.time()
        
        # ==== PHASE 2: EXPLORE ====
        elif self.current_phase == Phase.EXPLORE:
            # Check if obstacles detected - TRIGGER IMMEDIATE AVOIDANCE (only in EXPLORE phase)
            if (self.strict_obstacle_handling or not self.nav2_handles_obstacles) and self.obstacle_detected:
                self.get_logger().error(f"💥💥💥 OBSTACLE at {self.obstacle_distance_m:.3f}m - OBSTACLE PHASE!")
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = time.time()
                return

            now = time.time()

            # ── Static-obstacle deep escape (NOT rate-limited — runs every loop tick) ──
            # If a STATIC obstacle has blocked the front for > trigger_time seconds the
            # normal spin recovery has clearly failed.  Execute: backup → turn → resume.
            static_stuck_duration = (now - self.static_stuck_start) if self.static_stuck_start > 0.0 else 0.0
            if static_stuck_duration >= self.static_stuck_trigger_time:
                if not self.static_stuck_escape_active:
                    self.static_stuck_escape_active = True
                    self.static_stuck_escape_step = 0
                    self.static_stuck_escape_step_start = now
                    self.static_stuck_escape_attempts += 1
                    self.static_stuck_escape_turn_dir = (
                        1.0 if (self.static_stuck_escape_attempts % 2 == 1) else -1.0
                    )
                    self.get_logger().error(
                        f"🆘 Static obstacle stuck {static_stuck_duration:.1f}s — "
                        f"deep escape #{self.static_stuck_escape_attempts} "
                        f"({'LEFT' if self.static_stuck_escape_turn_dir > 0 else 'RIGHT'})"
                    )
                    # Blacklist the Nav2 goal that brought us near this wall
                    if self.last_goal_target is not None:
                        self._blacklist_goal(self.last_goal_target)
                    # Blacklist robot's current stuck position so nearby frontiers are skipped
                    self.update_pose()
                    if self.pose_valid:
                        stuck_key = (round(self.robot_pose[0], 2), round(self.robot_pose[1], 2))
                        self._blacklist_goal(stuck_key)
                        self.get_logger().warn(
                            f"⚠️ Blacklisting stuck position ({stuck_key[0]:.2f}, {stuck_key[1]:.2f}) "
                            f"for {self.blacklist_duration:.0f}s"
                        )
                    # Cancel any active Nav2 goal and corner recovery spin
                    try:
                        if self.goal_handle is not None:
                            self.goal_handle.cancel_goal_async()
                            self.goal_handle = None
                            self.goal_in_progress = False
                    except Exception:
                        pass
                    self.corner_recovery_until = 0.0

            if self.static_stuck_escape_active:
                step_elapsed = now - self.static_stuck_escape_step_start
                escape_msg = Twist()
                if self.static_stuck_escape_step == 0:
                    # Step 0: back away from the wall
                    if step_elapsed < self.static_stuck_escape_backup_dur:
                        escape_msg.linear.x = self.backup_speed  # negative = backward
                        self.cmd_vel_pub.publish(escape_msg)
                        return
                    else:
                        self.static_stuck_escape_step = 1
                        self.static_stuck_escape_step_start = now
                        step_elapsed = 0.0
                if self.static_stuck_escape_step == 1:
                    # Step 1: rotate to a new heading
                    if step_elapsed < self.static_stuck_escape_turn_dur:
                        escape_msg.angular.z = (
                            self.static_stuck_escape_turn_dir * self.corner_recovery_turn_speed
                        )
                        self.cmd_vel_pub.publish(escape_msg)
                        return
                    else:
                        # Escape complete — reset state and resume frontier exploration
                        self.static_stuck_escape_active = False
                        self.static_stuck_start = 0.0
                        self.no_frontier_cycles = 0
                        self.last_goal_time = 0.0   # allow immediate goal pick
                        self.get_logger().warn("✅ Deep escape complete — resuming exploration")
                        return

            # Rate limit frontier checks to avoid spamming at 20Hz
            if now - self.last_explore_check < self.explore_check_interval:
                return  # Skip this cycle
            self.last_explore_check = now

            # If a corner recovery is active, keep it running for the duration
            if self.corner_recovery_until > 0.0:
                if now >= self.corner_recovery_until:
                    self.corner_recovery_until = 0.0
                    self.corner_recovery_mode = None
                else:
                    recovery_msg = Twist()
                    turn_dir = 1.0 if (self.no_frontier_cycles % 2 == 0) else -1.0
                    if self.corner_recovery_mode == "backup":
                        if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
                            recovery_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                        else:
                            recovery_msg.linear.x = self.backup_speed
                    else:
                        recovery_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                    self.cmd_vel_pub.publish(recovery_msg)
                    return

            # If a replan goal is pending and no goal is active, send it now
            if self.pending_replan_goal and not (self.goal_handle or self.goal_in_progress):
                if (now - self.last_goal_cancel_time) >= self.replan_cancel_cooldown:
                    goal_x, goal_y = self.pending_replan_goal
                    self.pending_replan_goal = None
                    self.last_goal_time = now
                    self.send_goal_to_nav2(goal_x, goal_y)
                return
            
            # Check if we have an active goal or one in progress - wait for it
            if self.goal_handle is not None or self.goal_in_progress:
                if self.cancel_active_goal_for_replan and self.replan_on_frontier_update and self.last_frontier_update_time > self.last_replan_time:
                    self._refresh_pending_goal_from_frontiers("frontier update")
                    if self.pending_replan_goal is not None:
                        if (now - self.last_goal_time) < self.goal_commit_before_replan_cancel:
                            return
                        if self.goal_handle is not None and (now - self.last_goal_cancel_time) >= self.replan_cancel_cooldown:
                            try:
                                cancel_future = self.goal_handle.cancel_goal_async()
                                cancel_future.add_done_callback(
                                    lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested (replan)")
                                )
                                self.replan_cancel_pending = True
                            except Exception as e:
                                self.get_logger().warn(f"⚠️ Replan cancel failed: {e}")
                        self.last_goal_cancel_time = now
                return
            
            # Check cooldown - don't spam goals
            time_since_last_goal = now - self.last_goal_time
            if time_since_last_goal < self.goal_cooldown:
                # Still in cooldown period, wait
                return
            
            # Pick best frontier
            goal_info = self.pick_best_frontier()
            if goal_info is None:
                if self.enable_simple_exploration and self.no_frontier_cycles >= 2:
                    if not self.simple_exploration_active:
                        self.simple_exploration_active = True
                        self.simple_exploration_start_time = now
                        self.get_logger().warn("🔄 No frontiers detected; starting simple exploration bootstrap")

                    elapsed_simple = now - self.simple_exploration_start_time
                    if elapsed_simple <= self.simple_exploration_duration:
                        bootstrap_msg = Twist()
                        cycle_period = self.simple_exploration_turn_time + self.simple_exploration_move_time
                        phase = elapsed_simple % cycle_period
                        cycle_index = int(elapsed_simple / cycle_period)
                        turn_dir = 1.0 if (cycle_index % 2 == 0) else -1.0
                        if phase < self.simple_exploration_turn_time:
                            bootstrap_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                        else:
                            if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
                                bootstrap_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                            else:
                                bootstrap_msg.linear.x = max(0.05, min(0.12, abs(self.backup_speed)))
                        self.cmd_vel_pub.publish(bootstrap_msg)
                        return
                    else:
                        self.simple_exploration_active = False
                else:
                    self.simple_exploration_active = False

                if self.stop_when_no_frontiers and self.last_frontier_skip_reason == "no_frontiers":
                    if self.rear_obstacle_detected:
                        stop_msg = Twist()
                        self.cmd_vel_pub.publish(stop_msg)
                    else:
                        backup_msg = Twist()
                        backup_msg.linear.x = self.backup_speed
                        self.cmd_vel_pub.publish(backup_msg)
                if self.last_frontier_skip_reason in ("pose_invalid", "pose_stale"):
                    return

                self.no_frontier_cycles += 1

                # ── Immediate completion: 0 frontiers + coverage >= 80% ────────────────
                # Don't wait for the full no_frontier_complete_cycles counter.
                # If the detector sees nothing AND we've already covered enough, stop now.
                _imm_frontiers = len(self.current_frontiers)
                if _imm_frontiers == 0 and self.last_percent_known >= self.zero_frontier_complete_percent:
                    self.get_logger().info(
                        f"🎉 EXPLORATION COMPLETE! 0 frontiers + coverage "
                        f"{self.last_percent_known:.1f}% >= {self.zero_frontier_complete_percent:.1f}%"
                    )
                    self.get_logger().info(f"🗺️ Final map coverage: {self.last_percent_known:.2f}%")
                    try:
                        if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                            self.goal_handle.cancel_goal_async()
                            self.goal_handle = None
                            self.goal_in_progress = False
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
                    try:
                        stop_msg = Twist()
                        self.cmd_vel_pub.publish(stop_msg)
                        if hasattr(self, 'cmd_vel_nav_pub'):
                            self.cmd_vel_nav_pub.publish(stop_msg)
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ Could not publish stop: {e}")
                    self.current_phase = Phase.DONE
                    self._on_exploration_complete()
                    return

                if (
                    self.no_frontier_cycles >= self.no_frontier_recovery_cycles and
                    self.no_frontier_cycles < self.no_frontier_complete_cycles and
                    (self.no_frontier_cycles % self.no_frontier_recovery_cycles) == 0
                ):
                    if self.corner_recovery_until <= 0.0:
                        if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
                            self.corner_recovery_mode = "rotate"
                        else:
                            self.corner_recovery_mode = "backup"
                        self.corner_recovery_until = now + self.corner_recovery_time
                        self.get_logger().warn(
                            f"🧭 No frontiers for {self.no_frontier_cycles} cycles - corner recovery: {self.corner_recovery_mode}"
                        )
                    recovery_msg = Twist()
                    turn_dir = 1.0 if (self.no_frontier_cycles % 2 == 0) else -1.0
                    if self.corner_recovery_mode == "backup":
                        if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
                            recovery_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                        else:
                            recovery_msg.linear.x = self.backup_speed
                    else:
                        recovery_msg.angular.z = turn_dir * self.corner_recovery_turn_speed
                    self.cmd_vel_pub.publish(recovery_msg)
                    return

                if self.no_frontier_cycles >= self.no_frontier_complete_cycles:
                    total_frontiers = len(self.current_frontiers)

                    # ── Rule 1: Zero frontiers ──────────────────────────────────────────
                    # If the detector finds no frontiers at all the robot cannot map further
                    # regardless of current coverage percentage. Declare complete immediately.
                    if total_frontiers == 0:
                        self.get_logger().info(
                            f"🎉 EXPLORATION COMPLETE! 0 frontiers detected. "
                            f"Final coverage: {self.last_percent_known:.1f}%"
                        )
                        self.get_logger().info(f"   Goals reached: {self.goals_reached}")
                        self.get_logger().info(f"🗺️ Map completion: {self.last_percent_known:.2f}% of the map explored.")
                        try:
                            if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                                cancel_future = self.goal_handle.cancel_goal_async()
                                cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested"))
                                self.goal_handle = None
                                self.goal_in_progress = False
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
                        try:
                            stop_msg = Twist()
                            self.cmd_vel_pub.publish(stop_msg)
                            if hasattr(self, 'cmd_vel_nav_pub'):
                                self.cmd_vel_nav_pub.publish(stop_msg)
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not publish stop command: {e}")
                        self.current_phase = Phase.DONE
                        self._on_exploration_complete()
                        return

                    # ── Rule 2: Coverage >= minimum threshold (frontiers may still exist) ──
                    # If coverage is already at/above the zero-frontier threshold the
                    # remaining frontiers are likely unreachable. Declare complete.
                    if self.last_percent_known >= self.zero_frontier_complete_percent:
                        self.get_logger().info(
                            f"🎉 EXPLORATION COMPLETE! Coverage {self.last_percent_known:.1f}% "
                            f">= minimum {self.zero_frontier_complete_percent:.1f}% "
                            f"(with {total_frontiers} unreachable frontier(s))."
                        )
                        self.get_logger().info(f"   Goals reached: {self.goals_reached}")
                        self.get_logger().info(f"🗺️ Map completion: {self.last_percent_known:.2f}% of the map explored.")
                        try:
                            if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                                cancel_future = self.goal_handle.cancel_goal_async()
                                cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested"))
                                self.goal_handle = None
                                self.goal_in_progress = False
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
                        try:
                            stop_msg = Twist()
                            self.cmd_vel_pub.publish(stop_msg)
                            if hasattr(self, 'cmd_vel_nav_pub'):
                                self.cmd_vel_nav_pub.publish(stop_msg)
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not publish stop command: {e}")
                        self.current_phase = Phase.DONE
                        self._on_exploration_complete()
                        return

                    # ── Rule 3: Frontiers exist and coverage target reached ──────────────
                    # Frontiers are present but the robot has already mapped enough of the
                    # known space. Declare complete.
                    if self.last_percent_known >= self.coverage_complete_percent:
                        self.get_logger().info(
                            f"🎉 EXPLORATION COMPLETE! Coverage {self.last_percent_known:.1f}% "
                            f">= target {self.coverage_complete_percent:.1f}%."
                        )
                        self.get_logger().info(f"   Goals reached: {self.goals_reached}")
                        self.get_logger().info(f"   Remaining frontiers: {total_frontiers}")
                        self.get_logger().info(f"🗺️ Map completion: {self.last_percent_known:.2f}% of the map explored.")
                        try:
                            if hasattr(self, 'goal_handle') and self.goal_handle is not None:
                                cancel_future = self.goal_handle.cancel_goal_async()
                                cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested"))
                                self.goal_handle = None
                                self.goal_in_progress = False
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
                        try:
                            stop_msg = Twist()
                            self.cmd_vel_pub.publish(stop_msg)
                            if hasattr(self, 'cmd_vel_nav_pub'):
                                self.cmd_vel_nav_pub.publish(stop_msg)
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Could not publish stop command: {e}")
                        self.current_phase = Phase.DONE
                        self._on_exploration_complete()
                        return

                    # ── Rule 3: Frontiers exist and coverage < target ───────────────────
                    # Still work to do — reset the no-frontier counter and keep exploring.
                    self.get_logger().warn(
                        f"⚠️ Coverage {self.last_percent_known:.1f}% < {self.coverage_complete_percent:.1f}% "
                        f"with {total_frontiers} frontier(s) remaining — continuing exploration."
                    )
                    self.no_frontier_cycles = 0
                    return
                else:
                    self.update_pose()
                    robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
                    self.get_logger().warn(
                        f"⚠️ No valid frontiers! (cycle {self.no_frontier_cycles}/{self.no_frontier_complete_cycles}) Robot at ({robot_x:.2f}, {robot_y:.2f})"
                    )
                return
            
            # Reset frontier tracking when we find a goal
            self.no_frontier_cycles = 0
            self.frontier_stable_count = 0
            self.last_known_frontier_count = len(self.current_frontiers)
            self.last_frontier_check_time = time.time()
            
            goal_x, goal_y = goal_info['goal']
            frontier_x, frontier_y = goal_info['frontier']
            total_frontiers = len(self.current_frontiers)
            self.update_pose()
            robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
            distance_to_goal = goal_info['dist']
            self.get_logger().info(
                f"🎯 New frontier detected: Robot at ({robot_x:.2f}, {robot_y:.2f}) → Frontier ({frontier_x:.2f}, {frontier_y:.2f}) → Goal ({goal_x:.2f}, {goal_y:.2f})"
            )
            self.get_logger().info(
                f"📏 Distance to goal: {distance_to_goal:.2f}m | Available frontiers: {total_frontiers}"
            )
            self.ever_sent_goal = True
            self.last_goal_time = now  # Record goal send time
            self.send_goal_to_nav2(goal_x, goal_y, goal_info.get('costmap_filtered'), frontier_xy=(frontier_x, frontier_y))
        
        # ==== PHASE 3: OBSTACLE HANDLING ====
        elif self.current_phase == Phase.OBSTACLE:
            self._refresh_pending_goal_from_frontiers("obstacle")
            self.get_logger().error("📍 OBSTACLE PHASE: Cancel goal and prepare for RESCAN")
            self._clear_costmaps("obstacle")

            # Avoid selecting the same blocked path again after rescan
            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)

            self.emergency_backup()
            # Switch to RESCAN phase (will execute on next loop)
            self.rescan_done = False  # Reset flag so scan will execute
            self.current_phase = Phase.RESCAN
            self.phase_start_time = time.time()
        
        # ==== PHASE 4: RESCAN ====
        elif self.current_phase == Phase.RESCAN:
            self._refresh_pending_goal_from_frontiers("rescan")
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
                        self.update_pose()
                        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
                        self.get_logger().info(
                            f"✅ Resuming exploration: Robot at ({robot_x:.2f}, {robot_y:.2f}) → Frontier ({fx:.2f}, {fy:.2f}) → Goal ({gx:.2f}, {gy:.2f})"
                        )
                        self.current_phase = Phase.EXPLORE
                        self.last_goal_time = 0.0
                        self.send_goal_to_nav2(gx, gy, new_frontier.get('costmap_filtered'), frontier_xy=(fx, fy))
                    else:
                        self.get_logger().warn("⚠️ Rescan clear but no frontier selected yet; returning to EXPLORE")
                        self.current_phase = Phase.EXPLORE
                elif elapsed > 8.0:
                    # Still blocked after 8 seconds, retry
                    self.get_logger().error("⚠️ RESCAN: Path STILL blocked after 8s - retrying backup")
                    self.current_phase = Phase.OBSTACLE
                    self.rescan_done = False
        
        # ==== PHASE 5: RECOVERY (Stuck in corner) ====
        elif self.current_phase == Phase.RECOVERY:
            self._clear_costmaps("recovery")
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
            if self.current_frontiers and self._nav2_active(require_active=False):
                self.get_logger().info("🔄 New frontiers detected; resuming exploration")
                self.no_frontier_cycles = 0
                self.current_phase = Phase.EXPLORE
                self.phase_start_time = time.time()
                return
            now = time.time()
            if now - self.last_done_log_time >= self.done_log_interval:
                self.last_done_log_time = now
                self.get_logger().info("🏁 Mission complete!")
            if not self.complete_marker_sent:
                self._on_exploration_complete()

    def _on_exploration_complete(self):
        # Guard: only handle once — atomically checked and set under a lock
        with self._exploration_complete_lock:
            if self._exploration_complete_handled:
                return
            self._exploration_complete_handled = True

        if not self.complete_marker_sent:
            self._publish_complete_marker()
            self.complete_marker_sent = True
        self.get_logger().info(f"🗺️ Map completion: {self.last_percent_known:.2f}% of the map explored.")
        self._save_mapped_area_series()
        self._save_robot_path_series()
        # Save map then shut down
        self._save_and_shutdown()

    def _save_and_shutdown(self):
        """Save map then kill all nodes."""
        import signal as _signal
        import threading as _threading
        import sys as _sys

        def _do_shutdown():
            import time as _time
            # Step 1: Save map
            if self.auto_save_on_complete and not self.map_save_requested:
                self.map_save_requested = True
                self._request_map_save()
            # Wait for file write to finish
            _time.sleep(4.0)
            # Step 2: Stop motors
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.cmd_vel_nav_pub.publish(stop)
            # Step 3: Kill all nodes
            self.get_logger().info("🛑 Auto-shutdown: map saved, terminating all nodes.")
            ppid = os.getppid()
            try:
                pgid = os.getpgid(ppid)
                os.killpg(pgid, _signal.SIGINT)
            except Exception:
                try:
                    os.kill(ppid, _signal.SIGINT)
                except Exception:
                    pass
            _time.sleep(3.0)
            try:
                pgid = os.getpgid(ppid)
                os.killpg(pgid, _signal.SIGTERM)
            except Exception:
                pass
            _time.sleep(2.0)
            _sys.exit(0)

        _threading.Thread(target=_do_shutdown, daemon=False).start()

    def _save_mapped_area_series(self):
        if not self.mapped_area_series:
            self.get_logger().warn('⚠️ No mapped area data to save')
            return
        os.makedirs(self.map_save_dir, exist_ok=True)
        out_path = os.path.join(self.map_save_dir, self.mapped_area_file)
        try:
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('elapsed_s,area_m2,percent_known,known_cells,total_cells\n')
                for row in self.mapped_area_series:
                    f.write(f"{row[0]:.2f},{row[1]:.3f},{row[2]:.2f},{row[3]},{row[4]}\n")
            self.get_logger().info(f"📈 Mapped area data saved to {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save mapped area data: {e}")

    def _record_path_point(self, x: float, y: float):
        """Record robot position when it has moved at least path_record_min_dist metres."""
        min_dist = getattr(self, '_path_record_min_dist_val', None)
        if min_dist is None:
            try:
                min_dist = self.get_parameter('path_record_min_dist').value
            except Exception:
                min_dist = 0.2
            self._path_record_min_dist_val = min_dist
        lx = self.last_recorded_path_x
        ly = self.last_recorded_path_y
        if lx is None or ((x - lx) ** 2 + (y - ly) ** 2) >= min_dist ** 2:
            elapsed = time.time() - self.startup_time
            self.robot_path_series.append((elapsed, x, y))
            self.last_recorded_path_x = x
            self.last_recorded_path_y = y

    def _save_robot_path_series(self):
        """Save the robot's visited waypoints to CSV."""
        if not self.robot_path_series:
            self.get_logger().warn('⚠️ No robot path data to save')
            return
        os.makedirs(self.map_save_dir, exist_ok=True)
        path_file = getattr(self, 'robot_path_file', 'auto_explore_path.csv')
        out_path = os.path.join(self.map_save_dir, path_file)
        try:
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('elapsed_s,x_m,y_m\n')
                for row in self.robot_path_series:
                    f.write(f"{row[0]:.2f},{row[1]:.4f},{row[2]:.4f}\n")
            self.get_logger().info(
                f"🗺️ Robot path saved: {len(self.robot_path_series)} waypoints → {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save robot path: {e}")

    def _publish_complete_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.robot_pose[0])
        marker.pose.position.y = float(self.robot_pose[1])
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.4
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = 'EXPLORATION COMPLETE'
        self.status_marker_pub.publish(marker)

    def _request_map_save(self):
        import struct
        import zlib
        import threading
        import datetime
        def _write_map():
            slam_map = getattr(self, 'last_slam_map', None)
            if slam_map is None:
                self.get_logger().warn('⚠️ No map data received yet; skipping save')
                return
            os.makedirs(self.map_save_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            timed_name = f'map_{ts}'
            map_name = os.path.join(self.map_save_dir, timed_name)
            pgm_path = map_name + '.pgm'
            yaml_path = map_name + '.yaml'
            png_path = map_name + '.png'
            info = slam_map.info
            width = info.width
            height = info.height
            resolution = info.resolution
            ox = info.origin.position.x
            oy = info.origin.position.y
            data = slam_map.data
            # Convert OccupancyGrid to grayscale pixel values
            # ROS convention: -1=unknown→205(gray), 0=free→254(white), 100=occ→0(black)
            pixels = bytearray(width * height)
            for row in range(height):
                for col in range(width):
                    # OccupancyGrid row 0 = bottom, image row 0 = top → flip vertically
                    idx = (height - 1 - row) * width + col
                    v = data[idx]
                    if v < 0:
                        pixels[row * width + col] = 205
                    elif v == 0:
                        pixels[row * width + col] = 254
                    else:
                        pixels[row * width + col] = max(0, 255 - int(v * 2.55))
            try:
                # --- PGM ---
                with open(pgm_path, 'wb') as f:
                    header = f'P5\n# CREATOR: fea_slam\n{width} {height}\n255\n'
                    f.write(header.encode('ascii'))
                    f.write(bytes(pixels))
                # --- YAML ---
                with open(yaml_path, 'w') as f:
                    f.write(f'image: {timed_name}.pgm\n')
                    f.write(f'resolution: {resolution}\n')
                    f.write(f'origin: [{ox:.6f}, {oy:.6f}, 0.000000]\n')
                    f.write('negate: 0\n')
                    f.write('occupied_thresh: 0.65\n')
                    f.write('free_thresh: 0.25\n')
                # --- PNG (pure stdlib: zlib + struct, no Pillow needed) ---
                def _png_chunk(tag, data):
                    c = zlib.crc32(tag + data) & 0xFFFFFFFF
                    return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', c)
                ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)  # 8-bit grayscale
                raw_rows = b''
                for row in range(height):
                    raw_rows += b'\x00' + bytes(pixels[row * width:(row + 1) * width])
                idat = zlib.compress(raw_rows, 6)
                png_bytes = (
                    b'\x89PNG\r\n\x1a\n'
                    + _png_chunk(b'IHDR', ihdr)
                    + _png_chunk(b'IDAT', idat)
                    + _png_chunk(b'IEND', b'')
                )
                with open(png_path, 'wb') as f:
                    f.write(png_bytes)
                self.get_logger().info(
                    f'✅ Map saved: {pgm_path} + {timed_name}.png ({width}x{height})')
            except Exception as e:
                self.get_logger().error(f'❌ Map save error: {e}')
        threading.Thread(target=_write_map, daemon=True).start()

    def _map_save_done(self, future):
        try:
            result = future.result()
            if result is not None and result.result:
                self.get_logger().info('✅ Map saved successfully')
            else:
                self.get_logger().warn('⚠️ Map save failed')
        except Exception as e:
            self.get_logger().error(f'❌ Map save error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        if 'context is not valid' not in str(e):
            raise
    finally:
        try:
            stop_msg = Twist()
            node.cmd_vel_pub.publish(stop_msg)
            node.cmd_vel_nav_pub.publish(stop_msg)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
