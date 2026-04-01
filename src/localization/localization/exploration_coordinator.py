#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Float32
from tf2_ros import TransformListener, Buffer

from enum import Enum
import math
import time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .exploration_sensing_mixin import ExplorationSensingMixin
from .exploration_planning_mixin import ExplorationPlanningMixin
from .exploration_execution_mixin import ExplorationExecutionMixin
from .exploration_persistence_mixin import ExplorationPersistenceMixin
from .phase_enum import Phase

# --- Graceful shutdown handler for guaranteed completion logging ---
import signal
import sys
def _graceful_shutdown_handler(signalnum, frame):
    try:
        node = ExplorationCoordinator._active_instance if hasattr(ExplorationCoordinator, '_active_instance') else None
        if node is not None and hasattr(node, '_on_exploration_complete'):
            node.get_logger().warn('🛑 Signal received: running graceful shutdown handler.')
            node._on_exploration_complete()
    except Exception as e:
        print(f"[GracefulShutdown] Exception: {e}")
    sys.exit(0)
signal.signal(signal.SIGINT, _graceful_shutdown_handler)
signal.signal(signal.SIGTERM, _graceful_shutdown_handler)


class ExplorationCoordinator(
    ExplorationSensingMixin,
    ExplorationPlanningMixin,
    ExplorationExecutionMixin,
    ExplorationPersistenceMixin,
    Node,
):
    # Track the active instance for signal handler
    _active_instance = None

    def __init__(self, *args, **kwargs):
        ExplorationCoordinator._active_instance = self
        super().__init__(*args, **kwargs)

    def _declare_params(self, defaults):
        """Declare all node parameters from a single defaults map."""
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_param(self, name, default):
        """Read one parameter and cast to the type of its default value."""
        value = self.get_parameter(name).value
        if isinstance(default, bool):
            return bool(value)
        if isinstance(default, int) and not isinstance(default, bool):
            return int(value)
        if isinstance(default, float):
            return float(value)
        return value

    def initialpose_cb(self, msg):
        if getattr(self, 'initial_pose_locked', False) and self.lock_initial_pose_after_set:
            now = time.time()
            if (now - self.last_initial_pose_ignore_log_time) >= 2.0:
                self.last_initial_pose_ignore_log_time = now
                self.get_logger().warn("🔒 Ignoring /initialpose: initial pose is locked after startup")
            return
        if self.current_phase != Phase.INIT and not self.allow_initial_pose_updates_after_init:
            now = time.time()
            if (now - self.last_initial_pose_ignore_log_time) >= 2.0:
                self.last_initial_pose_ignore_log_time = now
                self.get_logger().warn("🔒 Ignoring /initialpose outside INIT phase")
            return

        self.initial_pose_set = True
        if self.lock_initial_pose_after_set:
            self.initial_pose_locked = True
        # Disable lethal escape after initial pose
        if hasattr(self, 'disable_lethal_escape'):
            self.disable_lethal_escape()
        self.get_logger().info("✅ Initial pose received. Exploration can begin.")

    def __init__(self):
        super().__init__('exploration_coordinator_v2')
        self.shutdown_by_signal = False
        self.initial_pose_set = False
        self.initialpose_sub_abs = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10
        )
        self.initialpose_sub_rel = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initialpose_cb, 10
        )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.max_total_cells = 0                                        
        self.max_known_cells = 0                                                    
        defaults = {
            'nav2_timeout': 12.0,
            'obstacle_distance': 0.25,
            'backup_speed': -0.18,
            'backup_time': 0.8,
            'use_lidar_obstacle': True,
            'strict_obstacle_handling': True,
            'use_ultrasonic_backup': True,
            'scan_min_range': 0.27,
            'lidar_obstacle_distance': 0.45,
            'ultrasonic_backup_distance': 0.15,
            'ultrasonic_confirm_count': 2,
            'ultrasonic_clear_confirm_count': 2,
            'ultrasonic_min_valid_distance': 0.02,
            'ultrasonic_max_jump': 0.35,
            'ultrasonic_jump_window': 0.20,
            'lidar_motion_window': 0.8,
            'lidar_motion_distance_delta': 0.08,
            'lidar_obstacle_class_log_interval': 2.0,
            'lidar_stale_timeout': 2.0,
            'lidar_backup_on_obstacle': True,
            'swap_lidar_front_back': False,
            'rear_obstacle_threshold': 0.25,
            'rear_obstacle_hold_time': 0.8,
            'front_obstacle_half_angle_deg': 40.0,
            'rear_obstacle_half_angle_deg': 30.0,
            'front_obstacle_min_hits': 2,
            'rear_obstacle_min_hits': 2,
            'rear_emergency_distance': 0.25,
            'front_emergency_rotate_distance': 0.30,
            'frontier_goal_offset': 0.35,
            'min_frontier_distance': 0.3,
            'relaxed_frontier_after_cycles': 2,
            'relaxed_min_frontier_distance': 0.1,
            'relaxed_frontier_goal_offset': 0.05,
            'relaxed_use_costmap_filter': False,
            'blacklist_duration': 3.0,
            'blacklist_radius': 0.25,
            'avoid_revisit': False,
            'strict_no_revisit': False,
            'force_frontier_goal': False,
            'visited_goal_radius': 0.6,
            'known_frontier_avoid_radius': 1.2,
            'prune_mapped_frontiers': True,
            'prune_only_moving_obstacles': True,
            'dynamic_obstacle_prune_hold_sec': 1.5,
            'exclude_invalid_frontiers': True,
            'invalid_frontier_max_occupancy': 70,
            'invalid_frontier_log_interval': 2.0,
            'frontier_lidar_range_m': 3.5,
            'frontier_lidar_range_margin_m': 0.2,
            'frontier_skip_if_within_scan_range': True,
            'frontier_scanned_max_unknown_ratio': 0.03,
            'frontier_scanned_max_unknown_cells': 2,
            'frontier_use_lidar_standoff_goal': True,
            'frontier_goal_standoff_min_m': 1.0,
            'frontier_goal_standoff_max_m': 2.5,
            'frontier_unknown_check_radius_m': 0.45,
            'frontier_min_unknown_ratio': 0.15,
            'frontier_min_unknown_cells': 10,
            'frontier_prune_log_interval': 2.0,
            'recent_goal_radius': 0.8,
            'recent_goal_hold_time': 90.0,
            'nav2_handles_obstacles': True,
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'scan_raw_topic': '/scan_raw',
            'costmap_topic': '/global_costmap/costmap',
            'local_costmap_topic': '/local_costmap/costmap',
            'costmap_raw_topic': '/global_costmap/costmap_raw',
            'local_costmap_raw_topic': '/local_costmap/costmap_raw',
            'frontier_lidar_fallback_timeout': 2.0,
            'costmap_free_threshold': 90,
            'use_costmap_goal_filter': True,
            'pose_movement_threshold': 0.05,
            'pose_stale_timeout': 3.0,
            'tf_recovery_hold_sec': 1.2,
            'defer_lethal_block_until_first_goal': False,
            'post_abort_goal_cooldown_sec': 2.0,
            'require_costmap': True,
            'costmap_wait_timeout': 10.0,
            'require_nav2_active': False,
            'nav2_state_timeout_sec': 0.5,
            'nav2_active_grace_sec': 8.0,
            'nav2_active_fallback_on_action': True,
            'enable_simple_exploration': True,
            'stop_when_no_frontiers': True,  # Stop if there are no frontiers
            'no_frontier_recovery_cycles': 1,
            'no_frontier_complete_cycles': 3,
            'corner_recovery_time': 1.5,
            'corner_recovery_turn_speed': 0.65,
            'coverage_complete_percent': 90.0,
            'zero_frontier_complete_percent': 80.0,
            'min_goals_for_complete': 3,
            'require_motion_and_map_growth_for_completion': True,
            'completion_min_displacement_m': 0.30,
            'completion_min_known_cell_gain': 40,
            'small_test_mode': False,
            'complete_on_zero_frontiers': False,  # Only stop when coverage and min_goals are met
            'auto_save_on_complete': True,
            'map_topic': '/map',
            'map_save_dir': '/home/pi/FEA_SLAM_WS/saved_maps',
            'map_save_name': 'auto_explore_map',
            'mapped_area_file': 'auto_explore_area.csv',
            'robot_path_file': 'auto_explore_path.csv',
            'nav_goals_file': 'auto_explore_nav_goals.csv',
            'frontier_history_file': 'auto_explore_frontiers.csv',
            'path_record_min_dist': 0.2,
            'replan_on_frontier_update': True,
            'replan_interval': 2.0,  # More frequent replanning
            'replan_goal_change_distance': 0.5,  # More sensitive to new/better goals
            'replan_min_improvement': 0.1,  # Accept smaller improvements
            'always_replan_on_frontier_update': True,  # Always replan on new frontiers
            'replan_hard_min_interval': 3.0,  # Lower minimum interval
            'replan_cancel_cooldown': 0.5,  # Allow faster cancel/replan
            'cancel_active_goal_for_replan': True,  # Cancel current goal for better one
            'goal_commit_before_replan_cancel': 2.0,  # Shorter commit time before cancel
            'allow_costmapless_replan_candidates': True,  # Consider more candidates
            'enable_bootstrap_frontier_fallback': True,
            'bootstrap_fallback_min_distance': 0.75,
            'avoid_return_radius': 1.0,
            'avoid_last_goal_radius': 0.8,
            'frontier_pick_farthest': False,
            'frontier_selection_method': 'astar',
            'astar_max_candidates': 30,
            'astar_max_expansions': 12000,
            'clear_costmap_on_obstacle': True,
            'costmap_clear_cooldown': 2.0,
            'frontier_stagnation_timeout': 240.0,
            'frontier_stagnation_drop_pct': 10.0,
            'frontier_goal_refresh_min_interval': 0.8,
            'goal_ack_timeout': 2.0,
            'goal_watchdog_timeout': 18.0,
            'goal_watchdog_min_motion': 0.15,
            'frontier_change_pos_quant': 0.10,
            'frontier_history_sample_interval': 1.0,
            'startup_clear_hold_time': 0.0,
            'startup_frontier_timeout': 20.0,
            'auto_publish_initial_pose': True,
            'auto_publish_initial_pose_timeout': 15.0,
            'initial_pose_wait_timeout': 30.0,
            'lock_initial_pose_after_set': True,
            'allow_initial_pose_updates_after_init': False,
            'startup_health_window_sec': 4.0,
            'startup_require_scan_recent': False,
            'startup_require_odom_recent': False,
            'startup_require_fresh_pose': True,
            'startup_max_tf_age_sec': 1.0,
        }

        self._declare_params(defaults)
        for name, default in defaults.items():
            setattr(self, name, self._read_param(name, default))

        self.backup_iterations = int(self.backup_time / 0.05)
        self.front_obstacle_half_angle = math.radians(self.front_obstacle_half_angle_deg)
        self.rear_obstacle_half_angle = math.radians(self.rear_obstacle_half_angle_deg)
        self._path_record_min_dist_val = self.path_record_min_dist
        self.pose_movement_threshold = 0.0
        self.costmap_waited_out = False
        if not self.strict_no_revisit:
            self.avoid_revisit = False
        
        self.startup_time = time.time()
        self.init_start_time = self.startup_time
        import datetime as _dt_csv
        _session_ts = _dt_csv.datetime.fromtimestamp(self.startup_time).strftime('%Y-%m-%d_%H-%M-%S')
        self.map_save_dir = f'/home/pi/FEA_SLAM_WS/saved_maps/session_{_session_ts}'
        self.mapped_area_file = f'explore_area_{_session_ts}.csv'
        self.robot_path_file = f'explore_path_{_session_ts}.csv'
        self.nav_goals_file = f'explore_nav_goals_{_session_ts}.csv'
        self.frontier_history_file = f'explore_frontiers_{_session_ts}.csv'
        self.session_summary_file = f'explore_summary_{_session_ts}.csv'
        self.nav2_activation_time = 5.0
        self.current_phase = Phase.INIT
        self.phase_start_time = None                                        
        self.robot_pose = (0.0, 0.0, 0.0)               
        self.home_pose = None                                                 
        self.exploration_start_known_cells = 0
        self.last_completion_guard_log_time = 0.0
        self.current_frontiers = []
        self.obstacle_detected = False
        self.obstacle_distance_m = float('inf')
        self.last_obstacle_time = 0.0
        self.obstacle_hold_time = 1.0                              
        self.ultrasonic_obstacle_hits = 0
        self.ultrasonic_clear_hits = 0
        self.last_ultrasonic_distance = None
        self.last_ultrasonic_time = 0.0
        self.last_ultrasonic_filter_log_time = 0.0
        self.ultrasonic_filter_log_interval = 2.0
        self.front_obstacle_detected = False
        self.last_front_obstacle_time = 0.0
        self.last_lidar_front_distance = None
        self.last_lidar_left_distance = None
        self.last_lidar_right_distance = None
        self.last_lidar_front_time = 0.0
        self.current_lidar_obstacle_type = "unknown"
        self.last_lidar_obstacle_class_log_time = 0.0
        self.last_scan_time = 0.0
        self.last_scan_stale_log_time = 0.0
        self.scan_stale_log_interval = 2.0
        self.rescan_done = False
        self.ultrasonic_emergency_until = 0.0
        self.nav2_ready = False
        self.startup_scan_end_time = None
        self.first_lidar_time = None
        self.first_frontier_time = None
        self.first_pose_time = None
        self.startup_frontier_timeout_warned = False
        self.auto_initial_pose_published = False
        self.initial_pose_locked = False
        self.last_initial_pose_ignore_log_time = 0.0
        self.last_auto_pose_log_time = 0.0
        self.last_frontier_time = 0.0
        self.last_startup_status_log = 0.0
        self.startup_status_log_interval = 2.0
        self.startup_diag_interval = 2.0
        self.last_startup_diag_log = 0.0
        self.goal_handle = None
        self.goal_in_progress = False                                                           
        self.no_frontier_cycles = 0
        self.corner_recovery_until = 0.0
        self.corner_recovery_mode = None
        self.last_goal_time = 0.0                                  
        self.last_goal_dispatch_time = 0.0
        self.goal_dispatch_pose = None
        self.last_goal_watchdog_log_time = 0.0
        self.post_abort_cooldown_until = 0.0
        self.last_abort_cooldown_log_time = 0.0
        self._lethal_fail_pos = None                                             
        self._lethal_fail_count = 0                                                       
        # static environment logic removed
        self.last_costmap_clear_time = 0.0
        self.goal_cooldown = 0.5                                           
        self.ever_had_frontiers = False
        self.ever_sent_goal = False
        # Warning tracking for goal send predicate checks
        self.last_scan_stale_warn_time = 0.0
        self.last_front_clearance_warn_time = 0.0
        self.last_lethal_cell_warn_time = 0.0
        self.last_done_log_time = 0.0
        self.done_log_interval = 5.0
        self.simple_exploration_active = False                                    
        self.simple_exploration_start_time = 0.0
        self.simple_exploration_duration = 10.0                                    
        self.simple_exploration_turn_time = 2.0                      
        self.simple_exploration_move_time = 3.0                      
        self.recovery_turn_dir = 1.0
        self.map_save_requested = False
        self.complete_marker_sent = False
        self._exploration_complete_lock = __import__('threading').Lock()
        self._exploration_complete_handled = False
        self.mapped_area_series = []
        self.last_mapped_area_m2 = 0.0
        self.last_percent_known = 0.0
        self.last_stable_percent_known = 0.0                                                       
        self._frontier_stagnation_start = None                                       
        self._frontier_stagnation_baseline = None                                 
        self.robot_path_series = []                                     
        self.last_recorded_path_x = None                                        
        self.last_recorded_path_y = None                                        
        self.nav_goals_series = []                                                                        
        self.frontier_history_series = []                                                 
        self.completion_datetime = None                                          
        self.completion_elapsed_s = None                                    
        self._csv_flushed = False                                             
        
        self.goals_reached = 0                                                     
        self.last_frontier_check_time = 0.0                                       
        self.last_frontier_prune_log_time = 0.0
        self.last_invalid_frontier_log_time = 0.0
        self.frontier_stable_count = 0                                                    
        self.min_frontier_stable_time = 5.0                                                                
        self.exploration_start_time = 0.0                                      
        self.nav2_fully_active = False                                          
        self.last_known_frontier_count = 0                                             
        self.obstacle_blocking_start = 0.0                                  
        self.max_obstacle_block_time = 10.0                                                
        
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3                                          
        self.last_successful_goal_pos = (0.0, 0.0)
        self.stuck_recovery_in_progress = False
        self.recovery_rotation_duration = 0.0                                                                             
        self.recovery_start_time = 0.0
        self.last_goal_target = None
        self.blacklisted_goals = {}                         

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
        self.visited_goals = []                                           
        self.recent_goals = []                                                       
        self.strict_avoid_goals = []                                               
        self.attempted_frontiers = []                                               
        
        self.scan_in_progress = False
        self.scan_step = 0
        self.scan_segment_index = 0
        self.scan_segments = 3                                                   
        self.scan_steps_per_angle = 10                                         
        self.scan_total_time = 0.0
        self.last_explore_check = 0.0
        self.explore_check_interval = 0.05
        
        self.last_phase_log_time = 0.0
        self.phase_log_interval = 5.0                                   
        self.last_init_gate_log_time = 0.0
        self.init_gate_log_interval = 2.0
        self.startup_health_ok_since = 0.0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_valid = False
        self.last_pose = None
        self.last_pose_change_time = time.time()
        self.pose_change_threshold = 0.02          
        self.pose_stale_timeout = max(1.0, float(getattr(self, 'pose_stale_timeout', 3.0)))
        self.pose_stale = False
        self.last_pose_stale_log_time = 0.0
        self.pose_stale_log_interval = 5.0
        self.last_tf_recovery_log_time = 0.0
        self.last_tf_stamp = None
        self.tf_fresh_since = 0.0
        self.origin_warn_interval = 5.0
        self.last_origin_warn_time = 0.0
        self.last_frontier_skip_reason = None
        self.last_frontier_relax_log_time = 0.0
        self.frontier_relax_log_interval = 5.0
        self.last_tf_check_log_time = 0.0
        self.tf_check_log_interval = 5.0
        self.costmap = None
        self.local_costmap = None
        self.costmap_raw_received = False
        self.local_costmap_raw_received = False

        self.bt_state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.controller_state_client = self.create_client(GetState, '/controller_server/get_state')
        self.planner_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.last_odom_time = 0.0
        self.odom_stale_timeout = 1.0           
        self.last_odom_stale_log_time = 0.0
        self.odom_stale_log_interval = 5.0
        self.last_frontier_update_time = 0.0
        self.last_replan_time = 0.0
        self.frontier_signature = None
        self.frontiers_dirty = True
        self.last_frontier_goal_eval_time = 0.0
        self.last_frontier_history_log_time = 0.0
        self.pending_replan_goal = None
        self.last_goal_cancel_time = 0.0
        self.replan_cancel_pending = False
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.mapped_area_pub = self.create_publisher(Float32, '/mapped_area_m2', 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_marker_pub = self.create_publisher(Marker, '/exploration_status', status_qos)
        self.goal_marker_pub = self.create_publisher(Marker, '/current_goal_arrow', status_qos)

        self.map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        self.create_subscription(MarkerArray, '/frontiers', self.frontiers_cb, 10)
        self.create_subscription(Float32, '/safety_stop', self.obstacle_distance_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
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
        
        self.rear_obstacle_detected = False
        self.last_rear_obstacle_time = 0.0
        self.last_rear_distance = float('inf')
        self.lidar_backup_until = 0.0
        
        self.create_timer(0.10, self.main_loop)                                                     
        
        self.get_logger().info("🚀 Exploration Coordinator Started (Simplified)")

    def destroy_node(self):
        """Save outputs on normal exit, but keep signal interrupt as shutdown-only."""
        if getattr(self, 'shutdown_by_signal', False):
            self.get_logger().info('🛑 Signal shutdown requested: skipping map/CSV save on exit')
            return super().destroy_node()
        try:
            self._flush_csv_data()
        except Exception:
            pass
        try:
            if not getattr(self, 'map_save_requested', False):
                self._save_map_sync()
        except Exception as e:
            try:
                self.get_logger().error(f'❌ Map save on shutdown failed: {e}')
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.shutdown_by_signal = True
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
