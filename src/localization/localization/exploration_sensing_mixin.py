#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

class ExplorationSensingMixin:
    def _save_map_sync(self):
        from .map_saver import save_occupancy_grid_map

        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            return
        map_save_dir = getattr(self, 'map_save_dir', '/home/pi/FEA_SLAM_WS/saved_maps')
        result = save_occupancy_grid_map(slam_map, map_save_dir, name_prefix='map')
        try:
            self.get_logger().info(
                f"💾 Shutdown map saved: {result['pgm_path']} + {result['png_path']} "
                f"({result['width']}x{result['height']})"
            )
        except Exception:
            pass

    def frontiers_cb(self, msg: MarkerArray):
        """Store frontier positions"""
        self.current_frontiers = []
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.current_frontiers.append((x, y))

        now = time.time()
        new_signature = self._compute_frontier_signature(self.current_frontiers)
        if new_signature != self.frontier_signature:
            self.frontier_signature = new_signature
            self.frontiers_dirty = True
            self.last_frontier_update_time = now

        if self.current_frontiers and (now - self.last_frontier_history_log_time) >= self.frontier_history_sample_interval:
            _elapsed = now - self.startup_time
            for _fx, _fy in self.current_frontiers:
                self.frontier_history_series.append((_elapsed, _fx, _fy))
            self.last_frontier_history_log_time = now

        if self.current_frontiers:
            self.ever_had_frontiers = True
            self.last_frontier_time = now
            self.last_scan_time = now
    
    def _compute_frontier_signature(self, frontiers):
        q = max(0.01, float(self.frontier_change_pos_quant))
        return frozenset((int(round(x / q)), int(round(y / q))) for x, y in frontiers)

    def obstacle_distance_cb(self, msg: Float32):
        """Handle Arduino safety stop signals"""
        if self.nav2_handles_obstacles and not self.strict_obstacle_handling:
            return
        if not self.use_ultrasonic_backup:
            return

        distance = msg.data
        now = time.time()

        is_clear_signal = distance >= 999.0                                                               

        if not is_clear_signal and distance < self.ultrasonic_min_valid_distance:
            if (now - self.last_ultrasonic_filter_log_time) >= self.ultrasonic_filter_log_interval:
                self.last_ultrasonic_filter_log_time = now
                self.get_logger().info(
                    f"ℹ️ Ignoring ultrasonic spike: {distance:.3f}m < min_valid {self.ultrasonic_min_valid_distance:.3f}m"
                )
            return

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

        if not is_clear_signal:
            self.last_ultrasonic_distance = distance
            self.last_ultrasonic_time = now

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
                if self.current_phase == Phase.RECOVERY:
                    self.consecutive_failures = 0
                    self._lethal_fail_count = 0
                    self._lethal_fail_pos = None
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = now
            return

        if (now - self.last_scan_time) <= self.lidar_stale_timeout:
            return
        
        if distance < 999.0 and distance > self.ultrasonic_backup_distance:
            self.ultrasonic_obstacle_hits = 0
            return
        
        if distance >= 999.0:
            self.ultrasonic_clear_hits += 1
            self.ultrasonic_obstacle_hits = 0
            if self.ultrasonic_clear_hits < self.ultrasonic_clear_confirm_count:
                return
            if self.obstacle_detected and (now - self.last_obstacle_time) < self.obstacle_hold_time:
                return
            if self.current_phase == Phase.OBSTACLE:
                return
            self.get_logger().info(f"✅ Obstacle cleared (SAFETY_STOP:0)")
            self.obstacle_detected = False
            return
        
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
        
        len(msg.ranges)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        rear_obstacle = False
        front_obstacle = False
        min_front_distance = float('inf')
        min_rear_distance = float('inf')
        min_left_distance = float('inf')
        min_right_distance = float('inf')
        front_hit_count = 0
        rear_hit_count = 0
        for i, distance in enumerate(msg.ranges):
            if distance <= max(msg.range_min, self.scan_min_range) or distance > msg.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            
            in_rear_zone = (abs(abs(angle) - math.pi) <= self.rear_obstacle_half_angle)
            in_front_zone = (-self.front_obstacle_half_angle <= angle <= self.front_obstacle_half_angle)
            in_left_zone = (math.radians(20.0) <= angle <= math.radians(160.0))
            in_right_zone = (-math.radians(160.0) <= angle <= -math.radians(20.0))
            
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

            if in_left_zone and distance < min_left_distance:
                min_left_distance = distance
            if in_right_zone and distance < min_right_distance:
                min_right_distance = distance

        if self.swap_lidar_front_back:
            min_front_distance, min_rear_distance = min_rear_distance, min_front_distance
            front_hit_count, rear_hit_count = rear_hit_count, front_hit_count
            min_left_distance, min_right_distance = min_right_distance, min_left_distance

        front_obstacle = front_hit_count >= max(1, self.front_obstacle_min_hits)
        rear_obstacle = rear_hit_count >= max(1, self.rear_obstacle_min_hits)
        
        if min_rear_distance < float('inf'):
            self.last_rear_distance = min_rear_distance
        if min_left_distance < float('inf'):
            self.last_lidar_left_distance = min_left_distance
        if min_right_distance < float('inf'):
            self.last_lidar_right_distance = min_right_distance

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
            if obstacle_type == "static":
                if self.static_stuck_start == 0.0:
                    self.static_stuck_start = now
            else:
                self.static_stuck_start = 0.0
                self.static_stuck_escape_active = False
        elif self.use_lidar_obstacle:
            if min_front_distance < float('inf'):
                self.last_lidar_front_distance = min_front_distance
                self.last_lidar_front_time = time.time()
            if self.obstacle_detected and (time.time() - self.last_obstacle_time) >= self.obstacle_hold_time:
                self.obstacle_detected = False
            if self.front_obstacle_detected and (time.time() - self.last_front_obstacle_time) >= self.obstacle_hold_time:
                self.front_obstacle_detected = False
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
        known_cells = 0
        total_cells = len(data)
        for val in data:
            if 0 <= val <= 100:
                known_cells += 1
        if total_cells > self.max_total_cells:
            self.max_total_cells = total_cells
        if known_cells > self.max_known_cells:
            self.max_known_cells = known_cells
        resolution = msg.info.resolution
        area_m2 = self.max_known_cells * (resolution * resolution)
        now = time.time()
        elapsed = now - self.startup_time
        current_percent = (known_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
        percent_known = (self.max_known_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
        stable_percent_known = max(self.last_stable_percent_known, current_percent)
        self.last_mapped_area_m2 = area_m2
        self.last_percent_known = stable_percent_known
        self.last_stable_percent_known = stable_percent_known
        self.last_slam_map = msg                        
        self.mapped_area_series.append((elapsed, area_m2, percent_known, known_cells, total_cells, stable_percent_known))
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
        timeout = Duration(seconds=0.05)                
        
        if (now - self.last_odom_time) <= self.odom_stale_timeout:
            return True

        try:
            if not self.tf_buffer.can_transform('odom', self.base_frame, rclpy.time.Time(), timeout=timeout):
                return False
            tf = self.tf_buffer.lookup_transform('odom', self.base_frame, rclpy.time.Time(), timeout=timeout)
            tf_age = (self.get_clock().now() - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if tf_age <= self.odom_stale_timeout:
                return True
        except Exception:
            pass
        
        try:
            if self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time(), timeout=timeout):
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

        if value < 0:
            return False
        if value >= self.costmap_free_threshold:
            return False

        free_neighbors = 0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = mx + dx
            ny = my + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                return False
            nindex = ny * width + nx
            nvalue = data[nindex]
            if 0 <= nvalue < self.costmap_free_threshold:
                free_neighbors += 1

        if free_neighbors < 3:
            return False

        return True
    
    def update_pose(self):
        """Get robot pose from TF with proper timeout handling"""
        try:
            now = self.get_clock().now()
            timeout = Duration(seconds=0.1)                 
            
            if not self.tf_buffer.can_transform('map', self.base_frame, rclpy.time.Time(), timeout=timeout):
                self.pose_valid = False
                return
                
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time(), timeout=timeout)
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            self.robot_pose
            self.robot_pose = (x, y, 0.0)
            self.pose_valid = True
            self._update_pose_stale_state(x, y, tf.header.stamp)
            self._record_path_point(x, y)
            
            if abs(x) < 0.01 and abs(y) < 0.01:
                now = time.time()
                odom_recent = (now - self.last_odom_time) <= self.odom_stale_timeout
                origin_stale = (now - self.last_pose_change_time) >= 5.0
                if odom_recent and origin_stale and (now - self.last_origin_warn_time) >= self.origin_warn_interval:
                    self.last_origin_warn_time = now
                    self.get_logger().warn("⚠️ Robot still at origin (0,0) - odom/TF may not be updating")
        except Exception:
            try:
                fallback_frame = 'base_link' if self.base_frame != 'base_link' else 'base_footprint'
                
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
            except Exception:
                self.pose_valid = False

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
            return None
        if not self._is_costmap_free(goal[0], goal[1], costmap_info):
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
                return path_length

            _, _, _, width, height, _ = costmap_info
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if not self._is_costmap_free(nx, ny, costmap_info):
                    continue
                ng = g + 1.0
                if (nx, ny) not in g_scores or ng < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = ng
                    h = abs(gx - nx) + abs(gy - ny)
                    f = ng + h
                    heapq.heappush(open_set, (f, ng, nx, ny))

        return None
    
