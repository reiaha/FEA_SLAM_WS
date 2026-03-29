#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration

class ExplorationExecutionMixin:
    def _maybe_clear_ghost_obstacles(self):
        """Clear costmaps if obstacles disappear quickly (ghost obstacles)."""
        # If an obstacle was detected but is now gone within a short time, clear costmaps
        now = time.time()
        if hasattr(self, 'last_obstacle_time') and hasattr(self, 'obstacle_detected'):
            if self.obstacle_detected:
                self._last_ghost_obstacle_time = now
            elif hasattr(self, '_last_ghost_obstacle_time'):
                # If obstacle disappeared within 2s, treat as ghost and clear
                if (now - self._last_ghost_obstacle_time) < 2.0:
                    self.get_logger().warn("👻 Ghost obstacle detected: clearing costmaps!")
                    self._clear_costmaps("ghost_obstacle")
                self._last_ghost_obstacle_time = 0

    def _clear_turn_dir(self) -> float:
        """Choose turn direction using side clearances (left:+, right:-)."""
        left = self.last_lidar_left_distance
        right = self.last_lidar_right_distance
        if left is None and right is None:
            return self.recovery_turn_dir if self.recovery_turn_dir != 0.0 else 1.0
        if left is None:
            return -1.0
        if right is None:
            return 1.0
        if abs(left - right) <= 0.05:
            return self.recovery_turn_dir if self.recovery_turn_dir != 0.0 else 1.0
        return 1.0 if left > right else -1.0

    def _publish_front_escape(self, turn_scale: float = 1.0):
        """Back up and turn toward clearer side when front obstacle is detected."""
        msg = Twist()
        self.recovery_turn_dir = self._clear_turn_dir()
        msg.angular.z = self.recovery_turn_dir * self.corner_recovery_turn_speed * max(0.2, turn_scale)
        if not (self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance):
            msg.linear.x = self.backup_speed
        self.cmd_vel_pub.publish(msg)

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
        self.last_goal_time = time.time() + 4.0
        if self.clear_global_costmap_client.service_is_ready():
            try:
                self.clear_global_costmap_client.call_async(ClearEntireCostmap.Request())
                self.get_logger().warn("🧹 Clearing global costmap for lethal-space escape")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Global costmap clear in lethal escape failed: {e}")

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
        
        if self.goal_handle is not None:
            self.get_logger().error("🛑 Cancelling Nav2 goal")
            try:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Nav2 goal cancel requested"))
            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not cancel goal: {e}")
            self.goal_handle = None
            self.goal_in_progress = False
        
        stop_msg = Twist()
        self.cmd_vel_nav_pub.publish(stop_msg)
    
    def start_backward_scan(self):
        """Initialize backward scan - NON-BLOCKING"""
        self.get_logger().info("🔍 Starting BACKWARD SCAN...")
        self.scan_in_progress = True
        self.scan_step = 0
        self.scan_segment_index = 0
        self.obstacle_detected = False                                                
        self.scan_total_time = self.scan_segments * self.scan_steps_per_angle * 0.05
        
    def execute_scan_step(self):
        """Execute one step of backward scan - called from main loop (NON-BLOCKING)"""
        if not self.scan_in_progress:
            return False                   
        
        if self.rear_obstacle_detected and self.last_rear_distance <= self.rear_emergency_distance:
            self.scan_in_progress = False
            self.get_logger().error(
                "🛑 Rear obstacle too close; stopping backward scan and holding position."
            )
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            return False
        elif self.rear_obstacle_detected:
            rotate_msg = Twist()
            rotate_msg.angular.z = 0.4
            self.cmd_vel_pub.publish(rotate_msg)
            self.scan_step += 1
            if self.scan_step >= self.scan_steps_per_angle:
                self.scan_step = 0
                self.scan_segment_index += 1
            return True
        
        if self.scan_segment_index >= self.scan_segments:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.scan_in_progress = False
            self.get_logger().info("✅ Backward scan complete")
            return False                 
        
        if self.front_obstacle_detected and self.obstacle_distance_m <= self.front_emergency_rotate_distance:
            self._publish_front_escape(turn_scale=0.9)
        else:
            backup_msg = Twist()
            backup_msg.linear.x = -0.12                                   
            self.cmd_vel_pub.publish(backup_msg)
        
        self.scan_step += 1
        
        if self.scan_step >= self.scan_steps_per_angle:
            self.scan_step = 0
            self.scan_segment_index += 1
        
        return True                          

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
        if self.lock_initial_pose_after_set:
            self.initial_pose_locked = True
        self.get_logger().warn(
            f"⚠️ Auto-published initial pose after {init_time_elapsed:.1f}s at x={x:.2f}, y={y:.2f}, yaw={math.degrees(theta):.1f}°"
        )
    
    def main_loop(self):
        """Main exploration state machine"""
        # Resolve phase enum from current state to avoid cross-module enum import coupling.
        Phase = self.current_phase.__class__
        if self.current_phase == Phase.DONE:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            return

        if (self.current_phase not in (Phase.INIT,) and
                self.goals_reached >= self.min_goals_for_complete and
                self.frontier_stagnation_timeout > 0 and
                hasattr(self, 'current_frontiers') and len(self.current_frontiers) > 0):
            _front_count = len(self.current_frontiers)
            now_t = time.time()
            if self._frontier_stagnation_baseline is None:
                self._frontier_stagnation_baseline = _front_count
                self._frontier_stagnation_start = now_t
            else:
                _drop_pct = (self._frontier_stagnation_baseline - _front_count) / max(1, self._frontier_stagnation_baseline) * 100.0
                if _drop_pct >= self.frontier_stagnation_drop_pct:
                    self._frontier_stagnation_baseline = _front_count
                    self._frontier_stagnation_start = now_t
                elif (now_t - self._frontier_stagnation_start) >= self.frontier_stagnation_timeout:
                    self.get_logger().warn(
                        f"🎉 EXPLORATION COMPLETE (frontier stagnation): {_front_count} frontiers unchanged "
                        f"for {self.frontier_stagnation_timeout:.0f}s. Map coverage: {self.last_percent_known:.1f}%"
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
        elif self.goals_reached < self.min_goals_for_complete:
            self._frontier_stagnation_baseline = None
            self._frontier_stagnation_start = None

        if (self.current_phase not in (Phase.INIT,) and
                self.goals_reached >= self.min_goals_for_complete and
                self.last_percent_known >= self.zero_frontier_complete_percent):
            self.get_logger().info(
                f"🎉 EXPLORATION COMPLETE! Coverage {self.last_percent_known:.1f}% "
                f">= {self.zero_frontier_complete_percent:.1f}% threshold "
                f"(goals reached: {self.goals_reached}/{self.min_goals_for_complete})."
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
            if time.time() < self.ultrasonic_emergency_until:
                if self.rear_obstacle_detected or (
                    self.front_obstacle_detected and
                    self.obstacle_distance_m <= self.front_emergency_rotate_distance
                ):
                    self._publish_front_escape(turn_scale=1.0)
                    return
                backup_msg = Twist()
                backup_msg.linear.x = self.backup_speed
                self.cmd_vel_pub.publish(backup_msg)
                return
            if time.time() < self.lidar_backup_until:
                if not self.front_obstacle_detected:
                    self.lidar_backup_until = 0.0
                    return
                if self.rear_obstacle_detected or self.obstacle_distance_m <= self.front_emergency_rotate_distance:
                    self._publish_front_escape(turn_scale=1.0)
                    return
                backup_msg = Twist()
                backup_msg.linear.x = self.backup_speed
                self.cmd_vel_pub.publish(backup_msg)
                return
        if ((self.strict_obstacle_handling or not self.nav2_handles_obstacles) and self.obstacle_detected and
                self.current_phase not in (Phase.OBSTACLE, Phase.RESCAN)):
            self.get_logger().error(
                f"💥 SAFETY_STOP detected ({self.obstacle_distance_m:.3f}m) - forcing OBSTACLE phase"
            )
            if self.current_phase == Phase.RECOVERY:
                self.consecutive_failures = 0
                self._lethal_fail_count = 0
                self._lethal_fail_pos = None
            self.current_phase = Phase.OBSTACLE
            self.phase_start_time = time.time()
            return

        now = time.time()
        if now - self.last_phase_log_time >= self.phase_log_interval:
            self.last_phase_log_time = now
            phase_name = self.current_phase.name
            frontier_count = len(self.current_frontiers)
            percent_complete = self.last_percent_known if hasattr(self, 'last_percent_known') else 0.0
            self.get_logger().info(
                f"📊 Phase: {phase_name} | Frontiers: {frontier_count} | Failures: {self.consecutive_failures}/{self.max_consecutive_failures} | Map completion: {percent_complete:.2f}%"
            )
            if not self._odom_recent():
                if now - self.last_odom_stale_log_time > self.odom_stale_log_interval:
                    self.last_odom_stale_log_time = now
                    self.get_logger().warn("⚠️ No /odom or odom TF updates in the last 1s. Check Arduino bridge and TF.")
        
        if self.current_phase == Phase.INIT:
            self.update_pose()

            init_time_elapsed = time.time() - self.startup_time
            self._maybe_auto_publish_initial_pose(init_time_elapsed)
            frontiers_ready = len(self.current_frontiers) > 0
            scan_ready = (time.time() - self.last_scan_time) <= self.lidar_stale_timeout
            lidar_ready = frontiers_ready or scan_ready or (
                (time.time() - self.last_frontier_time) <= self.frontier_lidar_fallback_timeout
            )
            self.pose_valid and not self.pose_stale

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

            if obstacle_blocking:
                if self.obstacle_blocking_start == 0.0:
                    self.obstacle_blocking_start = time.time()
                    self.get_logger().warn(f"⚠️ Obstacle detected at startup - waiting up to {self.max_obstacle_block_time}s for clearance")
                elif (time.time() - self.obstacle_blocking_start) >= self.max_obstacle_block_time:
                    self.get_logger().warn(f"⚠️ Obstacle timeout reached ({self.max_obstacle_block_time}s) - proceeding anyway")
                    obstacle_blocking = False
            else:
                self.obstacle_blocking_start = 0.0

            nav2_acceptable = nav2_lifecycle_active if self.require_nav2_active else nav2_ready
            odom_ready = self._odom_recent()

            tf_age_sec = float('inf')
            if self.last_tf_stamp is not None:
                try:
                    tf_age_sec = (self.get_clock().now() - rclpy.time.Time.from_msg(self.last_tf_stamp)).nanoseconds / 1e9
                except Exception:
                    tf_age_sec = float('inf')
            pose_fresh_enough = self.pose_valid and (tf_age_sec <= self.startup_max_tf_age_sec)
            pose_health_ok = pose_fresh_enough if self.startup_require_fresh_pose else self.pose_valid
            scan_health_ok = scan_ready if self.startup_require_scan_recent else True
            odom_health_ok = odom_ready if self.startup_require_odom_recent else True

            instant_health_ok = pose_health_ok and scan_health_ok and odom_health_ok
            now_t = time.time()
            if instant_health_ok:
                if self.startup_health_ok_since <= 0.0:
                    self.startup_health_ok_since = now_t
            else:
                self.startup_health_ok_since = 0.0

            health_window_ok = (
                self.startup_health_window_sec <= 0.0 or
                (
                    self.startup_health_ok_since > 0.0 and
                    (now_t - self.startup_health_ok_since) >= self.startup_health_window_sec
                )
            )

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

            init_pose_ok = self.pose_valid
            gate_ok = (
                init_time_elapsed >= self.nav2_activation_time and
                lidar_gate and init_pose_ok and
                costmap_gate and tf_ready and nav2_acceptable and odom_ready and clear_for_hold and
                not obstacle_blocking and health_window_ok
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
                    f"tf_age={tf_age_sec:.2f}s, pose_health={pose_health_ok}, scan_health={scan_health_ok}, odom_health={odom_health_ok}, "
                    f"health_window={health_window_ok}, "
                    f"clear={clear_for_hold}, obstacle={self.obstacle_detected}, blocking={obstacle_blocking}"
                )
            if gate_ok:
                if not self.initial_pose_set:
                    if self.initial_pose_wait_timeout > 0.0 and init_time_elapsed >= self.initial_pose_wait_timeout and self.pose_valid:
                        self.initial_pose_set = True
                        self.get_logger().warn(
                            f"⚠️ Initial pose wait timed out after {self.initial_pose_wait_timeout:.1f}s; proceeding with current TF pose."
                        )
                    else:
                        self.get_logger().warn("Waiting for initial pose to be set via RViz 2D Pose Estimate before starting exploration.")
                        return
                self.exploration_start_time = time.time()                                         
                self.update_pose()
                self.home_pose = self.robot_pose                                  
                self.get_logger().info(
                    f"🏠 Home pose recorded: ({self.home_pose[0]:.2f}, {self.home_pose[1]:.2f})")
                self.get_logger().info("✅ Startup complete! Starting exploration")
                self.current_phase = Phase.EXPLORE
                self.phase_start_time = time.time()
        
        elif self.current_phase == Phase.EXPLORE:
            if (time.time() - self.last_scan_time) > self.lidar_stale_timeout:
                if (time.time() - self.last_scan_stale_log_time) >= self.scan_stale_log_interval:
                    self.last_scan_stale_log_time = time.time()
                    self.get_logger().warn("⚠️ Scan stale during EXPLORE - pausing until fresh scan arrives")
                return

            if (self.strict_obstacle_handling or not self.nav2_handles_obstacles) and self.obstacle_detected:
                self.get_logger().error(f"💥💥💥 OBSTACLE at {self.obstacle_distance_m:.3f}m - OBSTACLE PHASE!")
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = time.time()
                return

            now = time.time()

            # Dynamic mode: static_stuck_trigger_time logic removed
            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)
            self.update_pose()
            if self.pose_valid:
                stuck_key = (round(self.robot_pose[0], 2), round(self.robot_pose[1], 2))
                self._blacklist_goal(stuck_key)
                now = time.time()
                if not hasattr(self, '_last_blacklist_stuck_warn') or (now - getattr(self, '_last_blacklist_stuck_warn', 0)) > 10.0:
                    self.get_logger().warn(
                        f"⚠️ Blacklisting stuck position ({stuck_key[0]:.2f}, {stuck_key[1]:.2f}) "
                        f"for {self.blacklist_duration:.0f}s"
                    )
                    self._last_blacklist_stuck_warn = now
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
                    if step_elapsed < self.static_stuck_escape_backup_dur:
                        escape_msg.linear.x = self.backup_speed                       
                        self.cmd_vel_pub.publish(escape_msg)
                        return
                    else:
                        self.static_stuck_escape_step = 1
                        self.static_stuck_escape_step_start = now
                        step_elapsed = 0.0
                if self.static_stuck_escape_step == 1:
                    if step_elapsed < self.static_stuck_escape_turn_dur:
                        escape_msg.angular.z = (
                            self.static_stuck_escape_turn_dir * self.corner_recovery_turn_speed
                        )
                        self.cmd_vel_pub.publish(escape_msg)
                        return
                    else:
                        self.static_stuck_escape_active = False
                        self.static_stuck_start = 0.0
                        self.no_frontier_cycles = 0
                        self.last_goal_time = 0.0                              
                        self.get_logger().warn("✅ Deep escape complete — resuming exploration")
                        return

            if now - self.last_explore_check < self.explore_check_interval:
                return                   
            self.last_explore_check = now

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

            # Ghost obstacle suppression
            self._maybe_clear_ghost_obstacles()

            if self.pending_replan_goal and not (self.goal_handle or self.goal_in_progress):
                if (now - self.last_goal_cancel_time) >= self.replan_cancel_cooldown:
                    goal_x, goal_y = self.pending_replan_goal
                    self.pending_replan_goal = None
                    is_blacklisted = any(
                        math.hypot(goal_x - bx, goal_y - by) <= self.blacklist_radius
                        for (bx, by) in self.blacklisted_goals
                    )
                    if not is_blacklisted:
                        if self.send_goal_to_nav2(goal_x, goal_y):
                            self.last_goal_time = now
                        else:
                            self.last_goal_time = 0.0
                            self.frontiers_dirty = True
                    else:
                        self.get_logger().warn(
                            f"🚫 Pending replan goal ({goal_x:.2f}, {goal_y:.2f}) is now blacklisted — skipping"
                        )
                return
            
            if self._maintain_active_goal(now):
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
            
            time_since_last_goal = now - self.last_goal_time
            if time_since_last_goal < self.goal_cooldown:
                return
            
            need_frontier_eval = self.frontiers_dirty or ((now - self.last_frontier_goal_eval_time) >= self.frontier_goal_refresh_min_interval)
            if not need_frontier_eval:
                return
            self.last_frontier_goal_eval_time = now
            goal_info = self.pick_best_frontier()
            self.frontiers_dirty = False
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
                                bootstrap_msg.linear.x = max(0.08, min(0.18, abs(self.backup_speed)))
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

                _imm_frontiers = len(self.current_frontiers)
                if (_imm_frontiers == 0 and
                        self.goals_reached >= self.min_goals_for_complete and
                        self.last_percent_known >= self.zero_frontier_complete_percent):
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

                    if (self.goals_reached >= self.min_goals_for_complete and
                            self.last_percent_known >= self.zero_frontier_complete_percent):
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
            sent = self.send_goal_to_nav2(
                goal_x, goal_y, goal_info.get('costmap_filtered'), frontier_xy=(frontier_x, frontier_y)
            )
            if sent:
                self.last_goal_time = now                                        
            else:
                self.last_goal_time = 0.0
                self.frontiers_dirty = True
        
        elif self.current_phase == Phase.OBSTACLE:
            self._refresh_pending_goal_from_frontiers("obstacle")
            self.get_logger().error("📍 OBSTACLE PHASE: Cancel goal and prepare for RESCAN")
            self._clear_costmaps("obstacle")

            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)

            self.emergency_backup()
            self.rescan_done = False                                   
            self.current_phase = Phase.RESCAN
            self.phase_start_time = time.time()
        
        elif self.current_phase == Phase.RESCAN:
            self._refresh_pending_goal_from_frontiers("rescan")
            elapsed = time.time() - self.phase_start_time
            
            if not self.rescan_done:
                self.get_logger().info("📍 RESCAN: Executing backward scan now...")
                self.start_backward_scan()
                self.rescan_done = True
            
            scan_active = self.execute_scan_step()

            if not scan_active:
                if elapsed < (self.scan_total_time + 0.5):                             
                    return
                
                clear_for_hold = (time.time() - self.last_obstacle_time) >= self.obstacle_hold_time
                if not self.obstacle_detected and clear_for_hold:
                    self.get_logger().info("✅ Path clear after rescan! Looking for new frontier...")
                    
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
                    self.get_logger().error("⚠️ RESCAN: Path STILL blocked after 8s - retrying backup")
                    self.current_phase = Phase.OBSTACLE
                    self.rescan_done = False
        
        elif self.current_phase == Phase.RECOVERY:
            self._clear_costmaps("recovery")
            elapsed = time.time() - self.recovery_start_time
            
            if elapsed < self.recovery_rotation_duration:
                rotate_msg = Twist()
                rotate_msg.angular.z = 0.75                        
                self.cmd_vel_pub.publish(rotate_msg)
                if int(elapsed * 2) % 2 == 0:                         
                    self.get_logger().info(f"🔄 RECOVERY: Rotating in place ({elapsed:.1f}s/{self.recovery_rotation_duration}s)")
            else:
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                self.get_logger().info("✅ RECOVERY complete - resetting failure counter and picking new frontier")
                self.last_costmap_clear_time = 0.0                   
                self._clear_costmaps('post_recovery', clear_global=True)
                self._lethal_fail_count = 0
                self._lethal_fail_pos = None
                self.consecutive_failures = 0                        
                self.stuck_recovery_in_progress = False
                self.current_phase = Phase.EXPLORE
                self.phase_start_time = time.time()
        
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

