#!/usr/bin/env python3

import math
import time

import rclpy
import rclpy.time
import serial
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float32

class ArduinoMotorBridgeRuntimeMixin:
    def _classify_motion(self, pwm_left, pwm_right) -> str:
        db = self.min_pwm
        fwd  = pwm_left  > db and pwm_right > db
        back = pwm_left  < -db and pwm_right < -db
        turn = (
            (pwm_left > db and pwm_right < -db) or
            (pwm_left < -db and pwm_right > db) or
            (pwm_left > db and abs(pwm_right) <= db) or
            (abs(pwm_left) <= db and pwm_right > db) or
            (pwm_left < -db and abs(pwm_right) <= db) or
            (abs(pwm_left) <= db and pwm_right < -db)
        )
        if fwd:   return 'FORWARD'
        if back:  return 'BACKWARD'
        if turn:  return 'TURN'
        return 'STOPPED'

    def _track_motion_oscillation(self, pwm_left, pwm_right):
        """Detect motor stuck in FORWARD/TURN ↔ STOPPED oscillation and fire a deep escape."""
        now  = time.time()

        if now < self._osc_escape_until:
            return

        state = self._classify_motion(pwm_left, pwm_right)

        cutoff = now - self._osc_window_sec
        while self._motion_state_history and self._motion_state_history[0][0] < cutoff:
            self._motion_state_history.popleft()

        self._motion_state_history.append((now, state))

        transitions = 0
        prev = None
        for _, s in self._motion_state_history:
            if prev is not None:
                was_moving = prev  != 'STOPPED'
                is_moving  = s     != 'STOPPED'
                if was_moving != is_moving:
                    transitions += 1
            prev = s

        backward_count = sum(1 for _, s in self._motion_state_history if s == 'BACKWARD')
        history_len = len(self._motion_state_history)
        if history_len == 0:
            return
        backward_ratio = backward_count / history_len

        if transitions >= self._osc_trip_count and backward_ratio < 0.2:
            self.get_logger().warn(
                f"🔁 Motion oscillation detected: {transitions} STOP↔MOVE flips "
                f"in {self._osc_window_sec:.0f}s — firing deep escape"
            )
            self._oscillation_escape()

    def _oscillation_escape(self):
        """Deep escape when robot is oscillating FORWARD/TURN ↔ STOPPED.
        Backs up longer and turns further than the normal front-obstacle escape."""
        back_pwm = max(0, min(255, abs(self.fixed_pwm_backward)))
        self.cmd_vel_override_until = time.time() + self._osc_escape_backup_dur + self._osc_escape_turn_dur + 0.5
        self._osc_escape_active     = True
        self._osc_escape_step       = 0
        self._osc_escape_step_start = time.time()
        self._osc_escape_turn_dir   = self.escape_turn_dir                                      
        self.escape_turn_dir       *= -1.0                           
        self._send_motor_pwm(-back_pwm, -back_pwm, source='osc_escape_backup')
        self._osc_escape_until = time.time() + self._osc_window_sec            
        self._motion_state_history.clear()
        self.get_logger().warn(
            f"🆘 Oscillation escape: backing up {self._osc_escape_backup_dur:.1f}s "
            f"then turning {self._osc_escape_turn_dur:.1f}s"
        )

    def cmd_vel_nav_cb(self, msg: Twist):
        """Handle cmd_vel_nav only when no override is active"""
        if time.time() < self.cmd_vel_override_until:
            return                                              
        self._handle_cmd_vel(msg, source='cmd_vel_nav')

    def scan_cb(self, msg: LaserScan):
        now = time.time()
        self.last_scan_time = now
        min_rear = float('inf')
        min_front = float('inf')
        min_any = float('inf')
        min_any_front_half = float('inf')                                              
        front_hit_count = 0
        rear_hit_count = 0
        any_front_half_hit_count = 0
        effective_min_range = max(msg.range_min, self.scan_min_range)
        angle = msg.angle_min
        angle_increment = msg.angle_increment
        range_max = msg.range_max
        swap_front_back = self.swap_lidar_front_back
        any_stop_distance = self.any_obstacle_stop_distance
        front_stop_distance = self.front_stop_distance
        rear_stop_distance = self.rear_stop_distance
        front_half_limit = math.pi / 2.0
        front_zone_half_angle = self.front_obstacle_half_angle
        rear_zone_half_angle = self.rear_obstacle_half_angle
        pi = math.pi

        for distance in msg.ranges:
            if distance <= effective_min_range or distance > msg.range_max:
                angle += angle_increment
                continue
            robot_angle = angle
            if swap_front_back:
                robot_angle += pi
                if robot_angle > pi:
                    robot_angle -= 2.0 * math.pi
            if distance < min_any:
                min_any = distance
            in_front_half = (-front_half_limit <= robot_angle <= front_half_limit)
            if in_front_half:
                if distance < min_any_front_half:
                    min_any_front_half = distance
                if distance <= any_stop_distance:
                    any_front_half_hit_count += 1
            in_front_zone = (-front_zone_half_angle <= robot_angle <= front_zone_half_angle)
            in_rear_zone = (abs(abs(robot_angle) - pi) <= rear_zone_half_angle)

            if in_front_zone:
                if distance < min_front:
                    min_front = distance
                if distance <= front_stop_distance:
                    front_hit_count += 1
            if in_rear_zone:
                if distance < min_rear:
                    min_rear = distance
                if distance <= rear_stop_distance:
                    rear_hit_count += 1
            angle += angle_increment

        newly_blocked = False
        was_front_blocked     = (now < self.front_blocked_until)
        was_rear_blocked      = (now < self.rear_blocked_until)
        was_any_front_blocked = (now < self.any_obstacle_blocked_until)

        if min_front < float('inf'):
            self.last_front_distance = min_front
            if min_front <= self.front_stop_distance and front_hit_count >= max(1, self.front_block_min_hits):
                self.front_blocked_until = now + self.front_stop_hold_time
                if not was_front_blocked:                                               
                    newly_blocked = True

        if min_rear < float('inf'):
            self.last_rear_distance = min_rear
            if min_rear <= self.rear_stop_distance and rear_hit_count >= max(1, self.rear_block_min_hits):
                self.rear_blocked_until = now + self.rear_stop_hold_time
                if not was_rear_blocked:                                                
                    newly_blocked = True
        else:
            self.last_rear_distance = range_max

        if min_any < float('inf'):
            self.last_any_obstacle_distance = min_any
        if min_any_front_half < float('inf'):
            if min_any_front_half <= any_stop_distance and any_front_half_hit_count >= max(1, self.any_obstacle_min_hits):
                self.any_obstacle_blocked_until = now + self.any_obstacle_stop_hold_time
                if not was_any_front_blocked:                                           
                    newly_blocked = True

        if newly_blocked:
            was_forward = (self.last_cmd_pwm_left > 0 and self.last_cmd_pwm_right > 0)
            front_just_blocked = (now < self.front_blocked_until) and (min_front <= self.front_stop_distance)
            only_rear_newly_blocked = (
                (now < self.rear_blocked_until)
                and not front_just_blocked
                and not self._any_obstacle_blocked()
            )
            turning_to_escape = (
                front_just_blocked and
                abs(self.last_cmd_pwm_left) >= max(1, self.min_pwm_turn) and
                (self.last_cmd_pwm_left * self.last_cmd_pwm_right < 0)                                  
            )
            if front_just_blocked and was_forward and not self.safety_override_active and not self.stuck_recovery_active:
                self._fire_obstacle_escape()
            elif only_rear_newly_blocked:
                pass                                                                                          
            elif turning_to_escape:
                pass                                                                       
            else:
                if front_just_blocked:
                    self._send_motor_pwm(0, 0, source='scan')

    def _fire_obstacle_escape(self):
        """Immediately drive backward when a front obstacle is detected while moving forward.
        Bypasses the Nav2 cmd_vel cycle so reaction is instant (bounded only by scan rate)."""
        back_pwm = max(0, min(255, abs(self.fixed_pwm_backward)))
        turn_pwm = max(0, min(255, abs(self.fixed_pwm_turn)))
        escape_dir = self._select_escape_turn(0.0)                          
        if escape_dir > 0.0:                                        
            pwm_left  = -turn_pwm
            pwm_right = -back_pwm
        else:                                                         
            pwm_left  = -back_pwm
            pwm_right = -turn_pwm
        self.cmd_vel_override_until = time.time() + self.front_stop_hold_time
        self._send_motor_pwm(pwm_left, pwm_right, source='scan_escape')
        self.get_logger().info(
            f"⚡ Immediate escape: front={self.last_front_distance:.2f}m — backing away now"
        )

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

        for name, client in self.nav2_clients.items():
            if not client.service_is_ready():
                continue
            pending_future = self.nav2_state_futures.get(name)
            if pending_future is not None:
                request_time = self.nav2_state_request_time.get(name, 0.0)
                if (now - request_time) > self.nav2_state_response_timeout:
                    self.nav2_state_futures[name] = None
                else:
                    continue
            if self.nav2_state_futures.get(name) is None:
                try:
                    future = client.call_async(GetState.Request())
                    self.nav2_state_futures[name] = future
                    self.nav2_state_request_time[name] = now
                    future.add_done_callback(lambda f, n=name: self._handle_nav2_state_result(n, f))
                except Exception as e:
                    self.get_logger().error(f"🧭 Error sending {name} state request: {e}")

        active_count = 0
        status_lines = []
        active_nodes = set()
        explicit_non_active_nodes = set()
        required_nodes = {'controller_server', 'planner_server'}

        for name in self.nav2_clients.keys():
            if not self.nav2_clients[name].service_is_ready():
                status_lines.append(f"{name}: service_not_ready")
                continue

            if name not in self.nav2_state_cache:
                if name == 'bt_navigator' and self.nav2_tolerate_bt_pending:
                    status_lines.append(f"{name}: pending(tolerated)")
                else:
                    status_lines.append(f"{name}: pending")
                continue

            last_update = self.nav2_state_update_time.get(name, 0.0)
            if now - last_update > self.nav2_state_response_timeout:
                if name == 'bt_navigator' and self.nav2_tolerate_bt_pending:
                    status_lines.append(f"{name}: timeout(tolerated)")
                else:
                    status_lines.append(f"{name}: timeout")
                continue

            state_id = self.nav2_state_cache.get(name)
            if state_id == 3:
                active_count += 1
                active_nodes.add(name)
            elif state_id in (0, None):
                status_lines.append(f"{name}: {state_map.get(state_id, str(state_id))}")
            else:
                explicit_non_active_nodes.add(name)
                status_lines.append(f"{name}: {state_map.get(state_id, str(state_id))}")

        required_active = required_nodes.issubset(active_nodes)
        explicit_non_active_required = any(n in required_nodes for n in explicit_non_active_nodes)
        explicit_non_active_bt = 'bt_navigator' in explicit_non_active_nodes
        explicit_non_active_for_ready = explicit_non_active_required or (
            explicit_non_active_bt and not self.nav2_tolerate_bt_pending
        )
        enough_active_nodes = active_count >= max(1, self.nav2_min_active_nodes)
        instant_ready = required_active and enough_active_nodes and not explicit_non_active_for_ready

        if self.nav2_allow_goal_override and self.has_active_goal and (
            (not explicit_non_active_for_ready) or active_count >= 1):
            instant_ready = True

        if instant_ready:
            self.nav2_not_ready_since = 0.0
            self.nav2_ready = True
            self.last_nav2_ready_true_at = now
        else:
            if self.nav2_not_ready_since <= 0.0:
                self.nav2_not_ready_since = now
            self.nav2_ready = (now - self.nav2_not_ready_since) < self.nav2_inactive_confirm_sec

        self.nav2_explicitly_inactive = explicit_non_active_for_ready

        if self.nav2_ready != self.last_nav2_ready:
            self.last_nav2_ready = self.nav2_ready
            state = 'ACTIVE' if self.nav2_ready else 'INACTIVE'
            self.get_logger().info(
                f"🧭 Nav2 state changed: {state} ({active_count}/3 nodes active, "
                f"required={sorted(required_nodes)})"
            )

        if not self.nav2_ready:
            if now - self.last_nav2_state_log_time >= self.nav2_state_log_interval:
                self.last_nav2_state_log_time = now
                if status_lines:
                    status = '; '.join(status_lines)
                    self.get_logger().warn(
                        f"🧭 Nav2 not ready: {status} "
                        f"(confirming for {self.nav2_inactive_confirm_sec:.1f}s)"
                    )
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
            self.nav2_state_request_time[name] = 0.0

    def _publish_odom(self):
        if not self.publish_odom:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:                                                   
            self.last_time = now
            return

        v_cmd = self.last_cmd_linear
        w_cmd = self.last_cmd_angular
        v = v_cmd
        w = w_cmd
        odom_linear_scale = float(getattr(self, 'odom_linear_scale', 1.0))
        odom_angular_scale = float(getattr(self, 'odom_angular_scale', 1.0))
        freeze_pose_update = False
        now_wall = time.time()
        imu_yaw_allowed = self.use_imu_yaw_in_odom and (now_wall >= self.imu_yaw_override_disabled_until)

        if abs(w_cmd) > self.angular_deadband:
            imu_angular_fresh = (
                self.imu_angular_vel_z is not None and
                self.imu_last_stamp is not None and
                (self.get_clock().now() - rclpy.time.Time.from_msg(self.imu_last_stamp)).nanoseconds / 1e9 < 0.25
            )
            if imu_angular_fresh and not self.imu_hardware_dead and imu_yaw_allowed:
                if self.imu_sign_check_enabled and abs(w_cmd) >= self.imu_sign_check_min_cmd_radps:
                    if (w_cmd * self.imu_angular_vel_z) < 0.0:
                        self.imu_sign_mismatch_streak += 1
                        if self.imu_sign_mismatch_streak >= max(1, self.imu_sign_mismatch_trip_count):
                            self.imu_yaw_override_disabled_until = now_wall + max(0.5, self.imu_sign_disable_sec)
                            self.imu_sign_mismatch_streak = 0
                            if (now_wall - self.last_imu_sign_mismatch_log_time) >= 1.0:
                                self.last_imu_sign_mismatch_log_time = now_wall
                                self.get_logger().warn(
                                    "⚠️ IMU sign mismatch detected during turn; "
                                    "temporarily disabling IMU yaw fusion and using PWM-ratio yaw fallback."
                                )
                    else:
                        self.imu_sign_mismatch_streak = 0
                w = self.imu_angular_vel_z
            elif abs(v_cmd) <= self.velocity_deadband:
                _pivot_wheel_vel = (self.fixed_pwm_turn / float(self.max_speed_forward)) * self.max_linear_speed_mps
                w = math.copysign(_pivot_wheel_vel / self.wheel_base, w_cmd)
            if abs(v_cmd) <= self.velocity_deadband:
                v = 0.0

        v *= odom_linear_scale
        w *= odom_angular_scale

        if self.odom_feedback_gate_enabled:
            feedback_recent = (time.time() - self.last_motor_speed_time) <= 0.5
            if feedback_recent and self.last_motor_speed_left is not None and self.last_motor_speed_right is not None:
                motors_stationary = (
                    abs(self.last_motor_speed_left) <= self.odom_stationary_speed_threshold and
                    abs(self.last_motor_speed_right) <= self.odom_stationary_speed_threshold
                )
                cmd_demands_linear_motion = abs(v_cmd) > self.velocity_deadband
                cmd_demands_rotation = abs(w_cmd) > self.angular_deadband

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
        if not freeze_pose_update:
            if imu_yaw_allowed and self.imu_yaw is not None:
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

        stamp = now - rclpy.duration.Duration(seconds=0.002)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        if imu_yaw_allowed and self.imu_quat is not None and not freeze_pose_update:
            t.transform.rotation = self.imu_quat
        else:
            t.transform.rotation.z = math.sin(self.yaw / 2.0)
            t.transform.rotation.w = math.cos(self.yaw / 2.0)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        if imu_yaw_allowed and self.imu_quat is not None and not freeze_pose_update:
            odom.pose.pose.orientation = self.imu_quat
        else:
            odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        _PC = 0.05                                                     
        _YC = 0.15                                                                                 
        odom.pose.covariance[0]  = _PC      
        odom.pose.covariance[7]  = _PC      
        odom.pose.covariance[14] = 1e6                     
        odom.pose.covariance[21] = 1e6                        
        odom.pose.covariance[28] = 1e6                         
        odom.pose.covariance[35] = _YC        
        odom.twist.covariance[0]  = 0.02      
        odom.twist.covariance[7]  = 1e6                                                      
        odom.twist.covariance[14] = 1e6       
        odom.twist.covariance[21] = 1e6              
        odom.twist.covariance[28] = 1e6               
        odom.twist.covariance[35] = 0.05            

        self.odom_pub.publish(odom)

        x_rel_msg = Float32()
        y_rel_msg = Float32()
        x_rel_msg.data = float(self.x - getattr(self, 'x_origin', 0.0))
        y_rel_msg.data = float(self.y - getattr(self, 'y_origin', 0.0))
        self.x_rel_pub.publish(x_rel_msg)
        self.y_rel_pub.publish(y_rel_msg)

        if self.serial_disabled and not self.odom_fallback_active:
            self.odom_fallback_active = True
            self.get_logger().warn('⚠️ Using FALLBACK odometry (no serial feedback from Arduino)')
        elif not self.serial_disabled and self.odom_fallback_active:
            self.odom_fallback_active = False
            self.get_logger().info('✅ Serial communication restored')

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
        return '↔️  MIXED'

    def _send_rear_limited_backward(self, pwm, backward_action, backward_log='warn') -> bool:
        if self._rear_limit_hit():
            self._send_stop(action_override='🛑 STOPPED (REAR LIMIT)', log_level='warn')
            return False
        self._send_backward(pwm, action_override=backward_action, log_level=backward_log)
        return True

    def _send_motor_pwm(self, pwm_left, pwm_right, action_override=None, log_level='info', source='unknown'):
        try:
            if self.ser is None:
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
            self.last_cmd_pwm_left = pwm_left
            self.last_cmd_pwm_right = pwm_right
            self.last_cmd_time = now
            # Always log motor output as warn for visibility
            self.get_logger().warn(msg)
            self.last_motor_log_time = now
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _send_stop(self, action_override='🛑 STOPPED', log_level='warn', source='unknown'):
        self._send_motor_pwm(0, 0, action_override=action_override, log_level=log_level, source=source)

    def _send_backward(self, pwm, action_override='⬇️  BACKWARD', log_level='warn', source='unknown'):
        pwm = -abs(int(pwm))
        self._send_motor_pwm(pwm, pwm, action_override=action_override, log_level=log_level, source=source)

    def _rear_limit_hit(self) -> bool:
        return self.last_rear_distance <= self.rear_backup_min_distance

    def _send_osc_turn(self, turn_pwm):
        if self._osc_escape_turn_dir > 0:
            self._send_motor_pwm(0, turn_pwm, source='osc_turn')
        else:
            self._send_motor_pwm(turn_pwm, 0, source='osc_turn')

    def _handle_serial_unavailable(self):
        now = time.time()
        if (now - self.last_serial_reconnect_time) >= self.serial_reconnect_interval:
            self.last_serial_reconnect_time = now
            if self._open_serial_connection():
                self.get_logger().warn('🔌 Arduino serial reconnected after startup failure')
            elif (now - self.last_serial_error_log_time) >= self.serial_error_log_interval:
                self.last_serial_error_log_time = now
                self.get_logger().warn('⏳ Arduino serial not available yet; retrying...')

    def _publish_ultrasonic_distance(self, distance_m: float):
        msg = Float32()
        msg.data = distance_m
        self.ultrasonic_pub.publish(msg)
        self._publish_ultrasonic_range(distance_m)

    def _handle_safety_stop_clear(self):
        msg = Float32()
        msg.data = 999.0
        self.safety_stop_pub.publish(msg)
        self.safety_stop_clear_hits += 1
        self.safety_stop_obstacle_hits = 0
        if self.safety_stop_clear_hits >= self.safety_stop_clear_confirm_count:
            self.get_logger().warn('✅ SAFETY_STOP:0 - Obstacle cleared')

    def _apply_ultrasonic_emergency(self, distance_cm: float, distance_m: float):
        now = time.time()
        self.front_blocked_until = max(self.front_blocked_until, now + self.safety_stop_front_latch_time)
        self.last_front_distance = min(self.last_front_distance, distance_m)
        self.get_logger().error(
            f'🚨 SAFETY_STOP! Published /safety_stop: {distance_cm}cm ({distance_m:.4f}m)'
        )

        if (now - self.last_safety_stop_brake_time) >= 0.2:
            self.last_safety_stop_brake_time = now
            self.last_cmd_pwm_left = 0
            self.last_cmd_pwm_right = 0
            self.last_cmd_time = now
            self._send_stop(
                action_override='🛑 STOPPED (ULTRASONIC FRONT)',
                log_level='warn',
                source='safety_stop'
            )

        if self.enable_safety_override:
            self.safety_override_until = max(self.safety_override_until, now + self.safety_stop_backup_time)
            if not self.safety_override_active:
                self.safety_override_active = True
                self.get_logger().warn(
                    f'🚨 SAFETY OVERRIDE: backing up for {self.safety_stop_backup_time:.1f}s'
                )

    def _handle_safety_stop_line(self, line: str):
        self.get_logger().warn(f'🚨 Arduino: {line}')
        try:
            if ',' not in line:
                self._handle_safety_stop_clear()
                return

            parts = line.split(',')
            if len(parts) < 2:
                return
            distance_cm = float(parts[1])
            distance_m = distance_cm / 100.0

            self._publish_ultrasonic_distance(distance_m)
            msg = Float32()
            msg.data = distance_m
            self.safety_stop_pub.publish(msg)

            if distance_m <= self.safety_stop_trigger_distance:
                self.safety_stop_obstacle_hits += 1
                self.safety_stop_clear_hits = 0
                if self.safety_stop_obstacle_hits >= self.safety_stop_confirm_count:
                    self._apply_ultrasonic_emergency(distance_cm, distance_m)
            else:
                self.safety_stop_obstacle_hits = 0
                self.safety_stop_clear_hits += 1
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Failed to parse SAFETY_STOP: {line} - {e}')

    def _publish_imu_from_parts(self, parts):
        if self.imu_pub is None or len(parts) < 6:
            return
        try:
            ax = float(parts[0]) * self.imu_accel_scale
            ay = float(parts[1]) * self.imu_accel_scale
            az = float(parts[2]) * self.imu_accel_scale
            gx = float(parts[3]) * self.imu_gyro_scale
            gy = float(parts[4]) * self.imu_gyro_scale
            gz = float(parts[5]) * self.imu_gyro_scale
            if self.imu_rotated_180:
                ax, ay, gz = -ax, -ay, -gz

            all_zero = (ax == 0.0 and ay == 0.0 and az == 0.0 and gx == 0.0 and gy == 0.0 and gz == 0.0)
            if all_zero:
                self._imu_zero_streak += 1
                if self._imu_zero_streak >= 5 and not self.imu_hardware_dead:
                    self.imu_hardware_dead = True
                    self.get_logger().warn(
                        '⚠️ IMU hardware DEAD: all-zero for 5+ reads. '
                        'Odom heading switching to PWM-ratio fallback. '
                        'Check MPU6050 I2C wiring on Arduino.'
                    )
            else:
                self._imu_zero_streak = 0
                if self.imu_hardware_dead:
                    self.imu_hardware_dead = False
                    self.get_logger().info('✅ IMU hardware RECOVERED — switching back to gyro heading.')

            imu_msg = self._Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.orientation_covariance[0] = -1.0
            gyro_cov = 9999.0 if self.imu_hardware_dead else 0.01
            imu_msg.angular_velocity_covariance[0] = gyro_cov
            imu_msg.angular_velocity_covariance[4] = gyro_cov
            imu_msg.angular_velocity_covariance[8] = gyro_cov
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1
            self.imu_pub.publish(imu_msg)
        except (ValueError, IndexError):
            pass

    def _update_motor_feedback_from_parts(self, parts):
        if len(parts) < 9:
            return
        try:
            self.last_motor_speed_left = float(parts[7])
            self.last_motor_speed_right = float(parts[8])
            self.last_motor_speed_time = time.time()
        except (ValueError, IndexError):
            pass

    def _handle_csv_line(self, line: str):
        if not line or ',' not in line or line.startswith('ACK'):
            return
        parts = line.split(',')
        if len(parts) < 7:
            return

        try:
            distance_raw = float(parts[6])
            if distance_raw > 0.0:
                self._publish_ultrasonic_distance(distance_raw / 100.0)
        except (ValueError, IndexError):
            pass

        self._publish_imu_from_parts(parts)
        self._update_motor_feedback_from_parts(parts)

    def _handle_serial_reader_exception(self, err):
        self.serial_error_streak += 1
        now = time.time()
        if (now - self.last_serial_error_log_time) >= self.serial_error_log_interval:
            self.last_serial_error_log_time = now
            self.get_logger().error(f'Serial read error ({self.serial_error_streak}): {err}')

        should_retry = (
            self.serial_error_streak >= self.serial_max_error_streak and
            (now - self.last_serial_reconnect_time) >= self.serial_reconnect_interval
        )
        if not should_retry:
            return

        self.last_serial_reconnect_time = now
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        if (not self._open_serial_connection() and
                (now - self.last_serial_error_log_time) >= self.serial_error_log_interval):
            self.last_serial_error_log_time = now
            self.get_logger().error('❌ Serial reconnect failed; will retry')

    def _osc_escape_step_loop(self):
        """Timer callback (10 Hz) that drives the backup→turn sequence of an oscillation escape."""
        if not self._osc_escape_active:
            return
        now  = time.time()
        back_pwm = max(0, min(255, abs(self.fixed_pwm_backward)))
        turn_pwm = max(0, min(255, abs(self.fixed_pwm_turn)))

        if self._osc_escape_step == 0:                
            if now - self._osc_escape_step_start < self._osc_escape_backup_dur:
                self._send_backward(back_pwm, source='osc_backup')
            else:
                if self._osc_escape_turn_dur <= 0.0:
                    self._osc_escape_active = False
                    self.cmd_vel_override_until = 0.0
                    self._send_stop(source='osc_done')
                    self.get_logger().info("✅ Oscillation escape complete (backup only) — resuming normal control")
                    return
                self._osc_escape_step       = 1
                self._osc_escape_step_start = now
                self._send_osc_turn(turn_pwm)
        elif self._osc_escape_step == 1:              
            if now - self._osc_escape_step_start < self._osc_escape_turn_dur:
                self._send_osc_turn(turn_pwm)
            else:
                self._osc_escape_active = False
                self.cmd_vel_override_until = 0.0
                self._send_stop(source='osc_done')
                self.get_logger().info("✅ Oscillation escape complete — resuming normal control")

    def _stuck_recovery_loop(self):
        if self.ser is None:
            return

        now = time.time()

        if self.stuck_recovery_active:
            recovery_done = now >= self.stuck_recovery_end_time
            if recovery_done:
                self.stuck_recovery_active = False
                self.stuck_start_time = None

            action = '⬇️  BACKWARD (STUCK EXIT)' if recovery_done else '⬇️  BACKWARD (STUCK RECOVERY)'
            moving = self._send_rear_limited_backward(self.stuck_backup_pwm, action, backward_log='warn')
            if not moving:
                self.stuck_recovery_active = False
                self.stuck_start_time = None
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
            now = time.time()
            uncertain_state = not self.nav2_explicitly_inactive
            within_uncertain_grace = (now - self.last_nav2_ready_true_at) <= self.nav2_uncertain_grace_sec
            allow_hold = uncertain_state and (within_uncertain_grace or self.has_active_goal)
            if allow_hold:
                return

            self.safety_override_active = False
            self.safety_override_until = 0.0
            if (
                self.last_cmd_pwm_left != 0 or
                self.last_cmd_pwm_right != 0 or
                (now - self.last_nav2_inactive_stop_time) >= self.nav2_inactive_stop_repeat_sec
            ):
                self.last_nav2_inactive_stop_time = now
                self._send_stop(action_override='🛑 STOPPED (NAV2 INACTIVE)', log_level='none', source='nav2_inactive')
            return

        if not self.enable_safety_override:
            return

        now = time.time()
        if now < self.safety_override_until:
            moving = self._send_rear_limited_backward(
                self.safety_stop_backup_pwm,
                '⬇️  BACKWARD (SAFETY OVERRIDE)',
                backward_log='warn'
            )
            if not moving:
                self.safety_override_active = False
                self.safety_override_until = 0.0
            return

        if self.safety_override_active:
            self.safety_override_active = False
            self._send_rear_limited_backward(
                self.safety_stop_backup_pwm,
                '⬇️  BACKWARD (SAFETY EXIT)',
                backward_log='info'
            )
    
    def _serial_reader(self):
        """Background thread to read and parse Arduino CSV data"""
        while self.serial_reading:
            try:
                if self.ser is None or self.serial_disabled:
                    self._handle_serial_unavailable()
                    time.sleep(0.1)
                    continue

                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('SAFETY_STOP'):
                        self._handle_safety_stop_line(line)
                    else:
                        self._handle_csv_line(line)
                self.serial_error_streak = 0
                
                time.sleep(0.01)                   
            except Exception as e:
                self._handle_serial_reader_exception(e)
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

