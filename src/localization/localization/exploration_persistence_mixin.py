#!/usr/bin/env python3

import os
import time
import math
import csv
import json
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class ExplorationPersistenceMixin:
    def _lookup_pose_in_frame(self, frame_name: str):
        """Lookup robot pose in a target frame from TF."""
        try:
            import rclpy
            from rclpy.duration import Duration

            timeout = Duration(seconds=0.1)
            if not self.tf_buffer.can_transform(frame_name, self.base_frame, rclpy.time.Time(), timeout=timeout):
                return None
            tf = self.tf_buffer.lookup_transform(frame_name, self.base_frame, rclpy.time.Time(), timeout=timeout)
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            q = tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except Exception:
            return None

    def _save_path_diagnostics_from_csv(self, csv_path: str):
        """Save per-session path diagnostics files next to the path CSV."""
        try:
            if not csv_path or not os.path.exists(csv_path):
                return

            def _pick_float(row, keys, default=0.0):
                for key in keys:
                    value = row.get(key)
                    if value is not None and value != '':
                        return float(value)
                return float(default)

            rows = []
            with open(csv_path, 'r', encoding='ascii') as f:
                reader = csv.DictReader(f)
                for r in reader:
                    try:
                        t = float(r.get('elapsed_s', 0.0))
                        x = _pick_float(r, ('x_m', 'x_rel_m', 'x_explored_m'))
                        y = _pick_float(r, ('y_m', 'y_rel_m', 'y_explored_m'))
                        rows.append((t, x, y))
                    except Exception:
                        continue

            if not rows:
                self.get_logger().warn('️ Path diagnostics skipped: no valid path rows')
                return

            times = [r[0] for r in rows]
            xs = [r[1] for r in rows]
            ys = [r[2] for r in rows]
            seg_dists = []
            seg_speeds = []
            for i in range(1, len(rows)):
                dt = max(1e-6, times[i] - times[i - 1])
                dist = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                seg_dists.append(dist)
                seg_speeds.append(dist / dt)

            total_dist = sum(seg_dists)
            duration = max(0.0, times[-1] - times[0])
            max_speed = max(seg_speeds) if seg_speeds else 0.0
            mean_speed = (sum(seg_speeds) / len(seg_speeds)) if seg_speeds else 0.0
            speed_spike_threshold = 0.8
            spike_count = sum(1 for v in seg_speeds if v > speed_spike_threshold)

            # Late-map-jump detector for stop mismatch: if a large segment appears
            # near the end, keep a "stable" stop estimate before the jump.
            jump_threshold_m = 0.35
            stable_end_idx = len(rows) - 1
            jump_flag = False
            jump_dist = 0.0
            jump_dt = 0.0
            lookback = min(3, max(0, len(rows) - 1))
            if lookback > 0:
                seg_start = len(rows) - lookback
                largest = None
                for i in range(max(1, seg_start), len(rows)):
                    dt_i = max(1e-6, times[i] - times[i - 1])
                    dist_i = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                    if dist_i > jump_threshold_m:
                        if largest is None or dist_i > largest[0]:
                            largest = (dist_i, dt_i, i - 1, i)
                if largest is not None:
                    jump_flag = True
                    jump_dist = float(largest[0])
                    jump_dt = float(largest[1])
                    stable_end_idx = int(largest[2])

            # Special case: final point snaps back very close to initial pose,
            # which is often a shutdown TF reset rather than a true robot stop.
            if len(rows) >= 2:
                try:
                    init_x = float(self.get_parameter('auto_initial_pose_x_m').value)
                    init_y = float(self.get_parameter('auto_initial_pose_y_m').value)
                except Exception:
                    init_x = 0.93
                    init_y = 0.15
                end_to_init = math.hypot(xs[-1] - init_x, ys[-1] - init_y)
                prev_to_init = math.hypot(xs[-2] - init_x, ys[-2] - init_y)
                if end_to_init <= 0.06 and prev_to_init >= 0.18:
                    jump_flag = True
                    stable_end_idx = len(rows) - 2
                    jump_dist = math.hypot(xs[-1] - xs[-2], ys[-1] - ys[-2])
                    jump_dt = max(1e-6, times[-1] - times[-2])

            stats = {
                'rows': len(rows),
                'duration_s': duration,
                'total_distance_m': total_dist,
                'start_x_m': xs[0],
                'start_y_m': ys[0],
                'end_x_m': xs[-1],
                'end_y_m': ys[-1],
                'max_segment_speed_mps': max_speed,
                'mean_segment_speed_mps': mean_speed,
                'speed_spike_threshold_mps': speed_spike_threshold,
                'speed_spike_count': spike_count,
                'end_stable_x_m': xs[stable_end_idx],
                'end_stable_y_m': ys[stable_end_idx],
                'end_jump_flag': int(jump_flag),
                'end_jump_distance_m': jump_dist,
                'end_jump_dt_s': jump_dt,
            }

            stem, _ = os.path.splitext(csv_path)
            stats_csv_path = f'{stem}_stats.csv'
            stats_json_path = f'{stem}_stats.json'
            stop_note_csv_path = f'{stem}_stop_note.csv'
            stop_note_json_path = f'{stem}_stop_note.json'

            with open(stats_csv_path, 'w', encoding='ascii') as f:
                f.write('metric,value\n')
                for k, v in stats.items():
                    if isinstance(v, float):
                        f.write(f'{k},{v:.6f}\n')
                    else:
                        f.write(f'{k},{v}\n')

            with open(stats_json_path, 'w', encoding='ascii') as f:
                json.dump(stats, f, indent=2)

            stop_note = {
                'data_end_x_m': xs[-1],
                'data_end_y_m': ys[-1],
                'stable_end_x_m': xs[stable_end_idx],
                'stable_end_y_m': ys[stable_end_idx],
                'jump_flag': int(jump_flag),
                'jump_distance_m': jump_dist,
                'jump_dt_s': jump_dt,
                'note': 'jump_flag=1 means final saved end may not match physical stop; use stable_end_* for stop comparison',
            }
            with open(stop_note_csv_path, 'w', encoding='ascii') as f:
                f.write('metric,value\n')
                for k, v in stop_note.items():
                    f.write(f'{k},{v}\n')
            with open(stop_note_json_path, 'w', encoding='ascii') as f:
                json.dump(stop_note, f, indent=2)

            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt

                plot_origin_x = 0.0
                plot_origin_y = 0.0
                try:
                    start_map_x = float(self.robot_path_series[0][1])
                    start_map_y = float(self.robot_path_series[0][2])
                    plot_origin_x, plot_origin_y = self._path_absolute_room_coords(start_map_x, start_map_y)
                except Exception:
                    plot_origin_x = float(xs[0]) if xs else 0.0
                    plot_origin_y = float(ys[0]) if ys else 0.0

                def _room_plot_point(map_x: float, map_y: float):
                    rx, ry = self._path_absolute_room_coords(float(map_x), float(map_y))
                    return (rx - plot_origin_x, ry - plot_origin_y)

                nav_goal_points = []
                frontier_points = []
                frontier_links = []
                for row in getattr(self, 'nav_goals_series', []):
                    try:
                        goal_pt = _room_plot_point(float(row[1]), float(row[2]))
                        frontier_pt = _room_plot_point(float(row[3]), float(row[4]))
                        nav_goal_points.append(goal_pt)
                        frontier_links.append((frontier_pt, goal_pt))
                    except Exception:
                        continue
                for row in getattr(self, 'frontier_history_series', []):
                    try:
                        frontier_points.append(_room_plot_point(float(row[1]), float(row[2])))
                    except Exception:
                        continue

                def _plot_overlays(ax):
                    if frontier_points:
                        fx = [p[0] for p in frontier_points]
                        fy = [p[1] for p in frontier_points]
                        ax.scatter(fx, fy, s=10, marker='.', alpha=0.28, color='tab:green', label='frontier points')
                    if frontier_links:
                        for frontier_pt, goal_pt in frontier_links:
                            ax.plot(
                                [frontier_pt[0], goal_pt[0]],
                                [frontier_pt[1], goal_pt[1]],
                                linestyle='--',
                                linewidth=0.8,
                                alpha=0.30,
                                color='tab:orange'
                            )
                    if nav_goal_points:
                        gx = [p[0] for p in nav_goal_points]
                        gy = [p[1] for p in nav_goal_points]
                        ax.scatter(gx, gy, s=70, marker='*', color='tab:orange', edgecolors='black', linewidths=0.3, label='Nav2 goals')

                # Raw path plot
                fig = plt.figure(figsize=(6.5, 6.0))
                ax = fig.add_subplot(111)
                ax.plot(xs, ys, '-', linewidth=1.6)
                ax.scatter([xs[0]], [ys[0]], s=50, marker='o', label='start')
                ax.scatter([xs[-1]], [ys[-1]], s=50, marker='x', label='end')
                _plot_overlays(ax)
                ax.set_xlabel('x (m)')
                ax.set_ylabel('y (m)')
                ax.set_title('Robot Path')
                ax.axis('equal')
                ax.grid(True, alpha=0.35)
                ax.legend(loc='best')
                fig.tight_layout()
                fig.savefig(f'{stem}_plot.png', dpi=140)
                plt.close(fig)

                # Time-colored path plot
                fig = plt.figure(figsize=(6.5, 6.0))
                ax = fig.add_subplot(111)
                sc = ax.scatter(xs, ys, c=times, s=18, cmap='viridis')
                ax.plot(xs, ys, '-', linewidth=1.0, alpha=0.55)
                _plot_overlays(ax)
                ax.set_xlabel('x (m)')
                ax.set_ylabel('y (m)')
                ax.set_title('Robot Path (Time Colored)')
                ax.axis('equal')
                ax.grid(True, alpha=0.35)
                cb = fig.colorbar(sc, ax=ax)
                cb.set_label('elapsed_s')
                fig.tight_layout()
                fig.savefig(f'{stem}_plot_timecolor.png', dpi=140)
                plt.close(fig)

                # Raw vs smoothed path
                window = 5
                half = window // 2
                sm_x = []
                sm_y = []
                for i in range(len(xs)):
                    lo = max(0, i - half)
                    hi = min(len(xs), i + half + 1)
                    sm_x.append(sum(xs[lo:hi]) / float(hi - lo))
                    sm_y.append(sum(ys[lo:hi]) / float(hi - lo))

                fig = plt.figure(figsize=(6.5, 6.0))
                ax = fig.add_subplot(111)
                ax.plot(xs, ys, '-', linewidth=1.0, alpha=0.45, label='raw')
                ax.plot(sm_x, sm_y, '-', linewidth=2.0, label='smoothed')
                _plot_overlays(ax)
                ax.set_xlabel('x (m)')
                ax.set_ylabel('y (m)')
                ax.set_title('Robot Path (Raw vs Smoothed)')
                ax.axis('equal')
                ax.grid(True, alpha=0.35)
                ax.legend(loc='best')
                fig.tight_layout()
                fig.savefig(f'{stem}_plot_smoothed.png', dpi=140)
                plt.close(fig)
            except Exception as plot_err:
                self.get_logger().warn(f'⚠️ Plot generation skipped: {plot_err}')

            self.get_logger().info(
                f'📊 Path diagnostics saved: {stats_csv_path}, {stats_json_path}, {stop_note_csv_path}'
            )
            if jump_flag:
                self.get_logger().warn(
                    f'⚠️ Stop mismatch note: late jump detected ({jump_dist:.3f}m over {jump_dt:.2f}s). '
                    f'Using stable_end=({xs[stable_end_idx]:.4f},{ys[stable_end_idx]:.4f}) for physical stop check.'
                )
        except Exception as e:
            self.get_logger().error(f'❌ Failed to save path diagnostics: {e}')

    def _path_absolute_room_coords(self, x: float, y: float):
        """Return absolute room-frame coordinates (not relative to initial pose start).
        This includes the configured initial pose offset plus rotation to align with initial heading."""
        if getattr(self, '_path_initial_pose_x', None) is None:
            # Initialize with configured initial pose parameters
            try:
                self._path_initial_pose_x = float(self.get_parameter('auto_initial_pose_x_m').value)
                self._path_initial_pose_y = float(self.get_parameter('auto_initial_pose_y_m').value)
                self._path_initial_pose_yaw = float(self.get_parameter('auto_initial_pose_yaw_rad').value)
            except Exception:
                self._path_initial_pose_x = 0.93
                self._path_initial_pose_y = 0.15
                self._path_initial_pose_yaw = 0.0
            # If initial pose callback did not seed an origin, fall back to first TF sample.
            if getattr(self, '_path_origin_x', None) is None or getattr(self, '_path_origin_y', None) is None:
                self._path_origin_x = float(x)
                self._path_origin_y = float(y)

        # Map frame coordinates relative to map origin
        dx = float(x) - float(self._path_origin_x)
        dy = float(y) - float(self._path_origin_y)

        try:
            align_heading = bool(self.get_parameter('path_log_align_to_initial_heading').value)
        except Exception:
            align_heading = True

        # Rotate to align with initial heading
        if align_heading:
            yaw0 = float(self._path_initial_pose_yaw)
            c = math.cos(yaw0)
            s = math.sin(yaw0)
            x_rot = (c * dx) + (s * dy)
            y_rot = (-s * dx) + (c * dy)
        else:
            x_rot = dx
            y_rot = dy

        # Add initial pose offset to get absolute room coordinates
        x_room = self._path_initial_pose_x + x_rot
        y_room = self._path_initial_pose_y + y_rot

        # Optional room-frame calibration transform for real-world alignment.
        # This lets us align logged coordinates to floor markers without changing
        # navigation frames or the published initial pose.
        try:
            yaw_off = float(getattr(self, 'path_room_yaw_offset_rad', 0.0))
            x_off = float(getattr(self, 'path_room_x_offset_m', 0.0))
            y_off = float(getattr(self, 'path_room_y_offset_m', 0.0))
        except Exception:
            yaw_off = 0.0
            x_off = 0.0
            y_off = 0.0

        if abs(yaw_off) > 1e-9:
            c2 = math.cos(yaw_off)
            s2 = math.sin(yaw_off)
            x_cal = (c2 * x_room) - (s2 * y_room)
            y_cal = (s2 * x_room) + (c2 * y_room)
        else:
            x_cal = x_room
            y_cal = y_room

        return (x_cal + x_off), (y_cal + y_off)

    def _path_relative_coords(self, x: float, y: float):
        """Return start-relative coordinates (offset from map origin, rotated to align with heading)."""
        if getattr(self, '_path_origin_x', None) is None or getattr(self, '_path_origin_y', None) is None:
            self._path_origin_x = float(x)
            self._path_origin_y = float(y)
            yaw0 = 0.0
            try:
                pose = getattr(self, 'robot_pose', None)
                if pose is not None and len(pose) >= 3:
                    yaw0 = float(pose[2])
            except Exception:
                yaw0 = 0.0
            self._path_origin_yaw = yaw0

        dx = float(x) - float(self._path_origin_x)
        dy = float(y) - float(self._path_origin_y)

        try:
            align_heading = bool(self.get_parameter('path_log_align_to_initial_heading').value)
        except Exception:
            align_heading = True

        if not align_heading:
            return dx, dy

        yaw0 = float(getattr(self, '_path_origin_yaw', 0.0))
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        x_rel = (c * dx) + (s * dy)
        y_rel = (-s * dx) + (c * dy)
        return x_rel, y_rel

    def _on_exploration_complete(self):
        if hasattr(self, '_completion_cells_known') and not self._completion_cells_known():
            unknown_cells = int(getattr(self, 'last_unknown_cells', 0))
            max_unknown = max(0, int(getattr(self, 'completion_max_unknown_cells', 0)))
            self.get_logger().warn(
                f"⛔ Completion blocked: unknown cells remain ({unknown_cells}>{max_unknown})."
            )
            return

        with self._exploration_complete_lock:
            if self._exploration_complete_handled:
                return
            self._exploration_complete_handled = True

        if not self.complete_marker_sent:
            self._publish_complete_marker()
            self.complete_marker_sent = True
        import datetime as _dt
        self.completion_datetime = _dt.datetime.now()
        self.completion_elapsed_s = time.time() - self.startup_time
        self.get_logger().info(
            f"🗺️ Map completion: {self.last_percent_known:.2f}% explored | "
            f"elapsed {self.completion_elapsed_s:.1f}s | "
            f"completed at {self.completion_datetime.strftime('%Y-%m-%d %H:%M:%S')}"
        )
        self.get_logger().info("EXPLORATION COMPLETE - Saving map and outputs...")
        self._flush_csv_data()
        self._archive_exploration_outputs()
        self._save_and_shutdown()

    def _save_and_shutdown(self):
        import signal as _signal
        import threading as _threading
        import sys as _sys

        def _do_shutdown():
            import time as _time
            if self.auto_save_on_complete and not self.map_save_requested:
                self.map_save_requested = True
                self._request_map_save()
            _time.sleep(4.0)
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            self.cmd_vel_nav_pub.publish(stop)
            self.get_logger().info("Auto-shutdown: map saved, terminating all nodes.")
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
            self.get_logger().warn('️ No mapped area data to save')
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
        x = float(x)
        y = float(y)

        # Do not record from stale TF snapshots.
        if bool(getattr(self, 'pose_stale', False)):
            return

        max_pose_age = getattr(self, '_path_pose_max_age_sec_val', None)
        if max_pose_age is None:
            try:
                max_pose_age = float(self.get_parameter('path_pose_max_age_sec').value)
            except Exception:
                max_pose_age = 0.5
            self._path_pose_max_age_sec_val = max_pose_age

        try:
            stamp = getattr(self, 'last_tf_stamp', None)
            if stamp is not None:
                now_msg = self.get_clock().now().to_msg()
                now_s = float(now_msg.sec) + (float(now_msg.nanosec) * 1e-9)
                stamp_s = float(stamp.sec) + (float(stamp.nanosec) * 1e-9)
                tf_age = max(0.0, now_s - stamp_s)
                if tf_age > max(0.05, float(max_pose_age)):
                    now_t = time.time()
                    if (now_t - float(getattr(self, 'last_path_pose_age_log_time', 0.0))) >= 2.0:
                        self.last_path_pose_age_log_time = now_t
                        self.get_logger().warn(
                            f"⚠️ Skipping path sample: TF pose too old ({tf_age:.2f}s > {float(max_pose_age):.2f}s)"
                        )
                    return
        except Exception:
            pass

        min_dist = getattr(self, '_path_record_min_dist_val', None)
        if min_dist is None:
            try:
                min_dist = self.get_parameter('path_record_min_dist').value
            except Exception:
                min_dist = 0.2
            self._path_record_min_dist_val = min_dist
        max_interval = getattr(self, '_path_record_max_interval_val', None)
        if max_interval is None:
            try:
                max_interval = self.get_parameter('path_record_max_interval').value
            except Exception:
                max_interval = 1.0
            self._path_record_max_interval_val = max_interval
        time_only = getattr(self, '_path_record_time_only_val', None)
        if time_only is None:
            try:
                time_only = bool(self.get_parameter('path_record_time_only').value)
            except Exception:
                time_only = False
            self._path_record_time_only_val = time_only
        max_speed = getattr(self, '_path_max_speed_mps_val', None)
        if max_speed is None:
            try:
                max_speed = float(self.get_parameter('path_max_speed_mps').value)
            except Exception:
                max_speed = 1.2
            self._path_max_speed_mps_val = max_speed
        lx = self.last_recorded_path_x
        ly = self.last_recorded_path_y
        now = time.time()
        dt = now - float(getattr(self, 'last_recorded_path_time', 0.0) or 0.0)
        if lx is not None and ly is not None and dt > 1e-3:
            dist = math.hypot(x - lx, y - ly)
            speed = dist / dt
            if speed > max(0.2, float(max_speed)):
                if (now - float(getattr(self, 'last_path_speed_outlier_log_time', 0.0))) >= 2.0:
                    self.last_path_speed_outlier_log_time = now
                    self.get_logger().warn(
                        f"⚠️ Skipping path outlier: {speed:.2f}m/s > {float(max_speed):.2f}m/s"
                    )
                return
        moved_enough = (lx is None or ly is None or ((x - lx) ** 2 + (y - ly) ** 2) >= min_dist ** 2)
        timed_out = dt >= max(0.2, float(max_interval))
        is_duplicate = (lx is not None and ly is not None and ((x - lx) ** 2 + (y - ly) ** 2) < 1e-8)
        should_record = (timed_out and not is_duplicate) if bool(time_only) else (moved_enough or (timed_out and not is_duplicate))
        if should_record:
            elapsed = time.time() - self.startup_time
            self.robot_path_series.append((elapsed, x, y))
            self.last_recorded_path_x = x
            self.last_recorded_path_y = y
            self.last_recorded_path_time = now

    def _save_robot_path_series(self):
        if not self.robot_path_series:
            self.get_logger().warn('️ No robot path data to save')
            return
        os.makedirs(self.map_save_dir, exist_ok=True)
        path_file = getattr(self, 'robot_path_file', 'auto_explore_path.csv')
        out_path = os.path.join(self.map_save_dir, path_file)
        stem, _ = os.path.splitext(out_path)
        relative_path_out = f'{stem}_relative.csv'
        raw_smoothed_out = f'{stem}_raw_smoothed.csv'
        compare_plot_out = f'{stem}_plot_explored_vs_relative.png'
        try:
            write_room_aligned = bool(getattr(self, 'path_log_relative_to_initial_pose', True))
        except Exception:
            write_room_aligned = True
        force_zero_start = True

        # Build explored (absolute) rows first, then derive relative rows from start.
        explored_rows = []
        for row in self.robot_path_series:
            t = float(row[0])
            x_map = float(row[1])
            y_map = float(row[2])
            if write_room_aligned:
                x_abs, y_abs = self._path_absolute_room_coords(x_map, y_map)
            else:
                x_abs, y_abs = x_map, y_map
            explored_rows.append((t, float(x_abs), float(y_abs)))

        relative_rows = list(explored_rows)
        if force_zero_start and relative_rows:
            x0 = relative_rows[0][1]
            y0 = relative_rows[0][2]
            relative_rows = [(t, x - x0, y - y0) for (t, x, y) in relative_rows]

        def _moving_average(values, window=5):
            if not values:
                return []
            w = max(1, int(window))
            half = w // 2
            out = []
            n = len(values)
            for i in range(n):
                lo = max(0, i - half)
                hi = min(n, i + half + 1)
                out.append(sum(values[lo:hi]) / float(max(1, hi - lo)))
            return out

        try:
            # Primary file now carries both the relative and explored coordinates.
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('elapsed_s,x_rel_m,y_rel_m,x_explored_m,y_explored_m\n')
                for rel_row, exp_row in zip(relative_rows, explored_rows):
                    t = rel_row[0]
                    x_rel, y_rel = rel_row[1], rel_row[2]
                    x_exp, y_exp = exp_row[1], exp_row[2]
                    f.write(f"{t:.2f},{x_rel:.4f},{y_rel:.4f},{x_exp:.4f},{y_exp:.4f}\n")

            with open(relative_path_out, 'w', encoding='ascii') as f:
                f.write('elapsed_s,x_rel_m,y_rel_m\n')
                for t, x_rel, y_rel in relative_rows:
                    f.write(f"{t:.2f},{x_rel:.4f},{y_rel:.4f}\n")

            rel_x = [r[1] for r in relative_rows]
            rel_y = [r[2] for r in relative_rows]
            exp_x = [r[1] for r in explored_rows]
            exp_y = [r[2] for r in explored_rows]
            rel_x_s = _moving_average(rel_x, window=5)
            rel_y_s = _moving_average(rel_y, window=5)
            exp_x_s = _moving_average(exp_x, window=5)
            exp_y_s = _moving_average(exp_y, window=5)

            with open(raw_smoothed_out, 'w', encoding='ascii') as f:
                f.write(
                    'elapsed_s,'
                    'x_rel_raw_m,y_rel_raw_m,x_rel_smooth_m,y_rel_smooth_m,'
                    'x_explored_raw_m,y_explored_raw_m,x_explored_smooth_m,y_explored_smooth_m\n'
                )
                for i in range(len(relative_rows)):
                    t = relative_rows[i][0]
                    f.write(
                        f"{t:.2f},"
                        f"{rel_x[i]:.4f},{rel_y[i]:.4f},{rel_x_s[i]:.4f},{rel_y_s[i]:.4f},"
                        f"{exp_x[i]:.4f},{exp_y[i]:.4f},{exp_x_s[i]:.4f},{exp_y_s[i]:.4f}\n"
                    )

            # Additional comparison graph: explored absolute path vs relative path, with overlays.
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt

                abs_x = exp_x
                abs_y = exp_y

                # Prepare overlays (reuse logic from diagnostics plot)
                nav_goal_points_exp = []
                nav_goal_points_rel = []
                frontier_points_exp = []
                frontier_points_rel = []
                frontier_links_exp = []
                frontier_links_rel = []

                def _room_plot_point_explored(map_x: float, map_y: float):
                    return self._path_absolute_room_coords(float(map_x), float(map_y))

                def _room_plot_point_relative(map_x: float, map_y: float):
                    x_abs, y_abs = _room_plot_point_explored(map_x, map_y)
                    if force_zero_start and explored_rows:
                        x0 = explored_rows[0][1]
                        y0 = explored_rows[0][2]
                        return (x_abs - x0, y_abs - y0)
                    return (x_abs, y_abs)

                for row in getattr(self, 'nav_goals_series', []):
                    try:
                        goal_exp = _room_plot_point_explored(float(row[1]), float(row[2]))
                        frontier_exp = _room_plot_point_explored(float(row[3]), float(row[4]))
                        goal_rel = _room_plot_point_relative(float(row[1]), float(row[2]))
                        frontier_rel = _room_plot_point_relative(float(row[3]), float(row[4]))
                        nav_goal_points_exp.append(goal_exp)
                        nav_goal_points_rel.append(goal_rel)
                        frontier_links_exp.append((frontier_exp, goal_exp))
                        frontier_links_rel.append((frontier_rel, goal_rel))
                    except Exception:
                        continue
                for row in getattr(self, 'frontier_history_series', []):
                    try:
                        frontier_points_exp.append(_room_plot_point_explored(float(row[1]), float(row[2])))
                        frontier_points_rel.append(_room_plot_point_relative(float(row[1]), float(row[2])))
                    except Exception:
                        continue

                fig, axes = plt.subplots(1, 2, figsize=(12.4, 6.2))

                ax_exp = axes[0]
                ax_exp.plot(abs_x, abs_y, '-', linewidth=1.8, color='tab:blue', label='explored path')
                if abs_x and abs_y:
                    ax_exp.scatter([abs_x[0]], [abs_y[0]], s=42, marker='o', color='tab:blue')
                    ax_exp.scatter([abs_x[-1]], [abs_y[-1]], s=48, marker='x', color='tab:blue')
                if frontier_points_exp:
                    fx = [p[0] for p in frontier_points_exp]
                    fy = [p[1] for p in frontier_points_exp]
                    ax_exp.scatter(fx, fy, s=10, marker='.', alpha=0.28, color='tab:green', label='frontier points')
                if frontier_links_exp:
                    for frontier_pt, goal_pt in frontier_links_exp:
                        ax_exp.plot(
                            [frontier_pt[0], goal_pt[0]],
                            [frontier_pt[1], goal_pt[1]],
                            linestyle='--',
                            linewidth=0.8,
                            alpha=0.30,
                            color='tab:orange'
                        )
                if nav_goal_points_exp:
                    gx = [p[0] for p in nav_goal_points_exp]
                    gy = [p[1] for p in nav_goal_points_exp]
                    ax_exp.scatter(gx, gy, s=70, marker='*', color='tab:orange', edgecolors='black', linewidths=0.3, label='Nav2 goals')
                ax_exp.set_xlabel('x (m)')
                ax_exp.set_ylabel('y (m)')
                ax_exp.set_title('Explored Path')
                ax_exp.axis('equal')
                ax_exp.grid(True, alpha=0.35)
                ax_exp.legend(loc='best')

                ax_rel = axes[1]
                ax_rel.plot(rel_x, rel_y, '-', linewidth=1.8, color='tab:red', alpha=0.90, label='relative path')
                if rel_x and rel_y:
                    ax_rel.scatter([rel_x[0]], [rel_y[0]], s=42, marker='o', color='tab:red')
                    ax_rel.scatter([rel_x[-1]], [rel_y[-1]], s=48, marker='x', color='tab:red')
                if frontier_points_rel:
                    fx = [p[0] for p in frontier_points_rel]
                    fy = [p[1] for p in frontier_points_rel]
                    ax_rel.scatter(fx, fy, s=10, marker='.', alpha=0.28, color='tab:green', label='frontier points')
                if frontier_links_rel:
                    for frontier_pt, goal_pt in frontier_links_rel:
                        ax_rel.plot(
                            [frontier_pt[0], goal_pt[0]],
                            [frontier_pt[1], goal_pt[1]],
                            linestyle='--',
                            linewidth=0.8,
                            alpha=0.30,
                            color='tab:orange'
                        )
                if nav_goal_points_rel:
                    gx = [p[0] for p in nav_goal_points_rel]
                    gy = [p[1] for p in nav_goal_points_rel]
                    ax_rel.scatter(gx, gy, s=70, marker='*', color='tab:orange', edgecolors='black', linewidths=0.3, label='Nav2 goals')
                ax_rel.set_xlabel('x (m)')
                ax_rel.set_ylabel('y (m)')
                ax_rel.set_title('Relative Path')
                ax_rel.axis('equal')
                ax_rel.grid(True, alpha=0.35)
                ax_rel.legend(loc='best')

                fig.suptitle('Explored Path and Relative Path')
                fig.tight_layout()
                fig.savefig(compare_plot_out, dpi=140)
                plt.close(fig)
            except Exception as plot_err:
                self.get_logger().warn(f'⚠️ Explored-vs-relative plot skipped: {plot_err}')

            self.get_logger().info(
                f"🗺️ Robot path saved: {len(self.robot_path_series)} waypoints → {out_path}")
            self.get_logger().info(
                f"🧭 Relative path CSV saved → {relative_path_out}")
            self.get_logger().info(
                f"🧮 Raw+smoothed path CSV saved → {raw_smoothed_out}")
            self.get_logger().info(
                f"🖼️ Explored-vs-relative plot saved → {compare_plot_out}")

            # Log a compact displacement summary so each session shows stop direction vs initial pose.
            start = self.robot_path_series[0]
            end = self.robot_path_series[-1]
            dx_map = float(end[1]) - float(start[1])
            dy_map = float(end[2]) - float(start[2])
            dist_map = math.hypot(dx_map, dy_map)
            start_abs = self._path_absolute_room_coords(float(start[1]), float(start[2]))
            end_abs = self._path_absolute_room_coords(float(end[1]), float(end[2]))
            dx_abs = float(end_abs[0]) - float(start_abs[0])
            dy_abs = float(end_abs[1]) - float(start_abs[1])
            self.get_logger().info(
                "📍 Path delta: "
                f"map_start=({float(start[1]):.4f},{float(start[2]):.4f}) "
                f"map_end=({float(end[1]):.4f},{float(end[2]):.4f}) "
                f"map_dx={dx_map:.4f} map_dy={dy_map:.4f} map_dist={dist_map:.4f} "
                f"room_start=({float(start_abs[0]):.4f},{float(start_abs[1]):.4f}) "
                f"room_end=({float(end_abs[0]):.4f},{float(end_abs[1]):.4f}) "
                f"room_dx={dx_abs:.4f} room_dy={dy_abs:.4f}"
            )

            # Keep diagnostics files in sync with each new path CSV write.
            self._save_path_diagnostics_from_csv(out_path)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save robot path: {e}")

    def _flush_csv_data(self):
        if getattr(self, '_csv_flushed', False):
            return

        # Capture one final pose sample at shutdown so CSV stop point matches real robot stop.
        try:
            self.update_pose()
            if bool(getattr(self, 'pose_valid', False)):
                x = float(self.robot_pose[0])
                y = float(self.robot_pose[1])
                self._record_path_point(x, y)
        except Exception:
            pass

        # Capture final frame snapshots for mismatch debugging.
        try:
            self.final_pose_map = self._lookup_pose_in_frame('map')
            self.final_pose_odom = self._lookup_pose_in_frame('odom')
            if self.final_pose_map is not None:
                rx, ry = self._path_absolute_room_coords(self.final_pose_map[0], self.final_pose_map[1])
                self.final_pose_room = (float(rx), float(ry), float(self.final_pose_map[2]))
            else:
                self.final_pose_room = None
        except Exception:
            self.final_pose_map = None
            self.final_pose_odom = None
            self.final_pose_room = None

        self._csv_flushed = True
        self._save_mapped_area_series()
        self._save_robot_path_series()
        self._save_nav_goals_series()
        self._save_frontier_history_series()
        self._save_session_summary()

    def _save_session_summary(self):
        import datetime as _dt
        os.makedirs(self.map_save_dir, exist_ok=True)
        out_path = os.path.join(self.map_save_dir, getattr(self, 'session_summary_file', 'explore_summary.csv'))
        now_dt = getattr(self, 'completion_datetime', None) or _dt.datetime.now()
        elapsed = getattr(self, 'completion_elapsed_s', None)
        if elapsed is None:
            elapsed = time.time() - self.startup_time
        start_dt = now_dt - _dt.timedelta(seconds=elapsed)
        status = 'complete' if getattr(self, '_exploration_complete_handled', False) else 'interrupted'
        try:
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('status,start_datetime,completion_datetime,elapsed_s,'
                        'coverage_pct,goals_reached,path_points,nav_goals,frontier_points,'
                        'final_map_x_m,final_map_y_m,final_odom_x_m,final_odom_y_m,final_room_x_m,final_room_y_m\n')
                fm = getattr(self, 'final_pose_map', None)
                fo = getattr(self, 'final_pose_odom', None)
                fr = getattr(self, 'final_pose_room', None)
                fm_x = '' if fm is None else f'{float(fm[0]):.4f}'
                fm_y = '' if fm is None else f'{float(fm[1]):.4f}'
                fo_x = '' if fo is None else f'{float(fo[0]):.4f}'
                fo_y = '' if fo is None else f'{float(fo[1]):.4f}'
                fr_x = '' if fr is None else f'{float(fr[0]):.4f}'
                fr_y = '' if fr is None else f'{float(fr[1]):.4f}'
                f.write(
                    f"{status},"
                    f"{start_dt.strftime('%Y-%m-%d %H:%M:%S')},"
                    f"{now_dt.strftime('%Y-%m-%d %H:%M:%S')},"
                    f"{elapsed:.1f},"
                    f"{getattr(self, 'last_percent_known', 0.0):.2f},"
                    f"{getattr(self, 'goals_reached', 0)},"
                    f"{len(getattr(self, 'robot_path_series', []))},"
                    f"{len(getattr(self, 'nav_goals_series', []))},"
                    f"{len(getattr(self, 'frontier_history_series', []))},"
                    f"{fm_x},{fm_y},{fo_x},{fo_y},{fr_x},{fr_y}\n"
                )
            self.get_logger().info(f"📋 Session summary saved → {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save session summary: {e}")

    def _save_nav_goals_series(self):
        if not self.nav_goals_series:
            self.get_logger().warn('️ No nav goal data to save')
            return
        os.makedirs(self.map_save_dir, exist_ok=True)
        out_path = os.path.join(self.map_save_dir, getattr(self, 'nav_goals_file', 'auto_explore_nav_goals.csv'))
        try:
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('elapsed_s,goal_x_m,goal_y_m,frontier_x_m,frontier_y_m\n')
                for row in self.nav_goals_series:
                    f.write(f"{row[0]:.2f},{row[1]:.4f},{row[2]:.4f},{row[3]:.4f},{row[4]:.4f}\n")
            self.get_logger().info(
                f"🎯 Nav goals saved: {len(self.nav_goals_series)} goals → {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save nav goals: {e}")

    def _save_frontier_history_series(self):
        if not self.frontier_history_series:
            self.get_logger().warn('️ No frontier history data to save')
            return
        os.makedirs(self.map_save_dir, exist_ok=True)
        out_path = os.path.join(self.map_save_dir, getattr(self, 'frontier_history_file', 'auto_explore_frontiers.csv'))
        try:
            with open(out_path, 'w', encoding='ascii') as f:
                f.write('elapsed_s,frontier_x_m,frontier_y_m\n')
                for row in self.frontier_history_series:
                    f.write(f"{row[0]:.2f},{row[1]:.4f},{row[2]:.4f}\n")
            self.get_logger().info(
                f"🗺️ Frontier history saved: {len(self.frontier_history_series)} points → {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save frontier history: {e}")

    def _publish_goal_arrow(self, robot_x: float, robot_y: float, goal_x: float, goal_y: float):
        from geometry_msgs.msg import Point
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_goal_arrow'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        tail = Point()
        tail.x = float(robot_x)
        tail.y = float(robot_y)
        tail.z = 0.1
        head = Point()
        head.x = float(goal_x)
        head.y = float(goal_y)
        head.z = 0.1
        marker.points = [tail, head]
        marker.scale.x = 0.05                   
        marker.scale.y = 0.12                       
        marker.scale.z = 0.15                     
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 10                                
        self.goal_marker_pub.publish(marker)

    def _clear_goal_arrow(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_goal_arrow'
        marker.id = 0
        marker.action = Marker.DELETE
        self.goal_marker_pub.publish(marker)

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
        from .map_saver import save_occupancy_grid_map_async

        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            self.get_logger().warn('️ No map data received yet; skipping save')
            return

        def _on_success(result):
            self.get_logger().info(
                f"✅ Map saved: {result['pgm_path']} + {result['png_path']} "
                f"({result['width']}x{result['height']})"
            )
            # Ensure per-session diagnostics are refreshed on every map save.
            try:
                path_file = getattr(self, 'robot_path_file', 'auto_explore_path.csv')
                path_csv = os.path.join(self.map_save_dir, path_file)
                self._save_path_diagnostics_from_csv(path_csv)
            except Exception:
                pass

        def _on_error(err):
            self.get_logger().error(f'❌ Map save error: {err}')

        save_occupancy_grid_map_async(
            slam_map,
            self.map_save_dir,
            name_prefix='map',
            on_success=_on_success,
            on_error=_on_error,
        )

    def _map_save_done(self, future):
        try:
            result = future.result()
            if result is not None and result.result:
                self.get_logger().info('Map saved successfully')
            else:
                self.get_logger().warn('️ Map save failed')
        except Exception as e:
            self.get_logger().error(f'❌ Map save error: {e}')

    def _archive_exploration_outputs(self):
        import shutil
        import datetime as _dt
        # Gather all output files for this session
        files_to_copy = []
        map_dir = getattr(self, 'map_save_dir', 'saved_maps')
        # List of known output files
        file_attrs = [
            'mapped_area_file', 'robot_path_file', 'nav_goals_file', 'frontier_history_file', 'session_summary_file'
        ]
        for attr in file_attrs:
            fname = getattr(self, attr, None)
            if fname:
                fpath = os.path.join(map_dir, fname)
                if os.path.exists(fpath):
                    files_to_copy.append(fpath)
                if attr == 'robot_path_file':
                    stem, _ = os.path.splitext(fpath)
                    extra = [
                        f'{stem}_plot.png',
                        f'{stem}_plot_timecolor.png',
                        f'{stem}_plot_smoothed.png',
                        f'{stem}_plot_explored_vs_relative.png',
                        f'{stem}_relative.csv',
                        f'{stem}_raw_smoothed.csv',
                        f'{stem}_stats.csv',
                        f'{stem}_stats.json',
                        f'{stem}_stop_note.csv',
                        f'{stem}_stop_note.json',
                    ]
                    for ef in extra:
                        if os.path.exists(ef):
                            files_to_copy.append(ef)
        # Add map files if present
        for ext in ('.pgm', '.yaml', '.png'):
            for f in os.listdir(map_dir):
                if f.endswith(ext):
                    files_to_copy.append(os.path.join(map_dir, f))
        # Archive folder name with timestamp
        now = _dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        archive_dir = os.path.join(map_dir, f'exploration_results_{now}')
        try:
            os.makedirs(archive_dir, exist_ok=True)
            for f in files_to_copy:
                shutil.copy2(f, archive_dir)
            self.get_logger().info(f'📦 All session outputs saved in one folder: {archive_dir}')
            self.session_archive_dir = archive_dir
        except Exception as e:
            self.get_logger().error(f'❌ Failed to copy exploration outputs: {e}')

