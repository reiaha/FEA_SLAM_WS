#!/usr/bin/env python3

import os
import time
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class ExplorationPersistenceMixin:
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
        self.get_logger().info("✅ EXPLORATION COMPLETE - Saving map and outputs...")
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

    def _flush_csv_data(self):
        if getattr(self, '_csv_flushed', False):
            return
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
                        'coverage_pct,goals_reached,path_points,nav_goals,frontier_points\n')
                f.write(
                    f"{status},"
                    f"{start_dt.strftime('%Y-%m-%d %H:%M:%S')},"
                    f"{now_dt.strftime('%Y-%m-%d %H:%M:%S')},"
                    f"{elapsed:.1f},"
                    f"{getattr(self, 'last_percent_known', 0.0):.2f},"
                    f"{getattr(self, 'goals_reached', 0)},"
                    f"{len(getattr(self, 'robot_path_series', []))},"
                    f"{len(getattr(self, 'nav_goals_series', []))},"
                    f"{len(getattr(self, 'frontier_history_series', []))}\n"
                )
            self.get_logger().info(f"📋 Session summary saved → {out_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save session summary: {e}")

    def _save_nav_goals_series(self):
        if not self.nav_goals_series:
            self.get_logger().warn('⚠️ No nav goal data to save')
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
            self.get_logger().warn('⚠️ No frontier history data to save')
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
            self.get_logger().warn('⚠️ No map data received yet; skipping save')
            return

        def _on_success(result):
            self.get_logger().info(
                f"✅ Map saved: {result['pgm_path']} + {result['png_path']} "
                f"({result['width']}x{result['height']})"
            )

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
                self.get_logger().info('✅ Map saved successfully')
            else:
                self.get_logger().warn('⚠️ Map save failed')
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

