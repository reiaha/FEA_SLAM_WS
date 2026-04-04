#!/usr/bin/env python3


import math
import time
from .phase_enum import Phase

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration

class ExplorationPlanningMixin:
    def pick_unknown_cell_goal(self):
        """Pick a reachable goal adjacent to unknown cells when no frontiers are available."""
        if not bool(getattr(self, 'enable_unknown_cell_seek', True)):
            return None

        self.update_pose()
        if not self.pose_valid:
            return None

        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            return None

        coverage = float(getattr(self, 'last_percent_known', 0.0) or 0.0)
        min_cov = float(getattr(self, 'unknown_cell_seek_min_coverage', 90.0))
        if coverage < min_cov:
            return None

        unknown_cells = int(getattr(self, 'last_unknown_cells', 0))
        min_unknown = int(getattr(self, 'unknown_cell_seek_min_unknown_cells', 20))
        if unknown_cells < min_unknown:
            return None

        try:
            info = slam_map.info
            resolution = float(info.resolution)
            if resolution <= 0.0:
                return None
            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            width = int(info.width)
            height = int(info.height)
            data = slam_map.data
            if width <= 2 or height <= 2 or not data:
                return None
        except Exception:
            return None

        robot_x, robot_y, _ = self.robot_pose
        stride = max(1, int(getattr(self, 'unknown_cell_seek_stride', 2)))
        max_occ = min(99, int(getattr(self, 'costmap_free_threshold', 100)))

        best = None
        best_dist = float('inf')
        examined = 0
        max_candidates = max(50, int(getattr(self, 'unknown_cell_seek_max_candidates', 1200)))

        for my in range(1, height - 1, stride):
            for mx in range(1, width - 1, stride):
                idx = my * width + mx
                if int(data[idx]) != -1:
                    continue

                # Select a neighboring known-free cell as the actual navigation goal.
                chosen = None
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx = mx + dx
                    ny = my + dy
                    nval = int(data[ny * width + nx])
                    if 0 <= nval < max_occ:
                        chosen = (nx, ny)
                        break
                if chosen is None:
                    continue

                gx = origin_x + (chosen[0] + 0.5) * resolution
                gy = origin_y + (chosen[1] + 0.5) * resolution
                ux = origin_x + (mx + 0.5) * resolution
                uy = origin_y + (my + 0.5) * resolution

                if not self._goal_in_free_space(gx, gy, self.use_costmap_goal_filter, allow_unknown=False):
                    continue
                if not self._goal_needs_unknown_support(gx, gy):
                    continue

                d = math.hypot(gx - robot_x, gy - robot_y)
                if d < 0.20:
                    continue
                if d < best_dist:
                    best_dist = d
                    best = {
                        'frontier': (ux, uy),
                        'goal': (gx, gy),
                        'dist': d,
                        'costmap_filtered': self.use_costmap_goal_filter,
                        'source': 'unknown_cell_seek',
                    }

                examined += 1
                if examined >= max_candidates:
                    break
            if examined >= max_candidates:
                break

        if best is not None:
            self.get_logger().warn(
                f"🧭 Unknown-cell seek: selected boundary goal ({best['goal'][0]:.2f}, {best['goal'][1]:.2f}) "
                f"toward unknown cell ({best['frontier'][0]:.2f}, {best['frontier'][1]:.2f}), dist {best['dist']:.2f}m"
            )
        return best

    def _dynamic_obstacle_pruning_active(self) -> bool:
        """Return True only when pruning should react to moving obstacles."""
        if not bool(getattr(self, 'prune_only_moving_obstacles', False)):
            return True

        if not bool(getattr(self, 'obstacle_detected', False)):
            return False

        obstacle_type = str(getattr(self, 'current_lidar_obstacle_type', 'unknown')).lower()
        if obstacle_type != 'dynamic':
            return False

        hold_sec = max(0.0, float(getattr(self, 'dynamic_obstacle_prune_hold_sec', 1.5)))
        last_obs = float(getattr(self, 'last_obstacle_time', 0.0))
        return (time.time() - last_obs) <= hold_sec

    def _frontier_is_valid(self, fx: float, fy: float) -> bool:
        """Return True only for finite, in-map, traversable frontier points near unknown space."""
        if not math.isfinite(fx) or not math.isfinite(fy):
            return False

        if not bool(getattr(self, 'exclude_invalid_frontiers', True)):
            return True

        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            return True

        try:
            info = slam_map.info
            resolution = float(info.resolution)
            if resolution <= 0.0:
                return False

            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            width = int(info.width)
            height = int(info.height)
            data = slam_map.data
            if width <= 0 or height <= 0 or not data:
                return False

            mx = int((fx - origin_x) / resolution)
            my = int((fy - origin_y) / resolution)
            if mx < 0 or my < 0 or mx >= width or my >= height:
                return False

            idx = my * width + mx
            cell = int(data[idx])
            if cell < 0:
                return False

            max_occ = int(getattr(self, 'invalid_frontier_max_occupancy', 70))
            if cell >= max_occ:
                return False

            unknown_neighbors = 0
            traversable_neighbors = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx = mx + dx
                    ny = my + dy
                    if nx < 0 or ny < 0 or nx >= width or ny >= height:
                        continue
                    nval = int(data[ny * width + nx])
                    if nval == -1:
                        unknown_neighbors += 1
                    elif 0 <= nval < max_occ:
                        traversable_neighbors += 1

            if unknown_neighbors <= 0:
                return False
            if traversable_neighbors <= 0:
                return False
            return True
        except Exception:
            return False

    def _frontier_is_already_scanned(self, robot_x: float, robot_y: float, fx: float, fy: float) -> bool:
        """Return True when frontier is already within effective lidar scan coverage."""
        if not bool(getattr(self, 'frontier_skip_if_within_scan_range', True)):
            return False

        lidar_range = max(0.3, float(getattr(self, 'frontier_lidar_range_m', 8.0)))
        lidar_margin = max(0.05, float(getattr(self, 'frontier_lidar_range_margin_m', 0.6)))
        effective_scan_range = max(0.2, lidar_range - lidar_margin)
        dist = math.hypot(fx - robot_x, fy - robot_y)
        if dist > effective_scan_range:
            return False

        # If map around this frontier has little unknown, treat it as already scanned.
        stats = self._frontier_unknown_stats(
            fx,
            fy,
            float(getattr(self, 'frontier_unknown_check_radius_m', 0.45))
        )
        if stats is None:
            return True

        max_unknown_ratio = float(getattr(self, 'frontier_scanned_max_unknown_ratio', 0.08))
        max_unknown_cells = int(getattr(self, 'frontier_scanned_max_unknown_cells', 8))
        return (
            stats['unknown_ratio'] <= max(0.0, max_unknown_ratio) or
            stats['unknown_cells'] <= max(0, max_unknown_cells)
        )

    def _compute_lidar_standoff_goal(self, robot_x: float, robot_y: float, fx: float, fy: float, goal_x: float, goal_y: float):
        """Place goal just beyond the frontier on the unknown side, while keeping the frontier in scan range."""
        if not bool(getattr(self, 'frontier_use_lidar_standoff_goal', True)):
            return (goal_x, goal_y)

        dist_rf = math.hypot(fx - robot_x, fy - robot_y)
        if dist_rf <= 0.10:
            return None

        ux = (fx - robot_x) / dist_rf
        uy = (fy - robot_y) / dist_rf

        lidar_range = max(0.3, float(getattr(self, 'frontier_lidar_range_m', 8.0)))
        lidar_margin = max(0.05, float(getattr(self, 'frontier_lidar_range_margin_m', 0.6)))
        max_frontier_goal = max(0.20, lidar_range - lidar_margin)

        standoff_min = max(0.20, float(getattr(self, 'frontier_goal_standoff_min_m', 1.0)))
        standoff_max = max(standoff_min, float(getattr(self, 'frontier_goal_standoff_max_m', 2.5)))

        current_standoff = math.hypot(fx - goal_x, fy - goal_y)
        desired_standoff = max(standoff_min, current_standoff)
        desired_standoff = min(desired_standoff, standoff_max)
        desired_standoff = min(desired_standoff, max_frontier_goal)

        max_possible = max(0.05, dist_rf - 0.10)
        standoff = min(desired_standoff, max_possible)
        if standoff <= 0.05:
            return None

        # Push the goal beyond the frontier so Nav2 heads toward unexplored space.
        # A small extra bias keeps the final target on the unknown side instead of the already mapped side.
        unknown_push = max(0.15, float(getattr(self, 'frontier_goal_unknown_push_m', 0.30)))
        gx = fx + (ux * max(standoff, unknown_push))
        gy = fy + (uy * max(standoff, unknown_push))
        return (gx, gy)

    def _frontier_unknown_stats(self, x: float, y: float, radius_m: float):
        """Return unknown-cell ratio around a frontier from the SLAM map."""
        slam_map = getattr(self, 'last_slam_map', None)
        if slam_map is None:
            return None

        try:
            info = slam_map.info
            resolution = float(info.resolution)
            if resolution <= 0.0:
                return None

            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            width = int(info.width)
            height = int(info.height)
            data = slam_map.data
            if width <= 0 or height <= 0 or not data:
                return None

            cx = int((x - origin_x) / resolution)
            cy = int((y - origin_y) / resolution)
            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                return None

            radius_cells = max(1, int(max(0.05, radius_m) / resolution))
            r2 = radius_cells * radius_cells

            unknown = 0
            known = 0
            for dy in range(-radius_cells, radius_cells + 1):
                ny = cy + dy
                if ny < 0 or ny >= height:
                    continue
                for dx in range(-radius_cells, radius_cells + 1):
                    if (dx * dx + dy * dy) > r2:
                        continue
                    nx = cx + dx
                    if nx < 0 or nx >= width:
                        continue
                    idx = ny * width + nx
                    val = data[idx]
                    if val == -1:
                        unknown += 1
                    elif 0 <= val <= 100:
                        known += 1

            total = unknown + known
            if total <= 0:
                return None

            return {
                'unknown_ratio': float(unknown) / float(total),
                'unknown_cells': unknown,
                'known_cells': known,
                'total_cells': total,
            }
        except Exception:
            return None

    def _frontier_needs_exploration(self, fx: float, fy: float, robot_x: float, robot_y: float) -> bool:
        """Reject frontiers that are already mapped inside effective lidar range."""
        if not bool(getattr(self, 'prune_mapped_frontiers', True)):
            return True

        lidar_range = float(getattr(self, 'frontier_lidar_range_m', 8.0))
        dist = math.hypot(fx - robot_x, fy - robot_y)
        if dist > max(0.1, lidar_range):
            return True

        radius_m = float(getattr(self, 'frontier_unknown_check_radius_m', 0.45))
        min_unknown_ratio = float(getattr(self, 'frontier_min_unknown_ratio', 0.10))
        min_unknown_cells = int(getattr(self, 'frontier_min_unknown_cells', 6))
        if getattr(self, 'last_slam_map', None) is None:
            return True
        stats = self._frontier_unknown_stats(fx, fy, radius_m)
        if stats is None:
            return False

        if stats['unknown_cells'] < max(1, min_unknown_cells):
            return False
        if stats['unknown_ratio'] < max(0.0, min_unknown_ratio):
            return False
        return True

    def _goal_needs_unknown_support(self, goal_x: float, goal_y: float) -> bool:
        """Reject goals that land entirely inside already known map space."""
        if not bool(getattr(self, 'prune_mapped_frontiers', True)):
            return True

        radius_m = float(getattr(self, 'frontier_unknown_check_radius_m', 0.45))
        min_unknown_ratio = float(getattr(self, 'frontier_min_unknown_ratio', 0.10))
        min_unknown_cells = int(getattr(self, 'frontier_min_unknown_cells', 6))

        stats = self._frontier_unknown_stats(goal_x, goal_y, radius_m)
        if stats is None:
            return False

        if stats['unknown_cells'] < max(1, min_unknown_cells):
            return False
        if stats['unknown_ratio'] < max(0.0, min_unknown_ratio):
            return False
        return True

    def pick_best_frontier(self):
        if not self.current_frontiers:
            self.last_frontier_skip_reason = "no_frontiers"
            return None

        self.update_pose()
        
        if not self.pose_valid:
            self.last_frontier_skip_reason = "pose_invalid"
            return None
            
        if self.pose_stale:
            now = time.time()
            if (now - self.last_origin_warn_time) >= self.origin_warn_interval:
                self.last_origin_warn_time = now
                self.get_logger().warn(
                    f"⚠️ Skipping frontier selection: TF pose stale (age > {self.pose_stale_timeout}s)."
                )
            self.last_frontier_skip_reason = "pose_stale"
            return None

        tf_recovery_hold = max(0.0, float(getattr(self, 'tf_recovery_hold_sec', 0.0)))
        tf_fresh_since = float(getattr(self, 'tf_fresh_since', 0.0))
        if tf_recovery_hold > 0.0 and tf_fresh_since > 0.0:
            since_fresh = time.time() - tf_fresh_since
            if since_fresh < tf_recovery_hold:
                now = time.time()
                if (now - getattr(self, 'last_tf_recovery_log_time', 0.0)) >= 2.0:
                    self.last_tf_recovery_log_time = now
                    self.get_logger().warn(
                        f"⏳ TF recovered; waiting {tf_recovery_hold - since_fresh:.2f}s before selecting next frontier."
                    )
                self.last_frontier_skip_reason = "pose_recovering"
                return None

        robot_x, robot_y, _ = self.robot_pose
        now = time.time()
        prune_active = self._dynamic_obstacle_pruning_active()

        frontier_pool = []
        invalid_frontier_count = 0
        pruned_mapped_count = 0
        pruned_scanned_count = 0
        for fx, fy in self.current_frontiers:
            if not self._frontier_is_valid(fx, fy):
                invalid_frontier_count += 1
                continue
            if prune_active and self._frontier_is_already_scanned(robot_x, robot_y, fx, fy):
                pruned_scanned_count += 1
                continue
            if (not prune_active) or self._frontier_needs_exploration(fx, fy, robot_x, robot_y):
                frontier_pool.append((fx, fy))
            else:
                pruned_mapped_count += 1

        if invalid_frontier_count > 0:
            log_interval = max(0.2, float(getattr(self, 'invalid_frontier_log_interval', 2.0)))
            if (now - float(getattr(self, 'last_invalid_frontier_log_time', 0.0))) >= log_interval:
                self.last_invalid_frontier_log_time = now
                self.get_logger().warn(
                    f"🚫 Excluded {invalid_frontier_count} invalid frontier(s) before goal selection"
                )

        if (pruned_mapped_count > 0) or (pruned_scanned_count > 0):
            self.current_frontiers = frontier_pool
            log_interval = float(getattr(self, 'frontier_prune_log_interval', 2.0))
            last_log_time = float(getattr(self, 'last_frontier_prune_log_time', 0.0))
            if (now - last_log_time) >= max(0.2, log_interval):
                self.last_frontier_prune_log_time = now
                if prune_active:
                    self.get_logger().info(
                        f"🧹 Pruned mapped={pruned_mapped_count}, scanned={pruned_scanned_count} frontier(s); selecting only not-yet-scanned frontiers"
                    )
                else:
                    self.get_logger().info(
                        "🧹 Frontier pruning paused (dynamic obstacle not active); keeping static frontiers"
                    )

        if not frontier_pool:
            self.last_frontier_skip_reason = "no_unexplored_frontiers"
            return None

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
                'already_mapped': 0,
                'already_scanned': 0,
                'strict_revisit': 0,
                'strict_attempted': 0,
                'costmap_or_free_space': 0,
                'out_of_map': 0,
                'no_free_neighbor': 0,
                'blacklisted': 0,
                'visited_goal': 0,
                'near_last_success': 0,
                'near_last_goal': 0,
                'recent_goal': 0,
            }

            def build_candidates(costmap_filter: bool):
                built = []
                for fx, fy in frontier_pool:
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

                    standoff_goal = self._compute_lidar_standoff_goal(robot_x, robot_y, fx, fy, goal_x, goal_y)
                    if standoff_goal is None:
                        continue
                    goal_x, goal_y = standoff_goal

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

                    if not self._goal_in_free_space(goal_x, goal_y, costmap_filter, allow_unknown=True):
                        rejection_counts['costmap_or_free_space'] += 1
                        continue

                    if not self._goal_needs_unknown_support(goal_x, goal_y):
                        rejection_counts['already_mapped'] += 1
                        continue

                    if costmap_filter and self.costmap is not None:
                        costmap_info = self._get_costmap_info()
                        if costmap_info is not None:
                            gc = self._world_to_costmap(goal_x, goal_y, costmap_info)
                            if gc is None:
                                rejection_counts['out_of_map'] += 1
                                continue
                            _, _, _, width, height, _ = costmap_info
                            gx_c, gy_c = gc
                            has_free_neighbor = False
                            for ddx, ddy in ((1,0),(-1,0),(0,1),(0,-1)):
                                nx2, ny2 = gx_c + ddx, gy_c + ddy
                                if 0 <= nx2 < width and 0 <= ny2 < height:
                                    if self._is_costmap_free(nx2, ny2, costmap_info):
                                        has_free_neighbor = True
                                        break
                            if not has_free_neighbor:
                                rejection_counts['no_free_neighbor'] += 1
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

        if (
            best_frontier is None and
            frontier_pool and
            self.goals_reached < self.min_goals_for_complete and
            bool(getattr(self, 'enable_bootstrap_frontier_fallback', True))
        ):
            fallback = None
            fallback_dist = float('inf')
            for fx, fy in frontier_pool:
                dist = math.hypot(fx - robot_x, fy - robot_y)
                if dist < max(self.bootstrap_fallback_min_distance, self.relaxed_min_frontier_distance):
                    continue
                goal_x = fx
                goal_y = fy
                standoff_goal = self._compute_lidar_standoff_goal(robot_x, robot_y, fx, fy, goal_x, goal_y)
                if standoff_goal is None:
                    continue
                goal_x, goal_y = standoff_goal
                if self.strict_no_revisit and self.strict_avoid_goals:
                    strict_radius = max(
                        self.visited_goal_radius,
                        self.recent_goal_radius,
                        self.avoid_return_radius,
                        self.avoid_last_goal_radius,
                        self.known_frontier_avoid_radius,
                    )
                    if any(math.hypot(fx - sx, fy - sy) <= strict_radius for sx, sy in self.strict_avoid_goals):
                        continue
                if self.strict_no_revisit and self.attempted_frontiers:
                    strict_radius = max(
                        self.visited_goal_radius,
                        self.recent_goal_radius,
                        self.avoid_return_radius,
                        self.avoid_last_goal_radius,
                        self.known_frontier_avoid_radius,
                    )
                    if any(math.hypot(fx - sx, fy - sy) <= strict_radius for sx, sy in self.attempted_frontiers):
                        continue
                if self.blacklisted_goals:
                    skip = any(
                        math.hypot(goal_x - bx, goal_y - by) <= self.blacklist_radius
                        for (bx, by), _ in self.blacklisted_goals.items()
                    )
                    if skip:
                        continue
                if self.avoid_revisit and self.last_successful_goal_pos is not None:
                    if math.hypot(goal_x - self.last_successful_goal_pos[0], goal_y - self.last_successful_goal_pos[1]) <= self.avoid_return_radius:
                        continue
                if self.avoid_revisit and self.last_goal_target is not None:
                    if math.hypot(goal_x - self.last_goal_target[0], goal_y - self.last_goal_target[1]) <= self.avoid_last_goal_radius:
                        continue
                if self.recent_goals:
                    skip = any(
                        math.hypot(goal_x - rx, goal_y - ry) <= self.recent_goal_radius
                        for rx, ry, _ in self.recent_goals
                    )
                    if skip:
                        continue
                if not self._goal_in_free_space(goal_x, goal_y, self.use_costmap_goal_filter, allow_unknown=True):
                    continue
                if not self._goal_needs_unknown_support(goal_x, goal_y):
                    continue
                goal_dist = math.hypot(goal_x - robot_x, goal_y - robot_y)
                if goal_dist < fallback_dist:
                    fallback_dist = goal_dist
                    fallback = {
                        'frontier': (fx, fy),
                        'goal': (goal_x, goal_y),
                        'dist': goal_dist,
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
        """Send goal to Nav2 navigate_to_pose with proper TF timeout handling and fallback logic"""
        now = time.time()
        timeout = Duration(seconds=0.1)                                   
        if use_costmap_filter is None:
            use_costmap_filter = self.use_costmap_goal_filter

        # Force-send mode: if > 5 consecutive failures, bypass some strict checks
        force_send_mode = (self.consecutive_failures >= 5 and len(self.current_frontiers) > 0)
        if force_send_mode and not hasattr(self, '_force_send_log_time'):
            self._force_send_log_time = now
        if force_send_mode and (now - getattr(self, '_force_send_log_time', 0)) > 10.0:
            self.get_logger().warn(
                f"⚠️ Force-send mode: {self.consecutive_failures} failures — bypassing strict checks to keep exploring"
            )
            self._force_send_log_time = now

        # In force-send mode, skip immediate obstacle check
        if (self.obstacle_detected or self.front_obstacle_detected) and not force_send_mode:
            self.get_logger().warn("⚠️ Skipping goal send: obstacle currently detected")
            return False

        scan_age = now - self.last_scan_time
        current_phase = getattr(self, 'current_phase', None)
        is_init_phase = (current_phase is not None and current_phase.__class__.__name__ == 'Phase' and 
                         str(current_phase).split('.')[-1] == 'INIT')
        
        # During INIT, warn about stale scan but don't block goal sending (startup costmap may be settling)
        # In force-send mode, also skip stale scan check
        if scan_age > self.lidar_stale_timeout and not is_init_phase and not force_send_mode:
            if (now - self.last_scan_stale_warn_time) > 5.0:
                self.get_logger().warn(
                    f"⚠️ Scan stale ({scan_age:.2f}s > {self.lidar_stale_timeout:.2f}s) — may affect path planning"
                )
                self.last_scan_stale_warn_time = now
            return False
        elif scan_age > self.lidar_stale_timeout and is_init_phase:
            if (now - self.last_scan_stale_warn_time) > 5.0:
                self.get_logger().warn(
                    f"⚠️ Startup: scan stale ({scan_age:.2f}s); proceeding with first frontier anyway"
                )
                self.last_scan_stale_warn_time = now

        # Front clearance check: during INIT allow goal send even if slightly close
        # In force-send mode, also skip front clearance check
        front_clearance = self.last_lidar_front_distance
        if front_clearance is not None and front_clearance <= (self.lidar_obstacle_distance + 0.04) and not is_init_phase and not force_send_mode:
            if (now - self.last_front_clearance_warn_time) > 5.0:
                self.get_logger().warn(
                    f"⚠️ Front clearance low ({front_clearance:.2f}m); waiting for clearer path"
                )
                self.last_front_clearance_warn_time = now
            return False
        
        try:
            can_map_base = self.tf_buffer.can_transform('map', self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            if not can_map_base:
                return False
        except Exception:
            return False  # Silent fail for speed

        if not self._goal_in_free_space(goal_x, goal_y, use_costmap_filter, allow_unknown=True):
            self.get_logger().warn(f"⚠️ Goal ({goal_x:.2f}, {goal_y:.2f}) no longer in free space - skipping")
            self._blacklist_goal((goal_x, goal_y))
            return False

        # Optionally defer lethal start-cell checks until first success.
        # On jittery systems this can hide persistent start-cell blockage and create goal abort loops,
        # so make this behavior configurable from launch.
        defer_lethal = bool(getattr(self, 'defer_lethal_block_until_first_goal', False))
        lethal_logic_enabled = (self.goals_reached >= 1) if defer_lethal else True
        if use_costmap_filter and not is_init_phase and lethal_logic_enabled:
            self.update_pose()
            if self.pose_valid:
                costmap_info = self._get_costmap_info()
                if costmap_info is not None:
                    robot_cell = self._world_to_costmap(self.robot_pose[0], self.robot_pose[1], costmap_info)
                    robot_cell_free = False
                    if robot_cell is not None:
                        robot_cell_free = self._is_costmap_free(robot_cell[0], robot_cell[1], costmap_info)
                    if not robot_cell_free:
                        if (now - self.last_lethal_cell_warn_time) > 5.0:
                            self.get_logger().warn(
                                f"⚠️ Robot in lethal/occupied costmap cell at ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f}); clearing costmap"
                            )
                            self.last_lethal_cell_warn_time = now
                        self._clear_costmaps('start_cell_lethal', clear_global=True)
                        return False
        elif use_costmap_filter and not is_init_phase and (not lethal_logic_enabled):
            if (now - self.last_lethal_cell_warn_time) > 5.0:
                self.last_lethal_cell_warn_time = now
                self.get_logger().info("🟡 Lethal-cell blocking is disabled until first goal is reached")

        nav2_server_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        nav2_services_ready = self._nav2_services_ready()
        nav2_lifecycle_active = self._nav2_active(require_active=self.require_nav2_active, services_ready=nav2_services_ready)
        nav2_ready = nav2_server_ready and (nav2_lifecycle_active if self.require_nav2_active else True)
        if self.require_nav2_active and not nav2_ready:
            self.get_logger().warn("⚠️ Skipping goal send: Nav2 not active")
            return False

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

        # Set orientation to face the goal from the robot's current position
        if hasattr(self, 'robot_pose') and self.robot_pose is not None:
            dx = goal_x - self.robot_pose[0]
            dy = goal_y - self.robot_pose[1]
            yaw = math.atan2(dy, dx)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw
        else:
            goal_msg.pose.pose.orientation.w = 1.0

        self.last_goal_target = (goal_x, goal_y)
        _elapsed = time.time() - self.startup_time
        _fx, _fy = frontier_xy if frontier_xy is not None else (goal_x, goal_y)
        self.nav_goals_series.append((_elapsed, goal_x, goal_y, _fx, _fy))
        self.recent_goals.append((goal_x, goal_y, time.time() + self.recent_goal_hold_time))

        self.goal_in_progress = True                                                   
        
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
        self.last_goal_dispatch_time = time.time()
        self.goal_dispatch_pose = (robot_x, robot_y)
        self._publish_goal_arrow(robot_x, robot_y, goal_x, goal_y)

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

        bt_state = self._get_lifecycle_state(self.bt_state_client, 'bt_navigator')
        controller_state = self._get_lifecycle_state(self.controller_state_client, 'controller_server')
        planner_state = self._get_lifecycle_state(self.planner_state_client, 'planner_server')

        active = (bt_state == 'active' and controller_state == 'active' and planner_state == 'active')
        fallback_states = {'unknown', 'error'}
        all_states_uncertain = (
            bt_state in fallback_states and
            controller_state in fallback_states and
            planner_state in fallback_states
        )
        bt_only_uncertain = (
            bt_state in fallback_states and
            controller_state == 'active' and
            planner_state == 'active'
        )
        mixed_uncertain_with_active = (
            (bt_state in fallback_states or controller_state in fallback_states or planner_state in fallback_states) and
            (bt_state == 'active' or controller_state == 'active' or planner_state == 'active')
        )
        if require_active and (not active) and self.nav2_active_fallback_on_action and (
            all_states_uncertain or bt_only_uncertain or mixed_uncertain_with_active
        ):
            if (now - self.startup_time) >= self.nav2_active_grace_sec:
                if self.nav_client.wait_for_server(timeout_sec=0.05):
                    if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                        self.last_tf_check_log_time = now
                        if bt_only_uncertain:
                            self.get_logger().warn(
                                "⚠️ bt_navigator lifecycle state uncertain while planner/controller are active; using action-server readiness fallback"
                            )
                        elif mixed_uncertain_with_active:
                            self.get_logger().warn(
                                f"⚠️ Nav2 mixed lifecycle states (bt={bt_state}, controller={controller_state}, planner={planner_state}); using action-server readiness fallback"
                            )
                        else:
                            self.get_logger().warn(
                                "⚠️ Nav2 lifecycle states uncertain; using action-server readiness fallback"
                            )
                    return True
        if require_active and not active:
            if (now - self.last_tf_check_log_time) > self.tf_check_log_interval:
                self.last_tf_check_log_time = now
                self.get_logger().warn(
                    f"⚠️ Nav2 not active: bt={bt_state}, controller={controller_state}, planner={planner_state}"
                )
        return active if require_active else True

    def _get_lifecycle_state(self, client, key: str = 'unknown'):
        """Non-blocking lifecycle state poll safe to call from timer callbacks."""
        try:
            cache = getattr(self, '_nav2_lifecycle_state_cache', {})
            futures = getattr(self, '_nav2_lifecycle_state_futures', {})
            last_req = getattr(self, '_nav2_lifecycle_state_last_req', {})
            now = time.time()

            fut = futures.get(key)
            if fut is not None and fut.done():
                try:
                    res = fut.result()
                    if res is None or res.current_state is None:
                        cache[key] = 'unknown'
                    else:
                        cache[key] = str(res.current_state.label).lower()
                except Exception:
                    cache[key] = 'error'
                futures[key] = None

            if futures.get(key) is None and client.service_is_ready():
                if (now - float(last_req.get(key, 0.0))) >= 0.25:
                    try:
                        futures[key] = client.call_async(GetState.Request())
                        last_req[key] = now
                    except Exception:
                        cache[key] = 'error'

            self._nav2_lifecycle_state_cache = cache
            self._nav2_lifecycle_state_futures = futures
            self._nav2_lifecycle_state_last_req = last_req
            return str(cache.get(key, 'unknown')).lower()
        except Exception:
            return 'error'
    
    def goal_response_cb(self, future):
        """Handle Nav2 goal response"""
        Phase = self.current_phase.__class__
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info("✅ Goal accepted by Nav2")
            self.nav2_ready = True
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_cb)
        else:
            self.get_logger().warn("❌ Goal rejected by Nav2")
            self.goal_handle = None
            self.goal_in_progress = False
            self.frontiers_dirty = True
            self.last_goal_time = time.time()
            self.consecutive_failures += 1

            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)
                if self.strict_no_revisit:
                    self.strict_avoid_goals.append(self.last_goal_target)

            self.get_logger().warn(
                f"🔁 Goal REJECTED — treating as planning failure #{self.consecutive_failures}/{self.max_consecutive_failures}"
            )

            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().error(
                    f"🚨 STUCK DETECTED! {self.consecutive_failures} consecutive failures — entering RECOVERY mode"
                )
                self.current_phase = Phase.RECOVERY
                self.stuck_recovery_in_progress = True
                self.recovery_start_time = time.time()
            else:
                self._clear_costmaps('goal_rejected', clear_global=True)
                self.update_pose()
                if self.pose_valid and getattr(self, 'current_phase', None) != Phase.INIT:
                    pass

    def _maintain_active_goal(self, now: float) -> bool:
        """Return True when a goal is still active or still being processed."""
        if not (self.goal_handle is not None or self.goal_in_progress):
            return False

        if self.goal_in_progress and self.goal_handle is None:
            if self.last_goal_dispatch_time > 0.0 and (now - self.last_goal_dispatch_time) >= self.goal_ack_timeout:
                self.goal_in_progress = False
                self.frontiers_dirty = True
                self.last_goal_time = 0.0
                self.get_logger().warn(
                    f"⚠️ Nav2 goal ack timeout ({self.goal_ack_timeout:.1f}s); retrying frontier dispatch"
                )
                return False
            return True

        if self.goal_handle is None:
            return self.goal_in_progress

        nav2_services_ready = self._nav2_services_ready()
        nav2_healthy = self._nav2_active(
            require_active=self.require_nav2_active,
            services_ready=nav2_services_ready
        )
        if self.require_nav2_active and not nav2_healthy:
            return True

        if self.goal_watchdog_timeout <= 0.0:
            return True

        if self.last_goal_dispatch_time <= 0.0:
            self.last_goal_dispatch_time = now
            return True

        if (now - self.last_goal_dispatch_time) < self.goal_watchdog_timeout:
            return True

        self.update_pose()
        if self.pose_valid and self.goal_dispatch_pose is not None:
            moved = math.hypot(
                self.robot_pose[0] - self.goal_dispatch_pose[0],
                self.robot_pose[1] - self.goal_dispatch_pose[1]
            )
            if moved >= self.goal_watchdog_min_motion:
                self.goal_dispatch_pose = (self.robot_pose[0], self.robot_pose[1])
                self.last_goal_dispatch_time = now
                return True

        if self.last_goal_target is not None:
            self._blacklist_goal(self.last_goal_target)
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as e:
            if (now - self.last_goal_watchdog_log_time) >= 2.0:
                self.last_goal_watchdog_log_time = now
                self.get_logger().warn(f"⚠️ Goal watchdog cancel failed: {e}")
            return True

        self.goal_handle = None
        self.goal_in_progress = False
        self.frontiers_dirty = True
        self.last_goal_time = 0.0
        if (now - self.last_goal_watchdog_log_time) >= 2.0:
            self.last_goal_watchdog_log_time = now
            self.get_logger().warn(
                f"🔁 Goal watchdog: preempted stale goal after {self.goal_watchdog_timeout:.1f}s without motion"
            )
        return False
    
    def goal_result_cb(self, future):
        """Handle Nav2 goal completion"""
        Phase = self.current_phase.__class__
        if self.current_phase == Phase.DONE:
            self.goal_handle = None
            self.goal_in_progress = False
            return
        result = future.result()
        self.goal_handle = None                                                
        self.frontiers_dirty = True                                                          
        self.goal_in_progress = False                                 
        self._clear_goal_arrow()
        
        self.update_pose()
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        
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

        near_goal_acceptance = max(0.0, float(getattr(self, 'near_goal_acceptance_distance', 0.0)))
        goal_error = None
        if self.last_goal_target is not None:
            goal_error = math.hypot(
                robot_x - self.last_goal_target[0],
                robot_y - self.last_goal_target[1]
            )

        if (
            result.status in (5, 6)
            and near_goal_acceptance > 0.0
            and goal_error is not None
            and goal_error <= near_goal_acceptance
            and not self.obstacle_detected
        ):
            self.get_logger().info(
                f"✅ Near-goal acceptance: treating {('ABORTED' if result.status == 5 else 'CANCELED')} as success "
                f"(goal error {goal_error:.2f}m <= {near_goal_acceptance:.2f}m)"
            )
            self.post_abort_cooldown_until = 0.0
            self.consecutive_failures = 0
            self.last_successful_goal_pos = (robot_x, robot_y)
            self.goals_reached += 1
            self.get_logger().info(f"📊 Goals reached: {self.goals_reached}/{self.min_goals_for_complete}")
            if self.goals_reached == 1 and hasattr(self, 'enable_lethal_escape'):
                self.enable_lethal_escape()
                self.get_logger().info("✅ First goal completed: lethal logic is now enabled")
            if self.avoid_revisit:
                self.visited_goals.append((robot_x, robot_y))
            if self.strict_no_revisit and self.last_goal_target is not None:
                self.strict_avoid_goals.append(self.last_goal_target)
            return

        if result.status == 4:             
            self.get_logger().info(f"✅ Reached frontier goal! Robot at ({robot_x:.2f}, {robot_y:.2f})")
            self.post_abort_cooldown_until = 0.0
            self.consecutive_failures = 0                                    
            self.last_successful_goal_pos = (robot_x, robot_y)
            self.goals_reached += 1                                               
            self.get_logger().info(f"📊 Goals reached: {self.goals_reached}/{self.min_goals_for_complete}")
            if self.goals_reached == 1 and hasattr(self, 'enable_lethal_escape'):
                self.enable_lethal_escape()
                self.get_logger().info("✅ First goal completed: lethal logic is now enabled")
            if self.avoid_revisit:
                self.visited_goals.append((robot_x, robot_y))
            if self.strict_no_revisit and self.last_goal_target is not None:
                self.strict_avoid_goals.append(self.last_goal_target)
        elif result.status == 5:           
            self.post_abort_cooldown_until = max(
                getattr(self, 'post_abort_cooldown_until', 0.0),
                time.time() + max(0.0, float(getattr(self, 'post_abort_goal_cooldown_sec', 0.0)))
            )
            self.consecutive_failures += 1
            self.get_logger().error(f"❌ Goal ABORTED! Failure #{self.consecutive_failures}/{self.max_consecutive_failures} at pos ({robot_x:.2f}, {robot_y:.2f})")
            self.get_logger().error(f"   Likely cause: No valid path found by Nav2 planner")

            # Check for lethal-space planner failures only after at least one successful goal.
            # Before first goal, this escape can overreact to startup/planner transients.
            if self.goals_reached >= 1 and hasattr(self, '_maybe_lethal_escape'):
                escaped = self._maybe_lethal_escape(robot_x, robot_y)
                if escaped:
                    self.get_logger().error("⚠️ LETHAL SPACE DETECTED — executing immediate escape sequence")
                    return  # Skip normal failure handling, let lethal escape take control

            if self.last_goal_target is not None:
                self._blacklist_goal(self.last_goal_target)
                if self.strict_no_revisit:
                    self.strict_avoid_goals.append(self.last_goal_target)
            
            if self.obstacle_detected:
                # Only log obstacle warning if not recently logged
                now = time.time()
                if not hasattr(self, '_last_obstacle_abort_warn') or (now - getattr(self, '_last_obstacle_abort_warn', 0)) > 10.0:
                    self.get_logger().warn(f"🚧 Goal aborted AND obstacle detected at {self.obstacle_distance_m:.3f}m - entering OBSTACLE handling")
                    self._last_obstacle_abort_warn = now
                self.current_phase = Phase.OBSTACLE
                self.phase_start_time = time.time()
            else:
                # Suppress repeated path-clear abort warnings
                now = time.time()
                if not hasattr(self, '_last_path_clear_abort_warn') or (now - getattr(self, '_last_path_clear_abort_warn', 0)) > 10.0:
                    self.get_logger().warn(f"🔄 Goal aborted but path clear - will try new frontier immediately")
                    self._last_path_clear_abort_warn = now
                self.last_goal_time = 0.0
            
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().error(f"🚨 STUCK DETECTED! {self.consecutive_failures} consecutive failures - entering RECOVERY mode")
                self.current_phase = Phase.RECOVERY
                self.stuck_recovery_in_progress = True
                self.recovery_start_time = time.time()
            else:
                self._clear_costmaps('aborted_planning_failure', clear_global=True)
                if getattr(self, 'current_phase', None) != Phase.INIT:
                    pass
        elif result.status == 6:            
            self.get_logger().warn(f"⚠️ Navigation canceled at ({robot_x:.2f}, {robot_y:.2f})")
            self.post_abort_cooldown_until = max(
                getattr(self, 'post_abort_cooldown_until', 0.0),
                time.time() + max(0.0, float(getattr(self, 'post_abort_goal_cooldown_sec', 0.0)))
            )
            if self.last_goal_target is not None and self.strict_no_revisit:
                self.strict_avoid_goals.append(self.last_goal_target)
            if self.obstacle_detected:
                self.get_logger().warn(f"🚧 Goal canceled with obstacle present - will wait for obstacle handling")
            else:
                self.consecutive_failures += 1
                self.get_logger().warn(
                    f"🔄 Goal canceled (no obstacle) — treating as planning failure "
                    f"#{self.consecutive_failures}/{self.max_consecutive_failures}"
                )
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
                    self._clear_costmaps('canceled_planning_failure', clear_global=True)
                    if getattr(self, 'current_phase', None) != Phase.INIT:
                        pass
        else:
            self.get_logger().warn(f"⚠️ Navigation ended with status: {result.status} at ({robot_x:.2f}, {robot_y:.2f})")

    def _blacklist_goal(self, goal_xy):
        now = time.time()
        self.blacklisted_goals[goal_xy] = now + self.blacklist_duration
        if not hasattr(self, '_last_blacklist_goal_warn') or (now - getattr(self, '_last_blacklist_goal_warn', 0)) > 10.0:
            self.get_logger().warn(
                f"⚠️ Blacklisting failed goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) for {self.blacklist_duration:.0f}s"
            )
            self._last_blacklist_goal_warn = now

    def _clear_costmaps(self, reason: str, clear_global: bool = False):
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
        if clear_global and self.clear_global_costmap_client.service_is_ready():
            try:
                self.clear_global_costmap_client.call_async(req)
                self.get_logger().warn(f"🧹 Requested global costmap clear ({reason})")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Global costmap clear failed ({reason}): {e}")

    def _send_fallback_goal(self) -> bool:
        """Send a fallback goal when no frontiers are available.
        Tries to send goal 1-2m ahead in direction robot is facing."""
        if not self.pose_valid:
            self.update_pose()
        if not self.pose_valid:
            return False
        
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        # Try to move forward 1.5m in the robot's general forward direction
        fallback_goal_x = robot_x + 1.5
        fallback_goal_y = robot_y
        
        # Check if goal is in free space
        if not self._goal_in_free_space(fallback_goal_x, fallback_goal_y, use_costmap_filter=True, allow_unknown=False):
            # Try left diagonal
            fallback_goal_x = robot_x + 1.0
            fallback_goal_y = robot_y + 1.0
            if not self._goal_in_free_space(fallback_goal_x, fallback_goal_y, use_costmap_filter=True, allow_unknown=False):
                # Try right diagonal
                fallback_goal_x = robot_x + 1.0
                fallback_goal_y = robot_y - 1.0
                if not self._goal_in_free_space(fallback_goal_x, fallback_goal_y, use_costmap_filter=True, allow_unknown=False):
                    return False
        
        self.get_logger().warn(
            f"📍 No frontiers available; sending fallback goal at ({fallback_goal_x:.2f}, {fallback_goal_y:.2f})"
        )
        return self.send_goal_to_nav2(fallback_goal_x, fallback_goal_y, use_costmap_filter=True, frontier_xy=None)

