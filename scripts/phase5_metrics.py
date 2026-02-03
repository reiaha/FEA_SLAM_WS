#!/usr/bin/env python3
"""
Phase 5 Verification Metrics Tracker
Monitors exploration performance in real-time
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from action_msgs.msg import GoalStatusArray
import time
from datetime import datetime, timedelta

class Phase5Metrics(Node):
    def __init__(self):
        super().__init__('phase5_metrics')
        
        # Metrics
        self.start_time = time.time()
        self.frontier_count = 0
        self.last_frontier_update = time.time()
        self.goals_attempted = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.goals_aborted = 0
        self.map_coverage = 0.0
        self.last_map_cells = {'unknown': 0, 'free': 0, 'occupied': 0}
        
        # Subscribers
        self.frontiers_sub = self.create_subscription(
            PoseArray, '/frontiers', self.frontiers_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, 10)
        
        # Timer for periodic reporting
        self.report_timer = self.create_timer(10.0, self.report_metrics)
        
        self.get_logger().info('📊 Phase 5 Metrics Tracker Started')
        self.get_logger().info('   Monitoring: Frontiers, Map Coverage, Navigation Success')
        self.get_logger().info('   Updates every 10 seconds')
        
    def frontiers_callback(self, msg: PoseArray):
        """Track frontier detection"""
        self.frontier_count = len(msg.poses)
        self.last_frontier_update = time.time()
    
    def map_callback(self, msg: OccupancyGrid):
        """Calculate map coverage"""
        data = list(msg.data)
        total_cells = len(data)
        
        if total_cells == 0:
            return
        
        # Count cell types
        unknown = sum(1 for cell in data if cell == -1)
        free = sum(1 for cell in data if 0 <= cell < 50)
        occupied = sum(1 for cell in data if cell >= 50)
        
        self.last_map_cells = {
            'unknown': unknown,
            'free': free,
            'occupied': occupied,
            'total': total_cells
        }
        
        # Coverage = known area / total area
        known_cells = free + occupied
        self.map_coverage = (known_cells / total_cells * 100.0) if total_cells > 0 else 0.0
    
    def goal_status_callback(self, msg: GoalStatusArray):
        """Track navigation goal success/failure"""
        for status in msg.status_list:
            # Status codes: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status.status == 4:  # SUCCEEDED
                # Only count if this is a new success
                new_total = self.goals_succeeded + self.goals_failed + self.goals_aborted
                if new_total > self.goals_attempted - 1:
                    self.goals_succeeded += 1
                    self.goals_attempted += 1
            elif status.status in [5, 6]:  # CANCELED or ABORTED
                new_total = self.goals_succeeded + self.goals_failed + self.goals_aborted
                if new_total > self.goals_attempted - 1:
                    if status.status == 6:
                        self.goals_aborted += 1
                    else:
                        self.goals_failed += 1
                    self.goals_attempted += 1
    
    def report_metrics(self):
        """Print current metrics"""
        elapsed = time.time() - self.start_time
        elapsed_str = str(timedelta(seconds=int(elapsed)))
        
        # Calculate success rate
        success_rate = 0.0
        if self.goals_attempted > 0:
            success_rate = (self.goals_succeeded / self.goals_attempted) * 100.0
        
        self.get_logger().info('═' * 60)
        self.get_logger().info(f'📊 PHASE 5 METRICS REPORT - Elapsed: {elapsed_str}')
        self.get_logger().info('─' * 60)
        
        # Frontier Detection
        self.get_logger().info(f'🎯 Frontiers Detected: {self.frontier_count} clusters')
        frontier_age = time.time() - self.last_frontier_update
        if frontier_age < 5.0:
            self.get_logger().info('   ✅ Frontier detection active (recent update)')
        else:
            self.get_logger().info(f'   ⚠️  No frontier update for {frontier_age:.1f}s')
        
        # Navigation Success
        self.get_logger().info(f'🚀 Navigation Goals:')
        self.get_logger().info(f'   Total Attempted: {self.goals_attempted}')
        self.get_logger().info(f'   Succeeded: {self.goals_succeeded}')
        self.get_logger().info(f'   Failed/Canceled: {self.goals_failed}')
        self.get_logger().info(f'   Aborted: {self.goals_aborted}')
        if self.goals_attempted > 0:
            self.get_logger().info(f'   Success Rate: {success_rate:.1f}%')
            if success_rate >= 80:
                self.get_logger().info('   ✅ Excellent success rate!')
            elif success_rate >= 60:
                self.get_logger().info('   ✓ Good success rate')
            else:
                self.get_logger().info('   ⚠️  Low success rate - check obstacles/parameters')
        
        # Map Coverage
        self.get_logger().info(f'🗺️  Map Coverage: {self.map_coverage:.1f}%')
        if self.last_map_cells.get('total', 0) > 0:
            self.get_logger().info(f'   Free cells: {self.last_map_cells["free"]}')
            self.get_logger().info(f'   Occupied cells: {self.last_map_cells["occupied"]}')
            self.get_logger().info(f'   Unknown cells: {self.last_map_cells["unknown"]}')
        
        if self.map_coverage >= 85:
            self.get_logger().info('   ✅ High coverage achieved!')
        elif self.map_coverage >= 70:
            self.get_logger().info('   ✓ Good coverage')
        else:
            self.get_logger().info('   ⏳ Still exploring...')
        
        self.get_logger().info('═' * 60)
    
    def final_report(self):
        """Print final summary"""
        total_time = time.time() - self.start_time
        total_time_str = str(timedelta(seconds=int(total_time)))
        
        self.get_logger().info('')
        self.get_logger().info('🏁 ' + '═' * 58)
        self.get_logger().info('   PHASE 5 FINAL REPORT')
        self.get_logger().info('═' * 60)
        self.get_logger().info(f'⏱️  Total Exploration Time: {total_time_str}')
        self.get_logger().info(f'🗺️  Final Map Coverage: {self.map_coverage:.1f}%')
        
        if self.goals_attempted > 0:
            success_rate = (self.goals_succeeded / self.goals_attempted) * 100.0
            self.get_logger().info(f'🎯 Navigation Success Rate: {success_rate:.1f}%')
            self.get_logger().info(f'   ({self.goals_succeeded}/{self.goals_attempted} goals reached)')
        
        self.get_logger().info(f'🔍 Final Frontier Count: {self.frontier_count}')
        self.get_logger().info('═' * 60)
        
        # Overall assessment
        if self.map_coverage >= 80 and (self.goals_attempted == 0 or 
                                         (self.goals_succeeded / self.goals_attempted) >= 0.7):
            self.get_logger().info('✅ PHASE 5: PASSED - Excellent performance!')
        elif self.map_coverage >= 60:
            self.get_logger().info('✓ PHASE 5: GOOD - Acceptable performance')
        else:
            self.get_logger().info('⚠️  PHASE 5: NEEDS IMPROVEMENT - Check parameters')
        self.get_logger().info('═' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = Phase5Metrics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('⏹️  Stopping metrics tracker...')
        node.final_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
