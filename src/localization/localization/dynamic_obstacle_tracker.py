
#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class KalmanTrack:
    def __init__(self, track_id, x, y, now, process_noise, meas_noise):
        self.track_id = track_id
        self.state = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.eye(4, dtype=float)
        self.last_update = now
        self.process_noise = process_noise
        self.meas_noise = meas_noise

    def predict(self, dt):
        if dt <= 0.0:
            return
        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
        q = self.process_noise
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = q * np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2]
        ], dtype=float)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=float)
        R = self.meas_noise * np.eye(2, dtype=float)
        y = z - (H @ self.state)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + (K @ y)
        I = np.eye(4, dtype=float)
        self.P = (I - K @ H) @ self.P

    def predict_position(self, dt):
        x, y, vx, vy = self.state
        return x + vx * dt, y + vy * dt


class DynamicObstacleTracker(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_tracker')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/dynamic_obstacles')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('cluster_dist', 0.20)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 200)
        self.declare_parameter('assoc_distance', 0.60)
        self.declare_parameter('prediction_horizon', 0.6)
        self.declare_parameter('track_timeout', 1.0)
        self.declare_parameter('process_noise', 0.5)
        self.declare_parameter('meas_noise', 0.2)
        self.declare_parameter('publish_rate', 10.0)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.cluster_dist = float(self.get_parameter('cluster_dist').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.max_cluster_size = int(self.get_parameter('max_cluster_size').value)
        self.assoc_distance = float(self.get_parameter('assoc_distance').value)
        self.prediction_horizon = float(self.get_parameter('prediction_horizon').value)
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.process_noise = float(self.get_parameter('process_noise').value)
        self.meas_noise = float(self.get_parameter('meas_noise').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.tracks = []
        self.next_track_id = 1

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        period = 1.0 / max(1.0, publish_rate)
        self.create_timer(period, self.publish_predictions)

        self.get_logger().info('Dynamic obstacle tracker started')

    def scan_cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds / 1e9
        points = self._scan_to_points(msg)
        clusters = self._cluster_points(points)
        centroids = [self._cluster_centroid(c) for c in clusters]
        self._update_tracks(centroids, now)

    def _scan_to_points(self, msg: LaserScan):
        points = []
        angle = msg.angle_min
        for distance in msg.ranges:
            if distance < self.min_range or distance > min(self.max_range, msg.range_max):
                angle += msg.angle_increment
                continue
            x = math.cos(angle) * distance
            y = math.sin(angle) * distance
            points.append((x, y))
            angle += msg.angle_increment
        return points

    def _cluster_points(self, points):
        clusters = []
        if not points:
            return clusters
        current = [points[0]]
        for i in range(1, len(points)):
            px, py = points[i - 1]
            cx, cy = points[i]
            if math.hypot(cx - px, cy - py) <= self.cluster_dist:
                current.append(points[i])
            else:
                if self.min_cluster_size <= len(current) <= self.max_cluster_size:
                    clusters.append(current)
                current = [points[i]]
        if self.min_cluster_size <= len(current) <= self.max_cluster_size:
            clusters.append(current)
        return clusters

    def _cluster_centroid(self, cluster):
        sx = sum(p[0] for p in cluster)
        sy = sum(p[1] for p in cluster)
        n = len(cluster)
        return (sx / n, sy / n)

    def _update_tracks(self, centroids, now):
        for track in self.tracks:
            dt = now - track.last_update
            track.predict(max(0.0, dt))

        used_tracks = set()
        for cx, cy in centroids:
            best_track = None
            best_dist = self.assoc_distance
            for track in self.tracks:
                if track.track_id in used_tracks:
                    continue
                tx, ty = track.state[0], track.state[1]
                dist = math.hypot(cx - tx, cy - ty)
                if dist < best_dist:
                    best_dist = dist
                    best_track = track
            if best_track is not None:
                best_track.update(np.array([cx, cy], dtype=float))
                best_track.last_update = now
                used_tracks.add(best_track.track_id)
            else:
                track = KalmanTrack(self.next_track_id, cx, cy, now, self.process_noise, self.meas_noise)
                self.next_track_id += 1
                self.tracks.append(track)

        self.tracks = [t for t in self.tracks if (now - t.last_update) <= self.track_timeout]

    def publish_predictions(self):
        now = self.get_clock().now().nanoseconds / 1e9
        points = []
        for track in self.tracks:
            dt = max(0.0, now - track.last_update) + self.prediction_horizon
            px, py = track.predict_position(dt)
            if math.hypot(px, py) <= self.max_range:
                points.append((px, py, 0.0))

        header = self._make_header()
        cloud = point_cloud2.create_cloud(header, self._point_fields(), points)
        self.cloud_pub.publish(cloud)

    def _make_header(self):
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def _point_fields(self):
        return [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleTracker()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
