#!/usr/bin/env python3

import math
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf2_ros


class FaceCluster:
    MAX_POSITIONS = 60

    def __init__(self, x, y, z, timestamp):
        self.positions = [(x, y, z)]
        self.published = False
        self.last_updated = timestamp

    def add(self, x, y, z, timestamp):
        self.positions.append((x, y, z))
        self.last_updated = timestamp
        if len(self.positions) > self.MAX_POSITIONS:
            self.positions.pop(0)

    def reset(self, x, y, z, timestamp):
        self.positions = [(x, y, z)]
        self.last_updated = timestamp

    @property
    def count(self):
        return len(self.positions)

    @property
    def centroid(self):
        xs = [p[0] for p in self.positions]
        ys = [p[1] for p in self.positions]
        zs = [p[2] for p in self.positions]
        return float(np.mean(xs)), float(np.mean(ys)), float(np.mean(zs))

    @property
    def xy_variance(self):
        if len(self.positions) < 2:
            return float('inf')
        xs = [p[0] for p in self.positions]
        ys = [p[1] for p in self.positions]
        return float(np.var(xs) + np.var(ys))

    def distance_to(self, x, y):
        cx, cy, _ = self.centroid
        return math.sqrt((cx - x) ** 2 + (cy - y) ** 2)


class FaceLocalizator(Node):
    MAX_DETECTION_DIST = 1.7  # m  — ignore detections further than this in camera space
    CLUSTER_TTL       = 20.0  # s  — reset a cluster if no new detection arrives within this time
    MATCH_RADIUS      = 0.4   # m  — max distance to assign a detection to an existing cluster
    OUTLIER_THRESHOLD = 0.3   # m  — reject detections this far from the cluster centroid
    OUTLIER_MIN_COUNT = 6     # wait for this many detections before applying outlier rejection #1st time was 8, 2nd time was 5
    MIN_DETECTIONS    = 9     # detections needed before a cluster can be published  #1st time was 9, 2nd was 7
    MAX_VARIANCE      = 0.7  # m² — cluster must be reasonably tight (≈26 cm std) before publishing
    DUPLICATE_RADIUS  = 0.5   # m  — min separation between published face markers
    TF_TIMEOUT        = 0.3   # s  — how long to wait for TF at the marker's exact timestamp

    def __init__(self):
        super().__init__('face_localizator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            Marker,
            '/people_marker',
            self.marker_callback,
            qos_profile_sensor_data
        )

        self.face_locations_pub = self.create_publisher(
            Marker,
            '/detected_face_locations',
            10
        )

        self.handled_face_sub = self.create_subscription(
            Point,
            '/handled_face_location',
            self.handled_face_callback,
            10
        )

        self.clusters = []
        self.published_locations = []  # list of (x, y, marker_id)
        self.marker_id_counter = 0

        self.get_logger().info('Face localizator initialized.')

    def marker_callback(self, marker_msg):
        frame_id = marker_msg.header.frame_id.lstrip('/')
        stamp = marker_msg.header.stamp

        # Use the marker's own timestamp so the TF matches when the image was captured,
        # not when this callback fires (robot may have moved since then over WiFi).
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                frame_id,
                stamp,
                timeout=rclpy.duration.Duration(seconds=self.TF_TIMEOUT)
            )
        except tf2_ros.TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    frame_id,
                    rclpy.time.Time()
                )
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'TF lookup failed: {ex}')
                return

        x_cam = marker_msg.pose.position.x
        y_cam = marker_msg.pose.position.y
        z_cam = marker_msg.pose.position.z

        cam_dist = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        if cam_dist > self.MAX_DETECTION_DIST:
            return

        x_map, y_map, z_map = self.transform_point(x_cam, y_cam, z_cam, transform)

        now = self.get_clock().now().nanoseconds / 1e9
        self.add_detection(x_map, y_map, z_map, now)

    def add_detection(self, x, y, z, now):
        # Drop detections near already-published/visited faces — prevents re-clustering visited spots
        for pub_x, pub_y, _ in self.published_locations:
            if math.sqrt((pub_x - x) ** 2 + (pub_y - y) ** 2) < self.DUPLICATE_RADIUS:
                return

        # Find the closest existing cluster within MATCH_RADIUS
        best_cluster = None
        best_dist = self.MATCH_RADIUS

        for cluster in self.clusters:
            d = cluster.distance_to(x, y)
            if d < best_dist:
                best_dist = d
                best_cluster = cluster

        if best_cluster is None:
            self.clusters.append(FaceCluster(x, y, z, now))
            return

        # Reset stale cluster — too much time passed since last detection, start fresh
        if (now - best_cluster.last_updated) > self.CLUSTER_TTL:
            best_cluster.reset(x, y, z, now)
            return

        # Once the cluster has enough points, reject outliers to protect the centroid
        if best_cluster.count >= self.OUTLIER_MIN_COUNT:
            if best_dist > self.OUTLIER_THRESHOLD:
                return

        best_cluster.add(x, y, z, now)

        # Publish once the cluster is large and tight enough
        if (not best_cluster.published
                and best_cluster.count >= self.MIN_DETECTIONS
                and best_cluster.xy_variance <= self.MAX_VARIANCE):

            cx, cy, _ = best_cluster.centroid

            # Suppress duplicates
            for pub_x, pub_y, _ in self.published_locations:
                if math.sqrt((pub_x - cx) ** 2 + (pub_y - cy) ** 2) < self.DUPLICATE_RADIUS:
                    best_cluster.published = True
                    return

            best_cluster.published = True
            marker_id = self.publish_marker(cx, cy)
            self.published_locations.append((cx, cy, marker_id))
            self.get_logger().info(
                f'Face confirmed at ({cx:.2f}, {cy:.2f}) '
                f'— {best_cluster.count} detections, variance={best_cluster.xy_variance:.4f} m²'
            )

    def publish_marker(self, x, y, r=0.0, g=1.0, b=0.0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.face_locations_pub.publish(marker)
        return marker.id

    def handled_face_callback(self, msg):
        """Re-publish the nearest face marker in grey to indicate it has been visited."""
        best = None
        best_dist = 0.8

        for pub_x, pub_y, marker_id in self.published_locations:
            d = math.sqrt((pub_x - msg.x) ** 2 + (pub_y - msg.y) ** 2)
            if d < best_dist:
                best_dist = d
                best = (pub_x, pub_y, marker_id)

        if best is None:
            return

        fx, fy, existing_id = best

        # Overwrite the existing marker in-place using its original ID
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.id = existing_id

        marker.pose.position.x = fx
        marker.pose.position.y = fy
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        self.face_locations_pub.publish(marker)
        self.get_logger().info(f'Face marker {existing_id} at ({fx:.2f}, {fy:.2f}) marked as visited.')

    def transform_point(self, x, y, z, transform):
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        R = self.quat_to_rotation_matrix(qx, qy, qz, qw)

        rotated = [
            R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z,
        ]

        return rotated[0] + tx, rotated[1] + ty, rotated[2] + tz

    def quat_to_rotation_matrix(self, qx, qy, qz, qw):
        return [
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)],
        ]


def main():
    rclpy.init(args=None)
    node = FaceLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
