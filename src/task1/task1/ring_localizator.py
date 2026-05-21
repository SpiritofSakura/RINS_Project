#!/usr/bin/env python3
"""
Ring localizator — accumulates per-frame detections, clusters them in map
frame, and publishes a persistent coloured marker once a location is confirmed.
Logic mirrors face_localizator: outlier rejection, variance gate, TTL reset,
timestamp-based TF lookup, and visited-marker greying.
"""

import math
from collections import Counter

import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf2_ros

# ── Tuning ────────────────────────────────────────────────────────────────────
MAX_DETECTION_DIST  = 3.0   # m  — ignore camera-space detections further than this
CLUSTER_TTL         = 20.0  # s  — reset a cluster if no detection arrives within TTL
MATCH_RADIUS        = 0.7   # m  — max distance to assign detection to existing cluster
OUTLIER_THRESHOLD   = 0.4   # m  — reject points this far from centroid (once enough samples)
OUTLIER_MIN_COUNT   = 6     # wait for N detections before applying outlier rejection
MIN_DETECTIONS      = 6     # detections needed before a cluster can be published
MAX_VARIANCE        = 0.7   # m² — cluster must be tight enough before publishing
DUPLICATE_RADIUS    = 0.7   # m  — min separation between two published ring markers
TF_TIMEOUT          = 0.3   # s  — how long to wait for TF at the marker's exact timestamp
MAX_POSITIONS       = 60    # rolling cap on stored positions per cluster

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# RViz colours per ring colour
MARKER_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "black":  (0.1, 0.1, 0.1),
}


def _rgb_to_colour_name(r, g, b, tolerance=0.15):
    for colour, (cr, cg, cb) in MARKER_COLOURS.items():
        if abs(r - cr) < tolerance and abs(g - cg) < tolerance and abs(b - cb) < tolerance:
            return colour
    return "unknown"


class RingCluster:
    def __init__(self, x, y, z, colour, timestamp):
        self.positions     = [(x, y, z)]
        self.colour_votes  = Counter({colour: 1})
        self.published     = False
        self.last_updated  = timestamp

    def add(self, x, y, z, colour, timestamp):
        self.positions.append((x, y, z))
        self.colour_votes[colour] += 1
        self.last_updated = timestamp
        if len(self.positions) > MAX_POSITIONS:
            self.positions.pop(0)

    def reset(self, x, y, z, colour, timestamp):
        self.positions    = [(x, y, z)]
        self.colour_votes = Counter({colour: 1})
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

    @property
    def best_colour(self):
        return self.colour_votes.most_common(1)[0][0]

    def distance_to(self, x, y):
        cx, cy, _ = self.centroid
        return math.sqrt((cx - x) ** 2 + (cy - y) ** 2)


class RingLocalizator(Node):
    def __init__(self):
        super().__init__('ring_localizator')

        self.declare_parameter('real_robot', False)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            Marker, '/ring_marker', self.marker_callback, SENSOR_QOS)

        self.ring_locations_pub = self.create_publisher(
            Marker, '/detected_ring_locations', 10)

        self.handled_ring_sub = self.create_subscription(
            Point, '/handled_ring_location', self.handled_ring_callback, 10)

        self.clusters: list[RingCluster] = []
        self.published_locations: list[tuple] = []  # (x, y, marker_id, colour)
        self.marker_id_counter = 0

        self.get_logger().info(
            f'RingLocalizator ready — '
            f'match_r={MATCH_RADIUS} m, confirm={MIN_DETECTIONS} detections, '
            f'max_var={MAX_VARIANCE} m²'
        )

    def marker_callback(self, marker_msg: Marker):
        frame_id = marker_msg.header.frame_id.lstrip('/')
        stamp    = marker_msg.header.stamp

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', frame_id, stamp,
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT))
        except tf2_ros.TransformException:
            try:
                tf = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'TF lookup failed: {ex}')
                return

        x_cam = marker_msg.pose.position.x
        y_cam = marker_msg.pose.position.y
        z_cam = marker_msg.pose.position.z

        if math.sqrt(x_cam**2 + y_cam**2 + z_cam**2) > MAX_DETECTION_DIST:
            return

        x, y, z = self._transform_point(x_cam, y_cam, z_cam, tf)
        colour   = _rgb_to_colour_name(marker_msg.color.r, marker_msg.color.g, marker_msg.color.b)
        now      = self.get_clock().now().nanoseconds / 1e9

        self._add_detection(x, y, z, colour, now)

    def _add_detection(self, x, y, z, colour, now):
        # Drop detections near already-published rings
        for pub_x, pub_y, _, _ in self.published_locations:
            if math.sqrt((pub_x - x) ** 2 + (pub_y - y) ** 2) < DUPLICATE_RADIUS:
                return

        # Find closest cluster within MATCH_RADIUS
        best, best_d = None, MATCH_RADIUS
        for c in self.clusters:
            d = c.distance_to(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None:
            self.clusters.append(RingCluster(x, y, z, colour, now))
            return

        # Reset stale cluster
        if (now - best.last_updated) > CLUSTER_TTL:
            best.reset(x, y, z, colour, now)
            return

        # Outlier rejection once cluster is established
        if best.count >= OUTLIER_MIN_COUNT and best_d > OUTLIER_THRESHOLD:
            return

        best.add(x, y, z, colour, now)

        if (not best.published
                and best.count >= MIN_DETECTIONS
                and best.xy_variance <= MAX_VARIANCE):

            cx, cy, cz = best.centroid

            # Suppress duplicates at publish time
            for pub_x, pub_y, _, _ in self.published_locations:
                if math.sqrt((pub_x - cx) ** 2 + (pub_y - cy) ** 2) < DUPLICATE_RADIUS:
                    best.published = True
                    return

            best.published = True
            ring_colour = best.best_colour
            marker_id   = self._publish_marker(cx, cy, cz, ring_colour)
            self.published_locations.append((cx, cy, marker_id, ring_colour))
            self.get_logger().info(
                f'Ring confirmed: colour={ring_colour}  '
                f'pos=({cx:.2f}, {cy:.2f})  '
                f'detections={best.count}  variance={best.xy_variance:.4f} m²  '
                f'total={len(self.published_locations)}'
            )

    def handled_ring_callback(self, msg: Point):
        """Re-publish nearest ring marker in grey to show it has been visited."""
        best, best_d = None, 0.8
        for pub_x, pub_y, marker_id, colour in self.published_locations:
            d = math.sqrt((pub_x - msg.x) ** 2 + (pub_y - msg.y) ** 2)
            if d < best_d:
                best_d = d
                best   = (pub_x, pub_y, marker_id)

        if best is None:
            return

        fx, fy, existing_id = best
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.type            = Marker.CYLINDER
        marker.id              = existing_id

        marker.pose.position.x = fx
        marker.pose.position.y = fy
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.25
        marker.scale.z = 0.05

        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        self.ring_locations_pub.publish(marker)
        self.get_logger().info(f'Ring marker {existing_id} at ({fx:.2f}, {fy:.2f}) marked as visited.')

    def _publish_marker(self, x, y, z, colour) -> int:
        r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.type            = Marker.CYLINDER
        marker.id              = self.marker_id_counter
        self.marker_id_counter += 1

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.25
        marker.scale.z = 0.05

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0

        self.ring_locations_pub.publish(marker)
        return marker.id

    def _transform_point(self, x, y, z, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)
        rx = R[0][0]*x + R[0][1]*y + R[0][2]*z
        ry = R[1][0]*x + R[1][1]*y + R[1][2]*z
        rz = R[2][0]*x + R[2][1]*y + R[2][2]*z
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _quat_to_mat(qx, qy, qz, qw):
        return [
            [1-2*(qy**2+qz**2),  2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),  1-2*(qx**2+qz**2),  2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),  2*(qy*qz+qx*qw),  1-2*(qx**2+qy**2)],
        ]


def main():
    rclpy.init(args=None)
    node = RingLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
