#!/usr/bin/env python3
"""
Ring localizator — accumulates per-frame detections, clusters them in map
frame, and publishes a persistent coloured marker once a location is confirmed.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from visualization_msgs.msg import Marker
from std_msgs.msg import String
import tf2_ros
import math
from collections import Counter

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS   = 0.5    # m — detections within this radius → same ring
CONFIRM_THRESH   = 10     # detections needed to confirm a ring
MIN_MARK_DIST    = 0.5    # m — minimum distance between two confirmed rings
MAX_RAW_PTS      = 500    # max points stored per cluster before trimming

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class Cluster:
    """Running accumulator for a candidate ring location."""
    __slots__ = ('points', 'centroid', 'colour_votes', 'confirmed')

    def __init__(self, x, y, z, colour):
        self.points       = [(x, y, z)]
        self.centroid     = (x, y, z)
        self.colour_votes = Counter({colour: 1})
        self.confirmed    = False

    def add(self, x, y, z, colour):
        if len(self.points) < MAX_RAW_PTS:
            self.points.append((x, y, z))
        # Incremental mean
        n = len(self.points)
        cx, cy, cz = self.centroid
        self.centroid = (cx + (x - cx) / n,
                         cy + (y - cy) / n,
                         cz + (z - cz) / n)
        self.colour_votes[colour] += 1

    @property
    def count(self):
        return len(self.points)

    @property
    def best_colour(self):
        return self.colour_votes.most_common(1)[0][0]

    def dist2d(self, x, y):
        return math.sqrt((self.centroid[0] - x) ** 2 +
                         (self.centroid[1] - y) ** 2)


# RViz colours per ring colour
MARKER_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "black":  (0.1, 0.1, 0.1),
}


class RingLocalizator(Node):
    def __init__(self):
        super().__init__('ring_localizator')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # We receive position markers and colour strings separately.
        # Buffer the latest colour so we can pair it with the next marker.
        self._latest_colour = "unknown"

        self.marker_sub = self.create_subscription(
            Marker, '/ring_marker', self.marker_callback, SENSOR_QOS)
        self.colour_sub = self.create_subscription(
            String, '/ring_colour', self.colour_callback, SENSOR_QOS)

        self.ring_locations_pub = self.create_publisher(
            Marker, '/detected_ring_locations', 10)

        self.clusters: list[Cluster] = []
        self.marker_id_counter = 0
        self.confirmed_markers = {}  # Store confirmed rings to republish: {id: (x, y, z, colour)}

        # Timer to republish confirmed rings every 100ms so behavior_manager keeps receiving them
        self.create_timer(0.1, self._republish_confirmed_rings)

        self.get_logger().info(
            f"RingLocalizator ready — "
            f"cluster_r={CLUSTER_RADIUS} m, confirm={CONFIRM_THRESH} detections")

    # ──────────────────────────────────────────────────────────────────────────
    def colour_callback(self, msg: String):
        self._latest_colour = msg.data

    # ──────────────────────────────────────────────────────────────────────────
    def marker_callback(self, marker_msg: Marker):
        try:
            x_bl = marker_msg.pose.position.x
            y_bl = marker_msg.pose.position.y
            z_bl = marker_msg.pose.position.z

            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            x, y, z = self._transform_point(x_bl, y_bl, z_bl, tf)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF failed: {ex}")
            return
        except Exception as e:
            self.get_logger().error(f"marker_callback: {e}")
            return

        colour = self._latest_colour

        # Find nearest cluster
        best, best_d = None, float('inf')
        for c in self.clusters:
            d = c.dist2d(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None or best_d > CLUSTER_RADIUS:
            best = Cluster(x, y, z, colour)
            self.clusters.append(best)
        else:
            best.add(x, y, z, colour)

        # Check for confirmation
        if best.count >= CONFIRM_THRESH and not best.confirmed:
            self._try_confirm(best)

    # ──────────────────────────────────────────────────────────────────────────
    def _try_confirm(self, cluster: Cluster):
        cx, cy, cz = cluster.centroid

        # Don't confirm if too close to an already-confirmed ring
        for c in self.clusters:
            if c.confirmed and c.dist2d(cx, cy) < MIN_MARK_DIST:
                cluster.confirmed = True  # suppress future checks
                return

        cluster.confirmed = True
        colour = cluster.best_colour
        
        # Store the confirmed marker for republishing
        marker_id = self.marker_id_counter
        self.marker_id_counter += 1
        self.confirmed_markers[marker_id] = (cx, cy, cz, colour)
        
        # Publish immediately
        self._publish_marker(cx, cy, cz, colour, marker_id)
        
        self.get_logger().info(
            f"Ring confirmed: colour={colour}  "
            f"pos=({cx:.2f}, {cy:.2f})  "
            f"detections={cluster.count}  "
            f"total rings={sum(1 for c in self.clusters if c.confirmed)}"
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _republish_confirmed_rings(self):
        """Continuously republish all confirmed rings."""
        for marker_id, (x, y, z, colour) in self.confirmed_markers.items():
            self._publish_marker(x, y, z, colour, marker_id)

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_marker(self, x, y, z, colour, marker_id=None):
        r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.type            = Marker.CYLINDER
        
        # Use provided marker_id or generate new one
        if marker_id is None:
            marker_id = self.marker_id_counter
            self.marker_id_counter += 1
        marker.id = marker_id

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.25
        marker.scale.z = 0.05

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0

        self.ring_locations_pub.publish(marker)

    # ──────────────────────────────────────────────────────────────────────────
    def _transform_point(self, x, y, z, tf):
        t  = tf.transform.translation
        q  = tf.transform.rotation
        R  = self._quat_to_mat(q.x, q.y, q.z, q.w)
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
