#!/usr/bin/env python3
"""
Ring localizator — accumulates per-frame detections, clusters them in map
frame, and publishes a persistent coloured marker once a location is confirmed.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String
import tf2_ros
import math
from collections import Counter
from statistics import median

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS   = 1.0    # m — detections within this radius → same ring
FAIR_THRESH      = 2      # detections before showing a temporary candidate marker
ACTION_THRESH    = 4      # detections before behavior may approach the ring
CONFIRM_THRESH   = 6      # detections needed to confirm a ring marker solidly
MIN_MARK_DIST    = 1.2    # m — minimum distance between two confirmed rings
ACTION_INLIER_RADIUS = 0.30
MAX_RAW_PTS      = 500    # max points stored per cluster before trimming

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class Cluster:
    """Running accumulator for a candidate ring location."""
    __slots__ = ('points', 'centroid', 'colour_votes', 'actionable', 'confirmed', 'suppressed', 'cluster_id')

    def __init__(self, x, y, z, colour, cluster_id):
        self.points       = [(x, y, z)]
        self.centroid     = (x, y, z)
        self.colour_votes = Counter({colour: 1})
        self.actionable   = False
        self.confirmed    = False
        self.suppressed   = False
        self.cluster_id   = cluster_id

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

    @property
    def robust_centroid(self):
        points = self.compact_points
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        return median(xs), median(ys), median(zs)

    @property
    def compact_points(self):
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        mx = median(xs)
        my = median(ys)
        compact = [
            p for p in self.points
            if math.sqrt((p[0] - mx) ** 2 + (p[1] - my) ** 2) <= ACTION_INLIER_RADIUS
        ]
        return compact if len(compact) >= max(3, int(0.6 * len(self.points))) else self.points

    def compact_enough(self, min_count):
        return len(self.compact_points) >= min_count

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

def _rgb_to_colour_name(r, g, b, tolerance=0.15):
    """Reverse-lookup colour name from RGB marker values."""
    for colour, (cr, cg, cb) in MARKER_COLOURS.items():
        if abs(r - cr) < tolerance and abs(g - cg) < tolerance and abs(b - cb) < tolerance:
            return colour
    return "unknown"


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
        self.cluster_id_counter = 0

        self.get_logger().info(
            f"RingLocalizator ready — "
            f"cluster_r={CLUSTER_RADIUS} m, confirm={CONFIRM_THRESH} detections")

    # ──────────────────────────────────────────────────────────────────────────
    def colour_callback(self, msg: String):
        self._latest_colour = msg.data

    # ──────────────────────────────────────────────────────────────────────────
    def marker_callback(self, marker_msg: Marker):
        try:
            x, y, z = self._marker_to_map(marker_msg)
            if not all(math.isfinite(v) for v in (x, y, z)):
                return

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF failed: {ex}")
            return
        except Exception as e:
            self.get_logger().error(f"marker_callback: {e}")
            return

        # Extract colour from marker's RGB values
        colour = _rgb_to_colour_name(marker_msg.color.r, marker_msg.color.g, marker_msg.color.b)

        # Find nearest cluster
        best, best_d = None, float('inf')
        for c in self.clusters:
            d = c.dist2d(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None or best_d > CLUSTER_RADIUS:
            best = Cluster(x, y, z, colour, self.cluster_id_counter)
            self.cluster_id_counter += 1
            self.clusters.append(best)
        else:
            best.add(x, y, z, colour)

        if best.suppressed:
            return

        if best.confirmed:
            self._publish_marker(*best.robust_centroid, best.best_colour,
                                 status="ring_confirmed", marker_id=best.cluster_id)
            return

        if best.actionable:
            self._publish_marker(*best.robust_centroid, best.best_colour,
                                 status="ring_actionable", marker_id=best.cluster_id)

        elif best.count >= FAIR_THRESH and not best.confirmed:
            self._publish_marker(*best.robust_centroid, best.best_colour,
                                 status="ring_candidate", marker_id=best.cluster_id)

        if best.count >= ACTION_THRESH and not best.actionable and best.compact_enough(ACTION_THRESH):
            self._try_make_actionable(best)

        # Check for confirmation
        if best.count >= CONFIRM_THRESH and not best.confirmed and best.compact_enough(CONFIRM_THRESH):
            self._try_confirm(best)

    # ──────────────────────────────────────────────────────────────────────────
    def _marker_to_map(self, marker_msg: Marker):
        source_frame = marker_msg.header.frame_id.lstrip('/') or 'base_link'
        if source_frame == 'map':
            p = marker_msg.pose.position
            return p.x, p.y, p.z

        stamp = marker_msg.header.stamp
        lookup_time = (
            rclpy.time.Time.from_msg(stamp)
            if stamp.sec != 0 or stamp.nanosec != 0
            else rclpy.time.Time()
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', source_frame, lookup_time, timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException:
            tf = self.tf_buffer.lookup_transform(
                'map', source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )

        p = marker_msg.pose.position
        return self._transform_point(p.x, p.y, p.z, tf)

    # ──────────────────────────────────────────────────────────────────────────
    def _try_make_actionable(self, cluster: Cluster):
        cx, cy, cz = cluster.robust_centroid
        for c in self.clusters:
            if (c.confirmed or c.actionable) and c.dist2d(cx, cy) < MIN_MARK_DIST:
                cluster.suppressed = True
                return

        cluster.actionable = True
        self._publish_marker(cx, cy, cz, cluster.best_colour,
                             status="ring_actionable", marker_id=cluster.cluster_id)
        self.get_logger().info(
            f"Ring actionable: colour={cluster.best_colour} "
            f"pos=({cx:.2f}, {cy:.2f}) detections={cluster.count}"
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _try_confirm(self, cluster: Cluster):
        cx, cy, cz = cluster.robust_centroid

        # Don't confirm if too close to an already-confirmed ring
        for c in self.clusters:
            if c.confirmed and c.dist2d(cx, cy) < MIN_MARK_DIST:
                cluster.suppressed = True
                return

        cluster.confirmed = True
        cluster.actionable = True
        colour = cluster.best_colour
        self._publish_marker(cx, cy, cz, colour, status="ring_confirmed", marker_id=cluster.cluster_id)
        self.get_logger().info(
            f"Ring confirmed: colour={colour}  "
            f"pos=({cx:.2f}, {cy:.2f})  "
            f"detections={cluster.count}  "
            f"total rings={sum(1 for c in self.clusters if c.confirmed)}"
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_marker(self, x, y, z, colour, status="ring_confirmed", marker_id=None):
        r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))
        confirmed = status == "ring_confirmed"
        actionable = status == "ring_actionable"

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = status
        marker.type            = Marker.LINE_STRIP
        marker.id              = int(marker_id) if marker_id is not None else self.marker_id_counter
        if marker_id is None:
            self.marker_id_counter += 1
        marker.action          = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        if confirmed:
            radius = 0.175
            marker.scale.x = 0.030
            marker.color.a = 1.0
        elif actionable:
            radius = 0.150
            marker.scale.x = 0.025
            marker.color.a = 0.85
        else:
            radius = 0.110
            marker.scale.x = 0.020
            marker.color.a = 0.45
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 700_000_000

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.text = status.replace("ring_", "")
        # No lifetime for confirmed rings; candidates are refreshed while visible.

        n = 36
        for i in range(n + 1):
            a = 2.0 * math.pi * i / n
            p = Point()
            p.x = radius * math.cos(a)
            p.y = radius * math.sin(a)
            p.z = 0.0
            marker.points.append(p)

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
