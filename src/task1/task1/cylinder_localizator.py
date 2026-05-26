#!/usr/bin/env python3
"""
Cylinder localizator — clusters detections from cylinder_segmentation (already in
map frame), confirms each barrel once, detects orientation (vertical/horizontal),
logs a JSON report, and saves a camera image for each barrel.
Spill detection is handled by the pointcloud_viewer node via service call.
"""

import colorsys
import json
import math
import os
from collections import Counter
from datetime import datetime
from statistics import median

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import tf2_ros

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS      = 0.30  # m — stable merge radius for repeated sightings
CLUSTER_RADIUS_H    = 0.65  # m — horizontal/front-facing cylinders vary more
FAIR_THRESH         = 4     # detections before showing a temporary candidate marker
CONFIRM_THRESH      = 10    # detections before confirming
MIN_MARK_DIST       = 0.18  # m — physical overlap guard; touching barrels are ~0.22 m apart
SAME_COLOUR_SUPPRESS_RADIUS   = 0.45
SAME_COLOUR_SUPPRESS_RADIUS_H = 1.5   # horizontal barrels: centroid varies more with viewing angle
COMPACT_INLIER_RADIUS = 0.32
MAX_RAW_PTS         = 300

REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "barrell_detection")
REPORT_JSON = os.path.join(REPORT_DIR, "barrel_report.json")

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

MARKER_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "purple": (0.6, 0.0, 1.0),
    "brown":  (0.45, 0.22, 0.08),
    "black":  (0.1, 0.1, 0.1),
}


def _rgb_to_colour_name(r, g, b):
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    h_deg = h * 360.0

    if v < 0.15:
        return "black"
    if s < 0.25:
        return "unknown"
    if h_deg < 15 or h_deg >= 330:
        return "red"
    if 15 <= h_deg < 45 and v < 0.45:
        return "brown"
    if h_deg < 40:
        return "orange"
    if h_deg < 80:
        return "yellow"
    if h_deg < 165:
        return "green"
    if h_deg < 270:
        return "blue"
    if h_deg < 315:
        return "purple"
    return "unknown"


def _colours_compatible(a, b):
    if a in (None, "unknown") or b in (None, "unknown"):
        return True
    return a == b


class Cluster:
    __slots__ = ("points", "centroid", "colour_votes", "orientation_votes", "confirmed", "cluster_id")

    def __init__(self, x, y, z, colour, orientation, cluster_id):
        self.points = [(x, y, z)]
        self.centroid = (x, y, z)
        self.colour_votes = Counter({colour: 1})
        self.orientation_votes = Counter({orientation: 1})
        self.confirmed = False
        self.cluster_id = cluster_id

    def add(self, x, y, z, colour, orientation):
        if len(self.points) < MAX_RAW_PTS:
            self.points.append((x, y, z))
        n = len(self.points)
        cx, cy, cz = self.centroid
        self.centroid = (
            cx + (x - cx) / n,
            cy + (y - cy) / n,
            cz + (z - cz) / n,
        )
        self.colour_votes[colour] += 1
        self.orientation_votes[orientation] += 1

    @property
    def count(self):
        return len(self.points)

    @property
    def best_colour(self):
        return self.colour_votes.most_common(1)[0][0]

    @property
    def best_orientation(self):
        return self.orientation_votes.most_common(1)[0][0]

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
            if math.sqrt((p[0] - mx) ** 2 + (p[1] - my) ** 2) <= COMPACT_INLIER_RADIUS
        ]
        return compact if len(compact) >= max(3, int(0.6 * len(self.points))) else self.points

    def compact_enough(self, min_count):
        return len(self.compact_points) >= min_count

    def dist2d(self, x, y):
        return math.sqrt(
            (self.centroid[0] - x) ** 2 + (self.centroid[1] - y) ** 2
        )


class CylinderLocalizator(Node):
    def __init__(self):
        super().__init__("cylinder_localizator")

        self._reset_report()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.latest_image = None

        self.marker_sub = self.create_subscription(
            Marker, "cylinder_markers", self.marker_callback, SENSOR_QOS
        )
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self._image_callback, SENSOR_QOS
        )
        self.cylinder_locations_pub = self.create_publisher(
            Marker, "/detected_cylinder_locations", 10
        )

        self.clusters: list[Cluster] = []
        self.marker_id_counter = 0
        self.cluster_id_counter = 0
        self.confirmed_markers = {}

        self.create_timer(2.0, self._republish_confirmed_markers)

        self.get_logger().info(
            f"CylinderLocalizator ready — report: {REPORT_JSON}"
        )

    # ── Image buffer ──────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            pass

    # ── Report persistence ────────────────────────────────────────────────────
    def _reset_report(self):
        import shutil
        if os.path.exists(REPORT_DIR):
            shutil.rmtree(REPORT_DIR)
        os.makedirs(REPORT_DIR, exist_ok=True)
        self.report = {"barrels": []}
        self._save_report()

    def _load_report(self):
        if os.path.exists(REPORT_JSON):
            with open(REPORT_JSON, "r") as f:
                self.report = json.load(f)
        else:
            self.report = {"barrels": []}

    def _save_report(self):
        with open(REPORT_JSON, "w") as f:
            json.dump(self.report, f, indent=2)

    # ── Detection ─────────────────────────────────────────────────────────────
    def marker_callback(self, msg: Marker):
        try:
            x, y, z = self._marker_to_map(msg)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Barrel marker TF failed: {ex}")
            return
        except Exception as exc:
            self.get_logger().warn(f"Invalid barrel marker: {exc}")
            return

        colour = _rgb_to_colour_name(msg.color.r, msg.color.g, msg.color.b)
        orientation = msg.text if msg.text in ("vertical", "horizontal") else "vertical"

        radius = CLUSTER_RADIUS_H if orientation == "horizontal" else CLUSTER_RADIUS

        best, best_d = None, float("inf")
        for c in self.clusters:
            if c.best_orientation != orientation:
                continue
            if not _colours_compatible(c.best_colour, colour):
                continue
            d = c.dist2d(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None or best_d > radius:
            best = Cluster(x, y, z, colour, orientation, self.cluster_id_counter)
            self.cluster_id_counter += 1
            self.clusters.append(best)
        else:
            best.add(x, y, z, colour, orientation)

        if best.count >= FAIR_THRESH and not best.confirmed and best.compact_enough(FAIR_THRESH):
            cx, cy, cz = best.robust_centroid
            self._publish_marker(
                best.cluster_id, cx, cy, cz, best.best_colour,
                best.best_orientation, confirmed=False
            )

        if best.count >= CONFIRM_THRESH and not best.confirmed and best.compact_enough(CONFIRM_THRESH):
            self._try_confirm(best)

    def _try_confirm(self, cluster: Cluster):
        cx, cy, cz = cluster.robust_centroid
        colour = cluster.best_colour
        orientation = cluster.best_orientation

        for c in self.clusters:
            if not c.confirmed:
                continue
            d = c.dist2d(cx, cy)
            # Physical overlap: always suppress
            if d < MIN_MARK_DIST:
                cluster.confirmed = True
                return
            suppress_r = SAME_COLOUR_SUPPRESS_RADIUS_H if orientation == "horizontal" else SAME_COLOUR_SUPPRESS_RADIUS
            if (
                d < suppress_r
                and c.best_orientation == orientation
                and _colours_compatible(c.best_colour, colour)
            ):
                self.get_logger().info(
                    f"Suppressed duplicate barrel ({colour}/{orientation}) at ({cx:.2f},{cy:.2f}); "
                    f"same-colour confirmed barrel is {d:.2f} m away"
                )
                cluster.confirmed = True
                return

        cluster.confirmed = True
        leak = None if orientation == "horizontal" else False
        barrel_id = len(self.report["barrels"]) + 1

        self.get_logger().info(
            f"Barrel confirmed #{barrel_id}: colour={colour} "
            f"orientation={orientation} leak={leak} "
            f"pos=({cx:.2f}, {cy:.2f})"
        )

        if orientation == "horizontal":
            self.get_logger().warn(
                f"WARNING: Barrel #{barrel_id} ({colour}) is horizontal — behavior_manager will inspect."
            )

        image_path = self._save_image(barrel_id, leak)
        self._log_barrel(barrel_id, colour, orientation, leak, cx, cy, image_path)
        self._publish_marker(
            barrel_id, cx, cy, cz, colour, orientation,
            confirmed=True, leak_detected=leak
        )

    def _marker_to_map(self, marker_msg: Marker):
        source_frame = marker_msg.header.frame_id.lstrip("/") or "map"
        p = marker_msg.pose.position
        if source_frame == "map":
            return p.x, p.y, p.z

        stamp = marker_msg.header.stamp
        lookup_time = (
            rclpy.time.Time.from_msg(stamp)
            if stamp.sec != 0 or stamp.nanosec != 0
            else rclpy.time.Time()
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", source_frame, lookup_time, timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException:
            tf = self.tf_buffer.lookup_transform(
                "map", source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )

        return self._transform_point(p.x, p.y, p.z, tf)

    def _transform_point(self, x, y, z, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)
        rx = R[0][0] * x + R[0][1] * y + R[0][2] * z
        ry = R[1][0] * x + R[1][1] * y + R[1][2] * z
        rz = R[2][0] * x + R[2][1] * y + R[2][2] * z
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _quat_to_mat(qx, qy, qz, qw):
        return [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)],
        ]

    # ── Image saving ──────────────────────────────────────────────────────────
    def _save_image(self, barrel_id, leak):
        if self.latest_image is None:
            return None
        try:
            suffix = "_LEAK" if leak else ""
            fname = f"barrel_{barrel_id}{suffix}_{datetime.now().strftime('%H%M%S')}.jpg"
            path = os.path.join(REPORT_DIR, fname)
            cv2.imwrite(path, self.latest_image)
            self.get_logger().info(f"Image saved: {path}")
            return path
        except Exception as e:
            self.get_logger().warn(f"Image save failed: {e}")
            return None

    # ── Report ────────────────────────────────────────────────────────────────
    def _log_barrel(self, barrel_id, colour, orientation, leak, x, y, image_path):
        entry = {
            "id": barrel_id,
            "colour": colour,
            "orientation": orientation,
            "leak_detected": leak,
            "position_map": [round(x, 3), round(y, 3)],
            "image_path": image_path,
            "confirmed_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.report["barrels"].append(entry)
        self._save_report()

    # ── Marker publishing ─────────────────────────────────────────────────────
    def _publish_marker(self, barrel_id, x, y, z, colour, orientation, confirmed=True, leak_detected=None):
        r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "barrel_confirmed" if confirmed else "barrel_candidate"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.id = int(barrel_id)

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        # Horizontal cylinders shown on their side (flattened)
        if orientation == "horizontal":
            marker.scale.x = 0.4
            marker.scale.y = 0.22
            marker.scale.z = 0.22
        else:
            marker.scale.x = 0.22
            marker.scale.y = 0.22
            marker.scale.z = 0.4

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0 if confirmed else 0.45
        marker.text = orientation  # read by behavior_manager to detect horizontal barrels
        if not confirmed:
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 700_000_000

        self.cylinder_locations_pub.publish(marker)

        if confirmed:
            self.confirmed_markers[int(barrel_id)] = {
                "x": x,
                "y": y,
                "z": z,
                "colour": colour,
                "orientation": orientation,
                "leak_detected": leak_detected,
            }
            if leak_detected is not None:
                self._publish_condition_label(int(barrel_id), x, y, z, leak_detected)

    def _update_barrel_condition_label(self, barrel_id, leak_detected):
        if barrel_id is None:
            return
        info = self.confirmed_markers.get(int(barrel_id))
        if info is None:
            return
        self._publish_condition_label(
            int(barrel_id), info["x"], info["y"], info["z"], leak_detected
        )

    def _publish_condition_label(self, barrel_id, x, y, z, leak_detected):
        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns = "barrel_label"
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.id = int(barrel_id)
        label.pose.position.x = x + 0.25
        label.pose.position.y = y + 0.25
        label.pose.position.z = z + 0.45
        label.pose.orientation.w = 1.0
        label.scale.z = 0.22
        label.color.a = 1.0

        if leak_detected is True:
            label.text = "LEAK"
            label.color.r = 1.0
            label.color.g = 0.1
            label.color.b = 0.0
        else:
            label.text = "OK"
            label.color.r = 0.1
            label.color.g = 1.0
            label.color.b = 0.1

        self.cylinder_locations_pub.publish(label)


    def _republish_confirmed_markers(self):
        for barrel_id, info in self.confirmed_markers.items():
            self._publish_marker(
                barrel_id, info["x"], info["y"], info["z"],
                info["colour"], info["orientation"],
                confirmed=True, leak_detected=info.get("leak_detected"),
            )


def main():
    rclpy.init(args=None)
    node = CylinderLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
