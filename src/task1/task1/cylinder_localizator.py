#!/usr/bin/env python3
"""
Cylinder localizator — clusters detections from cylinder_segmentation (already in
map frame), confirms each barrel once, detects orientation (vertical/horizontal),
logs a JSON report, saves a camera image for horizontal (leaking) barrels, and
prints a terminal warning.
"""

import json
import math
import os
from collections import Counter
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS      = 0.6   # m — detections within this radius → same barrel (vertical)
CLUSTER_RADIUS_H    = 1.2   # m — larger radius for horizontal cylinders (detections spread along axis)
CONFIRM_THRESH      = 25    # detections before confirming
MIN_MARK_DIST       = 0.5   # m — physical overlap guard (two barrels can't share a spot)
SAME_BARREL_RADIUS  = 3.0   # m — suppress if same colour+orientation within this range
MAX_RAW_PTS         = 300

REPORT_DIR = os.path.expanduser("~/RINS_report")
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
    "black":  (0.1, 0.1, 0.1),
}


def _rgb_to_colour_name(r, g, b):
    best, best_dist = "unknown", float("inf")
    for name, (cr, cg, cb) in MARKER_COLOURS.items():
        d = (r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2
        if d < best_dist:
            best, best_dist = name, d
    return best


class Cluster:
    __slots__ = ("points", "centroid", "colour_votes", "orientation_votes", "confirmed")

    def __init__(self, x, y, z, colour, orientation):
        self.points = [(x, y, z)]
        self.centroid = (x, y, z)
        self.colour_votes = Counter({colour: 1})
        self.orientation_votes = Counter({orientation: 1})
        self.confirmed = False

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

    def dist2d(self, x, y):
        return math.sqrt(
            (self.centroid[0] - x) ** 2 + (self.centroid[1] - y) ** 2
        )


class CylinderLocalizator(Node):
    def __init__(self):
        super().__init__("cylinder_localizator")

        os.makedirs(REPORT_DIR, exist_ok=True)
        self._load_report()

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
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        colour = _rgb_to_colour_name(msg.color.r, msg.color.g, msg.color.b)
        orientation = msg.text if msg.text in ("vertical", "horizontal") else "vertical"

        radius = CLUSTER_RADIUS_H if orientation == "horizontal" else CLUSTER_RADIUS

        best, best_d = None, float("inf")
        for c in self.clusters:
            d = c.dist2d(x, y)
            if d < best_d:
                best, best_d = c, d

        if best is None or best_d > radius:
            best = Cluster(x, y, z, colour, orientation)
            self.clusters.append(best)
        else:
            best.add(x, y, z, colour, orientation)

        if best.count >= CONFIRM_THRESH and not best.confirmed:
            self._try_confirm(best)

    def _try_confirm(self, cluster: Cluster):
        cx, cy, cz = cluster.centroid
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
            # Same colour + same orientation within SAME_BARREL_RADIUS → same barrel
            if d < SAME_BARREL_RADIUS and c.best_colour == colour and c.best_orientation == orientation:
                self.get_logger().info(
                    f"Suppressed duplicate barrel ({colour}/{orientation}) at ({cx:.2f},{cy:.2f}) "
                    f"— matches confirmed barrel at dist={d:.2f} m"
                )
                return

        cluster.confirmed = True
        leak = orientation == "horizontal"
        barrel_id = len(self.report["barrels"]) + 1

        self.get_logger().info(
            f"Barrel confirmed #{barrel_id}: colour={colour} "
            f"orientation={orientation} leak={leak} "
            f"pos=({cx:.2f}, {cy:.2f})"
        )

        if leak:
            self.get_logger().warn(
                f"⚠  WARNING: Barrel #{barrel_id} ({colour}) is HORIZONTAL — possible leak!"
            )

        image_path = self._save_image(barrel_id, leak)
        self._log_barrel(barrel_id, colour, orientation, leak, cx, cy, image_path)
        self._publish_marker(barrel_id, cx, cy, cz, colour, orientation)

    # ── Image saving ──────────────────────────────────────────────────────────
    def _save_image(self, barrel_id, leak):
        if self.latest_image is None:
            return None
        try:
            import cv2
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
    def _publish_marker(self, barrel_id, x, y, z, colour, orientation):
        r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cylinder_confirmed"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1

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
        marker.color.a = 1.0

        self.cylinder_locations_pub.publish(marker)


def main():
    rclpy.init(args=None)
    node = CylinderLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
