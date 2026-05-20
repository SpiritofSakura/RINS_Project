#!/usr/bin/env python3
"""
Cylinder localizator — clusters detections from cylinder_segmentation (already in
map frame), confirms each barrel once, detects orientation (vertical/horizontal),
logs a JSON report, saves a camera image for horizontal (leaking) barrels, and
prints a terminal warning.
"""

import colorsys
import json
import math
import os
from collections import Counter
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker

# ── Tuning ────────────────────────────────────────────────────────────────────
CLUSTER_RADIUS      = 0.6   # m — detections within this radius → same barrel (vertical)
CLUSTER_RADIUS_H    = 1.2   # m — larger radius for horizontal cylinders (detections spread along axis)
CONFIRM_THRESH      = 25    # detections before confirming
MIN_MARK_DIST       = 0.5   # m — physical overlap guard (two barrels can't share a spot)
SAME_BARREL_RADIUS  = 3.0   # m — suppress if same colour+orientation within this range
MAX_RAW_PTS         = 300

REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "barrell_detection")
REPORT_JSON = os.path.join(REPORT_DIR, "barrel_report.json")

# Spill detection — HSV ranges (OpenCV: H 0-180, S/V 0-255)
# Red has two ranges because it wraps around H=0/180
COLOR_HSV_RANGES = {
    "red":    [( 0, 100, 80), (10, 255, 255), (170, 100, 80), (180, 255, 255)],
    "green":  [(40, 100, 80), (80, 255, 255)],
    "blue":   [(100, 100, 80), (130, 255, 255)],
    "yellow": [(20, 100, 80), (35, 255, 255)],
    "orange": [(10, 100, 80), (20, 255, 255)],
    "black":  [( 0,   0,  0), (180, 255, 50)],
}
SPILL_PIXEL_THRESHOLD = 500   # matching pixels in top-cam image → spill confirmed
ARM_MOVE_WAIT_SECS    = 4     # seconds to wait after publishing arm command

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
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    h_deg = h * 360.0

    if v < 0.15:
        return "black"
    if s < 0.25:
        return "unknown"
    if h_deg < 15 or h_deg >= 330:
        return "red"
    if h_deg < 40:
        return "orange"
    if h_deg < 80:
        return "yellow"
    if h_deg < 165:
        return "green"
    if h_deg < 270:
        return "blue"
    return "unknown"


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

        self._reset_report()

        self.bridge = CvBridge()
        self.latest_image = None
        self.top_camera_image = None

        self.marker_sub = self.create_subscription(
            Marker, "cylinder_markers", self.marker_callback, SENSOR_QOS
        )
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self._image_callback, SENSOR_QOS
        )
        self.top_camera_sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._top_camera_cb, SENSOR_QOS
        )
        self.cylinder_locations_pub = self.create_publisher(
            Marker, "/detected_cylinder_locations", 10
        )
        self.arm_command_pub = self.create_publisher(String, "/arm_command", 1)
        self.spill_done_pub = self.create_publisher(Empty, "/spill_check_done", 1)

        self.create_subscription(Empty, "/start_spill_check", self._start_spill_cb, 10)

        self.clusters: list[Cluster] = []
        self.marker_id_counter = 0

        # Spill check state machine
        self._spill_queue: list[tuple] = []  # (barrel_id, colour)
        self._spill_state = "idle"           # idle → moving → capturing → returning
        self._spill_wait = 0
        self._current_spill: tuple | None = None
        self._spill_check_triggered = False  # set by /start_spill_check from behavior_manager
        self.create_timer(1.0, self._spill_timer_cb)

        self.get_logger().info(
            f"CylinderLocalizator ready — report: {REPORT_JSON}"
        )

    # ── Image buffer ──────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            pass

    def _top_camera_cb(self, msg: Image):
        try:
            self.top_camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            self._spill_queue.append((barrel_id, colour))

        image_path = self._save_image(barrel_id, leak)
        self._log_barrel(barrel_id, colour, orientation, leak, cx, cy, image_path)
        self._publish_marker(barrel_id, cx, cy, cz, colour, orientation)

    # ── Spill detection ───────────────────────────────────────────────────────
    def _start_spill_cb(self, _msg: Empty):
        """Called by behavior_manager when the robot has arrived at a horizontal barrel."""
        if not self._spill_queue:
            self.get_logger().warn("Spill check triggered but queue is empty — signalling done immediately.")
            self.spill_done_pub.publish(Empty())
            return
        self._spill_check_triggered = True
        self.get_logger().info("Spill check triggered by behavior_manager.")

    def _publish_arm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.arm_command_pub.publish(msg)

    def _spill_timer_cb(self):
        if self._spill_state == "idle":
            if self._spill_queue and self._spill_check_triggered:
                self._spill_check_triggered = False
                self._current_spill = self._spill_queue.pop(0)
                barrel_id, colour = self._current_spill
                self.get_logger().info(
                    f"Spill check: moving arm for barrel #{barrel_id} ({colour})"
                )
                self._publish_arm("look_for_spill")
                self._spill_state = "moving"
                self._spill_wait = 0

        elif self._spill_state == "moving":
            self._spill_wait += 1
            if self._spill_wait >= ARM_MOVE_WAIT_SECS:
                self._check_spill()
                self._publish_arm("garage")
                self._spill_state = "returning"
                self._spill_wait = 0

        elif self._spill_state == "returning":
            self._spill_wait += 1
            if self._spill_wait >= ARM_MOVE_WAIT_SECS:
                self._spill_state = "idle"
                self.spill_done_pub.publish(Empty())
                self.get_logger().info("Spill check complete — signalling behavior_manager.")

    def _check_spill(self):
        barrel_id, colour = self._current_spill

        if self.top_camera_image is None:
            self.get_logger().warn(f"Spill check barrel #{barrel_id}: no top camera image")
            return

        img = self.top_camera_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        ranges = COLOR_HSV_RANGES.get(colour)
        if ranges is None:
            self.get_logger().warn(f"No HSV range defined for colour '{colour}'")
            return

        if colour == "red":
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1])),
                cv2.inRange(hsv, np.array(ranges[2]), np.array(ranges[3])),
            )
        else:
            mask = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))

        pixel_count = int(cv2.countNonZero(mask))
        spill_detected = pixel_count >= SPILL_PIXEL_THRESHOLD

        self.get_logger().info(
            f"Spill check barrel #{barrel_id} ({colour}): "
            f"{pixel_count} px → {'SPILL DETECTED' if spill_detected else 'no spill'}"
        )
        if spill_detected:
            self.get_logger().warn(
                f"⚠  SPILL confirmed for barrel #{barrel_id} ({colour})!"
            )

        for entry in self.report["barrels"]:
            if entry["id"] == barrel_id:
                entry["spill_detected"] = spill_detected
                entry["spill_pixel_count"] = pixel_count
                break
        self._save_report()

        if spill_detected:
            fname = (
                f"barrel_{barrel_id}_SPILL_{colour.upper()}_"
                f"{datetime.now().strftime('%H%M%S')}.jpg"
            )
            path = os.path.join(REPORT_DIR, fname)
            cv2.imwrite(path, img)
            self.get_logger().info(f"Spill image saved: {path}")

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
        marker.text = orientation  # read by behavior_manager to detect horizontal barrels

        self.cylinder_locations_pub.publish(marker)


def main():
    rclpy.init(args=None)
    node = CylinderLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
