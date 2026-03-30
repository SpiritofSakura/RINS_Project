#!/usr/bin/python3
"""
Ring detector for small (~20 cm) coloured rings dangling from stands.

Strategy:
  1. Segment by colour in HSV (red, green, blue, yellow — extend as needed)
  2. Find contours in each colour mask
  3. Fit ellipse and check shape (small rings viewed at angles → ellipses)
  4. Verify with depth: centre must be hollow (sky/nothing), rim must be solid
  5. Publish marker with colour embedded, plus a /ring_colour String topic

Why colour-first beats edge-first for this scene:
  - Rings are strongly saturated solid colours
  - Background (sky, wooden boxes, grey floor) has very different hue
  - Colour segmentation kills false positives immediately
  - We get the ring colour for free (needed later for speech)
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ── QoS ───────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Colour HSV ranges ─────────────────────────────────────────────────────────
# Red wraps around 180° so it needs two ranges.
# Tune S/V minimums if the simulator renders rings darker or more washed-out.
COLOUR_RANGES = {
    "red": [
        (np.array([0,   130,  60]), np.array([10,  255, 255])),
        (np.array([168, 130,  60]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40,  80,  50]), np.array([90, 255, 255])),
    ],
    "blue": [
        (np.array([100, 80,  50]), np.array([140, 255, 255])),
    ],
    "yellow": [
        (np.array([20,  100, 100]), np.array([35, 255, 255])),
    ],
    "orange": [
        (np.array([10,  150,  80]), np.array([20, 255, 255])),
    ],
    "black": [
        (np.array([0, 0, 0]), np.array([180, 255, 50])),
    ],
}

# RViz marker colours (r, g, b) per ring colour
MARKER_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "black":  (0.1, 0.1, 0.1),
}

# ── Shape filter tuning ───────────────────────────────────────────────────────
MIN_CONTOUR_PTS  = 15       # need this many points to fit an ellipse
MIN_AREA_PX2     = 200      # px² — a 20 cm ring at ~3 m is still >200 px²
MAX_AREA_PX2     = 50_000   # upper sanity bound
MIN_ASPECT       = 0.28     # minor/major — allows ~75° viewing angle
MIN_CIRCULARITY  = 0.15     # relaxed for perspective + small size
MAX_CIRCULARITY  = 1.40

# ── Cross-frame confirmation ──────────────────────────────────────────────────
CONFIRM_HITS     = 8    # frames a ring must be seen before it's "confirmed"
MAX_MISSED       = 10   # frames without a match before dropping a candidate
MATCH_DIST_PX    = 40   # pixel radius to match a detection to an existing candidate

# ── Depth verification ────────────────────────────────────────────────────────
MAX_RANGE_M      = 7.0      # beyond this → "nothing" / sky
RIM_SOLID_FRAC   = 0.35     # fraction of 12 rim samples that must be solid
                             # (low because the stand can occlude some samples)


class RingDetector(Node):
    def __init__(self):
        super().__init__('ring_detector')
        self.bridge    = CvBridge()
        self.depth_raw = None

        # Startup delay: don't confirm rings until N seconds have passed
        self.declare_parameter('startup_delay_seconds', 0)
        startup_delay_seconds = self.get_parameter('startup_delay_seconds').value
        self.startup_time = self.get_clock().now().nanoseconds
        self.startup_delay_ns = int(startup_delay_seconds * 1_000_000_000)
        if self.startup_delay_ns > 0:
            self.get_logger().info(f"Ring confirmation delayed by {startup_delay_seconds} seconds")

        # Rings confirmed in the current frame — list of dicts
        self.confirmed_rings = []

        # Cross-frame accumulator: list of {colour, cx, cy, hits, missed}
        self._candidates = []

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.image_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/depth", self.depth_callback, SENSOR_QOS)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points",
            self.pointcloud_callback, SENSOR_QOS)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, "/ring_marker", 10)
        self.colour_pub = self.create_publisher(String, "/ring_colour", 10)

        # Debug windows
        cv2.namedWindow("Colour masks",  cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth window",   cv2.WINDOW_NORMAL)

        self.get_logger().info("RingDetector ready (colour-first mode).")

    # ── Main image callback ───────────────────────────────────────────────────
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        if self.depth_raw is None:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        combined_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        frame_detections = []  # raw detections this frame

        for colour_name, ranges in COLOUR_RANGES.items():
            # Build colour mask
            mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            # Remove speckle noise, then close small gaps in the ring outline
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))

            combined_mask |= mask

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                ring = self._evaluate_contour(cnt, colour_name)
                if ring is not None:
                    frame_detections.append(ring)

        # ── Update cross-frame candidates ────────────────────────────────────
        matched = set()
        for det in frame_detections:
            best_idx, best_dist = None, MATCH_DIST_PX
            for i, cand in enumerate(self._candidates):
                if cand["colour"] != det["colour"]:
                    continue
                dist = np.hypot(cand["cx"] - det["cx"], cand["cy"] - det["cy"])
                if dist < best_dist:
                    best_dist, best_idx = dist, i
            if best_idx is not None:
                self._candidates[best_idx]["hits"] += 1
                self._candidates[best_idx]["missed"] = 0
                self._candidates[best_idx]["cx"] = det["cx"]
                self._candidates[best_idx]["cy"] = det["cy"]
                self._candidates[best_idx]["ring"] = det
                matched.add(best_idx)
            else:
                self._candidates.append(
                    {"colour": det["colour"], "cx": det["cx"], "cy": det["cy"],
                     "hits": 1, "missed": 0, "ring": det})

        # Increment missed counter for unmatched candidates, prune stale ones
        for i in range(len(self._candidates) - 1, -1, -1):
            if i not in matched:
                self._candidates[i]["missed"] += 1
            if self._candidates[i]["missed"] > MAX_MISSED:
                self._candidates.pop(i)

        # Only draw/report candidates that have enough hits (but only after startup delay)
        current_time = self.get_clock().now().nanoseconds
        startup_elapsed_ns = current_time - self.startup_time
        if startup_elapsed_ns >= self.startup_delay_ns:
            # Startup delay has passed, confirm rings normally
            self.confirmed_rings = [
                c["ring"] for c in self._candidates if c["hits"] >= CONFIRM_HITS
            ]
        else:
            # Still in startup delay, don't confirm yet (but keep accumulating candidates)
            self.confirmed_rings = []
        
        for ring in self.confirmed_rings:
            self._draw_ring(cv_image, ring)
        if self.confirmed_rings:
            self.get_logger().info(f"RING IN VIEW: {[r['colour'] for r in self.confirmed_rings]}")

        # Show combined colour mask for debugging
        cv2.imshow("Colour masks",
                   cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR))
        cv2.imshow("Detected rings", cv_image)
        cv2.waitKey(1)

    # ── Contour → ring dict (or None) ─────────────────────────────────────────
    def _evaluate_contour(self, cnt, colour_name):
        if len(cnt) < MIN_CONTOUR_PTS:
            return None

        area = cv2.contourArea(cnt)
        if not (MIN_AREA_PX2 < area < MAX_AREA_PX2):
            self.get_logger().debug(f"[{colour_name}] area fail: {area:.0f}")
            return None

        try:
            ellipse = cv2.fitEllipse(cnt)
        except cv2.error:
            return None

        (ex, ey), (minor_d, major_d), angle = ellipse
        if major_d == 0:
            return None

        # Aspect ratio filter
        aspect = minor_d / major_d
        if aspect < MIN_ASPECT:
            self.get_logger().debug(f"[{colour_name}] aspect fail: {aspect:.2f}")
            return None

        # Circularity relative to fitted ellipse area (more stable than hull)
        ellipse_area = np.pi * (minor_d / 2) * (major_d / 2)
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            return None
        circularity = (4 * np.pi * ellipse_area) / (perimeter ** 2)
        if not (MIN_CIRCULARITY < circularity < MAX_CIRCULARITY):
            self.get_logger().debug(f"[{colour_name}] circularity fail: {circularity:.2f}")
            return None

        cx, cy   = int(ex), int(ey)
        semi_px  = int(major_d / 2)

        if not self._depth_is_ring(cx, cy, semi_px):
            self.get_logger().debug(f"[{colour_name}] depth fail at ({cx},{cy}) semi={semi_px}")
            return None

        return {
            "cx": cx, "cy": cy,
            "semi_major_px": semi_px,
            "ellipse": ellipse,
            "colour": colour_name,
        }

    # ── Depth verification ────────────────────────────────────────────────────
    def _depth_is_ring(self, cx, cy, semi_px):
        h, w = self.depth_raw.shape
        # Only reject if the ring centre itself is out of bounds
        if not (0 <= cx < w and 0 <= cy < h):
            return False

        # Centre must be hollow: sample a small patch and take the median
        half = max(2, semi_px // 6)
        patch = self.depth_raw[cy - half: cy + half + 1,
                               cx - half: cx + half + 1].ravel()
        valid_centre = patch[np.isfinite(patch) & (patch > 0)]
        if len(valid_centre) > 0 and float(np.median(valid_centre)) < MAX_RANGE_M:
            return False  # solid surface at centre → not a ring

        # Rim: 12 evenly spaced samples around the ellipse
        solid = sum(
            1 for k in range(12)
            for rx, ry in [(
                int(cx + semi_px * np.cos(2 * np.pi * k / 12)),
                int(cy + semi_px * np.sin(2 * np.pi * k / 12)),
            )]
            if 0 <= ry < h and 0 <= rx < w
            and np.isfinite(self.depth_raw[ry, rx])
            and 0 < self.depth_raw[ry, rx] < MAX_RANGE_M
        )

        return (solid / 12) >= RIM_SOLID_FRAC

    # ── Depth callback ────────────────────────────────────────────────────────
    def depth_callback(self, data):
        try:
            self.depth_raw = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError:
            return

        d = self.depth_raw.copy()
        d[~np.isfinite(d)] = 0
        cv2.normalize(d, d, 0, 255, cv2.NORM_MINMAX)
        viz = cv2.cvtColor(np.uint8(d), cv2.COLOR_GRAY2BGR)

        for ring in self.confirmed_rings:
            cv2.circle(viz, (ring["cx"], ring["cy"]),
                       ring["semi_major_px"], (0, 255, 0), 2)

        cv2.imshow("Depth window", viz)
        cv2.waitKey(1)

    # ── Point cloud callback ──────────────────────────────────────────────────
    def pointcloud_callback(self, data):
        if not self.confirmed_rings:
            return

        try:
            pts = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            pts = pts.reshape((data.height, data.width, 3))

            for i, ring in enumerate(self.confirmed_rings):
                cx, cy, semi_px = ring["cx"], ring["cy"], ring["semi_major_px"]
                colour = ring["colour"]

                # Use rim point to the right (least likely to be occluded by stand)
                rx = min(cx + semi_px, data.width - 1)
                ry = min(cy, data.height - 1)
                p  = pts[ry, rx, :]

                if np.isnan(p).any() or np.isinf(p).any():
                    continue

                # Marker
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp    = data.header.stamp
                marker.type            = Marker.CYLINDER
                marker.id              = i
                marker.scale.x = marker.scale.y = marker.scale.z = 0.1

                r, g, b = MARKER_COLOURS.get(colour, (1.0, 1.0, 1.0))
                marker.color.r, marker.color.g, marker.color.b = r, g, b
                marker.color.a = 1.0

                marker.pose.position.x = float(p[0])
                marker.pose.position.y = float(p[1])
                marker.pose.position.z = float(p[2])
                marker.pose.orientation.w = 1.0

                self.marker_pub.publish(marker)

                # Publish colour string for the localizator
                self.colour_pub.publish(String(data=colour))

        except Exception as e:
            self.get_logger().error(f"pointcloud_callback: {e}")

    # ── Draw helper ───────────────────────────────────────────────────────────
    def _draw_ring(self, img, ring):
        bgr = {
            "red": (0, 0, 220), "green": (0, 200, 0), "blue": (220, 80, 0),
            "yellow": (0, 220, 220), "orange": (0, 140, 255), "black": (50, 50, 50),
        }.get(ring["colour"], (200, 200, 200))

        cv2.ellipse(img, ring["ellipse"], bgr, 2)
        cv2.circle(img, (ring["cx"], ring["cy"]), 4, (255, 255, 255), -1)
        cv2.putText(img, ring["colour"],
                    (ring["cx"] - 20, ring["cy"] - ring["semi_major_px"] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = RingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
