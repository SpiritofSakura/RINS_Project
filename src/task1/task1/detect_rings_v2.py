#!/usr/bin/python3
"""
Ring detector v2 — Hough circle-based detection on disparity map.

Strategy:
  1. Convert depth to disparity (inverse)
  2. Apply Hough circle detection to find ring candidates
  3. Verify with depth: centre must be hollow (far/sky), rim must be solid (near)
  4. Cross-frame confirmation for stability
  5. Use ML-based color classifier to extract ring colour
  6. Publish marker with colour and /ring_colour String topic
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
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ── QoS ───────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Hough circle tuning ───────────────────────────────────────────────────────
HOUGH_DP = 1                    # Inverse ratio of accumulator resolution
HOUGH_MIN_DIST = 40             # Minimum distance between circle centres
HOUGH_PARAM1 = 48               # Upper threshold for Canny edge (balanced strictness)
HOUGH_PARAM2 = 25               # Accumulator threshold (strict but allows real rings)
HOUGH_MIN_RADIUS = 6          # Very small circles are usually texture/noise
HOUGH_MAX_RADIUS = 70          # Maximum circle radius (px)

# ── Cross-frame confirmation ──────────────────────────────────────────────────
CONFIRM_HITS = 8               # frames a ring must be seen before confirmed (strict but fair)
MAX_MISSED = 4                  # frames without match before dropping candidate (balanced)
MATCH_DIST_PX = 35              # pixel radius to match circles between frames

# ── Depth verification ────────────────────────────────────────────────────────
MAX_RANGE_M = 7.0               # beyond this → "nothing" / sky
RIM_SOLID_FRAC = 0.35           # fraction of rim samples that must be solid
CENTER_RADIUS_FRAC = 0.42       # centre disk that should be hollow/far
RIM_INNER_FRAC = 0.62           # annulus inner radius
RIM_OUTER_FRAC = 1.12           # annulus outer radius
CENTER_VALID_MAX_FRAC = 0.22    # hollow centre has few valid/depth pixels
CENTER_FAR_DELTA_M = 0.12       # or centre surface is this much behind rim
SOLID_CENTER_MIN_FRAC = 0.35    # barrel caps usually have many centre depth pixels
SOLID_CENTER_MAX_GAP_M = 0.22   # centre close to rim => solid cap, not a hole
SOLID_CENTER_MAX_STD_M = 0.12   # smooth centre depth => planar barrel end-cap
CENTER_VISUAL_SOLID_MAX_FRAC = 0.30  # missing depth but visually filled => likely barrel cap
FRONT_BARREL_VISUAL_FILL_MIN_FRAC = 0.60
MIN_RIM_POINTS = 25             # minimum point-cloud points for localization

# ── Colour HSV ranges (from detect_rings.py) ──────────────────────────────────
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

# ── Colour class names ────────────────────────────────────────────────────────
COLOUR_NAMES = ['red', 'green', 'blue', 'yellow', 'orange', 'black']

# RViz marker colours (r, g, b) per ring colour
MARKER_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
    "orange": (1.0, 0.5, 0.0),
    "black":  (0.1, 0.1, 0.1),
}


class RingDetectorV2(Node):
    def __init__(self):
        super().__init__('ring_detector_v2')
        self.bridge = CvBridge()
        self.depth_raw = None
        self.rgb_image = None
        self.image_header = None  # Store image header for marker timestamps
        self.pointcloud_xyz = None  # Point cloud XYZ positions
        self.pointcloud_rgb = None  # Point cloud RGB colors
        self.pointcloud_frame_id = None

        # Robot state for conditional publishing
        self.robot_state = 'IDLE'

        # Cross-frame candidates: list of {cx, cy, hits, missed, colour, depth_m}
        self._candidates = []
        
        # Marker ID counter
        self.marker_id_counter = 0

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.image_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/depth", self.depth_callback, SENSOR_QOS)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, SENSOR_QOS)
        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self.robot_state_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, "/ring_marker", 10)
        self.front_barrel_pub = self.create_publisher(Marker, "cylinder_markers", 10)
        self.colour_pub = self.create_publisher(String, "/ring_colour", 10)

        # Keep one lightweight camera window for ring debugging. Extra disparity
        # and mask windows are expensive and make the full task stack sluggish.
        cv2.namedWindow("Ring Camera", cv2.WINDOW_NORMAL)
        
        # Store combined mask for color extraction
        self.combined_mask = None

        self.get_logger().info("RingDetector V2 ready (Hough circle mode).")

    def depth_callback(self, data):
        try:
            self.depth_raw = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError:
            return

    def pointcloud_callback(self, data):
        """Store point cloud XYZ and RGB for accurate 3D position and color lookup."""
        try:
            # Log available fields once
            if not hasattr(self, 'pc_fields_logged'):
                field_names = [f.name for f in data.fields]
                self.get_logger().debug(f"Point cloud fields: {field_names}")
                self.pc_fields_logged = True
            
            # Extract XYZ positions
            pts_xyz = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            if pts_xyz is None or len(pts_xyz) == 0:
                self.get_logger().error(f"Point cloud XYZ empty or None")
                return
            self.pointcloud_xyz = pts_xyz.reshape((data.height, data.width, 3))
            self.pointcloud_frame_id = data.header.frame_id
            
            # Try to extract RGB field
            pts_rgb = None
            for field_name in ["rgb", "rgba", "RGB"]:
                try:
                    pts_rgb = pc2.read_points_numpy(data, field_names=(field_name,))
                    if pts_rgb is not None and len(pts_rgb) > 0:
                        self.pointcloud_rgb = pts_rgb.reshape((data.height, data.width))
                        return
                except:
                    pass
            
            # If no RGB field found, will use image patch color fallback
            self.pointcloud_rgb = None
                
        except Exception as e:
            self.get_logger().error(f"✗ Point cloud failed: {e}")

    def image_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        # Store image header for marker timestamps
        self.image_header = data.header

        if self.depth_raw is None or self.rgb_image is None:
            return

        # Convert depth to disparity (inverse, normalized)
        depth_m = self.depth_raw.astype(np.float32) / 1000.0  # mm → m
        with np.errstate(divide='ignore', invalid='ignore'):
            disparity = np.where(depth_m > 0, 1.0 / depth_m, 0)
        disparity_8u = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        disparity_8u = cv2.GaussianBlur(disparity_8u, (5, 5), 0)

        # Also compute disparity mask once for later use
        disparity_mask = (disparity > 0).astype(np.uint8) * 255

        # Hough circle detection
        circles = cv2.HoughCircles(
            disparity_8u,
            cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP,
            minDist=HOUGH_MIN_DIST,
            param1=HOUGH_PARAM1,
            param2=HOUGH_PARAM2,
            minRadius=HOUGH_MIN_RADIUS,
            maxRadius=HOUGH_MAX_RADIUS,
        )

        # Compute disparity mask for later use
        disparity_mask = (disparity > 0).astype(np.uint8) * 255
        
        frame_detections = []
        accepted_circles = set()
        front_barrel_circles = set()
        if circles is not None:
            self.get_logger().debug(f"Hough found {len(circles[0])} circles")
            for circle in circles[0]:
                cx, cy, radius = int(circle[0]), int(circle[1]), int(circle[2])
                ring = self._evaluate_circle(cx, cy, radius, depth_m, self.rgb_image, disparity_mask)
                if ring is not None:
                    if ring.get("kind") == "front_barrel":
                        self._publish_front_barrel_marker(ring)
                        front_barrel_circles.add((cx, cy, radius))
                    else:
                        frame_detections.append(ring)
                        accepted_circles.add((cx, cy, radius))
            if frame_detections:
                self.get_logger().info(f"Accepted {len(frame_detections)} hollow ring detections")

        # ── Publish each valid detection IMMEDIATELY ───────────────────────────────────
        for detection in frame_detections:
            # Publish to /ring_marker (color will be inferred from patch mask)
            # Let ring_localizator handle aggregation and confirmation
            self._publish_marker_raw(detection)

        # Lightweight camera visualization
        debug_img = self.rgb_image.copy()
        if circles is not None:
            for circle in circles[0]:
                cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
                accepted = (cx, cy, r) in accepted_circles
                front_barrel = (cx, cy, r) in front_barrel_circles
                if accepted:
                    draw_color = (0, 255, 0)
                elif front_barrel:
                    draw_color = (255, 128, 0)
                else:
                    draw_color = (0, 0, 255)
                cv2.circle(debug_img, (cx, cy), r, draw_color, 2)
                cv2.circle(debug_img, (cx, cy), 2, draw_color, 3)

        cv2.imshow("Ring Camera", debug_img)
        cv2.waitKey(1)

    def robot_state_callback(self, data):
        """Update robot state for conditional marker publishing."""
        self.robot_state = data.data

    def _evaluate_circle(self, cx, cy, radius, depth_m, rgb_img, disparity_mask):
        """
        Verify that a Hough circle is a 3D ring, not a barrel end or printed circle.

        A real ring has a solid depth annulus and a centre that is either invalid
        through the hole or significantly farther away than the rim.
        """
        h, w = depth_m.shape
        margin = int(radius * RIM_OUTER_FRAC) + 2
        if not (margin < cx < w - margin and margin < cy < h - margin):
            return None

        valid_depth = (depth_m > 0.1) & (depth_m < MAX_RANGE_M)

        center_mask = self._circle_mask(
            (h, w), cx, cy, max(2, int(radius * CENTER_RADIUS_FRAC))
        )
        rim_mask = self._annulus_mask(
            (h, w),
            cx,
            cy,
            max(3, int(radius * RIM_INNER_FRAC)),
            max(4, int(radius * RIM_OUTER_FRAC)),
        )

        rim_total = int(np.count_nonzero(rim_mask))
        center_total = int(np.count_nonzero(center_mask))
        if rim_total == 0 or center_total == 0:
            return None

        rim_valid = rim_mask & valid_depth
        center_valid = center_mask & valid_depth
        rim_valid_frac = float(np.count_nonzero(rim_valid)) / rim_total
        center_valid_frac = float(np.count_nonzero(center_valid)) / center_total
        if rim_valid_frac < RIM_SOLID_FRAC:
            return None

        rim_depths = depth_m[rim_valid]
        if len(rim_depths) < MIN_RIM_POINTS:
            return None

        rim_depth = float(np.median(rim_depths))
        rim_surface = rim_valid & (np.abs(depth_m - rim_depth) <= max(0.12, 0.08 * rim_depth))
        if np.count_nonzero(rim_surface) < MIN_RIM_POINTS:
            rim_surface = rim_valid

        rim_colour = self._classify_ring_colour_with_mask(
            rgb_img, rim_surface.astype(np.uint8) * 255
        )
        if rim_colour == "unknown":
            return None

        center_colour = self._classify_ring_colour_with_mask(
            rgb_img, center_mask.astype(np.uint8) * 255
        )
        if rim_colour not in ("unknown", "black") and center_colour == rim_colour:
            front_barrel = self._make_front_barrel_detection(
                cx, cy, radius, rgb_img, valid_depth, rim_surface, center_mask
            )
            if front_barrel is not None:
                return front_barrel
            return None

        center_visual_fill = self._visual_fill_fraction(rgb_img, center_mask)
        center_depths = depth_m[center_valid]
        hollow_by_missing_depth = center_valid_frac <= CENTER_VALID_MAX_FRAC
        hollow_by_far_depth = False
        if len(center_depths) > 0:
            center_depth = float(np.median(center_depths))
            center_depth_std = float(np.std(center_depths))
            center_gap = center_depth - rim_depth
            solid_center = (
                center_valid_frac >= SOLID_CENTER_MIN_FRAC
                and center_gap <= max(SOLID_CENTER_MAX_GAP_M, 0.06 * rim_depth)
                and center_depth_std <= SOLID_CENTER_MAX_STD_M
            )
            if solid_center:
                front_barrel = self._make_front_barrel_detection(
                    cx, cy, radius, rgb_img, valid_depth, rim_surface, center_mask
                )
                if front_barrel is not None:
                    return front_barrel
                return None
            hollow_by_far_depth = (center_depth - rim_depth) >= max(CENTER_FAR_DELTA_M, 0.03 * rim_depth)

        if hollow_by_missing_depth and center_visual_fill >= FRONT_BARREL_VISUAL_FILL_MIN_FRAC:
            front_barrel = self._make_front_barrel_detection(
                cx, cy, radius, rgb_img, valid_depth, rim_surface, center_mask
            )
            if front_barrel is not None:
                return front_barrel
            hollow_by_missing_depth = False

        if center_visual_fill >= CENTER_VISUAL_SOLID_MAX_FRAC and not hollow_by_far_depth:
            hollow_by_missing_depth = False

        if not (hollow_by_missing_depth or hollow_by_far_depth):
            return None

        point = self._point_from_mask(rim_surface)
        if point is None:
            return None

        # Extract ring patch for colour classification
        patch_size = max(int(radius * 2.5), 64)
        x1 = max(0, cx - patch_size // 2)
        x2 = min(w, cx + patch_size // 2)
        y1 = max(0, cy - patch_size // 2)
        y2 = min(h, cy + patch_size // 2)
        
        # Extract patch and its corresponding mask
        ring_patch = rgb_img[y1:y2, x1:x2].copy()
        patch_mask = (rim_surface.astype(np.uint8) * 255)[y1:y2, x1:x2]

        return {
            "cx": cx, "cy": cy, "radius": radius,
            "point": point,
            "frame_id": self.pointcloud_frame_id or "base_link",
            "ring_patch": ring_patch,
            "patch_mask": patch_mask,
            "hollow_score": 1.0 - center_valid_frac,
            "kind": "ring",
        }

    @staticmethod
    def _circle_mask(shape, cx, cy, radius):
        mask = np.zeros(shape, dtype=np.uint8)
        cv2.circle(mask, (cx, cy), radius, 255, -1)
        return mask > 0

    @staticmethod
    def _annulus_mask(shape, cx, cy, inner_radius, outer_radius):
        outer = np.zeros(shape, dtype=np.uint8)
        inner = np.zeros(shape, dtype=np.uint8)
        cv2.circle(outer, (cx, cy), outer_radius, 255, -1)
        cv2.circle(inner, (cx, cy), inner_radius, 255, -1)
        return (outer > 0) & (inner == 0)

    def _visual_fill_fraction(self, rgb_img, mask):
        if rgb_img is None or rgb_img.size == 0:
            return 0.0
        total = int(np.count_nonzero(mask))
        if total == 0:
            return 0.0

        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        coloured_surface = (s > 45) & (v > 35)
        dark_surface = (v > 12) & (v < 85)
        object_like = mask & (coloured_surface | dark_surface)
        return float(np.count_nonzero(object_like)) / total

    def _mean_rgb_from_mask(self, rgb_img, mask):
        if rgb_img is None or rgb_img.size == 0:
            return None
        if np.count_nonzero(mask) == 0:
            return None

        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        useful = mask & (((s > 35) & (v > 30)) | ((v > 12) & (v < 95)))
        pixels = rgb_img[useful]
        if len(pixels) < 10:
            pixels = rgb_img[mask & (v > 12)]
        if len(pixels) < 10:
            return None

        median_bgr = np.median(pixels, axis=0)
        b, g, r = median_bgr / 255.0
        return float(r), float(g), float(b)

    def _make_front_barrel_detection(self, cx, cy, radius, rgb_img, valid_depth, rim_surface, center_mask):
        cap_mask = self._circle_mask(
            valid_depth.shape, cx, cy, max(3, int(radius * 0.88))
        )
        surface_mask = rim_surface | (center_mask & valid_depth)
        if np.count_nonzero(surface_mask) < MIN_RIM_POINTS:
            surface_mask = rim_surface

        point = self._point_from_mask(surface_mask)
        if point is None:
            return None

        visual_fill = self._visual_fill_fraction(rgb_img, center_mask)
        center_depth_points = int(np.count_nonzero(center_mask & valid_depth))
        if visual_fill < FRONT_BARREL_VISUAL_FILL_MIN_FRAC:
            return None
        if center_depth_points < MIN_RIM_POINTS and visual_fill < 0.75:
            return None

        rgb = self._mean_rgb_from_mask(rgb_img, cap_mask | rim_surface)
        if rgb is None:
            rgb = (0.5, 0.5, 0.5)

        return {
            "kind": "front_barrel",
            "cx": cx,
            "cy": cy,
            "radius": radius,
            "point": point,
            "frame_id": self.pointcloud_frame_id or "base_link",
            "rgb": rgb,
            "visual_fill": visual_fill,
        }

    def _point_from_mask(self, mask):
        if self.pointcloud_xyz is None:
            return None
        if self.pointcloud_xyz.shape[:2] != mask.shape:
            return None

        pts = self.pointcloud_xyz[mask]
        if len(pts) < MIN_RIM_POINTS:
            return None
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) < MIN_RIM_POINTS:
            return None
        return np.median(pts, axis=0)

    def _classify_ring_colour_with_mask(self, ring_patch, patch_mask):
        """
        Classify ring colour using only pixels where patch_mask is active (255).
        This filters out the grey holder and other non-ring material.
        Uses mean colour of masked pixels.
        """
        if ring_patch.size == 0 or patch_mask.size == 0:
            return "unknown"

        # Get only pixels where mask is active (255)
        mask_pixels = patch_mask == 255
        masked_patch = ring_patch[mask_pixels]
        
        if len(masked_patch) < 10:
            return "unknown"

        # Get mean color of masked pixels
        mean_bgr = np.mean(masked_patch, axis=0).astype(np.uint8)
        
        # Convert BGR to HSV for classification
        bgr_array = np.uint8([[[mean_bgr[0], mean_bgr[1], mean_bgr[2]]]])
        hsv = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = hsv

        # Simple HSV-based classification
        if s < 50:  # Low saturation → black/grey
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:
            return "red"
        elif 10 <= h < 20:
            return "orange"
        elif 20 <= h < 32:  # Yellow
            return "yellow"
        elif 32 <= h < 85:  # Green
            return "green"
        elif 100 <= h < 130:
            return "blue"
        else:
            return "unknown"

    def _classify_ring_colour(self, ring_patch):
        """
        Classify ring colour using mean colour of bright pixels in patch.
        No sklearn required—uses only numpy and OpenCV.
        """
        if ring_patch.size == 0:
            return "unknown"

        # Reshape patch to list of BGR pixels
        pixels = ring_patch.reshape(-1, 3).astype(np.float32)

        # Remove near-black pixels (background/shadow)
        bright_mask = np.sum(pixels, axis=1) > 30
        pixels_fg = pixels[bright_mask]

        if len(pixels_fg) < 10:
            return "unknown"

        # Use mean colour of foreground pixels
        dominant_bgr = np.mean(pixels_fg, axis=0).astype(np.uint8)

        # Convert BGR to HSV for colour matching
        dominant_hsv = cv2.cvtColor(np.uint8([[dominant_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = dominant_hsv

        # Simple HSV-based classification
        if s < 50:  # Low saturation → black/white
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:
            return "red"
        elif 10 <= h < 20:
            return "orange"
        elif 20 <= h < 32:  # Yellow (narrower to avoid green confusion)
            return "yellow"
        elif 32 <= h < 85:  # Green (wider range, starts earlier)
            return "green"
        elif 100 <= h < 130:
            return "blue"
        else:
            return "unknown"

    def _extract_ring_colour_from_image(self, cx, cy, radius, image):
        """
        Extract ring color from image using the detected circle boundary.
        Samples pixels within the ring region to get the most accurate color.
        This is more reliable than point cloud because we know the exact ring pixels.
        """
        if image is None or image.size == 0:
            return "unknown"
        
        h, w = image.shape[:2]
        
        # Create circular mask for the ring (inner boundary to avoid background)
        # Sample from inner 70% to center, avoiding the outer edge which touches background
        mask = np.zeros((h, w), dtype=np.uint8)
        inner_radius = max(int(radius * 0.5), 1)  # Inner 50% of radius
        outer_radius = radius
        
        # Draw filled circle for outer boundary
        cv2.circle(mask, (cx, cy), outer_radius, 255, -1)
        # Draw filled circle for inner boundary (to exclude center)
        cv2.circle(mask, (cx, cy), inner_radius, 0, -1)
        
        # Extract pixels within the ring region
        ring_pixels = image[mask > 0]  # All pixels where mask is non-zero
        
        if len(ring_pixels) < 10:
            return "unknown"
        
        # Convert to float and reshape
        pixels = ring_pixels.reshape(-1, 3).astype(np.float32)
        
        # Remove near-black pixels (shadow at center)
        bright_mask = np.sum(pixels, axis=1) > 30
        pixels_fg = pixels[bright_mask]
        
        if len(pixels_fg) < 10:
            return "unknown"
        
        # Get mean color of ring pixels
        dominant_bgr = np.mean(pixels_fg, axis=0).astype(np.uint8)
        
        # Convert BGR to HSV for classification
        dominant_hsv = cv2.cvtColor(np.uint8([[dominant_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = dominant_hsv
        
        # HSV-based color classification
        if s < 50:  # Low saturation → black/white/gray
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:  # Red hues
            return "red"
        elif 10 <= h < 20:  # Orange-red
            return "orange"
        elif 20 <= h < 32:  # Yellow (narrower to avoid green)
            return "yellow"
        elif 32 <= h < 85:  # Green (wider range)
            return "green"
        elif 100 <= h < 130:  # Blue
            return "blue"
        else:
            return "unknown"

    def _classify_ring_colour(self, ring_patch):
        """
        Classify ring colour using mean colour of bright pixels in patch.
        No sklearn required—uses only numpy and OpenCV.
        """
        if ring_patch.size == 0:
            return "unknown"

        # Reshape patch to list of BGR pixels
        pixels = ring_patch.reshape(-1, 3).astype(np.float32)

        # Remove near-black pixels (background/shadow)
        bright_mask = np.sum(pixels, axis=1) > 30
        pixels_fg = pixels[bright_mask]

        if len(pixels_fg) < 10:
            return "unknown"

        # Use mean colour of foreground pixels
        dominant_bgr = np.mean(pixels_fg, axis=0).astype(np.uint8)

        # Convert BGR to HSV for colour matching
        dominant_hsv = cv2.cvtColor(np.uint8([[dominant_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = dominant_hsv

        # Simple HSV-based classification
        if s < 50:  # Low saturation → black/white
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:
            return "red"
        elif 10 <= h < 20:
            return "orange"
        elif 20 <= h < 32:  # Yellow (narrower to avoid green confusion)
            return "yellow"
        elif 32 <= h < 85:  # Green (wider range, starts earlier)
            return "green"
        elif 100 <= h < 130:
            return "blue"
        else:
            return "unknown"

    def _classify_colour_from_pc_rgb(self, rgb_val):
        """
        Classify ring color directly from point cloud RGB value.
        Extracts RGB from float32 packed integer format and classifies using HSV.
        Returns color name: red, orange, yellow, green, blue, black, or unknown.
        """
        try:
            # Reinterpret float32 bits as uint32 to get packed RGB
            if isinstance(rgb_val, (float, np.floating)):
                rgb_bits = np.float32(rgb_val)
                rgb_bytes = rgb_bits.tobytes()
                rgb_int = np.frombuffer(rgb_bytes, dtype=np.uint32)[0]
            else:
                rgb_int = int(rgb_val)
            
            # Extract R, G, B components from packed RGB (0x00RRGGBB format)
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = rgb_int & 0xFF
            
            # If all zeros, try BGR endian format
            if r == 0 and g == 0 and b == 0:
                r = rgb_int & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = (rgb_int >> 16) & 0xFF
            
            # Reject if all zeros or all 255 (invalid data)
            if (r == 0 and g == 0 and b == 0) or (r == 255 and g == 255 and b == 255):
                return "unknown"
            
            # Convert RGB to HSV (OpenCV expects BGR, so swap)
            bgr_array = np.uint8([[[b, g, r]]])
            hsv = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)[0, 0]
            h, s, v = hsv
            
            # Classification based on HSV hue and saturation
            if s < 30:  # Low saturation → black/white/gray
                return "black"
            elif v < 50:  # Very dark
                return "black"
            elif 0 <= h < 10 or 170 <= h <= 180:  # Red hues
                return "red"
            elif 10 <= h < 20:  # Orange-red
                return "orange"
            elif 20 <= h < 32:  # Yellow (narrower to avoid green confusion)
                return "yellow"
            elif 32 <= h < 85:  # Green (wider range, starts earlier)
                return "green"
            elif 100 <= h < 130:  # Blue
                return "blue"
            else:
                return "unknown"
                
        except Exception as e:
            self.get_logger().warn(f"Failed to classify PC RGB: {e}")
            return "unknown"

    def _detection_enabled(self):
        return self.robot_state in (
            "IDLE",
            "PATROL",
            "APPROACH_FACE",
            "INTERACT_FACE",
            "APPROACH_RING",
            "INTERACT_RING",
            "APPROACH_BARREL",
            "INTERACT_BARREL",
        )

    def _publish_front_barrel_marker(self, detection):
        if not self._detection_enabled():
            return
        if self.image_header is None:
            return

        point = detection["point"]
        r, g, b = detection.get("rgb", (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = detection["frame_id"]
        marker.header.stamp = self.image_header.stamp
        marker.ns = "front_barrel_raw"
        marker.type = Marker.CYLINDER
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=250_000_000)

        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.24
        marker.scale.y = 0.24
        marker.scale.z = 0.35
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        marker.text = "horizontal"

        self.front_barrel_pub.publish(marker)
        self.get_logger().info(
            f"Published front-facing barrel candidate: "
            f"pos=({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) "
            f"frame={marker.header.frame_id}"
        )

    def _publish_marker_raw(self, detection):
        """
        Publish raw detection to /ring_marker using 3D position and color from image.
        Uses patch_mask to extract color from only the valid ring pixels (filters grey holder).
        Ring_localizator will aggregate 10+ detections in 0.6m radius for confirmation.
        Only publishes while the behavior manager is in a detection-capable state.
        """
        if not self._detection_enabled():
            return
        
        if self.image_header is None:
            return

        # Extract color from ring patch using the mask (filters out grey holder)
        # This is more reliable than point cloud samples because we know the ring pixels
        colour = self._classify_ring_colour_with_mask(
            detection['ring_patch'], detection['patch_mask']
        )
        point = detection['point']

        r, g, b = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = detection['frame_id']
        marker.header.stamp = self.image_header.stamp  # Use image timestamp, not clock
        marker.ns = "ring_raw"
        marker.type = Marker.SPHERE
        marker.id = self.marker_id_counter  # Incrementing ID like face detector
        self.marker_id_counter += 1
        marker.action = Marker.ADD
        # Raw markers disappear after 0.2 seconds (let aggregator handle persistence)
        marker.lifetime = Duration(sec=0, nanosec=200_000_000)  # 200ms

        # Position from the median of valid annulus points in point-cloud frame.
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.15
        marker.scale.z = 0.08

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0
        marker.text = "raw"

        self.marker_pub.publish(marker)
        colour_msg = String()
        colour_msg.data = colour
        self.colour_pub.publish(colour_msg)
        self.get_logger().info(
            f"Published /ring_marker: id={marker.id}, "
            f"pos=({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) "
            f"frame={marker.header.frame_id}, colour={colour}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RingDetectorV2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
