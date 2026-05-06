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

import copy
import os
import threading

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

CALIB_PATH = os.path.expanduser("~/.ros/ring_hsv_calibration.yaml")

# ── QoS ───────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── Hough circle tuning ───────────────────────────────────────────────────────
HOUGH_DP = 1                    # Inverse ratio of accumulator resolution
HOUGH_MIN_DIST = 80             # Minimum distance between circle centres
HOUGH_PARAM1 = 48               # Upper threshold for Canny edge (balanced strictness)
HOUGH_PARAM2 = 25               # Accumulator threshold (sim)
HOUGH_PARAM2_REAL = 40          # Accumulator threshold (real robot)
HOUGH_MIN_RADIUS = 1            # Minimum circle radius in pixels (lowered to detect black ring)
HOUGH_MAX_RADIUS = 150          # Maximum circle radius (px)

# ── Cross-frame confirmation (handled by ring_localizator) ────────────────────
MAX_MISSED    = 4               # frames without match before dropping candidate
MATCH_DIST_PX = 35              # pixel radius to match circles between frames

# ── Depth verification ────────────────────────────────────────────────────────
MAX_RANGE_M = 7.0               # beyond this → "nothing" / sky
RIM_SOLID_FRAC = 0.35           # fraction of rim samples that must be solid

# ── Colour HSV ranges (simulation) ────────────────────────────────────────────
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
    "black": [
        (np.array([0, 0, 0]), np.array([180, 255, 50])),
    ],
}

# ── Colour HSV ranges (real robot — looser S/V for real-world lighting) ────────
COLOUR_RANGES_REAL = {
    "red": [
        (np.array([0,   60,  40]), np.array([10,  255, 255])),
        (np.array([165, 60,  40]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([35,  50,  30]), np.array([90, 255, 255])),
    ],
    "blue": [
        (np.array([95,  50,  30]), np.array([145, 255, 255])),
    ],
    "black": [
        (np.array([0, 0, 0]), np.array([180, 255, 60])),
    ],
}

# ── Colour class names ────────────────────────────────────────────────────────
COLOUR_NAMES = ['red', 'green', 'blue', 'black']

# RViz marker colours (r, g, b) per ring colour
MARKER_COLOURS = {
    "red":   (1.0, 0.0, 0.0),
    "green": (0.0, 1.0, 0.0),
    "blue":  (0.0, 0.4, 1.0),
    "black": (0.1, 0.1, 0.1),
}


class RingDetectorV2(Node):
    def __init__(self):
        super().__init__('ring_detector_v2')
        self.bridge = CvBridge()
        self.depth_raw = None
        self.rgb_image = None
        self.image_header = None  # Store image header for marker timestamps
        self.pointcloud_xyz = None        # Point cloud XYZ positions
        self.pointcloud_rgb = None        # Point cloud RGB colors
        self.pointcloud_frame_id = 'base_link'  # overwritten from first PC message

        # Robot state for conditional publishing
        self.robot_state = 'IDLE'

        # Cross-frame candidates: list of {cx, cy, hits, missed, colour, depth_m}
        self._candidates = []

        # Marker ID counter
        self.marker_id_counter = 0

        # Topic selection: real robot vs simulation
        self.declare_parameter('real_robot', False)
        real = self.get_parameter('real_robot').get_parameter_value().bool_value

        if real:
            img_topic   = '/gemini/color/image_raw/compressed'
            depth_topic = '/gemini/depth/image_raw'
            pc_topic    = '/gemini/depth/points'
        else:
            img_topic   = '/oakd/rgb/preview/image_raw'
            depth_topic = '/oakd/rgb/preview/depth'
            pc_topic    = '/oakd/rgb/preview/depth/points'

        self.real_robot = real

        # Load calibrated HSV ranges (real robot) or fall back to hardcoded defaults
        if real:
            self.colour_ranges = self._load_colour_ranges()
        else:
            self.colour_ranges = {
                c: [(lo, hi) for lo, hi in ranges]
                for c, ranges in COLOUR_RANGES.items()
            }

        # Real robot: camera intrinsics for depth back-projection
        self.camera_intrinsics = None   # (fx, fy, ppx, ppy)
        self.camera_frame_id   = 'base_link'

        self.get_logger().info(
            f"{'[REAL]' if real else '[SIM]'} "
            f"image={img_topic}  depth={depth_topic}  pc={pc_topic}")

        # Subscriptions — real robot uses compressed topic to cut WiFi bandwidth ~15x
        img_cb = self.image_compressed_callback if real else self.image_callback
        img_msg_type = CompressedImage if real else Image
        self.image_sub = self.create_subscription(
            img_msg_type, img_topic, img_cb, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, SENSOR_QOS)
        if not real:
            self.pointcloud_sub = self.create_subscription(
                PointCloud2, pc_topic, self.pointcloud_callback, SENSOR_QOS)

        if real:
            from sensor_msgs.msg import CameraInfo
            self.camera_info_sub = self.create_subscription(
                CameraInfo, '/gemini/color/camera_info',
                self._real_camera_info_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self.robot_state_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, "/ring_marker", 10)
        self.colour_pub = self.create_publisher(String, "/ring_colour", 10)

        self.combined_mask = None

        # Debug images written by worker, displayed by main-thread timer (OpenCV GUI is not thread-safe)
        self._debug_img      = None
        self._debug_mask     = None
        self._debug_depth    = None
        self._debug_img_lock = threading.Lock()

        # Worker thread — detection runs here so image_callback never blocks the executor
        self._latest_rgb  = None
        self._frame_lock  = threading.Lock()
        self._frame_event = threading.Event()
        self._worker = threading.Thread(target=self._detection_worker, daemon=True)
        self._worker.start()

        # Timer to push debug images to OpenCV windows from the ROS executor thread
        self.create_timer(0.05, self._display_timer_callback)

        self.get_logger().info("RingDetector V2 ready (Hough circle mode).")

    def depth_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # Normalise to uint16 mm regardless of what the camera sends
            if img.dtype == np.float32 or img.dtype == np.float64:
                # metres → millimetres
                self.depth_raw = (img * 1000.0).astype(np.uint16)
            else:
                self.depth_raw = img.astype(np.uint16)
            if not hasattr(self, '_depth_encoding_logged'):
                self.get_logger().info(
                    f"Depth encoding: {data.encoding}  dtype: {img.dtype}  "
                    f"shape: {img.shape}")
                self._depth_encoding_logged = True
        except CvBridgeError as e:
            self.get_logger().error(f"depth_callback CvBridgeError: {e}", throttle_duration_sec=5.0)

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
            self.pointcloud_frame_id = data.header.frame_id.lstrip('/')

            # Try to extract RGB field
            for field_name in ["rgb", "rgba", "RGB"]:
                try:
                    pts_rgb = pc2.read_points_numpy(data, field_names=(field_name,))
                    if pts_rgb is not None and len(pts_rgb) > 0:
                        self.pointcloud_rgb = pts_rgb.reshape((data.height, data.width))
                        return
                except Exception:
                    pass

            # If no RGB field found, will use image patch color fallback
            self.pointcloud_rgb = None

        except Exception as e:
            self.get_logger().error(f"✗ Point cloud failed: {e}")

    def image_callback(self, data):
        """Snapshot the latest frame and wake the detection worker. Never blocks."""
        try:
            rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return
        self.image_header = data.header
        with self._frame_lock:
            self._latest_rgb = rgb
        self._frame_event.set()  # worker discards old unprocessed frames automatically

    def image_compressed_callback(self, data):
        """Decode CompressedImage — used on real robot to reduce WiFi bandwidth."""
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is None:
                return
        except Exception as e:
            self.get_logger().error(f'Compressed image decode error: {e}')
            return
        self.image_header = data.header
        with self._frame_lock:
            self._latest_rgb = rgb
        self._frame_event.set()

    def _detection_worker(self):
        """All heavy detection runs here — off the ROS executor thread."""
        while True:
            self._frame_event.wait()
            self._frame_event.clear()
            try:
                self._process_frame()
            except Exception as e:
                self.get_logger().error(f"Detection worker error: {e}", throttle_duration_sec=5.0)

    def _process_frame(self):
        with self._frame_lock:
            cv_image = self._latest_rgb
        depth_snapshot = self.depth_raw  # atomic Python reference read (GIL-safe)

        if cv_image is None or depth_snapshot is None:
            if cv_image is not None:
                self.get_logger().warn(
                    "Waiting for depth data", throttle_duration_sec=3.0)
            return

        # Convert depth to disparity (inverse, normalized)
        depth_m = depth_snapshot.astype(np.float32) / 1000.0  # mm → m
        with np.errstate(divide='ignore', invalid='ignore'):
            disparity = np.where(depth_m > 0, 1.0 / depth_m, 0)
        disparity_8u = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        disparity_8u = cv2.GaussianBlur(disparity_8u, (5, 5), 0)

        # Disparity mask: pixels with valid depth
        disparity_mask = (disparity > 0).astype(np.uint8) * 255

        if self.real_robot:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            colour_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            for ranges in self.colour_ranges.values():
                for lo, hi in ranges:
                    colour_mask |= cv2.inRange(hsv, lo, hi)
            colour_mask = cv2.morphologyEx(
                colour_mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
            hough_input = disparity_8u
            param2 = HOUGH_PARAM2_REAL
        else:
            hough_input = disparity_8u
            colour_mask = None
            param2 = HOUGH_PARAM2

        # Hough circle detection
        circles = cv2.HoughCircles(
            hough_input,
            cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP,
            minDist=HOUGH_MIN_DIST,
            param1=HOUGH_PARAM1,
            param2=param2,
            minRadius=HOUGH_MIN_RADIUS,
            maxRadius=HOUGH_MAX_RADIUS,
        )

        frame_detections = []
        if circles is not None:
            self.get_logger().info(f"Hough found {len(circles[0])} circles - publishing all")
            for circle in circles[0]:
                cx, cy, radius = int(circle[0]), int(circle[1]), int(circle[2])
                ring = self._evaluate_circle(cx, cy, radius, depth_m, cv_image, disparity_mask)
                if ring is not None:
                    frame_detections.append(ring)
            self.get_logger().info(f"Publishing {len(frame_detections)} detections")

        # ── Publish each valid detection ──────────────────────────────────────
        for detection in frame_detections:
            self._publish_marker_raw(detection['cx'], detection['cy'], detection['depth_m'],
                                     detection['ring_patch'], detection['radius'],
                                     detection['patch_mask'], colour_mask)

        # Debug visualization
        debug_img = cv_image.copy()
        if circles is not None:
            for circle in circles[0]:
                cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
                cv2.circle(debug_img, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(debug_img, (cx, cy), 2, (0, 255, 0), 3)

        mask_bgr = cv2.cvtColor(colour_mask, cv2.COLOR_GRAY2BGR) if colour_mask is not None else None
        with self._debug_img_lock:
            self._debug_img   = debug_img
            self._debug_mask  = mask_bgr
            self._debug_depth = cv2.cvtColor(disparity_8u, cv2.COLOR_GRAY2BGR)

    def _display_timer_callback(self):
        """Push latest debug images to OpenCV windows — always called from the ROS executor thread."""
        with self._debug_img_lock:
            debug_img = self._debug_img
            debug_mask = self._debug_mask
            debug_depth = self._debug_depth
        try:
            if debug_img is not None:
                cv2.imshow("Hough Circles", debug_img)
            if self.real_robot and debug_mask is not None:
                cv2.imshow("Colour mask (real)", debug_mask)
            if debug_depth is not None:
                cv2.imshow("Disparity", debug_depth)
            cv2.waitKey(1)
        except Exception:
            pass

    def robot_state_callback(self, data):
        """Update robot state for conditional marker publishing."""
        self.robot_state = data.data

    def _evaluate_circle(self, cx, cy, radius, depth_m, rgb_img, disparity_mask):
        """
        Accept any circle Hough found - let ring_localizator handle robustness via aggregation.
        Extract ring patch for color classification, using combined dilated ring + disparity mask.
        """
        h, w = depth_m.shape
        if not (radius < cx < w - radius and radius < cy < h - radius):
            return None

        # Hollowness check: sample rim depth and centre depth.
        # A real ring's centre (through the hole) must be farther than its rim,
        # or have mostly invalid depth (you see through to background).
        n_rim = 16
        rim_depths = []
        for k in range(n_rim):
            angle = 2 * np.pi * k / n_rim
            rx = int(np.clip(cx + radius * np.cos(angle), 0, w - 1))
            ry = int(np.clip(cy + radius * np.sin(angle), 0, h - 1))
            d = depth_m[ry, rx]
            if d > 0.1:
                rim_depths.append(d)

        if len(rim_depths) < n_rim // 3:
            return None  # too few valid rim points — not a solid ring edge

        rim_median = float(np.median(rim_depths))

        inner_r = max(int(radius * 0.4), 2)
        centre_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(centre_mask, (cx, cy), inner_r, 255, -1)
        centre_pixels = depth_m[centre_mask > 0]
        invalid_frac = float(np.mean(centre_pixels < 0.1))
        valid_centre = centre_pixels[centre_pixels > 0.1]
        centre_median = float(np.median(valid_centre)) if len(valid_centre) > 0 else MAX_RANGE_M

        # Relaxed hollowness: real rings hang close to walls so depth difference can be tiny.
        # Accept if centre is even slightly farther, or if >10% of centre pixels are invalid.
        hollow = invalid_frac > 0.10 or centre_median > rim_median + 0.02
        if not hollow:
            return None

        centre_depth = rim_median

        # Create individual ring mask for this circle
        ring_circle_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(ring_circle_mask, (cx, cy), radius, 255, -1)

        # Dilate ring mask by 20% (to expand ring boundaries)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated_circle_mask = cv2.dilate(ring_circle_mask, kernel, iterations=2)

        # Combine: dilated ring circle mask AND disparity mask (valid depth)
        combined_circle_mask = cv2.bitwise_and(dilated_circle_mask, disparity_mask)

        # Extract ring patch for colour classification
        patch_size = max(int(radius * 2.5), 64)
        x1 = max(0, cx - patch_size // 2)
        x2 = min(w, cx + patch_size // 2)
        y1 = max(0, cy - patch_size // 2)
        y2 = min(h, cy + patch_size // 2)

        ring_patch = rgb_img[y1:y2, x1:x2].copy()
        patch_mask = combined_circle_mask[y1:y2, x1:x2]

        return {
            "cx": cx, "cy": cy, "radius": radius,
            "depth_m": centre_depth, "ring_patch": ring_patch,
            "patch_mask": patch_mask
        }

    def _classify_ring_colour_with_mask(self, ring_patch, patch_mask):
        """
        Classify ring colour by counting how many ring pixels (patch_mask==255)
        match each colour's calibrated HSV range.  Uses self.colour_ranges which
        is loaded from ~/.ros/ring_hsv_calibration.yaml if available.
        """
        if ring_patch.size == 0 or patch_mask.size == 0:
            return "unknown"

        roi_mask = (patch_mask == 255).astype(np.uint8) * 255
        total = int(roi_mask.sum() // 255)
        if total < 10:
            return "unknown"

        hsv_patch = cv2.cvtColor(ring_patch, cv2.COLOR_BGR2HSV)

        best_colour = "unknown"
        best_count  = 0

        # Check all colours except black — black is determined by exclusion
        for colour, ranges in self.colour_ranges.items():
            if colour == "black":
                continue
            colour_mask = np.zeros(ring_patch.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                colour_mask |= cv2.inRange(hsv_patch, np.array(lo), np.array(hi))
            count = int(cv2.bitwise_and(colour_mask, roi_mask).sum() // 255)
            if count > best_count:
                best_count  = count
                best_colour = colour

        # Require at least 8% of masked pixels to match a colour
        if best_count >= max(10, int(total * 0.08)):
            return best_colour

        # No colour matched well enough — classify as black by exclusion
        return "black"

    def _classify_ring_colour(self, ring_patch):
        """
        Classify ring colour using mean colour of bright pixels in patch.
        No sklearn required—uses only numpy and OpenCV.
        """
        if ring_patch.size == 0:
            return "unknown"

        pixels = ring_patch.reshape(-1, 3).astype(np.float32)
        bright_mask = np.sum(pixels, axis=1) > 30
        pixels_fg = pixels[bright_mask]

        if len(pixels_fg) < 10:
            return "unknown"

        dominant_bgr = np.mean(pixels_fg, axis=0).astype(np.uint8)
        dominant_hsv = cv2.cvtColor(np.uint8([[dominant_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = dominant_hsv

        if s < 50:
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:
            return "red"
        elif 32 <= h < 85:
            return "green"
        elif 100 <= h < 130:
            return "blue"
        else:
            return "unknown"

    def _extract_ring_colour_from_image(self, cx, cy, radius, image):
        """
        Extract ring color from image using the detected circle boundary.
        Samples pixels within the ring region to get the most accurate color.
        """
        if image is None or image.size == 0:
            return "unknown"

        h, w = image.shape[:2]

        mask = np.zeros((h, w), dtype=np.uint8)
        inner_radius = max(int(radius * 0.5), 1)
        outer_radius = radius

        cv2.circle(mask, (cx, cy), outer_radius, 255, -1)
        cv2.circle(mask, (cx, cy), inner_radius, 0, -1)

        ring_pixels = image[mask > 0]

        if len(ring_pixels) < 10:
            return "unknown"

        pixels = ring_pixels.reshape(-1, 3).astype(np.float32)
        bright_mask = np.sum(pixels, axis=1) > 30
        pixels_fg = pixels[bright_mask]

        if len(pixels_fg) < 10:
            return "unknown"

        dominant_bgr = np.mean(pixels_fg, axis=0).astype(np.uint8)
        dominant_hsv = cv2.cvtColor(np.uint8([[dominant_bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
        h, s, v = dominant_hsv

        if s < 50:
            return "black"
        elif 0 <= h < 10 or 170 <= h <= 180:
            return "red"
        elif 32 <= h < 85:
            return "green"
        elif 100 <= h < 130:
            return "blue"
        else:
            return "unknown"

    def _classify_colour_from_pc_rgb(self, rgb_val):
        """
        Classify ring color directly from point cloud RGB value.
        Extracts RGB from float32 packed integer format and classifies using HSV.
        """
        try:
            if isinstance(rgb_val, (float, np.floating)):
                rgb_bits = np.float32(rgb_val)
                rgb_bytes = rgb_bits.tobytes()
                rgb_int = np.frombuffer(rgb_bytes, dtype=np.uint32)[0]
            else:
                rgb_int = int(rgb_val)

            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = rgb_int & 0xFF

            # If all zeros, try BGR endian format
            if r == 0 and g == 0 and b == 0:
                r = rgb_int & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = (rgb_int >> 16) & 0xFF

            if (r == 0 and g == 0 and b == 0) or (r == 255 and g == 255 and b == 255):
                return "unknown"

            bgr_array = np.uint8([[[b, g, r]]])
            hsv = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2HSV)[0, 0]
            h, s, v = hsv

            if s < 30:
                return "black"
            elif v < 50:
                return "black"
            elif 0 <= h < 10 or 170 <= h <= 180:
                return "red"
            elif 32 <= h < 85:
                return "green"
            elif 100 <= h < 130:
                return "blue"
            else:
                return "unknown"

        except Exception as e:
            self.get_logger().warn(f"Failed to classify PC RGB: {e}")
            return "unknown"

    def _load_colour_ranges(self):
        """
        Load HSV ranges from CALIB_PATH if it exists, otherwise fall back to
        COLOUR_RANGES_REAL.  Returned dict maps colour name → list of (lo, hi)
        numpy-array tuples, matching the format of COLOUR_RANGES_REAL.
        """
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH) as f:
                    data = yaml.safe_load(f)
                result = {
                    colour: [(np.array(r['lo']), np.array(r['hi'])) for r in ranges]
                    for colour, ranges in data.items()
                }
                self.get_logger().info(
                    f"[REAL] Loaded HSV calibration from {CALIB_PATH}")
                return result
            except Exception as e:
                self.get_logger().warn(
                    f"Could not load HSV calibration ({e}) — using built-in defaults")
        self.get_logger().info(
            f"[REAL] No calibration file found at {CALIB_PATH} — using built-in defaults")
        return {c: list(r) for c, r in COLOUR_RANGES_REAL.items()}

    def _real_camera_info_callback(self, data):
        """Store camera intrinsics and frame_id once."""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = (data.k[0], data.k[4], data.k[2], data.k[5])
            self.camera_frame_id   = data.header.frame_id.lstrip('/')
            self.get_logger().info(
                f"[REAL] Camera intrinsics: fx={data.k[0]:.1f} fy={data.k[4]:.1f} "
                f"cx={data.k[2]:.1f} cy={data.k[5]:.1f}  frame={self.camera_frame_id}")

    def _real_get_3d_from_colour_mask(self, cx, cy, radius, colour_mask):
        """
        Back-project the ring to 3D using depth pixels that are inside the ring's
        circle area and on the colour mask. Fully vectorized — no Python pixel loop.
        """
        if self.depth_raw is None or self.camera_intrinsics is None or colour_mask is None:
            return None

        dh, dw = self.depth_raw.shape[:2]
        ih, iw = colour_mask.shape[:2]

        # Build a filled circle mask in image space
        circle_mask = np.zeros((ih, iw), dtype=np.uint8)
        cv2.circle(circle_mask, (cx, cy), radius, 255, -1)

        # Intersect with colour mask → only ring-coloured pixels inside the circle
        ring_pixels_mask = cv2.bitwise_and(circle_mask, colour_mask)

        ys, xs = np.where(ring_pixels_mask > 0)
        if len(ys) == 0:
            return None

        fx, fy, ppx, ppy = self.camera_intrinsics

        # Scale image pixel coords to depth image coords (vectorized)
        dxs = np.clip((xs * dw / iw).astype(int), 0, dw - 1)
        dys = np.clip((ys * dh / ih).astype(int), 0, dh - 1)

        # Read all depth values at once
        zs = self.depth_raw[dys, dxs].astype(np.float32)
        if self.depth_raw.dtype == np.uint16:
            zs /= 1000.0

        valid = (zs > 0.1) & np.isfinite(zs)
        if valid.sum() < 5:
            return None

        x3d = (xs[valid] - ppx) * zs[valid] / fx
        y3d = (ys[valid] - ppy) * zs[valid] / fy

        return float(np.median(x3d)), float(np.median(y3d)), float(np.median(zs[valid]))

    def _publish_marker_raw(self, cx, cy, depth_m, ring_patch, radius, patch_mask, colour_mask=None):
        """
        Publish raw detection to /ring_marker using 3D position and color from image.
        Uses patch_mask to extract color from only the valid ring pixels (filters grey holder).
        Ring_localizator will aggregate 10+ detections in 0.6m radius for confirmation.
        Only publishes if robot is in IDLE or PATROL state.
        """
        if self.robot_state not in ['IDLE', 'PATROL']:
            return

        if self.image_header is None:
            return

        x, y, z = None, None, None

        if self.real_robot:
            # ── REAL: back-project coloured ring pixels from depth image ─────
            result = self._real_get_3d_from_colour_mask(cx, cy, radius, colour_mask)
            if result is not None:
                x, y, z = result
            elif self.camera_intrinsics is not None and depth_m > 0.1:
                # Fallback: back-project centre pixel directly
                fx, fy, ppx, ppy = self.camera_intrinsics
                z = depth_m
                x = (cx - ppx) * z / fx
                y = (cy - ppy) * z / fy
            marker_frame = self.camera_frame_id
        else:
            # ── SIM: sample rim in organized point cloud ──────────────────────
            if self.pointcloud_xyz is not None:
                try:
                    pc_h, pc_w = self.pointcloud_xyz.shape[:2]
                    rim_pts = []
                    for k in range(12):
                        angle = 2 * np.pi * k / 12
                        rx = int(np.clip(cx + radius * np.cos(angle), 0, pc_w - 1))
                        ry = int(np.clip(cy + radius * np.sin(angle), 0, pc_h - 1))
                        pt = self.pointcloud_xyz[ry, rx, :]
                        if np.all(np.isfinite(pt)) and np.any(pt != 0):
                            rim_pts.append(pt)
                    if len(rim_pts) >= 3:
                        rim_arr = np.array(rim_pts)
                        x = float(np.median(rim_arr[:, 0]))
                        y = float(np.median(rim_arr[:, 1]))
                        z = float(np.median(rim_arr[:, 2]))
                except Exception as e:
                    self.get_logger().debug(f"Failed to get rim point cloud position: {e}")
            marker_frame = self.pointcloud_frame_id

        if x is None or y is None or z is None:
            return

        dist = float(np.sqrt(x**2 + y**2 + z**2))
        if self.real_robot and dist > 2.0:
            self.get_logger().debug(f"Ring too far ({dist:.2f} m), skipping")
            return

        colour = self._classify_ring_colour_with_mask(ring_patch, patch_mask)
        r, g, b = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.image_header.stamp  # Use image timestamp, not clock
        marker.type = Marker.CYLINDER
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.action = Marker.ADD
        # Raw markers disappear after 0.2 seconds (let aggregator handle persistence)
        marker.lifetime = Duration(sec=0, nanosec=200_000_000)  # 200ms

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.15
        marker.scale.z = 0.05

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published /ring_marker: id={marker.id}, pos=({x:.2f}, {y:.2f}, {z:.2f}), colour={colour}")

    def _publish_marker(self, ring):
        """Publish confirmed ring marker. Only publishes if robot is in IDLE or PATROL state."""
        if self.robot_state not in ['IDLE', 'PATROL']:
            return

        r, g, b = MARKER_COLOURS.get(ring["colour"], (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.id = id(ring)  # Use object id as unique marker id

        marker.pose.position.x = ring["depth_m"]
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.2
        marker.scale.z = 0.05

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        colour_msg = String()
        colour_msg.data = ring["colour"]
        self.colour_pub.publish(colour_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RingDetectorV2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
