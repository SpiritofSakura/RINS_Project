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
HOUGH_PARAM2 = 25               # Accumulator threshold (sim)
HOUGH_PARAM2_REAL = 30          # Accumulator threshold (real robot)
HOUGH_MIN_RADIUS = 1            # Minimum circle radius in pixels (lowered to detect black ring)
HOUGH_MAX_RADIUS = 70           # Maximum circle radius (px)

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
    "yellow": [
        (np.array([18,  60,  60]), np.array([38, 255, 255])),
    ],
    "orange": [
        (np.array([8,   80,  50]), np.array([22, 255, 255])),
    ],
    "black": [
        (np.array([0, 0, 0]), np.array([180, 255, 60])),
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
            img_topic   = '/gemini/color/image_raw'
            depth_topic = '/gemini/depth/image_raw'
            pc_topic    = '/gemini/depth/points'
        else:
            img_topic   = '/oakd/rgb/preview/image_raw'
            depth_topic = '/oakd/rgb/preview/depth'
            pc_topic    = '/oakd/rgb/preview/depth/points'

        self.real_robot = real

        # Real robot: camera intrinsics for depth back-projection
        self.camera_intrinsics = None   # (fx, fy, ppx, ppy)
        self.camera_frame_id   = 'base_link'

        self.get_logger().info(
            f"{'[REAL]' if real else '[SIM]'} "
            f"image={img_topic}  depth={depth_topic}  pc={pc_topic}")

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, img_topic, self.image_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, SENSOR_QOS)
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

        # Debug windows
        cv2.namedWindow("Hough Circles", cv2.WINDOW_NORMAL)
        if self.real_robot:
            cv2.namedWindow("Colour mask (real)", cv2.WINDOW_NORMAL)
        
        # Store combined mask for color extraction
        self.combined_mask = None

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
            self.get_logger().warn(
                f"Waiting for data: depth={'ok' if self.depth_raw is not None else 'MISSING'}  "
                f"rgb={'ok' if self.rgb_image is not None else 'MISSING'}",
                throttle_duration_sec=3.0)
            return

        # Convert depth to disparity (inverse, normalized)
        depth_m = self.depth_raw.astype(np.float32) / 1000.0  # mm → m
        with np.errstate(divide='ignore', invalid='ignore'):
            disparity = np.where(depth_m > 0, 1.0 / depth_m, 0)
        disparity_8u = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        disparity_8u = cv2.GaussianBlur(disparity_8u, (5, 5), 0)

        # Disparity mask: pixels with valid depth
        disparity_mask = (disparity > 0).astype(np.uint8) * 255

        if self.real_robot:
            # ── REAL ROBOT: colour-mask the disparity before Hough ───────────
            # Build a combined HSV colour mask covering all ring colours
            hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
            colour_mask = np.zeros(self.rgb_image.shape[:2], dtype=np.uint8)
            for ranges in COLOUR_RANGES_REAL.values():
                for lo, hi in ranges:
                    colour_mask |= cv2.inRange(hsv, lo, hi)
            # Close small gaps (pole occlusion etc.)
            colour_mask = cv2.morphologyEx(
                colour_mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
            # Only keep disparity pixels that sit on a coloured region
            hough_input = cv2.bitwise_and(disparity_8u, disparity_8u, mask=colour_mask)
            param2 = HOUGH_PARAM2_REAL
        else:
            # ── SIMULATION: run Hough on full disparity image ────────────────
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
        
        img_height = disparity_8u.shape[0]
        frame_detections = []
        if circles is not None:
            self.get_logger().info(f"Hough found {len(circles[0])} circles - publishing all")
            for circle in circles[0]:
                cx, cy, radius = int(circle[0]), int(circle[1]), int(circle[2])
                if self.real_robot and cy >= img_height // 2:
                    continue  # on real robot, only detect rings in top half of image
                ring = self._evaluate_circle(cx, cy, radius, depth_m, self.rgb_image, disparity_mask)
                if ring is not None:
                    frame_detections.append(ring)
            self.get_logger().info(f"Publishing {len(frame_detections)} detections")

        # ── Publish each valid detection IMMEDIATELY ───────────────────────────────────
        for detection in frame_detections:
            # Publish to /ring_marker (color will be inferred from patch mask)
            # Let ring_localizator handle aggregation and confirmation
            self._publish_marker_raw(detection['cx'], detection['cy'], detection['depth_m'],
                                     detection['ring_patch'], detection['radius'], detection['patch_mask'],
                                     colour_mask)

        # Debug visualization
        debug_img = self.rgb_image.copy()
        ring_mask = np.zeros(self.rgb_image.shape[:2], dtype=np.uint8)
        combined_visualization_mask = np.zeros(self.rgb_image.shape[:2], dtype=np.uint8)
        if circles is not None:
            for circle in circles[0]:
                cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
                filtered_out = self.real_robot and cy >= img_height // 2
                colour = (0, 0, 255) if filtered_out else (0, 255, 0)
                cv2.circle(debug_img, (cx, cy), r, colour, 2)
                cv2.circle(debug_img, (cx, cy), 2, colour, 3)
                # Create mask only for circles that passed the filter
                if not filtered_out:
                    cv2.circle(ring_mask, (cx, cy), r, 255, -1)
        
        # Dilate ring mask by 20% (to expand ring boundaries)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated_ring_mask = cv2.dilate(ring_mask, kernel, iterations=2)
        
        # Combine: dilated ring mask AND disparity mask (valid depth)
        combined_mask_display = cv2.bitwise_and(dilated_ring_mask, disparity_mask)
        
        # Apply combined mask (dilated ring + disparity) to RGB image for visualization
        masked_view = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=combined_mask_display)
        
        # Combine ring mask with color+disparity mask to see intersection
        combined_visualization = cv2.bitwise_and(ring_mask, combined_mask_display)

        cv2.imshow("Hough Circles", debug_img)
        # if self.real_robot and colour_mask is not None:
        #     cv2.imshow("Colour mask (real)", cv2.cvtColor(colour_mask, cv2.COLOR_GRAY2BGR))
        cv2.waitKey(1)

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

        # Get center depth for marker position
        centre_depth = depth_m[cy, cx]
        if centre_depth < 0.1 or centre_depth > MAX_RANGE_M:
            # Use a default distance if depth is invalid
            centre_depth = 1.0

        # Create individual ring mask for this circle
        ring_circle_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(ring_circle_mask, (cx, cy), radius, 255, -1)
        
        # Dilate ring mask by 20% (to expand ring boundaries)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated_circle_mask = cv2.dilate(ring_circle_mask, kernel, iterations=2)
        
        # Combine: dilated ring circle mask AND disparity mask (valid depth)
        combined_circle_mask = cv2.bitwise_and(dilated_circle_mask, disparity_mask)

        # Extract ring patch for colour classification
        # Use combined_mask (dilated ring + disparity) to get only valid ring pixels
        patch_size = max(int(radius * 2.5), 64)
        x1 = max(0, cx - patch_size // 2)
        x2 = min(w, cx + patch_size // 2)
        y1 = max(0, cy - patch_size // 2)
        y2 = min(h, cy + patch_size // 2)
        
        # Extract patch and its corresponding mask
        ring_patch = rgb_img[y1:y2, x1:x2].copy()
        patch_mask = combined_circle_mask[y1:y2, x1:x2]

        return {
            "cx": cx, "cy": cy, "radius": radius,
            "depth_m": centre_depth, "ring_patch": ring_patch,
            "patch_mask": patch_mask  # Pass mask for color extraction
        }

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
        Back-project the ring to 3D using depth pixels that are:
          - inside the ring's circle area
          - coloured (on the colour mask) → guaranteed ring material, not hollow
        Returns (x, y, z) in camera frame, or None if not enough valid pixels.
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
        valid_pts = []
        for px, py in zip(xs, ys):
            # Scale to depth image coords if resolutions differ
            dx = int(np.clip(px * dw / iw, 0, dw - 1))
            dy = int(np.clip(py * dh / ih, 0, dh - 1))
            raw = self.depth_raw[dy, dx]
            z = float(raw) / 1000.0 if self.depth_raw.dtype == np.uint16 else float(raw)
            if z > 0.1 and np.isfinite(z):
                x3d = (px - ppx) * z / fx
                y3d = (py - ppy) * z / fy
                valid_pts.append((x3d, y3d, z))

        if len(valid_pts) < 5:
            return None

        arr = np.array(valid_pts)
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1])), float(np.median(arr[:, 2]))

    def _publish_marker_raw(self, cx, cy, depth_m, ring_patch, radius, patch_mask, colour_mask=None):
        """
        Publish raw detection to /ring_marker using 3D position and color from image.
        Uses patch_mask to extract color from only the valid ring pixels (filters grey holder).
        Ring_localizator will aggregate 10+ detections in 0.6m radius for confirmation.
        Only publishes if robot is in IDLE or PATROL state.
        """
        # Only publish when idle or on patrol - avoid interfering with other tasks
        if self.robot_state not in ['IDLE', 'PATROL']:
            return
        
        if self.image_header is None:
            return

        # Get 3D position — real robot uses depth image + colour mask,
        # sim uses the organized point cloud rim samples.
        x, y, z = None, None, None

        if self.real_robot:
            # ── REAL: back-project coloured ring pixels from depth image ─────
            result = self._real_get_3d_from_colour_mask(cx, cy, radius, colour_mask)
            if result is not None:
                x, y, z = result
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

        # Reject detections too far from the robot (depth unreliable at range)
        dist = float(np.sqrt(x**2 + y**2 + z**2))
        if self.real_robot and dist > 2.0:
            self.get_logger().debug(f"Ring too far ({dist:.2f} m), skipping")
            return

        # Extract color from ring patch using the mask (filters out grey holder)
        # This is more reliable than point cloud samples because we know the ring pixels
        colour = self._classify_ring_colour_with_mask(ring_patch, patch_mask)

        r, g, b = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = marker_frame
        marker.header.stamp = self.image_header.stamp  # Use image timestamp, not clock
        marker.type = Marker.CYLINDER
        marker.id = self.marker_id_counter  # Incrementing ID like face detector
        self.marker_id_counter += 1
        marker.action = Marker.ADD
        # Raw markers disappear after 0.2 seconds (let aggregator handle persistence)
        marker.lifetime = Duration(sec=0, nanosec=200_000_000)  # 200ms

        # Position from point cloud (accurate 3D in base_link frame)
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
        # Only publish when idle or on patrol - avoid interfering with other tasks
        if self.robot_state not in ['IDLE', 'PATROL']:
            return
        
        r, g, b = MARKER_COLOURS.get(ring["colour"], (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.id = id(ring)  # Use object id as unique marker id

        # Position relative to camera
        marker.pose.position.x = ring["depth_m"]
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = 0.2
        marker.scale.z = 0.05

        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        # Publish colour as string
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
