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
HOUGH_MIN_RADIUS = 1          # Minimum circle radius in pixels (lowered to detect black ring)
HOUGH_MAX_RADIUS = 70          # Maximum circle radius (px)

# ── Cross-frame confirmation ──────────────────────────────────────────────────
CONFIRM_HITS = 11               # frames a ring must be seen before confirmed (strict but fair)
MAX_MISSED = 4                  # frames without match before dropping candidate (balanced)
MATCH_DIST_PX = 35              # pixel radius to match circles between frames

# ── Depth verification ────────────────────────────────────────────────────────
MAX_RANGE_M = 7.0               # beyond this → "nothing" / sky
RIM_SOLID_FRAC = 0.35           # fraction of rim samples that must be solid

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
        self.colour_pub = self.create_publisher(String, "/ring_colour", 10)

        # Debug windows
        cv2.namedWindow("Disparity", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Hough Circles", cv2.WINDOW_NORMAL)

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

        frame_detections = []
        if circles is not None:
            self.get_logger().info(f"Hough found {len(circles[0])} circles - publishing all")
            for circle in circles[0]:
                cx, cy, radius = int(circle[0]), int(circle[1]), int(circle[2])
                ring = self._evaluate_circle(cx, cy, radius, depth_m, self.rgb_image)
                if ring is not None:
                    frame_detections.append(ring)
            self.get_logger().info(f"Publishing {len(frame_detections)} detections")

        # ── Publish each valid detection IMMEDIATELY ───────────────────────────────────
        for detection in frame_detections:
            # Publish to /ring_marker (color will be inferred from point cloud RGB)
            # Let ring_localizator handle aggregation and confirmation
            self._publish_marker_raw(detection['cx'], detection['cy'], detection['depth_m'], detection['ring_patch'], detection['radius'])

        # Debug visualization
        debug_img = self.rgb_image.copy()
        if circles is not None:
            for circle in circles[0]:
                cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
                cv2.circle(debug_img, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(debug_img, (cx, cy), 2, (0, 0, 255), 3)

        cv2.imshow("Disparity", disparity_8u)
        cv2.imshow("Hough Circles", debug_img)
        cv2.waitKey(1)

    def robot_state_callback(self, data):
        """Update robot state for conditional marker publishing."""
        self.robot_state = data.data

    def _evaluate_circle(self, cx, cy, radius, depth_m, rgb_img):
        """
        Accept any circle Hough found - let ring_localizator handle robustness via aggregation.
        Just extract the ring patch for color classification.
        """
        h, w = depth_m.shape
        if not (radius < cx < w - radius and radius < cy < h - radius):
            return None

        # Get center depth for marker position
        centre_depth = depth_m[cy, cx]
        if centre_depth < 0.1 or centre_depth > MAX_RANGE_M:
            # Use a default distance if depth is invalid
            centre_depth = 1.0

        # Extract ring patch for colour classification
        patch_size = max(int(radius * 2.5), 64)
        x1 = max(0, cx - patch_size // 2)
        x2 = min(w, cx + patch_size // 2)
        y1 = max(0, cy - patch_size // 2)
        y2 = min(h, cy + patch_size // 2)
        ring_patch = rgb_img[y1:y2, x1:x2].copy()

        return {
            "cx": cx, "cy": cy, "radius": radius,
            "depth_m": centre_depth, "ring_patch": ring_patch
        }

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

    def _publish_marker_raw(self, cx, cy, depth_m, ring_patch, radius):
        """
        Publish raw detection to /ring_marker using 3D position and color from image.
        Uses disparity mask to extract color from within the ring boundary (most reliable).
        Ring_localizator will aggregate 10+ detections in 0.6m radius for confirmation.
        Only publishes if robot is in IDLE or PATROL state.
        """
        # Only publish when idle or on patrol - avoid interfering with other tasks
        if self.robot_state not in ['IDLE', 'PATROL']:
            return
        
        if self.image_header is None:
            return

        # Get 3D position from point cloud (accurate, in base_link frame)
        x, y, z = depth_m, 0.0, 0.0  # Fallback to simple depth
        used_pointcloud = False
        
        if self.pointcloud_xyz is not None:
            try:
                # Get 3D position from center or nearby points
                neighbors_for_position = [
                    (cy, cx),  # center
                    (cy, cx + 10),  # right
                    (cy, cx - 10),  # left
                    (cy + 10, cx),  # down
                    (cy - 10, cx),  # up
                ]
                
                for ny, nx in neighbors_for_position:
                    ny_clamped = max(0, min(ny, self.pointcloud_xyz.shape[0] - 1))
                    nx_clamped = max(0, min(nx, self.pointcloud_xyz.shape[1] - 1))
                    pt = self.pointcloud_xyz[int(ny_clamped), int(nx_clamped), :]
                    
                    if np.all(np.isfinite(pt)):
                        x, y, z = pt[0], pt[1], pt[2]
                        used_pointcloud = True
                        break
                        
            except Exception as e:
                self.get_logger().debug(f"Failed to get point cloud position: {e}")

        # Extract color from within the ring boundary using the image
        # This is more reliable than point cloud samples because we know the ring pixels
        colour = self._extract_ring_colour_from_image(cx, cy, radius, self.rgb_image)

        r, g, b = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = "base_link"
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
        pc_str = "from point cloud" if used_pointcloud else "fallback"
        self.get_logger().info(f"Published /ring_marker: id={marker.id}, pos=({x:.2f}, {y:.2f}, {z:.2f}) {pc_str}, colour={colour}")

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
