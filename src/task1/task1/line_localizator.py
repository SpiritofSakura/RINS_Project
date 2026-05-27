import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

DEFAULT_RANGES = {
    "red": [
        ([0, 100, 80], [10, 255, 255]),
        ([170, 100, 80], [180, 255, 255]),
    ],
    "green": [
        ([40, 100, 80], [80, 255, 255]),
    ],
    "blue": [
        ([100, 100, 80], [130, 255, 255]),
    ],
    "yellow": [
        ([20, 100, 80], [35, 255, 255]),
    ],
}

COLORS = list(DEFAULT_RANGES.keys())

MARKER_COLORS = {
    "red":    (1.0, 0.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
    "blue":   (0.0, 0.4, 1.0),
    "yellow": (1.0, 1.0, 0.0),
}

MIN_LINE_AREA = 80
MIN_ASPECT_RATIO = 3.0
MAX_RANGE_M = 7.0
MIN_POINTS_3D = 15
MORPH_KERNEL = 5


class LineLocalizator(Node):
    def __init__(self):
        super().__init__("line_localizator")

        self.bridge = CvBridge()
        self.depth_raw = None
        self.pointcloud_xyz = None
        self.pointcloud_frame_id = None
        self.image_header = None
        self.robot_state = 'IDLE'

        self.declare_parameter("max_flat_std", 0.03)

        self._num_ranges = {}
        for color in COLORS:
            ranges = DEFAULT_RANGES[color]
            self._num_ranges[color] = len(ranges)
            for i, (lo, hi) in enumerate(ranges):
                idx = i + 1
                self.declare_parameter(f"{color}_lo{idx}", lo)
                self.declare_parameter(f"{color}_hi{idx}", hi)
        self._load_params()

        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.image_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/depth", self.depth_callback, SENSOR_QOS)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, SENSOR_QOS)
        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self.robot_state_callback, 10)

        self.marker_pub = self.create_publisher(Marker, "/line_markers", 10)

        self.marker_id_counter = 0

        self.get_logger().info("LineLocalizator ready.")

    _INACTIVE_STATES = frozenset((
        'FINISHING_ROUNDS', 'LINE_FOLLOWING', 'FOLLOW_BLUE_LINE',
        'BLUE_LINE_SEARCH', 'BLUE_LINE_FOLLOW', 'BLUE_LINE_DEAD_END',
    ))

    def robot_state_callback(self, msg: String):
        self.robot_state = msg.data

    def _load_params(self):
        self.ranges = {}
        for color in COLORS:
            self.ranges[color] = []
            for i in range(self._num_ranges[color]):
                lo = self.get_parameter(f"{color}_lo{i+1}").value
                hi = self.get_parameter(f"{color}_hi{i+1}").value
                self.ranges[color].append(
                    (np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
                )

    def depth_callback(self, data):
        if self.robot_state in self._INACTIVE_STATES:
            return
        try:
            self.depth_raw = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError:
            return

    def pointcloud_callback(self, data):
        if self.robot_state in self._INACTIVE_STATES:
            return
        try:
            pts_xyz = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            if pts_xyz is None or len(pts_xyz) == 0:
                return
            self.pointcloud_xyz = pts_xyz.reshape((data.height, data.width, 3))
            self.pointcloud_frame_id = data.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Point cloud failed: {e}")

    def _get_3d_points(self, mask):
        if self.pointcloud_xyz is None:
            return None
        if self.pointcloud_xyz.shape[:2] != mask.shape:
            return None
        pts = self.pointcloud_xyz[mask > 0]
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        dists = np.linalg.norm(pts, axis=1)
        pts = pts[dists < MAX_RANGE_M]
        if len(pts) < MIN_POINTS_3D:
            return None
        return pts

    def _fit_line_3d(self, pts_3d):
        mean = np.mean(pts_3d, axis=0)
        centered = pts_3d - mean
        cov = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        stds = np.sqrt(np.maximum(eigenvalues, 0))
        principal = eigenvectors[:, 2]
        return mean, principal, stds

    def _is_line_flat(self, stds):
        return stds[0] < self.get_parameter("max_flat_std").value

    @staticmethod
    def _skeletonize(mask):
        skel = np.zeros_like(mask, dtype=np.uint8)
        temp = (mask > 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while True:
            eroded = cv2.erode(temp, kernel)
            if cv2.countNonZero(eroded) == 0:
                skel = cv2.bitwise_or(skel, temp)
                break
            opening = cv2.morphologyEx(temp, cv2.MORPH_OPEN, kernel)
            skel = cv2.bitwise_or(skel, cv2.bitwise_and(temp, cv2.bitwise_not(opening)))
            temp = eroded
        return skel

    def _split_by_skeleton(self, cnt_mask):
        skel = self._skeletonize(cnt_mask)
        if cv2.countNonZero(skel) < 10:
            return [cnt_mask]

        lines = cv2.HoughLinesP(skel, 1, np.pi / 180, threshold=15, minLineLength=15, maxLineGap=8)

        if lines is None:
            return [cnt_mask]

        segments = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            seg_mask = np.zeros_like(cnt_mask)
            cv2.line(seg_mask, (x1, y1), (x2, y2), 255, 8)
            seg_mask = cv2.bitwise_and(seg_mask, cnt_mask)
            if cv2.countNonZero(seg_mask) > 10:
                segments.append(seg_mask)

        return segments if segments else [cnt_mask]

    def _publish_line_marker(self, p1, p2, color_name, stamp):
        marker = Marker()
        marker.header.frame_id = self.pointcloud_frame_id
        marker.header.stamp = stamp
        marker.ns = "lines"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=300_000_000)

        pt = Point()
        pt.x, pt.y, pt.z = float(p1[0]), float(p1[1]), float(p1[2])
        marker.points.append(pt)
        pt = Point()
        pt.x, pt.y, pt.z = float(p2[0]), float(p2[1]), float(p2[2])
        marker.points.append(pt)

        marker.scale.x = 0.05

        r, g, b = MARKER_COLORS.get(color_name, (1.0, 1.0, 1.0))
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def image_callback(self, data):
        if self.robot_state in self._INACTIVE_STATES:
            return
        if self.depth_raw is None or self.pointcloud_xyz is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        self.image_header = data.header
        h, w = cv_image.shape[:2]
        floor_y = h // 2

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        for color in COLORS:
            color_mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in self.ranges[color]:
                mask = cv2.inRange(hsv, lo, hi)
                color_mask = cv2.bitwise_or(color_mask, mask)

            color_mask[:floor_y, :] = 0

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL, MORPH_KERNEL))
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_LINE_AREA:
                    continue

                rect = cv2.minAreaRect(cnt)
                (rect_cx, rect_cy), (rect_w, rect_h), _ = rect
                if rect_w == 0 or rect_h == 0:
                    continue

                aspect = max(rect_w, rect_h) / min(rect_w, rect_h)
                if aspect < MIN_ASPECT_RATIO:
                    continue

                cnt_mask = np.zeros((h, w), dtype=np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)

                seg_masks = self._split_by_skeleton(cnt_mask)
                for seg_mask in seg_masks:
                    pts_3d = self._get_3d_points(seg_mask)
                    if pts_3d is None:
                        continue

                    center, direction, stds = self._fit_line_3d(pts_3d)

                    # -- Flatness filter: comment out to disable -------------
                    if not self._is_line_flat(stds):
                        continue
                    # -------------------------------------------------------

                    proj = pts_3d @ direction
                    half_len = max(np.std(proj) * 2.0, 0.2)
                    p1 = center - half_len * direction
                    p2 = center + half_len * direction

                    self._publish_line_marker(p1, p2, color, data.header.stamp)

                    self.get_logger().debug(
                        f"{color} segment @ ({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f})"
                        f" → ({p2[0]:.2f}, {p2[1]:.2f}, {p2[2]:.2f})"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = LineLocalizator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
