#!/usr/bin/env python3
"""
Cylinder detection debug overlay.

Two-layer display:
  1. Text banner — works always, no TF needed. Shows every raw marker received.
  2. Projected circle — uses TF map→camera_frame + K matrix.
     If TF fails, layer 1 is still shown (so you always see detections).
"""

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # registers PointStamped transform support

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

COLOUR_BGR = {
    name: (int(b * 255), int(g * 255), int(r * 255))
    for name, (r, g, b) in MARKER_COLOURS.items()
}

FADE_SECS = 0.8


def _rgb_to_name(r, g, b):
    best, best_d = "unknown", float("inf")
    for name, (cr, cg, cb) in MARKER_COLOURS.items():
        d = (r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2
        if d < best_d:
            best, best_d = name, d
    return best


class CylinderDebugView(Node):
    def __init__(self):
        super().__init__("cylinder_debug_view")

        self.bridge = CvBridge()
        self.latest_image = None
        self.K = None
        self.camera_frame = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Layer 1: text detections (always populated when marker arrives)
        self.text_detections: list[dict] = []
        # Layer 2: projected circles (populated only when TF succeeds)
        self.circle_overlays: list[dict] = []

        self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self._image_cb, SENSOR_QOS
        )
        # Use default (reliable) QoS for camera_info — it's latched/reliable
        self.create_subscription(
            CameraInfo, "/oakd/rgb/preview/camera_info",
            self._info_cb, 10
        )
        self.create_subscription(
            Marker, "cylinder_markers", self._marker_cb, 10
        )

        self.debug_pub = self.create_publisher(Image, "/cylinder_debug_image", 10)

        # Periodic status log so we can see if K / camera_frame are set
        self.create_timer(3.0, self._status_log)
        self.get_logger().info("CylinderDebugView ready")

    def _status_log(self):
        self.get_logger().info(
            f"[debug] K={'set' if self.K is not None else 'MISSING'}, "
            f"camera_frame={self.camera_frame!r}, "
            f"text_dets={len(self.text_detections)}, "
            f"circles={len(self.circle_overlays)}"
        )

    # ── Camera image ──────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            pass
        self._render()

    # ── Camera intrinsics ─────────────────────────────────────────────────────
    def _info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(
                f"Camera info received: frame={self.camera_frame}, "
                f"fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, "
                f"cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}"
            )

    # ── Raw cylinder marker ───────────────────────────────────────────────────
    def _marker_cb(self, msg: Marker):
        colour = _rgb_to_name(msg.color.r, msg.color.g, msg.color.b)
        orientation = msg.text if msg.text in ("vertical", "horizontal") else "vertical"
        now = self.get_clock().now()

        # Layer 1: always record (text banner, no TF needed)
        self.text_detections.append({
            "colour": colour,
            "orientation": orientation,
            "t": now,
        })
        self.text_detections = self.text_detections[-10:]

        # Layer 2: try TF projection
        if self.K is None or not self.camera_frame:
            return

        pt = PointStamped()
        pt.header.frame_id = "map"
        pt.header.stamp = rclpy.time.Time().to_msg()  # latest available TF
        pt.point.x = msg.pose.position.x
        pt.point.y = msg.pose.position.y
        pt.point.z = msg.pose.position.z

        try:
            pt_cam = self.tf_buffer.transform(
                pt, self.camera_frame,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF map→{self.camera_frame} failed: {ex}",
                throttle_duration_sec=5.0,
            )
            return

        px, py, pz = pt_cam.point.x, pt_cam.point.y, pt_cam.point.z
        if pz <= 0.05:
            return

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx_k, cy_k = self.K[0, 2], self.K[1, 2]
        u = int(fx * px / pz + cx_k)
        v = int(fy * py / pz + cy_k)

        self.circle_overlays.append({
            "u": u, "v": v,
            "colour": colour,
            "orientation": orientation,
            "t": now,
        })
        self.circle_overlays = self.circle_overlays[-30:]

    # ── Render ────────────────────────────────────────────────────────────────
    def _render(self):
        if self.latest_image is None:
            return

        img = self.latest_image.copy()
        now = self.get_clock().now()
        cutoff_ns = int(FADE_SECS * 1e9)
        h, w = img.shape[:2]

        # ── Layer 2: projected circles ────────────────────────────────────────
        fresh_circles = [o for o in self.circle_overlays
                         if (now - o["t"]).nanoseconds < cutoff_ns]
        self.circle_overlays = fresh_circles

        for o in fresh_circles:
            u, v = o["u"], o["v"]
            bgr = COLOUR_BGR.get(o["colour"], (200, 200, 200))
            label = o["colour"][0].upper() + ("V" if o["orientation"] == "vertical" else "H⚠")
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(img, (u, v), 22, bgr, 3)
                cv2.putText(img, label, (u + 26, v + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, bgr, 2, cv2.LINE_AA)

        # ── Layer 1: text banner (always works) ───────────────────────────────
        fresh_text = [d for d in self.text_detections
                      if (now - d["t"]).nanoseconds < cutoff_ns]
        self.text_detections = fresh_text

        y_pos = 24
        if fresh_text:
            # Deduplicate by colour+orientation for display
            seen = set()
            for d in fresh_text:
                key = (d["colour"], d["orientation"])
                if key in seen:
                    continue
                seen.add(key)
                bgr = COLOUR_BGR.get(d["colour"], (200, 200, 200))
                orient = "VERTICAL" if d["orientation"] == "vertical" else "HORIZONTAL ⚠"
                text = f"{d['colour'].upper()} {orient}"
                cv2.putText(img, text, (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2, cv2.LINE_AA)
                y_pos += 26

            if not fresh_circles:
                note = "(TF fail - no circle projection)"
                cv2.putText(img, note, (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 200), 1, cv2.LINE_AA)
        else:
            cv2.putText(img, "no raw cylinder detections", (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1, cv2.LINE_AA)

        cv2.imshow("Cylinder Debug", img)
        cv2.waitKey(1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError:
            pass


def main():
    rclpy.init()
    node = CylinderDebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
