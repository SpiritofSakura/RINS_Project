#!/usr/bin/env python3
"""
Yellow line map scanner.

Drive the robot around room 1 while this node runs.
- Main window: camera feed with contours drawn around detected yellow lines
  (like face detection boxes but following the line shape exactly).
  Confirmed cells turn green so you can see what has already been captured.
- Mask window: raw HSV yellow mask so you can verify the colour tuning.

Press Ctrl+C when done — yellow cells are burned into map2.pgm and saved.
"""

import shutil
import signal
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

MAP_PGM  = Path(__file__).parent / "map2.pgm"
MAP_YAML = Path(__file__).parent / "map2.yaml"

# HSV range for yellow
YELLOW_LO = np.array([18, 100, 100], dtype=np.uint8)
YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)

# Morphology kernel — closes small gaps in the mask
MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

# Cell needs this many hits before it is written to the map
HIT_THRESHOLD = 3

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class YellowLineScanner(Node):
    def __init__(self):
        super().__init__("yellow_line_scanner")

        # ── load map ──────────────────────────────────────────────────────────
        with open(MAP_YAML) as f:
            cfg = yaml.safe_load(f)
        self.resolution = float(cfg["resolution"])
        self.origin     = cfg["origin"][:2]

        from PIL import Image as PILImage
        pil = PILImage.open(MAP_PGM).convert("L")
        self.map_arr  = np.array(pil)
        self.map_h, self.map_w = self.map_arr.shape
        self.hit_grid = np.zeros((self.map_h, self.map_w), dtype=np.int32)

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)

        self.K      = None
        self.bridge = CvBridge()
        self._last_proc   = 0.0
        self._frame_count = 0

        self.create_subscription(
            CameraInfo, "/top_camera/rgb/preview/camera_info",
            self._info_cb, 10)
        self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw",
            self._image_cb, SENSOR_QOS)

        self.get_logger().info(
            "Yellow scanner ready — drive around room 1, press Ctrl+C to save.")

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"Camera intrinsics: fx={self.K[0,0]:.1f}")

    def _image_cb(self, msg):
        if self.K is None:
            return

        # Throttle to 2 Hz
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_proc < 0.5:
            return
        self._last_proc = now
        self._frame_count += 1

        try:
            tf = self.tf_buf.lookup_transform(
                "map", "top_camera_rgb_camera_optical_frame",
                rclpy.time.Time())
        except Exception:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        # ── yellow mask ───────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_K)

        # ── project confirmed cells back to image for green overlay ───────────
        confirmed_img_mask = self._confirmed_cells_to_image(tf)

        # ── contours around detected yellow regions ───────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # ── build main debug image ────────────────────────────────────────────
        debug = img.copy()

        # Paint confirmed cells green
        debug[confirmed_img_mask > 0] = (0, 200, 0)

        # Draw filled contours in semi-transparent yellow, then outline
        overlay = debug.copy()
        cv2.drawContours(overlay, contours, -1, (0, 220, 255), -1)   # filled
        cv2.addWeighted(overlay, 0.35, debug, 0.65, 0, debug)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 2)       # outline

        yellow_px  = int(cv2.countNonZero(mask))
        confirmed  = int(np.sum(self.hit_grid >= HIT_THRESHOLD))
        detecting  = yellow_px > 50

        status_txt = "YELLOW DETECTED" if detecting else "scanning..."
        hdr_col    = (0, 220, 255) if detecting else (180, 180, 180)

        self._label(debug, f"Frame #{self._frame_count}  {status_txt}", (8, 20), hdr_col)
        self._label(debug, f"Yellow px   : {yellow_px}",   (8, 42), hdr_col)
        self._label(debug, f"Map cells   : {confirmed}",   (8, 62), (0, 220, 0))
        self._label(debug, "Ctrl+C to save map",           (8, 82), (200, 200, 200))

        cv2.imshow("Yellow Scanner — camera", debug)

        # ── mask window ───────────────────────────────────────────────────────
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(mask_bgr, contours, -1, (0, 180, 255), 2)
        self._label(mask_bgr, "HSV yellow mask", (8, 20), (0, 220, 255))
        cv2.imshow("Yellow Scanner — mask", mask_bgr)

        cv2.waitKey(1)

        # ── project yellow pixels → map hits ──────────────────────────────────
        hits = self._project_to_map(mask, tf)
        if hits:
            self.get_logger().info(
                f"[YELLOW] hits: {hits}  |  confirmed cells: "
                f"{int(np.sum(self.hit_grid >= HIT_THRESHOLD))}")

    # ── projection ────────────────────────────────────────────────────────────

    def _project_to_map(self, mask, tf):
        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]

        t = tf.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)

        ys, xs = np.where(mask[::4, ::4] > 0)
        ys, xs = ys * 4, xs * 4

        hits = 0
        for u, v in zip(xs, ys):
            ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            ray_map = R @ ray_cam
            if abs(ray_map[2]) < 1e-6:
                continue
            t_hit = -cam_pos[2] / ray_map[2]
            if t_hit < 0:
                continue
            wx = cam_pos[0] + t_hit * ray_map[0]
            wy = cam_pos[1] + t_hit * ray_map[1]
            col = int(round((wx - self.origin[0]) / self.resolution))
            row = int(round(self.map_h - (wy - self.origin[1]) / self.resolution))
            if 0 <= col < self.map_w and 0 <= row < self.map_h:
                self.hit_grid[row, col] += 1
                hits += 1
        return hits

    def _confirmed_cells_to_image(self, tf):
        """Project confirmed map cells back into the image frame for green overlay."""
        confirmed_mask = np.zeros(
            (int(self.K[1, 2] * 2), int(self.K[0, 2] * 2)), dtype=np.uint8)

        t = tf.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)
        R_inv = R.T

        rows, cols = np.where(self.hit_grid >= HIT_THRESHOLD)
        if len(rows) == 0:
            return confirmed_mask

        wx = cols * self.resolution + self.origin[0]
        wy = (self.map_h - rows) * self.resolution + self.origin[1]
        wz = np.zeros_like(wx)

        pts_map = np.stack([wx, wy, wz], axis=1) - cam_pos
        pts_cam = (R_inv @ pts_map.T).T

        valid = pts_cam[:, 2] > 0.01
        pts_cam = pts_cam[valid]
        if len(pts_cam) == 0:
            return confirmed_mask

        fx = self.K[0, 0]; fy = self.K[1, 1]
        cx = self.K[0, 2]; cy = self.K[1, 2]

        u = (pts_cam[:, 0] / pts_cam[:, 2] * fx + cx).astype(int)
        v = (pts_cam[:, 1] / pts_cam[:, 2] * fy + cy).astype(int)

        h, w = confirmed_mask.shape
        valid2 = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        confirmed_mask[v[valid2], u[valid2]] = 255

        # Dilate slightly so single pixels are visible
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        confirmed_mask = cv2.dilate(confirmed_mask, k)
        return confirmed_mask

    # ── save ──────────────────────────────────────────────────────────────────

    def save_map(self):
        from PIL import Image as PILImage

        backup = MAP_PGM.with_suffix(".pgm.bak")
        if not backup.exists():
            shutil.copy(MAP_PGM, backup)
            self.get_logger().info(f"Backup → {backup}")
        else:
            # Always start from the clean backup
            self.map_arr = np.array(PILImage.open(backup).convert("L"))

        out = self.map_arr.copy()
        confirmed = int(np.sum(self.hit_grid >= HIT_THRESHOLD))
        out[self.hit_grid >= HIT_THRESHOLD] = 0
        PILImage.fromarray(out).save(MAP_PGM)
        cv2.destroyAllWindows()
        self.get_logger().info(f"Saved {confirmed} yellow cells → {MAP_PGM}")

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _label(img, text, pos, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color,    1)

    @staticmethod
    def _quat_to_mat(x, y, z, w):
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
            [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
            [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])


def main():
    rclpy.init()
    node = YellowLineScanner()

    def _shutdown(sig, frame):
        node.get_logger().info("Saving map…")
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
