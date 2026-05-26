import math
import time as time_module
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import numpy as np


class TileDetect(Node):
    def __init__(self):
        super().__init__("tile_detect")
        self.declare_parameter("debug_mode", False)
        self._debug_mode = self.get_parameter("debug_mode").value
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._img_cb, 10
        )
        self.latest_gray = None
        self.latest_bgr = None
        self._tile_was_visible = False
        self._tile_missed = 0
        self._count = 0
        self._bright_hit = 0
        self._bright_missed = 0
        self._bright_ready = False
        self.status_pub = self.create_publisher(String, "/tile_status", 10)
        self.warped_pub = self.create_publisher(Image, "/tile_warped", 10)
        self._inspector_phase = -1
        self.phase_sub = self.create_subscription(
            Int32, "/inspector_phase", self._phase_callback, 10
        )
        self._last_box = None
        self._last_corners = None
        self._capture_pending = False
        self._capture_trigger_time = 0.0
        self._capture_done = False
        self._latest_warped = None
        self.create_timer(0.1, self._update)
        mode = "DEBUG" if self._debug_mode else "NORMAL"
        self.get_logger().info(f"TileDetect ready — Otsu detect + warp + publish  [{mode}]")

    def _img_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_bgr = bgr
            self.latest_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        except Exception:
            pass

    def _phase_callback(self, msg):
        self._inspector_phase = msg.data

    def _brightness_trigger(self, gray):
        h, w = gray.shape
        roi = gray[h // 2 - 5 : h // 2 + 5, w // 3 - 10 : w // 3 + 10]
        bright_ratio = (roi > 100).sum() / roi.size

        if bright_ratio >= 0.5:
            self._bright_hit += 1
            self._bright_missed = 0
            if self._bright_hit >= 3:
                self._bright_ready = True
        else:
            self._bright_missed += 1
            self._bright_hit = 0
            if self._bright_missed >= 10:
                self._bright_ready = False

        return self._bright_ready

    def _find_tile(self, gray):
        h, w = gray.shape
        crop_y = h // 5
        crop = gray[crop_y:, :]

        _, thresh = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        cy, cx = crop.shape[0] // 2, crop.shape[1] // 2
        if thresh[cy, cx] == 0:
            thresh = 255 - thresh

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        mask_full = np.zeros(gray.shape[:2], dtype=np.uint8)
        mask_full[crop_y:, :] = thresh

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frame_area = crop.shape[0] * crop.shape[1]
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < frame_area * 0.05 or area > frame_area * 0.5:
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            bw = math.hypot(box[0][0] - box[1][0], box[0][1] - box[1][1])
            bh = math.hypot(box[1][0] - box[2][0], box[1][1] - box[2][1])
            if max(bw, bh) / max(min(bw, bh), 1) > 2.0:
                continue
            if best is None or area > cv2.contourArea(best):
                best = c

        if best is None:
            return None, mask_full

        rect = cv2.minAreaRect(best)
        box = cv2.boxPoints(rect)
        box[:, 1] += crop_y

        return box.astype(np.int32), mask_full

    def _get_tile_quad(self, mask_full):
        """Return the 4 true corners of the tile (convex hull + poly approx), or None."""
        contours, _ = cv2.findContours(mask_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(best)
        quad = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True)
        if len(quad) != 4:
            return None
        return quad.reshape(4, 2).astype(np.float32)

    def _get_edge_points(self, mask_full):
        corners = self._get_tile_quad(mask_full)
        if corners is None:
            return None
        pts = []
        n = len(corners)
        for i in range(n):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % n]
            for t in range(16):
                frac = t / 15.0
                ix = int(round(x1 + (x2 - x1) * frac))
                iy = int(round(y1 + (y2 - y1) * frac))
                pts.append((ix, iy))
        return pts

    @staticmethod
    def _order_points(pts):
        sorted_by_y = pts[pts[:, 1].argsort()]
        top = sorted_by_y[:2][sorted_by_y[:2, 0].argsort()]
        bot = sorted_by_y[2:][sorted_by_y[2:, 0].argsort()]
        return np.array([top[0], top[1], bot[1], bot[0]], dtype=np.float32)

    def _warp_tile(self):
        """Warp detected tile region to fronto-parallel view and publish."""
        if self.latest_bgr is None or self._last_corners is None:
            return
        src = self._order_points(self._last_corners)
        inset = 0.06
        for i in range(4):
            prev = src[(i - 1) % 4]
            nxt = src[(i + 1) % 4]
            src[i] = src[i] + inset * (prev - src[i]) + inset * (nxt - src[i])
        w = max(np.linalg.norm(src[1] - src[0]), np.linalg.norm(src[2] - src[3]))
        h = max(np.linalg.norm(src[3] - src[0]), np.linalg.norm(src[2] - src[1]))
        if w < 10 or h < 10:
            return
        dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(self.latest_bgr, H, (int(w), int(h)))
        self._latest_warped = warped.copy()
        msg = self.bridge.cv2_to_imgmsg(warped, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._count)
        self.warped_pub.publish(msg)
        self.get_logger().info(f"Published warped tile ({int(w)}×{int(h)})")

    def _update(self):
        if self.latest_gray is None:
            self.get_logger().info("Waiting for camera...", throttle_duration_sec=2.0)
            return
        gray = self.latest_gray
        ready = self._brightness_trigger(gray)

        box, mask_full = None, None
        if ready:
            box, mask_full = self._find_tile(gray)
        else:
            mask_full = np.zeros_like(gray)

        # Only detect and count during scan phase (inspector phase == 4),
        # unless debug_mode is set (always detect).
        scanning = self._debug_mode or self._inspector_phase == 4
        has_tile = box is not None and scanning

        if scanning and has_tile:
            self._tile_missed = 0
            if not self._tile_was_visible:
                self._count += 1
                self._tile_was_visible = True
                self._last_box = box.copy()
                hull_corners = self._get_tile_quad(mask_full)
                self._last_corners = hull_corners if hull_corners is not None else box.astype(np.float32)
                self._capture_pending = True
                self._capture_trigger_time = time_module.time()
                self._capture_done = False
                self.get_logger().info(f"Tile #{self._count}")
                self.status_pub.publish(String(data="TILE_FOUND"))
        elif scanning and not has_tile:
            if self._tile_was_visible:
                self._tile_missed += 1
                if self._tile_missed >= 5:
                    self._tile_was_visible = False
                    self._tile_missed = 0
                    self._last_corners = None
                    self._capture_pending = False
                    self._capture_done = False
                    self.status_pub.publish(String(data="TILE_LEFT"))
        elif not scanning:
            self._tile_was_visible = False
            self._tile_missed = 0
            self._last_corners = None
            self._capture_pending = False
            self._capture_done = False

        # Fire capture 0.5s after first detection (robot should be stationary)
        if self._capture_pending and not self._capture_done:
            if time_module.time() - self._capture_trigger_time >= 0.5:
                self._capture_done = True
                self._capture_pending = False
                self._warp_tile()

        # Window 1: Otsu mask with fitted rectangle
        vis = cv2.cvtColor(mask_full, cv2.COLOR_GRAY2BGR)
        if has_tile:
            cv2.drawContours(vis, [box], -1, (0, 200, 0), 2)
        cv2.putText(vis, f"Tiles: {self._count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if scanning:
            if has_tile:
                cv2.putText(vis, "TILE", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                cv2.putText(vis, "scanning...", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, 128, 1)
        else:
            cv2.putText(vis, f"phase={self._inspector_phase} waiting...", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, 128, 1)
        cv2.imshow("Tile detect", vis)

        # Window 2: raw camera with hull-refined red dots
        raw_vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        h, w = gray.shape
        x1, x2 = w // 3 - 10, w // 3 + 10
        y1, y2 = h // 2 - 5, h // 2 + 5
        cv2.rectangle(raw_vis, (x1, y1), (x2, y2), (0, 255, 0) if ready else (0, 0, 255), 1)
        if has_tile:
            pts = self._get_edge_points(mask_full)
            if pts:
                for pt in pts:
                    cv2.circle(raw_vis, pt, 2, (0, 0, 255), -1)
        cv2.putText(raw_vis, f"bright={ready} phase={self._inspector_phase}", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.imshow("Tile raw edges", raw_vis)

        if self._latest_warped is not None:
            h_w, w_w = self._latest_warped.shape[:2]
            max_disp = 400
            scale = min(max_disp / w_w, max_disp / h_w, 1.0)
            if scale < 1.0:
                disp = cv2.resize(self._latest_warped, (int(w_w * scale), int(h_w * scale)))
            else:
                disp = self._latest_warped
            cv2.imshow("Tile warped", disp)

        cv2.waitKey(1)


def main():
    rclpy.init()
    node = TileDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()