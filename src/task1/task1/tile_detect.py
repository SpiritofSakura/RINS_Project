import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class TileDetect(Node):
    def __init__(self):
        super().__init__("tile_detect")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._img_cb, 10
        )
        self.latest_gray = None
        self._tile_was_visible = False
        self._tile_missed = 0
        self._count = 0
        self.create_timer(0.1, self._update)
        self.get_logger().info("TileDetect ready — contour mode")

    def _img_cb(self, msg):
        try:
            self.latest_gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            pass

    def _find_square(self, gray):
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 120, 240)
        edges = cv2.dilate(edges, np.ones((7, 7), np.uint8), iterations=3)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = gray.shape
        frame_area = h * w
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < frame_area * 0.08:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) != 4:
                continue
            if not cv2.isContourConvex(approx):
                continue
            _, _, bw, bh = cv2.boundingRect(approx)
            if max(bw, bh) / max(min(bw, bh), 1) > 2.0:
                continue
            best = approx
            break
        return best, edges

    def _update(self):
        if self.latest_gray is None:
            self.get_logger().info("Waiting for camera...", throttle_duration_sec=2.0)
            return
        gray = self.latest_gray
        square, edges = self._find_square(gray)

        has_tile = square is not None

        if has_tile:
            self._tile_missed = 0
            if not self._tile_was_visible:
                self._count += 1
                self._tile_was_visible = True
                self.get_logger().info(f"Tile #{self._count}")
        else:
            if self._tile_was_visible:
                self._tile_missed += 1
                if self._tile_missed >= 5:
                    self._tile_was_visible = False
                    self._tile_missed = 0

        vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if square is not None:
            cv2.drawContours(vis, [square], -1, (0, 200, 0), 3)

        cv2.putText(
            vis, f"Tiles: {self._count}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )
        if has_tile:
            cv2.putText(
                vis, "TILE", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

        cv2.imshow("Tile detect", vis)
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
