import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np


class ParallelAlign(Node):
    def __init__(self):
        super().__init__("parallel_align")
        self.declare_parameter("view_only", False)
        self.view_only = self.get_parameter("view_only").value
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw", self._img_cb, 10
        )
        self.deriv_pub = self.create_publisher(Image, "/parallel_align/debug/derivative", 10)
        self.binary_pub = self.create_publisher(Image, "/parallel_align/debug/binary", 10)
        self.hough_pub = self.create_publisher(Image, "/parallel_align/debug/hough", 10)
        self.latest_gray = None
        self.aligned = False
        self.create_timer(0.1, self._update)
        self.get_logger().info("ParallelAlign ready — Hough mode")

    def _img_cb(self, msg):
        try:
            self.latest_gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            pass

    def _get_hough_tilt(self):
        if self.latest_gray is None:
            return None, None, None, None, None
        gray = self.latest_gray
        h, w = gray.shape

        dy = np.abs(np.diff(gray.astype(np.int16), axis=0)).astype(np.uint16)
        dy_norm = (dy / (dy.max() + 1e-6) * 255).astype(np.uint8)

        top_h = max(1, h // 3)
        top_norm = dy_norm[:top_h, :]

        top_blur = cv2.GaussianBlur(top_norm, (3, 3), 0)
        _, binary = cv2.threshold(top_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        lines = cv2.HoughLines(binary, 1, math.pi / 180, threshold=80)

        best_theta = None
        best_rho = None
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                dev = abs(theta - math.pi / 2)
                if dev < math.radians(20):
                    best_rho = rho
                    best_theta = theta
                    break

        return best_rho, best_theta, top_norm, binary, (gray, w)

    def _draw_hough_line(self, img, rho, theta, color, thickness):
        h, w = img.shape[:2]
        a = math.cos(theta)
        b = math.sin(theta)
        if abs(b) < 1e-6:
            return
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 2000 * (-b)), int(y0 + 2000 * a))
        pt2 = (int(x0 - 2000 * (-b)), int(y0 - 2000 * a))
        cv2.line(img, pt1, pt2, color, thickness)

    def _update(self):
        if self.aligned:
            return
        rho, theta, deriv, binary, (gray, w) = self._get_hough_tilt()

        if theta is None:
            self.get_logger().info(
                "No Hough line found — waiting...", throttle_duration_sec=1.0
            )
            # publish debug anyway
            if deriv is not None:
                try:
                    self.deriv_pub.publish(
                        self.bridge.cv2_to_imgmsg(deriv, "mono8")
                    )
                except Exception:
                    pass
            return

        angle_deg = math.degrees(theta - math.pi / 2)
        self.get_logger().info(
            f"Hough theta={math.degrees(theta):.1f}deg "
            f"tilt={angle_deg:+.1f}deg rho={rho:.0f}"
        )

        # build debug views
        hough_view = cv2.cvtColor(deriv, cv2.COLOR_GRAY2BGR)
        self._draw_hough_line(hough_view, rho, theta, (0, 200, 0), 2)

        cv2.imshow("Derivative + Hough line", hough_view)
        cv2.waitKey(1)

        # publish debug topics
        try:
            self.deriv_pub.publish(self.bridge.cv2_to_imgmsg(deriv, "mono8"))
            bin_msg = self.bridge.cv2_to_imgmsg(binary, "mono8")
            self.binary_pub.publish(bin_msg)
            hough_msg = self.bridge.cv2_to_imgmsg(hough_view, "bgr8")
            self.hough_pub.publish(hough_msg)
        except Exception:
            pass

        if abs(angle_deg) <= 0.5:
            if not self.view_only:
                self._stop()
                self.aligned = True
            self.get_logger().info(f"Parallel! tilt={angle_deg:+.1f}deg")
            return

        if self.view_only:
            return

        Kp = 0.15
        ang = max(min(-Kp * angle_deg, 0.1), -0.1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = ang
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"Rotating: ang.z={ang:.3f} (tilt={angle_deg:+.1f}deg)"
        )

    def _stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ParallelAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
