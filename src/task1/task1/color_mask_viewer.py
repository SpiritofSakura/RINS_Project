import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Default HSV ranges (matching existing ranges in the codebase)
# Red wraps around H=0/180, so it has two ranges.
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


class ColorMaskViewer(Node):
    def __init__(self):
        super().__init__("color_mask_viewer")

        self.bridge = CvBridge()

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

        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Color Mask", cv2.WINDOW_NORMAL)

        self.get_logger().info(
            "ColorMaskViewer ready. Press 'q' in a window to exit.")

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

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        combined_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        for color in COLORS:
            for lo, hi in self.ranges[color]:
                mask = cv2.inRange(hsv, lo, hi)
                combined_mask = cv2.bitwise_or(combined_mask, mask)

        masked = cv2.bitwise_and(cv_image, cv_image, mask=combined_mask)

        cv2.imshow("Original", cv_image)
        cv2.imshow("Color Mask", masked)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ColorMaskViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
