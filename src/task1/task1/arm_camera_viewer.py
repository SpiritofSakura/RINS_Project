import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArmCameraViewer(Node):
    def __init__(self):
        super().__init__("arm_camera_viewer")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw",
            self._image_callback, 10
        )

        self.get_logger().info("Arm camera viewer started.")

    def _image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Conversion error: {e}")
            return

        cv2.namedWindow("Arm Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Arm Camera", 640, 480)
        cv2.imshow("Arm Camera", cv_image)
        cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArmCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
