import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class QrReader(Node):
    def __init__(self):
        super().__init__("qr_reader")
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.wechat = cv2.wechat_qrcode_WeChatQRCode()

        self.last_data = None
        self.last_person = None
        self.active = False
        self._face_announced = False

        self.qr_pub = self.create_publisher(String, "/qr", 10)
        self.report_pub = self.create_publisher(String, "/report_commands", 10)

        self.img_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw",
            self._image_callback, SENSOR_QOS
        )

        self.state_sub = self.create_subscription(
            String, "/robot_state", self._state_callback, 10
        )

        self.person_sub = self.create_subscription(
            String, "/recognized_person", self._person_callback, 10
        )

        self.get_logger().info("QR reader started, waiting for APPROACH_FACE state...")

    def _publish_qr(self, text):
        msg = String()
        msg.data = text
        self.qr_pub.publish(msg)
        self.get_logger().info(f"Published to /qr: {text}")

    def _state_callback(self, msg):
        was_active = self.active
        self.active = (msg.data == "APPROACH_FACE")

        if self.active and not was_active:
            self._face_announced = False
            self.last_person = None
            self.last_data = None
            self.get_logger().info("Entered APPROACH_FACE, enabling QR scanning")
            self._publish_qr("approaching face")

        if not self.active and was_active:
            self.get_logger().info("Left APPROACH_FACE, disabling QR scanning")

    def _person_callback(self, msg):
        if not self.active:
            return
        try:
            data = json.loads(msg.data)
            name = data.get("name", "Unknown")
        except Exception:
            name = msg.data.strip()

        if name != self.last_person:
            self.last_person = name
            self._publish_qr(f"person {name}")

    def _decode(self, gray):
        try:
            res = self.wechat.detectAndDecode(gray)
            if res[0]:
                return res[0][0], res[1][0]
        except Exception:
            pass

        data, bbox, _ = self.detector.detectAndDecode(gray)
        if data:
            return data, bbox[0] if bbox is not None else None

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        data, bbox, _ = self.detector.detectAndDecode(enhanced)
        if data:
            return data, bbox[0] if bbox is not None else None

        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        data, bbox, _ = self.detector.detectAndDecode(thresh)
        if data:
            return data, bbox[0] if bbox is not None else None

        return None, None

    def _check_defects(self, data):
        lower = data.lower()
        msg = String()

        if "report" in lower:
            self.report_pub.publish(String(data="make"))
            self.get_logger().info("Triggered report generation")
            return

        if "barrel" in lower:
            msg.data = "barrels"
        elif "ring" in lower:
            msg.data = "rings"
        else:
            defect_keywords = ["defect", "anomaly"]
            if not any(k in lower for k in defect_keywords):
                return
            color = "green"
            if "red" in lower:
                color = "red"
            msg.data = f"defects {color}"

        self.qr_pub.publish(msg)
        self.get_logger().info(f"Published to /qr: {msg.data}")

    def _image_callback(self, msg):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Conversion error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        data, corners = self._decode(gray)

        if data and data != self.last_data:
            self.last_data = data
            self.get_logger().info(f"QR Code detected: {data}")
            self._check_defects(data)

        if corners is not None and len(corners) == 4:
            corners = corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(cv_image, data, tuple(corners[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        show = cv2.resize(cv_image, (640, 480))
        cv2.imshow("QR Reader", show)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = QrReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
