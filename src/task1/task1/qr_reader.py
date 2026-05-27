import json
import subprocess
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

FACE_QR_STATES = frozenset(("APPROACH_FACE", "INTERACT_FACE"))
BLUE_LINE_QR_STATES = frozenset((
    "FOLLOW_BLUE_LINE",
    "LINE_FOLLOWING",
    "BLUE_LINE_SEARCH",
    "BLUE_LINE_FOLLOW",
    "BLUE_LINE_DEAD_END",
))
REPORT_ONLY_KEEP_PATTERNS = ("report_manager",)
REPORT_FINISH_KILL_PATTERNS = (
    "detect_faces",
    "detect_people.py",
    "face_recognizer",
    "face_localizator",
    "blue_line_explorer",
    "behavior_manager",
    "robot_state_overlay",
    "qr_reader",
    "detect_rings_v2",
    "ring_localizator",
    "cylinder_segmentation",
    "cylinder_localizator",
    "cylinder_debug_view",
    "barrel_inspector",
    "yellow_line_avoider",
    "line_localizator",
    "color_mask_viewer",
    "workstation_recorder",
    "waypoint_navigator",
    "station_inspector",
    "tile_detect",
    "tile_classifier",
    "orchestrator",
    "pointcloud_viewer",
)


class QrReader(Node):
    def __init__(self):
        super().__init__("qr_reader")
        self.declare_parameter("start_mode", "")
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.wechat = cv2.wechat_qrcode_WeChatQRCode()

        self.last_data = None
        self.last_person = None
        self.active = False
        self.mode = "off"
        self._blue_line_mode_started = False
        self._face_announced = False
        self._seen_blue_qr = set()
        self._seen_popup_qr = set()
        self._blue_line_frame_index = 0
        self._report_finish_deadline = None
        self._report_finish_started = False

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
        self.report_finish_timer = self.create_timer(0.2, self._check_report_finish)

        start_mode = self.get_parameter("start_mode").value.strip().lower()
        if start_mode in ("blue_line", "follow_blue_line"):
            self.mode = "blue_line"
            self.active = True
            self._blue_line_mode_started = True
            self.get_logger().info("QR reader started directly in blue-line scan mode.")
        elif start_mode in ("face", "approach_face"):
            self.mode = "face"
            self.active = True
            self.get_logger().info("QR reader started directly in face QR mode.")
            self._publish_qr("approaching face")
        else:
            self.get_logger().info(
                "QR reader started, waiting for face approach/interaction or blue-line state...")

    def _publish_qr(self, text):
        msg = String()
        msg.data = text
        self.qr_pub.publish(msg)
        self.get_logger().info(f"Published to /qr: {text}")

    def _state_callback(self, msg):
        was_active = self.active
        previous_mode = self.mode

        if msg.data in FACE_QR_STATES:
            self.mode = "face"
        elif msg.data in BLUE_LINE_QR_STATES:
            self.mode = "blue_line"
            self._blue_line_mode_started = True
        elif self._blue_line_mode_started:
            self.mode = "blue_line"
        else:
            self.mode = "off"
        self.active = self.mode != "off"

        if self.mode == "face" and previous_mode != "face":
            self._face_announced = False
            self.last_person = None
            self.last_data = None
            self.get_logger().info("Entered face approach/interaction, enabling QR scanning")
            self._publish_qr("approaching face")

        if self.mode == "blue_line" and previous_mode != "blue_line":
            self.last_data = None
            self.get_logger().info("Entered blue-line QR sweep, scanning all QR codes")

        if not self.active and was_active:
            self.get_logger().info("Left QR scanning state, disabling QR scanning")

    def _person_callback(self, msg):
        if self.mode != "face":
            return
        try:
            data = json.loads(msg.data)
            name = data.get("name", "Unknown")
        except Exception:
            name = msg.data.strip()

        if name != self.last_person:
            self.last_person = name
            self._publish_qr(f"person {name}")

    def _decode_all(self, gray):
        detections = []
        try:
            res = self.wechat.detectAndDecode(gray)
            decoded = res[0] if res[0] is not None else []
            points = res[1] if res[1] is not None else []
            for i, data in enumerate(decoded):
                if data:
                    corners = points[i] if i < len(points) else None
                    detections.append((data, corners))
        except Exception:
            pass
        if detections:
            return detections

        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(gray)
            if ok:
                detections = [
                    (data, points[i] if points is not None else None)
                    for i, data in enumerate(decoded_info)
                    if data
                ]
                if detections:
                    return detections
        except Exception:
            pass

        data, bbox, _ = self.detector.detectAndDecode(gray)
        if data:
            return [(data, bbox[0] if bbox is not None else None)]

        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(enhanced)
            if ok:
                detections = [
                    (data, points[i] if points is not None else None)
                    for i, data in enumerate(decoded_info)
                    if data
                ]
                if detections:
                    return detections
        except Exception:
            pass

        data, bbox, _ = self.detector.detectAndDecode(enhanced)
        if data:
            return [(data, bbox[0] if bbox is not None else None)]

        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(thresh)
            if ok:
                detections = [
                    (data, points[i] if points is not None else None)
                    for i, data in enumerate(decoded_info)
                    if data
                ]
                if detections:
                    return detections
        except Exception:
            pass

        data, bbox, _ = self.detector.detectAndDecode(thresh)
        if data:
            return [(data, bbox[0] if bbox is not None else None)]

        return []

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

    def _check_blue_line_qr(self, data):
        lower = data.lower()
        if "report" not in lower:
            self.get_logger().info(f"Blue-line QR seen and ignored: {data}")
            return

        self.report_pub.publish(String(data="make"))
        self.get_logger().info("Blue-line report QR detected — generated report and starting finish timer")
        if self._report_finish_deadline is None:
            self._report_finish_deadline = self.get_clock().now().nanoseconds + int(3.0 * 1e9)

    def _check_report_finish(self):
        if self._report_finish_deadline is None or self._report_finish_started:
            return
        if self.get_clock().now().nanoseconds < self._report_finish_deadline:
            return

        self._report_finish_started = True
        self.get_logger().info("Report finish timer elapsed — shutting down all nodes except report_manager")
        for pattern in REPORT_FINISH_KILL_PATTERNS:
            if pattern in REPORT_ONLY_KEEP_PATTERNS:
                continue
            try:
                subprocess.run(
                    ["pkill", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            except Exception as exc:
                self.get_logger().warn(f"Could not shut down {pattern}: {exc}")

    def _show_unique_qr_popup(self, data):
        if data in self._seen_popup_qr:
            return
        self._seen_popup_qr.add(data)

        width, height = 760, 220
        popup = np.full((height, width, 3), 245, dtype=np.uint8)
        cv2.rectangle(popup, (0, 0), (width - 1, height - 1), (30, 120, 30), 3)
        cv2.putText(
            popup,
            "QR code read:",
            (24, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (20, 80, 20),
            2,
            cv2.LINE_AA,
        )

        words = str(data).split()
        lines = []
        current = ""
        for word in words:
            candidate = word if not current else f"{current} {word}"
            (text_width, _), _ = cv2.getTextSize(
                candidate, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2
            )
            if text_width > width - 48 and current:
                lines.append(current)
                current = word
            else:
                current = candidate
        if current:
            lines.append(current)
        if not lines:
            lines = [str(data)]

        y = 90
        for line in lines[:4]:
            cv2.putText(
                popup,
                line,
                (24, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
            y += 34

        cv2.imshow("QR Code Text", popup)
        cv2.waitKey(1)

    def _image_callback(self, msg):
        if not self.active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Conversion error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.mode == "blue_line":
            self._blue_line_frame_index += 1
            if self._blue_line_frame_index % 4 != 0:
                show = cv2.resize(cv_image, (640, 480))
                cv2.imshow("QR Reader", show)
                cv2.waitKey(1)
                return

        detections = self._decode_all(gray)

        for data, corners in detections:
            self._show_unique_qr_popup(data)

            if self.mode == "blue_line":
                if data in self._seen_blue_qr:
                    continue
                self._seen_blue_qr.add(data)
                self.get_logger().info(f"Blue-line QR Code detected: {data}")
                self._check_blue_line_qr(data)
            elif data != self.last_data:
                self.last_data = data
                self.get_logger().info(f"QR Code detected: {data}")
                self._check_defects(data)

            if corners is None or len(corners) != 4:
                continue
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
