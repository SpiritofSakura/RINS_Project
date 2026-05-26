#!/usr/bin/env python3
"""
Yellow line avoider.

Uses the top (downward) camera to detect yellow floor lines in real time.
Only reacts when the yellow line is in the DANGER zone (very close, bottom
of the frame) — robot stops, says "Prohibited", and backs away.

Debug image is published to /yellow_line/debug_image (view in RViz or
rqt_image_view — same pattern as blue_line_explorer).

States
------
  CLEAR      No yellow in danger zone.
  BACKING    Reversing after detecting a prohibited crossing.
"""

import subprocess

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# HSV range for yellow
YELLOW_LO = np.array([18, 100, 80], dtype=np.uint8)
YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)

MORPH_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


class YellowLineAvoider(Node):
    def __init__(self):
        super().__init__("yellow_line_avoider")

        self.declare_parameter("enabled", True)
        self.declare_parameter("camera_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("publish_debug_image", True)

        # Danger zone — fraction of image height from top.
        # Set low so it only fires when the line is very close (almost under robot).
        self.declare_parameter("danger_zone_top",    0.65)
        self.declare_parameter("danger_zone_bottom", 0.95)
        # Horizontal crop — ignore image edges (wall reflections)
        self.declare_parameter("roi_left",  0.15)
        self.declare_parameter("roi_right", 0.85)

        # Pixel count to trigger backing
        self.declare_parameter("danger_px_threshold", 300)

        # Backing parameters
        self.declare_parameter("back_speed",    0.12)   # m/s
        self.declare_parameter("back_duration", 1.8)    # seconds

        camera_topic  = self.get_parameter("camera_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_cb, SENSOR_QOS)

        self.cmd_pub    = self.create_publisher(Twist,  cmd_vel_topic,            10)
        self.manual_pub = self.create_publisher(Bool,   "/manual_control_active", 10)
        self.status_pub = self.create_publisher(String, "/yellow_line_status",    10)
        self.debug_pub  = self.create_publisher(Image,  "/yellow_line/debug_image", 10)

        self.state    = "CLEAR"
        self.back_end = 0.0
        self._spoke   = False

        self.timer = self.create_timer(0.1, self._update)
        self.get_logger().info(
            f"YellowLineAvoider ready on {camera_topic}. "
            "Debug image: /yellow_line/debug_image")

    # ── image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    # ── main loop ─────────────────────────────────────────────────────────────

    def _update(self):
        if not self.get_parameter("enabled").value:
            return
        if not hasattr(self, "_latest"):
            return

        img  = self._latest
        h, w = img.shape[:2]
        now  = self._now()

        # ── detect ────────────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_K)

        xl = int(w * float(self.get_parameter("roi_left").value))
        xr = int(w * float(self.get_parameter("roi_right").value))
        dt = int(h * float(self.get_parameter("danger_zone_top").value))
        db = int(h * float(self.get_parameter("danger_zone_bottom").value))

        danger_px    = int(cv2.countNonZero(mask[dt:db, xl:xr]))
        danger_thresh = int(self.get_parameter("danger_px_threshold").value)

        # ── state machine ──────────────────────────────────────────────────────
        if self.state == "BACKING":
            self._pub_bool(self.manual_pub, True)
            if now >= self.back_end:
                self._stop()
                self._pub_bool(self.manual_pub, False)
                self.state  = "CLEAR"
                self._spoke = False
                self.get_logger().info("Backing done — resuming CLEAR.")
            else:
                self._pub_twist(-float(self.get_parameter("back_speed").value), 0.0)

        elif danger_px >= danger_thresh:
            self._pub_bool(self.manual_pub, True)
            self._stop()
            if not self._spoke:
                self.get_logger().warn(
                    f"PROHIBITED — yellow in danger zone ({danger_px} px). Backing away.")
                self._speak("Prohibited")
                self._spoke   = True
                self.state    = "BACKING"
                self.back_end = now + float(self.get_parameter("back_duration").value)

        else:
            if self.state != "CLEAR":
                self.get_logger().info("CLEAR — no yellow in danger zone.")
                self._pub_bool(self.manual_pub, False)
            self.state = "CLEAR"

        # ── publish status ────────────────────────────────────────────────────
        self.status_pub.publish(String(
            data=f"state={self.state} danger_px={danger_px}"))

        # ── debug image ───────────────────────────────────────────────────────
        if self.get_parameter("publish_debug_image").value:
            self._publish_debug(img, mask, xl, xr, dt, db,
                                danger_px, danger_thresh)

    # ── debug ─────────────────────────────────────────────────────────────────

    def _publish_debug(self, img, mask, xl, xr, dt, db,
                       danger_px, danger_thresh):
        debug = img.copy()

        # Yellow mask overlay
        overlay = debug.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.40, debug, 0.60, 0, debug)

        # Contours around yellow blobs
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 2)

        # Danger zone box
        d_col = (0, 0, 255) if danger_px >= danger_thresh else (80, 80, 80)
        cv2.rectangle(debug, (xl, dt), (xr, db), d_col, 2)
        self._label(debug, f"DANGER {danger_px}px", (xl + 4, dt + 16), d_col)

        # State banner
        state_colors = {
            "CLEAR":   (0, 200, 0),
            "BACKING": (0, 100, 255),
        }
        col = state_colors.get(self.state, (200, 200, 200))
        self._label(debug, f"STATE: {self.state}", (8, 20), col)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception:
            pass

    # ── helpers ───────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pub_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._pub_twist(0.0, 0.0)

    def _pub_bool(self, pub, val):
        msg = Bool()
        msg.data = bool(val)
        pub.publish(msg)

    @staticmethod
    def _label(img, text, pos, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 2)
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.48, color,     1)

    @staticmethod
    def _speak(text):
        for cmd in (["espeak-ng", text], ["espeak", text], ["spd-say", text]):
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                return
            except FileNotFoundError:
                continue

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YellowLineAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
