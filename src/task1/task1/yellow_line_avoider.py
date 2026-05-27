#!/usr/bin/env python3
"""
Yellow line avoider — camera-only, no costmap editing.

The top (downward) camera watches for yellow floor lines.
When the line enters the DANGER zone (close, bottom of frame) the robot
immediately stops, says "Prohibited", and backs away.

Override strategy
-----------------
The iRobot Create3 motion_control node accepts velocity from TWO topics:
  /cmd_vel_unstamped  geometry_msgs/Twist        (task1 nodes, keyboard)
  /cmd_vel            geometry_msgs/TwistStamped (Nav2 final output)

During BACKING the avoider publishes at 50 Hz to BOTH topics, outrunning
Nav2 (~10-20 Hz) and any teleop node.  When CLEAR the avoider is passive
and does not interfere with normal driving or navigation.

Debug image → /yellow_line/debug_image  (shown in RViz YellowLineCamera
panel or rqt_image_view, same pattern as blue_line_explorer).
"""

import subprocess

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

YELLOW_LO = np.array([18, 100, 80], dtype=np.uint8)
YELLOW_HI = np.array([35, 255, 255], dtype=np.uint8)
MORPH_K   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


class YellowLineAvoider(Node):
    def __init__(self):
        super().__init__("yellow_line_avoider")

        self.declare_parameter("enabled",             True)
        self.declare_parameter("camera_topic",        "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("publish_debug_image", True)

        # Danger zone — very close to robot (lower portion of frame)
        self.declare_parameter("danger_zone_top",    0.65)
        self.declare_parameter("danger_zone_bottom", 0.95)
        # Horizontal crop to ignore wall reflections at edges
        self.declare_parameter("roi_left",  0.30)
        self.declare_parameter("roi_right", 0.70)

        self.declare_parameter("danger_px_threshold", 300)
        self.declare_parameter("back_speed",    0.12)   # m/s
        self.declare_parameter("back_duration", 1.8)    # seconds

        camera_topic = self.get_parameter("camera_topic").value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_cb, SENSOR_QOS)

        # /cmd_vel_unstamped — overrides task1 nodes and keyboard teleop
        self.cmd_unstamped = self.create_publisher(
            Twist, "/cmd_vel_unstamped", 10)
        # /cmd_vel — overrides Nav2's collision_monitor TwistStamped output
        self.cmd_stamped = self.create_publisher(
            TwistStamped, "/cmd_vel", 10)

        self.status_pub = self.create_publisher(String, "/yellow_line_status",      10)
        self.debug_pub  = self.create_publisher(Image,  "/yellow_line/debug_image", 10)

        self.state    = "CLEAR"
        self.back_end = 0.0
        self._spoke   = False
        self.robot_state = 'IDLE'
        self.runtime_enabled = True

        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self._state_cb, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/yellow_line_enabled', self._enable_cb, 10)

        # 50 Hz — faster than Nav2 (~10-20 Hz) and teleop (~10 Hz) so
        # backing commands win the last-write-wins race on both topics.
        self.timer = self.create_timer(0.02, self._update)
        self.get_logger().info(
            f"YellowLineAvoider ready on {camera_topic}. "
            "Overrides /cmd_vel_unstamped + /cmd_vel at 50 Hz during backing.")

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _state_cb(self, msg: String):
        self.robot_state = msg.data

    def _enable_cb(self, msg: Bool):
        self.runtime_enabled = bool(msg.data)
        if not self.runtime_enabled:
            self._pub_vel(0.0, 0.0)
            self.state = "CLEAR"
            self._spoke = False
            if hasattr(self, "_latest"):
                del self._latest
            self.get_logger().info("YellowLineAvoider disabled.")
        else:
            self.get_logger().info("YellowLineAvoider enabled.")

    def _image_cb(self, msg):
        if not self._should_run():
            return
        try:
            self._latest = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    # ── main loop ──────────────────────────────────────────────────────────────

    def _update(self):
        if not self._should_run():
            return
        if not hasattr(self, "_latest"):
            return

        img  = self._latest
        h, w = img.shape[:2]
        now  = self._now()

        # ── yellow detection ──────────────────────────────────────────────────
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_K)

        xl = int(w * float(self.get_parameter("roi_left").value))
        xr = int(w * float(self.get_parameter("roi_right").value))
        dt = int(h * float(self.get_parameter("danger_zone_top").value))
        db = int(h * float(self.get_parameter("danger_zone_bottom").value))

        danger_px     = int(cv2.countNonZero(mask[dt:db, xl:xr]))
        danger_thresh = int(self.get_parameter("danger_px_threshold").value)

        # ── state machine ──────────────────────────────────────────────────────
        if self.state == "BACKING":
            if now >= self.back_end:
                # Done backing — zero out both topics once, then go passive
                self._pub_vel(0.0, 0.0)
                self.state  = "CLEAR"
                self._spoke = False
                self.get_logger().info("Backing done — CLEAR.")
            else:
                # Flood both topics at 50 Hz — overrides ALL external commands
                self._pub_vel(-float(self.get_parameter("back_speed").value), 0.0)

        elif danger_px >= danger_thresh:
            # Zero immediately, then schedule the timed reverse
            self._pub_vel(0.0, 0.0)
            if not self._spoke:
                self.get_logger().warn(
                    f"PROHIBITED — yellow in danger zone ({danger_px} px). Backing.")
                self._speak("Prohibited")
                self._spoke   = True
                self.state    = "BACKING"
                self.back_end = now + float(self.get_parameter("back_duration").value)

        else:
            # CLEAR — avoider is fully passive; don't publish anything so
            # Nav2 and manual control work normally without interference.
            if self.state != "CLEAR":
                self.get_logger().info("CLEAR — no yellow in danger zone.")
            self.state = "CLEAR"

        # ── status + debug ────────────────────────────────────────────────────
        self.status_pub.publish(String(
            data=f"state={self.state} danger_px={danger_px}"))

        if self.get_parameter("publish_debug_image").value:
            self._publish_debug(img, mask, xl, xr, dt, db,
                                danger_px, danger_thresh)

    # ── debug image ────────────────────────────────────────────────────────────

    def _publish_debug(self, img, mask, xl, xr, dt, db,
                       danger_px, danger_thresh):
        debug = img.copy()

        overlay = debug.copy()
        overlay[mask > 0] = (0, 220, 255)
        cv2.addWeighted(overlay, 0.40, debug, 0.60, 0, debug)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (0, 180, 255), 2)

        d_col = (0, 0, 255) if danger_px >= danger_thresh else (80, 80, 80)
        cv2.rectangle(debug, (xl, dt), (xr, db), d_col, 2)
        self._label(debug, f"DANGER {danger_px}px", (xl + 4, dt + 16), d_col)

        col = (0, 200, 0) if self.state == "CLEAR" else (0, 100, 255)
        self._label(debug, f"STATE: {self.state}", (8, 20), col)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        except Exception:
            pass

    # ── helpers ────────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _should_run(self):
        if not self.get_parameter("enabled").value or not self.runtime_enabled:
            return False
        return self.robot_state not in (
            'APPROACH_WORKSTATION', 'WORKSTATION', 'APPROACH_FINAL', 'FINISHING_ROUNDS',
            'LINE_FOLLOWING', 'FOLLOW_BLUE_LINE', 'BLUE_LINE_SEARCH', 'BLUE_LINE_FOLLOW',
            'BLUE_LINE_DEAD_END',
        )

    def _pub_vel(self, linear: float, angular: float):
        """Publish to both velocity topics simultaneously."""
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_unstamped.publish(t)

        ts = TwistStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.twist.linear.x  = linear
        ts.twist.angular.z = angular
        self.cmd_stamped.publish(ts)

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
        self._pub_vel(0.0, 0.0)
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
