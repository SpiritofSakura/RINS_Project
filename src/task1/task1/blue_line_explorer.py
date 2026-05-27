#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

try:
    from irobot_create_msgs.msg import HazardDetectionVector as _HazardMsg
    _HAZARD_TOPIC = "/hazard_detection"
except ImportError:
    _HazardMsg = None
    _HAZARD_TOPIC = None

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BlueLineExplorer(Node):
    def __init__(self):
        super().__init__("blue_line_explorer")

        # ── enable / topic ───────────────────────────────────────────────────
        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("start_on_patrol_finished", False)
        self.declare_parameter("camera_topic", "/top_camera/rgb/preview/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("debug_view", False)
        self.declare_parameter("publish_debug_image", True)

        # ── color detection ──────────────────────────────────────────────────
        self.declare_parameter("blue_lo", [82, 120, 60])
        self.declare_parameter("blue_hi", [102, 255, 255])
        self.declare_parameter("use_blue_dominance", True)
        self.declare_parameter("blue_dominance_margin", 40)
        self.declare_parameter("blue_min_channel", 80)
        self.declare_parameter("min_blue_pixels", 150)
        # Crop the top fraction of the image (sky / walls above floor).
        self.declare_parameter("roi_top_fraction", 0.0)

        # ── direction ROIs ───────────────────────────────────────────────────
        # Three horizontal bands (left / straight / right) used only to decide
        # speed and to stop rightward centroid pull when no left branch exists.
        self.declare_parameter("branch_roi_top_fraction", 0.42)
        self.declare_parameter("branch_roi_bottom_fraction", 0.90)
        self.declare_parameter("straight_roi_top_fraction", 0.20)
        self.declare_parameter("straight_roi_bottom_fraction", 0.65)
        self.declare_parameter("min_branch_pixels", 50)
        self.declare_parameter("min_branch_ratio", 0.008)

        # ── steering ─────────────────────────────────────────────────────────
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("fast_linear_speed", 0.40)   # used on straight sections
        self.declare_parameter("kp_steer", 1.2)
        self.declare_parameter("max_angular_speed", 0.7)
        # Applied ONLY while split_active (branch visible or re-centering).
        # Zero on clear straight so the robot doesn't drift.
        self.declare_parameter("left_bias", 0.25)
        # EMA smoothing on angular: 0 = frozen, 1 = no smoothing.
        # Prevents jerky step-changes in the turn command.
        self.declare_parameter("angular_smoothing", 0.35)
        # How long (s) only "straight" must be lit before exiting split mode.
        self.declare_parameter("split_exit_hold", 0.5)

        # ── LIDAR forward obstacle ───────────────────────────────────────────
        self.declare_parameter("lidar_topic", "/scan")
        # RPLIDAR is mounted yaw=+90° CCW, so forward is at -90° in scan frame.
        self.declare_parameter("lidar_forward_angle_deg", -90.0)
        # ±degrees around the forward ray — keep narrow to avoid window frames
        self.declare_parameter("lidar_cone_half_angle_deg", 15.0)
        # Slow down when obstacle is closer than this (m).
        self.declare_parameter("lidar_slow_distance", 0.55)
        # Hard U-turn when obstacle is closer than this (m).
        self.declare_parameter("lidar_stop_distance", 0.37)

        # ── U-turn / recovery ────────────────────────────────────────────────
        self.declare_parameter("line_lost_timeout", 2.0)
        self.declare_parameter("uturn_angular_speed", 0.5)
        # 180° at 0.5 rad/s = 6.28 s — a little extra ensures full turn.
        self.declare_parameter("uturn_duration", 6.5)

        # ── wiring ───────────────────────────────────────────────────────────
        camera_topic = self.get_parameter("camera_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, camera_topic, self._image_callback, SENSOR_QOS)
        self.lidar_sub = self.create_subscription(
            LaserScan, self.get_parameter("lidar_topic").value,
            self._lidar_callback, SENSOR_QOS)
        self.enable_sub = self.create_subscription(
            Bool, "/blue_line_enabled", self._enable_callback, 10)
        self.patrol_sub = self.create_subscription(
            Bool, "/patrol_finished", self._patrol_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, "/robot_state", self._robot_state_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        self.manual_pub = self.create_publisher(Bool, "/manual_control_active", 10)
        self.patrol_cmd_pub = self.create_publisher(Bool, "/patrol_command", 10)
        self.finished_pub = self.create_publisher(Bool, "/blue_line_finished", 10)
        self.debug_pub = self.create_publisher(Image, "/blue_line/debug_image", 10)
        self.status_pub = self.create_publisher(String, "/blue_line/status", 10)

        if _HazardMsg is not None:
            self.create_subscription(
                _HazardMsg, _HAZARD_TOPIC, self._hazard_callback, 10)
            self.get_logger().info("Bump detection active on " + _HAZARD_TOPIC)

        self.enabled = self.get_parameter("enabled_on_start").value
        self.debug_view = self.get_parameter("debug_view").value

        # IDLE | SEARCH | FOLLOW | UTURN
        self.mode = "SEARCH" if self.enabled else "IDLE"
        self.line_lost_since = None
        self.uturn_end = 0.0
        self.latest_image = None
        self.prev_angular = 0.0      # last published angular (for EMA smoothing)
        self.split_active = False    # True while navigating a split or re-centering
        self.split_exit_since = None # when only-straight condition first held
        self.front_dist = float("inf")  # nearest obstacle directly ahead (m)

        self.timer = self.create_timer(0.1, self._update)
        self.get_logger().info(
            f"BlueLineExplorer ready on {camera_topic}. enabled={self.enabled}")

    # ── callbacks ────────────────────────────────────────────────────────────

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion: {exc}")

    def _enable_callback(self, msg):
        if msg.data:
            self._start()
        else:
            self._finish("disabled")

    def _patrol_callback(self, msg):
        if msg.data and self.get_parameter("start_on_patrol_finished").value:
            self._start()

    def _robot_state_callback(self, msg: String):
        if msg.data == 'LINE_FOLLOWING' and not self.enabled:
            self.get_logger().info('LINE_FOLLOWING — waiting for explicit blue-line enable.')
            self.debug_view = True

    def _hazard_callback(self, msg):
        if not self.enabled or self.mode == "UTURN":
            return
        for det in msg.detections:
            if det.type == 1:  # BUMP
                self._start_uturn("bump")
                return

    def _lidar_callback(self, msg):
        if not msg.ranges:
            return
        # RPLIDAR mounted yaw=+90° CCW → robot forward = scan angle -90°.
        fwd_rad = math.radians(float(self.get_parameter("lidar_forward_angle_deg").value))
        half_rad = math.radians(float(self.get_parameter("lidar_cone_half_angle_deg").value))
        n = len(msg.ranges)
        center = int(round((fwd_rad - msg.angle_min) / msg.angle_increment))
        half_idx = max(1, int(round(half_rad / msg.angle_increment)))
        lo, hi = max(0, center - half_idx), min(n, center + half_idx + 1)
        valid = [r for r in msg.ranges[lo:hi]
                 if msg.range_min <= r <= msg.range_max and r == r]  # r==r drops NaN
        self.front_dist = min(valid) if valid else float("inf")

    # ── state transitions ────────────────────────────────────────────────────

    def _start(self):
        if self.enabled:
            return
        self.enabled = True
        self.mode = "SEARCH"
        self.line_lost_since = None
        self._pub_bool(self.manual_pub, True)
        self._pub_bool(self.patrol_cmd_pub, False)
        self._pub_state("BLUE_LINE_SEARCH")
        self.get_logger().info("Blue-line explorer enabled.")

    def _finish(self, reason="done"):
        self.get_logger().info(f"Blue-line explorer finished: {reason}")
        self.enabled = False
        self.mode = "IDLE"
        self._stop()
        self._pub_bool(self.manual_pub, False)
        self._pub_bool(self.finished_pub, True)
        self._pub_state("IDLE")

    def _start_uturn(self, reason=""):
        self.get_logger().info(f"U-turn ({reason}).")
        self._pub_state("BLUE_LINE_DEAD_END")
        self.mode = "UTURN"
        self.line_lost_since = None
        self.split_active = False
        self.split_exit_since = None
        self.prev_angular = 0.0
        self.uturn_end = self._now_s() + float(
            self.get_parameter("uturn_duration").value)
        self._stop()

    # ── perception ───────────────────────────────────────────────────────────

    def _direction_rois(self, roi_h, roi_w):
        """Return {name: (x0, x1, y0, y1)} for left / straight / right bands."""
        bt = int(roi_h * float(self.get_parameter("branch_roi_top_fraction").value))
        bb = int(roi_h * float(self.get_parameter("branch_roi_bottom_fraction").value))
        st = int(roi_h * float(self.get_parameter("straight_roi_top_fraction").value))
        sb = int(roi_h * float(self.get_parameter("straight_roi_bottom_fraction").value))
        bt = clamp(bt, 0, roi_h - 1)
        bb = clamp(bb, bt + 1, roi_h)
        st = clamp(st, 0, roi_h - 1)
        sb = clamp(sb, st + 1, roi_h)
        return {
            "left":     (0,              int(roi_w * 0.33), bt, bb),
            "straight": (int(roi_w * 0.34), int(roi_w * 0.66), st, sb),
            "right":    (int(roi_w * 0.67), roi_w,            bt, bb),
        }

    def _detect_directions(self, mask, roi_h, roi_w):
        """Return set of visible direction names."""
        min_px = int(self.get_parameter("min_branch_pixels").value)
        min_ratio = float(self.get_parameter("min_branch_ratio").value)
        dirs = set()
        for name, (x0, x1, y0, y1) in self._direction_rois(roi_h, roi_w).items():
            region = mask[y0:y1, x0:x1]
            px = int(cv2.countNonZero(region))
            ratio = px / max(1, region.shape[0] * region.shape[1])
            if px >= min_px and ratio >= min_ratio:
                dirs.add(name)
        return dirs

    def _detect(self):
        """Return (cx_norm, directions, blue_pixels, debug_img).
        cx_norm = −1.0 (far left) … +1.0 (far right), or None when no line."""
        if self.latest_image is None:
            return None, set(), 0, None

        img = self.latest_image
        h, w = img.shape[:2]
        top = int(h * float(self.get_parameter("roi_top_fraction").value))
        roi = img[top:, :]
        roi_h, roi_w = roi.shape[:2]

        # ── HSV mask ─────────────────────────────────────────────────────────
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lo = np.array(self.get_parameter("blue_lo").value, dtype=np.uint8)
        hi = np.array(self.get_parameter("blue_hi").value, dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)

        # ── cyan dominance (B and G both high, R low) ────────────────────────
        if self.get_parameter("use_blue_dominance").value:
            b, g, r = cv2.split(roi)
            bi, gi, ri = b.astype(np.int16), g.astype(np.int16), r.astype(np.int16)
            margin = int(self.get_parameter("blue_dominance_margin").value)
            mc = int(self.get_parameter("blue_min_channel").value)
            dom = (
                (bi >= mc) & (gi >= mc) &
                (bi - ri >= margin) & (gi - ri >= margin)
            ).astype(np.uint8) * 255
            mask = cv2.bitwise_or(mask, dom)

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        blue_pixels = int(cv2.countNonZero(mask))
        directions = self._detect_directions(mask, roi_h, roi_w)

        # ── debug image ───────────────────────────────────────────────────────
        debug = img.copy()
        overlay = np.zeros_like(debug)
        blue_ovl = np.zeros((roi_h, roi_w, 3), dtype=np.uint8)
        blue_ovl[mask > 0] = (255, 0, 0)
        overlay[top:, :] = blue_ovl
        debug = cv2.addWeighted(debug, 0.75, overlay, 0.55, 0)
        cv2.line(debug, (w // 2, top), (w // 2, h), (255, 255, 255), 1)
        cv2.drawMarker(debug, (w // 2, h - 1), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        # Draw direction ROI boxes (green = active, grey = inactive)
        for name, (x0, x1, y0, y1) in self._direction_rois(roi_h, roi_w).items():
            color = (0, 255, 0) if name in directions else (80, 80, 80)
            cv2.rectangle(debug, (x0, top + y0), (x1 - 1, top + y1 - 1), color, 1)
            cv2.putText(debug, name[0].upper(), (x0 + 4, top + y0 + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if blue_pixels < int(self.get_parameter("min_blue_pixels").value):
            self._draw_text(debug, f"{self.mode} px={blue_pixels} NO LINE")
            return None, set(), blue_pixels, debug

        m = cv2.moments(mask)
        if m["m00"] <= 0:
            return None, set(), blue_pixels, debug

        cx_pix = m["m10"] / m["m00"]
        cy_pix = top + int(m["m01"] / m["m00"])
        cx_norm = (cx_pix - roi_w / 2.0) / max(1.0, roi_w / 2.0)

        cv2.circle(debug, (int(cx_pix), cy_pix), 10, (0, 255, 0), -1)
        cv2.line(debug, (w // 2, cy_pix), (int(cx_pix), cy_pix), (0, 255, 0), 2)
        dirs_str = ",".join(sorted(directions)) or "-"
        split_tag = " SPLIT" if self.split_active else ""
        self._draw_text(debug,
                        f"{self.mode}{split_tag} px={blue_pixels} "
                        f"err={cx_norm:+.2f} [{dirs_str}]")

        return cx_norm, directions, blue_pixels, debug

    def _draw_text(self, img, text):
        cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (255, 255, 255), 2)
        cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (0, 0, 0), 1)

    # ── main loop ────────────────────────────────────────────────────────────

    def _update(self):
        if not self.enabled:
            return

        self._pub_bool(self.manual_pub, True)
        self._pub_bool(self.patrol_cmd_pub, False)

        now = self._now_s()
        cx_norm, directions, blue_pixels, debug = self._detect()

        if debug is not None:
            # LIDAR distance overlay — top-left second line
            dist = self.front_dist
            lbl = f"LIDAR fwd: {dist:.2f} m" if dist < float("inf") else "LIDAR fwd: --"
            stop_d = float(self.get_parameter("lidar_stop_distance").value)
            slow_d = float(self.get_parameter("lidar_slow_distance").value)
            if dist <= stop_d:
                col = (0, 0, 255)       # red  — U-turn imminent
            elif dist <= slow_d:
                col = (0, 165, 255)     # orange — slowing
            else:
                col = (200, 200, 200)   # grey — clear
            cv2.putText(debug, lbl, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 2)
            cv2.putText(debug, lbl, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)

            if self.get_parameter("publish_debug_image").value:
                try:
                    self.debug_pub.publish(
                        self.bridge.cv2_to_imgmsg(debug, "bgr8"))
                except Exception:
                    pass
            if self.debug_view:
                cv2.imshow("Blue line explorer", debug)
                cv2.waitKey(1)

        self.status_pub.publish(String(
            data=f"mode={self.mode} px={blue_pixels} cx={cx_norm} "
                 f"dirs={','.join(sorted(directions)) or '-'}"))

        # ── U-turn ───────────────────────────────────────────────────────────
        if self.mode == "UTURN":
            if now >= self.uturn_end:
                self.get_logger().info("U-turn done — searching.")
                self.mode = "SEARCH"
                self._stop()
            else:
                # Rotate clockwise (negative = CW) to re-approach from the right side.
                self._pub_twist(
                    0.0, -float(self.get_parameter("uturn_angular_speed").value))
            return

        # ── Search ───────────────────────────────────────────────────────────
        if self.mode == "SEARCH":
            self._pub_state("BLUE_LINE_SEARCH")
            if cx_norm is not None:
                self.get_logger().info("Blue line acquired.")
                self.mode = "FOLLOW"
                self.line_lost_since = None
            else:
                # Rotate clockwise slowly until the line is found.
                spd = float(self.get_parameter("uturn_angular_speed").value)
                self._pub_twist(0.0, -spd * 0.5)
            return

        # ── Follow ───────────────────────────────────────────────────────────
        self._pub_state("BLUE_LINE_FOLLOW")

        if self.front_dist <= float(self.get_parameter("lidar_stop_distance").value):
            self._start_uturn("obstacle ahead")
            return

        if cx_norm is None:
            if self.line_lost_since is None:
                self.line_lost_since = now
            if now - self.line_lost_since >= float(
                    self.get_parameter("line_lost_timeout").value):
                self._start_uturn("line lost")
            else:
                self._stop()
            return

        self.line_lost_since = None

        kp = float(self.get_parameter("kp_steer").value)
        max_ang = float(self.get_parameter("max_angular_speed").value)
        left_bias = float(self.get_parameter("left_bias").value)
        alpha = float(self.get_parameter("angular_smoothing").value)
        exit_hold = float(self.get_parameter("split_exit_hold").value)

        has_branch = bool(directions & {"left", "right"})

        # ── Split-mode state machine ──────────────────────────────────────────
        # Enter: any left/right branch fires → split_active = True.
        # Timer reset: only when "left" is visible (genuine new decision point).
        #   A residual "right" from a just-passed intersection keeps split_active
        #   alive but must NOT reset the exit timer — otherwise we never exit.
        # Exit: centroid inside the straight-box column (abs < 0.30) AND
        #   "straight" is lit, held for split_exit_hold s.
        if has_branch:
            self.split_active = True
            if "left" in directions:
                # Genuine new left branch — restart the re-centre countdown.
                self.split_exit_since = None

        if self.split_active:
            in_straight_zone = "straight" in directions and abs(cx_norm) < 0.30
            if in_straight_zone:
                if self.split_exit_since is None:
                    self.split_exit_since = now
                if now - self.split_exit_since >= exit_hold:
                    self.split_active = False
                    self.split_exit_since = None
            else:
                self.split_exit_since = None

        # ── Speed ─────────────────────────────────────────────────────────────
        linear = float(self.get_parameter(
            "linear_speed" if self.split_active else "fast_linear_speed").value)
        # Creep through the slow zone so the camera has time to spot a left branch.
        if self.front_dist <= float(self.get_parameter("lidar_slow_distance").value):
            linear = float(self.get_parameter("linear_speed").value)

        # ── Steering ──────────────────────────────────────────────────────────
        steer = cx_norm

        if self.split_active:
            # At the actual decision point (left branch visible): steer left.
            # Clamp rightward pull when no left branch AND no straight — avoids
            # the robot drifting right at a straight+right-only junction.
            if "left" in directions:
                # Genuine intersection — prefer left with bias.
                angular_raw = clamp(-kp * steer + left_bias, -max_ang, max_ang)
            else:
                # Re-centering after the turn: pure centroid, no bias fighting it.
                # Clamp only when straight is also absent (right-only residual pixel).
                if "straight" not in directions:
                    steer = min(steer, 0.0)
                angular_raw = clamp(-kp * steer, -max_ang, max_ang)
        else:
            # Pure straight: no bias, centroid only.
            angular_raw = clamp(-kp * steer, -max_ang, max_ang)

        # EMA smoothing — prevents step-changes from causing jerks.
        angular = alpha * angular_raw + (1.0 - alpha) * self.prev_angular
        self.prev_angular = angular

        self._pub_twist(linear, angular)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pub_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._pub_twist(0.0, 0.0)

    def _pub_bool(self, pub, val):
        msg = Bool()
        msg.data = bool(val)
        pub.publish(msg)

    def _pub_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def destroy_node(self):
        self._stop()
        if self.debug_view:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlueLineExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
