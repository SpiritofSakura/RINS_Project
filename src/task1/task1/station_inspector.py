import os
import sys
import math
import subprocess
import signal
import cv2
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, PointStamped, Point
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point

import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


INSPECTOR_STATES = {
    "INSPECTOR_INACTIVE",
    "LOAD_YAML",
    "NAV_TO_WS",
    "FINE_POSITION",
    "SCAN_TILES",
    "TILE_FOUND",
    "ESCAPING_WORKSTATION",
}


class StationInspector(Node):
    def __init__(self):
        super().__init__("station_inspector")
        self.declare_parameter("workstation", "green")
        self.declare_parameter("use_yaml", True)
        self.declare_parameter("use_orchestrator", False)
        self.ws_key = self.get_parameter("workstation").value
        self.use_yaml = self.get_parameter("use_yaml").value
        self.use_orchestrator = self.get_parameter("use_orchestrator").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)

        self._set_state("INSPECTOR_INACTIVE")
        self.nav_goal_sent = False
        self.nav_goal_done = False
        self.nav_succeeded = False

        self.workspace_data = None
        self.approach_pose = None

        self.fine_pos_phase = 0
        self.latest_scan = None
        self._scan_logged = False
        self.bridge = CvBridge()
        self.latest_gray = None
        self.camera_sub = self.create_subscription(
            Image, "/top_camera/rgb/preview/image_raw",
            self._camera_callback, 10
        )
        self.latest_oakd = None
        self.oakd_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw",
            self._oakd_callback, 10
        )
        self.marker_pub = self.create_publisher(Marker, "/inspector/rear_cone", 10)
        self._tile_stop_start = None
        self._tile_found = False
        self._stop_reason = None
        self.phase_pub = self.create_publisher(Int32, "/inspector_phase", 10)
        self.station_pub = self.create_publisher(String, "/inspector_station", 10)
        self.finish_pub = self.create_publisher(String, "/inspector_finish", 10)
        self.tile_status_sub = self.create_subscription(
            String, "/tile_status", self._tile_status_callback, 10
        )
        self._orch_sub = self.create_subscription(
            PoseStamped, "/orchestrator_out", self._orchestrator_cb, 10
        )
        self._orch_in_pub = self.create_publisher(
            String, "/orchestrator_in", 10
        )
        self._orch_requested = False
        self._orch_received = False
        self._tile_process = None
        self._classifier_process = None
        self._phase5_sub = 0
        self._end_belt_start = None
        self._phase5_start = None
        self._finish_sent = False
        self._finish_shutdown_at = None

        self.create_timer(0.1, self._update)
        self.get_logger().info(
            f"StationInspector ready (workstation={self.ws_key})."
        )

    def _set_state(self, new_state):
        if new_state not in INSPECTOR_STATES:
            return
        self.state = new_state
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)

    def _publish_substate(self, label):
        msg = String()
        msg.data = label
        self.state_pub.publish(msg)

    def _resolve_yaml_path(self):
        try:
            pkg_share = get_package_share_directory("task1")
            path = os.path.join(pkg_share, "config", "test_workstation_locations.yaml")
            if os.path.exists(path):
                return path
        except Exception:
            pass
        src_path = os.path.join(
            os.path.dirname(__file__), "..", "config",
            "test_workstation_locations.yaml"
        )
        return os.path.abspath(src_path)

    def _load_yaml(self):
        path = self._resolve_yaml_path()
        if not os.path.exists(path):
            return False
        with open(path, "r", encoding="utf-8") as f:
            self.workspace_data = yaml.safe_load(f)
        if (self.workspace_data is None or "workstations" not in self.workspace_data
                or self.ws_key not in self.workspace_data["workstations"]):
            self.get_logger().error(f"Workstation '{self.ws_key}' not found")
            return False
        ws = self.workspace_data["workstations"][self.ws_key]
        ap = ws["approach"]
        self.approach_pose = (ap["x"], ap["y"], ap["yaw"])
        self.get_logger().info(f"Loaded {self.ws_key} approach: ({ap['x']:.2f}, {ap['y']:.2f})")
        return True

    def _send_nav_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        self.get_logger().info(f"Sending Nav2 goal: ({x:.2f}, {y:.2f}), yaw={yaw:.2f}")
        self.nav_goal_sent = True
        self.nav_goal_done = False
        self.nav_succeeded = False
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        try:
            handle = future.result()
        except Exception:
            self.nav_goal_done = True
            return
        if not handle.accepted:
            self.nav_goal_done = True
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        self.nav_goal_done = True
        try:
            result = future.result()
            self.nav_succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception:
            pass

    def _get_robot_map_pose(self):
        for frame in ["base_footprint", "base_link"]:
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                pos = np.array([t.transform.translation.x, t.transform.translation.y])
                qz, qw = t.transform.rotation.z, t.transform.rotation.w
                yaw = 2.0 * math.atan2(qz, qw)
                return pos, yaw
            except Exception:
                continue
        return None, None

    def _stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _publish_arm(self, cmd):
        msg = String()
        msg.data = cmd
        self.arm_pub.publish(msg)

    def _camera_callback(self, msg):
        try:
            self.latest_gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            pass

    def _hough_tilt(self):
        if self.latest_gray is None:
            return None, None
        gray = self.latest_gray
        h, w = gray.shape

        dy = np.abs(np.diff(gray.astype(np.int16), axis=0)).astype(np.uint16)
        dy_norm = (dy / (dy.max() + 1e-6) * 255).astype(np.uint8)

        top = dy_norm[:max(1, h // 2), :]
        blur = cv2.GaussianBlur(top, (3, 3), 0)
        _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        lines = cv2.HoughLines(binary, 1, math.pi / 180, threshold=80)
        if lines is None:
            return None, None

        for line in lines:
            rho, theta = line[0]
            if abs(theta - math.pi / 2) < math.radians(20):
                return rho, theta
        return None, None

    def _oakd_callback(self, msg):
        try:
            self.latest_oakd = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _check_yellow_bottom_left(self):
        if self.latest_oakd is None:
            return False
        hsv = cv2.cvtColor(self.latest_oakd, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        roi_hsv = hsv[max(0, h - 30):, :w // 2]
        lower = np.array([20, 80, 80])
        upper = np.array([35, 255, 255])
        mask = cv2.inRange(roi_hsv, lower, upper)
        ratio = cv2.countNonZero(mask) / max(1, roi_hsv.shape[0] * roi_hsv.shape[1])
        return ratio > 0.5

    def _tile_status_callback(self, msg):
        if msg.data == "TILE_FOUND":
            self._tile_found = True
        elif msg.data == "TILE_LEFT":
            self._tile_found = False

    def _orchestrator_cb(self, msg):
        color = msg.header.frame_id
        if color != self.ws_key:
            return
        qz, qw = msg.pose.orientation.z, msg.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.approach_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self._orch_received = True
        self.get_logger().info(
            f"Orchestrator gave {color} waypoint: "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), yaw={yaw:.2f}"
        )

    def _check_workstation_color_bottom_left(self):
        if self.latest_oakd is None:
            return True
        hsv = cv2.cvtColor(self.latest_oakd, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        roi = hsv[max(0, h - 30):, :w // 2]
        if self.ws_key == "red":
            m1 = cv2.inRange(roi, np.array([0, 80, 80]), np.array([10, 255, 255]))
            m2 = cv2.inRange(roi, np.array([170, 80, 80]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(roi, np.array([40, 80, 80]), np.array([80, 255, 255]))
        ratio = cv2.countNonZero(mask) / max(1, roi.shape[0] * roi.shape[1])
        return ratio > 0.15

    def _get_rear_distance(self):
        if self.latest_scan is None:
            return None
        center = math.radians(200) - math.pi / 2.0
        half = math.radians(20.0)
        return self._min_in_cone(self.latest_scan, center, half)

    def _publish_rear_cone_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rear_cone"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.a = 0.8
        for deg in range(180, 221, 2):
            rad = math.radians(deg)
            p = Point()
            p.x = math.cos(rad) * 0.8
            p.y = math.sin(rad) * 0.8
            p.z = 0.0
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def _scan_callback(self, msg):
        self.latest_scan = msg
        if not self._scan_logged:
            self._scan_logged = True
            forward_idx = int((-math.pi / 2.0 - msg.angle_min) / msg.angle_increment)
            forward_range = msg.ranges[forward_idx] if 0 <= forward_idx < len(msg.ranges) else -1
            self.get_logger().info(
                f"LiDAR: angle_min={msg.angle_min:.2f} angle_max={msg.angle_max:.2f} "
                f"inc={msg.angle_increment:.4f} n={len(msg.ranges)} "
                f"forward_idx={forward_idx} forward_range={forward_range:.2f}m"
            )

    def _get_min_forward_distance(self):
        if self.latest_scan is None:
            return None
        return self._min_in_cone(self.latest_scan, -math.pi / 2.0, math.radians(45.0))

    @staticmethod
    def _min_in_cone(scan, center_angle, half_cone):
        min_dist = float("inf")
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            diff = angle - center_angle
            while diff > math.pi:
                diff -= 2.0 * math.pi
            while diff < -math.pi:
                diff += 2.0 * math.pi
            if abs(diff) <= half_cone and r < min_dist:
                min_dist = r
        return min_dist if min_dist != float("inf") else None

    def _update(self):
        if self._finish_sent:
            self.finish_pub.publish(String(data="finished"))
            if (self._finish_shutdown_at is not None
                    and self.get_clock().now().nanoseconds >= self._finish_shutdown_at):
                rclpy.shutdown()
            return

        if self.state == "INSPECTOR_INACTIVE":
            if self.use_yaml and self._load_yaml():
                self._set_state("NAV_TO_WS")
                self.nav_goal_sent = False
                return
            if self.use_orchestrator and self._orch_received:
                self._orch_received = False
                self._set_state("NAV_TO_WS")
                self.nav_goal_sent = False
                return
            if self.use_orchestrator and not self._orch_requested:
                cmd = String()
                cmd.data = f"get_{self.ws_key}_waypoint"
                self._orch_in_pub.publish(cmd)
                self._orch_requested = True
                self.get_logger().info(
                    f"Requested {self.ws_key} waypoint from orchestrator"
                )
            return

        if self.state == "NAV_TO_WS":
            if not self.nav_goal_sent:
                if not self.nav_client.wait_for_server(timeout_sec=0.1):
                    return
                x, y, yaw = self.approach_pose
                self._send_nav_goal(x, y, yaw)
            elif self.nav_goal_done:
                if self.nav_succeeded:
                    self.get_logger().info("Approach point reached. Fine-positioning.")
                    if self._tile_process is None:
                        from ament_index_python.packages import get_package_prefix
                        _prefix = get_package_prefix("task1")
                        _lib = os.path.join(_prefix, "lib", "task1")
                        self._tile_process = subprocess.Popen(
                            [sys.executable, os.path.join(_lib, "tile_detect")],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
                        )
                        self.get_logger().info(f"Started tile_detect ({_lib})")
                        self._classifier_process = subprocess.Popen(
                            [sys.executable, os.path.join(_lib, "tile_classifier")],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
                        )
                        self.get_logger().info(f"Started tile_classifier ({_lib})")
                    self.station_pub.publish(String(data=self.ws_key))
                    self.get_logger().info(f"Published station: {self.ws_key}")
                    self._publish_arm("look_at_belt_left")
                    self._set_state("FINE_POSITION")
                    self.fine_pos_phase = 0
                else:
                    self.get_logger().error("Nav to approach point failed, retrying")
                    self.nav_goal_sent = False

        elif self.state == "FINE_POSITION":
            phase_msg = Int32()
            phase_msg.data = self.fine_pos_phase
            self.phase_pub.publish(phase_msg)
            if self.fine_pos_phase == 0:
                min_dist = self._get_min_forward_distance()
                if min_dist is None:
                    self._stop_robot()
                    return

                self.get_logger().info(
                    f"Phase 0 forward={min_dist:.3f}m",
                    throttle_duration_sec=0.5,
                )

                if min_dist <= 0.30:
                    self._stop_robot()
                    self.get_logger().info(
                        f"Forward wall reached at {min_dist:.2f}m, starting parallel alignment"
                    )
                    self.fine_pos_phase = 1
                    return

                speed = 0.03 if min_dist <= 0.45 else 0.15
                msg = Twist()
                msg.linear.x = speed
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)

            elif self.fine_pos_phase == 1:
                target_yaw = math.pi if self.ws_key == "red" else math.pi / 2.0
                _, current_yaw = self._get_robot_map_pose()
                if current_yaw is None:
                    return

                diff = target_yaw - current_yaw
                while diff > math.pi:
                    diff -= 2.0 * math.pi
                while diff < -math.pi:
                    diff += 2.0 * math.pi

                self.get_logger().info(
                    f"Phase 1 yaw={current_yaw:.3f} target={target_yaw:.3f} diff={diff:.3f}",
                    throttle_duration_sec=0.3,
                )

                if abs(diff) <= 0.05:
                    self._stop_robot()
                    self.get_logger().info(
                        f"Reached target yaw {target_yaw:.3f} rad, starting camera fine-tune"
                    )
                    self.fine_pos_phase = 2
                    return

                ang = max(min(0.5 * diff, 0.4), -0.4)
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = ang
                self.cmd_pub.publish(msg)

            elif self.fine_pos_phase == 2:
                rho, theta = self._hough_tilt()

                if self.latest_gray is not None:
                    h, w = self.latest_gray.shape
                    dy = np.abs(np.diff(self.latest_gray.astype(np.int16), axis=0)).astype(np.uint16)
                    dy_norm = (dy / (dy.max() + 1e-6) * 255).astype(np.uint8)
                    top = dy_norm[:max(1, h // 2), :]
                    blur = cv2.GaussianBlur(top, (3, 3), 0)
                    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                    if theta is not None:
                        a = math.cos(theta)
                        b = math.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        tilt = math.degrees(theta - math.pi / 2)
                        cv2.putText(vis, f"tilt={tilt:+.1f}deg", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow("Phase 2 Hough alignment", vis)
                    cv2.waitKey(1)

                if theta is None:
                    self.get_logger().info(
                        "Phase 2 waiting for Hough line...",
                        throttle_duration_sec=1.0,
                    )
                    return

                angle_deg = math.degrees(theta - math.pi / 2)
                self.get_logger().info(
                    f"Phase 2 tilt={angle_deg:+.1f}deg",
                    throttle_duration_sec=0.3,
                )

                if abs(angle_deg) <= 0.5:
                    self._stop_robot()
                    self.get_logger().info(
                        f"Perfectly aligned (tilt={angle_deg:+.1f}deg), starting backup"
                    )
                    cv2.destroyWindow("Phase 2 Hough alignment")
                    self.fine_pos_phase = 3
                    return

                ang = max(min(-0.15 * angle_deg, 0.1), -0.1)
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = ang
                self.cmd_pub.publish(msg)

            elif self.fine_pos_phase == 3:
                self._publish_rear_cone_marker()

                rear_dist = self._get_rear_distance()
                yellow_ok = False

                if self.latest_oakd is not None:
                    h, w = self.latest_oakd.shape[:2]
                    roi = self.latest_oakd[max(0, h - 30):, :w // 2]
                    roi_pix = roi.shape[0] * roi.shape[1]
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                    yel = (cv2.inRange(hsv_roi, np.array([20, 80, 80]), np.array([35, 255, 255])) > 0).sum()
                    red = ((cv2.inRange(hsv_roi, np.array([0, 80, 80]), np.array([10, 255, 255])) > 0).sum() +
                           (cv2.inRange(hsv_roi, np.array([170, 80, 80]), np.array([180, 255, 255])) > 0).sum())
                    grn = (cv2.inRange(hsv_roi, np.array([40, 80, 80]), np.array([80, 255, 255])) > 0).sum()

                    yel_ratio = yel / max(roi_pix, 1)
                    yellow_ok = yel > (red + grn) / 2.0

                    debug = roi.copy()
                    debug[cv2.inRange(hsv_roi, np.array([20, 80, 80]), np.array([35, 255, 255])) > 0] = (0, 255, 255)
                    dist_str = f"{rear_dist:.2f}m" if rear_dist else "N/A"
                    cv2.putText(debug, f"Y_ratio={yel_ratio:.2f} dist={dist_str}",
                                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    color_reason = getattr(self, "_stop_reason", None)
                    if color_reason:
                        cv2.putText(debug, f"trigger: {color_reason}",
                                    (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    cv2.imshow("Phase 3 backup", debug)
                    cv2.waitKey(1)

                self.get_logger().info(
                    f"Phase 3 backup rear={rear_dist:.2f}m yellow_ok={yellow_ok}",
                    throttle_duration_sec=0.5,
                )

                if yellow_ok:
                    self._stop_robot()
                    self._stop_reason = "True (yellow)"
                    self.get_logger().info("Backup complete: yellow line detected")
                    self.fine_pos_phase = 4
                    self._publish_substate("SCAN_TILES")
                    return

                if rear_dist is not None and rear_dist <= 0.40:
                    self._stop_robot()
                    self._stop_reason = "False (rear obstacle)"
                    self.get_logger().info("Backup complete: rear obstacle at 0.40m")
                    self.fine_pos_phase = 4
                    self._publish_substate("SCAN_TILES")
                    return

                msg = Twist()
                msg.linear.x = -0.15
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)

            elif self.fine_pos_phase == 4:
                color_present = self._check_workstation_color_bottom_left()
                no_colour = not color_present

                if self.latest_oakd is not None:
                    debug = self.latest_oakd[max(0, self.latest_oakd.shape[0] - 30):, :self.latest_oakd.shape[1] // 2].copy()
                    if no_colour:
                        cv2.putText(debug, "END OF STATION", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    else:
                        cv2.putText(debug, "scanning...", (5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.imshow("Phase 4 station status", debug)
                    cv2.waitKey(1)

                now = self.get_clock().now()

                if no_colour:
                    if self._end_belt_start is None:
                        self._end_belt_start = now
                        self.get_logger().info("No colour — driving 5s more for final tiles")

                if self._tile_stop_start is not None:
                    elapsed_s = (now - self._tile_stop_start).nanoseconds / 1e9
                    if elapsed_s >= (4.0 if self._end_belt_start is not None else 2.0):
                        self._tile_stop_start = None
                        self._tile_found = False
                        if self._end_belt_start is not None:
                            self.get_logger().info("Final tile done, escaping")
                            self._stop_robot()
                            self._publish_arm("look_for_spill")
                            self.fine_pos_phase = 5
                            self._publish_substate("ESCAPING_WORKSTATION")
                            self._phase5_sub = 0
                            self._phase5_start = None
                            return
                        self._publish_substate("SCAN_TILES")
                        self.get_logger().info("Resuming after tile stop")
                    else:
                        return
                    return

                if self._tile_found and self._tile_stop_start is None:
                    self._stop_robot()
                    self._tile_stop_start = now
                    self._publish_substate("TILE_FOUND")
                    self.get_logger().info("Tile found, stopping for 2s")
                    return

                if no_colour:
                    elapsed = (now - self._end_belt_start).nanoseconds / 1e9
                    if elapsed >= 5.0:
                        self.get_logger().info("No tile in 5s without color, escaping")
                        self._stop_robot()
                        self._publish_arm("look_for_spill")
                        self.fine_pos_phase = 5
                        self._publish_substate("ESCAPING_WORKSTATION")
                        self._phase5_sub = 0
                        self._phase5_start = None
                        return
                else:
                    self._end_belt_start = None

                msg = Twist()
                msg.linear.x = 0.08
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)

            elif self.fine_pos_phase == 5:
                if self._phase5_sub == 0:
                    _, yaw = self._get_robot_map_pose()
                    if yaw is None:
                        return
                    if self._phase5_start is None:
                        self._phase5_start = yaw
                        self.get_logger().info("Starting 130deg CW turn")
                    target = self._phase5_start - math.radians(130)
                    diff = target - yaw
                    while diff > math.pi:
                        diff -= 2.0 * math.pi
                    while diff < -math.pi:
                        diff += 2.0 * math.pi
                    if abs(diff) <= 0.05:
                        self._stop_robot()
                        self._phase5_sub = 1
                        self._phase5_start = self.get_clock().now()
                        self.get_logger().info("Turn complete, driving forward 1.5s")
                        return
                    ang = max(min(0.5 * diff, 0.4), -0.4)
                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = ang
                    self.cmd_pub.publish(msg)

                elif self._phase5_sub == 1:
                    elapsed = (self.get_clock().now() - self._phase5_start).nanoseconds / 1e9
                    if elapsed >= 4:
                        self._stop_robot()
                        self._phase5_sub = 2
                        self._publish_finish_and_shutdown()
                        return
                    msg = Twist()
                    msg.linear.x = 0.3
                    msg.angular.z = 0.0
                    self.cmd_pub.publish(msg)


    def _publish_finish_and_shutdown(self):
        if self._finish_sent:
            return
        self._stop_robot()
        self._finish_sent = True
        self._finish_shutdown_at = self.get_clock().now().nanoseconds + int(0.5 * 1e9)
        self.finish_pub.publish(String(data="finished"))
        self.get_logger().info("Escape complete, published inspector_finish=finished; shutting down")

    def _kill_proc(self, proc, name):
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        self.get_logger().info(f"Terminated {name}")

    def destroy_node(self):
        self._kill_proc(self._tile_process, "tile_detect")
        self._tile_process = None
        self._kill_proc(self._classifier_process, "tile_classifier")
        self._classifier_process = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StationInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
