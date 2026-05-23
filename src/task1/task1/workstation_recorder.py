import sys
import os
import math

import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point

from ament_index_python.packages import get_package_share_directory

CONFIRM_COUNT = 10
ROBOT_FRAMES = ["base_footprint", "base_link"]


class WorkstationRecorder(Node):
    def __init__(self):
        super().__init__("workstation_recorder")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.line_sub = self.create_subscription(
            Marker, "/line_markers", self.line_callback, 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/workstation_markers", 10
        )

        self.declare_parameter("stop_distance", 1)

        self.red_candidates = []
        self.green_candidates = []
        self.red_endpoints = []
        self.green_endpoints = []
        self.red_locked = False
        self.green_locked = False
        self.red_markers = []
        self.green_markers = []

        self.mode = "normal"
        if "--mode" in sys.argv:
            idx = sys.argv.index("--mode")
            if idx + 1 < len(sys.argv):
                self.mode = sys.argv[idx + 1]

        self.create_timer(2.0, self._republish)
        self.get_logger().info(f"WorkstationRecorder ready (mode={self.mode}).")

    def _get_robot_pos(self):
        for frame in ROBOT_FRAMES:
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                return np.array([
                    t.transform.translation.x,
                    t.transform.translation.y,
                ])
            except Exception:
                continue
        return None

    def _to_map(self, point_xyz, source_frame, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = stamp
        ps.point.x = float(point_xyz[0])
        ps.point.y = float(point_xyz[1])
        ps.point.z = float(point_xyz[2])
        t = do_transform_point(ps, transform)
        return np.array([t.point.x, t.point.y, t.point.z])

    def _make_markers(self, pos, yaw, is_red):
        base_id = 0 if is_red else 1
        color_name = "RED" if is_red else "GREEN"

        cyl = Marker()
        cyl.header.frame_id = "map"
        cyl.header.stamp = self.get_clock().now().to_msg()
        cyl.ns = "workstations"
        cyl.id = base_id
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.lifetime = Duration(sec=0)
        cyl.pose.position.x = float(pos[0])
        cyl.pose.position.y = float(pos[1])
        cyl.pose.position.z = 0.0
        cyl.pose.orientation.z = math.sin(yaw / 2.0)
        cyl.pose.orientation.w = math.cos(yaw / 2.0)
        cyl.scale.x = 0.3
        cyl.scale.y = 0.3
        cyl.scale.z = 0.1
        if is_red:
            cyl.color.r = 1.0
            cyl.color.g = 0.0
            cyl.color.b = 0.0
        else:
            cyl.color.r = 0.0
            cyl.color.g = 1.0
            cyl.color.b = 0.0
        cyl.color.a = 1.0

        txt = Marker()
        txt.header.frame_id = "map"
        txt.header.stamp = cyl.header.stamp
        txt.ns = "workstations"
        txt.id = base_id + 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.lifetime = Duration(sec=0)
        txt.pose.position.x = float(pos[0])
        txt.pose.position.y = float(pos[1])
        txt.pose.position.z = 0.12
        txt.scale.z = 0.2
        txt.color.r = 0.0
        txt.color.g = 0.0
        txt.color.b = 0.0
        txt.color.a = 1.0
        txt.text = "WS"
        return [cyl, txt]

    def line_callback(self, msg):
        if msg.type != Marker.LINE_STRIP:
            return
        if len(msg.points) < 2:
            return

        r, g = msg.color.r, msg.color.g
        is_red = r > 0.5 and g < 0.3
        is_green = g > 0.5 and r < 0.3
        if not (is_red or is_green):
            return
        if (is_red and self.red_locked) or (is_green and self.green_locked):
            return

        robot_xy = self._get_robot_pos()
        if robot_xy is None:
            self.get_logger().warn("Could not get robot position")
            return

        p1 = np.array([msg.points[0].x, msg.points[0].y, msg.points[0].z])
        p2 = np.array([msg.points[1].x, msg.points[1].y, msg.points[1].z])
        center = (p1 + p2) / 2.0

        map_center = self._to_map(center, msg.header.frame_id, msg.header.stamp)
        if map_center is None:
            return
        map_p1 = self._to_map(p1, msg.header.frame_id, msg.header.stamp)
        map_p2 = self._to_map(p2, msg.header.frame_id, msg.header.stamp)
        if map_p1 is None or map_p2 is None:
            return

        dx = robot_xy[0] - map_center[0]
        dy = robot_xy[1] - map_center[1]
        robot_dist = math.hypot(dx, dy)
        if robot_dist < 0.01:
            return

        stop_dist = self.get_parameter("stop_distance").value
        nx, ny = dx / robot_dist, dy / robot_dist
        marker_pos = np.array([
            map_center[0] + stop_dist * nx,
            map_center[1] + stop_dist * ny,
        ])
        yaw = math.atan2(-dy, -dx)

        candidates = self.red_candidates if is_red else self.green_candidates
        candidates.append((marker_pos[0], marker_pos[1], yaw))
        endpoints = self.red_endpoints if is_red else self.green_endpoints
        endpoints.append((map_p1, map_p2))

        if len(candidates) < CONFIRM_COUNT:
            return

        arr = np.array(candidates)
        median_pos = np.array([np.median(arr[:, 0]), np.median(arr[:, 1])])
        median_yaw = np.median(arr[:, 2])

        markers = self._make_markers(median_pos, median_yaw, is_red)

        if is_red:
            self.red_markers = markers
            self.red_locked = True
            self.get_logger().info(
                f"RED workstation locked at ({median_pos[0]:.2f}, {median_pos[1]:.2f})"
            )
        else:
            self.green_markers = markers
            self.green_locked = True
            self.get_logger().info(
                f"GREEN workstation locked at ({median_pos[0]:.2f}, {median_pos[1]:.2f})"
            )

        self._republish()

        if self.mode == "toYAML":
            self._write_yaml()

    def _write_yaml(self):
        if not (self.red_locked or self.green_locked):
            return

        pkg_path = get_package_share_directory("task1")
        yaml_path = os.path.join(pkg_path, "config", "test_workstation_locations.yaml")

        data = {"workstations": {}}

        for key, is_red, markers, ep_list in [
            ("red", True, self.red_markers, self.red_endpoints),
            ("green", False, self.green_markers, self.green_endpoints),
        ]:
            if not markers:
                continue
            m = markers[0]
            yaw = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)

            ep_arr = np.array(ep_list)
            median_p1 = np.median(ep_arr[:, 0, :], axis=0)
            median_p2 = np.median(ep_arr[:, 1, :], axis=0)

            data["workstations"][key] = {
                "approach": {
                    "x": float(m.pose.position.x),
                    "y": float(m.pose.position.y),
                    "yaw": yaw,
                },
                "line_start": {
                    "x": float(median_p1[0]),
                    "y": float(median_p1[1]),
                    "z": float(median_p1[2]),
                },
                "line_end": {
                    "x": float(median_p2[0]),
                    "y": float(median_p2[1]),
                    "z": float(median_p2[2]),
                },
            }

        os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, indent=2)
        self.get_logger().info(f"Workstation locations written to {yaml_path}")

    def _republish(self):
        arr = MarkerArray()
        arr.markers.extend(self.red_markers)
        arr.markers.extend(self.green_markers)
        if arr.markers:
            self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WorkstationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
