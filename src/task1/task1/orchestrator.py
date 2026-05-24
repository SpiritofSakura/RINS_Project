#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")

        self._waypoints = {}

        self._ws_sub = self.create_subscription(
            MarkerArray, "/workstation_markers", self._ws_cb, 10
        )

        self._cmd_sub = self.create_subscription(
            String, "/orchestrator_in", self._cmd_cb, 10
        )

        self._out_pub = self.create_publisher(
            PoseStamped, "/orchestrator_out", 10
        )

        self.get_logger().info("Orchestrator ready.")

    def _ws_cb(self, msg):
        for marker in msg.markers:
            if marker.type != Marker.CYLINDER:
                continue
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.orientation.z
            w = marker.pose.orientation.w
            yaw = 2.0 * math.atan2(z, w)

            if marker.color.r > 0.5 and marker.color.g < 0.3:
                color = "red"
            elif marker.color.g > 0.5 and marker.color.r < 0.3:
                color = "green"
            else:
                continue

            if color not in self._waypoints:
                self.get_logger().info(
                    f"Stored {color} waypoint: ({x:.2f}, {y:.2f}), yaw={yaw:.2f}"
                )
            self._waypoints[color] = (x, y, yaw)

    def _cmd_cb(self, msg):
        cmd = msg.data.strip()
        if cmd == "get_green_waypoint":
            color = "green"
        elif cmd == "get_red_waypoint":
            color = "red"
        else:
            self.get_logger().warn(f"Unknown command: '{cmd}'")
            return

        if color not in self._waypoints:
            self.get_logger().warn(f"No waypoint stored for {color}")
            return

        x, y, yaw = self._waypoints[color]
        out = PoseStamped()
        out.header.frame_id = color
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self._out_pub.publish(out)
        self.get_logger().info(
            f"Published {color} waypoint: ({x:.2f}, {y:.2f}), yaw={yaw:.2f}"
        )


def main():
    rclpy.init()
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
