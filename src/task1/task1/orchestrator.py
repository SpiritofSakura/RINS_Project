import math
import os
import subprocess
import sys
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Orchestrator(Node):
    def __init__(self):
        super().__init__("orchestrator")

        self._waypoints = {}
        self._pending_defects = set()
        self._inspector_proc = None
        self._saved_pose = None
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._return_goal_done = False
        self._return_goal_sent = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._ws_sub = self.create_subscription(
            MarkerArray, "/workstation_markers", self._ws_cb, 10
        )

        self._cmd_sub = self.create_subscription(
            String, "/orchestrator_in", self._cmd_cb, 10
        )

        self._qr_sub = self.create_subscription(
            String, "/qr", self._qr_cb, 10
        )

        self._out_pub = self.create_publisher(
            PoseStamped, "/orchestrator_out", 10
        )

        self.create_timer(0.1, self._update)
        self.get_logger().info("Orchestrator ready.")

    def _get_robot_pose(self):
        for frame in ["base_footprint", "base_link"]:
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                pos = (t.transform.translation.x, t.transform.translation.y)
                qz, qw = t.transform.rotation.z, t.transform.rotation.w
                yaw = 2.0 * math.atan2(qz, qw)
                return pos, yaw
            except Exception:
                continue
        return None, None

    def _nav_to_pose(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f}), yaw={yaw:.2f}")
        self._return_goal_sent = False
        self._return_goal_done = False
        future = self._nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._nav_response_cb)

    def _nav_response_cb(self, future):
        try:
            handle = future.result()
        except Exception:
            self._return_goal_done = True
            return
        if not handle.accepted:
            self._return_goal_done = True
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        self._return_goal_done = True
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation succeeded")
            else:
                self.get_logger().warn("Navigation failed")
        except Exception:
            pass

    def _publish_waypoint(self, color):
        if color not in self._waypoints:
            self.get_logger().warn(f"No waypoint for {color}")
            return False
        x, y, yaw = self._waypoints[color]
        out = PoseStamped()
        out.header.frame_id = color
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = x
        out.pose.position.y = y
        out.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.orientation.w = math.cos(yaw / 2.0)
        self._out_pub.publish(out)
        self.get_logger().info(f"Published {color} waypoint to orchestrator_out")
        return True

    def _launch_inspector(self, color):
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix("task1")
        inspector_path = os.path.join(prefix, "lib", "task1", "station_inspector")
        if not os.path.exists(inspector_path):
            self.get_logger().error(f"Inspector not found: {inspector_path}")
            return False
        self._inspector_proc = subprocess.Popen(
            [sys.executable, inspector_path,
             "--ros-args", "-p", f"workstation:={color}",
             "-p", "use_yaml:=False",
             "-p", "use_orchestrator:=True"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
        )
        self.get_logger().info(f"Launched inspector for {color} (pid={self._inspector_proc.pid})")
        return True

    def _start_defect_flow(self, color):
        self.get_logger().info(f"Starting defect flow for {color}")
        pos, yaw = self._get_robot_pose()
        if pos is None:
            self.get_logger().error("Cannot get current robot pose, aborting defect flow")
            return
        self._saved_pose = (pos[0], pos[1], yaw)
        self.get_logger().info(f"Saved current pose: ({pos[0]:.2f}, {pos[1]:.2f}), yaw={yaw:.2f}")
        self._publish_waypoint(color)
        self._launch_inspector(color)

    def _qr_cb(self, msg):
        lower = msg.data.strip().lower()
        if lower.startswith("defects "):
            color = lower.split(" ", 1)[1]
            if color in ("red", "green"):
                self._pending_defects.add(color)
                self.get_logger().info(f"QR requested defects for {color}")
                self._try_defect_flow(color)

    def _try_defect_flow(self, color):
        if color not in self._waypoints:
            self.get_logger().info(f"Waiting for {color} waypoint before starting defect flow")
            return
        if self._inspector_proc is not None:
            self.get_logger().info("Inspector already running, deferring")
            return
        self._pending_defects.discard(color)
        self._start_defect_flow(color)

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

            was_new = color not in self._waypoints
            self._waypoints[color] = (x, y, yaw)

            if was_new:
                self.get_logger().info(
                    f"Stored {color} waypoint: ({x:.2f}, {y:.2f}), yaw={yaw:.2f}"
                )

            if color in self._pending_defects:
                self._try_defect_flow(color)

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

    def _update(self):
        if self._inspector_proc is not None:
            ret = self._inspector_proc.poll()
            if ret is not None:
                self.get_logger().info(f"Inspector finished (return code {ret})")
                self._inspector_proc = None
                if self._saved_pose is not None:
                    x, y, yaw = self._saved_pose
                    self._saved_pose = None
                    self._nav_to_pose(x, y, yaw)
                    self._return_goal_sent = True

        if self._return_goal_sent and self._return_goal_done:
            self.get_logger().info("Returned to original position")
            self._return_goal_sent = False


def main():
    rclpy.init()
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node._inspector_proc is not None:
        node._inspector_proc.terminate()
        try:
            node._inspector_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            node._inspector_proc.kill()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
