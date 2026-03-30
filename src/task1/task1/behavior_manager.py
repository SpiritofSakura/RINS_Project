#!/usr/bin/env python3

import math
import random
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String, Empty
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        self.manual_control_active = False
        self.patrol_requested = False
        self.patrol_finished = False
        self.current_state = 'IDLE'

        self.active_target = None
        self.handled_targets = []
        self.target_match_threshold = 0.6

        # Queue for pending detections (captured before AMCL is ready)
        self.pending_targets = []  # List of {type, x, y, z, color}
        self.amcl_pose_ready = False

        self.latest_robot_pose = None
        self.saved_patrol_pose = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_server_ready = False
        self.waiting_for_goal_accept = False
        self.goal_active = False
        self.goal_handle = None
        self.result_future = None
        self.nav_goal_type = None

        self.interaction_active = False
        self.interaction_end_time = None
        self.interaction_duration = 2.5

        self.face_lines = [
            "Hello there. I come in peace.",
            "Greetings, human.",
            "Hello there. You have been officially detected.",
            "Another successful social interaction.",
            "Hello. I am legally required to be polite.",
            "Greetings. Nice face. Very recognizable.",
            "Hello there. My sensors approve.",
        ]

        self.ring_lines = [
            "I found a {color} ring. Very stylish.",
            "Behold. A {color} ring.",
            "This ring is {color}. Excellent taste.",
            "A {color} ring has been detected. Fancy.",
            "I found a {color} ring. Fashion approved.",
            "A {color} ring. Quite the dramatic choice.",
            "This appears to be a {color} ring. Impressive.",
        ]

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.state_publisher = self.create_publisher(String, '/robot_state', qos_latched)
        self.patrol_enabled_publisher = self.create_publisher(Bool, '/patrol_enabled', qos_latched)

        self.manual_control_subscriber = self.create_subscription(
            Bool,
            '/manual_control_active',
            self.manual_control_callback,
            qos_latched
        )

        self.patrol_command_subscriber = self.create_subscription(
            Bool,
            '/patrol_command',
            self.patrol_command_callback,
            qos_latched
        )

        self.patrol_finished_subscriber = self.create_subscription(
            Bool,
            '/patrol_finished',
            self.patrol_finished_callback,
            qos_latched
        )

        self.face_subscriber = self.create_subscription(
            Marker,
            '/detected_face_locations',
            self.face_callback,
            10
        )

        self.ring_subscriber = self.create_subscription(
            Marker,
            '/detected_ring_locations',
            self.ring_callback,
            10
        )

        self.target_done_subscriber = self.create_subscription(
            Empty,
            '/target_done',
            self.target_done_callback,
            10
        )

        self.resume_patrol_subscriber = self.create_subscription(
            Empty,
            '/resume_patrol',
            self.resume_patrol_callback,
            10
        )

        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.main_loop)

        self.refresh_state(force_publish=True)

        self.get_logger().info('Behavior manager started.')

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def publish_state(self, state, force_publish=False):
        if state == self.current_state and not force_publish:
            return

        self.current_state = state
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Robot state -> {state}')

    def publish_patrol_enabled(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.patrol_enabled_publisher.publish(msg)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_robot_pose = msg.pose.pose
        
        # Mark AMCL as ready on first message
        if not self.amcl_pose_ready:
            self.amcl_pose_ready = True
            self.get_logger().info('AMCL pose is now ready. Processing pending detections.')

    def save_current_pose(self):
        if self.latest_robot_pose is None:
            self.get_logger().warn('Robot pose not available yet from /amcl_pose.')
            return False

        self.saved_patrol_pose = PoseStamped()
        self.saved_patrol_pose.header.frame_id = 'map'
        self.saved_patrol_pose.header.stamp = self.get_clock().now().to_msg()
        self.saved_patrol_pose.pose = self.latest_robot_pose
        return True

    def speak_text(self, text):
        self.get_logger().info(f'SPEAK: {text}')

        try:
            if shutil.which('spd-say') is not None:
                subprocess.run(['spd-say', '--wait', text], check=False)
                return

            if shutil.which('espeak') is not None:
                subprocess.run(['espeak', text], check=False)
                return

        except Exception as exc:
            self.get_logger().warn(f'Speech failed: {exc}')

    def marker_to_ring_color(self, marker: Marker):
        red = round(marker.color.r, 2)
        green = round(marker.color.g, 2)
        blue = round(marker.color.b, 2)

        color_options = {
            (1.0, 0.0, 0.0): 'red',
            (0.0, 1.0, 0.0): 'green',
            (0.0, 0.4, 1.0): 'blue',
            (1.0, 1.0, 0.0): 'yellow',
            (1.0, 0.5, 0.0): 'orange',
            (0.1, 0.1, 0.1): 'black',
        }

        best_color = 'unknown'
        smallest_difference = float('inf')

        for (ref_r, ref_g, ref_b), color_name in color_options.items():
            difference = abs(red - ref_r) + abs(green - ref_g) + abs(blue - ref_b)
            if difference < smallest_difference:
                smallest_difference = difference
                best_color = color_name

        return best_color

    def random_face_line(self):
        return random.choice(self.face_lines)

    def random_ring_line(self, color):
        template = random.choice(self.ring_lines)
        return template.format(color=color)

    def start_interaction(self):
        self.interaction_active = True
        self.interaction_end_time = self.get_clock().now().nanoseconds + int(self.interaction_duration * 1e9)

    def finish_interaction(self):
        self.interaction_active = False
        self.interaction_end_time = None
        self.target_done_callback(Empty())

    def main_loop(self):
        # Process any pending detections that were queued before AMCL was ready
        if self.amcl_pose_ready and self.pending_targets and self.active_target is None:
            target = self.pending_targets.pop(0)
            
            # Double-check that this target hasn't already been handled since it was queued
            if self.is_already_handled(target['type'], target['x'], target['y']):
                self.get_logger().info(f"Queued {target['type']} detection already handled, skipping.")
            else:
                self.get_logger().info(f'Processing queued {target["type"]} detection (x={target["x"]:.2f}, y={target["y"]:.2f})')
                if target['color'] is not None:
                    self.activate_target(target['type'], target['x'], target['y'], target['z'], target['color'])
                else:
                    self.activate_target(target['type'], target['x'], target['y'], target['z'])

        if not self.nav_server_ready:
            if self.nav_client.wait_for_server(timeout_sec=0.01):
                self.nav_server_ready = True
                self.get_logger().info('Behavior manager connected to navigate_to_pose.')
            return

        if self.interaction_active and self.interaction_end_time is not None:
            if self.get_clock().now().nanoseconds >= self.interaction_end_time:
                if self.current_state == 'INTERACT_FACE':
                    self.speak_text(self.random_face_line())

                elif self.current_state == 'INTERACT_RING':
                    if self.active_target is not None and 'color' in self.active_target:
                        self.speak_text(self.random_ring_line(self.active_target['color']))
                    else:
                        self.speak_text('I found a ring. Quite mysterious.')

                self.finish_interaction()
            return

        if self.result_future is None:
            return

        if not self.result_future.done():
            return

        try:
            result = self.result_future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(f'Navigation result error: {exc}')
            status = None

        nav_goal_type = self.nav_goal_type

        self.goal_active = False
        self.goal_handle = None
        self.result_future = None
        self.nav_goal_type = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            if nav_goal_type == 'approach_face':
                self.publish_state('INTERACT_FACE')
                self.get_logger().info('Reached face target. Starting interaction.')
                self.start_interaction()

            elif nav_goal_type == 'approach_ring':
                self.publish_state('INTERACT_RING')
                self.get_logger().info('Reached ring target. Starting interaction.')
                self.start_interaction()

            elif nav_goal_type == 'return_to_patrol':
                self.get_logger().info('Returned to saved patrol pose.')
                self.active_target = None
                self.saved_patrol_pose = None
                self.refresh_state()

            else:
                self.get_logger().info('Temporary navigation goal succeeded.')

        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ):
            self.get_logger().info(f'Temporary navigation cancelled: {nav_goal_type}')
            self.refresh_state()

        else:
            self.get_logger().warn(f'Temporary navigation failed: {nav_goal_type}, status={status}')
            self.refresh_state()

    def is_already_handled(self, target_type, x, y):
        for target in self.handled_targets:
            if target['type'] != target_type:
                continue
            if self.distance(x, y, target['x'], target['y']) <= self.target_match_threshold:
                return True
        return False

    def compute_approach_point(self, target_x, target_y):
        if self.latest_robot_pose is None:
            return None

        robot_x = self.latest_robot_pose.position.x
        robot_y = self.latest_robot_pose.position.y

        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.05:
            return None

        offset = 0.6
        if dist <= offset:
            goal_x = robot_x
            goal_y = robot_y
        else:
            factor = (dist - offset) / dist
            goal_x = robot_x + dx * factor
            goal_y = robot_y + dy * factor

        yaw = math.atan2(dy, dx)
        return goal_x, goal_y, yaw

    def send_nav_goal(self, x, y, yaw, nav_goal_type):
        if not self.nav_server_ready:
            self.get_logger().warn('navigate_to_pose server is not ready yet.')
            return False

        if self.goal_active or self.waiting_for_goal_accept or self.result_future is not None:
            self.get_logger().warn('A temporary navigation goal is already active.')
            return False

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation = yaw_to_quaternion(float(yaw))

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_goal_type = nav_goal_type
        self.waiting_for_goal_accept = True

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(
            f'Sending temporary goal [{nav_goal_type}] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )
        return True

    def goal_response_callback(self, future):
        self.waiting_for_goal_accept = False

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Error while sending temporary goal: {exc}')
            self.nav_goal_type = None
            self.refresh_state()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f'Temporary goal rejected: {self.nav_goal_type}')
            self.nav_goal_type = None
            self.refresh_state()
            return

        self.goal_handle = goal_handle
        self.goal_active = True
        self.result_future = goal_handle.get_result_async()

        self.get_logger().info(f'Temporary goal accepted: {self.nav_goal_type}')

    def cancel_temporary_goal(self):
        if self.goal_handle is not None and self.goal_active:
            self.get_logger().info('Cancelling temporary navigation goal...')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Cancel temporary goal failed: {exc}')

        self.goal_active = False
        self.waiting_for_goal_accept = False
        self.goal_handle = None
        self.result_future = None
        self.nav_goal_type = None

    def refresh_state(self, force_publish=False):
        if self.manual_control_active:
            self.publish_patrol_enabled(False)
            self.publish_state('MANUAL_CONTROL', force_publish)
            return

        if self.active_target is not None:
            if self.nav_goal_type == 'return_to_patrol' or self.current_state == 'RETURN_TO_PATROL':
                self.publish_patrol_enabled(False)
                self.publish_state('RETURN_TO_PATROL', force_publish)
                return

            if self.nav_goal_type == 'approach_face' or self.current_state == 'INTERACT_FACE':
                self.publish_patrol_enabled(False)
                if self.current_state == 'INTERACT_FACE':
                    self.publish_state('INTERACT_FACE', force_publish)
                else:
                    self.publish_state('APPROACH_FACE', force_publish)
                return

            if self.nav_goal_type == 'approach_ring' or self.current_state == 'INTERACT_RING':
                self.publish_patrol_enabled(False)
                if self.current_state == 'INTERACT_RING':
                    self.publish_state('INTERACT_RING', force_publish)
                else:
                    self.publish_state('APPROACH_RING', force_publish)
                return

            if self.active_target['type'] == 'face':
                self.publish_patrol_enabled(False)
                self.publish_state('APPROACH_FACE', force_publish)
                return

            if self.active_target['type'] == 'ring':
                self.publish_patrol_enabled(False)
                self.publish_state('APPROACH_RING', force_publish)
                return

        if self.patrol_requested and not self.patrol_finished:
            self.publish_patrol_enabled(True)
            self.publish_state('PATROL', force_publish)
            return

        self.publish_patrol_enabled(False)
        self.publish_state('IDLE', force_publish)

    def activate_target(self, target_type, x, y, z, color=None):
        if self.active_target is not None:
            return

        if self.latest_robot_pose is None:
            self.get_logger().warn('Ignoring target because /amcl_pose is not available yet.')
            return

        if not self.save_current_pose():
            return

        approach = self.compute_approach_point(x, y)
        if approach is None:
            self.get_logger().warn('Could not compute an approach goal.')
            return

        self.active_target = {
            'type': target_type,
            'x': x,
            'y': y,
            'z': z,
        }

        if color is not None:
            self.active_target['color'] = color

        self.get_logger().info(
            f'New target selected -> type={target_type}, x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

        self.publish_patrol_enabled(False)

        goal_x, goal_y, goal_yaw = approach
        nav_goal_type = 'approach_face' if target_type == 'face' else 'approach_ring'

        if not self.send_nav_goal(goal_x, goal_y, goal_yaw, nav_goal_type):
            self.active_target = None
            self.saved_patrol_pose = None
            self.refresh_state()
            return

        self.refresh_state()

    def face_callback(self, msg: Marker):
        if self.current_state not in ('PATROL', 'IDLE'):
            return

        if self.active_target is not None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self.is_already_handled('face', x, y):
            return

        # Check if already in queue at same position (deduplication)
        if any(abs(t['x'] - x) < 0.3 and abs(t['y'] - y) < 0.3 and t['type'] == 'face' for t in self.pending_targets):
            return

        # Queue the detection; it will be processed in main_loop when AMCL is ready
        self.pending_targets.append({'type': 'face', 'x': x, 'y': y, 'z': z, 'color': None})

    def ring_callback(self, msg: Marker):
        if self.current_state not in ('PATROL', 'IDLE'):
            return

        if self.active_target is not None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self.is_already_handled('ring', x, y):
            return

        color = self.marker_to_ring_color(msg)
        
        # Check if already in queue at same position (deduplication)
        if any(abs(t['x'] - x) < 0.3 and abs(t['y'] - y) < 0.3 and t['type'] == 'ring' for t in self.pending_targets):
            return
        
        # Queue the detection; it will be processed in main_loop when AMCL is ready
        self.pending_targets.append({'type': 'ring', 'x': x, 'y': y, 'z': z, 'color': color})

    def target_done_callback(self, msg: Empty):
        if self.active_target is None:
            self.get_logger().info('No active target to mark as done.')
            return

        if self.current_state not in ('INTERACT_FACE', 'INTERACT_RING', 'APPROACH_FACE', 'APPROACH_RING'):
            self.get_logger().info('Target done received, but current state is not target handling.')
            return

        self.handled_targets.append(self.active_target)

        self.get_logger().info(
            f"Target completed -> type={self.active_target['type']}, "
            f"x={self.active_target['x']:.2f}, y={self.active_target['y']:.2f}"
        )

        if self.saved_patrol_pose is None:
            self.get_logger().warn('No saved patrol pose. Clearing target without return.')
            self.active_target = None
            self.refresh_state()
            return

        x = self.saved_patrol_pose.pose.position.x
        y = self.saved_patrol_pose.pose.position.y
        yaw = 2.0 * math.atan2(
            self.saved_patrol_pose.pose.orientation.z,
            self.saved_patrol_pose.pose.orientation.w
        )

        if self.goal_active:
            self.cancel_temporary_goal()

        if self.send_nav_goal(x, y, yaw, 'return_to_patrol'):
            self.publish_state('RETURN_TO_PATROL')
        else:
            self.get_logger().warn('Could not start return-to-patrol navigation.')
            self.active_target = None
            self.saved_patrol_pose = None
            self.refresh_state()

    def resume_patrol_callback(self, msg: Empty):
        if self.active_target is not None:
            self.get_logger().info('Clearing active target and resuming patrol.')
        else:
            self.get_logger().info('No active target. Refreshing state.')

        if self.goal_active:
            self.cancel_temporary_goal()

        self.interaction_active = False
        self.interaction_end_time = None
        self.active_target = None
        self.saved_patrol_pose = None
        self.refresh_state()

    def manual_control_callback(self, msg: Bool):
        self.manual_control_active = msg.data

        if self.manual_control_active and self.goal_active:
            self.cancel_temporary_goal()

        if self.manual_control_active:
            self.interaction_active = False
            self.interaction_end_time = None

        self.refresh_state()

    def patrol_command_callback(self, msg: Bool):
        self.patrol_requested = msg.data

        if self.patrol_requested:
            self.patrol_finished = False

        if not self.patrol_requested and self.active_target is None and not self.manual_control_active:
            self.publish_patrol_enabled(False)

        self.refresh_state()

    def patrol_finished_callback(self, msg: Bool):
        if not msg.data:
            return

        self.patrol_finished = True
        self.patrol_requested = False
        self.refresh_state()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()