#!/usr/bin/env python3

import math
import json
import os
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
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2


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
        self.barrel_match_threshold = 0.65
        self.pending_match_threshold = {
            'face': 0.35,
            'ring': 0.35,
            'barrel': 0.65,
        }
        self.approach_offset = 0.5

        # Queue for pending detections (captured before AMCL is ready)
        self.pending_targets = []  # List of {type, x, y, z, color}
        self.amcl_pose_ready = False

        self.latest_robot_pose = None

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

        self.waiting_for_spill_check = False
        self.spill_check_start_time = None
        self.spill_check_timeout = 10.0  # seconds before giving up waiting

        self.bridge = CvBridge()
        self._latest_oakd = None
        self._pending_leak_capture = None
        self._leak_image_seq = 0
        self._oakd_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self._oakd_cb, 10
        )

        # Approach timeout: auto-enter interact if not reached in 15 seconds
        self.approach_start_time = None
        self.approach_timeout = 15.0  # seconds

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

        self.barrel_lines = [
            "I found a {color} {orientation} barrel. {leak_sentence}",
            "Barrel inspection complete. The barrel is {color} and {orientation}. {leak_sentence}",
            "This is a {color} {orientation} barrel. {leak_sentence}",
            "A {color} barrel has been detected. It is {orientation}. {leak_sentence}",
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
            10
        )

        self.patrol_command_subscriber = self.create_subscription(
            Bool,
            '/patrol_command',
            self.patrol_command_callback,
            10
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

        self.cylinder_subscriber = self.create_subscription(
            Marker,
            '/detected_cylinder_locations',
            self.cylinder_callback,
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

        self.spill_check_client = self.create_client(Trigger, '/spill_check')
        self._pending_spill_future = None
        self._barrel_result_pub = self.create_publisher(String, '/barrel_inspection_result', 10)

        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.patrol_group_end_sub = self.create_subscription(
            Empty,
            '/patrol_group_end',
            self.patrol_group_end_callback,
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

    def speak_text(self, text):
        self.get_logger().info(f'SPEAK: {text}')

        commands = []
        if shutil.which('espeak-ng') is not None:
            commands.append(['espeak-ng', '-a', '180', '-s', '145', text])
        if shutil.which('espeak') is not None:
            commands.append(['espeak', '-a', '180', '-s', '145', text])
        if shutil.which('spd-say') is not None:
            commands.append(['spd-say', text])

        for command in commands:
            try:
                subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                return
            except Exception as exc:
                self.get_logger().warn(f'Speech command failed ({command[0]}): {exc}')

        self.get_logger().warn('No speech command found. Install espeak-ng or espeak.')

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
            (0.6, 0.0, 1.0): 'purple',
            (0.45, 0.22, 0.08): 'brown',
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

    def _oakd_cb(self, msg):
        try:
            self._latest_oakd = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _save_leak_image(self):
        if self._latest_oakd is None:
            self.get_logger().warn("No OAK-D image available for leak capture")
            return False
        leak_dir = os.path.expanduser("~/RINS_Project/reports/img/barrels")
        os.makedirs(leak_dir, exist_ok=True)
        filename = f"leak_{self._leak_image_seq:04d}.jpg"
        self._leak_image_seq += 1
        path = os.path.join(leak_dir, filename)
        cv2.imwrite(path, self._latest_oakd)
        self.get_logger().info(f"Saved leak image: {path}")
        return True

    def random_barrel_line(self, color, orientation, leak_detected):
        template = random.choice(self.barrel_lines)
        if leak_detected is True:
            leak_sentence = "Warning. It is leaking."
        elif leak_detected is False:
            leak_sentence = "No leak detected."
        else:
            leak_sentence = "Leak inspection was inconclusive."
        return template.format(
            color=color,
            orientation=orientation,
            leak_sentence=leak_sentence,
        )

    def start_interaction(self):
        self.interaction_active = True
        self.interaction_end_time = self.get_clock().now().nanoseconds + int(self.interaction_duration * 1e9)
        text = self.interaction_text()
        if text:
            self.speak_text(text)

    def interaction_text(self):
        if self.current_state == 'INTERACT_FACE':
            return self.random_face_line()

        if self.current_state == 'INTERACT_RING':
            if self.active_target is not None and 'color' in self.active_target:
                return self.random_ring_line(self.active_target['color'])
            return 'I found a ring. Quite mysterious.'

        if self.current_state == 'INTERACT_BARREL':
            if self.active_target is None:
                return 'I found a barrel. Inspection complete.'
            return self.random_barrel_line(
                self.active_target.get('color', 'unknown'),
                self.active_target.get('orientation', 'unknown'),
                self.active_target.get('leak_detected'),
            )

        return None

    def finish_interaction(self):
        self.interaction_active = False
        self.interaction_end_time = None
        self.waiting_for_spill_check = False
        self.spill_check_start_time = None
        self.target_done_callback(Empty())



    def main_loop(self):
        if not self.nav_server_ready:
            if self.nav_client.wait_for_server(timeout_sec=0.01):
                self.nav_server_ready = True
                self.get_logger().info('Behavior manager connected to navigate_to_pose.')
            return

        # Process pending detections only after navigation is ready. Confirmed markers
        # may be one-shot publications, so never pop them before they can become goals.
        if self.amcl_pose_ready and self.pending_targets and self.active_target is None:
            # Scan for the first ready target. Barrels marked wait_for_group_end are
            # held until the current waypoint rotation group finishes.
            for i, target in enumerate(self.pending_targets):
                if self.is_already_handled(target['type'], target['x'], target['y'], target.get('color')):
                    self.pending_targets.pop(i)
                    self.get_logger().info(f"Queued {target['type']} detection already handled, skipping.")
                    break
                if target.get('wait_for_group_end', False):
                    continue  # deferred — skip until group_end fires
                self.get_logger().info(
                    f'Processing queued {target["type"]} detection (x={target["x"]:.2f}, y={target["y"]:.2f})')
                metadata = {
                    key: value for key, value in target.items()
                    if key not in ('type', 'x', 'y', 'z', 'color', 'wait_for_group_end')
                }
                if self.activate_target(
                    target['type'], target['x'], target['y'], target['z'],
                    target.get('color'), **metadata
                ):
                    self.pending_targets.pop(i)
                break

        if self.interaction_active and self.interaction_end_time is not None:
            if self.get_clock().now().nanoseconds >= self.interaction_end_time:
                self.finish_interaction()
            return

        # While waiting for the spill check to finish, hold here and check for timeout
        if self.waiting_for_spill_check:
            if self._pending_spill_future is not None and self._pending_spill_future.done():
                try:
                    response = self._pending_spill_future.result()
                    if response.success:
                        data = json.loads(response.message)
                        self.active_target['leak_detected'] = data.get('spill_detected')
                        self.active_target['spill_point_count'] = data.get('point_count', 0)
                        self.get_logger().info(
                            f"Spill check: {data.get('point_count', 0)} pts → "
                            f"{'SPILL' if data.get('spill_detected') else 'OK'}"
                        )
                    else:
                        err = json.loads(response.message).get('error', 'unknown')
                        self.get_logger().warn(f'Spill check failed: {err}')
                        if self.active_target is not None:
                            self.active_target['leak_detected'] = None
                except Exception as e:
                    self.get_logger().warn(f'Spill check error: {e}')
                    if self.active_target is not None:
                        self.active_target['leak_detected'] = None
                self.waiting_for_spill_check = False
                self.spill_check_start_time = None
                self._pending_spill_future = None
                if self.active_target is not None:
                    result = String()
                    result.data = json.dumps({
                        "barrel_id": self.active_target.get("barrel_id"),
                        "leak_detected": self.active_target.get("leak_detected"),
                    })
                    self._barrel_result_pub.publish(result)
                if (self.active_target is not None
                        and self.active_target.get('leak_detected')
                        and self._latest_oakd is not None):
                    self.get_logger().info("Leak detected — capturing image immediately")
                    self._save_leak_image()
                    self._pending_leak_capture = self.get_clock().now().nanoseconds
                else:
                    self.start_interaction()
            elif self.spill_check_start_time is not None:
                elapsed = (self.get_clock().now().nanoseconds - self.spill_check_start_time) / 1e9
                if elapsed >= self.spill_check_timeout:
                    self.get_logger().warn('Spill check timed out — speaking inconclusive result.')
                    self.waiting_for_spill_check = False
                    self.spill_check_start_time = None
                    self._pending_spill_future = None
                    if self.active_target is not None:
                        self.active_target['leak_detected'] = None
                    self.start_interaction()
            return

        # Leak capture: stay still for 1 second after image, then interact
        if self._pending_leak_capture is not None:
            elapsed = (self.get_clock().now().nanoseconds - self._pending_leak_capture) / 1e9
            if elapsed >= 1.0:
                self._pending_leak_capture = None
                self.start_interaction()
            return

        # Approach timeout: auto-transition to interact if 15 seconds elapsed without reaching target
        # This is a FAILSAFE that cancels the stuck nav goal and forces interaction to start
        if self.approach_start_time is not None and self.active_target is not None:
            if self.current_state in ('APPROACH_FACE', 'APPROACH_RING', 'APPROACH_BARREL') and self.nav_goal_type in ('approach_face', 'approach_ring', 'approach_barrel'):
                elapsed = (self.get_clock().now().nanoseconds - self.approach_start_time) / 1e9
                if elapsed >= self.approach_timeout:
                    self.get_logger().warn(
                        f'Approach timeout ({self.approach_timeout}s) reached. Cancelling stuck goal and entering interact.')
                    self.approach_start_time = None
                    # Cancel the stuck navigation goal
                    self.cancel_temporary_goal()
                    # Gracefully transition to interact - use same interaction flow as normal
                    if self.current_state == 'APPROACH_FACE':
                        self.publish_state('INTERACT_FACE')
                    elif self.current_state == 'APPROACH_BARREL':
                        self.publish_state('INTERACT_BARREL')
                    else:  # APPROACH_RING
                        self.publish_state('INTERACT_RING')
                    if self.current_state == 'INTERACT_BARREL' and self.active_target is not None and self.active_target.get('is_horizontal', False):
                        if self.spill_check_client.service_is_ready():
                            self.get_logger().info('Horizontal barrel (timeout path) — calling spill check service.')
                            self.waiting_for_spill_check = True
                            self.spill_check_start_time = self.get_clock().now().nanoseconds
                            req = Trigger.Request()
                            self._pending_spill_future = self.spill_check_client.call_async(req)
                        else:
                            self.get_logger().warn('Spill check service not available (timeout path) — marking inconclusive.')
                            if self.active_target is not None:
                                self.active_target['leak_detected'] = None
                            self.start_interaction()
                    else:
                        self.start_interaction()
                    # Do NOT return - let normal interaction logic proceed

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
        self.approach_start_time = None  # Clear approach timeout

        if status == GoalStatus.STATUS_SUCCEEDED:
            if nav_goal_type == 'approach_face':
                self.publish_state('INTERACT_FACE')
                self.get_logger().info('Reached face target. Starting interaction.')
                self.start_interaction()

            elif nav_goal_type == 'approach_ring':
                self.publish_state('INTERACT_RING')
                self.get_logger().info('Reached ring target. Starting interaction.')
                self.start_interaction()

            elif nav_goal_type == 'approach_barrel':
                self.publish_state('INTERACT_BARREL')
                self.get_logger().info('Reached barrel target. Starting interaction.')
                if self.active_target is not None and self.active_target.get('is_horizontal', False):
                    if self.spill_check_client.service_is_ready():
                        self.get_logger().info('Horizontal barrel — calling spill check service.')
                        self.waiting_for_spill_check = True
                        self.spill_check_start_time = self.get_clock().now().nanoseconds
                        req = Trigger.Request()
                        self._pending_spill_future = self.spill_check_client.call_async(req)
                    else:
                        self.get_logger().warn('Spill check service not available — skipping, marking inconclusive.')
                        if self.active_target is not None:
                            self.active_target['leak_detected'] = None
                        self.start_interaction()
                else:
                    if self.active_target is not None:
                        result = String()
                        result.data = json.dumps({
                            "barrel_id": self.active_target.get("barrel_id"),
                            "leak_detected": self.active_target.get("leak_detected"),
                        })
                        self._barrel_result_pub.publish(result)
                    self.start_interaction()

            else:
                self.get_logger().info('Temporary navigation goal succeeded.')

        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ):
            self.get_logger().info(f'Temporary navigation cancelled: {nav_goal_type}')
            self.refresh_state()

        else:
            # Face approach: retry at progressively farther distances when the planned
            # path is blocked (e.g. by the inflation zone near a wall).
            if nav_goal_type == 'approach_face' and self.active_target is not None:
                attempt = self.active_target.get('face_approach_attempt', 0) + 1
                self.active_target['face_approach_attempt'] = attempt
                face_yaw = self.active_target.get('face_yaw')
                retry_dists = [0.6, 1.0]  # fallback distances after the initial 0.3 m
                if face_yaw is not None and attempt <= len(retry_dists):
                    dist = retry_dists[attempt - 1]
                    fx = self.active_target['x']
                    fy = self.active_target['y']
                    gx = fx + dist * math.cos(face_yaw)
                    gy = fy + dist * math.sin(face_yaw)
                    ry = math.atan2(fy - gy, fx - gx)
                    self.get_logger().info(
                        f'Face approach attempt {attempt} failed — retrying at {dist:.1f} m.')
                    if self.send_nav_goal(gx, gy, ry, 'approach_face'):
                        self.approach_start_time = self.get_clock().now().nanoseconds
                        self.refresh_state()
                        return
            self.get_logger().warn(f'Temporary navigation failed: {nav_goal_type}, status={status}')
            self.refresh_state()

    def colors_compatible(self, color_a, color_b):
        if color_a in (None, 'unknown') or color_b in (None, 'unknown'):
            return True
        return color_a == color_b

    def is_barrel_duplicate(self, existing, x, y, color):
        distance = self.distance(x, y, existing['x'], existing['y'])
        if distance <= 0.18:
            return True
        return (
            distance <= self.barrel_match_threshold
            and self.colors_compatible(existing.get('color'), color)
        )

    def is_already_handled(self, target_type, x, y, color=None):
        threshold = self.match_threshold_for_type(target_type)
        for target in self.handled_targets:
            if target['type'] != target_type:
                continue
            if target_type in ('barrel', 'cylinder'):
                if self.is_barrel_duplicate(target, x, y, color):
                    return True
                continue
            if self.distance(x, y, target['x'], target['y']) <= threshold:
                return True
        return False

    def match_threshold_for_type(self, target_type):
        if target_type in ('barrel', 'cylinder'):
            return self.barrel_match_threshold
        return self.target_match_threshold

    def pending_threshold_for_type(self, target_type):
        return self.pending_match_threshold.get(target_type, 0.35)

    def accepts_new_detections(self):
        return self.current_state == 'PATROL'

    def accepts_face_detections(self):
        # No face detection during barrel approach/interact
        if self.current_state in ('APPROACH_BARREL', 'INTERACT_BARREL'):
            return False
        return self.current_state in ('PATROL', 'APPROACH_FACE', 'INTERACT_FACE')

    def is_active_target_match(self, target_type, x, y, color=None):
        if self.active_target is None:
            return False
        if self.active_target.get('type') != target_type:
            return False
        if target_type in ('barrel', 'cylinder'):
            return self.is_barrel_duplicate(self.active_target, x, y, color)
        threshold = self.match_threshold_for_type(target_type)
        return self.distance(x, y, self.active_target['x'], self.active_target['y']) <= threshold

    def is_pending_target_match(self, target_type, x, y, color=None):
        threshold = self.pending_threshold_for_type(target_type)
        for target in self.pending_targets:
            if target.get('type') != target_type:
                continue
            if target_type in ('barrel', 'cylinder'):
                if self.is_barrel_duplicate(target, x, y, color):
                    return True
                continue
            if self.distance(x, y, target['x'], target['y']) <= threshold:
                return True
        return False

    def queue_target(self, target):
        target_type = target['type']
        x = target['x']
        y = target['y']
        color = target.get('color')

        if self.is_already_handled(target_type, x, y, color):
            return False
        if self.is_active_target_match(target_type, x, y, color):
            return False
        if self.is_pending_target_match(target_type, x, y, color):
            return False

        self.pending_targets.append(target)
        self.get_logger().info(
            f"Queued {target_type} target at x={x:.2f}, y={y:.2f} "
            f"(queue={len(self.pending_targets)})"
        )
        return True

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

        offset = self.approach_offset
        if dist <= offset:
            goal_x = robot_x
            goal_y = robot_y
        else:
            factor = (dist - offset) / dist
            goal_x = robot_x + dx * factor
            goal_y = robot_y + dy * factor

        yaw = math.atan2(dy, dx)
        return goal_x, goal_y, yaw

    def compute_barrel_approach_point(self, target_x, target_y):
        """Approach a horizontal barrel from slightly to the right so the depth
        camera has a clear view of any spill on the floor beside it."""
        result = self.compute_approach_point(target_x, target_y)
        if result is None:
            return None
        goal_x, goal_y, goal_yaw = result

        robot_x = self.latest_robot_pose.position.x
        robot_y = self.latest_robot_pose.position.y
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.05:
            return result

        # Right-perpendicular of the approach direction (clockwise 90°)
        lateral = 0.3  # metres to the right
        right_x = dy / dist
        right_y = -dx / dist
        goal_x += lateral * right_x
        goal_y += lateral * right_y
        # Keep yaw pointing toward the barrel
        goal_yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        return goal_x, goal_y, goal_yaw

    def compute_face_approach_point(self, face_x, face_y, face_yaw):
        # Stand directly in front of the face (along its viewing direction) at 0.3 m
        dist = 0.3
        goal_x = face_x + dist * math.cos(face_yaw)
        goal_y = face_y + dist * math.sin(face_yaw)
        # Robot looks back toward the face
        robot_yaw = math.atan2(face_y - goal_y, face_x - goal_x)
        return goal_x, goal_y, robot_yaw

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
        self.approach_start_time = None  # Reset timer when goal is cancelled

    def refresh_state(self, force_publish=False):
        if self.manual_control_active and self.active_target is None:
            self.publish_patrol_enabled(False)
            self.publish_state('MANUAL_CONTROL', force_publish)
            return

        if self.active_target is not None:
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

            if self.nav_goal_type == 'approach_barrel' or self.current_state == 'INTERACT_BARREL':
                self.publish_patrol_enabled(False)
                if self.current_state == 'INTERACT_BARREL':
                    self.publish_state('INTERACT_BARREL', force_publish)
                else:
                    self.publish_state('APPROACH_BARREL', force_publish)
                return

            if self.active_target['type'] == 'face':
                self.publish_patrol_enabled(False)
                self.publish_state('APPROACH_FACE', force_publish)
                return

            if self.active_target['type'] == 'ring':
                self.publish_patrol_enabled(False)
                self.publish_state('APPROACH_RING', force_publish)
                return

            if self.active_target['type'] == 'barrel':
                self.publish_patrol_enabled(False)
                self.publish_state('APPROACH_BARREL', force_publish)
                return

        if self.patrol_requested and not self.patrol_finished:
            self.publish_patrol_enabled(True)
            self.publish_state('PATROL', force_publish)
            return

        self.publish_patrol_enabled(False)
        self.publish_state('IDLE', force_publish)

    def activate_target(self, target_type, x, y, z, color=None, **metadata):
        if self.active_target is not None:
            return False

        if self.latest_robot_pose is None:
            self.get_logger().warn('Ignoring target because /amcl_pose is not available yet.')
            return False

        face_yaw = metadata.get('face_yaw') if target_type == 'face' else None
        if face_yaw is not None:
            approach = self.compute_face_approach_point(x, y, face_yaw)
        elif target_type in ('barrel', 'cylinder') and metadata.get('is_horizontal', False):
            approach = self.compute_barrel_approach_point(x, y)
        else:
            approach = self.compute_approach_point(x, y)
        if approach is None:
            self.get_logger().warn('Could not compute an approach goal.')
            return False

        self.active_target = {
            'type': target_type,
            'x': x,
            'y': y,
            'z': z,
        }
        self.active_target.update(metadata)

        if color is not None:
            self.active_target['color'] = color

        self.get_logger().info(
            f'New target selected -> type={target_type}, x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

        self.publish_patrol_enabled(False)

        goal_x, goal_y, goal_yaw = approach
        nav_goal_type = {
            'face': 'approach_face',
            'ring': 'approach_ring',
            'barrel': 'approach_barrel',
            'cylinder': 'approach_barrel',
        }.get(target_type, 'approach_ring')

        if not self.send_nav_goal(goal_x, goal_y, goal_yaw, nav_goal_type):
            self.active_target = None
            self.refresh_state()
            return False

        # Start approach timeout: auto-transition to interact after 15 seconds (fresh timer for this approach)
        self.approach_start_time = self.get_clock().now().nanoseconds

        self.refresh_state()
        return True

    def face_callback(self, msg: Marker):
        if not self.accepts_face_detections():
            return

        if msg.ns and msg.ns != 'face_confirmed':
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract face-toward-robot yaw encoded in marker orientation (set by face_localizator)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        face_yaw = 2.0 * math.atan2(qz, qw)

        self.queue_target({'type': 'face', 'x': x, 'y': y, 'z': z, 'color': None, 'face_yaw': face_yaw,
                           'wait_for_group_end': True})

    def ring_callback(self, msg: Marker):
        # Rings no longer require approach or interaction — detection only
        pass

    def cylinder_callback(self, msg: Marker):
        if not self.accepts_new_detections():
            return

        if msg.ns and msg.ns not in ('barrel_confirmed', 'cylinder_confirmed'):
            return

        is_horizontal = (msg.text == 'horizontal')
        if not is_horizontal:
            # Vertical (standing) barrels need no approach — detection only
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        color = self.marker_to_ring_color(msg)
        orientation = 'horizontal'

        self.queue_target({
            'type': 'barrel',
            'x': x,
            'y': y,
            'z': z,
            'color': color,
            'is_horizontal': is_horizontal,
            'orientation': orientation,
            'barrel_id': msg.id,
            'leak_detected': False if not is_horizontal else None,
            'wait_for_group_end': True,  # hold until current waypoint rotations finish
        })

    def target_done_callback(self, msg: Empty):
        if self.active_target is None:
            self.get_logger().info('No active target to mark as done.')
            return

        if self.current_state not in ('INTERACT_FACE', 'INTERACT_RING', 'INTERACT_BARREL', 'APPROACH_FACE', 'APPROACH_RING', 'APPROACH_BARREL'):
            self.get_logger().info('Target done received, but current state is not target handling.')
            return

        self.handled_targets.append(self.active_target)

        self.get_logger().info(
            f"Target completed -> type={self.active_target['type']}, "
            f"x={self.active_target['x']:.2f}, y={self.active_target['y']:.2f}"
        )

        if self.goal_active:
            self.cancel_temporary_goal()

        self.active_target = None
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
        self.refresh_state()

    def manual_control_callback(self, msg: Bool):
        self.manual_control_active = msg.data

        if self.manual_control_active and self.goal_active and self.nav_goal_type != 'approach_barrel':
            self.cancel_temporary_goal()

        if self.manual_control_active:
            self.interaction_active = False
            self.interaction_end_time = None
            self.waiting_for_spill_check = False
            self.spill_check_start_time = None
            # Clear active target if mid-interaction so refresh_state() can
            # transition to MANUAL_CONTROL/IDLE instead of looping on INTERACT_*
            if self.active_target is not None and self.current_state in (
                'INTERACT_FACE', 'INTERACT_RING', 'INTERACT_BARREL',
            ):
                self.handled_targets.append(self.active_target)
                self.get_logger().info(
                    f"Manual control interrupted {self.active_target['type']} interaction — marking as done."
                )
                self.active_target = None

        self.refresh_state()

    def patrol_group_end_callback(self, msg: Empty):
        """Fired by waypoint_navigator when the robot finishes all rotations at one
        (x, y) position and is about to move to a new waypoint group."""
        deferred = [t for t in self.pending_targets if t.get('wait_for_group_end')]
        if not deferred:
            return
        for t in deferred:
            t.pop('wait_for_group_end', None)
        # Pause patrol immediately so the navigator doesn't start moving to the next
        # waypoint before the barrel interaction is activated on the next main_loop tick.
        if self.patrol_requested and not self.patrol_finished and self.active_target is None:
            self.publish_patrol_enabled(False)
            self.get_logger().info(
                f'Patrol group ended — {len(deferred)} barrel(s) now ready, pausing patrol.')

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
