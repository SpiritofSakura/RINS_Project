#!/usr/bin/env python3

import math
import json
import random
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String, Empty
from visualization_msgs.msg import Marker, MarkerArray


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
        self.approach_offset = 0.4
        self.face_approach_offset = 0.35

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
        self.barrel_interaction_duration = 10.0


        # Approach timeout: auto-enter interact if not reached in 15 seconds
        self.approach_start_time = None
        self.approach_timeout = 15.0  # seconds
        self.barrel_approach_timeout = 60.0  # seconds
        self.skip_barrel_after_cancel = False

        self.face_qr_turn_active = False
        self.face_qr_turn_end = None
        self.face_qr_turn_target = None
        self.face_qr_turn_duration = 2.0
        self.face_qr_turn_speed = 0.45

        # Final waypoint handoff before blue-line following
        self.final_wp_spin_active = False
        self.final_wp_spin_end = 0.0
        self.final_nav_failures = 0
        self.blue_line_runtime_started = False

        # Post-patrol: workstation visit + final waypoint
        self.pending_workstation_color = None   # set by QR code
        self.workstation_positions = {}          # color → (x, y, yaw)
        self.workstation_done_flag = False
        self.inspector_finished_flag = False
        self.workstation_start_time = None
        self.workstation_timeout = 60.0          # seconds at workstation before giving up
        self.post_patrol_active = False
        self.need_post_patrol = False

        self.declare_parameter('final_wp_x', 2.755)
        self.declare_parameter('final_wp_y', 0.288)
        self.declare_parameter('final_wp_yaw', 0.0)

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
        self.blue_line_enabled_pub = self.create_publisher(Bool, '/blue_line_enabled', 10)

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

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.blue_line_pub = self.create_publisher(Bool, '/blue_line_enabled', 10)
        self.yellow_line_pub = self.create_publisher(Bool, '/yellow_line_enabled', 10)
        self.spill_check_trigger_pub = self.create_publisher(String, '/spill_check_trigger', 10)

        self.qr_sub = self.create_subscription(String, '/qr', self.qr_callback, 10)
        self.workstation_markers_sub = self.create_subscription(
            MarkerArray, '/workstation_markers', self.workstation_markers_callback, 10)
        self.workstation_done_sub = self.create_subscription(
            Empty, '/workstation_done', self.workstation_done_callback, 10)
        self.inspector_finish_sub = self.create_subscription(
            String, '/inspector_finish', self.inspector_finish_callback, 10)

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

    def publish_yellow_line_enabled(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.yellow_line_pub.publish(msg)

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

    def start_interaction(self):
        duration = self.barrel_interaction_duration if self.current_state == 'INTERACT_BARREL' else self.interaction_duration
        self.interaction_active = True
        self.interaction_end_time = self.get_clock().now().nanoseconds + int(duration * 1e9)
        if self.current_state == 'INTERACT_BARREL':
            self.trigger_spill_check_for_active_barrel()
        else:
            text = self.interaction_text()
            if text:
                self.speak_text(text)

    def trigger_spill_check_for_active_barrel(self):
        if self.active_target is None:
            return
        if self.active_target.get('spill_check_triggered'):
            return
        if self.active_target.get('orientation') != 'horizontal':
            self.active_target['spill_check_triggered'] = True
            self.get_logger().info('Vertical barrel interaction — no spill check needed.')
            return
        barrel_id = self.active_target.get('barrel_id')
        if barrel_id is None:
            self.get_logger().warn('Reached barrel target, but active target has no barrel_id for spill check.')
            return

        msg = String()
        msg.data = f'check {int(barrel_id)}'
        self.spill_check_trigger_pub.publish(msg)
        self.active_target['spill_check_triggered'] = True
        self.get_logger().info(f'Triggered spill check for approached barrel id={barrel_id}.')

    def interaction_text(self):
        if self.current_state == 'INTERACT_FACE':
            return self.random_face_line()

        if self.current_state == 'INTERACT_RING':
            if self.active_target is not None and 'color' in self.active_target:
                return self.random_ring_line(self.active_target['color'])
            return 'I found a ring. Quite mysterious.'

        return None

    def finish_interaction(self):
        self.interaction_active = False
        self.interaction_end_time = None
        self.target_done_callback(Empty())



    def main_loop(self):
        if not self.nav_server_ready:
            if self.nav_client.wait_for_server(timeout_sec=0.01):
                self.nav_server_ready = True
                self.get_logger().info('Behavior manager connected to navigate_to_pose.')
            return

        # Start post-patrol sequence once the robot is free and all queued interactions done.
        if (self.need_post_patrol and self.active_target is None
                and not self.interaction_active and self.result_future is None
                and not self.pending_targets):
            self.need_post_patrol = False
            self._start_post_patrol_sequence()
            return

        # Workstation dwell: hold until the inspector explicitly reports finished.
        if self.current_state == 'WORKSTATION':
            if self.inspector_finished_flag:
                self.inspector_finished_flag = False
                self.workstation_done_flag = False
                self.workstation_start_time = None
                self.pending_workstation_color = None
                self._navigate_to_final_wp()
                return
            if (self.workstation_start_time is not None
                    and (self.get_clock().now().nanoseconds - self.workstation_start_time) / 1e9
                    >= self.workstation_timeout):
                self.get_logger().warn(
                    'Still waiting for /inspector_finish before sending final waypoint.')
                self.workstation_start_time = self.get_clock().now().nanoseconds
            return

        # After seeing a face during blue-line mode, keep rotating briefly so
        # the QR reader gets a wider view before the face approach starts.
        if self.face_qr_turn_active:
            self._update_face_qr_turn()
            return

        # Legacy final spin guard; kept harmless if an older path sets it.
        if self.final_wp_spin_active:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns < self.final_wp_spin_end:
                twist = Twist()
                twist.angular.z = 1.0
                self.cmd_vel_pub.publish(twist)
                return
            self._stop_direct_drive()
            self.final_wp_spin_active = False
            self._start_blue_line_runtime()
            return

        # Process pending detections only after navigation is ready.
        # Faces are highest priority; barrels are processed only when no face is pending.
        _TARGET_PRIORITY = {'face': 0, 'barrel': 1, 'cylinder': 1, 'ring': 2}
        if self.amcl_pose_ready and self.pending_targets and self.active_target is None:
            # Purge stale (already-handled) entries first.
            for i in range(len(self.pending_targets) - 1, -1, -1):
                t = self.pending_targets[i]
                if self.is_already_handled(t['type'], t['x'], t['y'], t.get('color')):
                    self.pending_targets.pop(i)
                    self.get_logger().info(
                        f"Queued {t['type']} detection already handled, skipping.")
            # Collect non-deferred targets and sort faces before barrels.
            ready = [
                (i, t) for i, t in enumerate(self.pending_targets)
                if (not t.get('wait_for_group_end', False)
                    and not (self.manual_control_active and t['type'] == 'face'))
            ]
            ready.sort(key=lambda it: _TARGET_PRIORITY.get(it[1]['type'], 2))
            if ready:
                i, target = ready[0]
                self.get_logger().info(
                    f'Processing queued {target["type"]} detection '
                    f'(x={target["x"]:.2f}, y={target["y"]:.2f})')
                metadata = {
                    key: value for key, value in target.items()
                    if key not in ('type', 'x', 'y', 'z', 'color', 'wait_for_group_end')
                }
                if self.activate_target(
                    target['type'], target['x'], target['y'], target['z'],
                    target.get('color'), **metadata
                ):
                    self.pending_targets.pop(i)

        if self.interaction_active and self.interaction_end_time is not None:
            if self.get_clock().now().nanoseconds >= self.interaction_end_time:
                self.finish_interaction()
            return

        # Approach timeout: auto-transition to interact for lightweight targets only.
        # Barrels must wait for the real Nav2 result so spill checks happen at the barrel.
        if self.approach_start_time is not None and self.active_target is not None:
            if self.current_state == 'APPROACH_BARREL' and self.nav_goal_type == 'approach_barrel':
                elapsed = (self.get_clock().now().nanoseconds - self.approach_start_time) / 1e9
                if elapsed >= self.barrel_approach_timeout:
                    self.get_logger().warn(
                        f'Barrel approach timeout ({self.barrel_approach_timeout}s) reached. '
                        'Skipping barrel interaction and resuming navigation.')
                    self._skip_active_barrel_after_timeout()
                    return

            if self.current_state in ('APPROACH_FACE', 'APPROACH_RING') and self.nav_goal_type in ('approach_face', 'approach_ring'):
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
                    else:  # APPROACH_RING
                        self.publish_state('INTERACT_RING')
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
                self.start_interaction()

            elif nav_goal_type == 'approach_workstation':
                self.publish_state('WORKSTATION')
                self.workstation_start_time = self.get_clock().now().nanoseconds
                self.get_logger().info('Arrived at workstation — waiting for done signal.')

            elif nav_goal_type == 'approach_final':
                self.final_nav_failures = 0
                self.get_logger().info('At final waypoint — enabling blue-line runtime.')
                self._start_blue_line_runtime()

            else:
                self.get_logger().info('Temporary navigation goal succeeded.')

        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ):
            self.get_logger().info(f'Temporary navigation cancelled: {nav_goal_type}')
            if nav_goal_type in ('approach_workstation', 'approach_final'):
                self._handle_post_patrol_nav_failure(nav_goal_type)
            else:
                self.refresh_state()

        else:
            if nav_goal_type in ('approach_workstation', 'approach_final'):
                self._handle_post_patrol_nav_failure(nav_goal_type)
                return
            # Face approach: retry at progressively farther distances when the planned
            # path is blocked (e.g. by the inflation zone near a wall).
            if nav_goal_type == 'approach_face' and self.active_target is not None:
                attempt = self.active_target.get('face_approach_attempt', 0) + 1
                self.active_target['face_approach_attempt'] = attempt
                face_yaw = self.active_target.get('face_yaw')
                retry_dists = [0.6, 1.0]  # fallback distances after the initial face approach
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
        if self.current_state in ('APPROACH_BARREL', 'INTERACT_BARREL'):
            return False
        if self.current_state in ('APPROACH_WORKSTATION', 'WORKSTATION', 'APPROACH_FINAL',
                                  'FINISHING_ROUNDS'):
            return False
        # Allow face detection in the room (line-following phase)
        if self.current_state in (
            'LINE_FOLLOWING', 'FOLLOW_BLUE_LINE', 'BLUE_LINE_SEARCH', 'BLUE_LINE_FOLLOW',
            'BLUE_LINE_DEAD_END',
        ):
            return True
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

    def compute_barrel_approach_point(self, target_x, target_y, approach_offset=None, lateral_offset=0.3):
        """Approach a barrel with a configurable stand-off and lateral shift.
        Positive lateral offset is robot-right; negative is robot-left."""
        if self.latest_robot_pose is None:
            return None

        robot_x = self.latest_robot_pose.position.x
        robot_y = self.latest_robot_pose.position.y
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.05:
            return None

        offset = self.approach_offset if approach_offset is None else approach_offset
        if dist <= offset:
            goal_x = robot_x
            goal_y = robot_y
        else:
            factor = (dist - offset) / dist
            goal_x = robot_x + dx * factor
            goal_y = robot_y + dy * factor

        # Right-perpendicular of the approach direction (clockwise 90°)
        right_x = dy / dist
        right_y = -dx / dist
        goal_x += lateral_offset * right_x
        goal_y += lateral_offset * right_y
        # Keep yaw pointing toward the barrel
        goal_yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        return goal_x, goal_y, goal_yaw

    def compute_face_approach_point(self, face_x, face_y, face_yaw):
        # Stand directly in front of the face.
        dist = self.face_approach_offset
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
            rejected_type = self.nav_goal_type
            self.nav_goal_type = None
            if self.post_patrol_active:
                self._handle_post_patrol_nav_failure(rejected_type)
            else:
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

        if self.skip_barrel_after_cancel:
            self.skip_barrel_after_cancel = False
            skipped_target = self.active_target
            if skipped_target is not None:
                self.handled_targets.append(skipped_target)
                self.get_logger().info(
                    f"Skipped barrel target -> x={skipped_target['x']:.2f}, "
                    f"y={skipped_target['y']:.2f}"
                )
            self.active_target = None
            self.refresh_state()

    def _skip_active_barrel_after_timeout(self):
        if self.active_target is None:
            return
        self.interaction_active = False
        self.interaction_end_time = None
        if self.goal_active:
            self.skip_barrel_after_cancel = True
            self.cancel_temporary_goal()
            return

        skipped_target = self.active_target
        self.handled_targets.append(skipped_target)
        self.get_logger().info(
            f"Skipped barrel target -> x={skipped_target['x']:.2f}, "
            f"y={skipped_target['y']:.2f}"
        )
        self.active_target = None
        self.approach_start_time = None
        self.refresh_state()

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

        # Post-patrol phases (workstation, final WP, line following) are managed
        # directly — don't let refresh_state override them back to IDLE.
        if self.post_patrol_active:
            self.publish_patrol_enabled(False)
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
        elif target_type in ('barrel', 'cylinder'):
            approach = self.compute_barrel_approach_point(
                x,
                y,
                metadata.get('barrel_approach_offset'),
                metadata.get('barrel_lateral_offset', 0.3),
            )
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

        if msg.ns and msg.ns != 'face_confirmed':
            return

        if not self.accepts_face_detections():
            return

        if self.face_qr_turn_active:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract face-toward-robot yaw encoded in marker orientation (set by face_localizator)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        face_yaw = 2.0 * math.atan2(qz, qw)

        blue_line_states = (
            'FOLLOW_BLUE_LINE', 'LINE_FOLLOWING',
            'BLUE_LINE_SEARCH', 'BLUE_LINE_FOLLOW', 'BLUE_LINE_DEAD_END',
        )
        if self.current_state in blue_line_states:
            self.get_logger().info('Face detected during blue-line mode — switching to face approach.')
            disable_msg = Bool()
            disable_msg.data = False
            self.blue_line_enabled_pub.publish(disable_msg)
            self.blue_line_pub.publish(disable_msg)
            self._stop_direct_drive()
            wait = False
        else:
            wait = True

        self.queue_target({'type': 'face', 'x': x, 'y': y, 'z': z, 'color': None, 'face_yaw': face_yaw,
                           'wait_for_group_end': wait})

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
            # Vertical (standing) barrels need no approach — detection/report only
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        color = self.marker_to_ring_color(msg)
        orientation = 'horizontal' if is_horizontal else 'vertical'
        target = {
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
        }

        if is_horizontal:
            target['barrel_approach_offset'] = 0.5
            target['barrel_lateral_offset'] = 0.3

        self.queue_target(target)

    def target_done_callback(self, msg: Empty):
        if self.active_target is None:
            self.get_logger().info('No active target to mark as done.')
            return

        if self.current_state not in ('INTERACT_FACE', 'INTERACT_RING', 'INTERACT_BARREL', 'APPROACH_FACE', 'APPROACH_RING', 'APPROACH_BARREL'):
            self.get_logger().info('Target done received, but current state is not target handling.')
            return

        completed_target = self.active_target
        self.handled_targets.append(completed_target)

        self.get_logger().info(
            f"Target completed -> type={completed_target['type']}, "
            f"x={completed_target['x']:.2f}, y={completed_target['y']:.2f}"
        )

        if self.goal_active:
            self.cancel_temporary_goal()

        self.active_target = None
        if completed_target.get('type') == 'face' and self.blue_line_runtime_started:
            self._resume_blue_line_runtime_after_face()
            return
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

    def _start_face_qr_turn(self):
        self.face_qr_turn_active = True
        self.face_qr_turn_target = None
        self.face_qr_turn_end = (
            self.get_clock().now().nanoseconds
            + int(self.face_qr_turn_duration * 1e9)
        )
        twist = Twist()
        twist.angular.z = float(self.face_qr_turn_speed)
        self.cmd_vel_pub.publish(twist)

    def _update_face_qr_turn(self):
        now = self.get_clock().now().nanoseconds
        if self.face_qr_turn_end is not None and now < self.face_qr_turn_end:
            twist = Twist()
            twist.angular.z = float(self.face_qr_turn_speed)
            self.cmd_vel_pub.publish(twist)
            return

        self._stop_direct_drive()
        self.face_qr_turn_active = False
        self.face_qr_turn_end = None
        self.face_qr_turn_target = None
        self.manual_control_active = False
        self.get_logger().info('Face QR sweep done — robot stopped.')

    def _stop_direct_drive(self):
        self.cmd_vel_pub.publish(Twist())

    def qr_callback(self, msg: String):
        lower = msg.data.strip().lower()
        if lower.startswith('defects '):
            color = lower.split(' ', 1)[1].strip()
            if color in ('red', 'green') and self.pending_workstation_color is None:
                self.pending_workstation_color = color
                self.get_logger().info(
                    f'QR workstation queued: visit {color} workstation before final waypoint.')

    def workstation_markers_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.type != Marker.CYLINDER:
                continue
            x = marker.pose.position.x
            y = marker.pose.position.y
            qz = marker.pose.orientation.z
            qw = marker.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            if marker.color.r > 0.5 and marker.color.g < 0.3:
                self.workstation_positions['red'] = (x, y, yaw)
            elif marker.color.g > 0.5 and marker.color.r < 0.3:
                self.workstation_positions['green'] = (x, y, yaw)

    def workstation_done_callback(self, msg: Empty):
        self.workstation_done_flag = True
        self.get_logger().info(
            'Workstation done signal received; waiting for inspector_finish before final waypoint.')

    def inspector_finish_callback(self, msg: String):
        if msg.data.strip().lower() == 'finished':
            self.inspector_finished_flag = True
            self.get_logger().info('Inspector finished signal received.')

    def _start_post_patrol_sequence(self):
        self.post_patrol_active = True
        color = self.pending_workstation_color
        if color is not None and color in self.workstation_positions:
            x, y, yaw = self.workstation_positions[color]
            self.get_logger().info(f'Post-patrol: navigating to {color} workstation.')
            self.publish_yellow_line_enabled(False)
            self.publish_state('APPROACH_WORKSTATION')
            self.publish_patrol_enabled(False)
            self.send_nav_goal(x, y, yaw, 'approach_workstation')
        else:
            if color is not None:
                self.get_logger().warn(
                    f'Workstation {color} position not yet known — skipping, going to final WP.')
            self._navigate_to_final_wp()

    def _navigate_to_final_wp(self, reset_failures=True):
        x = float(self.get_parameter('final_wp_x').value)
        y = float(self.get_parameter('final_wp_y').value)
        yaw = float(self.get_parameter('final_wp_yaw').value)
        if reset_failures:
            self.final_nav_failures = 0
        self.get_logger().info(f'Navigating to final waypoint ({x:.2f}, {y:.2f}, yaw={yaw:.2f}).')
        self.publish_yellow_line_enabled(False)
        self.publish_state('FINISHING_ROUNDS')
        self.publish_patrol_enabled(False)
        self.send_nav_goal(x, y, yaw, 'approach_final')

    @staticmethod
    def _angle_error(target, current):
        err = target - current
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        return err

    def _start_blue_line_runtime(self):
        if self.blue_line_runtime_started:
            return
        self.blue_line_runtime_started = True
        self.get_logger().info(
            'Final waypoint reached — enabling blue-line runtime and shutting down nonessential nodes.')
        self.publish_state('FOLLOW_BLUE_LINE')
        self._shutdown_nonessential_runtime_nodes()
        m = Bool()
        m.data = True
        self.blue_line_pub.publish(m)

    def _resume_blue_line_runtime_after_face(self):
        self.get_logger().info('Face interaction done — resuming blue-line following.')
        self.publish_state('FOLLOW_BLUE_LINE')
        self.publish_patrol_enabled(False)
        msg = Bool()
        msg.data = True
        self.blue_line_enabled_pub.publish(msg)
        self.blue_line_pub.publish(msg)

    def _shutdown_nonessential_runtime_nodes(self):
        keep_patterns = (
            'detect_people.py',
            'face_recognizer',
            'face_localizator',
            'blue_line_explorer',
            'behavior_manager',
            'robot_state_overlay',
            'report_manager',
            'qr_reader',
        )
        kill_patterns = (
            'detect_rings_v2',
            'ring_localizator',
            'cylinder_segmentation',
            'cylinder_localizator',
            'cylinder_debug_view',
            'barrel_inspector',
            'yellow_line_avoider',
            'line_localizator',
            'color_mask_viewer',
            'workstation_recorder',
            'waypoint_navigator',
            'station_inspector',
            'tile_detect',
            'tile_classifier',
            'orchestrator',
        )
        for pattern in kill_patterns:
            if pattern in keep_patterns:
                continue
            try:
                result = subprocess.run(
                    ['pkill', '-f', pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
                if result.returncode == 0:
                    self.get_logger().info(f'Shut down nonessential node: {pattern}')
            except Exception as exc:
                self.get_logger().warn(f'Could not shut down {pattern}: {exc}')

    def _handle_post_patrol_nav_failure(self, goal_type):
        if goal_type == 'approach_workstation':
            self.get_logger().warn('Workstation nav failed — skipping to final waypoint.')
            self.pending_workstation_color = None
            self._navigate_to_final_wp()
        elif goal_type == 'approach_final':
            self.final_nav_failures += 1
            if self.final_nav_failures <= 2:
                self.get_logger().warn(
                    f'Final WP nav failed — retrying ({self.final_nav_failures}/2).')
                self._navigate_to_final_wp(reset_failures=False)
            else:
                self.get_logger().error(
                    'Final WP nav failed repeatedly; blue-line runtime will not start until the final waypoint is reached.')

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
        self.need_post_patrol = True
        # Unlock all deferred targets — patrol is over, no more rotation groups.
        for t in self.pending_targets:
            t.pop('wait_for_group_end', None)
        # main_loop will call _start_post_patrol_sequence() once the robot is free.


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
