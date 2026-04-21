#!/usr/bin/env python3
"""Drive the robot to each position, press Enter to capture, Ctrl+C to save."""

import math
import sys
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    return 2.0 * math.atan2(q.z, q.w)


class WaypointCapture(Node):
    def __init__(self):
        super().__init__('waypoint_capture')
        self.current_pose = None
        self.waypoints = []

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._pose_cb, qos)
        self.get_logger().info('Waiting for /amcl_pose...')

    def _pose_cb(self, msg):
        self.current_pose = msg.pose.pose

    def capture(self, label=''):
        if self.current_pose is None:
            print('  No pose yet — is AMCL running?')
            return
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = yaw_from_quaternion(self.current_pose.orientation)
        wp = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 2)}
        self.waypoints.append((label, wp))
        print(f'  Captured: x={x:.4f}  y={y:.4f}  yaw={yaw:.2f}  ({label})')

    def save(self, path):
        data = {'waypoints': []}
        for label, wp in self.waypoints:
            data['waypoints'].append(wp)
        with open(path, 'w') as f:
            for label, wp in self.waypoints:
                f.write(f'# {label}\n' if label else '')
                f.write(f"- x: {wp['x']}\n")
                f.write(f"  y: {wp['y']}\n")
                f.write(f"  yaw: {wp['yaw']}\n\n")
        print(f'\nSaved {len(self.waypoints)} waypoints to {path}')


def main():
    output = 'new_waypoints.yaml'
    if len(sys.argv) > 1:
        output = sys.argv[1]

    rclpy.init()
    node = WaypointCapture()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\n=== Waypoint Capture ===')
    print('Move the robot to each position, then press Enter to record it.')
    print('Type a label (optional) and press Enter. Ctrl+C when done.\n')

    try:
        while True:
            label = input('Label (or blank): ').strip()
            node.capture(label)
    except (KeyboardInterrupt, EOFError):
        pass

    node.save(output)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
