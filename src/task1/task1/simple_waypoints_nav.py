#!/usr/bin/env python3

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion."""
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


class SimpleWaypointsNavigator(Node):
    def __init__(self):
        super().__init__('simple_waypoints_nav')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = self.load_waypoints()
        self.current_wp_index = 0
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # Start navigation after 10 second delay
        time.sleep(10.0)
        self.navigate_waypoints()
    
    def load_waypoints(self):
        """Load waypoints from yaml config file."""
        try:
            pkg_path = get_package_share_directory('task1')
            yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')
            
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            if data and 'waypoints' in data:
                return data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
        
        return []
    
    def navigate_waypoints(self):
        """Navigate through all waypoints sequentially."""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded')
            return
        
        # Wait for server
        self.get_logger().info('Waiting for navigate_to_pose server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available')
            return
        
        self.get_logger().info('Starting waypoint navigation')
        self.send_next_waypoint()
    
    def send_next_waypoint(self):
        """Send the next waypoint in sequence."""
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed')
            return
        
        self.get_logger().info(
            f'Navigating to waypoint {self.current_wp_index + 1}/{len(self.waypoints)}'
        )
        self.navigate_to_waypoint(self.waypoints[self.current_wp_index])
    
    def navigate_to_waypoint(self, waypoint):
        """Send a navigation goal to a single waypoint."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(waypoint['x'])
        goal.pose.position.y = float(waypoint['y'])
        goal.pose.orientation = yaw_to_quaternion(float(waypoint.get('yaw', 0.0)))
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        
        self.get_logger().info(
            f"Sending goal: x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint.get('yaw', 0.0)}"
        )
        
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response from server."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle result of navigation goal."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self.current_wp_index + 1} reached')
            self.current_wp_index += 1
            self.send_next_waypoint()
        else:
            self.get_logger().error(f'Goal failed with status {status}')


def main(args=None):
    rclpy.init(args=args)
    nav = SimpleWaypointsNavigator()
    rclpy.spin(nav)
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
