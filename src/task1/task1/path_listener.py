#!/usr/bin/env python3

import os
import yaml
import glob
import math

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle in radians."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return yaw


class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        
        # Get the source directory (not install) for persistent storage
        # This ensures files are saved to the source, not the install directory
        pkg_dir = get_package_share_directory('task1')
        # Convert install path back to source path
        if '/install/' in pkg_dir:
            src_config_dir = pkg_dir.replace('/install/share/task1', '/src/task1/config')
        else:
            src_config_dir = os.path.join(pkg_dir, 'config')
        
        self.config_dir = src_config_dir
        
        # Ensure config directory exists
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)
            self.get_logger().info(f'Created config directory: {self.config_dir}')
        
        # Find the next available path number
        self.path_number = self.get_next_path_number()
        self.path_file = os.path.join(self.config_dir, f'path{self.path_number:02d}.yaml')
        
        # Initialize the path file with empty waypoints list
        self.waypoints = []
        self.save_path()
        
        # Subscribe to RViz clicked points
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )
        
        self.get_logger().info(
            f'Path listener initialized. Config dir: {self.config_dir}'
        )
        self.get_logger().info(
            f'Recording to: {self.path_file}'
        )
        self.get_logger().info(
            f'Listening on topic: /clicked_point (RViz Publish Point)'
        )
    
    def get_next_path_number(self):
        """Find the next available path number."""
        pattern = os.path.join(self.config_dir, 'path*.yaml')
        existing_files = glob.glob(pattern)
        
        if not existing_files:
            return 0
        
        # Extract numbers from filenames
        numbers = []
        for filepath in existing_files:
            filename = os.path.basename(filepath)
            # Extract number from pathXX.yaml format
            try:
                num = int(filename[4:6])
                numbers.append(num)
            except (ValueError, IndexError):
                pass
        
        if not numbers:
            return 0
        
        return max(numbers) + 1
    
    def point_callback(self, msg):
        """Callback for receiving RViz clicked points."""
        self.get_logger().info('=== POINT CLICKED ===')
        
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        # Default yaw to 0 since PointStamped doesn't have orientation
        yaw = 0.0
        
        # Create waypoint entry
        waypoint = {
            'x': round(x, 4),
            'y': round(y, 4),
            'z': round(z, 4),
            'yaw': round(yaw, 4)
        }
        
        # Add to waypoints list
        self.waypoints.append(waypoint)
        
        self.get_logger().info(f'Total waypoints so far: {len(self.waypoints)}')
        
        # Save immediately to file
        self.save_path()
        
        self.get_logger().info(
            f'Point received and saved (waypoint {len(self.waypoints)}): '
            f'x={x:.4f}, y={y:.4f}, z={z:.4f}, yaw={yaw:.4f}'
        )
        self.get_logger().info(f'Saved to: {self.path_file}')
    
    def save_path(self):
        """Save waypoints to YAML file."""
        data = {
            'waypoints': self.waypoints
        }
        
        try:
            with open(self.path_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f'Failed to save path file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
