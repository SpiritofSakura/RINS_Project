#!/usr/bin/env python3

import math
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import tf2_ros


class FaceLocalizator(Node):
    def __init__(self):
        super().__init__('face_localizator')

        self.threshold_detections = 10
        self.cluster_radius = 1.0
        self.duplicate_radius = 0.9

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_sub = self.create_subscription(
            Marker,
            '/people_marker',
            self.marker_callback,
            qos_profile_sensor_data
        )

        self.recognized_sub = self.create_subscription(
            String,
            '/recognized_person',
            self.recognized_callback,
            10
        )

        self.face_locations_pub = self.create_publisher(
            Marker,
            '/detected_face_locations',
            10
        )

        self.face_markers_pub = self.create_publisher(
            MarkerArray,
            '/detected_face_markers',
            10
        )

        self.all_detections = []
        # Each entry: {'x': float, 'y': float, 'name': str, 'role': str, 'pronouns': str}
        self.marked_locations = []
        self.marker_id_counter = 0

        # Recent recognitions buffer: list of dicts from /recognized_person
        self.recent_recognitions = []
        self.max_recognition_buffer = 10

        self.get_logger().info(
            'Face localizator node initialized. '
            'Subscribing to /people_marker and /recognized_person'
        )

    def recognized_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.recent_recognitions.append(data)
            if len(self.recent_recognitions) > self.max_recognition_buffer:
                self.recent_recognitions.pop(0)
        except json.JSONDecodeError:
            pass

    def marker_callback(self, marker_msg):
        try:
            x_bl = marker_msg.pose.position.x
            y_bl = marker_msg.pose.position.y
            z_bl = marker_msg.pose.position.z

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x_map, y_map, z_map = self.transform_point(
                x_bl, y_bl, z_bl, transform
            )

            self.all_detections.append((x_map, y_map, z_map))
            self.check_and_mark_location(x_map, y_map)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF transform failed, skipping detection: {ex}')
        except Exception as e:
            self.get_logger().error(f'Error processing marker: {e}')

    def transform_point(self, x, y, z, transform):
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        R = self.quat_to_rotation_matrix(qx, qy, qz, qw)

        point = [x, y, z]
        rotated = [
            R[0][0] * point[0] + R[0][1] * point[1] + R[0][2] * point[2],
            R[1][0] * point[0] + R[1][1] * point[1] + R[1][2] * point[2],
            R[2][0] * point[0] + R[2][1] * point[1] + R[2][2] * point[2],
        ]

        return rotated[0] + tx, rotated[1] + ty, rotated[2] + tz

    def quat_to_rotation_matrix(self, qx, qy, qz, qw):
        return [
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ]

    def _best_recognition_near(self, x, y):
        """Return the most confident recognition within cluster_radius, or None."""
        best = None
        best_conf = -1.0
        for rec in self.recent_recognitions:
            rx, ry = rec.get('map_x', 0.0), rec.get('map_y', 0.0)
            dist = math.sqrt((rx - x)**2 + (ry - y)**2)
            if dist <= self.cluster_radius:
                conf = rec.get('confidence', 0.0)
                if conf > best_conf:
                    best_conf = conf
                    best = rec
        return best

    def check_and_mark_location(self, x, y):
        detections_in_radius = sum(
            1 for dx, dy, _ in self.all_detections
            if math.sqrt((dx - x)**2 + (dy - y)**2) <= self.cluster_radius
        )

        if detections_in_radius < self.threshold_detections:
            return

        for loc in self.marked_locations:
            if math.sqrt((loc['x'] - x)**2 + (loc['y'] - y)**2) < self.duplicate_radius:
                return

        recognition = self._best_recognition_near(x, y)
        name = recognition['name'] if recognition else 'Unknown'
        role = recognition['role'] if recognition else ''
        pronouns = recognition['pronouns'] if recognition else ''

        entry = {'x': x, 'y': y, 'name': name, 'role': role, 'pronouns': pronouns}
        self.marked_locations.append(entry)
        self.publish_persistent_marker(x, y, name, role)

        self.get_logger().info(
            f'Person confirmed: {name} ({role}) '
            f'at ({x:.2f}, {y:.2f}) — {detections_in_radius} detections'
        )

    def publish_persistent_marker(self, x, y, name, role):
        marker_id = self.marker_id_counter
        self.marker_id_counter += 1

        # Sphere marker (position)
        sphere = Marker()
        sphere.header.frame_id = 'map'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.type = Marker.SPHERE
        sphere.id = marker_id * 2
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.0
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.3
        sphere.scale.y = 0.3
        sphere.scale.z = 0.3
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0

        # Text marker (name + role)
        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.type = Marker.TEXT_VIEW_FACING
        text.id = marker_id * 2 + 1
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        label = name if not role else f'{name}\n{role}'
        text.text = label

        self.face_locations_pub.publish(sphere)

        arr = MarkerArray()
        arr.markers = [sphere, text]
        self.face_markers_pub.publish(arr)


def main():
    print('Face localizator node starting.')
    rclpy.init(args=None)
    node = FaceLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
