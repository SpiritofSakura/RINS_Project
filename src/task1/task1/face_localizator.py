#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker
import tf2_ros


class FaceLocalizator(Node):
    def __init__(self):
        super().__init__('face_localizator')

        self.threshold_detections = 15
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

        self.face_locations_pub = self.create_publisher(
            Marker,
            '/detected_face_locations',
            10
        )

        self.all_detections = []
        self.marked_locations = []
        self.marker_id_counter = 0

        self.get_logger().info(
            'Face localizator node initialized. '
            'Subscribing to /people_marker'
        )

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
                x_bl,
                y_bl,
                z_bl,
                transform
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

        result_x = rotated[0] + tx
        result_y = rotated[1] + ty
        result_z = rotated[2] + tz

        return result_x, result_y, result_z

    def quat_to_rotation_matrix(self, qx, qy, qz, qw):
        R = [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)]
        ]
        return R

    def check_and_mark_location(self, x, y):
        detections_in_radius = 0

        for det_x, det_y, _ in self.all_detections:
            distance = math.sqrt((det_x - x) ** 2 + (det_y - y) ** 2)
            if distance <= self.cluster_radius:
                detections_in_radius += 1

        if detections_in_radius >= self.threshold_detections:
            already_marked = False

            for marked_x, marked_y in self.marked_locations:
                distance = math.sqrt((marked_x - x) ** 2 + (marked_y - y) ** 2)
                if distance < self.duplicate_radius:
                    already_marked = True
                    break

            if not already_marked:
                self.marked_locations.append((x, y))
                self.publish_persistent_marker(x, y)
                self.get_logger().info(
                    f'Face location marked! Total detections: {detections_in_radius} '
                    f'at ({x:.2f}, {y:.2f})'
                )

    def publish_persistent_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.face_locations_pub.publish(marker)


def main():
    print('Face localizator node starting.')
    rclpy.init(args=None)
    node = FaceLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()