#!/usr/bin/env python3

import math
import json
from collections import Counter
from statistics import median

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros


class FaceCluster:
    __slots__ = ('points', 'recognition_votes', 'best_recognition', 'confirmed')

    def __init__(self, x, y, z, recognition=None):
        self.points = [(x, y, z)]
        self.recognition_votes = Counter()
        self.best_recognition = None
        self.confirmed = False
        self.add_recognition(recognition)

    def add(self, x, y, z, recognition=None):
        self.points.append((x, y, z))
        self.add_recognition(recognition)

    def add_recognition(self, recognition):
        if not recognition:
            return
        key = (
            recognition.get('name', 'Unknown'),
            recognition.get('role', ''),
            recognition.get('pronouns', ''),
            recognition.get('gender', ''),
        )
        self.recognition_votes[key] += 1
        if (
            self.best_recognition is None
            or recognition.get('confidence', 0.0) > self.best_recognition.get('confidence', 0.0)
        ):
            self.best_recognition = recognition

    @property
    def count(self):
        return len(self.points)

    @property
    def centroid(self):
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        zs = [p[2] for p in self.points]
        return median(xs), median(ys), median(zs)

    def dist2d(self, x, y):
        cx, cy, _ = self.centroid
        return math.sqrt((cx - x) ** 2 + (cy - y) ** 2)


class FaceLocalizator(Node):
    def __init__(self):
        super().__init__('face_localizator')

        self.threshold_detections = 5
        self.cluster_radius = 0.6
        self.duplicate_radius = 0.8
        self.recognition_radius = 0.8
        self.max_detection_range = 3.5  # m — ignore faces farther than this

        self.robot_x = 0.0
        self.robot_y = 0.0

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

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

        self.face_locations_pub = self.create_publisher(
            Marker,
            '/detected_face_locations',
            10
        )

        self.clusters = []
        # Each entry: {'x': float, 'y': float, 'name': str, 'role': str, 'pronouns': str, 'gender': str}
        self.marked_locations = []
        self.marker_id_counter = 0

        # Recent recognitions buffer: list of dicts from /recognized_person
        self.recent_recognitions = []
        self.max_recognition_buffer = 10

        self.get_logger().info(
            'Face localizator node initialized. '
            'Subscribing to /people_marker and /recognized_person'
        )

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def recognized_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.recent_recognitions.append(data)
            if len(self.recent_recognitions) > self.max_recognition_buffer:
                self.recent_recognitions.pop(0)
            self.update_marked_location_from_recognition(data)
        except json.JSONDecodeError:
            pass

    def marker_callback(self, marker_msg):
        try:
            x_map, y_map, z_map = self.marker_to_map(marker_msg)
            if not all(math.isfinite(v) for v in (x_map, y_map, z_map)):
                return

            dist_to_robot = math.sqrt((x_map - self.robot_x)**2 + (y_map - self.robot_y)**2)
            if dist_to_robot > self.max_detection_range:
                return

            recognition = self._best_recognition_near(x_map, y_map)
            cluster = self._nearest_cluster(x_map, y_map)
            if cluster is None or cluster.dist2d(x_map, y_map) > self.cluster_radius:
                cluster = FaceCluster(x_map, y_map, z_map, recognition)
                self.clusters.append(cluster)
            else:
                cluster.add(x_map, y_map, z_map, recognition)

            if cluster.count >= self.threshold_detections and not cluster.confirmed:
                self.try_confirm_cluster(cluster)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF transform failed, skipping detection: {ex}')
        except Exception as e:
            self.get_logger().error(f'Error processing marker: {e}')

    def marker_to_map(self, marker_msg):
        source_frame = marker_msg.header.frame_id.lstrip('/') or 'base_link'
        if source_frame == 'map':
            p = marker_msg.pose.position
            return p.x, p.y, p.z

        stamp = marker_msg.header.stamp
        lookup_time = (
            rclpy.time.Time.from_msg(stamp)
            if stamp.sec != 0 or stamp.nanosec != 0
            else rclpy.time.Time()
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', source_frame, lookup_time, timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException:
            transform = self.tf_buffer.lookup_transform(
                'map', source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )

        return self.transform_point(
            marker_msg.pose.position.x,
            marker_msg.pose.position.y,
            marker_msg.pose.position.z,
            transform,
        )

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
            if dist <= self.recognition_radius:
                conf = rec.get('confidence', 0.0)
                if conf > best_conf:
                    best_conf = conf
                    best = rec
        return best

    def _nearest_cluster(self, x, y):
        best = None
        best_d = float('inf')
        for cluster in self.clusters:
            d = cluster.dist2d(x, y)
            if d < best_d:
                best = cluster
                best_d = d
        return best

    def try_confirm_cluster(self, cluster):
        x, y, z = cluster.centroid
        for loc in self.marked_locations:
            if math.sqrt((loc['x'] - x)**2 + (loc['y'] - y)**2) < self.duplicate_radius:
                cluster.confirmed = True
                return

        # Direction robot→face = direction the face is looking (face was visible to robot at detection)
        face_yaw = math.atan2(self.robot_y - y, self.robot_x - x)

        recognition = cluster.best_recognition or self._best_recognition_near(x, y)
        name = recognition.get('name', 'Unknown') if recognition else 'Unknown'
        role = recognition.get('role', '') if recognition else ''
        pronouns = recognition.get('pronouns', '') if recognition else ''
        gender = recognition.get('gender', '') if recognition else ''

        entry = {
            'x': x,
            'y': y,
            'name': name,
            'role': role,
            'pronouns': pronouns,
            'gender': gender,
            'confidence': recognition.get('confidence', 0.0) if recognition else 0.0,
            'marker_id': self.marker_id_counter,
            'face_yaw': face_yaw,
        }
        self.marked_locations.append(entry)
        cluster.confirmed = True
        self.publish_persistent_marker(x, y, name, role, gender, entry['marker_id'], face_yaw)
        self.marker_id_counter += 1

        self.get_logger().info(
            f'Person confirmed: {name} ({role}, {gender}) '
            f'at ({x:.2f}, {y:.2f}) — {cluster.count} detections'
        )

    def update_marked_location_from_recognition(self, recognition):
        rx = recognition.get('map_x')
        ry = recognition.get('map_y')
        if rx is None or ry is None:
            return

        best = None
        best_dist = float('inf')
        for loc in self.marked_locations:
            dist = math.sqrt((loc['x'] - rx) ** 2 + (loc['y'] - ry) ** 2)
            if dist < best_dist:
                best = loc
                best_dist = dist

        if best is None or best_dist > self.recognition_radius:
            return

        confidence = recognition.get('confidence', 0.0)
        if best.get('name') != 'Unknown' and confidence <= best.get('confidence', 0.0):
            return

        best['name'] = recognition.get('name', best.get('name', 'Unknown'))
        best['role'] = recognition.get('role', best.get('role', ''))
        best['pronouns'] = recognition.get('pronouns', best.get('pronouns', ''))
        best['gender'] = recognition.get('gender', best.get('gender', ''))
        best['confidence'] = confidence

        self.publish_persistent_marker(
            best['x'], best['y'], best['name'], best['role'], best['gender'],
            best['marker_id'], best.get('face_yaw')
        )
        self.get_logger().info(
            f"Updated face marker label: {best['name']} at ({best['x']:.2f}, {best['y']:.2f})"
        )

    def publish_persistent_marker(self, x, y, name, role, gender='', marker_id=None, face_yaw=None):
        if marker_id is None:
            marker_id = self.marker_id_counter
            self.marker_id_counter += 1

        sphere = Marker()
        sphere.header.frame_id = 'map'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'face_confirmed'
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.id = marker_id
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.0
        # Encode face_yaw in orientation so behavior_manager can approach head-on
        if face_yaw is not None:
            sphere.pose.orientation.z = math.sin(face_yaw / 2.0)
            sphere.pose.orientation.w = math.cos(face_yaw / 2.0)
        else:
            sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.3
        sphere.scale.y = 0.3
        sphere.scale.z = 0.3
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0

        self.face_locations_pub.publish(sphere)

        label = Marker()
        label.header.frame_id = 'map'
        label.header.stamp = sphere.header.stamp
        label.ns = 'face_label'
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.id = marker_id
        label.pose.position.x = x + 0.25
        label.pose.position.y = y + 0.25
        label.pose.position.z = 0.55
        label.pose.orientation.w = 1.0
        label.scale.z = 0.28
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = name if name else 'Unknown'

        self.face_locations_pub.publish(label)


def main():
    print('Face localizator node starting.')
    rclpy.init(args=None)
    node = FaceLocalizator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
