#!/usr/bin/env python3

import os
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros

import cv2
import numpy as np

try:
    import face_recognition
    FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    FACE_RECOGNITION_AVAILABLE = False

GENDER_LABELS = ['male', 'female']
GENDER_MODEL_MEAN = (78.4263377603, 87.7689143744, 114.895847746)


def parse_personnel_filename(filename):
    """Parse 'firstname_he_him_job_title.png' -> (name, pronouns, role)."""
    stem = os.path.splitext(filename)[0]
    parts = stem.split('_')
    name = parts[0].capitalize()
    pronouns = f'{parts[1]}/{parts[2]}'
    role = ' '.join(parts[3:])
    return name, pronouns, role


def pronouns_to_gender(pronouns):
    p = pronouns.lower()
    if 'he' in p or 'him' in p:
        return 'male'
    if 'she' in p or 'her' in p:
        return 'female'
    return 'unknown'


class FaceRecognizer(Node):
    def __init__(self):
        super().__init__('face_recognizer')

        if not FACE_RECOGNITION_AVAILABLE:
            self.get_logger().error(
                'face_recognition library not found. '
                'Install with: pip3 install face_recognition --break-system-packages'
            )
            raise RuntimeError('face_recognition not available')

        self.declare_parameter('personnel_dir', '')
        self.declare_parameter('recognition_tolerance', 0.6)
        self.declare_parameter('device', '')

        tolerance = self.get_parameter('recognition_tolerance').get_parameter_value().double_value
        personnel_dir = self.get_parameter('personnel_dir').get_parameter_value().string_value

        if not personnel_dir:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('task1')
            personnel_dir = os.path.join(pkg, 'config', 'personnel')

        models_dir = os.path.join(os.path.dirname(personnel_dir), 'models')
        self.gender_net = cv2.dnn.readNet(
            os.path.join(models_dir, 'gender_deploy.prototxt'),
            os.path.join(models_dir, 'gender_net.caffemodel'),
        )

        self.tolerance = tolerance
        self.bridge = CvBridge()
        self.latest_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_marker_map_xy = None   # updated by /people_marker, used for publishing coords
        self._last_recog_time = 0.0      # throttle recognition to 2 Hz
        self._last_pub_time = {}         # per-name throttle: don't spam /recognized_person
        # Cached results from last recognition run: list of (location, name, gender, role)
        self._cached_recognitions = []

        self.known_encodings = []
        self.known_names = []
        self.known_pronouns = []
        self.known_roles = []
        self.known_genders = []

        self._load_personnel(personnel_dir)

        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self._image_callback,
            qos_profile_sensor_data
        )

        self.marker_sub = self.create_subscription(
            Marker,
            '/people_marker',
            self._marker_callback,
            qos_profile_sensor_data
        )

        self.person_pub = self.create_publisher(String, '/recognized_person', 10)
        self.debug_pub = self.create_publisher(Image, '/face_debug_image', 10)

        self.get_logger().info(
            f'Face recognizer ready. '
            f'{len(self.known_names)} personnel loaded: {self.known_names}'
        )

    def _load_personnel(self, personnel_dir):
        if not os.path.isdir(personnel_dir):
            self.get_logger().error(f'Personnel directory not found: {personnel_dir}')
            return

        for filename in sorted(os.listdir(personnel_dir)):
            if not filename.lower().endswith('.png'):
                continue

            filepath = os.path.join(personnel_dir, filename)
            name, pronouns, role = parse_personnel_filename(filename)

            img = face_recognition.load_image_file(filepath)
            encodings = face_recognition.face_encodings(img)

            if not encodings:
                self.get_logger().warn(f'No face found in {filename}, skipping.')
                continue

            self.known_encodings.append(encodings[0])
            self.known_names.append(name)
            self.known_pronouns.append(pronouns)
            self.known_roles.append(role)
            self.known_genders.append(pronouns_to_gender(pronouns))
            self.get_logger().info(f'  Loaded: {name} ({role})')

    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError:
            return

        now = time.monotonic()

        # Run face detection + recognition at 2 Hz (face_locations is slow on CPU)
        if now - self._last_recog_time >= 0.5:
            self._last_recog_time = now
            img = self.latest_image.copy()
            face_locations = face_recognition.face_locations(img)
            self._cached_recognitions = []

            if face_locations:
                face_encs = face_recognition.face_encodings(img, face_locations)
                for enc, loc in zip(face_encs, face_locations):
                    name, gender, role, pronouns, confidence = 'Unknown', 'unknown', '', '', 0.0

                    # Detect gender first so we can use it to validate the face match
                    gender = self._detect_gender(img, loc)

                    distances = face_recognition.face_distance(self.known_encodings, enc)
                    if len(distances) > 0:
                        best_idx = int(np.argmin(distances))
                        best_dist = float(distances[best_idx])

                        # Cross-gender matches need higher confidence to avoid false positives
                        known_gender = self.known_genders[best_idx]
                        threshold = self.tolerance
                        if (gender != 'unknown' and known_gender != 'unknown'
                                and gender != known_gender):
                            threshold = 0.45

                        if best_dist <= threshold:
                            name = self.known_names[best_idx]
                            role = self.known_roles[best_idx]
                            pronouns = self.known_pronouns[best_idx]
                            confidence = round(1.0 - best_dist, 3)

                            last_pub = self._last_pub_time.get(name, 0.0)
                            if now - last_pub >= 2.0 and self.last_marker_map_xy is not None:
                                self._last_pub_time[name] = now
                                payload = {
                                    'name': name, 'pronouns': pronouns,
                                    'role': role, 'gender': gender,
                                    'confidence': confidence,
                                    'map_x': self.last_marker_map_xy[0],
                                    'map_y': self.last_marker_map_xy[1],
                                }
                                msg_out = String()
                                msg_out.data = json.dumps(payload)
                                self.person_pub.publish(msg_out)
                                self.get_logger().info(
                                    f'Recognized: {name} ({role}, {gender}) '
                                    f'confidence={confidence:.2f}'
                                )

                    self._cached_recognitions.append((loc, name, gender, role))

        # Draw live frame with cached recognition boxes
        draw_img = cv2.cvtColor(self.latest_image, cv2.COLOR_RGB2BGR)
        for loc, name, gender, role in self._cached_recognitions:
            top, right, bottom, left = loc
            color = (0, 220, 0) if name != 'Unknown' else (100, 100, 255)
            cv2.rectangle(draw_img, (left, top), (right, bottom), color, 2)
            label = f"{name} ({gender})" if name != 'Unknown' else '?'
            cv2.putText(draw_img, label, (left, max(top - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            if role:
                cv2.putText(draw_img, role, (left, bottom + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow('Face Recognition', draw_img)
        cv2.waitKey(1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, 'bgr8'))
        except CvBridgeError:
            pass

    def _marker_callback(self, marker_msg):
        map_xy = self._marker_map_xy(marker_msg)
        if map_xy is not None:
            self.last_marker_map_xy = map_xy

    def _detect_gender(self, img, location):
        try:
            top, right, bottom, left = location
            face_crop = cv2.cvtColor(img[top:bottom, left:right], cv2.COLOR_RGB2BGR)
            blob = cv2.dnn.blobFromImage(
                face_crop, 1.0, (227, 227), GENDER_MODEL_MEAN, swapRB=False
            )
            self.gender_net.setInput(blob)
            preds = self.gender_net.forward()
            return GENDER_LABELS[preds[0].argmax()]
        except Exception:
            return 'unknown'

    def _marker_map_xy(self, marker_msg):
        source_frame = marker_msg.header.frame_id.lstrip('/') or 'base_link'
        if source_frame == 'map':
            return marker_msg.pose.position.x, marker_msg.pose.position.y

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
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
                )
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'Face recognition TF failed: {ex}')
                return None

        x, y, _ = self._transform_point(
            marker_msg.pose.position.x,
            marker_msg.pose.position.y,
            marker_msg.pose.position.z,
            transform,
        )
        return x, y

    @staticmethod
    def _transform_point(x, y, z, transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = FaceRecognizer._quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        rx = rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z
        ry = rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z
        rz = rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _quat_to_rotation_matrix(qx, qy, qz, qw):
        return [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ]


def main():
    print('Face recognizer node starting.')
    rclpy.init(args=None)
    try:
        node = FaceRecognizer()
        rclpy.spin(node)
        node.destroy_node()
    except RuntimeError:
        pass
    finally:
        cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
