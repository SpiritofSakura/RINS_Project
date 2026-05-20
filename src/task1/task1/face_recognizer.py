#!/usr/bin/env python3

import os
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

try:
    import face_recognition
    FACE_RECOGNITION_AVAILABLE = True
except ImportError:
    FACE_RECOGNITION_AVAILABLE = False

try:
    os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '3')
    from deepface import DeepFace
    DEEPFACE_AVAILABLE = True
except Exception:
    DEEPFACE_AVAILABLE = False


def parse_personnel_filename(filename):
    """Parse 'firstname_he_him_job_title.png' -> (name, pronouns, role)."""
    stem = os.path.splitext(filename)[0]
    parts = stem.split('_')
    name = parts[0].capitalize()
    pronouns = f'{parts[1]}/{parts[2]}'
    role = ' '.join(parts[3:])
    return name, pronouns, role


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
        self.declare_parameter('recognition_tolerance', 0.5)
        self.declare_parameter('device', '')

        tolerance = self.get_parameter('recognition_tolerance').get_parameter_value().double_value
        personnel_dir = self.get_parameter('personnel_dir').get_parameter_value().string_value

        if not personnel_dir:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('task1')
            personnel_dir = os.path.join(pkg, 'config', 'personnel')

        self.tolerance = tolerance
        self.bridge = CvBridge()
        self.latest_image = None
        # Persistent overlay: list of {location, name, gender, role, stamp_ns}
        self.face_overlays = []
        self.overlay_fade_ns = int(3.0 * 1e9)

        self.known_encodings = []
        self.known_names = []
        self.known_pronouns = []
        self.known_roles = []

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
            self.get_logger().info(f'  Loaded: {name} ({role})')

    def _image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError:
            return

        now_ns = self.get_clock().now().nanoseconds
        fresh = [o for o in self.face_overlays
                 if now_ns - o['stamp_ns'] < self.overlay_fade_ns]
        self.face_overlays = fresh

        draw_img = cv2.cvtColor(self.latest_image, cv2.COLOR_RGB2BGR)
        for o in fresh:
            top, right, bottom, left = o['location']
            cv2.rectangle(draw_img, (left, top), (right, bottom), (0, 220, 0), 2)
            label = f"{o['name']} ({o['gender']})"
            cv2.putText(draw_img, label, (left, max(top - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            if o['role']:
                cv2.putText(draw_img, o['role'], (left, bottom + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow('Face Recognition', draw_img)
        cv2.waitKey(1)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(draw_img, 'bgr8'))
        except CvBridgeError:
            pass

    def _marker_callback(self, marker_msg):
        if self.latest_image is None:
            return

        img = self.latest_image.copy()

        face_locations = face_recognition.face_locations(img)
        if not face_locations:
            return

        face_encodings = face_recognition.face_encodings(img, face_locations)

        for encoding, location in zip(face_encodings, face_locations):
            distances = face_recognition.face_distance(self.known_encodings, encoding)

            if len(distances) == 0:
                continue

            best_idx = int(np.argmin(distances))
            best_dist = float(distances[best_idx])

            if best_dist > self.tolerance:
                self.get_logger().debug(
                    f'Face detected but no match (best dist={best_dist:.2f})'
                )
                continue

            name = self.known_names[best_idx]
            pronouns = self.known_pronouns[best_idx]
            role = self.known_roles[best_idx]
            confidence = round(1.0 - best_dist, 3)

            # Gender detection from face crop
            gender = 'unknown'
            if DEEPFACE_AVAILABLE:
                try:
                    top, right, bottom, left = location
                    face_crop = img[top:bottom, left:right]
                    result = DeepFace.analyze(
                        face_crop, actions=['gender'],
                        enforce_detection=False, silent=True
                    )
                    gender = result[0]['dominant_gender'].lower()  # 'man' or 'woman'
                except Exception:
                    pass

            payload = {
                'name': name,
                'pronouns': pronouns,
                'role': role,
                'gender': gender,
                'confidence': confidence,
                'map_x': marker_msg.pose.position.x,
                'map_y': marker_msg.pose.position.y,
            }

            msg_out = String()
            msg_out.data = json.dumps(payload)
            self.person_pub.publish(msg_out)

            self.get_logger().info(
                f'Recognized: {name} ({role}, {gender}) '
                f'confidence={confidence:.2f} '
                f'at map ({marker_msg.pose.position.x:.2f}, {marker_msg.pose.position.y:.2f})'
            )

            self.face_overlays.append({
                'location': location,
                'name': name,
                'gender': gender,
                'role': role,
                'stamp_ns': self.get_clock().now().nanoseconds,
            })
            self.face_overlays = self.face_overlays[-10:]


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
