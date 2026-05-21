#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
				('enabled', False),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value
		self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
		
		# Robot state for conditional publishing
		self.robot_state = 'IDLE'

		self.bridge = CvBridge()
		self.scan = None

		self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)
		self.robot_state_sub = self.create_subscription(String, "/robot_state", self.robot_state_callback, 10)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		status = "enabled" if self.enabled else "disabled"
		self.get_logger().info(f"Node has been initialized! Detection is {status}. Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		# Only process frames if detection is enabled
		if not self.enabled:
			return

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			#self.get_logger().info(f"Running inference on image...")

			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue

				self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0]

				cv_image = cv2.rectangle(
					cv_image,
					(int(bbox[0]), int(bbox[1])),
					(int(bbox[2]), int(bbox[3])),
					self.detection_color,
					3
				)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				cv_image = cv2.circle(cv_image, (cx, cy), 5, self.detection_color, -1)

				self.faces.append((cx, cy, [int(v) for v in bbox]))

			cv2.imshow("Face Camera", cv_image)
			cv2.waitKey(1)

		except CvBridgeError as e:
			print(e)

	def robot_state_callback(self, data):
		"""Update robot state for conditional marker publishing."""
		self.robot_state = data.data

	def pointcloud_callback(self, data):
		# Only publish markers when idle or on patrol - avoid interfering with other tasks
		if self.robot_state not in ['IDLE', 'PATROL']:
			return

		height = data.height
		width = data.width

		try:
			points = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
			points = points.reshape((height, width, 3))
		except Exception as e:
			self.get_logger().warn(f"Could not read point cloud for people markers: {e}")
			return

		# iterate over face coordinates
		for x, y, bbox in self.faces:

			d = self._robust_person_point(points, x, y, bbox)
			if d is None:
				continue

			# create marker
			marker = Marker()

			marker.header.frame_id = data.header.frame_id
			marker.header.stamp = data.header.stamp

			marker.type = 2
			marker.id = 0

			# Set the scale of the marker
			scale = 0.1
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			# Set the color
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 1.0
			marker.color.a = 1.0

			# Set the pose of the marker
			marker.pose.position.x = float(d[0])
			marker.pose.position.y = float(d[1])
			marker.pose.position.z = float(d[2])

			self.marker_pub.publish(marker)

	def _robust_person_point(self, points, cx, cy, bbox):
		"""Return a median point from the central torso patch instead of one noisy pixel."""
		height, width = points.shape[:2]
		x1, y1, x2, y2 = bbox

		box_w = max(1, x2 - x1)
		box_h = max(1, y2 - y1)
		roi_w = max(8, int(box_w * 0.25))
		roi_h = max(8, int(box_h * 0.25))

		# Person detections are full-body boxes. The torso is usually more stable
		# than the face or legs for point-cloud localization.
		torso_y = int(y1 + box_h * 0.45)
		rx1 = max(0, cx - roi_w // 2)
		rx2 = min(width, cx + roi_w // 2)
		ry1 = max(0, torso_y - roi_h // 2)
		ry2 = min(height, torso_y + roi_h // 2)

		patch = points[ry1:ry2, rx1:rx2].reshape(-1, 3)
		valid = patch[np.all(np.isfinite(patch), axis=1)]
		if len(valid) == 0:
			center = points[max(0, min(cy, height - 1)), max(0, min(cx, width - 1)), :]
			if np.all(np.isfinite(center)):
				return center
			return None

		return np.median(valid, axis=0)

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
