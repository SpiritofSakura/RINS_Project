#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
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
				('real_robot', False),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value
		real = self.get_parameter('real_robot').get_parameter_value().bool_value

		# Robot state for conditional publishing
		self.robot_state = 'IDLE'

		self.real_robot = real
		self.bridge = CvBridge()
		self.scan = None

		if self.real_robot:
			# ── REAL ROBOT: depth image + camera intrinsics ──────────────────
			self.rgb_image = None
			self.depth_image = None
			self.camera_intrinsics = None  # (fx, fy, ppx, ppy)
			self.camera_frame_id = 'base_link'  # overwritten once camera_info arrives

			self.rgb_image_sub = self.create_subscription(
				Image, '/gemini/color/image_raw', self.rgb_callback, qos_profile_sensor_data)
			self.depth_image_sub = self.create_subscription(
				Image, '/gemini/depth/image_raw', self._real_depth_callback, qos_profile_sensor_data)
			self.camera_info_sub = self.create_subscription(
				CameraInfo, '/gemini/color/camera_info', self._real_camera_info_callback, 10)
			self.robot_state_sub = self.create_subscription(
				String, "/robot_state", self.robot_state_callback, 10)

			self.get_logger().info('[REAL] image=/gemini/color/image_raw  depth=/gemini/depth/image_raw')
		else:
			# ── SIMULATION: organized point cloud ────────────────────────────
			self.rgb_image_sub = self.create_subscription(
				Image, '/oakd/rgb/preview/image_raw', self.rgb_callback, qos_profile_sensor_data)
			self.pointcloud_sub = self.create_subscription(
				PointCloud2, '/oakd/rgb/preview/depth/points', self.pointcloud_callback, qos_profile_sensor_data)
			self.robot_state_sub = self.create_subscription(
				String, "/robot_state", self.robot_state_callback, 10)

			self.get_logger().info('[SIM] image=/oakd/rgb/preview/image_raw  pc=/oakd/rgb/preview/depth/points')

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.rgb_image = cv_image

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

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))

			cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()

			# ── REAL ROBOT: publish face markers from depth image ─────────
			if self.real_robot:
				for cx, cy in self.faces:
					self._real_publish_face_marker(cx, cy)

		except CvBridgeError as e:
			print(e)

	def robot_state_callback(self, data):
		"""Update robot state for conditional marker publishing."""
		self.robot_state = data.data

	# ── REAL ROBOT ONLY ───────────────────────────────────────────────────────

	def _real_depth_callback(self, data):
		"""Store the latest depth image from the Gemini camera."""
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		except CvBridgeError as e:
			self.get_logger().error(f'Depth image conversion failed: {e}')

	def _real_camera_info_callback(self, data):
		"""Store camera intrinsics and frame_id once (they don't change)."""
		if self.camera_intrinsics is None:
			fx, fy = data.k[0], data.k[4]
			ppx, ppy = data.k[2], data.k[5]
			self.camera_intrinsics = (fx, fy, ppx, ppy)
			self.camera_frame_id = data.header.frame_id
			self.get_logger().info(
				f'[REAL] Camera intrinsics: fx={fx:.1f} fy={fy:.1f} cx={ppx:.1f} cy={ppy:.1f} '
				f'frame={self.camera_frame_id}')

	def _real_publish_face_marker(self, cx, cy):
		"""Back-project face pixel to 3D using depth image and camera intrinsics."""
		if self.depth_image is None or self.camera_intrinsics is None:
			return

		dh, dw = self.depth_image.shape[:2]

		# Scale face pixel coords from colour image space to depth image space
		# (they may have different resolutions)
		colour_h, colour_w = self.rgb_image.shape[:2] if self.rgb_image is not None else (dh, dw)
		dx = int(np.clip(cx * dw / colour_w, 0, dw - 1))
		dy = int(np.clip(cy * dh / colour_h, 0, dh - 1))

		# Sample a patch around the face centre and take the median of valid values
		patch_r = 10
		y0, y1 = max(0, dy - patch_r), min(dh, dy + patch_r + 1)
		x0, x1 = max(0, dx - patch_r), min(dw, dx + patch_r + 1)
		patch = self.depth_image[y0:y1, x0:x1].astype(np.float32).ravel()

		# Gemini depth is uint16 in millimetres; 32F would already be metres
		if self.depth_image.dtype == np.uint16:
			patch = patch / 1000.0

		valid = patch[(patch > 0.1) & np.isfinite(patch)]
		if len(valid) == 0:
			return
		z = float(np.median(valid))

		fx, fy, ppx, ppy = self.camera_intrinsics
		# Back-project using depth-image pixel coords scaled to depth intrinsics
		x = (dx - ppx) * z / fx
		y = (dy - ppy) * z / fy

		marker = Marker()
		# Publish in the camera optical frame — TF will handle the rest
		marker.header.frame_id = self.camera_frame_id
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.type = Marker.SPHERE
		marker.id = 0
		marker.scale.x = marker.scale.y = marker.scale.z = 0.1
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.color.a = 1.0
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z
		self.marker_pub.publish(marker)

	def pointcloud_callback(self, data):
		# Only publish markers when idle or on patrol - avoid interfering with other tasks
		if self.robot_state not in ['IDLE', 'PATROL']:
			return

		# get point cloud attributes
		height = data.height
		width = data.width

		# Unorganized point cloud (height==1): can't index by pixel coordinate
		if height == 1:
			self.get_logger().warn(
				"Unorganized point cloud (height=1) — cannot map pixel to 3D point. "
				"Check that /gemini/depth/points publishes an organized cloud.",
				throttle_duration_sec=5.0)
			return

		# iterate over face coordinates
		for x,y in self.faces:

			# get 3-channel representation of the point cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			a = a.reshape((height,width,3))

			# clamp to valid range to avoid index errors
			x = int(np.clip(x, 0, width - 1))
			y = int(np.clip(y, 0, height - 1))

			# read center coordinates
			d = a[y,x,:]

			# create marker
			marker = Marker()

			marker.header.frame_id = "/base_link"
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

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()