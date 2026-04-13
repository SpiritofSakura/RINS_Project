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

# ── Face detection tuning ────────────────────────────────────────────────────
FACE_Y_FRAC        = 0.50  # bbox bottom edge must be below this fraction of image height
                             # (rejects detections entirely above the border wall)
FACE_CONFIRM_HITS  = 6      # frames a face must be seen before it is reported
FACE_MAX_MISSED    = 8      # frames without a match before dropping a candidate
FACE_MATCH_DIST_PX = 50     # pixel radius to match detections across frames
FACE_DEPTH_STD_MAX = 0.20   # max depth std-dev (m) within bbox — flat-wall / picture check

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

		self.model = YOLO("yolov8m.pt")

		self.faces = []
		self._face_candidates = []   # cross-frame accumulator

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.rgb_image = cv_image
			img_h, img_w = cv_image.shape[:2]

			res = self.model.predict(cv_image, imgsz=640, conf=0.25, show=False, verbose=False, classes=[0], device=self.device)

			frame_detections = []

			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0:
					continue

				bbox = bbox[0]
				x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
				cx = int((x1 + x2) / 2)
				cy = int((y1 + y2) / 2)

				# Filter A: bbox bottom edge must reach the lower portion of the image.
				# Detections entirely above the border wall are rejected here.
				accepted = y2 >= int(img_h * FACE_Y_FRAC)

				colour = (0, 255, 0) if accepted else (0, 0, 255)
				cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), colour, 3)
				cv_image = cv2.circle(cv_image, (cx, cy), 5, colour, -1)

				if not accepted:
					continue

				frame_detections.append((cx, cy, x1, y1, x2, y2))

			# Filter C: cross-frame confirmation.
			# A detection must appear in FACE_CONFIRM_HITS consecutive-ish frames
			# before it is treated as a real face, killing single-frame false positives.
			matched = set()
			for det in frame_detections:
				dcx, dcy = det[0], det[1]
				best_idx, best_dist = None, FACE_MATCH_DIST_PX
				for i, cand in enumerate(self._face_candidates):
					d = np.hypot(cand['cx'] - dcx, cand['cy'] - dcy)
					if d < best_dist:
						best_dist, best_idx = d, i
				if best_idx is not None:
					self._face_candidates[best_idx]['hits']   += 1
					self._face_candidates[best_idx]['missed']  = 0
					self._face_candidates[best_idx]['cx']      = dcx
					self._face_candidates[best_idx]['cy']      = dcy
					self._face_candidates[best_idx]['det']     = det
					matched.add(best_idx)
				else:
					self._face_candidates.append(
						{'cx': dcx, 'cy': dcy, 'hits': 1, 'missed': 0, 'det': det})

			for i in range(len(self._face_candidates) - 1, -1, -1):
				if i not in matched:
					self._face_candidates[i]['missed'] += 1
				if self._face_candidates[i]['missed'] > FACE_MAX_MISSED:
					self._face_candidates.pop(i)

			self.faces = [c['det'] for c in self._face_candidates
						  if c['hits'] >= FACE_CONFIRM_HITS]
			if self.faces:
				self.get_logger().info(f"Person confirmed in view ({len(self.faces)} face(s))")

			cv2.imshow("image", cv_image)

			# ── REAL ROBOT: show disparity window ────────────────────────
			if self.real_robot and self.depth_image is not None:
				depth_f = self.depth_image.astype(np.float32)
				if self.depth_image.dtype == np.uint16:
					depth_f = depth_f / 1000.0
				with np.errstate(divide='ignore', invalid='ignore'):
					disp = np.where(depth_f > 0, 1.0 / depth_f, 0)
				disp_8u = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
				disp_bgr = cv2.cvtColor(disp_8u, cv2.COLOR_GRAY2BGR)
				dh, dw = disp_bgr.shape[:2]
				for (fx, fy, *_) in self.faces:
					cv2.circle(disp_bgr,
							   (int(fx * dw / img_w), int(fy * dh / img_h)),
							   8, (0, 255, 0), 2)
				cv2.imshow("Disparity (faces)", disp_bgr)

			key = cv2.waitKey(1)
			if key == 27:
				print("exiting")
				exit()

			# ── REAL ROBOT: publish face markers from depth image ─────────
			if self.real_robot:
				for det in self.faces:
					dcx, dcy, dx1, dy1, dx2, dy2 = det
					self._real_publish_face_marker(dcx, dcy, dx1, dy1, dx2, dy2)

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

	def _real_publish_face_marker(self, cx, cy, bbox_x1, bbox_y1, bbox_x2, bbox_y2):
		"""Back-project face pixel to 3D using depth image and camera intrinsics."""
		if self.depth_image is None or self.camera_intrinsics is None:
			return

		dh, dw = self.depth_image.shape[:2]
		colour_h, colour_w = self.rgb_image.shape[:2] if self.rgb_image is not None else (dh, dw)

		def _scale_x(px): return int(np.clip(px * dw / colour_w, 0, dw - 1))
		def _scale_y(py): return int(np.clip(py * dh / colour_h, 0, dh - 1))

		# Depth-uniformity check: faces are flat pictures on a wall, so depth
		# across the whole bounding box should be nearly constant.
		# High std-dev means different surfaces at different distances → not a flat face.
		bbx1, bby1 = _scale_x(bbox_x1), _scale_y(bbox_y1)
		bbx2, bby2 = _scale_x(bbox_x2), _scale_y(bbox_y2)
		bbox_depth = self.depth_image[bby1:bby2 + 1, bbx1:bbx2 + 1].astype(np.float32).ravel()
		if self.depth_image.dtype == np.uint16:
			bbox_depth = bbox_depth / 1000.0
		bbox_valid = bbox_depth[(bbox_depth > 0.1) & np.isfinite(bbox_depth)]
		if len(bbox_valid) < 10:
			return  # too few depth readings to evaluate
		depth_std = float(np.std(bbox_valid))
		if depth_std > FACE_DEPTH_STD_MAX:
			self.get_logger().debug(
				f"Depth not uniform (std={depth_std:.3f} m) — skipping")
			return

		# Back-project face centre to 3D
		dx = _scale_x(cx)
		dy = _scale_y(cy)
		p_x0, p_x1 = max(0, dx - 10), min(dw, dx + 11)
		p_y0, p_y1 = max(0, dy - 10), min(dh, dy + 11)
		patch = self.depth_image[p_y0:p_y1, p_x0:p_x1].astype(np.float32).ravel()
		if self.depth_image.dtype == np.uint16:
			patch = patch / 1000.0

		valid = patch[(patch > 0.1) & np.isfinite(patch)]
		if len(valid) == 0:
			return
		z = float(np.median(valid))

		cam_fx, cam_fy, ppx, ppy = self.camera_intrinsics
		pt_x = (dx - ppx) * z / cam_fx
		pt_y = (dy - ppy) * z / cam_fy

		dist = float(np.sqrt(pt_x**2 + pt_y**2 + z**2))
		if dist > 2.0:
			self.get_logger().debug(f"Face too far ({dist:.2f} m), skipping")
			return

		marker = Marker()
		marker.header.frame_id = self.camera_frame_id
		marker.header.stamp = self.get_clock().now().to_msg()
		marker.type = Marker.SPHERE
		marker.id = 0
		marker.scale.x = marker.scale.y = marker.scale.z = 0.1
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.color.a = 1.0
		marker.pose.position.x = pt_x
		marker.pose.position.y = pt_y
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
		for face_cx, face_cy, *_ in self.faces:

			# get 3-channel representation of the point cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			a = a.reshape((height,width,3))

			# clamp to valid range to avoid index errors
			face_cx = int(np.clip(face_cx, 0, width - 1))
			face_cy = int(np.clip(face_cy, 0, height - 1))

			# read center coordinates
			d = a[face_cy, face_cx, :]

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