#!/usr/bin/env python3
"""
GPU YOLO ring detector.

The node keeps the existing downstream interface:
  Pub: /ring_marker  (short-lived Marker in camera/point-cloud frame)
       /ring_colour  (String, for compatibility with older consumers)

The expensive image detection runs on the configured Ultralytics device. Depth
and point-cloud work stays small and only runs for YOLO candidates.
"""

import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


DEFAULT_MODEL_PATH = str(Path.home() / "ring_yolo" / "ring_det" / "weights" / "best.pt")
FALLBACK_CLASS_COLOURS = ["red", "green", "blue", "black"]

MARKER_COLOURS = {
    "red": (1.0, 0.0, 0.0),
    "green": (0.0, 1.0, 0.0),
    "blue": (0.0, 0.4, 1.0),
    "black": (0.1, 0.1, 0.1),
}

HSV_RANGES = {
    "red": [
        (np.array([0, 60, 35]), np.array([12, 255, 255])),
        (np.array([165, 60, 35]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([35, 45, 30]), np.array([92, 255, 255])),
    ],
    "blue": [
        (np.array([92, 55, 30]), np.array([148, 255, 255])),
    ],
    "black": [
        (np.array([0, 0, 0]), np.array([180, 255, 65])),
    ],
}

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

MARKER_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class RingDetectorYOLO(Node):
    def __init__(self):
        super().__init__("ring_detector_yolo")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("real_robot", False),
                ("model_path", DEFAULT_MODEL_PATH),
                ("device", "0"),
                ("conf", 0.30),
                ("imgsz", 640),
                ("inference_hz", 12.0),
                ("debug_image", True),
                ("display_windows", True),
                ("require_depth_hole", True),
                ("enable_colour_fallback", True),
                ("yolo_confirm_frames", 2),
                ("colour_confirm_frames", 4),
                ("max_detection_dist", 4.5),
                ("max_detection_dist_real", 2.2),
                ("image_topic", ""),
                ("depth_topic", ""),
                ("camera_info_topic", ""),
                ("pointcloud_topic", ""),
            ],
        )

        self.real_robot = self.get_parameter("real_robot").get_parameter_value().bool_value
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.conf = self.get_parameter("conf").get_parameter_value().double_value
        self.imgsz = self.get_parameter("imgsz").get_parameter_value().integer_value
        self.inference_hz = self.get_parameter("inference_hz").get_parameter_value().double_value
        self.debug_image = self.get_parameter("debug_image").get_parameter_value().bool_value
        self.display_windows = (
            self.get_parameter("display_windows").get_parameter_value().bool_value
        )
        self.require_depth_hole = (
            self.get_parameter("require_depth_hole").get_parameter_value().bool_value
        )
        self.enable_colour_fallback = (
            self.get_parameter("enable_colour_fallback").get_parameter_value().bool_value
        )
        self.yolo_confirm_frames = max(
            1, self.get_parameter("yolo_confirm_frames").get_parameter_value().integer_value
        )
        self.colour_confirm_frames = max(
            1, self.get_parameter("colour_confirm_frames").get_parameter_value().integer_value
        )
        self.max_detection_dist = (
            self.get_parameter("max_detection_dist_real").get_parameter_value().double_value
            if self.real_robot
            else self.get_parameter("max_detection_dist").get_parameter_value().double_value
        )
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.robot_state = "IDLE"
        self.depth_m = None
        self.camera_intrinsics = None
        self.camera_frame_id = "base_link"
        self.pointcloud_xyz = None
        self.pointcloud_frame_id = "base_link"
        self.marker_id = 0
        self._last_inference_time = 0.0
        self._frame_seq = 0
        self._candidate_tracks = []
        self.model = None
        self.model_names = {}

        self._latest_frame = None
        self._latest_header = None
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._latest_pc_msg = None
        self._pc_dirty = False

        self._init_model()
        self._init_ros_io()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"{'[REAL]' if self.real_robot else '[SIM]'} RingDetectorYOLO ready: "
            f"model={self.model_path}, device={self._device_name()}, imgsz={self.imgsz}, "
            f"conf={self.conf:.2f}, hz={self.inference_hz:.1f}"
        )

    def _init_model(self):
        if YOLO is None:
            raise RuntimeError("detect_rings_yolo requires the ultralytics package.")

        if not Path(self.model_path).is_file():
            self.get_logger().error(
                f"YOLO ring model not found: {self.model_path}. "
                "Train it with src/task1/scripts/train_ring_yolo.py or pass "
                "ring_model:=/path/to/best.pt."
            )
            return

        self.model = YOLO(self.model_path)
        self.model_names = self._normalise_model_names(self.model.names)

        self.model.predict(
            np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
            imgsz=self.imgsz,
            conf=self.conf,
            verbose=False,
            device=self._device_name(),
        )
        self.get_logger().info(
            f"YOLO ring model warmed up on {self._device_name()}; "
            f"classes={self.model_names}"
        )

    def _init_ros_io(self):
        if self.real_robot:
            image_topic = self.image_topic or "/gemini/color/image_raw/compressed"
            depth_topic = self.depth_topic or "/gemini/depth/image_raw"
            camera_info_topic = self.camera_info_topic or "/gemini/color/camera_info"
            self.create_subscription(
                CompressedImage,
                image_topic,
                self._compressed_image_cb,
                SENSOR_QOS,
            )
            self.create_subscription(Image, depth_topic, self._depth_cb, SENSOR_QOS)
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self._camera_info_cb,
                10,
            )
            self.get_logger().info(
                f"[REAL] image={image_topic} depth={depth_topic} "
                f"camera_info={camera_info_topic}"
            )
        else:
            image_topic = self.image_topic or "/oakd/rgb/preview/image_raw"
            depth_topic = self.depth_topic or "/oakd/rgb/preview/depth"
            pointcloud_topic = self.pointcloud_topic or "/oakd/rgb/preview/depth/points"
            self.create_subscription(
                Image,
                image_topic,
                self._image_cb,
                SENSOR_QOS,
            )
            self.create_subscription(Image, depth_topic, self._depth_cb, SENSOR_QOS)
            self.create_subscription(
                PointCloud2,
                pointcloud_topic,
                self._pointcloud_cb,
                SENSOR_QOS,
            )
            self.get_logger().info(
                f"[SIM] image={image_topic} depth={depth_topic} pc={pointcloud_topic}"
            )

        self.create_subscription(String, "/robot_state", self._state_cb, 10)
        self.marker_pub = self.create_publisher(Marker, "/ring_marker", MARKER_QOS)
        self.colour_pub = self.create_publisher(String, "/ring_colour", 10)
        self.debug_pub = self.create_publisher(Image, "/ring_debug_image", 10)

    def _device_name(self):
        return self.device if self.device else "cuda:0"

    @staticmethod
    def _normalise_model_names(names):
        if isinstance(names, dict):
            return {int(k): str(v).lower() for k, v in names.items()}
        return {idx: str(name).lower() for idx, name in enumerate(names)}

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}", throttle_duration_sec=5.0)
            return
        self._store_frame(frame, msg.header)

    def _compressed_image_cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as exc:
            self.get_logger().warn(f"Compressed RGB decode failed: {exc}", throttle_duration_sec=5.0)
            return
        self._store_frame(frame, msg.header)

    def _store_frame(self, frame, header):
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_header = header
        self._frame_event.set()

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}", throttle_duration_sec=5.0)
            return

        raw_dtype = depth.dtype
        depth = depth.astype(np.float32)
        if msg.encoding in ("16UC1", "mono16") or raw_dtype == np.uint16:
            depth /= 1000.0
        elif np.isfinite(depth).any() and np.nanmax(depth) > 20.0:
            depth /= 1000.0
        self.depth_m = depth

    def _pointcloud_cb(self, msg):
        self._latest_pc_msg = msg
        self._pc_dirty = True

    def _camera_info_cb(self, msg):
        if self.camera_intrinsics is not None:
            return
        self.camera_intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame_id = msg.header.frame_id.lstrip("/") or "base_link"
        self.get_logger().info(
            f"[REAL] camera intrinsics fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} "
            f"cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} frame={self.camera_frame_id}"
        )

    def _state_cb(self, msg):
        self.robot_state = msg.data

    def _worker_loop(self):
        while rclpy.ok():
            self._frame_event.wait()
            self._frame_event.clear()
            try:
                self._process_latest_frame()
            except Exception as exc:
                self.get_logger().error(f"Ring YOLO worker error: {exc}", throttle_duration_sec=5.0)

    def _process_latest_frame(self):
        now = time.monotonic()
        min_period = 1.0 / max(0.1, self.inference_hz)
        if now - self._last_inference_time < min_period:
            return
        self._last_inference_time = now

        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            header = self._latest_header

        if frame is None:
            return
        self._frame_seq += 1

        if self.robot_state not in ("IDLE", "PATROL"):
            self._publish_debug_status(frame, f"paused: robot_state={self.robot_state}")
            return

        if self.model is None:
            self.get_logger().warn(
                "Ring YOLO model is not loaded; no ring detections will run.",
                throttle_duration_sec=5.0,
            )
            self._publish_debug_status(frame, "ring YOLO model not loaded")
            return

        depth_snapshot = self.depth_m
        depth_ready = depth_snapshot is not None
        camera_ready = (not self.real_robot) or self.camera_intrinsics is not None

        debug_status = None
        if not depth_ready:
            self.get_logger().warn("Waiting for depth image", throttle_duration_sec=3.0)
            debug_status = "no depth: YOLO debug only"
        elif not camera_ready:
            self.get_logger().warn("Waiting for camera_info", throttle_duration_sec=3.0)
            debug_status = "no camera_info: YOLO debug only"

        if not self.real_robot:
            self._refresh_pointcloud()

        results = self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf,
            verbose=False,
            device=self._device_name(),
        )

        debug = frame.copy() if self.debug_image else None
        accepted = 0
        raw = 0
        depth_boxes = []

        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                raw += 1
                candidate = self._candidate_from_box(box, frame.shape)
                if candidate is None:
                    continue

                x1, y1, x2, y2, cls_id, conf = candidate
                colour = self._colour_for_detection(cls_id, frame, x1, y1, x2, y2)
                if self._handle_candidate(
                    frame,
                    debug,
                    depth_snapshot,
                    depth_ready,
                    camera_ready,
                    header,
                    depth_boxes,
                    x1,
                    y1,
                    x2,
                    y2,
                    colour,
                    conf,
                    "yolo",
                ):
                    accepted += 1

        fallback_raw = 0
        if self.enable_colour_fallback and accepted == 0:
            for x1, y1, x2, y2, colour, conf in self._colour_shape_candidates(frame):
                if self._overlaps_existing_box((x1, y1, x2, y2), depth_boxes):
                    continue
                fallback_raw += 1
                if self._handle_candidate(
                    frame,
                    debug,
                    depth_snapshot,
                    depth_ready,
                    camera_ready,
                    header,
                    depth_boxes,
                    x1,
                    y1,
                    x2,
                    y2,
                    colour,
                    conf,
                    "colour",
                ):
                    accepted += 1
                    break

        if raw or fallback_raw:
            self.get_logger().info(
                f"Ring YOLO raw={raw} colour_fallback={fallback_raw} "
                f"accepted={accepted}",
                throttle_duration_sec=1.0,
            )

        if self.debug_image and debug is not None:
            try:
                if debug_status is not None:
                    self._draw_status(debug, debug_status)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            except Exception:
                pass

        self._show_windows(debug, depth_snapshot, frame.shape, depth_boxes, debug_status)

    def _handle_candidate(
        self,
        frame,
        debug,
        depth_snapshot,
        depth_ready,
        camera_ready,
        header,
        depth_boxes,
        x1,
        y1,
        x2,
        y2,
        colour,
        conf,
        source,
    ):
        show_reject = source != "colour"
        if colour == "unknown":
            if show_reject:
                self._draw_debug(debug, x1, y1, x2, y2, "unknown", conf, accepted=False, source=source)
            return False

        if source == "yolo" and colour in ("red", "green", "blue", "black"):
            if self._colour_box_ring_score(frame, colour, x1, y1, x2, y2) is None:
                self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source="yolo-shape")
                depth_boxes.append((x1, y1, x2, y2, colour, False))
                return False

        if not depth_ready:
            if show_reject:
                self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source=source)
            return False

        if self.require_depth_hole:
            if not self._depth_looks_like_ring(depth_snapshot, frame.shape, x1, y1, x2, y2):
                if show_reject:
                    self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source=source)
                return False

        if not camera_ready:
            if show_reject:
                self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source=source)
            return False

        if self.real_robot:
            pos = self._real_3d_from_depth(depth_snapshot, frame.shape, x1, y1, x2, y2)
            marker_frame = self.camera_frame_id
        else:
            pos = self._sim_3d_from_pointcloud(depth_snapshot, frame.shape, x1, y1, x2, y2)
            marker_frame = self.pointcloud_frame_id

        if pos is None:
            if show_reject:
                self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source=source)
            return False

        dist = float(np.linalg.norm(pos))
        if dist > self.max_detection_dist:
            if show_reject:
                self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=False, source=source)
            return False

        if not self._candidate_is_confirmed((x1, y1, x2, y2), colour, source, frame.shape):
            return False

        self._publish_marker(pos, colour, marker_frame, header)
        self._draw_debug(debug, x1, y1, x2, y2, colour, conf, accepted=True, source=source)
        depth_boxes.append((x1, y1, x2, y2, colour, True))
        return True

    def _candidate_is_confirmed(self, box, colour, source, frame_shape):
        required_hits = (
            self.yolo_confirm_frames if source == "yolo" else self.colour_confirm_frames
        )
        if required_hits <= 1:
            return True

        self._prune_candidate_tracks()
        x1, y1, x2, y2 = box
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        side = max(1.0, max(x2 - x1, y2 - y1))
        frame_h, frame_w = frame_shape[:2]
        # Allow the ring centre to sweep up to 45% of the frame width between
        # frames — necessary when the robot is rotating and the ring drifts fast.
        max_jump = max(side * 1.5, min(frame_h, frame_w) * 0.45)

        best_track = None
        best_dist = None
        for track in self._candidate_tracks:
            if track["colour"] != colour:
                continue
            if track["last_frame"] == self._frame_seq:
                continue
            tx, ty = track["centre"]
            dist = float(np.hypot(cx - tx, cy - ty))
            if dist > max_jump:
                continue
            if best_dist is None or dist < best_dist:
                best_track = track
                best_dist = dist

        if best_track is None:
            self._candidate_tracks.append({
                "colour": colour,
                "centre": (cx, cy),
                "box": box,
                "hits": 1,
                "last_frame": self._frame_seq,
                "last_seen": time.monotonic(),
            })
            return False

        best_track["centre"] = (cx, cy)
        best_track["box"] = box
        best_track["hits"] += 1
        best_track["last_frame"] = self._frame_seq
        best_track["last_seen"] = time.monotonic()
        return best_track["hits"] >= required_hits

    def _prune_candidate_tracks(self):
        now = time.monotonic()
        self._candidate_tracks = [
            track for track in self._candidate_tracks
            if now - track["last_seen"] <= 2.0
        ]

    def _publish_debug_status(self, frame, status):
        if not self.debug_image and not self.display_windows:
            return
        debug = frame.copy()
        self._draw_status(debug, status)
        if self.debug_image:
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            except Exception:
                pass
        self._show_windows(debug, self.depth_m, frame.shape, [], status)

    @staticmethod
    def _draw_status(debug, status):
        cv2.rectangle(debug, (8, 8), (min(debug.shape[1] - 1, 720), 44), (0, 0, 0), -1)
        cv2.putText(
            debug,
            status,
            (16, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _show_windows(self, debug_img, depth_m, frame_shape, boxes, status=None):
        if not self.display_windows or debug_img is None:
            return
        try:
            cv2.imshow("Ring YOLO", debug_img)
            cv2.imshow("Disparity (rings)", self._make_disparity_view(
                depth_m, frame_shape, debug_img.shape, boxes, status))
            cv2.waitKey(1)
        except Exception:
            pass

    def _make_disparity_view(self, depth_m, frame_shape, display_shape, boxes, status=None):
        if depth_m is None:
            view = np.zeros(display_shape, dtype=np.uint8)
            self._draw_status(view, status or "waiting for depth image")
            return view

        depth_f = depth_m.astype(np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            disparity = np.where(depth_f > 0.1, 1.0 / depth_f, 0.0)
        disp_8u = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        view = cv2.cvtColor(disp_8u, cv2.COLOR_GRAY2BGR)

        depth_h, depth_w = depth_m.shape[:2]
        frame_h, frame_w = frame_shape[:2]
        for x1, y1, x2, y2, colour, accepted in boxes:
            dx1, dy1, dx2, dy2 = self._scale_box((x1, y1, x2, y2), (frame_h, frame_w), (depth_h, depth_w))
            marker_colour = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))
            if accepted:
                bgr = (
                    int(marker_colour[2] * 255),
                    int(marker_colour[1] * 255),
                    int(marker_colour[0] * 255),
                )
                thickness = 2
            else:
                bgr = (128, 128, 128)
                thickness = 1
            cv2.rectangle(view, (dx1, dy1), (dx2, dy2), bgr, thickness)

        if status is not None:
            self._draw_status(view, status)
        return view

    def _refresh_pointcloud(self):
        if not self._pc_dirty:
            return
        self._pc_dirty = False
        msg = self._latest_pc_msg
        if msg is None:
            return
        if msg.height <= 1:
            self.get_logger().warn(
                "Point cloud is not organized; cannot map image pixels to 3D.",
                throttle_duration_sec=5.0,
            )
            return
        try:
            pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
            if pts is not None and len(pts) > 0:
                self.pointcloud_xyz = pts.reshape((msg.height, msg.width, 3))
                self.pointcloud_frame_id = msg.header.frame_id.lstrip("/") or "base_link"
        except Exception as exc:
            self.get_logger().debug(f"Point cloud deserialize failed: {exc}")

    @staticmethod
    def _candidate_from_box(box, image_shape):
        h, w = image_shape[:2]
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
        x1 = int(np.clip(x1, 0, w - 1))
        x2 = int(np.clip(x2, 0, w - 1))
        y1 = int(np.clip(y1, 0, h - 1))
        y2 = int(np.clip(y2, 0, h - 1))
        bw = x2 - x1
        bh = y2 - y1
        if bw < 8 or bh < 8:
            return None
        if bw * bh < max(64, int(0.00015 * w * h)):
            return None
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        return x1, y1, x2, y2, cls_id, conf

    def _colour_shape_candidates(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        candidates = []

        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        for colour in ("red", "green", "blue"):
            mask = np.zeros((h, w), dtype=np.uint8)
            for lo, hi in HSV_RANGES[colour]:
                mask |= cv2.inRange(hsv, lo, hi)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)

            candidates.extend(self._colour_hough_candidates(mask, colour, frame.shape))
            # Contour fallback is useful only as a secondary path; most false
            # positives are rectangular objects, so it is gated very tightly.
            candidates.extend(self._colour_contour_candidates(mask, colour, frame.shape))

        return self._dedupe_colour_candidates(candidates)[:4]

    def _colour_hough_candidates(self, mask, colour, frame_shape):
        h, w = frame_shape[:2]
        min_radius = max(18, int(min(h, w) * 0.035))
        max_radius = max(min_radius + 1, int(min(h, w) * 0.16))

        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(28, int(min_radius * 2.0)),
            param1=80,
            param2=11,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return []

        candidates = []
        for cx, cy, radius in np.round(circles[0]).astype(int):
            score = self._ring_circle_score(mask, cx, cy, radius)
            if score is None:
                continue

            pad_radius = int(radius * 1.18)
            x1 = int(np.clip(cx - pad_radius, 0, w - 1))
            y1 = int(np.clip(cy - pad_radius, 0, h - 1))
            x2 = int(np.clip(cx + pad_radius, 0, w - 1))
            y2 = int(np.clip(cy + pad_radius, 0, h - 1))
            conf = float(np.clip(0.50 + score * 0.48, 0.50, 0.98))
            candidates.append((x1, y1, x2, y2, colour, conf))
        return candidates

    def _colour_contour_candidates(self, mask, colour, frame_shape):
        h, w = frame_shape[:2]
        min_area = max(350.0, 0.0009 * w * h)
        max_area = 0.12 * w * h
        max_side = int(min(h, w) * 0.34)
        candidates = []

        contour_mask = mask.copy()
        contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 36 or bh < 36:
                continue
            if bw > max_side or bh > max_side:
                continue

            aspect = bw / float(bh)
            if aspect < 0.55 or aspect > 1.85:
                continue

            score = self._ring_box_score(mask, x, y, bw, bh)
            if score is None:
                continue

            pad = max(4, int(0.08 * max(bw, bh)))
            x1 = int(np.clip(x - pad, 0, w - 1))
            y1 = int(np.clip(y - pad, 0, h - 1))
            x2 = int(np.clip(x + bw + pad, 0, w - 1))
            y2 = int(np.clip(y + bh + pad, 0, h - 1))
            conf = float(np.clip(0.48 + score * 0.45, 0.50, 0.95))
            candidates.append((x1, y1, x2, y2, colour, conf))
        return candidates

    def _colour_box_ring_score(self, frame, colour, x1, y1, x2, y2):
        if colour not in HSV_RANGES:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lo, hi in HSV_RANGES[colour]:
            mask |= cv2.inRange(hsv, lo, hi)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return self._ring_box_score(mask, x1, y1, max(1, x2 - x1), max(1, y2 - y1))

    def _ring_circle_score(self, mask, cx, cy, radius):
        h, w = mask.shape[:2]
        if radius < 8:
            return None
        if cx - radius < 0 or cy - radius < 0 or cx + radius >= w or cy + radius >= h:
            return None

        yy, xx = np.ogrid[:h, :w]
        dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
        annulus = (dist >= radius * 0.58) & (dist <= radius * 1.10)
        centre = dist <= radius * 0.42

        return self._score_ring_distribution(mask, annulus, centre, xx - cx, yy - cy)

    def _ring_box_score(self, mask, x, y, bw, bh):
        roi = mask[y:y + bh, x:x + bw]
        if roi.size == 0:
            return None

        yy, xx = np.ogrid[:bh, :bw]
        cx = (bw - 1) / 2.0
        cy = (bh - 1) / 2.0
        rx = max(1.0, bw * 0.50)
        ry = max(1.0, bh * 0.50)
        norm_r = np.sqrt(((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2)
        annulus = (norm_r >= 0.58) & (norm_r <= 1.08)
        centre = norm_r <= 0.42

        return self._score_ring_distribution(roi, annulus, centre, xx - cx, yy - cy)

    @staticmethod
    def _score_ring_distribution(mask, annulus, centre, dx, dy):
        colour_pixels = mask > 0
        annulus_total = max(1, int(np.count_nonzero(annulus)))
        centre_total = max(1, int(np.count_nonzero(centre)))
        annulus_frac = float(np.count_nonzero(colour_pixels & annulus)) / annulus_total
        centre_frac = float(np.count_nonzero(colour_pixels & centre)) / centre_total

        if annulus_frac < 0.18:
            return None
        if centre_frac > max(0.08, annulus_frac * 0.35):
            return None

        angles = (np.arctan2(dy, dx) + 2.0 * np.pi) % (2.0 * np.pi)
        sector_hits = 0
        sector_count = 16
        hit_sectors = []
        for sector in range(sector_count):
            lo = sector * 2.0 * np.pi / sector_count
            hi = (sector + 1) * 2.0 * np.pi / sector_count
            sector_mask = annulus & (angles >= lo) & (angles < hi)
            sector_total = int(np.count_nonzero(sector_mask))
            if sector_total == 0:
                continue
            sector_frac = float(np.count_nonzero(colour_pixels & sector_mask)) / sector_total
            if sector_frac >= 0.075:
                sector_hits += 1
                hit_sectors.append(sector)

        if sector_hits < 13:
            return None

        hit_set = set(hit_sectors)
        opposite_pairs = sum(
            1 for sector in range(sector_count // 2)
            if sector in hit_set and (sector + sector_count // 2) in hit_set
        )
        if opposite_pairs < 6:
            return None

        hole_score = max(0.0, 1.0 - centre_frac / max(annulus_frac, 0.001))
        coverage_score = sector_hits / float(sector_count)
        return annulus_frac * 0.45 + coverage_score * 0.40 + hole_score * 0.15

    def _dedupe_colour_candidates(self, candidates):
        candidates.sort(
            key=lambda item: (
                item[5],
                min(item[2] - item[0], item[3] - item[1]),
            ),
            reverse=True,
        )

        kept = []
        for candidate in candidates:
            box = candidate[:4]
            if any(self._box_iou(box, existing[:4]) > 0.35 for existing in kept):
                continue
            kept.append(candidate)
        return kept

    @staticmethod
    def _overlaps_existing_box(box, existing_boxes):
        for x1, y1, x2, y2, _colour, _accepted in existing_boxes:
            if RingDetectorYOLO._box_iou(box, (x1, y1, x2, y2)) > 0.25:
                return True
        return False

    @staticmethod
    def _box_iou(a, b):
        ax1, ay1, ax2, ay2 = a
        bx1, by1, bx2, by2 = b
        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
        area_b = max(1, (bx2 - bx1) * (by2 - by1))
        return inter / float(area_a + area_b - inter)

    def _colour_for_detection(self, cls_id, frame, x1, y1, x2, y2):
        name = self.model_names.get(cls_id, "")
        colour = self._canonical_colour(name)
        if colour != "unknown":
            return colour

        if 0 <= cls_id < len(FALLBACK_CLASS_COLOURS) and len(self.model_names) == 4:
            return FALLBACK_CLASS_COLOURS[cls_id]

        if "ring" in name or len(self.model_names) == 1:
            return self._classify_colour_from_roi(frame[y1:y2 + 1, x1:x2 + 1])

        return "unknown"

    @staticmethod
    def _canonical_colour(name):
        compact = name.lower().replace("-", "_")
        for colour in MARKER_COLOURS:
            if colour in compact:
                return colour
        return "unknown"

    def _classify_colour_from_roi(self, roi):
        if roi.size == 0:
            return "unknown"

        mask = self._annulus_mask(roi.shape[:2])
        total = int(np.count_nonzero(mask))
        if total < 20:
            return "unknown"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        best_colour = "unknown"
        best_count = 0

        for colour, ranges in HSV_RANGES.items():
            colour_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                colour_mask |= cv2.inRange(hsv, lo, hi)
            count = int(np.count_nonzero((colour_mask > 0) & (mask > 0)))
            if count > best_count:
                best_count = count
                best_colour = colour

        required_frac = 0.14 if best_colour == "black" else 0.06
        if best_count >= max(12, int(total * required_frac)):
            return best_colour
        return "unknown"

    @staticmethod
    def _annulus_mask(shape):
        h, w = shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cx = w // 2
        cy = h // 2
        rx = max(2, int(w * 0.48))
        ry = max(2, int(h * 0.48))
        inner_rx = max(1, int(rx * 0.45))
        inner_ry = max(1, int(ry * 0.45))
        cv2.ellipse(mask, (cx, cy), (rx, ry), 0, 0, 360, 255, -1)
        cv2.ellipse(mask, (cx, cy), (inner_rx, inner_ry), 0, 0, 360, 0, -1)
        return mask

    def _depth_looks_like_ring(self, depth_m, frame_shape, x1, y1, x2, y2):
        dx1, dy1, dx2, dy2 = self._scale_box((x1, y1, x2, y2), frame_shape, depth_m.shape)
        if dx2 <= dx1 or dy2 <= dy1:
            return False

        patch = depth_m[dy1:dy2 + 1, dx1:dx2 + 1]
        if patch.size == 0:
            return False

        h, w = patch.shape[:2]
        annulus = self._annulus_mask((h, w))
        centre = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(
            centre,
            (w // 2, h // 2),
            (max(1, int(w * 0.18)), max(1, int(h * 0.18))),
            0,
            0,
            360,
            255,
            -1,
        )

        rim_values = patch[(annulus > 0) & np.isfinite(patch) & (patch > 0.1)]
        if len(rim_values) < 5:
            return False
        rim_median = float(np.median(rim_values))

        centre_values = patch[centre > 0]
        if centre_values.size == 0:
            return False
        invalid_frac = float(np.mean((centre_values <= 0.1) | ~np.isfinite(centre_values)))
        valid_centre = centre_values[np.isfinite(centre_values) & (centre_values > 0.1)]
        centre_median = float(np.median(valid_centre)) if len(valid_centre) else 99.0

        return invalid_frac >= 0.18 or centre_median >= rim_median + 0.04

    @staticmethod
    def _scale_box(box, from_shape, to_shape):
        x1, y1, x2, y2 = box
        from_h, from_w = from_shape[:2]
        to_h, to_w = to_shape[:2]
        sx = to_w / float(from_w)
        sy = to_h / float(from_h)
        return (
            int(np.clip(round(x1 * sx), 0, to_w - 1)),
            int(np.clip(round(y1 * sy), 0, to_h - 1)),
            int(np.clip(round(x2 * sx), 0, to_w - 1)),
            int(np.clip(round(y2 * sy), 0, to_h - 1)),
        )

    def _real_3d_from_depth(self, depth_m, frame_shape, x1, y1, x2, y2):
        if self.camera_intrinsics is None:
            return None

        dx1, dy1, dx2, dy2 = self._scale_box((x1, y1, x2, y2), frame_shape, depth_m.shape)
        patch = depth_m[dy1:dy2 + 1, dx1:dx2 + 1]
        if patch.size == 0:
            return None

        h, w = patch.shape[:2]
        annulus = self._annulus_mask((h, w))
        ys, xs = np.where((annulus > 0) & np.isfinite(patch) & (patch > 0.1))
        if len(xs) < 5:
            ys, xs = np.where(np.isfinite(patch) & (patch > 0.1))
        if len(xs) < 5:
            return None

        zs = patch[ys, xs]
        xs = xs + dx1
        ys = ys + dy1

        depth_h, depth_w = depth_m.shape[:2]
        frame_h, frame_w = frame_shape[:2]
        fx, fy, ppx, ppy = self.camera_intrinsics
        fx *= depth_w / float(frame_w)
        fy *= depth_h / float(frame_h)
        ppx *= depth_w / float(frame_w)
        ppy *= depth_h / float(frame_h)

        x3 = (xs - ppx) * zs / fx
        y3 = (ys - ppy) * zs / fy
        return np.array([
            float(np.median(x3)),
            float(np.median(y3)),
            float(np.median(zs)),
        ])

    def _sim_3d_from_pointcloud(self, depth_m, frame_shape, x1, y1, x2, y2):
        pc = self.pointcloud_xyz
        if pc is None:
            return None

        dx1, dy1, dx2, dy2 = self._scale_box((x1, y1, x2, y2), frame_shape, depth_m.shape)
        cx = (dx1 + dx2) // 2
        cy = (dy1 + dy2) // 2
        rx = max(2, (dx2 - dx1) // 2)
        ry = max(2, (dy2 - dy1) // 2)

        pc_h, pc_w = pc.shape[:2]
        depth_h, depth_w = depth_m.shape[:2]
        points = []
        for angle in np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False):
            px = int(np.clip(cx + rx * 0.75 * np.cos(angle), 0, depth_w - 1))
            py = int(np.clip(cy + ry * 0.75 * np.sin(angle), 0, depth_h - 1))
            pc_x = int(np.clip(px * pc_w / depth_w, 0, pc_w - 1))
            pc_y = int(np.clip(py * pc_h / depth_h, 0, pc_h - 1))
            point = pc[pc_y, pc_x]
            if np.all(np.isfinite(point)) and np.any(point != 0):
                points.append(point)

        if len(points) < 3:
            return None
        arr = np.array(points)
        return np.array([
            float(np.median(arr[:, 0])),
            float(np.median(arr[:, 1])),
            float(np.median(arr[:, 2])),
        ])

    def _publish_marker(self, position, colour, frame_id, header):
        r, g, b = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = header.stamp if header is not None else self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.id = self.marker_id
        self.marker_id += 1
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=200_000_000)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.15
        marker.scale.z = 0.05
        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        colour_msg = String()
        colour_msg.data = colour
        self.colour_pub.publish(colour_msg)

    def _draw_debug(self, image, x1, y1, x2, y2, colour, conf, accepted, source=""):
        if image is None:
            return
        marker_colour = MARKER_COLOURS.get(colour, (0.5, 0.5, 0.5))
        if accepted:
            bgr = (
                int(marker_colour[2] * 255),
                int(marker_colour[1] * 255),
                int(marker_colour[0] * 255),
            )
            thickness = 2
        else:
            bgr = (128, 128, 128)
            thickness = 1
        cv2.rectangle(image, (x1, y1), (x2, y2), bgr, thickness)
        label_name = f"{source} {colour}".strip()
        label = f"{label_name} {conf:.2f}" if accepted else f"reject {label_name} {conf:.2f}"
        cv2.putText(
            image,
            label,
            (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            bgr,
            1,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = RingDetectorYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
