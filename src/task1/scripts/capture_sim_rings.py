#!/usr/bin/env python3
"""
Capture real sim ring frames for YOLO fine-tuning.

Drive the robot near rings — this node uses the old Hough detector to
auto-label frames and saves them into ~/ring_dataset/ ready for retraining.

Run alongside the sim:
    python3 capture_sim_rings.py
Then retrain with train_ring_yolo.py.
"""

import os
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

SAVE_DIR  = Path.home() / "ring_dataset"
MAX_SAVES = 500   # cap so we don't flood disk
MIN_RADIUS = 10
MAX_RADIUS = 100
HOUGH_THRESH = 20

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

CLASSES_BGR = {
    0: [(0,0,100), (50,50,255)],    # red
    1: [(0,100,0), (50,255,80)],    # green
    2: [(100,30,0), (255,120,50)],  # blue
    3: [(0,0,0),   (60,60,60)],     # black
}


def classify_colour(patch):
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    h = int(np.median(hsv[:,:,0]))
    s = int(np.median(hsv[:,:,1]))
    v = int(np.median(hsv[:,:,2]))
    if v < 60:
        return 3  # black
    if s < 60:
        return -1  # grey/unknown
    if h < 12 or h > 168:
        return 0  # red
    if 35 < h < 90:
        return 1  # green
    if 95 < h < 140:
        return 2  # blue
    return -1


class RingCapture(Node):
    def __init__(self):
        super().__init__('ring_capture')
        self.bridge = CvBridge()
        self.depth_raw = None
        self.saved = 0

        (SAVE_DIR / "images" / "train").mkdir(parents=True, exist_ok=True)
        (SAVE_DIR / "labels" / "train").mkdir(parents=True, exist_ok=True)

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw',
                                  self._rgb_cb, SENSOR_QOS)
        self.create_subscription(Image, '/oakd/rgb/preview/depth',
                                  self._depth_cb, SENSOR_QOS)
        self.get_logger().info(f"Capturing sim rings → {SAVE_DIR}  (max {MAX_SAVES})")

    def _depth_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if img.dtype in (np.float32, np.float64):
                self.depth_raw = (img * 1000).astype(np.uint16)
            else:
                self.depth_raw = img.astype(np.uint16)
        except CvBridgeError:
            pass

    def _rgb_cb(self, msg):
        if self.saved >= MAX_SAVES or self.depth_raw is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        h, w = frame.shape[:2]
        depth_m = self.depth_raw.astype(np.float32) / 1000.0
        disp = np.where(depth_m > 0, 1.0 / np.clip(depth_m, 0.1, 10), 0)
        disp_8u = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        disp_8u = cv2.GaussianBlur(disp_8u, (5, 5), 0)

        circles = cv2.HoughCircles(
            disp_8u, cv2.HOUGH_GRADIENT, dp=1,
            minDist=60, param1=48, param2=HOUGH_THRESH,
            minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS,
        )
        if circles is None:
            return

        annotations = []
        for cx, cy, r in circles[0]:
            cx, cy, r = int(cx), int(cy), int(r)
            x1 = max(0, cx - r); y1 = max(0, cy - r)
            x2 = min(w, cx + r); y2 = min(h, cy + r)
            patch = frame[y1:y2, x1:x2]
            if patch.size == 0:
                continue
            cls = classify_colour(patch)
            if cls < 0:
                continue
            bx = cx / w; by = cy / h
            bw = (r * 2) / w; bh = (r * 2) / h
            annotations.append(f"{cls} {bx:.6f} {by:.6f} {bw:.6f} {bh:.6f}")

        if not annotations:
            return

        stem = f"sim_{self.saved:05d}"
        cv2.imwrite(str(SAVE_DIR / "images" / "train" / f"{stem}.jpg"), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(SAVE_DIR / "labels" / "train" / f"{stem}.txt", "w") as f:
            f.write("\n".join(annotations))
        self.saved += 1
        if self.saved % 50 == 0:
            self.get_logger().info(f"Saved {self.saved}/{MAX_SAVES} frames")


def main():
    rclpy.init()
    node = RingCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
