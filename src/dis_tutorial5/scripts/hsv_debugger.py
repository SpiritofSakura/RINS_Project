#!/usr/bin/python3
"""
HSV color debugger for ring detection.
Shows what HSV values the blue ring actually has and where it fails the pipeline.
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

class HSVDebugger(Node):
    def __init__(self):
        super().__init__('hsv_debugger')
        self.bridge = CvBridge()
        self.depth_raw = None
        
        self.image_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self.image_callback, SENSOR_QOS)
        self.depth_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/depth", self.depth_callback, SENSOR_QOS)
        
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Blue HSV range (current)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Blue - relaxed (80 saturaton)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Blue - very relaxed (50 saturation)", cv2.WINDOW_NORMAL)
        cv2.namedWindow("All blue candidate contours", cv2.WINDOW_NORMAL)
        
        self.get_logger().info("HSV Debugger ready. Press 'q' in window to exit.")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w = cv_image.shape[:2]
        
        # Current blue range (from detect_rings.py)
        lo1 = np.array([100, 80, 50])
        hi1 = np.array([140, 255, 255])
        mask_current = cv2.inRange(hsv, lo1, hi1)
        
        # Relaxed: lower saturation threshold to 50
        lo2 = np.array([100, 50, 50])
        hi2 = np.array([140, 255, 255])
        mask_relaxed = cv2.inRange(hsv, lo2, hi2)
        
        # Very relaxed: saturation 30
        lo3 = np.array([100, 30, 50])
        hi3 = np.array([140, 255, 255])
        mask_very_relaxed = cv2.inRange(hsv, lo3, hi3)
        
        # Show masks
        cv2.imshow("Original", cv_image)
        cv2.imshow("Blue HSV range (current)", mask_current)
        cv2.imshow("Blue - relaxed (80 saturaton)", mask_relaxed)
        cv2.imshow("Blue - very relaxed (50 saturation)", mask_very_relaxed)
        
        # Clean up the very_relaxed mask and find contours
        mask_clean = cv2.morphologyEx(
            mask_very_relaxed, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        mask_clean = cv2.morphologyEx(
            mask_clean, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
        
        contours, _ = cv2.findContours(
            mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw all contours
        debug_img = cv_image.copy()
        cv2.drawContours(debug_img, contours, -1, (0, 255, 255), 2)
        
        # Log stats for each contour
        self.get_logger().info(f"Found {len(contours)} blue candidate contours")
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if len(cnt) >= 15:
                ellipse = cv2.fitEllipse(cnt)
                (ex, ey), (minor_d, major_d), angle = ellipse
                aspect = minor_d / major_d if major_d > 0 else 0
                
                # Sample center color in HSV
                cy, cx = int(ey), int(ex)
                if 0 <= cy < h and 0 <= cx < w:
                    h_val, s_val, v_val = hsv[cy, cx]
                    self.get_logger().info(
                        f"  Contour {i}: area={area:.0f}, aspect={aspect:.2f}, "
                        f"center HSV=({h_val},{s_val},{v_val})"
                    )
        
        cv2.imshow("All blue candidate contours", debug_img)
        cv2.waitKey(1)

    def depth_callback(self, data):
        try:
            self.depth_raw = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError:
            return

if __name__ == '__main__':
    rclpy.init()
    node = HSVDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()
