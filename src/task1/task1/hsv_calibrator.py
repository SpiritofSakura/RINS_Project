#!/usr/bin/env python3
"""
Interactive HSV calibration tool for ring colour detection.

Subscribes to the real robot camera, overlays a tinted mask for the
currently-selected colour, and lets you tune HSV bounds live with
trackbars.  On save it writes ~/.ros/ring_hsv_calibration.yaml which
detect_rings_v2 loads automatically at startup.

Keys (focus the "Live + Mask" window first):
  r / g / b / k  — switch active colour (Red / Green / Blue / blacK)
  s              — save calibration  (~/.ros/ring_hsv_calibration.yaml)
  d              — reset current colour to built-in defaults
  q              — quit

Mouse left-click on "Live + Mask" — print the HSV value at that pixel
(handy for centering the trackbar ranges on a real ring).
"""

import copy
import os

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

CALIB_PATH = os.path.expanduser("~/.ros/ring_hsv_calibration.yaml")

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Built-in defaults (mirrors COLOUR_RANGES_REAL in detect_rings_v2)
DEFAULTS = {
    "red": [
        {"lo": [0,   60,  40], "hi": [10,  255, 255]},
        {"lo": [165, 60,  40], "hi": [180, 255, 255]},
    ],
    "green": [
        {"lo": [35,  50,  30], "hi": [90,  255, 255]},
    ],
    "blue": [
        {"lo": [95,  50,  30], "hi": [145, 255, 255]},
    ],
    "black": [
        {"lo": [0,   0,   0],  "hi": [180, 255,  60]},
    ],
}

COLOUR_KEYS  = {ord('r'): 'red', ord('g'): 'green',
                ord('b'): 'blue', ord('k'): 'black'}
TINT_COLOURS = {
    'red':   (0,   0,   255),
    'green': (0,   255, 0),
    'blue':  (255, 100, 0),
    'black': (150, 150, 150),
}

WIN_LIVE = "Live + Mask"
WIN_TB   = "Trackbars"


class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')

        self.declare_parameter('image_topic',    '/gemini/color/image_raw/compressed')
        self.declare_parameter('use_compressed', True)

        topic      = self.get_parameter('image_topic').get_parameter_value().string_value
        compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value

        self.ranges = self._load_or_default()
        self.current = 'red'
        self._frame  = None
        self._hsv    = None
        self.bridge  = CvBridge()

        if compressed:
            self.sub = self.create_subscription(
                CompressedImage, topic, self._compressed_cb, SENSOR_QOS)
        else:
            self.sub = self.create_subscription(
                Image, topic, self._raw_cb, SENSOR_QOS)

        self._setup_windows()
        self.create_timer(0.033, self._tick)

        self.get_logger().info(
            f"HSV Calibrator ready on {topic}\n"
            f"  r/g/b/k = switch colour | s = save | d = reset colour | q = quit\n"
            f"  Left-click on image → print HSV at pixel\n"
            f"  Calibration path: {CALIB_PATH}"
        )

    # ── Loading / saving ───────────────────────────────────────────────────────

    def _load_or_default(self):
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH) as f:
                    data = yaml.safe_load(f)
                self.get_logger().info(f"Loaded existing calibration from {CALIB_PATH}")
                return data
            except Exception as e:
                self.get_logger().warn(f"Could not read {CALIB_PATH}: {e} — using defaults")
        return copy.deepcopy(DEFAULTS)

    def _save(self):
        os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
        with open(CALIB_PATH, 'w') as f:
            yaml.dump(self.ranges, f, default_flow_style=False)
        self.get_logger().info(f"Saved → {CALIB_PATH}")
        print(f"\n{'='*50}")
        print(f"Saved calibration to {CALIB_PATH}")
        print(yaml.dump(self.ranges, default_flow_style=False))

    # ── Window / trackbar setup ────────────────────────────────────────────────

    def _setup_windows(self):
        cv2.namedWindow(WIN_LIVE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_LIVE, 800, 500)
        cv2.setMouseCallback(WIN_LIVE, self._mouse_cb)
        self._rebuild_trackbars()

    def _rebuild_trackbars(self):
        try:
            cv2.destroyWindow(WIN_TB)
        except Exception:
            pass
        cv2.namedWindow(WIN_TB, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_TB, 500, 500)

        col    = self.current
        ranges = self.ranges[col]
        r0     = ranges[0]

        def tb(name, val, maxv, cb):
            cv2.createTrackbar(name, WIN_TB, int(np.clip(val, 0, maxv)), maxv, cb)

        tb("R0  H_lo",  r0["lo"][0], 180, lambda v: self._set(col, 0, "lo", 0, v))
        tb("R0  H_hi",  r0["hi"][0], 180, lambda v: self._set(col, 0, "hi", 0, v))
        tb("R0  S_lo",  r0["lo"][1], 255, lambda v: self._set(col, 0, "lo", 1, v))
        tb("R0  S_hi",  r0["hi"][1], 255, lambda v: self._set(col, 0, "hi", 1, v))
        tb("R0  V_lo",  r0["lo"][2], 255, lambda v: self._set(col, 0, "lo", 2, v))
        tb("R0  V_hi",  r0["hi"][2], 255, lambda v: self._set(col, 0, "hi", 2, v))

        # Red has a second hue range (wraps around 0/180)
        if col == "red" and len(ranges) > 1:
            r1 = ranges[1]
            tb("R1  H_lo", r1["lo"][0], 180, lambda v: self._set(col, 1, "lo", 0, v))
            tb("R1  H_hi", r1["hi"][0], 180, lambda v: self._set(col, 1, "hi", 0, v))

    def _set(self, colour, idx, lohi, ch, val):
        self.ranges[colour][idx][lohi][ch] = val

    # ── Image callbacks ────────────────────────────────────────────────────────

    def _compressed_cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self._frame = frame
            self._hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def _raw_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._frame = frame
            self._hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        except CvBridgeError:
            pass

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self._hsv is not None:
            h_val, s_val, v_val = self._hsv[y, x]
            print(f"[click ({x},{y})]  H={h_val}  S={s_val}  V={v_val}"
                  f"  — colour being tuned: {self.current}")

    # ── Main display loop ──────────────────────────────────────────────────────

    def _tick(self):
        if self._frame is None:
            return

        col    = self.current
        ranges = self.ranges[col]

        # Build combined mask for this colour
        mask = np.zeros(self._frame.shape[:2], dtype=np.uint8)
        for r in ranges:
            lo = np.array(r["lo"])
            hi = np.array(r["hi"])
            mask |= cv2.inRange(self._hsv, lo, hi)

        # Overlay tint on matched pixels
        overlay = self._frame.copy()
        overlay[mask > 0] = TINT_COLOURS.get(col, (255, 255, 255))
        vis = cv2.addWeighted(self._frame, 0.55, overlay, 0.45, 0)

        pct = 100.0 * float(mask.sum() // 255) / max(mask.size, 1)
        cv2.putText(vis, f"Active: {col.upper()}  [r/g/b/k | s=save | d=reset | q=quit]",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(vis, f"Masked pixels: {pct:.1f}%  (click to print HSV)",
                    (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 220), 2)

        cv2.imshow(WIN_LIVE, vis)

        key = cv2.waitKey(1) & 0xFF
        if key in COLOUR_KEYS:
            self.current = COLOUR_KEYS[key]
            self._rebuild_trackbars()
            self.get_logger().info(f"Switched to: {self.current}")
        elif key == ord('s'):
            self._save()
        elif key == ord('d'):
            self.ranges[col] = copy.deepcopy(DEFAULTS[col])
            self._rebuild_trackbars()
            self.get_logger().info(f"Reset {col} to defaults")
        elif key == ord('q'):
            self.get_logger().info("Quit.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()
    rclpy.spin(node)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
