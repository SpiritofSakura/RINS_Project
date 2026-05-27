#!/usr/bin/env python3
import json
import os
import random
import shutil
import subprocess

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

LEAK_DIR = os.path.expanduser("~/RINS_Project/reports/img/barrels")

BARREL_LINES = [
    "I found a {color} {orientation} barrel. {leak_sentence}",
    "Barrel inspection complete. The barrel is {color} and {orientation}. {leak_sentence}",
    "This is a {color} {orientation} barrel. {leak_sentence}",
    "A {color} barrel has been detected. It is {orientation}. {leak_sentence}",
]


class BarrelInspector(Node):
    def __init__(self):
        super().__init__("barrel_inspector")

        self.bridge = CvBridge()
        self._latest_oakd = None
        self._leak_image_seq = 0
        self._seen_barrel_ids = set()
        self._latest_barrel = None
        self._latest_horizontal_barrel = None
        self._barrels_by_id = {}

        self.cylinder_sub = self.create_subscription(
            Marker, "/detected_cylinder_locations", self._cylinder_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            String, "/spill_check_trigger", self._trigger_callback, 10
        )
        self.oakd_sub = self.create_subscription(
            Image, "/oakd/rgb/preview/image_raw", self._oakd_callback, 10
        )

        self.spill_check_client = self.create_client(Trigger, "/spill_check")
        self.result_pub = self.create_publisher(String, "/barrel_inspection_result", 10)

        self._pending_spill_future = None
        self._pending_barrel = None

        self.get_logger().info("BarrelInspector ready")

    def _oakd_callback(self, msg: Image):
        try:
            self._latest_oakd = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _marker_color_name(self, marker: Marker):
        r, g, b = round(marker.color.r, 2), round(marker.color.g, 2), round(marker.color.b, 2)
        options = {
            (1.0, 0.0, 0.0): "red",
            (0.0, 1.0, 0.0): "green",
            (0.0, 0.4, 1.0): "blue",
            (1.0, 1.0, 0.0): "yellow",
            (1.0, 0.5, 0.0): "orange",
            (0.6, 0.0, 1.0): "purple",
            (0.45, 0.22, 0.08): "brown",
            (0.1, 0.1, 0.1): "black",
        }
        best, best_dist = "unknown", float("inf")
        for (cr, cg, cb), name in options.items():
            d = abs(r - cr) + abs(g - cg) + abs(b - cb)
            if d < best_dist:
                best_dist = d
                best = name
        return best

    def _cylinder_callback(self, msg: Marker):
        if msg.ns not in ("barrel_confirmed", "cylinder_confirmed"):
            return
        if msg.id in self._seen_barrel_ids:
            return
        self._seen_barrel_ids.add(msg.id)

        color = self._marker_color_name(msg)
        orientation = msg.text if msg.text in ("horizontal", "vertical") else "vertical"

        self._latest_barrel = {
            "barrel_id": msg.id,
            "color": color,
            "orientation": orientation,
        }
        self._barrels_by_id[msg.id] = self._latest_barrel

        if orientation == "vertical":
            self.get_logger().info(
                f"Vertical barrel id={msg.id} color={color} — leak defaults to no"
            )
            self._publish_result(msg.id, False)
            return

        self._latest_horizontal_barrel = self._latest_barrel
        self.get_logger().info(
            f"New horizontal barrel id={msg.id} color={color} — publishing null placeholder until approach"
        )
        self._publish_result(msg.id, None)

    def _trigger_callback(self, msg: String):
        parts = msg.data.strip().lower().split()
        if not parts or parts[0] != "check":
            self.get_logger().warn(f"Unknown trigger command: {msg.data.strip()}")
            return

        barrel = None
        if len(parts) >= 2:
            try:
                barrel = self._barrels_by_id.get(int(parts[1]))
            except ValueError:
                self.get_logger().warn(f"Invalid barrel id in trigger: {parts[1]}")
                return
        else:
            barrel = self._latest_horizontal_barrel

        if barrel is None:
            self.get_logger().warn("No matching horizontal barrel known yet — trigger ignored")
            return
        if barrel.get("orientation") != "horizontal":
            self.get_logger().warn(
                f"Ignoring spill check for vertical barrel id={barrel['barrel_id']}"
            )
            return

        self.get_logger().info(
            f"Spill check triggered for approached barrel id={barrel['barrel_id']}"
        )
        self._run_spill_check(barrel)

    def _run_spill_check(self, barrel):
        if not self.spill_check_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Spill check service not available")
            self._publish_result(barrel["barrel_id"], None)
            self._speak_barrel(barrel["color"], barrel["orientation"], None)
            return
        self._pending_barrel = barrel
        req = Trigger.Request()
        self._pending_spill_future = self.spill_check_client.call_async(req)

    def _check_spill_result(self):
        if self._pending_spill_future is None:
            return
        if not self._pending_spill_future.done():
            return

        future = self._pending_spill_future
        barrel = self._pending_barrel
        self._pending_spill_future = None
        self._pending_barrel = None

        leak_detected = None
        point_count = None
        try:
            response = future.result()
            if response.success:
                data = json.loads(response.message)
                leak_detected = data.get("spill_detected")
                point_count = data.get("point_count", 0)
                self.get_logger().info(
                    f"Spill check: {point_count} pts → "
                    f"{'SPILL' if leak_detected else 'OK'}"
                )
            else:
                err = json.loads(response.message).get("error", "unknown")
                self.get_logger().warn(f"Spill check failed: {err}")
        except Exception as e:
            self.get_logger().warn(f"Spill check error: {e}")

        if barrel is not None:
            self._publish_result(barrel["barrel_id"], leak_detected, point_count)
            self._speak_barrel(barrel["color"], barrel["orientation"], leak_detected)
            if leak_detected:
                self._save_leak_image()

    def _publish_result(self, barrel_id, leak_detected, point_count=None):
        msg = String()
        msg.data = json.dumps({
            "barrel_id": barrel_id,
            "leak_detected": leak_detected,
            "point_count": point_count,
        })
        self.result_pub.publish(msg)

    def _speak_barrel(self, color, orientation, leak_detected):
        if leak_detected is True:
            leak_sentence = "Warning. It is leaking."
        elif leak_detected is False:
            leak_sentence = "No leak detected."
        else:
            leak_sentence = "Leak inspection was inconclusive."
        template = random.choice(BARREL_LINES)
        text = template.format(
            color=color,
            orientation=orientation,
            leak_sentence=leak_sentence,
        )
        self._speak(text)

    @staticmethod
    def _speak(text):
        commands = []
        if shutil.which("espeak-ng") is not None:
            commands.append(["espeak-ng", "-a", "180", "-s", "145", text])
        if shutil.which("espeak") is not None:
            commands.append(["espeak", "-a", "180", "-s", "145", text])
        if shutil.which("spd-say") is not None:
            commands.append(["spd-say", text])
        for cmd in commands:
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return
            except Exception:
                continue

    def _save_leak_image(self):
        if self._latest_oakd is None:
            self.get_logger().warn("No OAK-D image available for leak capture")
            return
        os.makedirs(LEAK_DIR, exist_ok=True)
        filename = f"leak_{self._leak_image_seq:04d}.jpg"
        self._leak_image_seq += 1
        path = os.path.join(LEAK_DIR, filename)
        cv2.imwrite(path, self._latest_oakd)
        self.get_logger().info(f"Saved leak image: {path}")

    def main_loop(self):
        self._check_spill_result()


def main():
    rclpy.init()
    node = BarrelInspector()
    node.create_timer(0.2, node.main_loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
