import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class Scan4(Node):
    def __init__(self):
        super().__init__("scan4")
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, 1)

    def cb(self, m):
        def r(deg):
            ang = math.radians(deg)
            idx = int((ang - m.angle_min) / m.angle_increment)
            idx = max(0, min(idx, len(m.ranges) - 1))
            d = m.ranges[idx]
            return f"{d:.3f}m" if m.range_min <= d <= m.range_max else "inv"
        print(f"  0° (fwd): {r(0)}  90° (left): {r(90)}  180° (back): {r(180)}  270° (right): {r(270)}", end="\r")


def main():
    rclpy.init()
    n = Scan4()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
