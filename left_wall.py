import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LeftWall(Node):
    def __init__(self):
        super().__init__("left_wall")
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, 1)
        self.get_logger().info("Measuring distance to left wall...")

    def cb(self, msg):
        idx = int(len(msg.ranges) / 4)
        d = msg.ranges[idx]
        if d < msg.range_min or d > msg.range_max:
            self.get_logger().info(f"Left wall: invalid ({d:.3f})")
        else:
            self.get_logger().info(f"Left wall: {d:.3f} m")


def main():
    rclpy.init()
    n = LeftWall()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
