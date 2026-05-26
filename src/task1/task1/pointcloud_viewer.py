#!/usr/bin/env python3
"""Analyse a thin Z-slice above ground: distance + colour check + RViz output."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros


DIST_MAX     = 1.0    # m — ignore points further than this from camera
SLICE_Z_MIN  = 0.005   # m above ground (map frame)
SLICE_Z_MAX  = 0.03    # m above ground (map frame)
GRAY_THRESH  = 30     # max(r,g,b) - min(r,g,b) below this = gray


class PointCloudViewer(Node):
    def __init__(self):
        super().__init__("pointcloud_viewer")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_cloud = None

        self.sub = self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points",
            self._cloud_cb, 1
        )
        self.slice_pub = self.create_publisher(PointCloud2, "/slice_points", 10)
        self.create_timer(1.0, self._timer_cb)

        self.get_logger().info("PointCloud viewer ready — view /slice_points in RViz.")

    def _cloud_cb(self, msg: PointCloud2):
        self.latest_cloud = msg

    def _timer_cb(self):
        if self.latest_cloud is None:
            return
        cloud = self.latest_cloud

        # --- 1. read raw points (sensor frame) ---
        try:
            structured = pc2.read_points(
                cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True
            )
        except Exception:
            return
        if len(structured) == 0:
            return

        # --- 2. distance filter (sensor frame) ---
        xs = structured["x"]
        ys = structured["y"]
        zs = structured["z"]
        dists = np.sqrt(xs**2 + ys**2 + zs**2)
        near_mask = dists <= DIST_MAX
        if near_mask.sum() == 0:
            return

        xs_near = xs[near_mask].astype(np.float64)
        ys_near = ys[near_mask].astype(np.float64)
        zs_near = zs[near_mask].astype(np.float64)
        rgb_near = np.asarray(structured["rgb"][near_mask], dtype=np.uint32)

        # --- 3. transform near points to map frame ---
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", cloud.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)

        pts = np.column_stack([xs_near, ys_near, zs_near])
        rotated = np.dot(pts, np.array(R).T)
        transformed = rotated + np.array([t.x, t.y, t.z])

        # --- 4. Z-slice filter (map frame) ---
        z = transformed[:, 2]
        slice_mask = (z >= SLICE_Z_MIN) & (z <= SLICE_Z_MAX)
        slice_count = slice_mask.sum()

        if slice_count == 0:
            self.get_logger().info(
                f"Slice [{SLICE_Z_MIN:.2f}, {SLICE_Z_MAX:.2f}]m @ ≤{DIST_MAX}m: 0 pts"
            )
            return

        # --- 5. colour analysis ---
        slice_rgb = np.array(rgb_near[slice_mask], dtype=np.uint32)
        r = ((slice_rgb >> 16) & 0xFF).astype(np.uint8)
        g = ((slice_rgb >> 8) & 0xFF).astype(np.uint8)
        b = (slice_rgb & 0xFF).astype(np.uint8)

        rg = r.astype(np.int16) - g.astype(np.int16)
        rb = r.astype(np.int16) - b.astype(np.int16)
        gb = g.astype(np.int16) - b.astype(np.int16)
        max_diff = np.maximum(np.abs(rg), np.maximum(np.abs(rb), np.abs(gb)))
        is_colourful = max_diff >= GRAY_THRESH
        is_black = np.maximum(np.maximum(r, g), b) < 50
        non_gray = (is_colourful | is_black).sum()
        pct = 100.0 * non_gray / slice_count

        self.get_logger().info(
            f"Slice [{SLICE_Z_MIN:.2f}, {SLICE_Z_MAX:.2f}]m @ ≤{DIST_MAX}m: "
            f"{slice_count} pts, {non_gray} non-gray ({pct:.1f}%)"
        )

        # --- 6. publish slice as PointCloud2 (map frame, with RGB) ---
        slice_xyz = transformed[slice_mask]
        points_with_rgb = [
            (float(x), float(y), float(z), int(rgb))
            for x, y, z, rgb in zip(
                slice_xyz[:, 0], slice_xyz[:, 1], slice_xyz[:, 2], slice_rgb
            )
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg = pc2.create_cloud(header, fields, points_with_rgb)
        self.slice_pub.publish(msg)

    @staticmethod
    def _quat_to_mat(qx, qy, qz, qw):
        return [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)],
        ]


def main():
    rclpy.init()
    node = PointCloudViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
