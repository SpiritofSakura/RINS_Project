#!/usr/bin/env python3
"""Point cloud spill analysis service — call /pointcloud_viewer/spill_check."""

import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
import tf2_ros


DIST_MAX            = 1.0   # m — ignore points further from camera
SLICE_Z_MIN         = 0.005 # m above ground (map frame)
SLICE_Z_MAX         = 0.15  # m above ground (map frame)
SPILL_POINT_THRESH  = 4000  # min points in Z-slice → spill


class PointCloudSpillCheck(Node):
    def __init__(self):
        super().__init__("pointcloud_viewer")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_cloud = None

        self.create_subscription(
            PointCloud2, "/oakd/rgb/preview/depth/points",
            self._cloud_cb, 1
        )
        self.slice_pub = self.create_publisher(PointCloud2, "/slice_points", 10)
        self.result_pub = self.create_publisher(String, "/spill_check_result", 10)
        self.status_marker_pub = self.create_publisher(Marker, "/spill_status_marker", 10)
        self.srv = self.create_service(Trigger, "spill_check", self._spill_check_cb)

        self.get_logger().info(f"Ready — slice [{SLICE_Z_MIN}, {SLICE_Z_MAX}]  threshold={SPILL_POINT_THRESH}")

    def _cloud_cb(self, msg):
        self.latest_cloud = msg

    def _finish_response(self, response, success, payload):
        response.success = success
        response.message = json.dumps(payload)
        result_msg = String()
        result_msg.data = response.message
        self.result_pub.publish(result_msg)
        spill_detected = payload.get("spill_detected")
        if spill_detected is not None:
            self._publish_status_marker(spill_detected)
        return response

    def _publish_status_marker(self, spill_detected):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spill_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.35
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        if spill_detected is True:
            marker.text = "SPILL"
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif spill_detected is False:
            marker.text = "OK"
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            marker.pose.position.x = tf.transform.translation.x
            marker.pose.position.y = tf.transform.translation.y
            marker.pose.position.z = tf.transform.translation.z + 0.9
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"Could not publish spill status marker at robot pose: {exc}")
            return

        self.status_marker_pub.publish(marker)

    def _spill_check_cb(self, _request, response):
        if self.latest_cloud is None:
            return self._finish_response(
                response,
                False,
                {"point_count": 0, "spill_detected": None, "error": "no point cloud received yet"},
            )

        cloud = self.latest_cloud

        # --- 1. read raw points ---
        try:
            structured = pc2.read_points(
                cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True
            )
        except Exception as e:
            return self._finish_response(
                response,
                False,
                {"point_count": 0, "spill_detected": None, "error": str(e)},
            )

        if len(structured) == 0:
            return self._finish_response(
                response,
                False,
                {"point_count": 0, "spill_detected": None, "error": "empty cloud"},
            )

        # --- 2. distance filter (sensor frame) ---
        xs = structured["x"].astype(np.float64)
        ys = structured["y"].astype(np.float64)
        zs = structured["z"].astype(np.float64)
        dists = np.sqrt(xs**2 + ys**2 + zs**2)
        near_mask = dists <= DIST_MAX
        if near_mask.sum() == 0:
            return self._finish_response(
                response,
                True,
                {"point_count": 0, "spill_detected": False},
            )

        xs_near = xs[near_mask]
        ys_near = ys[near_mask]
        zs_near = zs[near_mask]

        # --- 3. transform to map frame ---
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", cloud.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except tf2_ros.TransformException as e:
            return self._finish_response(
                response,
                False,
                {"point_count": 0, "spill_detected": None, "error": str(e)},
            )

        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_mat(q.x, q.y, q.z, q.w)

        pts = np.column_stack([xs_near, ys_near, zs_near])
        rotated = np.dot(pts, np.array(R).T)
        transformed = rotated + np.array([t.x, t.y, t.z])

        # --- 4. Z-slice filter ---
        z = transformed[:, 2]
        slice_mask = (z >= SLICE_Z_MIN) & (z <= SLICE_Z_MAX)
        count = int(slice_mask.sum())
        spill_detected = count >= SPILL_POINT_THRESH

        self.get_logger().info(
            f"[{SLICE_Z_MIN}, {SLICE_Z_MAX}] - {count} points - {SPILL_POINT_THRESH} threshold - "
            f"{'SPILL' if spill_detected else 'OK'}"
        )

        # --- 5. publish slice for RViz ---
        if count > 0:
            slice_xyz = transformed[slice_mask]
            rgb = np.asarray(structured["rgb"][near_mask], dtype=np.uint32)[slice_mask]
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            fields = [
                PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
            ]
            points_with_rgb = [
                (float(x), float(y), float(z), int(c))
                for x, y, z, c in zip(slice_xyz[:, 0], slice_xyz[:, 1], slice_xyz[:, 2], rgb)
            ]
            msg = pc2.create_cloud(header, fields, points_with_rgb)
            self.slice_pub.publish(msg)

        return self._finish_response(
            response,
            True,
            {
                "point_count": count,
                "spill_detected": spill_detected,
            },
        )

    @staticmethod
    def _quat_to_mat(qx, qy, qz, qw):
        return [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)],
        ]


def main():
    rclpy.init()
    node = PointCloudSpillCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
