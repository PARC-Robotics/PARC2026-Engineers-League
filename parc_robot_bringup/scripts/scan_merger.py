#!/usr/bin/env python3
"""Merge the lidar scan and the depth-camera scans into one scan in the laser frame.

Each beam is projected into the target frame and the nearest return per angular
bin wins.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

MAX_SCAN_AGE_S = 0.5


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ScanMerger(Node):
    def __init__(self):
        super().__init__("scan_merger")
        self.declare_parameter("target_frame", "laser")
        self.declare_parameter(
            "input_topics", ["/scan_filtered", "/camera1/scan", "/camera2/scan"]
        )
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("bins", 360)
        self.declare_parameter("range_min", 0.3)
        self.declare_parameter("range_max", 12.0)

        self.target = self.get_parameter("target_frame").value
        self.bins = self.get_parameter("bins").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest = {}
        for topic in self.get_parameter("input_topics").value:
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, topic=topic: self.latest.__setitem__(topic, msg),
                qos_profile_sensor_data,
            )

        self.pub = self.create_publisher(
            LaserScan, "/scan_merged", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.publish_merged)

    def to_target_points(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
        )
        r, a = ranges[valid], angles[valid]
        points = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)

        tf = self.tf_buffer.lookup_transform(
            self.target, scan.header.frame_id, Time(), Duration(seconds=0.1)
        )
        rotation = quaternion_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        return points @ rotation.T + np.array([t.x, t.y, t.z])

    def publish_merged(self):
        now = self.get_clock().now()
        increment = 2.0 * math.pi / self.bins
        merged = np.full(self.bins, np.inf)

        for scan in self.latest.values():
            age = (now - Time.from_msg(scan.header.stamp)).nanoseconds * 1e-9
            if age > MAX_SCAN_AGE_S:
                continue
            try:
                points = self.to_target_points(scan)
            except TransformException as err:
                self.get_logger().warn(str(err), throttle_duration_sec=5.0)
                continue
            ranges = np.hypot(points[:, 0], points[:, 1])
            bearings = np.arctan2(points[:, 1], points[:, 0])
            index = np.floor((bearings + math.pi) / increment).astype(int) % self.bins
            keep = (ranges >= self.range_min) & (ranges <= self.range_max)
            np.minimum.at(merged, index[keep], ranges[keep])

        out = LaserScan()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.target
        out.angle_min = -math.pi
        out.angle_max = math.pi - increment
        out.angle_increment = increment
        out.scan_time = 1.0 / self.get_parameter("rate_hz").value
        out.time_increment = 0.0
        out.range_min = float(self.range_min)
        out.range_max = float(self.range_max)
        out.ranges = merged.astype(np.float32).tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
