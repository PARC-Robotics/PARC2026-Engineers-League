#!/usr/bin/env python3
"""Drop lidar returns that come from the robot's own body.

Beams shorter than `min_range` are replaced with infinity, as for a missing return.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter")
        self.declare_parameter("min_range", 0.3)
        self.min_range = self.get_parameter("min_range").value

        self.pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(LaserScan, "/scan", self.filter, qos_profile_sensor_data)

    def filter(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges[np.isfinite(ranges) & (ranges < self.min_range)] = math.inf
        scan.ranges = ranges.tolist()
        scan.range_min = max(scan.range_min, float(self.min_range))
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ScanFilter()
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
