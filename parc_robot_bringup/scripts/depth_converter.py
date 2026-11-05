#!/usr/bin/env python3
"""Republish Gazebo float depth (metres) as RealSense-style 16UC1 depth (millimetres).

Stereo depth noise grows with the square of the distance, so each pixel gets
Gaussian noise with a standard deviation of `noise_coefficient * z^2` metres
(0.004 gives about 1.6 cm at 2 m, matching the D435's quoted accuracy). Returns
closer than `min_range` are dropped, as on the camera.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

INVALID_DEPTH_MM = 0


class DepthConverter(Node):
    def __init__(self):
        super().__init__("depth_converter")
        self.declare_parameter("cameras", ["camera1", "camera2"])
        self.declare_parameter("noise_coefficient", 0.004)
        self.declare_parameter("min_range", 0.3)
        self.noise_coefficient = self.get_parameter("noise_coefficient").value
        self.min_range = self.get_parameter("min_range").value
        self.rng = np.random.default_rng()
        out_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        for cam in self.get_parameter("cameras").value:
            pub = self.create_publisher(
                Image, f"/{cam}/{cam}/depth/image_rect_raw", out_qos
            )
            self.create_subscription(
                Image,
                f"/sim/{cam}/depth/depth_image",
                lambda msg, pub=pub: self.convert(msg, pub),
                qos_profile_sensor_data,
            )

    def convert(self, msg, pub):
        if msg.encoding != "32FC1":
            self.get_logger().error(
                f"Unsupported depth encoding {msg.encoding}", throttle_duration_sec=5.0
            )
            return

        depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.step // 4
        )[:, : msg.width]
        sigma = np.where(np.isfinite(depth_m), self.noise_coefficient * depth_m**2, 0.0)
        depth_m = depth_m + self.rng.normal(0.0, sigma)
        depth_mm = depth_m * 1000.0
        invalid = (
            ~np.isfinite(depth_mm)
            | (depth_m < self.min_range)
            | (depth_mm > 65535.0)
        )
        depth_mm[invalid] = INVALID_DEPTH_MM

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = np.ascontiguousarray(depth_mm.astype(np.uint16)).tobytes()
        pub.publish(out)


def main():
    rclpy.init()
    node = DepthConverter()
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
