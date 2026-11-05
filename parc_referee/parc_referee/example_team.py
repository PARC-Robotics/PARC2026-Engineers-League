"""Example of the referee interface for teams. Copy the parts you need.

Shows how to:
  * receive the tag order (/parc/tag_order, transient local)
  * send the start signal (/parc/start)
  * report a tag visit with a camera image (/parc/tag_visit), then hold 3 s
  * end the run early (/parc/finish)
  * watch /parc/run_status and stop when the run is over

This node does not drive the robot. With demo:=true it fakes a run by
"visiting" each tag in order every few seconds, for testing the referee.

ros2 run parc_referee example_team --ros-args -p demo:=true
"""

import time

import rclpy
from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

# Use exactly these QoS settings.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

HOLD_TIME_S = 3.0


class ExampleTeam(Node):
    def __init__(self):
        super().__init__('example_team')
        self.declare_parameter('demo', False)
        self.declare_parameter('camera_topic', '/top_camera_color/image_raw/compressed')

        self.order = []
        self.run_state = RunStatus.IDLE
        self.latest_image = None

        self.create_subscription(TagOrder, '/parc/tag_order', self.on_order, LATCHED)
        self.create_subscription(RunStatus, '/parc/run_status', self.on_status, LATCHED)
        self.create_subscription(CompressedImage, self.get_parameter('camera_topic').value,
                                 self.on_image, qos_profile_sensor_data)

        self.start_pub = self.create_publisher(StartSignal, '/parc/start', EVENTS)
        self.visit_pub = self.create_publisher(TagVisit, '/parc/tag_visit', EVENTS)
        self.finish_pub = self.create_publisher(Finish, '/parc/finish', EVENTS)

        if self.get_parameter('demo').value:
            self.demo_step = 0
            self.demo_next = time.monotonic() + 2.0
            self.create_timer(0.2, self.demo_tick)

    # ------------------------------------------------------------ referee -> you
    def on_order(self, msg):
        self.order = list(msg.tag_ids)
        if self.order:
            self.get_logger().info(f'Tag order received: {self.order}')

    def on_status(self, msg):
        if msg.state != self.run_state and msg.state in (
                RunStatus.FINISHED, RunStatus.TIMED_OUT, RunStatus.TERMINATED):
            self.get_logger().info('Run is over: stop the robot now.')
            # -> publish a zero velocity / cancel your Nav2 goal here.
        self.run_state = msg.state

    def on_image(self, msg):
        self.latest_image = msg

    # ------------------------------------------------------------ you -> referee
    def send_start(self, voice_word=''):
        """Call once, when your robot starts (typed command or start word)."""
        msg = StartSignal()
        msg.stamp = self.get_clock().now().to_msg()
        msg.method = StartSignal.METHOD_VOICE if voice_word else StartSignal.METHOD_COMMAND
        msg.heard = voice_word
        self.start_pub.publish(msg)

    def report_visit(self, tag_id, image):
        """Call once per tag, while stopped in the visit pose. Then hold still for 3 s."""
        msg = TagVisit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = image.header.frame_id if image else ''
        msg.tag_id = tag_id
        if image is not None:
            msg.image = image   # a CompressedImage (JPEG) in which the tag is visible
        self.visit_pub.publish(msg)
        # Your code: keep the robot stationary for HOLD_TIME_S, then drive to the next tag.

    def send_finish(self, gave_up=False):
        """Only needed to end early, e.g. after skipping a tag."""
        msg = Finish()
        msg.stamp = self.get_clock().now().to_msg()
        msg.reason = Finish.REASON_GAVE_UP if gave_up else Finish.REASON_COMPLETED
        self.finish_pub.publish(msg)

    # ---------------------------------------------------------------- demo only
    def demo_tick(self):
        if not self.order or time.monotonic() < self.demo_next:
            return
        if self.demo_step == 0 and self.run_state == RunStatus.ARMED:
            self.get_logger().info('Demo: sending start signal')
            self.send_start()
            self.demo_step, self.demo_next = 1, time.monotonic() + 4.0
        elif 1 <= self.demo_step <= len(self.order) and self.run_state == RunStatus.RUNNING:
            tag = self.order[self.demo_step - 1]
            self.get_logger().info(f'Demo: reporting tag {tag}')
            self.report_visit(tag, self.latest_image or _demo_image(tag))
            self.demo_step, self.demo_next = self.demo_step + 1, time.monotonic() + HOLD_TIME_S + 2.0


def _demo_image(tag_id):
    """A synthetic JPEG of the tag, used only when no camera image is available."""
    msg = CompressedImage()
    msg.format = 'jpeg'
    try:
        import cv2
        import numpy as np
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        gen = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') \
            else cv2.aruco.drawMarker
        tag = gen(d, tag_id, 240)
        img = np.full((480, 640), 200, np.uint8)
        img[120:360, 200:440] = tag
        msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    except Exception:
        pass
    return msg


def main():
    rclpy.init()
    node = ExampleTeam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
