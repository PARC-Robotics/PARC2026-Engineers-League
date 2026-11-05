"""PARC 2026 Real World Phase referee node.

Publishes  /parc/tag_order   (parc_msgs/TagOrder, transient local)
           /parc/run_status  (parc_msgs/RunStatus, transient local, 2 Hz + on change)
Subscribes /parc/start       (parc_msgs/StartSignal)
           /parc/tag_visit   (parc_msgs/TagVisit)
           /parc/finish      (parc_msgs/Finish)
           odometry          (nav_msgs/Odometry, for false starts and the 3 s hold)
Serves the dashboard on http://<robot>:<web_port>/ (judges: /judge?token=<judge_token>).
"""

import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from .run_state import ARMED, RunState
from .tag_check import AVAILABLE as TAG_CHECK_AVAILABLE
from .tag_check import check_tag
from .web_server import make_server

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class Referee(Node):
    def __init__(self):
        super().__init__('parc_referee')
        p = {
            'tag_ids': [1, 4, 7, 9],
            'time_limit_s': 600.0,
            'hold_time_s': 3.0,
            'odom_topic': '/odom',
            'move_threshold_m': 0.05,
            'move_threshold_deg': 5.0,
            'log_dir': '~/parc_runs',
            'web_host': '0.0.0.0',
            'web_port': 8080,
            'judge_token': 'parc2026',
            'safe_floor_zero': True,
            'terminate_keeps_tag_points': True,
        }
        for name, default in p.items():
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.state = RunState(
            tag_ids=g('tag_ids'), time_limit_s=g('time_limit_s'),
            hold_time_s=g('hold_time_s'), move_threshold_m=g('move_threshold_m'),
            move_threshold_rad=math.radians(g('move_threshold_deg')), log_dir=g('log_dir'),
            safe_floor_zero=g('safe_floor_zero'),
            terminate_keeps_tag_points=g('terminate_keeps_tag_points'))

        self.order_pub = self.create_publisher(TagOrder, '/parc/tag_order', LATCHED)
        self.status_pub = self.create_publisher(RunStatus, '/parc/run_status', LATCHED)
        self.create_subscription(StartSignal, '/parc/start', self.on_start, EVENTS)
        self.create_subscription(TagVisit, '/parc/tag_visit', self.on_visit, EVENTS)
        self.create_subscription(Finish, '/parc/finish', self.on_finish, EVENTS)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, qos_profile_sensor_data)

        self._order_key = None
        self._status_version = -1
        self.create_timer(0.1, self.on_tick)
        self.create_timer(0.5, self.publish_status)
        self.publish_order()
        self.publish_status()

        web_dir = os.path.join(get_package_share_directory('parc_referee'), 'web')
        token = g('judge_token')
        make_server(self.state, web_dir, g('web_host'), g('web_port'), token, self.on_change)
        self.get_logger().info(
            f'Dashboard: http://<this-machine>:{g("web_port")}/   '
            f'Judges: http://<this-machine>:{g("web_port")}/judge?token={token}')
        if token == 'parc2026':
            self.get_logger().warn('Using the default judge token. Set judge_token for the competition.')
        if not TAG_CHECK_AVAILABLE:
            self.get_logger().warn('OpenCV aruco not available: visit images will not be checked.')

    # ----------------------------------------------------------------- inputs
    def on_start(self, msg):
        self.state.on_start(msg.method, msg.heard)
        self.on_change()

    def on_visit(self, msg):
        data = bytes(msg.image.data)
        result, detected = check_tag(data, msg.tag_id)
        stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        v = self.state.on_visit(msg.tag_id, data, stamp, result, detected)
        if v:
            self.get_logger().info(f'Tag {msg.tag_id} reported at {v.elapsed_s:.1f} s '
                                   f'({v.order_status}, image {result})')
        self.on_change()

    def on_finish(self, msg):
        self.state.on_finish(msg.reason)
        self.on_change()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.state.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_tick(self):
        self.state.tick()
        if self.state.version != self._status_version:
            self.on_change()

    # ---------------------------------------------------------------- outputs
    def on_change(self):
        self.publish_order()
        self.publish_status()

    def publish_order(self):
        s = self.state
        with s.lock:
            # Publish on arm/restart; publish an empty order when idle so a stale
            # order is never handed to the next team.
            key = (s.run_dir, len(s.attempts)) if s.state == ARMED else ('idle',) if not s.order else None
            if key is None or key == self._order_key:
                return
            self._order_key = key
            msg = TagOrder()
            msg.stamp = self.get_clock().now().to_msg()
            msg.tag_ids = list(s.order) if s.state == ARMED else []
        self.order_pub.publish(msg)

    def publish_status(self):
        s = self.state
        with s.lock:
            a = s.attempts[-1] if s.attempts else None
            msg = RunStatus()
            msg.stamp = self.get_clock().now().to_msg()
            msg.state = s.state
            msg.elapsed_s = float(s.elapsed())
            msg.time_limit_s = float(s.time_limit_s)
            msg.tag_order = list(s.order)
            msg.tags_reported = s.tags_reported()
            msg.start_method = a.start_method if a and a.start_method is not None else 0
            msg.false_start = bool(a and a.false_start)
            self._status_version = s.version
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Referee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.state.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
