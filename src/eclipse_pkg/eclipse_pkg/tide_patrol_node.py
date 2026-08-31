#!/usr/bin/env python3
"""Wander the dry mudflat during the tide access window.

Does not replace Foxglove one-shot goals. Sends /navigation/patrol_goal
into gps_waypoint_commander so GPS gating, cancel, and autonomy_active
stay on that node. Stops before leave_now / retreat.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from eclipse_pkg.tide_patrol import (
    DEFAULT_KEEPOUT_MARGIN_M,
    DEFAULT_MIN_SECONDS_TO_RETREAT,
    DEFAULT_STEP_MAX_M,
    DEFAULT_STEP_MIN_M,
    keepout_lines_from_markers,
    parse_tide_status,
    patrol_is_allowed,
    sample_patrol_goal,
)


class TidePatrolNode(Node):
    def __init__(self):
        super().__init__('tide_patrol')
        self.declare_parameter('enable_patrol', False)
        self.declare_parameter('status_topic', '/mission/tide_status')
        self.declare_parameter('keepout_topic', '/tide/water_polygon_markers')
        self.declare_parameter('goal_topic', '/navigation/patrol_goal')
        self.declare_parameter('tick_sec', 2.0)
        self.declare_parameter('min_goal_interval_sec', 3.0)
        self.declare_parameter(
            'min_seconds_to_retreat', DEFAULT_MIN_SECONDS_TO_RETREAT)
        self.declare_parameter('step_min_m', DEFAULT_STEP_MIN_M)
        self.declare_parameter('step_max_m', DEFAULT_STEP_MAX_M)
        self.declare_parameter('keepout_margin_m', DEFAULT_KEEPOUT_MARGIN_M)

        self._enabled = bool(self.get_parameter('enable_patrol').value)
        self._status = None
        self._keepout_lines = []
        self._autonomy_active = False
        self._cancel_sent = False
        self._last_goal_mono = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        keepout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._status_callback,
            status_qos,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('keepout_topic').value),
            self._keepout_callback,
            keepout_qos,
        )
        self.create_subscription(
            Bool, '/drive/autonomy_active', self._autonomy_callback, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            10,
        )
        self.cancel_client = self.create_client(Trigger, '/navigation/cancel')
        self.create_service(
            SetBool, '/navigation/patrol_enable', self._enable_callback
        )
        tick = float(self.get_parameter('tick_sec').value)
        self.create_timer(max(0.5, tick), self._tick)
        self.get_logger().info(
            f'tide_patrol ready enable={self._enabled} '
            '(window=phase accessible, goals via gps_waypoint_commander)'
        )

    def _status_callback(self, msg):
        self._status = parse_tide_status(msg.data)

    def _keepout_callback(self, msg):
        self._keepout_lines = keepout_lines_from_markers(msg)

    def _autonomy_callback(self, msg):
        self._autonomy_active = bool(msg.data)

    def _enable_callback(self, request, response):
        self._enabled = bool(request.data)
        self._cancel_sent = False
        response.success = True
        response.message = 'patrol on' if self._enabled else 'patrol off'
        self.get_logger().info(response.message)
        if not self._enabled:
            self._request_cancel()
        return response

    def _robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
        except TransformException:
            return None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def _request_cancel(self):
        if self._cancel_sent:
            return
        if not self.cancel_client.service_is_ready():
            return
        self.cancel_client.call_async(Trigger.Request())
        self._cancel_sent = True

    def _tick(self):
        allowed = self._enabled and patrol_is_allowed(
            self._status,
            min_seconds_to_retreat=float(
                self.get_parameter('min_seconds_to_retreat').value
            ),
        )
        if not allowed:
            if self._enabled:
                self._request_cancel()
            return
        self._cancel_sent = False
        if self._autonomy_active:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        interval = float(self.get_parameter('min_goal_interval_sec').value)
        if (now - self._last_goal_mono) < interval:
            return
        robot = self._robot_xy()
        if robot is None:
            return
        xy = sample_patrol_goal(
            robot,
            self._keepout_lines,
            step_min_m=float(self.get_parameter('step_min_m').value),
            step_max_m=float(self.get_parameter('step_max_m').value),
            keepout_margin_m=float(self.get_parameter('keepout_margin_m').value),
        )
        if xy is None:
            self.get_logger().warn(
                'tide_patrol: no free sample (keepout margin)',
                throttle_duration_sec=10.0,
            )
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = xy[0]
        goal.pose.position.y = xy[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self._last_goal_mono = now
        self.get_logger().info(
            f'tide_patrol goal X={xy[0]:.1f} Y={xy[1]:.1f} '
            f'd={math.hypot(xy[0] - robot[0], xy[1] - robot[1]):.1f}m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TidePatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
