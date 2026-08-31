#!/usr/bin/env python3

import copy
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL, ToLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from eclipse_pkg.eclipse_test_config import (
    GPS_MIN_GOOD_FIX_STATUS,
    GPS_MIN_NAV_START_FIX_STATUS,
)
from eclipse_pkg_msgs.srv import GpsGoal


def gps_fix_ready_for_navigation(fix_status, minimum_status):
    """Whether a NavSatStatus is solid enough to start a new goal.

    Float can drift the map origin tens of cm while the robot is
    stationary, so starting navigation requires RTK Fixed specifically,
    not just the looser "corrected GNSS" threshold used elsewhere.
    """
    if fix_status is None:
        return False
    return int(fix_status) >= int(minimum_status)


def build_toll_request(goal_pose):
    """Build a robot_localization ToLL request from a map-frame goal pose."""
    request = ToLL.Request()
    request.map_point = goal_pose.pose.position
    return request


def geodetic_roundtrip_error(origin_point, roundtrip_point):
    """Planar map error after a map -> lat/lon -> map conversion.

    Structure borrowed from the collaborator's workspace (~/ewooni_docker_4),
    where every goal had to pass a ToLL -> FromLL round-trip check. Here the
    same idea comes for free on the /clicked_point path: the click already
    gives us a map point, so converting it out to lat/lon and back and
    comparing tells us whether the geodetic datum currently agrees with the
    map frame. A large error means navsat_transform's datum has drifted away
    from the map the operator is clicking on, so the lat/lon we would act on
    is not the place that was clicked.
    """
    return math.hypot(
        float(roundtrip_point.x) - float(origin_point.x),
        float(roundtrip_point.y) - float(origin_point.y),
    )


class GpsWaypointCommander(Node):
    def __init__(self):
        super().__init__('gps_waypoint_commander')

        # Foxglove /goal_pose 기본 QoS와 맞춘다.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Foxglove 3D 패널에서 지정한 위치로만 이동한다.
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            qos_profile
        )

        # 기존 운용 호환성을 위한 별칭. /goal_pose와 동일하게 단순 이동한다.
        self.goal_dropoff_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose_dropoff',
            self.goal_pose_dropoff_callback,
            qos_profile
        )

        self._last_feedback_log_sec = None
        self._goal_handle = None
        self._cancel_requested = False
        self.send_goal_future = None
        self._cancel_future = None
        self._gps_fix_status = None

        # GPS_node publishes /gps/fix best-effort; match it so this
        # subscription actually connects.
        gps_fix_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.gps_fix_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_fix_callback,
            gps_fix_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Client for Nav2 NavigateToPose
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Nav2가 /cmd_vel을 담당하는 동안 수동 heading hold를 비활성화한다.
        self.autonomy_active_pub = self.create_publisher(
            Bool, '/drive/autonomy_active', 10
        )
        self.navigation_stop_pub = self.create_publisher(
            Bool, '/drive/navigation_stop', 10
        )

        # Foxglove Service Call 패널에서 누를 수 있는 일반 취소 서비스.
        self.cancel_service = self.create_service(
            Trigger,
            '/navigation/cancel',
            self.cancel_navigation_callback,
        )

        # Foxglove /goal_pose는 map 로컬 좌표. /fromLL은 위경도를 같은 경로로 보낸다.
        self.fromll_client = self.create_client(FromLL, '/fromLL')
        self.gps_goal_service = self.create_service(
            GpsGoal,
            '/navigation/gps_goal',
            self.gps_goal_callback,
        )

        # navsat_transform_node가 제공하는 map -> 위경도 변환 서비스.
        # Foxglove 패널에 목표 위경도를 확인용으로 표시하기 위해서만 쓰이며,
        # 이 서비스가 준비되지 않아도 내비게이션 자체는 막지 않는다.
        self.toll_client = self.create_client(ToLL, 'toLL')
        goal_latlon_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.goal_latlon_pub = self.create_publisher(
            GeoPoint, '/goal_pose/latlon', goal_latlon_qos
        )

        # /clicked_point는 /toLL -> /fromLL 왕복으로 위경도 목표가 된다.
        # 왕복 오차는 map과 위경도가 맞는지 검사한다.
        # 조정가능 — 왕복 허용 오차(m).
        self.declare_parameter('clicked_point_roundtrip_max_error_m', 1.0)
        self._roundtrip_max_error_m = float(
            self.get_parameter('clicked_point_roundtrip_max_error_m').value
        )
        self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        self.get_logger().info(
            'GPS Waypoint Commander initialized. '
            'Waiting for /goal_pose (direct map goal) or /clicked_point '
            '(routed through lat/lon) from Foxglove...'
        )

    def gps_fix_callback(self, msg):
        self._gps_fix_status = msg.status.status

    def goal_pose_callback(self, msg):
        self.get_logger().info(
            'Received navigation goal from Foxglove! '
            f'[X: {msg.pose.position.x:.2f}, '
            f'Y: {msg.pose.position.y:.2f}, '
            f'frame: {msg.header.frame_id}]'
        )
        self._send_nav2_goal(msg)

    def goal_pose_dropoff_callback(self, msg):
        self.get_logger().info(
            'Received drop-off goal from Foxglove! '
            f'[X: {msg.pose.position.x:.2f}, '
            f'Y: {msg.pose.position.y:.2f}]'
        )
        self._send_nav2_goal(msg)

    def clicked_point_callback(self, msg: PointStamped):
        """Turn a Foxglove point click into a lat/lon goal.

        The click arrives as a map-frame point. We resolve it to a proper map
        goal first (this also derives the heading from base_link toward the
        point), then convert that point to lat/lon with /toLL and hand it to
        the same pipeline /navigation/gps_goal uses. The original map point is
        carried along so the /fromLL result can be checked against it.
        """
        self.get_logger().info(
            'Received clicked point from Foxglove! '
            f'[X: {msg.point.x:.2f}, Y: {msg.point.y:.2f}, '
            f'frame: {msg.header.frame_id}]'
        )
        probe = PoseStamped()
        probe.header = copy.deepcopy(msg.header)
        probe.pose.position = copy.deepcopy(msg.point)
        probe.pose.orientation.w = 1.0

        map_goal = self._prepare_map_goal(probe)
        if map_goal is None:
            return

        if not self.toll_client.service_is_ready():
            self.get_logger().error(
                'Rejecting clicked point: /toLL service is not available, so '
                'the click cannot be expressed as lat/lon.'
            )
            return

        origin_point = copy.deepcopy(map_goal.pose.position)
        future = self.toll_client.call_async(build_toll_request(map_goal))
        future.add_done_callback(
            lambda done, origin=origin_point:
            self._clicked_point_toll_callback(done, origin)
        )

    def _clicked_point_toll_callback(self, future, origin_point):
        try:
            ll_point = future.result().ll_point
        except Exception as exc:
            self.get_logger().error(
                f'/toLL failed for clicked point: {exc}'
            )
            return

        self.goal_latlon_pub.publish(ll_point)
        self.get_logger().info(
            'Clicked point resolved to lat/lon: '
            f'{ll_point.latitude:.7f}, {ll_point.longitude:.7f} — '
            'dispatching through the GPS goal path.'
        )
        self._dispatch_latlon_goal(
            ll_point.latitude,
            ll_point.longitude,
            expected_map_point=origin_point,
            source='clicked point',
        )

    def _dispatch_latlon_goal(
        self,
        latitude,
        longitude,
        expected_map_point=None,
        source='GPS goal',
    ):
        """Convert lat/lon to a map goal via /fromLL and dispatch it.

        Shared by /navigation/gps_goal and the /clicked_point path so both
        reach Nav2 through exactly one route. expected_map_point, when given,
        enables the round-trip error check in _fromll_done_callback.
        """
        if not self.fromll_client.service_is_ready():
            self.get_logger().error(
                f'/fromLL service not available; dropping {source}.'
            )
            return False

        ll_request = FromLL.Request()
        ll_request.ll_point = GeoPoint(
            latitude=float(latitude),
            longitude=float(longitude),
            altitude=0.0,
        )
        future = self.fromll_client.call_async(ll_request)
        future.add_done_callback(
            lambda done, expected=expected_map_point, label=source:
            self._fromll_done_callback(done, expected, label)
        )
        return True

    def gps_goal_callback(self, request, response):
        if not gps_fix_ready_for_navigation(
            self._gps_fix_status, GPS_MIN_GOOD_FIX_STATUS
        ):
            response.accepted = False
            response.message = (
                'Rejecting goal: GPS fix too weak '
                f'(current status={self._gps_fix_status}).'
            )
            self.get_logger().error(response.message)
            return response

        if not self._dispatch_latlon_goal(
            request.latitude, request.longitude, source='GPS goal'
        ):
            response.accepted = False
            response.message = '/fromLL service not available.'
            return response

        response.accepted = True
        response.message = (
            'Lat/lon conversion requested; goal dispatch is async. '
            'Check /navigate_to_pose feedback for progress.'
        )
        return response

    def _fromll_done_callback(
        self, future, expected_map_point=None, source='GPS goal'
    ):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'/fromLL call failed for {source}: {exc}'
            )
            return

        if expected_map_point is not None:
            error_m = geodetic_roundtrip_error(
                expected_map_point, result.map_point
            )
            if error_m > self._roundtrip_max_error_m:
                # Fail closed. The operator clicked a spot on the map; if the
                # map <-> lat/lon round trip lands somewhere else, acting on
                # that lat/lon drives to a different place than was clicked.
                self.get_logger().error(
                    f'Rejecting {source}: ToLL -> FromLL round-trip moved the '
                    f'goal by {error_m:.2f} m (limit '
                    f'{self._roundtrip_max_error_m:.2f} m). The map frame and '
                    'the geodetic datum disagree — check navsat_transform.'
                )
                return
            self.get_logger().info(
                f'Round-trip check passed for {source}: {error_m:.2f} m.'
            )

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = result.map_point
        goal.pose.orientation.w = 1.0
        self.get_logger().info(
            f'Converted lat/lon to map goal via /fromLL ({source}): '
            f'X={goal.pose.position.x:.2f}, Y={goal.pose.position.y:.2f}'
        )
        self._send_nav2_goal(goal, min_fix_status=GPS_MIN_GOOD_FIX_STATUS)

    def _send_nav2_goal(self, msg, min_fix_status=GPS_MIN_NAV_START_FIX_STATUS):
        if not gps_fix_ready_for_navigation(
            self._gps_fix_status, min_fix_status
        ):
            self.get_logger().error(
                'Rejecting goal: GPS fix too weak '
                f'(current status={self._gps_fix_status}, '
                f'required>={min_fix_status}). '
                'The map origin can drift while GPS is not RTK Fixed.'
            )
            return

        goal_pose = self._prepare_map_goal(msg)
        if goal_pose is None:
            return

        self._request_goal_latlon(goal_pose)

        # Wait for Action Server to be ready
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'Nav2 NavigateToPose action server not available!'
            )
            return

        # 새 목표만 정지 latch를 해제할 수 있다.
        self._publish_navigation_stop(False)
        # 목표를 보내는 순간부터 수동 명령을 차단한다.
        self._publish_autonomy_active(True)

        # Prepare Action Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send Action Goal
        self.get_logger().info('Sending goal to Nav2...')
        self._cancel_requested = False
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.send_goal_future = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send Nav2 goal: {exc}')
            self._publish_navigation_stop(True)
            self._publish_autonomy_active(False)
            return

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal!')
            self._publish_navigation_stop(True)
            self._publish_autonomy_active(False)
            return

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda result_future: self.get_result_callback(
                result_future, goal_handle
            )
        )
        if self._cancel_requested:
            self.get_logger().info(
                'Cancel was requested while the goal was being accepted.'
            )
            self._request_cancel(goal_handle)
            return

        self.get_logger().info('Nav2 accepted the goal. Navigating...')
        self._publish_autonomy_active(True)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if (
            self._last_feedback_log_sec is None
            or now_sec - self._last_feedback_log_sec >= 1.0
        ):
            self._last_feedback_log_sec = now_sec
            self.get_logger().info(
                f'Nav2 distance remaining: {feedback.distance_remaining:.2f} m'
            )

    def get_result_callback(self, future, goal_handle=None):
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')

        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('Arrived at destination.')
        is_current_goal = (
            goal_handle is None or self._goal_handle is goal_handle
        )
        if is_current_goal:
            self._goal_handle = None
            self._cancel_requested = False
            self._publish_navigation_stop(True)
            self._publish_autonomy_active(False)

    def cancel_navigation_callback(self, _request, response):
        self._cancel_requested = True
        # Nav2 취소 응답을 기다리지 않고 모터 단계에서 즉시 정지한다.
        self._publish_navigation_stop(True)
        self._publish_autonomy_active(False)

        if self._goal_handle is not None:
            self._request_cancel(self._goal_handle)
            response.success = True
            response.message = 'Navigation cancel requested.'
        elif self.send_goal_future is not None:
            response.success = True
            response.message = (
                'Navigation cancel queued while the goal is being accepted.'
            )
        else:
            self._cancel_requested = False
            response.success = True
            response.message = 'No active navigation goal; already stopped.'

        self.get_logger().info(response.message)
        return response

    def _request_cancel(self, goal_handle):
        if self._cancel_future is not None:
            return
        self._cancel_future = goal_handle.cancel_goal_async()
        self._cancel_future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future):
        self._cancel_future = None
        try:
            cancel_response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Navigation cancel failed: {exc}')
            return

        if cancel_response.goals_canceling:
            self.get_logger().info('Nav2 accepted the navigation cancel.')
        else:
            self.get_logger().warning(
                'Nav2 did not find an active goal to cancel.'
            )

    def _request_goal_latlon(self, goal_pose: PoseStamped) -> None:
        """Look up and publish the goal's lat/lon for Foxglove display only.

        Purely informational: failures or an unready service must never
        block navigation, so this only logs a warning and returns.
        """
        if not self.toll_client.service_is_ready():
            self.get_logger().warning(
                '/toLL service not ready; skipping goal lat/lon display.'
            )
            return
        future = self.toll_client.call_async(build_toll_request(goal_pose))
        future.add_done_callback(self._goal_latlon_response_callback)

    def _goal_latlon_response_callback(self, future) -> None:
        try:
            ll_point = future.result().ll_point
        except Exception as exc:
            self.get_logger().warning(f'/toLL request failed: {exc}')
            return
        self.goal_latlon_pub.publish(ll_point)
        self.get_logger().info(
            'Goal lat/lon: '
            f'{ll_point.latitude:.7f}, {ll_point.longitude:.7f}'
        )

    def _publish_autonomy_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self.autonomy_active_pub.publish(msg)

    def _publish_navigation_stop(self, stopped: bool) -> None:
        msg = Bool()
        msg.data = stopped
        self.navigation_stop_pub.publish(msg)

    def _prepare_map_goal(self, msg: PoseStamped):
        goal = copy.deepcopy(msg)
        source_frame = goal.header.frame_id.lstrip('/')
        if not source_frame:
            self.get_logger().error(
                'Rejecting goal without a frame_id.'
            )
            return None

        values = (
            goal.pose.position.x,
            goal.pose.position.y,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error(
                'Rejecting goal containing a non-finite value.'
            )
            return None

        if source_frame != 'map':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.5),
                )
                goal = do_transform_pose_stamped(goal, transform)
            except TransformException as exc:
                self.get_logger().error(
                    f'Cannot transform goal from {source_frame!r} to '
                    f"'map': {exc}"
                )
                return None

        try:
            base_transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().error(
                f"Cannot determine goal heading from 'base_link': {exc}"
            )
            return None

        base_position = base_transform.transform.translation
        dx = goal.pose.position.x - base_position.x
        dy = goal.pose.position.y - base_position.y
        distance = math.hypot(dx, dy)
        if distance >= 1e-3:
            goal_yaw = math.atan2(dy, dx)
        else:
            base_q = base_transform.transform.rotation
            goal_yaw = math.atan2(
                2.0 * (base_q.w * base_q.z + base_q.x * base_q.y),
                1.0 - 2.0 * (base_q.y * base_q.y + base_q.z * base_q.z),
            )

        # Foxglove 2D Pose의 드래그 화살표는 무시한다. 클릭 위치를 향하도록
        # 최종 자세를 자동 계산해 위치 선택만으로 주행할 수 있게 한다.
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        # Foxglove 클릭 시각이 오래된 TF보다 앞서면 Nav2가 거부한다.
        # 최신 TF로 변환한 목표이므로 현재 시각으로 갱신한다.
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'Prepared map goal from {source_frame!r}: '
            f'X={goal.pose.position.x:.2f}, Y={goal.pose.position.y:.2f}, '
            f'auto_yaw={math.degrees(goal_yaw):.1f} deg'
        )
        return goal


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
