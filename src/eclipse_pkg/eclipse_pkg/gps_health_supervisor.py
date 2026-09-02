#!/usr/bin/env python3
"""GPS loss health supervisor (W2 + soft recover + return-to-home).

When /gps/fix goes stale:
  1) Cancel Nav2 + hold stop
  2) Wait T_wait for fix return (W2)
  3) On timeout: call /gps/recover once (soft: serial reopen + NTRIP bounce)
  4) Wait T_wait2
  5) If still lost: NavigateToPose to "operating base" home pose in map
     (best-effort under degraded localization — no GPS hard restart)

Home pose: first good-fix map pose (auto) and/or parameters home_x/home_y.
Hard process restart is intentionally not used (recalibration complexity).

This node also publishes /navigation/readiness (see NAVIGATION READINESS
below), because it already owns every input that answer needs.
"""

from __future__ import annotations

import math
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

# ===========================================================================
# NAVIGATION READINESS
# ===========================================================================
# Nav2's lifecycle "active" means the servers are CONFIGURED, not that the
# robot can drive. This tree has no heading_calibration_node and no
# gps_fix_gate: EKF pose0 fuses /imu/mag_heading, and navsat_transform_node
# reads /gps/fix directly. Readiness is therefore GPS flowing + map TF.
#
# We publish the FIRST unmet link, not a bare boolean. Deliberately NOT
# latched (VOLATILE, periodic): this node lives for the whole session, and
# periodic publication carries liveness a latch cannot.
READINESS_TOPIC = '/navigation/readiness'

# How long /gps/fix may go quiet before the GPS stage is unmet. The driver
# publishes at ~5 Hz, so this only trips on a real stall. 조정가능.
FIX_STALE_SEC = 3.0


def evaluate_navigation_readiness(
    fix_age_sec: float | None,
    map_tf_available: bool,
    fix_stale_sec: float = FIX_STALE_SEC,
    tf_detail: str = '',
) -> tuple[str, str]:
    """Describe whether the robot can navigate, and if not, what is missing.

    Kept a pure function (no rclpy) so the stage ordering is testable.

    :param fix_age_sec: age of the newest /gps/fix, or None if none arrived.
    :param map_tf_available: map->base_link resolves in the TF buffer.
    :param fix_stale_sec: age above which the fix counts as stopped.
    :param tf_detail: tf2's own explanation, when it has one.
    :return: (stage, text). stage is a stable identifier; text is one line.
    """
    if fix_age_sec is None:
        return 'FIX_SILENT', (
            'BLOCKED: no /gps/fix has arrived — check GPS_node'
        )

    if fix_age_sec > fix_stale_sec:
        return 'FIX_STALED', (
            'BLOCKED: /gps/fix stopped '
            f'{fix_age_sec:.0f}s ago — GPS driver stalled'
        )

    if not map_tf_available:
        detail = f' ({tf_detail})' if tf_detail else ''
        return 'MAP_TF', (
            'BLOCKED: GPS flowing but no map->base_link yet'
            f'{detail} — navsat_transform/EKF has not produced map->odom'
        )

    return 'READY', 'READY: map->base_link available with live GPS'


def should_start_tide_return(
        phase_name, retreat_true, already_sent, home_available):
    """조석 철수 True 가 gps_home 복귀를 시작해야 하면 True.

    GPS 상실 FSM(LOST_*/RETURN_HOME/FAILED) 중에는 건드리지 않는다.
    복귀 목표를 이미 보냈으면 다시 보내지 않는다.
    """
    if not retreat_true or already_sent or not home_available:
        return False
    return str(phase_name) == 'OK'


class Phase(Enum):
    OK = auto()
    LOST_WAIT = auto()
    SOFT_RECOVER = auto()
    LOST_WAIT2 = auto()
    RETURN_HOME = auto()
    FAILED = auto()


class GpsHealthSupervisor(Node):
    def __init__(self):
        super().__init__('gps_health_supervisor')

        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('stale_sec', 2.0)
        self.declare_parameter('wait_sec', 45.0)
        self.declare_parameter('wait2_sec', 30.0)
        self.declare_parameter('min_fix_status', 0)
        self.declare_parameter('tick_hz', 2.0)
        self.declare_parameter('enable_return_home', True)
        self.declare_parameter('home_frame', 'map')
        # NaN => capture first good-fix pose as home
        self.declare_parameter('home_x', float('nan'))
        self.declare_parameter('home_y', float('nan'))
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('recover_service', '/gps/recover')
        self.declare_parameter('cancel_service', '/navigation/cancel')
        self.declare_parameter('fix_stale_sec', FIX_STALE_SEC)
        self.declare_parameter('retreat_topic', '/mission/retreat')
        self.declare_parameter('home_pose_topic', '/mission/home_pose')

        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.wait_sec = float(self.get_parameter('wait_sec').value)
        self.wait2_sec = float(self.get_parameter('wait2_sec').value)
        self.min_fix_status = int(self.get_parameter('min_fix_status').value)
        self.enable_return_home = bool(
            self.get_parameter('enable_return_home').value
        )
        self.home_frame = str(self.get_parameter('home_frame').value)
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.home_yaw = float(self.get_parameter('home_yaw').value)
        self.recover_service = str(self.get_parameter('recover_service').value)
        self.cancel_service = str(self.get_parameter('cancel_service').value)
        self.fix_stale_sec = float(
            self.get_parameter('fix_stale_sec').value
        )

        fix_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('fix_topic').value),
            self._fix_cb,
            fix_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.recover_client = self.create_client(Trigger, self.recover_service)
        self.cancel_client = self.create_client(Trigger, self.cancel_service)

        self.autonomy_active_pub = self.create_publisher(
            Bool, '/drive/autonomy_active', 10
        )
        self.navigation_stop_pub = self.create_publisher(
            Bool, '/drive/navigation_stop', 10
        )
        self.state_pub = self.create_publisher(String, '/gps/health_state', 10)
        self.readiness_pub = self.create_publisher(String, READINESS_TOPIC, 10)

        self._last_readiness = None

        self._last_fix_msg = None
        self._last_fix_mono = None
        self._phase = Phase.OK
        self._phase_entered_mono = time.monotonic()
        self._home_pose = None
        self._home_locked = False
        self._return_goal_sent = False
        self._pending_tide_home = False
        self._recover_future = None

        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.home_pose_topic = str(
            self.get_parameter('home_pose_topic').value)
        self.retreat_topic = str(self.get_parameter('retreat_topic').value)
        self._home_pose_pub = self.create_publisher(
            PoseStamped, self.home_pose_topic, latch_qos)
        self.create_subscription(
            Bool, self.retreat_topic, self._tide_retreat_cb, latch_qos)

        if math.isfinite(self.home_x) and math.isfinite(self.home_y):
            self._home_pose = self._make_home_pose(
                self.home_x, self.home_y, self.home_yaw
            )
            self._home_locked = True
            self._publish_home_pose()
            self.get_logger().info(
                f'Home from parameters: ({self.home_x:.2f}, {self.home_y:.2f}) '
                f'in {self.home_frame}'
            )

        tick = float(self.get_parameter('tick_hz').value)
        period = 1.0 / max(0.5, tick)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'gps_health_supervisor ready: stale={self.stale_sec}s '
            f'wait={self.wait_sec}s wait2={self.wait2_sec}s '
            f'return_home={self.enable_return_home} (soft recover only)'
        )

    def _fix_cb(self, msg: NavSatFix):
        self._last_fix_msg = msg
        self._last_fix_mono = time.monotonic()
        if not self._home_locked and self._fix_ok(msg):
            self._try_capture_home()

    def _publish_readiness(self):
        now = time.monotonic()
        fix_age = (
            None
            if self._last_fix_mono is None
            else now - self._last_fix_mono
        )
        available, tf_detail = self.tf_buffer.can_transform(
            self.home_frame,
            'base_link',
            Time(),
            return_debug_tuple=True,
        )
        stage, text = evaluate_navigation_readiness(
            fix_age_sec=fix_age,
            map_tf_available=bool(available),
            fix_stale_sec=self.fix_stale_sec,
            tf_detail=str(tf_detail or '').strip(),
        )

        msg = String()
        msg.data = text
        self.readiness_pub.publish(msg)

        # Publish every tick, log only on stage change: the topic carries
        # liveness, the log carries events. Comparing the stage id rather
        # than the text keeps the elapsed-seconds counters from spamming.
        if self._last_readiness != stage:
            self._last_readiness = stage
            self.get_logger().info(f'navigation readiness: {text}')

    def _fix_ok(self, msg: NavSatFix | None) -> bool:
        if msg is None:
            return False
        if int(msg.status.status) < self.min_fix_status:
            return False
        # status = -1 is NO_FIX in sensor_msgs
        if int(msg.status.status) < 0:
            return False
        return True

    def _fix_fresh(self) -> bool:
        if self._last_fix_mono is None or self._last_fix_msg is None:
            return False
        age = time.monotonic() - self._last_fix_mono
        if age > self.stale_sec:
            return False
        return self._fix_ok(self._last_fix_msg)

    def _try_capture_home(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.home_frame,
                'base_link',
                Time(),
                timeout=Duration(seconds=0.3),
            )
        except TransformException:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._home_pose = self._make_home_pose(t.x, t.y, yaw)
        self._home_locked = True
        self._publish_home_pose()
        self.get_logger().info(
            f'Home captured at good GPS: ({t.x:.2f}, {t.y:.2f}) yaw={math.degrees(yaw):.1f}deg'
        )

    def _publish_home_pose(self):
        """tide_watch 가 복귀 거리를 재도록 gps_home 을 발행한다."""
        if self._home_pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._home_pose.header.frame_id
        msg.pose = self._home_pose.pose
        self._home_pose_pub.publish(msg)

    def _tide_retreat_cb(self, msg):
        """조석 철수 True → 현재 목표 취소 후 gps_home 으로 NavigateToPose."""
        if not bool(getattr(msg, 'data', False)):
            if self._phase in (Phase.OK, Phase.FAILED):
                self._return_goal_sent = False
            return
        if not should_start_tide_return(
            self._phase.name,
            True,
            self._return_goal_sent,
            self._home_pose is not None,
        ):
            if self._home_pose is None:
                self.get_logger().error(
                    'tide retreat True but gps_home is missing'
                )
            return
        self.get_logger().warn(
            'tide retreat — cancel nav, return to gps_home'
        )
        self._pending_tide_home = True
        if self.cancel_client.service_is_ready():
            future = self.cancel_client.call_async(Trigger.Request())
            future.add_done_callback(self._after_tide_cancel)
            return
        self._pending_tide_home = False
        self._send_return_home()

    def _after_tide_cancel(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f'tide cancel: {getattr(result, "message", result)}'
            )
        except Exception as exc:
            self.get_logger().warn(f'tide cancel failed: {exc}')
        if not getattr(self, '_pending_tide_home', False):
            return
        self._pending_tide_home = False
        self._send_return_home()

    def _make_home_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.home_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _set_phase(self, phase: Phase):
        if phase == self._phase:
            return
        self.get_logger().info(
            f'GPS health phase: {self._phase.name} -> {phase.name}'
        )
        self._phase = phase
        self._phase_entered_mono = time.monotonic()
        msg = String()
        msg.data = phase.name
        self.state_pub.publish(msg)

    def _phase_age(self) -> float:
        return time.monotonic() - self._phase_entered_mono

    def _publish_stop(self, stopped: bool):
        m = Bool()
        m.data = stopped
        self.navigation_stop_pub.publish(m)

    def _publish_autonomy(self, active: bool):
        m = Bool()
        m.data = active
        self.autonomy_active_pub.publish(m)

    def _cancel_navigation(self):
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn('cancel service not ready')
            return
        req = Trigger.Request()
        future = self.cancel_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'navigation cancel: {getattr(f.result(), "message", f.result())}'
            )
        )
        self._publish_stop(True)
        self._publish_autonomy(False)

    def _call_soft_recover(self):
        if not self.recover_client.service_is_ready():
            self.get_logger().error(
                f'{self.recover_service} not ready — skip soft recover'
            )
            self._set_phase(Phase.LOST_WAIT2)
            return
        self.get_logger().warn('GPS still lost after W2 — calling soft /gps/recover')
        self._recover_future = self.recover_client.call_async(Trigger.Request())
        self._set_phase(Phase.SOFT_RECOVER)

    def _send_return_home(self):
        if not self.enable_return_home:
            self.get_logger().error('return_home disabled; marking FAILED')
            self._set_phase(Phase.FAILED)
            return
        if self._home_pose is None:
            self.get_logger().error(
                'No home pose (set home_x/home_y or capture under good GPS); FAILED'
            )
            self._set_phase(Phase.FAILED)
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose unavailable for return-home')
            self._set_phase(Phase.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._home_pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().warn(
            'return to gps_home '
            f'({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f}) '
            f'frame={goal.pose.header.frame_id}'
        )
        self._publish_stop(False)
        self._publish_autonomy(True)
        self._return_goal_sent = True
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._return_goal_response)
        self._set_phase(Phase.RETURN_HOME)

    def _return_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'return-home send failed: {exc}')
            self._set_phase(Phase.FAILED)
            self._publish_stop(True)
            self._publish_autonomy(False)
            return
        if not handle.accepted:
            self.get_logger().error('return-home goal rejected by Nav2')
            self._set_phase(Phase.FAILED)
            self._publish_stop(True)
            self._publish_autonomy(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._return_goal_result)

    def _return_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'return-home result error: {exc}')
            status = -1
        self.get_logger().info(f'return-home finished status={status}')
        self._publish_stop(True)
        self._publish_autonomy(False)
        # Stay FAILED until GPS recovers to OK (manual re-mission).
        self._set_phase(Phase.FAILED)

    def _tick(self):
        self._publish_readiness()
        fresh = self._fix_fresh()

        if self._phase == Phase.OK:
            # 아직 fix 를 한 번도 못 받은 것은 "상실"이 아니라 "미개시"다. 컨테이너
            # 기동 직후 GPS 드라이버가 첫 메시지를 내기까지 수 초가 걸리는데, 그
            # 구간을 상실로 판정하면 매 기동마다 오경보가 나고 _cancel_navigation()
            # 이 /drive/navigation_stop 을 True 로 걸어버린다. 2026-08-12 라이브에서
            # 실제로 이 오경보가 heading_calibration_bootstrap 의 창정렬 주행을
            # 시작 0.1 초 만에 중단시켰다(복구 후에도 stop 이 남은 경합은 별건).
            # 이 구간에는 아직 아무것도 주행하지 않으므로 대기가 곧 안전한 상태다.
            if self._last_fix_mono is None:
                return
            if not fresh:
                self.get_logger().error(
                    f'GPS lost (fix stale > {self.stale_sec}s) — cancel nav, W2 wait'
                )
                self._cancel_navigation()
                self._return_goal_sent = False
                self._set_phase(Phase.LOST_WAIT)
            return

        if self._phase == Phase.LOST_WAIT:
            if fresh:
                self.get_logger().info('GPS recovered during W2 wait')
                self._publish_stop(False)
                self._set_phase(Phase.OK)
                return
            if self._phase_age() >= self.wait_sec:
                self._call_soft_recover()
            return

        if self._phase == Phase.SOFT_RECOVER:
            if self._recover_future is not None and self._recover_future.done():
                try:
                    res = self._recover_future.result()
                    self.get_logger().info(
                        f'soft recover result: success={res.success} msg={res.message}'
                    )
                except Exception as exc:
                    self.get_logger().error(f'soft recover call failed: {exc}')
                self._recover_future = None
                self._set_phase(Phase.LOST_WAIT2)
            elif self._phase_age() > 15.0:
                self.get_logger().warn('soft recover call timed out; enter wait2')
                self._recover_future = None
                self._set_phase(Phase.LOST_WAIT2)
            return

        if self._phase == Phase.LOST_WAIT2:
            if fresh:
                self.get_logger().info('GPS recovered after soft recover')
                self._publish_stop(False)
                self._set_phase(Phase.OK)
                return
            if self._phase_age() >= self.wait2_sec:
                self._send_return_home()
            return

        if self._phase == Phase.RETURN_HOME:
            # Goal async; if GPS comes back mid-return, keep going home then OK later.
            if fresh:
                self.get_logger().info(
                    'GPS recovered during return-home (goal may still complete)'
                )
            return

        if self._phase == Phase.FAILED:
            if fresh:
                self.get_logger().info(
                    'GPS fresh again after FAILED — operator may send new goals'
                )
                self._set_phase(Phase.OK)
            return


def main(args=None):
    rclpy.init(args=args)
    node = GpsHealthSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
