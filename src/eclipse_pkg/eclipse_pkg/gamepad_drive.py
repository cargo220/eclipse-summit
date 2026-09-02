"""ROS2 gamepad teleoperation node for TARS manual driving."""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32, String

from eclipse_pkg.drive_control import (
    DriveControlConfig,
    DriveController,
    DriveInput,
    axis_value,
    button_pressed,
    clamp,
    normalize_trigger,
)

AUTONOMY_ACTIVE_TOPIC = '/drive/autonomy_active'


def bool_param(node: Node, name: str, default: bool) -> bool:
    node.declare_parameter(name, default)
    return bool(node.get_parameter(name).value)


def int_param(node: Node, name: str, default: int) -> int:
    node.declare_parameter(name, default)
    return int(node.get_parameter(name).value)


def float_param(node: Node, name: str, default: float) -> float:
    node.declare_parameter(name, default)
    return float(node.get_parameter(name).value)


def str_param(node: Node, name: str, default: str) -> str:
    node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


class GamepadDriveNode(Node):
    """Convert a ROS Joy message into a safety-gated Twist command."""

    def __init__(self):
        super().__init__('gamepad_drive')

        self.joy_topic = str_param(self, 'joy_topic', '/joy')  # 게임패드 입력 topic
        self.cmd_vel_topic = str_param(self, 'cmd_vel_topic', '/cmd_vel')  # 속도 명령 출력 topic
        self.drive_state_topic = str_param(
            self,
            'drive_state_topic',
            '/drive/state',
        )  # 운전 상태 출력 topic
        self.height_allowed_topic = str_param(
            self,
            'height_allowed_topic',
            '/drive/height_allowed',
        )  # 높낮이 판단 허용 출력 topic
        self.height_step_topic = str_param(
            self,
            'height_step_topic',
            '/drive/height_step_mm',
        )  # 수동 높낮이 step 출력 topic

        # 조정가능 — launch/파라미터로 덮어쓰기 가능 (기본=플랫폼 최대 선속도)
        max_linear_speeds = self._float_list_param(
            'max_linear_speeds',
            [0.7794],  # 조정가능 기본: m/s. D250·G=2·130 tick ≈ PLATFORM_MAX_LINEAR_MPS
        )  # 속도 모드별 최대 선속도 목록
        config = DriveControlConfig(
            max_linear_speeds=tuple(max_linear_speeds),  # 조정가능: 모드별 최대 선속도
            max_angular_speed=float_param(self, 'max_angular_speed', 0.6),  # 조정가능: 최대 각속도
            max_linear_accel=float_param(self, 'max_linear_accel', 0.10),  # 조정가능: 선가속 m/s²
            max_linear_decel=float_param(self, 'max_linear_decel', 0.25),  # 조정가능: 선감속 m/s²
            # 각가속 기본 0.7 — velocity_smoother max_accel[2] 와 맞춤 (예전 1.0)
            max_angular_accel=float_param(self, 'max_angular_accel', 0.7),  # 조정가능: 각가속 rad/s²
            accel_deadband=float_param(self, 'accel_deadband', 0.05),  # 액셀 무시 구간
            brake_deadband=float_param(self, 'brake_deadband', 0.05),  # 브레이크 무시 구간
            steer_deadband=float_param(self, 'steer_deadband', 0.08),  # 조향 무시 구간
            cruise_linear_tolerance=float_param(
                self,
                'cruise_linear_tolerance',
                0.01,
            ),  # 정속 판단용 선속도 변화 허용값
            cruise_angular_limit=float_param(self, 'cruise_angular_limit', 0.03),  # 직진 판단 각속도 상한
            cruise_min_duration=float_param(self, 'cruise_min_duration', 1.0),  # 정속 지속 시간
            min_height_speed=float_param(self, 'min_height_speed', 0.05),  # 높이 판단 최소 속도
            input_timeout_sec=float_param(self, 'input_timeout_sec', 0.3),  # 입력 timeout 기준
        )
        self.controller = DriveController(config)  # 수동운전 명령 필터

        # Defaults match the Xbox 360 receiver as published by game_controller_node.
        self.axis_steer = int_param(self, 'axis_steer', 3)  # 조향 스틱 축 번호
        self.axis_brake = int_param(self, 'axis_brake', 4)  # 브레이크 트리거 축 번호
        self.axis_accel = int_param(self, 'axis_accel', 5)  # 액셀 트리거 축 번호
        self.button_stop = int_param(self, 'button_stop', 0)  # 정지 버튼 번호
        self.button_enable = int_param(self, 'button_enable', 6)  # 수동운전 enable 버튼 번호
        self.button_disable = int_param(self, 'button_disable', 4)  # 수동운전 disable 버튼 번호
        self.button_deadman = int_param(self, 'button_deadman', 9)  # deadman 버튼 번호
        self.button_speed_mode = int_param(self, 'button_speed_mode', 7)  # 속도 모드 변경 버튼 번호
        self.button_speed_down = int_param(self, 'button_speed_down', 2)  # 버튼식 목표 속도 감소 버튼
        self.button_speed_up = int_param(self, 'button_speed_up', 1)  # 버튼식 목표 속도 증가 버튼
        self.button_speed_reset = int_param(self, 'button_speed_reset', 3)  # 버튼식 목표 속도 0 리셋 버튼
        self.button_linear_step = float_param(self, 'button_linear_step', 0.05)  # 버튼 1회당 선속도 변화량
        self.axis_height_step = int_param(self, 'axis_height_step', 7)  # D-pad 상하 높낮이 축 번호
        self.height_axis_deadband = float_param(self, 'height_axis_deadband', 0.5)  # D-pad 축 입력 판정 기준
        self.height_axis_up_value = float_param(self, 'height_axis_up_value', 1.0)  # D-pad 위쪽 입력 부호
        self.height_step_mm = float_param(self, 'height_step_mm', 3.0)  # 높낮이 버튼 1회당 down 거리 변화량
        self.steer_sign = float_param(self, 'steer_sign', 1.0)  # 조향 방향 반전 계수
        self.trigger_idle_value = float_param(self, 'trigger_idle_value', 0.0)  # 트리거 미입력 축 값
        self.trigger_pressed_value = float_param(
            self,
            'trigger_pressed_value',
            -1.0,
        )  # 트리거 완전 입력 축 값
        self.require_enable_button = bool_param(
            self,
            'require_enable_button',
            False,
        )  # enable 버튼을 별도로 요구할지 여부
        self.manual_enabled = not self.require_enable_button  # 현재 수동운전 enable 상태

        self.last_input = DriveInput(enabled=False)  # 마지막으로 해석된 게임패드 입력
        self.button_linear_target = 0.0  # 버튼식 속도 스텝이 유지하는 목표 선속도
        self.last_joy_time: float | None = None  # 마지막 Joy 메시지 수신 시간
        self.previous_buttons: list[int] = []  # rising edge 검출용 직전 버튼 배열
        self.previous_height_axis_direction = 0  # D-pad 높낮이 rising edge 검출 상태
        # 유휴 상태에서 0 Twist를 매 tick 재발행하면 수동 `ros2 topic pub` 테스트 등
        # 다른 /cmd_vel 퍼블리셔와 경합한다. 0 상태로 "전환"될 때만 한 번 발행하고,
        # 이미 0인 동안은 재발행하지 않는다(2026-08-10 docker_2/docker_3 병합).
        self._last_cmd_vel_was_zero = True

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.drive_state_pub = self.create_publisher(String, self.drive_state_topic, 10)
        self.height_allowed_pub = self.create_publisher(
            Bool,
            self.height_allowed_topic,
            10,
        )
        self.height_step_pub = self.create_publisher(
            Float32,
            self.height_step_topic,
            10,
        )
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        # Nav2 주행 동안(True) /cmd_vel 발행을 멈춰서 자율주행 명령과
        # 충돌하지 않게 한다. 평소(False)엔 원격조종이 기본값.
        self.autonomy_active = False
        self.create_subscription(
            Bool, AUTONOMY_ACTIVE_TOPIC, self._autonomy_active_callback, 10
        )

        # 2026-08-11: 10.0 -> 20.0 복원. load tuning 캠페인(커밋 91475d0a)에서
        # 20 -> 10 으로 낮췄으나 그 전제가 실측으로 반증됐다 — 부하의 원인은 발행
        # 비용이 아니라 노드 개수였고, CPU 는 70% 놀고 있었다. 이 경로는 조종자가
        # 자율주행을 수동으로 뺏는 경로라 응답 지연이 곧 안전 문제다(10Hz = 최대
        # 100ms 지연). 되돌리기 전에 부하 여유를 실측으로 확인했다.
        publish_rate_hz = float_param(self, 'publish_rate_hz', 20.0)
        timer_period = 1.0 / max(1.0, publish_rate_hz)
        self.create_timer(timer_period, self.publish_drive_command)

        self.get_logger().info(
            f'gamepad_drive ready: /joy -> {self.cmd_vel_topic}, '
            f'deadman button={self.button_deadman}'
        )

    def _float_list_param(self, name: str, default: list[float]) -> list[float]:
        self.declare_parameter(name, default)
        values = self.get_parameter(name).value
        parsed = [float(value) for value in values]
        if not parsed:
            raise ValueError(f'{name} must contain at least one speed value')
        return parsed

    def joy_callback(self, msg: Joy) -> None:
        now_sec = self._now_sec()  # 현재 ROS 시간
        self.last_joy_time = now_sec

        if (
            len(self.controller.config.max_linear_speeds) > 1
            and self._rising_edge(msg.buttons, self.button_speed_mode)
        ):
            mode = self.controller.next_speed_mode()  # 변경된 속도 모드 번호
            self.button_linear_target = clamp(
                self.button_linear_target,
                -self._current_speed_limit(),
                self._current_speed_limit(),
            )  # 속도 모드 상한이 낮아지면 버튼 목표 속도도 함께 제한
            self.get_logger().info(f'manual speed mode changed: {mode}')

        if self._rising_edge(msg.buttons, self.button_enable):
            self.manual_enabled = True
            self.get_logger().info('manual drive enabled')

        if self._rising_edge(msg.buttons, self.button_disable):
            self.manual_enabled = False
            self.get_logger().info('manual drive disabled')

        deadman_pressed = button_pressed(msg.buttons, self.button_deadman)  # deadman 버튼 입력 여부
        stop_pressed = button_pressed(msg.buttons, self.button_stop)  # 정지 버튼 입력 여부
        if stop_pressed and self.require_enable_button:
            self.manual_enabled = False

        accel = normalize_trigger(
            axis_value(msg.axes, self.axis_accel),
            idle_value=self.trigger_idle_value,
            pressed_value=self.trigger_pressed_value,
            deadband=self.controller.config.accel_deadband,
        )  # 0에서 1 사이로 정규화한 액셀 입력
        brake = normalize_trigger(
            axis_value(msg.axes, self.axis_brake),
            idle_value=self.trigger_idle_value,
            pressed_value=self.trigger_pressed_value,
            deadband=self.controller.config.brake_deadband,
        )  # 0에서 1 사이로 정규화한 브레이크 입력
        steer = self.steer_sign * axis_value(msg.axes, self.axis_steer)  # 방향 보정된 조향 입력
        height_axis_direction = self._height_axis_direction(msg.axes)
        if stop_pressed or brake > 0.0 or not deadman_pressed or not self.manual_enabled:
            self.button_linear_target = 0.0  # 안전 조건이 깨지면 버튼식 목표 속도를 남기지 않는다.
        else:
            self._update_button_linear_target(msg.buttons)
            self._publish_height_step_on_edge(height_axis_direction)
        self.previous_height_axis_direction = height_axis_direction

        target_linear = None
        if abs(self.button_linear_target) > 0.0:
            target_linear = self.button_linear_target

        self.last_input = DriveInput(
            accel=accel,
            brake=brake,
            steer=steer,
            target_linear=target_linear,
            deadman=deadman_pressed,
            stop=stop_pressed,
            enabled=self.manual_enabled,
        )
        self.previous_buttons = list(msg.buttons)

    def _update_button_linear_target(self, buttons: list[int]) -> None:
        speed_limit = self._current_speed_limit()
        if self._rising_edge(buttons, self.button_speed_reset):
            self.button_linear_target = 0.0
        if self._rising_edge(buttons, self.button_speed_down):
            self.button_linear_target = max(
                -speed_limit,
                self.button_linear_target - self.button_linear_step,
            )
        if self._rising_edge(buttons, self.button_speed_up):
            self.button_linear_target = min(
                speed_limit,
                self.button_linear_target + self.button_linear_step,
            )

    def _height_axis_direction(self, axes: list[float]) -> int:
        value = axis_value(axes, self.axis_height_step)
        if abs(value) < self.height_axis_deadband:
            return 0
        if value * self.height_axis_up_value > 0:
            return -1  # D-pad 위쪽은 바퀴축을 올리므로 down_mm를 줄인다.
        return 1  # D-pad 아래쪽은 바퀴축을 내리므로 down_mm를 늘린다.

    def _publish_height_step_on_edge(self, direction: int) -> None:
        if direction == 0 or self.previous_height_axis_direction != 0:
            return

        msg = Float32()
        msg.data = float(direction) * self.height_step_mm
        self.height_step_pub.publish(msg)

    def _current_speed_limit(self) -> float:
        return self.controller.config.max_linear_speeds[
            self.controller.speed_mode_index
        ]

    def _autonomy_active_callback(self, msg: Bool) -> None:
        self.autonomy_active = bool(msg.data)

    def publish_drive_command(self) -> None:
        if self.autonomy_active:
            # 자율주행(Nav2)이 /cmd_vel을 담당하는 동안은
            # 게임패드가 끼어들어 명령을 덮어쓰지 않도록 발행을 멈춘다.
            self.get_logger().warn(
                'gamepad /cmd_vel suppressed: /drive/autonomy_active is True',
                throttle_duration_sec=2.0,
            )
            return

        now_sec = self._now_sec()  # 현재 ROS 시간
        if self.last_joy_time is None:
            input_age_sec = math.inf  # 아직 입력이 없으면 timeout 상태로 취급
        else:
            input_age_sec = now_sec - self.last_joy_time  # 마지막 입력 이후 지난 시간

        output = self.controller.step(
            self.last_input,
            now_sec=now_sec,
            input_age_sec=input_age_sec,
        )  # 안전 게이트와 rate limit을 통과한 운전 출력

        is_zero = output.linear == 0.0 and output.angular == 0.0
        if not (is_zero and self._last_cmd_vel_was_zero):
            twist = Twist()
            twist.linear.x = output.linear
            twist.angular.z = output.angular
            self.cmd_vel_pub.publish(twist)
        self._last_cmd_vel_was_zero = is_zero

        joy_fresh = input_age_sec <= self.controller.config.input_timeout_sec
        if not is_zero:
            self.get_logger().info(
                f'gamepad cmd_vel linear={output.linear:.3f} '
                f'angular={output.angular:.3f} state={output.state}',
                throttle_duration_sec=1.0,
            )
        elif joy_fresh and not self.last_input.deadman:
            self.get_logger().warn(
                'gamepad cmd_vel=0: hold deadman button '
                f'{self.button_deadman}',
                throttle_duration_sec=2.0,
            )
        elif joy_fresh and self.last_input.stop:
            self.get_logger().warn(
                'gamepad cmd_vel=0: stop button pressed',
                throttle_duration_sec=2.0,
            )

        state_msg = String()
        state_msg.data = output.state
        self.drive_state_pub.publish(state_msg)

        allowed_msg = Bool()
        allowed_msg.data = output.height_allowed
        self.height_allowed_pub.publish(allowed_msg)

    def _rising_edge(self, buttons: list[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        was_pressed = button_pressed(self.previous_buttons, index)
        return bool(buttons[index]) and not was_pressed

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = GamepadDriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
