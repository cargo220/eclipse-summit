"""Manual drive command shaping for TARS gamepad teleoperation."""

from dataclasses import dataclass
from typing import Sequence

from eclipse_pkg.eclipse_test_config import PLATFORM_MAX_LINEAR_MPS


DRIVE_STATE_STOPPED = 'STOPPED'  # 정지 상태
DRIVE_STATE_ACCELERATING = 'ACCELERATING'  # 가속 또는 목표 속도 변화 중
DRIVE_STATE_CRUISING = 'CRUISING'  # 안정적인 정속 직진 상태
DRIVE_STATE_BRAKING = 'BRAKING'  # 브레이크 입력 상태
DRIVE_STATE_TURNING = 'TURNING'  # 조향 입력이 큰 회전 상태


@dataclass(frozen=True)
class DriveControlConfig:
    # 조정가능 — 수동/상한 속도·가감속
    # 선가속/감속은 수동이 자율 smoother(0.30/0.40)보다 보수적(0.10/0.25).
    # 각가속은 smoother wz 축(0.7)과 맞춤 (예전 1.0은 갯벌에 여유 과다).
    max_linear_speeds: tuple[float, ...] = (PLATFORM_MAX_LINEAR_MPS,)  # 조정가능: 최대 선속도(m/s). D250·G2.5·모터 130 tick
    max_angular_speed: float = 0.6  # 조정가능: 최대 각속도(rad/s)
    max_linear_accel: float = 0.10  # 조정가능: 선가속 상한 (m/s²)
    max_linear_decel: float = 0.25  # 조정가능: 선감속 상한 (m/s²)
    max_angular_accel: float = 0.7  # 조정가능: 각가속 상한 (rad/s²), velocity_smoother 와 정합
    accel_deadband: float = 0.05  # 액셀 무시 구간
    brake_deadband: float = 0.05  # 브레이크 무시 구간
    steer_deadband: float = 0.08  # 조향 스틱 무시 구간
    cruise_linear_tolerance: float = 0.01  # 정속 판단용 선속도 변화 허용값
    cruise_angular_limit: float = 0.03  # 정속 직진으로 보는 각속도 상한
    cruise_min_duration: float = 1.0  # 정속 상태로 인정하기 위한 지속 시간
    min_height_speed: float = 0.05  # 높이 판단을 허용하는 최소 주행 속도
    input_timeout_sec: float = 0.3  # joy 입력 timeout 기준


@dataclass(frozen=True)
class DriveInput:
    accel: float = 0.0  # 액셀 입력 비율
    brake: float = 0.0  # 브레이크 입력 비율
    steer: float = 0.0  # 조향 입력 비율
    target_linear: float | None = None  # 버튼식 속도 스텝이 직접 지정한 목표 선속도
    deadman: bool = False  # 누르고 있어야 주행 가능한 안전 버튼 상태
    stop: bool = False  # 즉시 정지 버튼 상태
    enabled: bool = True  # 수동운전 enable 상태


@dataclass(frozen=True)
class DriveOutput:
    linear: float  # 최종 cmd_vel 선속도
    angular: float  # 최종 cmd_vel 각속도
    state: str  # 현재 운전 상태 문자열
    height_allowed: bool  # 높낮이 알고리즘 판단 허용 여부
    speed_mode_index: int  # 현재 속도 모드 번호


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def block_autonomy_reverse(
    linear_velocity: float,
    autonomy_active: bool,
) -> float:
    """Prevent reverse translation during autonomous navigation only."""
    if autonomy_active and linear_velocity < 0.0:
        return 0.0
    return linear_velocity


def apply_navigation_stop(
    linear_velocity: float,
    angular_velocity: float,
    stop_latched: bool,
) -> tuple[float, float]:
    """Block all wheel motion while the navigation stop latch is set."""
    if stop_latched:
        return 0.0, 0.0
    return linear_velocity, angular_velocity


def scale_to_wheel_limit(
    left_ticks: float,
    right_ticks: float,
    limit_ticks: float,
) -> tuple[float, float, float]:
    """Shrink both wheel commands together when either exceeds the limit.

    Returns ``(left, right, scale)``.

    A differential drive turns according to the RATIO of its two wheel speeds.
    If only the faster wheel gets clipped — which is what the motor firmware
    does on its own — that ratio changes, so the robot silently drives a
    different arc than the one commanded: it under-rotates AND under-travels,
    Nav2 sees the resulting path error and steers harder, and the two can
    chase each other. Scaling both wheels by the same factor keeps the ratio,
    so the arc is preserved and only the speed along it drops.

    This matters at ordinary speeds, not just extreme ones: with a ~0.9742 m/s
    wheel limit, commanding 0.97 m/s along a gentle 2 m-radius curve already
    asks the outer wheel past the limit. Nav2's regulated pure pursuit only
    slows for curves tighter than regulated_linear_scaling_min_radius (0.9 m),
    so the gentle-curve case is exactly the one nothing else guards.

    ``limit_ticks <= 0`` means the limit is unknown (the startup read from the
    Dynamixel failed and stored 0); nothing is scaled in that case, because
    inventing a limit would be worse than leaving the firmware to clip.
    """
    peak = max(abs(left_ticks), abs(right_ticks))
    if limit_ticks <= 0.0 or peak <= limit_ticks:
        return left_ticks, right_ticks, 1.0
    scale = limit_ticks / peak
    return left_ticks * scale, right_ticks * scale, scale


def axis_value(values: Sequence[float], index: int, default: float = 0.0) -> float:
    if index < 0 or index >= len(values):
        return default
    return float(values[index])


def button_pressed(values: Sequence[int], index: int) -> bool:
    if index < 0 or index >= len(values):
        return False
    return bool(values[index])


def normalize_trigger(
    value: float,
    idle_value: float = 1.0,
    pressed_value: float = -1.0,
    deadband: float = 0.05,
) -> float:
    if pressed_value == idle_value:
        return 0.0
    normalized = (value - idle_value) / (pressed_value - idle_value)
    normalized = clamp(normalized, 0.0, 1.0)
    if normalized < deadband:
        return 0.0
    return normalized


def apply_signed_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return clamp(value, -1.0, 1.0)


def limit_step(current: float, target: float, max_delta: float) -> float:
    return current + clamp(target - current, -max_delta, max_delta)


def limit_linear_step(
    current: float,
    target: float,
    max_accel_delta: float,
    max_decel_delta: float,
) -> float:
    if current == target:
        return current

    if current != 0.0 and target != 0.0 and current * target < 0.0:
        if abs(current) <= max_decel_delta:
            return 0.0
        if current > 0.0:
            return current - max_decel_delta
        return current + max_decel_delta

    if target == 0.0 or abs(target) < abs(current):
        return limit_step(current, target, max_decel_delta)
    return limit_step(current, target, max_accel_delta)


class DriveController:
    """Stateful rate limiter and safety gate for manual drive commands."""

    def __init__(self, config: DriveControlConfig | None = None):
        self.config = config or DriveControlConfig()  # 운전 제한 파라미터 묶음
        self.linear = 0.0  # 현재 출력 선속도
        self.angular = 0.0  # 현재 출력 각속도
        self.speed_mode_index = 0  # 현재 속도 모드 번호
        self.last_time: float | None = None  # 직전 step 계산 시간
        self.last_target_linear = 0.0  # 직전 목표 선속도
        self.last_target_angular = 0.0  # 직전 목표 각속도
        self.stable_since: float | None = None  # 입력이 안정적으로 유지되기 시작한 시간

    def set_speed_mode(self, index: int) -> None:
        max_index = len(self.config.max_linear_speeds) - 1
        self.speed_mode_index = int(clamp(index, 0, max_index))

    def next_speed_mode(self) -> int:
        self.speed_mode_index = (
            self.speed_mode_index + 1
        ) % len(self.config.max_linear_speeds)
        return self.speed_mode_index

    def step(
        self,
        drive_input: DriveInput,
        now_sec: float,
        input_age_sec: float = 0.0,
    ) -> DriveOutput:
        dt = self._dt(now_sec)  # 이번 제어 계산에 사용할 시간 간격

        safety_stop = (
            not drive_input.enabled
            or drive_input.stop
            or not drive_input.deadman
            or input_age_sec > self.config.input_timeout_sec
        )  # 안전 조건 중 하나라도 깨지면 즉시 정지
        if safety_stop:
            self._reset_motion(now_sec)
            return self._output(DRIVE_STATE_STOPPED, False)

        accel = clamp(drive_input.accel, 0.0, 1.0)  # 제한된 액셀 입력
        brake = clamp(drive_input.brake, 0.0, 1.0)  # 제한된 브레이크 입력
        steer = apply_signed_deadband(
            drive_input.steer,
            self.config.steer_deadband,
        )  # deadband를 적용한 조향 입력

        speed_limit = self.config.max_linear_speeds[self.speed_mode_index]  # 현재 모드 최대 속도
        if brake > self.config.brake_deadband:
            target_linear = 0.0  # 브레이크 중에는 목표 선속도 0
            state = DRIVE_STATE_BRAKING
        else:
            if drive_input.target_linear is None:
                if accel < self.config.accel_deadband:
                    accel = 0.0
                target_linear = speed_limit * accel  # 액셀 비율을 반영한 목표 선속도
            else:
                target_linear = clamp(
                    drive_input.target_linear,
                    -speed_limit,
                    speed_limit,
                )  # 버튼식 목표 속도
            if abs(target_linear) <= self.config.accel_deadband * speed_limit:
                target_angular = 0.0  # 액셀이 없을 때 제자리 조향 방지
            else:
                target_angular = steer * self.config.max_angular_speed  # 목표 각속도
            state = self._drive_state(target_linear, target_angular, now_sec)

        if state == DRIVE_STATE_BRAKING:
            target_angular = 0.0
        self._update_stability(target_linear, target_angular, now_sec)

        max_linear_accel_delta = self.config.max_linear_accel * dt
        max_linear_decel_delta = self.config.max_linear_decel * dt
        max_angular_delta = self.config.max_angular_accel * dt

        self.linear = limit_linear_step(
            self.linear,
            target_linear,
            max_linear_accel_delta,
            max_linear_decel_delta,
        )
        self.angular = limit_step(self.angular, target_angular, max_angular_delta)

        height_allowed = self._height_allowed(state, now_sec)  # 높이 판단 허용 여부
        return self._output(state, height_allowed)

    def _dt(self, now_sec: float) -> float:
        if self.last_time is None:
            self.last_time = now_sec
            return 0.0
        dt = max(0.0, now_sec - self.last_time)
        self.last_time = now_sec
        return dt

    def _reset_motion(self, now_sec: float) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.last_time = now_sec
        self.last_target_linear = 0.0
        self.last_target_angular = 0.0
        self.stable_since = None

    def _drive_state(
        self,
        target_linear: float,
        target_angular: float,
        now_sec: float,
    ) -> str:
        if abs(target_linear) < self.config.accel_deadband * self.config.max_linear_speeds[-1]:
            return DRIVE_STATE_STOPPED
        if abs(target_angular) > self.config.cruise_angular_limit:
            return DRIVE_STATE_TURNING
        if self._stable_duration(now_sec) >= self.config.cruise_min_duration:
            return DRIVE_STATE_CRUISING
        return DRIVE_STATE_ACCELERATING

    def _update_stability(
        self,
        target_linear: float,
        target_angular: float,
        now_sec: float,
    ) -> None:
        changed = (
            abs(target_linear - self.last_target_linear)
            > self.config.cruise_linear_tolerance
            or abs(target_angular - self.last_target_angular)
            > self.config.cruise_angular_limit
        )
        if changed or self.stable_since is None:
            self.stable_since = now_sec
        self.last_target_linear = target_linear
        self.last_target_angular = target_angular

    def _stable_duration(self, now_sec: float) -> float:
        if self.stable_since is None:
            return 0.0
        return max(0.0, now_sec - self.stable_since)

    def _height_allowed(self, state: str, now_sec: float) -> bool:
        return (
            state == DRIVE_STATE_CRUISING
            and self.linear >= self.config.min_height_speed
            and abs(self.angular) <= self.config.cruise_angular_limit
            and self._stable_duration(now_sec) >= self.config.cruise_min_duration
        )

    def _output(self, state: str, height_allowed: bool) -> DriveOutput:
        return DriveOutput(
            linear=self.linear,
            angular=self.angular,
            state=state,
            height_allowed=height_allowed,
            speed_mode_index=self.speed_mode_index,
        )
