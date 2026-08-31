"""Current-variation based odometry covariance helpers."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CurrentCovarianceConfig:
    normal_std: float  # 기본 정상 지형 전류 표준편차 기준
    cov_gain: float  # 전류 흔들림을 공분산 증가량으로 바꾸는 gain
    terrain_samples: int  # shock와 continuous를 나누는 샘플 수
    continuous_max_cov: float  # 연속 rough 지형에서의 공분산 상한
    baseline_alpha: float  # 전류 baseline 갱신 비율
    baseline_margin: float  # 학습 baseline에 곱하는 여유 배율
    baseline_max_std: float  # 학습 baseline 최대값
    baseline_std_step_limit: float  # baseline 갱신을 막는 표준편차 변화량
    baseline_min_cmd_v: float  # baseline 갱신을 허용하는 최소 명령 속도
    baseline_max_cmd_w: float  # baseline 갱신을 허용하는 최대 명령 각속도
    baseline_min_wheel_speed: float  # baseline 갱신을 허용하는 최소 wheel 속도
    shock_std_step: float  # shock로 보는 전류 표준편차 증가량
    shock_ratio: float  # shock로 보는 기준 대비 흔들림 비율
    base_cov: float  # 기본 odom twist 공분산
    max_cov: float  # 공분산 최댓값


@dataclass(frozen=True)
class CurrentCovarianceState:
    baseline: float  # 학습된 정상 전류 흔들림 baseline
    exceed_count: int  # 기준 초과가 연속된 횟수
    rough_count: int  # 거친 지형으로 본 누적 횟수
    shock_active: bool  # 현재 shock 상태가 유지되는지 여부
    prev_avg_std: float | None  # 직전 좌우 평균 전류 표준편차
    baseline_sample_count: int  # baseline 갱신에 사용된 샘플 수


@dataclass(frozen=True)
class CurrentCovarianceResult:
    dynamic_cov: float  # 최종 odom twist 공분산
    current_delta: float  # 실제 적용한 공분산 증가량
    current_raw_delta: float  # 제한을 걸기 전 공분산 증가량
    current_mode: str  # 전류 기반 지형 모드
    effective_normal_std: float  # baseline을 반영한 정상 표준편차 기준
    baseline_updated: bool  # 이번 계산에서 baseline을 갱신했는지 여부
    baseline_update_reason: str  # baseline 갱신 또는 보류 이유
    state: CurrentCovarianceState  # 다음 계산에 넘길 공분산 내부 상태


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def apply_gps_good_wheel_covariance_floor(
    current_cov,
    gps_good,
    gps_good_cov,
    max_cov,
):
    if not gps_good:
        return current_cov
    return clamp(max(current_cov, gps_good_cov), current_cov, max_cov)


def gps_fix_status_is_good(fix_status, minimum_good_status):
    """Return whether a NavSatStatus meets the corrected-GNSS threshold."""
    if fix_status is None:
        return False
    return int(fix_status) >= int(minimum_good_status)


def effective_current_normal_std(baseline, normal_std, baseline_margin):
    return max(normal_std, baseline * baseline_margin)


def calculate_current_std_step(avg_std, prev_avg_std):
    if prev_avg_std is None:
        return 0.0
    return avg_std - prev_avg_std


def _maybe_update_baseline(
    avg_std,
    std_step,
    current_mode,
    samples_ready,
    cmd_v,
    cmd_w,
    wheel_odom_speed,
    state,
    config,
):
    # 안정 주행 조건에서만 정상 지형 전류 흔들림 baseline을 천천히 갱신한다.
    if current_mode == "shock":
        return state, False, "shock"
    if not samples_ready:
        return state, False, "samples_not_ready"
    if abs(cmd_v) < config.baseline_min_cmd_v:
        return state, False, "cmd_v_low"
    if abs(cmd_w) >= config.baseline_max_cmd_w:
        return state, False, "cmd_w_high"
    if wheel_odom_speed < config.baseline_min_wheel_speed:
        return state, False, "wheel_speed_low"
    if abs(std_step) >= config.baseline_std_step_limit:
        return state, False, "std_step_high"
    if not math.isfinite(avg_std):
        return state, False, "avg_std_invalid"

    baseline = clamp(
        ((1.0 - config.baseline_alpha) * state.baseline)
        + (config.baseline_alpha * avg_std),
        config.normal_std,
        config.baseline_max_std,
    )
    return (
        CurrentCovarianceState(
            baseline=baseline,
            exceed_count=state.exceed_count,
            rough_count=state.rough_count,
            shock_active=state.shock_active,
            prev_avg_std=state.prev_avg_std,
            baseline_sample_count=state.baseline_sample_count + 1,
        ),
        True,
        "updated",
    )


def calculate_current_covariance(
    avg_std,
    state,
    config,
    samples_ready,
    cmd_v,
    cmd_w,
    wheel_odom_speed,
):
    """Calculate covariance and updated state from current std."""
    effective_normal_std = effective_current_normal_std(
        state.baseline,
        config.normal_std,
        config.baseline_margin,
    )  # 학습 baseline까지 반영한 현재 정상 기준
    std_step = calculate_current_std_step(avg_std, state.prev_avg_std)  # 직전 샘플 대비 변화량

    if avg_std <= config.normal_std:
        next_state = CurrentCovarianceState(
            baseline=state.baseline,  # 기존 baseline 유지
            exceed_count=0,  # 기준 초과 카운트 초기화
            rough_count=0,  # rough 카운트 초기화
            shock_active=False,  # shock 해제
            prev_avg_std=state.prev_avg_std,  # 아래에서 현재 값으로 교체
            baseline_sample_count=state.baseline_sample_count,  # baseline 샘플 수 유지
        )
        next_state, baseline_updated, baseline_update_reason = _maybe_update_baseline(
            avg_std,
            std_step,
            "normal",
            samples_ready,
            cmd_v,
            cmd_w,
            wheel_odom_speed,
            next_state,
            config,
        )
        next_state = CurrentCovarianceState(
            **{**next_state.__dict__, "prev_avg_std": avg_std}
        )
        return CurrentCovarianceResult(
            dynamic_cov=config.base_cov,
            current_delta=0.0,
            current_raw_delta=0.0,
            current_mode="normal",
            effective_normal_std=effective_normal_std,
            baseline_updated=baseline_updated,
            baseline_update_reason=baseline_update_reason,
            state=next_state,
        )

    rough_count = state.rough_count + 1  # 정상 기준보다 큰 흔들림이 나온 누적 횟수

    if avg_std <= effective_normal_std:
        next_state = CurrentCovarianceState(
            baseline=state.baseline,  # learned normal 후보 baseline
            exceed_count=0,  # 유효 기준 안쪽이므로 기준 초과 카운트 초기화
            rough_count=rough_count,  # rough 관찰 횟수는 유지
            shock_active=False,  # shock 해제
            prev_avg_std=state.prev_avg_std,  # 아래에서 현재 값으로 교체
            baseline_sample_count=state.baseline_sample_count,  # baseline 샘플 수 유지
        )
        next_state, baseline_updated, baseline_update_reason = _maybe_update_baseline(
            avg_std,
            std_step,
            "learned_normal",
            samples_ready,
            cmd_v,
            cmd_w,
            wheel_odom_speed,
            next_state,
            config,
        )
        next_state = CurrentCovarianceState(
            **{**next_state.__dict__, "prev_avg_std": avg_std}
        )
        return CurrentCovarianceResult(
            dynamic_cov=config.base_cov,
            current_delta=0.0,
            current_raw_delta=0.0,
            current_mode="learned_normal",
            effective_normal_std=effective_normal_std,
            baseline_updated=baseline_updated,
            baseline_update_reason=baseline_update_reason,
            state=next_state,
        )

    if state.exceed_count == 0:
        shock_active = (
            rough_count < config.terrain_samples
            and (
                std_step >= config.shock_std_step
                or avg_std >= effective_normal_std * config.shock_ratio
            )
        )
    else:
        shock_active = state.shock_active

    exceed_count = state.exceed_count + 1  # 유효 정상 기준을 초과한 연속 횟수
    excess_std = avg_std - effective_normal_std  # 정상 기준을 넘어선 전류 흔들림
    raw_delta = (excess_std**2 * config.cov_gain) + 1.0  # 제한 전 공분산 증가량

    if not shock_active or exceed_count >= config.terrain_samples:
        max_delta = max(config.continuous_max_cov - config.base_cov, 0.0)  # continuous 상한 여유
        current_delta = min(raw_delta, max_delta)  # continuous에서는 증가량을 제한
        current_mode = "continuous"  # 지속적으로 거친 지형
    else:
        current_delta = raw_delta  # 순간 충격은 raw 증가량 사용
        current_mode = "shock"  # 짧고 큰 충격

    next_state = CurrentCovarianceState(
        baseline=state.baseline,
        exceed_count=exceed_count,
        rough_count=rough_count,
        shock_active=shock_active,
        prev_avg_std=state.prev_avg_std,
        baseline_sample_count=state.baseline_sample_count,
    )
    next_state, baseline_updated, baseline_update_reason = _maybe_update_baseline(
        avg_std,
        std_step,
        current_mode,
        samples_ready,
        cmd_v,
        cmd_w,
        wheel_odom_speed,
        next_state,
        config,
    )
    next_state = CurrentCovarianceState(
        **{**next_state.__dict__, "prev_avg_std": avg_std}
    )
    dynamic_cov = clamp(
        config.base_cov + current_delta,
        config.base_cov,
        config.max_cov,
    )
    return CurrentCovarianceResult(
        dynamic_cov=dynamic_cov,
        current_delta=current_delta,
        current_raw_delta=raw_delta,
        current_mode=current_mode,
        effective_normal_std=effective_normal_std,
        baseline_updated=baseline_updated,
        baseline_update_reason=baseline_update_reason,
        state=next_state,
    )
