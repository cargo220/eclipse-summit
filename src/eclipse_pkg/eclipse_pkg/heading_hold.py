"""Heading-hold math helpers for straight driving."""

import math


def quaternion_to_yaw(x, y, z, w):
    """Convert a normalized quaternion into yaw in radians."""
    siny_cosp = 2.0 * (w * z + x * y)  # yaw atan2의 sin 쪽 항
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)  # yaw atan2의 cos 쪽 항
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize an angle to the [-pi, pi] range."""
    return math.atan2(math.sin(angle), math.cos(angle))


def calculate_heading_hold_correction(
    cmd_v,
    cmd_w,
    has_imu_yaw,
    imu_yaw,
    heading_target_yaw,
    min_cmd_vel,
    cmd_w_threshold,
    kp,
    max_w,
):
    """Return a correction angular velocity and updated heading target."""
    if (
        abs(cmd_v) < min_cmd_vel
        or abs(cmd_w) >= cmd_w_threshold
        or not has_imu_yaw
    ):
        return 0.0, None

    if heading_target_yaw is None:
        return 0.0, imu_yaw

    yaw_error = normalize_angle(heading_target_yaw - imu_yaw)  # 목표 yaw와 현재 yaw 차이
    correction_w = kp * yaw_error  # yaw 오차 기반 보정 각속도
    correction_w = max(-max_w, min(max_w, correction_w))  # 보정 각속도 제한
    return correction_w, heading_target_yaw
