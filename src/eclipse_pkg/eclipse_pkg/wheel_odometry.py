"""Differential-drive wheel odometry helpers."""

import math


def differential_drive_twist(left_velocity, right_velocity, wheel_separation):
    """Return linear and angular body twist from left/right wheel speeds."""
    linear = (left_velocity + right_velocity) / 2.0
    angular = (right_velocity - left_velocity) / wheel_separation
    return linear, angular


def integrate_pose(x, y, theta, linear_velocity, angular_velocity, dt_sec):
    """Integrate planar odometry pose for one time step."""
    delta_x = (linear_velocity * math.cos(theta)) * dt_sec
    delta_y = (linear_velocity * math.sin(theta)) * dt_sec
    delta_theta = angular_velocity * dt_sec
    return x + delta_x, y + delta_y, theta + delta_theta
