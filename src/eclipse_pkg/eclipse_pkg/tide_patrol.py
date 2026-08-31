"""Tide-window mudflat wander helpers.

Patrol time is the existing access window from plan_tide_access:
phase==accessible after the ebb enter, until leave_now / retreat.
It is not a Foxglove click loop.
"""

import json
import math
import random

from eclipse_pkg.tide_plan import nearest_on_polyline

PATROL_PHASE = 'accessible'
DEFAULT_MIN_SECONDS_TO_RETREAT = 120.0
DEFAULT_STEP_MIN_M = 8.0
DEFAULT_STEP_MAX_M = 20.0
DEFAULT_KEEPOUT_MARGIN_M = 15.0
DEFAULT_SAMPLE_ATTEMPTS = 24


def parse_tide_status(raw):
    """Parse /mission/tide_status JSON. Invalid payload -> None."""
    if raw is None:
        return None
    if isinstance(raw, dict):
        return raw
    try:
        data = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    return data


def patrol_is_allowed(status, min_seconds_to_retreat=None):
    """True only while the tide access window is open for work."""
    if not status:
        return False
    if status.get('phase') != PATROL_PHASE:
        return False
    if status.get('should_leave'):
        return False
    if status.get('retreat_published'):
        return False
    limit = (
        DEFAULT_MIN_SECONDS_TO_RETREAT
        if min_seconds_to_retreat is None
        else min_seconds_to_retreat
    )
    try:
        limit = float(limit)
    except (TypeError, ValueError):
        limit = DEFAULT_MIN_SECONDS_TO_RETREAT
    secs = status.get('seconds_to_retreat')
    if secs is None:
        return True
    try:
        return float(secs) >= limit
    except (TypeError, ValueError):
        return False


def min_distance_to_lines(xy, lines):
    """Shortest distance from xy to any polyline. None if no usable line."""
    if xy is None:
        return None
    best = None
    for line in lines or ():
        hit = nearest_on_polyline(line, xy)
        if hit is None:
            continue
        dist = hit[2]
        if best is None or dist < best:
            best = dist
    return best


def keepout_lines_from_markers(markers):
    """Extract map-frame polylines from a MarkerArray-like object."""
    lines = []
    items = getattr(markers, 'markers', None)
    if items is None:
        items = markers or ()
    for marker in items:
        action = int(getattr(marker, 'action', 0) or 0)
        if action in (2, 3):  # DELETE, DELETEALL
            continue
        points = []
        for point in getattr(marker, 'points', ()) or ():
            try:
                x_val = float(point.x)
                y_val = float(point.y)
            except (TypeError, ValueError, AttributeError):
                continue
            if math.isfinite(x_val) and math.isfinite(y_val):
                points.append((x_val, y_val))
        if len(points) >= 2:
            lines.append(points)
    return lines


def sample_patrol_goal(
        robot_xy,
        keepout_lines,
        rng=None,
        step_min_m=None,
        step_max_m=None,
        keepout_margin_m=None,
        attempts=None):
    """Random nearby map point that stays keepout_margin away from lethal.

    Returns (x, y) or None.
    """
    if robot_xy is None:
        return None
    try:
        rx, ry = float(robot_xy[0]), float(robot_xy[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not math.isfinite(rx) or not math.isfinite(ry):
        return None
    rng = random.Random() if rng is None else rng
    step_min = DEFAULT_STEP_MIN_M if step_min_m is None else float(step_min_m)
    step_max = DEFAULT_STEP_MAX_M if step_max_m is None else float(step_max_m)
    if step_max < step_min:
        step_min, step_max = step_max, step_min
    margin = (
        DEFAULT_KEEPOUT_MARGIN_M
        if keepout_margin_m is None
        else float(keepout_margin_m)
    )
    tries = (
        DEFAULT_SAMPLE_ATTEMPTS if attempts is None else int(attempts)
    )
    if tries < 1:
        return None
    for _ in range(tries):
        yaw = rng.uniform(0.0, 2.0 * math.pi)
        step = rng.uniform(step_min, step_max)
        xy = (rx + step * math.cos(yaw), ry + step * math.sin(yaw))
        dist = min_distance_to_lines(xy, keepout_lines)
        if keepout_lines and (dist is None or dist < margin):
            continue
        return xy
    return None
