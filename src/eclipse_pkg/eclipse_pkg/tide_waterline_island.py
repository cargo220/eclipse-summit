"""섬형 갯벌 수위선. 제부처럼 해측 keepout 띠 → 섬 해안.

만(육지 해안)에는 쓰지 마라. 그건 tide_waterline_bay.
"""

from __future__ import annotations

import math

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from eclipse_pkg.tide_plan import interpolate_waterline
from eclipse_pkg.tide_waterline import (
    _last_inside_along,
    _stitch_polylines,
    _unit,
    _valid_polygon,
    filter_keepout_rings,
    high_tide_coast_rings,
    keepout_grow_span_m,
    waterline_lerp_from_keepout,
)


def island_high_tide_rings(keepout_rings, mud_polys, coast_geoms, grow_m=None):
    """만조 = keepout 을 갯벌 끝까지 키운 구멍(섬)."""
    if grow_m is None:
        grow_m = keepout_grow_span_m(keepout_rings, coast_geoms, fallback_m=400.0)
    return filter_keepout_rings(
        high_tide_coast_rings(keepout_rings, mud_polys, grow_m), 800.0), grow_m


def waterline_island_rings(
        keepout_rings, mud_polys, coast_geoms, alpha, grow_m=None):
    """α=0 keepout, α=1 섬, 중간은 안쪽 변을 섬까지 로컬 폭 비율."""
    island, _grow = island_high_tide_rings(
        keepout_rings, mud_polys, coast_geoms, grow_m=grow_m)
    try:
        weight = float(alpha)
    except (TypeError, ValueError):
        return []
    if weight <= 0.01:
        return [
            list(ring) for ring in keepout_rings or ()
            if ring and len(ring) >= 2]
    if weight >= 0.99:
        return [list(ring) for ring in island if ring and len(ring) >= 2]
    return waterline_lerp_from_keepout(
        keepout_rings, island, weight, mud_polys=mud_polys)


def _poly_from_ring(ring):
    coords = list(ring or ())
    if len(coords) < 3:
        return None
    if coords[0] != coords[-1]:
        coords = coords + [coords[0]]
    try:
        poly = Polygon(coords)
    except (TypeError, ValueError):
        return None
    return _valid_polygon(poly)


def _ring_is_target(poly, pin_xy):
    if pin_xy is None:
        return True
    try:
        pin = Point(float(pin_xy[0]), float(pin_xy[1]))
    except (TypeError, ValueError, IndexError):
        return True
    try:
        if poly.contains(pin) or poly.covers(pin):
            return True
        return poly.distance(pin) <= 80.0
    except (TypeError, ValueError):
        return False


def _outward_xy(poly, outline, along_m):
    """해안 접선의 바깥 법선."""
    try:
        length = float(outline.length)
        along = float(along_m)
    except (TypeError, ValueError):
        return None
    if length <= 0.0:
        return None
    delta = min(12.0, max(2.0, length * 0.01))
    p0 = outline.interpolate(max(0.0, along - delta), normalized=False)
    p1 = outline.interpolate(min(length, along + delta), normalized=False)
    tang = _unit(p1.x - p0.x, p1.y - p0.y)
    if tang is None:
        return None
    ux, uy = tang[1], -tang[0]
    pt = outline.interpolate(min(length, max(0.0, along)), normalized=False)
    probe = Point(pt.x + ux * 8.0, pt.y + uy * 8.0)
    try:
        if poly.contains(probe) or poly.covers(probe):
            ux, uy = -ux, -uy
    except (TypeError, ValueError):
        pass
    return (ux, uy)


def _smooth_widths(widths, window=2):
    values = [float(item) for item in widths]
    if len(values) <= 2:
        return values
    try:
        span = int(window)
    except (TypeError, ValueError):
        span = 2
    if span < 1:
        span = 1
    out = []
    for index in range(len(values)):
        lo = max(0, index - span)
        hi = min(len(values), index + span + 1)
        neigh = sorted(values[lo:hi])
        out.append(neigh[len(neigh) // 2])
    return out


def waterline_from_coast_width(
        island_rings, mud_polys, alpha,
        station_m=30.0, min_mud_m=80.0, search_m=4000.0,
        min_len_m=80.0, pin_xy=None):
    """해안 점에서 갯벌 폭 W. 수위선은 해안 + (1−α)·W.

    α=1 해안, α=0 갯벌 바깥 끝. keepout C 가 없어도 폭이 있으면 그린다.
    """
    try:
        weight = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        return []
    mud_list = []
    for poly in mud_polys or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            mud_list.append(valid)
    if not mud_list:
        return []
    try:
        mud_u = mud_list[0] if len(mud_list) == 1 else unary_union(mud_list)
    except (TypeError, ValueError):
        return []
    if mud_u is None or mud_u.is_empty:
        return []
    try:
        station = float(station_m)
    except (TypeError, ValueError):
        station = 30.0
    if station <= 0.0:
        station = 30.0
    segs = []
    for ring in island_rings or ():
        poly = _poly_from_ring(ring)
        if poly is None or poly.area < 1.0e4:
            continue
        if not _ring_is_target(poly, pin_xy):
            continue
        try:
            outline = LineString(poly.exterior.coords)
        except (TypeError, ValueError):
            continue
        if outline.length < min_mud_m:
            continue
        count = max(1, int(outline.length / station))
        samples = []
        for index in range(count + 1):
            along = outline.length * (index / count)
            pt = outline.interpolate(along, normalized=False)
            origin = (float(pt.x), float(pt.y))
            direction = _outward_xy(poly, outline, along)
            if direction is None:
                samples.append(None)
                continue
            width = _last_inside_along(
                origin, direction, mud_u, search_m, 20.0)
            if width < min_mud_m:
                samples.append(None)
                continue
            samples.append((origin, direction, width))
        run = []
        runs = []
        for item in samples:
            if item is None:
                if run:
                    runs.append(run)
                    run = []
                continue
            run.append(item)
        if run:
            runs.append(run)
        for group in runs:
            if len(group) < 2:
                continue
            widths = _smooth_widths([item[2] for item in group], window=2)
            current = []
            prev = None
            for item, width in zip(group, widths):
                origin, direction, _raw = item
                outer = (
                    origin[0] + direction[0] * width,
                    origin[1] + direction[1] * width,
                )
                water = interpolate_waterline(outer, origin, weight)
                if water is None:
                    continue
                if prev is not None:
                    jump = math.hypot(water[0] - prev[0], water[1] - prev[1])
                    jump_max = max(150.0, station * 4.0) + 0.08 * width * (
                        1.0 - weight)
                    if jump > jump_max:
                        if len(current) >= 2:
                            segs.append(current)
                        current = []
                current.append(water)
                prev = water
            if len(current) >= 2:
                segs.append(current)
    stitched = _stitch_polylines(segs, join_m=max(200.0, station * 4.0))
    try:
        min_len = float(min_len_m)
    except (TypeError, ValueError):
        min_len = 80.0
    out = []
    for seg in stitched:
        length = 0.0
        for a_pt, b_pt in zip(seg, seg[1:]):
            length += math.hypot(b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
        if length >= min_len:
            out.append(seg)
    return out
