"""여러 해안의 갯벌 폭으로 수위선을 만든다.

표고 없이, 갯벌 칸마다
  α_field = d_L / (d_C + d_L)
를 둔다. d_C 는 가장 가까운 해안. d_L 은 해안에 붙지 않은 갯벌
바깥 변 — 외해 입구와 조류 수로 둑을 포함한다. 그 변이 간조 수면이다.
만 입구만 L 로 남기지 않는다. 같은 α 에서 폭이 짧은 단면은 미터가
작고 긴 단면은 크다. 해안이 둘인 해협도 같은 장에서 처리한다.

거리장은 육지를 건너지 않는다(갯벌 geodesic).
keepout 띠는 씨앗으로 쓰지 않는다.
"""

from __future__ import annotations

import math

import numpy as np

from eclipse_pkg.tide_waterline import (
    _as_line,
    _coast_line_parts,
    _explode_polygons,
    _grid_bounds,
    _rasterize_lines,
    _rasterize_polys,
    _stitch_polylines,
    _valid_polygon,
    _window_box,
    _xy_pair,
)

DEFAULT_CELL_M = 10.0
DEFAULT_COAST_CLEAR_M = 40.0
DEFAULT_WINDOW_M = 10000.0
DEFAULT_MIN_LEN_M = 80.0
# 제부 본섬 해안은 열린 선. 갯벌 안 작은 섬만 닫힌 고리(실측 최대 ~1.6 km, 7 ha).
DEFAULT_MIN_CLOSED_LEN_M = 0.0
DEFAULT_MIN_CLOSED_AREA_M2 = 0.0
_CLOSED_TOL_M = 30.0
_BLOCK = 1.0e12
_MAX_CELLS = 9_000_000


def _positive(value, default):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(number) or number <= 0.0:
        return float(default)
    return number


def _polys(mud_polys):
    out = []
    for item in mud_polys or ():
        for part in _explode_polygons(item) or ():
            valid = _valid_polygon(part)
            if valid is not None and valid.area > 0.0:
                out.append(valid)
    return out


def _coast_parts(coast_geoms):
    parts = []
    for item in coast_geoms or ():
        line = _as_line(item)
        if line is None:
            continue
        if getattr(line, 'geom_type', '') == 'MultiLineString':
            parts.extend(_coast_line_parts(line))
        elif not line.is_empty and line.length > 0.0:
            parts.append(line)
    return parts


def _line_is_closed(line, tol_m=_CLOSED_TOL_M):
    try:
        coords = list(line.coords)
    except (TypeError, ValueError, AttributeError):
        return False
    if len(coords) < 4:
        return False
    try:
        gap = math.hypot(coords[0][0] - coords[-1][0], coords[0][1] - coords[-1][1])
    except (TypeError, ValueError, IndexError):
        return False
    return gap <= float(tol_m)


def _coords_are_closed(coords, tol_m=_CLOSED_TOL_M):
    if not coords or len(coords) < 4:
        return False
    first = _xy_pair(coords[0])
    last = _xy_pair(coords[-1])
    if first is None or last is None:
        return False
    return math.hypot(first[0] - last[0], first[1] - last[1]) <= float(tol_m)


def drop_closed_interior_blobs(rings, coast_geoms, join_m=40.0):
    """해안을 감싸지 않는 닫힌 등고선은 가운데 덩어리로 버린다.

    열린 전선은 유지. 본섬을 한 바퀴 도는 닫힌 선은 해안이 안에 있어 유지.
    """
    from shapely.geometry import Point as ShapelyPoint
    from shapely.geometry import Polygon as ShapelyPolygon

    coast = _coast_parts(coast_geoms)
    try:
        margin = float(join_m)
    except (TypeError, ValueError):
        margin = 40.0
    kept = []
    dropped = 0
    for ring in rings or ():
        if not _coords_are_closed(ring):
            kept.append(ring)
            continue
        try:
            poly = ShapelyPolygon(list(ring))
        except (TypeError, ValueError):
            kept.append(ring)
            continue
        if poly is None or poly.is_empty:
            dropped += 1
            continue
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly is None or poly.is_empty:
            dropped += 1
            continue
        wraps = False
        for part in coast:
            try:
                if part.within(poly) or poly.contains(part):
                    wraps = True
                    break
                if poly.intersects(part):
                    wraps = True
                    break
                for coord in list(part.coords)[:: max(1, len(list(part.coords)) // 8)]:
                    xy = _xy_pair(coord)
                    if xy is None:
                        continue
                    pt = ShapelyPoint(xy[0], xy[1])
                    if poly.covers(pt) or poly.distance(pt) <= margin:
                        wraps = True
                        break
                if wraps:
                    break
            except (TypeError, ValueError):
                continue
        if wraps:
            kept.append(ring)
        else:
            dropped += 1
    if dropped:
        print(
            f'width-grid drop {dropped} closed interior blobs, keep {len(kept)}',
            flush=True)
    return kept


def _closed_line_area(line):
    from shapely.geometry import Polygon as ShapelyPolygon
    try:
        poly = ShapelyPolygon(line.coords)
    except (TypeError, ValueError):
        return None
    if poly is None or poly.is_empty:
        return None
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly is None or poly.is_empty or poly.area <= 0.0:
        return None
    if poly.geom_type == 'MultiPolygon':
        return max((part.area for part in poly.geoms), default=None)
    return float(poly.area)


def filter_major_coast(
        coast_geoms,
        min_closed_len_m=DEFAULT_MIN_CLOSED_LEN_M,
        min_closed_area_m2=DEFAULT_MIN_CLOSED_AREA_M2):
    """닫힌 작은 섬 해안만 뺀다. 본섬·육지는 열린 선이라 그대로 둔다."""
    try:
        min_len = float(min_closed_len_m)
    except (TypeError, ValueError):
        min_len = 0.0
    try:
        min_area = float(min_closed_area_m2)
    except (TypeError, ValueError):
        min_area = 0.0
    if min_len <= 0.0 and min_area <= 0.0:
        return list(coast_geoms or ())
    kept = []
    dropped = 0
    for part in _coast_parts(coast_geoms):
        if not _line_is_closed(part):
            kept.append(part)
            continue
        length = float(part.length)
        area = _closed_line_area(part)
        too_short = min_len > 0.0 and length < min_len
        too_small = (
            min_area > 0.0 and area is not None and area < min_area)
        if too_short or too_small:
            dropped += 1
            continue
        kept.append(part)
    if dropped:
        print(
            f'width-grid drop {dropped} small closed coast '
            f'(len<{min_len:.0f}m area<{min_area:.0f}m2), keep {len(kept)}',
            flush=True)
    return kept


def _intersect_window(geoms, origin_xy, radius_m):
    """창으로 자른다. 창 밖 전국 면을 격자화하지 않기 위함.

    잘린 테두리는 L 씨앗에서 창 테두리 마스크로 버린다.
    """
    window = _window_box(origin_xy, radius_m)
    if window is None:
        return list(geoms or ())
    kept = []
    for geom in geoms or ():
        if geom is None or getattr(geom, 'is_empty', True):
            continue
        try:
            if not geom.intersects(window):
                continue
            hit = geom.intersection(window)
        except (TypeError, ValueError):
            continue
        if hit is None or hit.is_empty:
            continue
        gtype = getattr(hit, 'geom_type', '')
        if gtype in ('Polygon', 'MultiPolygon'):
            kept.extend(_explode_polygons(hit))
        elif gtype == 'LineString':
            if hit.length > 0.0:
                kept.append(hit)
        elif gtype == 'MultiLineString':
            kept.extend(_coast_line_parts(hit))
        elif gtype == 'GeometryCollection':
            kept.extend(_intersect_window(list(hit.geoms), origin_xy, radius_m))
    return kept


def _geodesic_on_mud(mud_mask, seeds, cell_m):
    """씨앗까지 갯벌만 지나는 거리(m). 육지 칸은 inf."""
    from skimage.graph import MCP_Geometric

    if not mud_mask.any() or not np.logical_and(seeds, mud_mask).any():
        return np.full(mud_mask.shape, np.inf, dtype=np.float64)
    costs = np.where(mud_mask, 1.0, _BLOCK).astype(np.float64)
    starts = list(zip(*np.nonzero(np.logical_and(seeds, mud_mask))))
    mcp = MCP_Geometric(costs, fully_connected=True)
    dist, _trace = mcp.find_costs(starts)
    out = dist.astype(np.float64) * float(cell_m)
    out[np.logical_not(mud_mask)] = np.inf
    out[dist >= (_BLOCK * 0.01)] = np.inf
    return out


def _outline_mask(mud_mask):
    from scipy.ndimage import binary_erosion

    if not mud_mask.any():
        return np.zeros_like(mud_mask, dtype=bool)
    eroded = binary_erosion(mud_mask, iterations=1)
    return np.logical_and(mud_mask, np.logical_not(eroded))


def _outer_outline_mask(mud_mask):
    """바깥 둘레만. 구멍(작은 섬을 뺀 자리)은 L 이 되면 가운데 덩어리가 된다."""
    from scipy.ndimage import binary_erosion, binary_fill_holes

    if not mud_mask.any():
        return np.zeros_like(mud_mask, dtype=bool)
    filled = binary_fill_holes(mud_mask)
    eroded = binary_erosion(filled, iterations=1)
    outer = np.logical_and(filled, np.logical_not(eroded))
    return np.logical_and(outer, mud_mask)


def _seaward_l_mask(outline, d_c, clear_m, cell_m):
    """해안에 안 붙은 둘레의 국소 최대에서, 비슷한 d_C 로 이어진 변을 L 로 채운다.

    고원(열린 바깥)은 봉우리가 같아 변 전체가 남고, 해협 입구는 그 봉우리
    근처만 남는다. 옆면은 해안 쪽으로 d_C 가 줄어 최대가 아니다.
    """
    from collections import deque
    from scipy.ndimage import binary_dilation

    far = np.logical_and(outline, np.isfinite(d_c) & (d_c >= float(clear_m)))
    if not far.any():
        return far
    rows, cols = np.nonzero(far)
    far_idx = set(zip(rows.tolist(), cols.tolist()))
    peaks = []
    tol = 0.5 * float(cell_m)
    for row, col in far_idx:
        here = d_c[row, col]
        higher = False
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nb = (row + dr, col + dc)
                if nb not in far_idx:
                    continue
                if d_c[nb[0], nb[1]] > here + tol:
                    higher = True
                    break
            if higher:
                break
        if not higher:
            peaks.append((row, col, here))
    if not peaks:
        keep = far
    else:
        keep = np.zeros_like(outline, dtype=bool)
        for prow, pcol, peak in peaks:
            if peak <= 0.0 or not math.isfinite(peak):
                continue
            floor = max(float(clear_m), 0.85 * peak)
            start = (prow, pcol)
            if keep[prow, pcol]:
                continue
            queue = deque([start])
            seen = {start}
            while queue:
                row, col = queue.popleft()
                if d_c[row, col] < floor:
                    continue
                keep[row, col] = True
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nb = (row + dr, col + dc)
                        if nb in far_idx and nb not in seen:
                            seen.add(nb)
                            queue.append(nb)
    if not keep.any():
        keep = far
    return np.logical_and(binary_dilation(keep, iterations=2), outline)


def _window_border_mask(shape, pad=3):
    border = np.ones(shape, dtype=bool)
    if shape[0] > 2 * pad and shape[1] > 2 * pad:
        border[pad:-pad, pad:-pad] = False
    return border


def build_width_grid(
        mud_polys, coast_geoms,
        origin_xy=None, radius_m=None,
        cell_m=DEFAULT_CELL_M,
        coast_clear_m=DEFAULT_COAST_CLEAR_M,
        min_closed_len_m=DEFAULT_MIN_CLOSED_LEN_M,
        min_closed_area_m2=DEFAULT_MIN_CLOSED_AREA_M2):
    """해안·바깥 변 거리장을 한 번 만든다. α 루프에서 재사용.

    origin/radius 가 있으면 그 창과 겹치는 면·선만  rasters.
    창 테두리는 L 씨앗으로 쓰지 않는다.
    닫힌 작은 섬 해안은 C 씨앗에서 빼서 구멍 고리를 만들지 않는다.
    """
    cell = _positive(cell_m, DEFAULT_CELL_M)
    clear_m = _positive(coast_clear_m, DEFAULT_COAST_CLEAR_M)
    mud = _polys(mud_polys)
    coast = filter_major_coast(
        _coast_parts(coast_geoms),
        min_closed_len_m=min_closed_len_m,
        min_closed_area_m2=min_closed_area_m2)
    if origin_xy is not None and radius_m is not None:
        mud = _intersect_window(mud, origin_xy, radius_m)
        coast = _intersect_window(coast, origin_xy, radius_m)
    if not mud or not coast:
        return None
    bounds = _grid_bounds(mud + coast, cell * 4.0)
    if bounds is None:
        return None
    xmin, ymin, xmax, ymax = bounds
    ncols = int(math.ceil((xmax - xmin) / cell)) + 1
    nrows = int(math.ceil((ymax - ymin) / cell)) + 1
    if nrows < 8 or ncols < 8 or nrows * ncols > _MAX_CELLS:
        return None
    print(
        f'width-grid {nrows}x{ncols} cell={cell:.1f}m '
        f'({nrows * ncols / 1e6:.2f}M cells)',
        flush=True)
    mud_mask = np.zeros((nrows, ncols), dtype=bool)
    coast_mask = np.zeros((nrows, ncols), dtype=bool)
    _rasterize_polys(mud_mask, mud, xmin, ymin, cell)
    _rasterize_lines(coast_mask, coast, xmin, ymin, cell)
    if not mud_mask.any() or not np.logical_and(coast_mask, mud_mask).any():
        # 해안이 갯벌 칸에 안 겹치면 한 칸 팽창해서 붙인다.
        from scipy.ndimage import binary_dilation
        coast_mask = np.logical_and(
            binary_dilation(coast_mask, iterations=2), mud_mask)
    if not mud_mask.any() or not np.logical_and(coast_mask, mud_mask).any():
        return None
    from scipy.ndimage import distance_transform_edt
    d_c_eucl = distance_transform_edt(np.logical_not(coast_mask)) * cell
    outline = _outer_outline_mask(mud_mask)
    border = _window_border_mask(
        mud_mask.shape, pad=max(2, int(round(clear_m / cell))))
    outline_inner = np.logical_and(outline, np.logical_not(border))
    l_full_mask = np.logical_and(outline_inner, d_c_eucl >= clear_m)
    l_mask = _seaward_l_mask(outline_inner, d_c_eucl, clear_m, cell)
    if not l_mask.any():
        return None
    if not l_full_mask.any():
        l_full_mask = l_mask
    n_c = int(np.logical_and(coast_mask, mud_mask).sum())
    print(
        f'width-grid geodesic C seeds={n_c} L seeds={int(l_mask.sum())} '
        f'L_full={int(l_full_mask.sum())}',
        flush=True)
    d_c = _geodesic_on_mud(mud_mask, coast_mask, cell)
    print('width-grid d_C done', flush=True)
    d_l = _geodesic_on_mud(mud_mask, l_mask, cell)
    print('width-grid d_L done', flush=True)
    denom = d_c + d_l
    field = np.full(mud_mask.shape, 2.0, dtype=np.float64)
    ok = np.logical_and.reduce((
        mud_mask,
        np.isfinite(d_c),
        np.isfinite(d_l),
        denom > cell * 0.5,
    ))
    field[ok] = d_l[ok] / denom[ok]
    return {
        'mud_mask': mud_mask,
        'coast_mask': coast_mask,
        'l_mask': l_mask,
        'l_full_mask': l_full_mask,
        'field': field,
        'ok': ok,
        'd_c': d_c,
        'd_l': d_l,
        'xmin': xmin,
        'ymin': ymin,
        'cell': cell,
        'coast_geoms': coast,
        'mud': mud,
        'clear_m': clear_m,
    }


def _geom_to_line_rings(geom, min_len_m):
    """LineString / MultiLineString 을 좌표 링 목록으로."""
    if geom is None or getattr(geom, 'is_empty', True):
        return []
    gtype = getattr(geom, 'geom_type', '')
    if gtype == 'LineString':
        parts = [geom]
    elif gtype in ('MultiLineString', 'GeometryCollection'):
        parts = list(geom.geoms)
    else:
        return []
    rings = []
    min_len = float(min_len_m)
    for part in parts:
        if part is None or getattr(part, 'is_empty', True):
            continue
        if getattr(part, 'geom_type', '') != 'LineString':
            rings.extend(_geom_to_line_rings(part, min_len_m))
            continue
        coords = [
            xy for xy in (_xy_pair(c) for c in part.coords)
            if xy is not None]
        if len(coords) < 2:
            continue
        length = 0.0
        for a_pt, b_pt in zip(coords, coords[1:]):
            length += math.hypot(b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
        if length < min_len:
            continue
        rings.append(coords)
    return rings


def vector_seaward_outer_rings(
        mud_polys, coast_geoms, clear_m, min_len_m=DEFAULT_MIN_LEN_M,
        window=None):
    """해안에 안 붙은 갯벌 바깥 변(외해·조류 수로). 간조 α=0 용 벡터."""
    from shapely.geometry import LineString as ShapelyLineString
    from shapely.ops import unary_union

    polys = _polys(mud_polys)
    if not polys:
        return []
    mud_u = polys[0] if len(polys) == 1 else unary_union(polys)
    faces = _explode_polygons(mud_u)
    outers = []
    for face in faces:
        if face is None or getattr(face, 'is_empty', True):
            continue
        try:
            line = ShapelyLineString(face.exterior.coords)
        except (TypeError, ValueError):
            continue
        if line.length > 0.0:
            outers.append(line)
    if not outers:
        return []
    outer = outers[0] if len(outers) == 1 else unary_union(outers)
    coast = _coast_parts(coast_geoms)
    if coast:
        coast_u = coast[0] if len(coast) == 1 else unary_union(coast)
        try:
            clear = float(clear_m)
        except (TypeError, ValueError):
            clear = DEFAULT_COAST_CLEAR_M
        if coast_u is not None and not coast_u.is_empty and clear > 0.0:
            try:
                outer = outer.difference(coast_u.buffer(clear))
            except (TypeError, ValueError):
                pass
    if window is not None:
        try:
            outer = outer.intersection(window)
        except (TypeError, ValueError):
            pass
    return _geom_to_line_rings(outer, min_len_m)


def _mask_to_rings(mask, xmin, ymin, cell, min_len_m):
    """이진 마스크 외곽을 폴리라인으로."""
    try:
        from skimage.measure import find_contours
    except ImportError:
        return []
    try:
        contours = find_contours(mask.astype(np.float64), 0.5)
    except (ValueError, TypeError):
        return []
    return _contours_to_rings(
        contours, xmin, ymin, cell, min_len_m, accept=None)


def _contours_to_rings(contours, xmin, ymin, cell, min_len_m, accept=None):
    lines = []

    def _flush(seg):
        if len(seg) < 2:
            return
        coords = seg
        from shapely.geometry import LineString as ShapelyLineString
        try:
            geom = ShapelyLineString(seg).simplify(
                max(cell, 2.0), preserve_topology=False)
            coords = [
                xy for xy in (_xy_pair(c) for c in geom.coords)
                if xy is not None]
        except (TypeError, ValueError):
            coords = seg
        if len(coords) < 2:
            return
        length = 0.0
        for a_pt, b_pt in zip(coords, coords[1:]):
            length += math.hypot(b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
        if length < float(min_len_m):
            return
        lines.append(coords)

    for contour in contours:
        current = []
        for row, col in contour:
            if accept is not None:
                ri = int(round(row))
                ci = int(round(col))
                if not (
                    0 <= ri < accept.shape[0]
                    and 0 <= ci < accept.shape[1]
                    and accept[ri, ci]
                ):
                    _flush(current)
                    current = []
                    continue
            x_val = xmin + (float(col) + 0.5) * cell
            y_val = ymin + (float(row) + 0.5) * cell
            current.append((x_val, y_val))
        _flush(current)
    join_m = max(80.0, 8.0 * cell)
    return _stitch_polylines(lines, join_m=join_m)


def _coast_near_mud(coast_geoms, mud_polys, min_len_m, near_m=80.0):
    from shapely.ops import unary_union
    mud = _polys(mud_polys)
    if not mud:
        return []
    mud_u = mud[0] if len(mud) == 1 else unary_union(mud)
    segs = []
    for part in _coast_parts(coast_geoms):
        try:
            if mud_u.distance(part) > near_m:
                continue
            coords = [
                xy for xy in (_xy_pair(c) for c in part.coords)
                if xy is not None]
        except (TypeError, ValueError):
            continue
        if len(coords) < 2:
            continue
        length = 0.0
        for a_pt, b_pt in zip(coords, coords[1:]):
            length += math.hypot(b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
        if length >= float(min_len_m):
            segs.append(coords)
    return segs


def waterline_from_width_grid(grid, alpha, min_len_m=DEFAULT_MIN_LEN_M):
    """거리장에서 α 등고선. α=0 바깥 변, α=1 해안."""
    if not grid:
        return []
    try:
        weight = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        return []
    cell = float(grid['cell'])
    min_len = _positive(min_len_m, DEFAULT_MIN_LEN_M)
    xmin = grid['xmin']
    ymin = grid['ymin']
    if weight <= 0.01:
        from shapely.geometry import box as shapely_box
        nrows, ncols = grid['mud_mask'].shape
        window = shapely_box(
            xmin, ymin,
            xmin + ncols * cell, ymin + nrows * cell)
        rings = vector_seaward_outer_rings(
            grid.get('mud'), grid.get('coast_geoms'),
            grid.get('clear_m', DEFAULT_COAST_CLEAR_M),
            min_len_m=min_len, window=window)
        if rings:
            return rings
        full = grid.get('l_full_mask')
        if full is None or not np.any(full):
            full = grid['l_mask']
        return _mask_to_rings(full, xmin, ymin, cell, min_len)
    if weight >= 0.99:
        return _coast_near_mud(
            grid.get('coast_geoms'), grid.get('mud'),
            max(min_len, 200.0), near_m=80.0)
    try:
        from skimage.measure import find_contours
    except ImportError:
        return []
    field = np.array(grid['field'], dtype=np.float64, copy=True)
    # 갯벌 밖을 2 로 두면 0.5 등고선이 바깥 변을 한 바퀴 돈다.
    field[np.logical_not(grid['mud_mask'])] = np.nan
    try:
        contours = find_contours(field, weight)
    except (ValueError, TypeError):
        return []
    rings = _contours_to_rings(
        contours, xmin, ymin, cell, min_len, accept=grid['ok'])
    return drop_closed_interior_blobs(rings, grid.get('coast_geoms'))


def waterline_width_rings(
        mud_polys, coast_geoms, alpha,
        origin_xy=None, radius_m=None,
        cell_m=DEFAULT_CELL_M,
        coast_clear_m=DEFAULT_COAST_CLEAR_M,
        min_len_m=DEFAULT_MIN_LEN_M,
        min_closed_len_m=DEFAULT_MIN_CLOSED_LEN_M,
        min_closed_area_m2=DEFAULT_MIN_CLOSED_AREA_M2,
        grid=None):
    """α 한 장. grid 를 넘기면 거리장을 재사용한다."""
    if grid is None:
        grid = build_width_grid(
            mud_polys, coast_geoms,
            origin_xy=origin_xy, radius_m=radius_m,
            cell_m=cell_m, coast_clear_m=coast_clear_m,
            min_closed_len_m=min_closed_len_m,
            min_closed_area_m2=min_closed_area_m2)
    return waterline_from_width_grid(grid, alpha, min_len_m=min_len_m)


def waterline_width_steps(
        mud_polys, coast_geoms, alphas,
        origin_xy=None, radius_m=None,
        cell_m=DEFAULT_CELL_M,
        coast_clear_m=DEFAULT_COAST_CLEAR_M,
        min_len_m=DEFAULT_MIN_LEN_M,
        min_closed_len_m=DEFAULT_MIN_CLOSED_LEN_M,
        min_closed_area_m2=DEFAULT_MIN_CLOSED_AREA_M2):
    """α 목록 → [(alpha, rings), ...]. 거리장은 한 번만."""
    grid = build_width_grid(
        mud_polys, coast_geoms,
        origin_xy=origin_xy, radius_m=radius_m,
        cell_m=cell_m, coast_clear_m=coast_clear_m,
        min_closed_len_m=min_closed_len_m,
        min_closed_area_m2=min_closed_area_m2)
    if grid is None:
        return []
    out = []
    for raw in alphas or ():
        try:
            alpha = round(float(raw), 2)
        except (TypeError, ValueError):
            continue
        rings = waterline_from_width_grid(
            grid, alpha, min_len_m=min_len_m)
        out.append((alpha, rings))
    return out
