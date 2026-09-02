"""갯벌 해측 바깥 변 고정 띠. 조위 수위선이 아니다."""

import math
import os

try:
    from shapely.geometry import (
        LineString as ShapelyLineString,
        Point as ShapelyPoint,
        Polygon as ShapelyPolygon,
        box as shapely_box,
        shape as shapely_shape,
    )
    from shapely.ops import transform as shapely_transform
    from shapely.ops import triangulate as shapely_triangulate
    from shapely.ops import unary_union as shapely_unary_union
except ImportError:
    ShapelyLineString = None
    ShapelyPoint = None
    ShapelyPolygon = None
    shapely_box = None
    shapely_shape = None
    shapely_transform = None
    shapely_triangulate = None
    shapely_unary_union = None

from eclipse_pkg.tide_plan import (
    _UTMK,
    _prj_looks_5186,
    interpolate_waterline,
    resolve_strtree_query,
)

_EPS_M = 1e-6
_SAMPLE_M = 5.0
# radius 는 면 바깥 변을 찾는 탐색 상한. 조위로 줄이지 않는다.
DEFAULT_RADIUS_M = 20000.0
DEFAULT_ALONG_HALF_M = 20000.0
# 안전띠 두께 = 그 점 최대 해측 폭의 이 비율. 4000→400, 50→5.
DEFAULT_EDGE_INSET_RATIO = 0.10
# 0 이면 비율을 쓴다. 양수면 그 미터로 덮어쓴다(시험).
DEFAULT_EDGE_INSET_M = 0.0
DEFAULT_COAST_CLEAR_M = 0.0
DEFAULT_STATION_M = 60.0
DEFAULT_SIMPLIFY_M = 8.0
DEFAULT_MIN_WIDTH_M = 20.0
# 오프라인 후보 A: 해안 복도 바깥 비율. 실시간 last-inside 가 아님.
DEFAULT_CORRIDOR_M = 400.0
DEFAULT_CORRIDOR_OUTER_RATIO = 0.10
DEFAULT_CORRIDOR_SIMPLIFY_M = 12.0
DEFAULT_CORRIDOR_MIN_AREA_M2 = 2000.0
# 합친 갯벌 둘레. 맞닿은 변은 union 으로 사라지고, C 쪽 변은 연다.
DEFAULT_COAST_EDGE_CLEAR_M = 30.0
DEFAULT_PERIMETER_RATIO = 0.10
DEFAULT_PERIMETER_MIN_BAND_M = 15.0
# 짧은 잔편(2011/2026 어긋남·선착장 구멍)을 점으로 만들지 않음.
DEFAULT_PERIMETER_MIN_LINE_M = 1000.0
DEFAULT_KEEPOUT_GEOJSON_NAME = 'keepout_jebu_perimeter.geojson'
_ATTACH_M = 80.0
_PROBE_M = 20.0
_WIDTH_STEP_M = 5.0
_R_OUTER_TRIM_M = 10.0
_QUAD_OVERLAP = 0.55


def _xy_pair(raw):
    """좌표 한 점을 (x, y) float 로. 깨지면 None."""
    try:
        x_val, y_val = float(raw[0]), float(raw[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not (math.isfinite(x_val) and math.isfinite(y_val)):
        return None
    return (x_val, y_val)



def _positive_m(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or number <= 0.0:
        return None
    return number


def _as_line(item):
    if item is None or ShapelyLineString is None:
        return None
    if hasattr(item, 'geom_type'):
        geom = item
    else:
        try:
            geom = ShapelyLineString(item)
        except (TypeError, ValueError):
            return None
    if geom.is_empty:
        return None
    if geom.geom_type == 'LineString' and geom.length > 0.0:
        return geom
    if geom.geom_type == 'MultiLineString':
        return geom
    return None


def _window_box(origin_xy, radius_m):
    origin = _xy_pair(origin_xy)
    radius = _positive_m(radius_m)
    if origin is None or radius is None or shapely_box is None:
        return None
    return shapely_box(
        origin[0] - radius, origin[1] - radius,
        origin[0] + radius, origin[1] + radius,
    )


def _valid_polygon(geom):
    if geom is None or getattr(geom, 'is_empty', True):
        return None
    if not geom.is_valid:
        geom = geom.buffer(0)
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type == 'Polygon' and geom.area > 0.0:
        return geom
    if geom.geom_type == 'MultiPolygon':
        parts = [
            part for part in geom.geoms
            if part.geom_type == 'Polygon' and part.area > 0.0]
        if not parts:
            return None
        if len(parts) == 1:
            return parts[0]
        if shapely_unary_union is not None:
            merged = shapely_unary_union(parts)
            if merged is not None and not merged.is_empty:
                if merged.geom_type == 'Polygon' and merged.area > 0.0:
                    return merged
                if merged.geom_type == 'MultiPolygon':
                    parts = [
                        part for part in merged.geoms
                        if part.geom_type == 'Polygon' and part.area > 0.0]
                    if not parts:
                        return None
                    if len(parts) == 1:
                        return parts[0]
        return max(parts, key=lambda item: item.area)
    return None


def _explode_polygons(geom):
    if geom is None or getattr(geom, 'is_empty', True):
        return []
    if geom.geom_type == 'Polygon':
        valid = _valid_polygon(geom)
        return [valid] if valid is not None else []
    if geom.geom_type in ('MultiPolygon', 'GeometryCollection'):
        out = []
        for part in geom.geoms:
            out.extend(_explode_polygons(part))
        return out
    return []


def _coast_union(coast_geoms):
    if shapely_unary_union is None:
        return None
    lines = []
    for item in coast_geoms or ():
        line = _as_line(item)
        if line is not None:
            lines.append(line)
    if not lines:
        return None
    if len(lines) == 1:
        return lines[0]
    merged = shapely_unary_union(lines)
    if merged is None or merged.is_empty:
        return None
    return merged


def _transformer_to_5186(prj_text):
    if _prj_looks_5186(prj_text):
        return None
    try:
        import pyproj
    except ImportError as exc:
        raise RuntimeError('UTM-K 변환에 pyproj 가 필요합니다') from exc
    return pyproj.Transformer.from_crs(
        _UTMK, 'EPSG:5186', always_xy=True)


def _read_shapefile_geoms(path):
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f'shapefile 이 없습니다: {path}')
    if shapely_shape is None:
        raise RuntimeError('shapely 가 필요합니다')
    prj_path = os.path.splitext(path)[0] + '.prj'
    prj_text = ''
    if os.path.isfile(prj_path):
        with open(prj_path, encoding='utf-8', errors='replace') as prj_file:
            prj_text = prj_file.read()
    transformer = _transformer_to_5186(prj_text)
    geoms = []
    try:
        import fiona
    except ImportError:
        fiona = None
    if fiona is not None:
        with fiona.open(path) as src:
            for rec in src:
                try:
                    geoms.append(shapely_shape(rec['geometry']))
                except (TypeError, ValueError, KeyError):
                    continue
    else:
        try:
            import shapefile as pyshp
        except ImportError as exc:
            raise RuntimeError('fiona 또는 pyshp 가 필요합니다') from exc
        reader = pyshp.Reader(path)
        for shp in reader.shapes():
            try:
                geoms.append(shapely_shape(shp.__geo_interface__))
            except (TypeError, ValueError, AttributeError):
                continue
        reader.close()
    if transformer is None or shapely_transform is None:
        return geoms

    def _project(x_val, y_val, z_val=None):
        east, north = transformer.transform(x_val, y_val)
        if z_val is None:
            return (east, north)
        return (east, north, z_val)

    converted = []
    for geom in geoms:
        if geom is None or geom.is_empty:
            continue
        try:
            converted.append(shapely_transform(_project, geom))
        except (TypeError, ValueError):
            continue
    return converted


def _closed_line_to_polygon(geom):
    """닫힌 LineString 을 면으로. 클립본(외곽 링)용."""
    if geom is None or getattr(geom, 'geom_type', '') != 'LineString':
        return None
    if geom.is_empty or geom.length <= 0.0:
        return None
    coords = list(geom.coords)
    if len(coords) < 3:
        return None
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    if len(coords) < 4:
        return None
    try:
        poly = ShapelyPolygon(coords)
    except (TypeError, ValueError):
        return None
    return _valid_polygon(poly)


def load_mudflat_polygons(path):
    """갯벌 shapefile 을 EPSG:5186 면으로 읽는다. 구멍은 유지한다.

    클립본이 닫힌 LineString 링이면 면으로 닫는다.
    """
    polys = []
    for geom in _read_shapefile_geoms(path):
        polys.extend(_explode_polygons(geom))
        closed = _closed_line_to_polygon(geom)
        if closed is not None:
            polys.append(closed)
    return polys


def load_line_geoms(path, origin_xy=None, radius_m=None):
    """선 shapefile 을 EPSG:5186 LineString 목록으로 읽는다.

    origin/radius 가 있으면 그 창과 겹치는 선만 남긴다.
    """
    window = None
    if origin_xy is not None and radius_m is not None:
        window = _window_box(origin_xy, radius_m)
    lines = []
    for geom in _read_shapefile_geoms(path):
        if geom is None or geom.is_empty:
            continue
        parts = []
        if geom.geom_type == 'LineString':
            if geom.length > 0.0:
                parts.append(geom)
        elif geom.geom_type == 'MultiLineString':
            parts.extend(
                part for part in geom.geoms if part.length > 0.0)
        elif geom.geom_type in ('Polygon', 'MultiPolygon'):
            for poly in _explode_polygons(geom):
                exterior = ShapelyLineString(poly.exterior.coords)
                if exterior.length > 0.0:
                    parts.append(exterior)
        for part in parts:
            if window is not None:
                try:
                    if not part.intersects(window):
                        continue
                except (TypeError, ValueError):
                    continue
            lines.append(part)
    return lines


def bbox_of_polygons(polys):
    """폴리곤 목록 AABB (minx, miny, maxx, maxy). 없으면 None."""
    minx = miny = maxx = maxy = None
    for poly in polys or ():
        if poly is None or getattr(poly, 'is_empty', True):
            continue
        try:
            bounds = poly.bounds
        except (TypeError, ValueError, AttributeError):
            continue
        if bounds is None or len(bounds) < 4:
            continue
        try:
            bx0, by0, bx1, by1 = (float(v) for v in bounds[:4])
        except (TypeError, ValueError):
            continue
        if minx is None:
            minx, miny, maxx, maxy = bx0, by0, bx1, by1
        else:
            minx = min(minx, bx0)
            miny = min(miny, by0)
            maxx = max(maxx, bx1)
            maxy = max(maxy, by1)
    if minx is None:
        return None
    return (minx, miny, maxx, maxy)


def window_from_rings(rings, pad_m=0.0):
    """링 AABB 중심과 반경. (origin, radius) 또는 (None, None)."""
    xs = []
    ys = []
    for ring in rings or ():
        for raw in ring:
            xy = _xy_pair(raw)
            if xy is None:
                continue
            xs.append(xy[0])
            ys.append(xy[1])
    if not xs:
        return None, None
    pad = _positive_m(pad_m) or 0.0
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    origin = ((minx + maxx) * 0.5, (miny + maxy) * 0.5)
    radius = math.hypot(maxx - minx, maxy - miny) * 0.5 + pad
    if radius <= 0.0:
        return None, None
    return origin, radius


def clip_geoms_to_window(geoms, origin_xy, radius_m):
    """창과 겹치는 도형만 남긴다. 전국선 unary_union 을 피한다."""
    window = _window_box(origin_xy, radius_m)
    if window is None:
        return []
    kept = []
    for item in geoms or ():
        geom = item
        if not hasattr(geom, 'geom_type'):
            geom = _as_line(item)
        if geom is None or geom.is_empty:
            continue
        try:
            if geom.intersects(window):
                kept.append(geom)
        except (TypeError, ValueError):
            continue
    return kept


def local_domain(polys, origin_xy, radius_m):
    """P 창과 갯벌 면의 교집합."""
    window = _window_box(origin_xy, radius_m)
    if window is None or shapely_unary_union is None:
        return None
    parts = []
    for poly in polys or ():
        valid = _valid_polygon(poly)
        if valid is None:
            continue
        try:
            hit = valid.intersection(window)
        except (TypeError, ValueError):
            continue
        parts.extend(_explode_polygons(hit))
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]
    return _valid_polygon(shapely_unary_union(parts))


def _nearest_coast_point(origin_xy, coast):
    origin = _xy_pair(origin_xy)
    if origin is None or coast is None or ShapelyPoint is None:
        return None
    try:
        point = ShapelyPoint(origin[0], origin[1])
        return coast.interpolate(coast.project(point))
    except (TypeError, ValueError):
        return None



def _last_inside_along(start_xy, direction_xy, geom, search_m, step_m=10.0):
    """start 에서 direction 으로 geom 안에 마지막으로 남는 거리."""
    origin = _xy_pair(start_xy)
    hint = _unit(
        float(direction_xy[0]), float(direction_xy[1])) if direction_xy else None
    search = _positive_m(search_m)
    if origin is None or hint is None or search is None or ShapelyPoint is None:
        return 0.0
    try:
        step = float(step_m)
    except (TypeError, ValueError):
        step = 10.0
    if step <= 0.0:
        step = 10.0
    last = 0.0
    dist = 0.0
    ux, uy = hint
    seen = False
    while dist <= search + _EPS_M:
        point = ShapelyPoint(origin[0] + ux * dist, origin[1] + uy * dist)
        try:
            inside = geom.contains(point) or geom.covers(point)
        except (TypeError, ValueError):
            inside = False
        if inside:
            last = dist
            seen = True
        elif seen:
            break
        dist += step
    return last


def seaward_mudflat_width(
        mudflat, coast_geoms, origin_xy, seaward_xy, along_m, search_m):
    """이 점 앞 갯벌이 C 에서 해측으로 끝나는 거리.

    복도 안 최댓값은 만 채움 면에서 과대(멀리 튀는 꼭짓점)다.
    C* 와 좌우 평행 광선이 면을 빠져나가는 거리의 중앙값을 쓴다.
    """
    search = _positive_m(search_m)
    along = _positive_m(along_m)
    valid = _valid_polygon(mudflat)
    hint = _unit(
        float(seaward_xy[0]), float(seaward_xy[1])) if seaward_xy else None
    if search is None or along is None or valid is None or hint is None:
        return 0.0
    coast = _coast_union(coast_geoms)
    nearest = _nearest_coast_point(origin_xy, coast)
    if nearest is None:
        return 0.0
    # 이 핀의 해측 폭. 평행 광선 중앙값은 만 채움에서 끝을 넘기기 쉽다.
    width = _last_inside_along(
        (nearest.x, nearest.y), hint, valid, search)
    if width > 0.0:
        return width
    sx, sy = hint
    tx, ty = -sy, sx
    for offset in (-min(200.0, along), min(200.0, along)):
        start_x = nearest.x + tx * offset
        start_y = nearest.y + ty * offset
        if coast is not None and ShapelyPoint is not None:
            try:
                snapped = coast.interpolate(
                    coast.project(ShapelyPoint(start_x, start_y)))
                start_x, start_y = float(snapped.x), float(snapped.y)
            except (TypeError, ValueError):
                pass
        width = _last_inside_along(
            (start_x, start_y), hint, valid, search)
        if width > 0.0:
            return width
    return 0.0


def domain_width_from_coast(domain, coast_geoms, sample_m=_SAMPLE_M):
    """D 안에서 C 까지 거리의 최댓값. 경계 샘플."""
    valid = _valid_polygon(domain)
    coast = _coast_union(coast_geoms)
    if valid is None or coast is None:
        return 0.0
    try:
        step = float(sample_m)
    except (TypeError, ValueError):
        step = _SAMPLE_M
    if step <= 0.0:
        step = _SAMPLE_M
    best = 0.0
    boundary = valid.boundary
    if boundary is not None and not boundary.is_empty and boundary.length > 0.0:
        count = max(2, int(boundary.length / step) + 1)
        for index in range(count + 1):
            point = boundary.interpolate(index / count, normalized=True)
            best = max(best, point.distance(coast))
    coords = []
    if valid.geom_type == 'Polygon':
        coords.extend(valid.exterior.coords)
    elif valid.geom_type == 'MultiPolygon':
        for part in valid.geoms:
            coords.extend(part.exterior.coords)
    for x_val, y_val in coords:
        best = max(best, ShapelyPoint(x_val, y_val).distance(coast))
    return best


def simple_ring(geom):
    """TideLayer 용 단순 링. 자기교차·빈 면은 None."""
    valid = _valid_polygon(geom)
    if valid is None:
        return None
    work = valid
    if work.geom_type == 'MultiPolygon':
        work = max(work.geoms, key=lambda item: item.area)
    if work.geom_type != 'Polygon' or work.area <= 0.0:
        return None
    if not work.is_simple or not work.is_valid:
        work = work.buffer(0)
        work = _valid_polygon(work)
        if work is None:
            return None
        if work.geom_type == 'MultiPolygon':
            work = max(work.geoms, key=lambda item: item.area)
        if work.geom_type != 'Polygon' or not work.is_simple:
            return None
    coords = list(work.exterior.coords)
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) < 3:
        return None
    return [(float(x_val), float(y_val)) for x_val, y_val in coords]



def _ring_keep_extent(geom):
    """여러 조각이면 AABB 가 가장 긴 링. 짧은 상자 잔편을 고르지 않는다."""
    parts = _rings_all(geom)
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]

    def _extent(ring):
        xs = [pt[0] for pt in ring]
        ys = [pt[1] for pt in ring]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    return max(parts, key=_extent)


def _rings_all(geom, min_area=0.0):
    """끊긴 띠를 링 목록으로. 빈 값이면 []."""
    try:
        floor = float(min_area)
    except (TypeError, ValueError):
        floor = 0.0
    rings = []
    for part in _explode_polygons(geom):
        if floor > 0.0 and getattr(part, 'area', 0.0) < floor:
            continue
        ring = simple_ring(part)
        if ring is not None:
            rings.append(ring)
    rings.sort(
        key=lambda ring: (
            min(pt[0] for pt in ring),
            min(pt[1] for pt in ring),
        ))
    return rings


def _alongshore_corridor(origin_xy, seaward_xy, along_m, d_inner, d_outer):
    """해안 평행 복도. 로봇 중심 정사각 창으로 먼 수위선을 자르지 않는다."""
    origin = _xy_pair(origin_xy)
    hint = _unit(float(seaward_xy[0]), float(seaward_xy[1])) if seaward_xy else None
    along = _positive_m(along_m)
    if origin is None or hint is None or along is None or ShapelyPolygon is None:
        return None
    try:
        inner = float(d_inner)
        outer = float(d_outer)
    except (TypeError, ValueError):
        return None
    if outer <= inner:
        return None
    sx, sy = hint
    tx, ty = -sy, sx
    ox, oy = origin
    return ShapelyPolygon([
        (ox + tx * (-along) + sx * inner, oy + ty * (-along) + sy * inner),
        (ox + tx * (-along) + sx * outer, oy + ty * (-along) + sy * outer),
        (ox + tx * along + sx * outer, oy + ty * along + sy * outer),
        (ox + tx * along + sx * inner, oy + ty * along + sy * inner),
    ])



def _unit(dx, dy):
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return None
    return (dx / length, dy / length)



def _seaward_hint(origin_xy, coast_xy, mudflat_polys):
    """C에서 갯벌 쪽으로 가는 벡터."""
    origin = _xy_pair(origin_xy)
    coast = _xy_pair(coast_xy)
    if origin is None or coast is None:
        return None
    dx, dy = origin[0] - coast[0], origin[1] - coast[1]
    unit = _unit(dx, dy)
    if unit is None:
        return (0.0, 1.0)
    if ShapelyPoint is None:
        return unit
    probe = ShapelyPoint(
        coast[0] + unit[0] * 20.0, coast[1] + unit[1] * 20.0)
    insides = 0
    for poly in mudflat_polys or ():
        valid = _valid_polygon(poly)
        if valid is None:
            continue
        try:
            if valid.contains(probe) or valid.covers(probe):
                insides += 1
        except (TypeError, ValueError):
            continue
    if insides:
        return unit
    return (-unit[0], -unit[1])


def _coast_line_parts(coast):
    if coast is None or getattr(coast, 'is_empty', True):
        return []
    if coast.geom_type == 'LineString':
        return [coast] if coast.length > 0.0 else []
    if coast.geom_type == 'MultiLineString':
        return [part for part in coast.geoms if part.length > 0.0]
    return []


def _normal_into_mud(origin_xy, tangent_xy, mudflat):
    """접선에서 면 안으로 들어가는 법선. 양쪽 다 아니면 None."""
    origin = _xy_pair(origin_xy)
    tang = _unit(
        float(tangent_xy[0]), float(tangent_xy[1])) if tangent_xy else None
    area = mudflat
    if area is not None and getattr(area, 'is_empty', True):
        area = None
    if area is not None and area.geom_type not in ('Polygon', 'MultiPolygon'):
        area = _valid_polygon(area)
    if origin is None or tang is None or area is None or ShapelyPoint is None:
        return None
    nx, ny = -tang[1], tang[0]
    for sign in (1.0, -1.0):
        ux, uy = nx * sign, ny * sign
        probe = ShapelyPoint(
            origin[0] + ux * _PROBE_M, origin[1] + uy * _PROBE_M)
        try:
            if area.contains(probe) or area.covers(probe):
                return (ux, uy)
        except (TypeError, ValueError):
            continue
    return None


def _keepout_quad(
        cx, cy, normal_xy, tangent_xy, width, inset, half_m, min_inner=0.0):
    """C 점에서 로컬 바깥 10% 사다리꼴."""
    hint = _unit(float(normal_xy[0]), float(normal_xy[1]))
    tang = _unit(float(tangent_xy[0]), float(tangent_xy[1]))
    try:
        wide = float(width)
        pad = float(half_m)
        cut = float(inset)
        floor = float(min_inner)
    except (TypeError, ValueError):
        return None
    if hint is None or tang is None or wide <= 0.0 or pad <= 0.0:
        return None
    if not math.isfinite(cut) or cut <= 0.0:
        return None
    if not math.isfinite(floor) or floor < 0.0:
        floor = 0.0
    r_outer = max(wide - _R_OUTER_TRIM_M, 0.0)
    r_inner = max(r_outer - cut, floor)
    if r_outer <= r_inner + _EPS_M:
        return None
    nx, ny = hint
    tx, ty = tang
    if ShapelyPolygon is None:
        return None
    try:
        return ShapelyPolygon([
            (cx + nx * r_inner + tx * (-pad), cy + ny * r_inner + ty * (-pad)),
            (cx + nx * r_outer + tx * (-pad), cy + ny * r_outer + ty * (-pad)),
            (cx + nx * r_outer + tx * pad, cy + ny * r_outer + ty * pad),
            (cx + nx * r_inner + tx * pad, cy + ny * r_inner + ty * pad),
        ])
    except (TypeError, ValueError):
        return None


def seaward_edge_keepout(
        mudflat_polys, coast_geoms, origin_xy,
        inset_m=DEFAULT_EDGE_INSET_M,
        inset_ratio=DEFAULT_EDGE_INSET_RATIO,
        search_m=DEFAULT_RADIUS_M,
        along_half_m=DEFAULT_ALONG_HALF_M,
        coast_clear_m=DEFAULT_COAST_CLEAR_M,
        station_m=DEFAULT_STATION_M):
    """본안 C 를 따라 로컬 바깥 변의 10% 띠.

    핀 한 점 폭으로 C 를 밀지 않는다. 각 역의 법선 last-inside 가 폭.
    inset_m > 0 이면 그 미터, 아니면 로컬 폭 × inset_ratio.
    조위로 줄이지 않는다. 해안선(C) 쪽은 막지 않는다.
    """
    search = _positive_m(search_m)
    along = _positive_m(along_half_m)
    station = _positive_m(station_m) or DEFAULT_STATION_M
    if search is None or along is None:
        return None
    inset_fixed = _positive_m(inset_m)
    try:
        ratio = float(inset_ratio)
    except (TypeError, ValueError):
        ratio = DEFAULT_EDGE_INSET_RATIO
    if not (0.0 < ratio < 1.0):
        ratio = DEFAULT_EDGE_INSET_RATIO
    origin = _xy_pair(origin_xy)
    if origin is None or ShapelyPoint is None:
        return None
    clear = _positive_m(coast_clear_m) or 0.0
    mud = []
    for poly in mudflat_polys or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            mud.append(valid)
    if not mud:
        return None
    seed = clip_geoms_to_window(coast_geoms, origin_xy, 800.0)
    seed_coast = _coast_union(seed) or _coast_union(coast_geoms)
    if seed_coast is None:
        return None
    nearest = _nearest_coast_point(origin, seed_coast)
    if nearest is None:
        return None
    coast_pt = ShapelyPoint(nearest.x, nearest.y)
    mud_seed = min(mud, key=lambda item: item.distance(coast_pt))
    seaward = _seaward_hint(
        origin, (nearest.x, nearest.y), [mud_seed])
    if seaward is None:
        return None
    shore_box = _alongshore_corridor(
        origin, seaward, along, -800.0, 1500.0)
    if shore_box is None:
        local_coast = clip_geoms_to_window(coast_geoms, origin_xy, along)
    else:
        local_coast = []
        for geom in coast_geoms or ():
            line = _as_line(geom)
            if line is None:
                continue
            try:
                if shore_box.intersects(line):
                    local_coast.append(line)
            except (TypeError, ValueError):
                continue
    if not local_coast:
        return None
    simplified = []
    for geom in local_coast:
        line = _as_line(geom)
        if line is None:
            continue
        try:
            line = line.simplify(
                DEFAULT_SIMPLIFY_M, preserve_topology=True)
        except (TypeError, ValueError):
            pass
        if line is not None and not line.is_empty and line.length > 0.0:
            simplified.append(line)
    coast = _coast_union(simplified) or _coast_union(local_coast)
    if coast is None:
        return None
    sample_lines = []
    for line in _coast_line_parts(coast):
        try:
            if line.distance(mud_seed) <= _ATTACH_M:
                sample_lines.append(line)
        except (TypeError, ValueError):
            continue
    if not sample_lines:
        sample_lines = _coast_line_parts(coast)
    faces = [mud_seed]
    for face in mud:
        if face is mud_seed:
            continue
        try:
            if any(face.distance(line) <= _ATTACH_M for line in sample_lines):
                faces.append(face)
        except (TypeError, ValueError):
            continue
    if len(faces) == 1:
        mud_work = mud_seed
    elif shapely_unary_union is not None:
        merged = shapely_unary_union(faces)
        if merged is not None and not merged.is_empty:
            mud_work = merged
        else:
            mud_work = mud_seed
    else:
        mud_work = mud_seed
    quads = []
    half = station * _QUAD_OVERLAP
    for line in sample_lines:
        count = max(1, int(line.length / station))
        for index in range(count + 1):
            point = line.interpolate(index / count, normalized=True)
            tang = None
            dist = min(line.length, max(0.0, line.project(point)))
            p0 = line.interpolate(max(0.0, dist - 1.0))
            p1 = line.interpolate(min(line.length, dist + 1.0))
            tang = _unit(p1.x - p0.x, p1.y - p0.y)
            if tang is None:
                continue
            normal = _normal_into_mud(
                (point.x, point.y), tang, mud_work)
            if normal is None:
                continue
            width = _last_inside_along(
                (point.x, point.y), normal, mud_work,
                search, _WIDTH_STEP_M)
            if width < DEFAULT_MIN_WIDTH_M:
                continue
            if inset_fixed is not None:
                inset = inset_fixed
            else:
                inset = width * ratio
            quad = _keepout_quad(
                point.x, point.y, normal, tang, width, inset, half,
                min_inner=clear)
            if quad is None or quad.is_empty:
                continue
            quads.append(quad)
    if not quads or shapely_unary_union is None:
        return None
    try:
        band = shapely_unary_union(quads)
    except (TypeError, ValueError):
        return None
    if band is None or band.is_empty:
        return None
    try:
        wet = mud_work.intersection(band)
    except (TypeError, ValueError):
        return None
    if wet is None or wet.is_empty:
        return None
    rings = _rings_all(wet, min_area=200.0)
    if not rings:
        return None
    return rings


def corridor_outer_keepout(
        mudflat_polys, coast_geoms,
        corridor_m=DEFAULT_CORRIDOR_M,
        outer_ratio=DEFAULT_CORRIDOR_OUTER_RATIO,
        simplify_m=DEFAULT_CORRIDOR_SIMPLIFY_M,
        min_area_m2=DEFAULT_CORRIDOR_MIN_AREA_M2,
        origin_xy=None,
        radius_m=None):
    """해안 복도의 바깥 비율 띠. M ∩ (C.buffer(w) − C.buffer((1−r)w)).

    last-inside 가 아니다. 폭이 corridor 보다 좁은 면은 비는다.
    origin/radius 가 있으면 그 창만 쓴다.
    """
    width = _positive_m(corridor_m)
    try:
        ratio = float(outer_ratio)
    except (TypeError, ValueError):
        ratio = DEFAULT_CORRIDOR_OUTER_RATIO
    if width is None or not (0.0 < ratio < 1.0):
        return []
    if shapely_unary_union is None:
        return []
    mud_src = mudflat_polys
    coast_src = coast_geoms
    if origin_xy is not None and radius_m is not None:
        mud_src = clip_geoms_to_window(mudflat_polys, origin_xy, radius_m)
        coast_src = clip_geoms_to_window(coast_geoms, origin_xy, radius_m)
    mud = []
    for poly in mud_src or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            mud.append(valid)
    if not mud:
        return []
    coast = _coast_union(coast_src)
    if coast is None:
        return []
    try:
        coast = coast.simplify(DEFAULT_SIMPLIFY_M, preserve_topology=True)
    except (TypeError, ValueError):
        pass
    if coast is None or coast.is_empty:
        return []
    mud_union = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
    if mud_union is None or mud_union.is_empty:
        return []
    inner = width * (1.0 - ratio)
    try:
        band = coast.buffer(width).difference(coast.buffer(inner))
        keep = mud_union.intersection(band)
    except (TypeError, ValueError):
        return []
    if keep is None or keep.is_empty:
        return []
    simp = _positive_m(simplify_m)
    if simp is not None:
        try:
            keep = keep.simplify(simp, preserve_topology=True)
        except (TypeError, ValueError):
            pass
    return _rings_all(keep, min_area=min_area_m2 or 0.0)


def _line_parts(geom):
    """Difference 결과가 선·다중선·컬렉션이어도 선만 남긴다."""
    if geom is None or getattr(geom, 'is_empty', True):
        return []
    gtype = getattr(geom, 'geom_type', '')
    if gtype == 'LineString' and geom.length > 0.0:
        return [geom]
    if gtype == 'MultiLineString':
        return [part for part in geom.geoms if part.length > 0.0]
    if gtype == 'GeometryCollection':
        parts = []
        for part in geom.geoms:
            parts.extend(_line_parts(part))
        return parts
    return []


def mudflat_perimeter_keepout(
        mudflat_polys, coast_geoms,
        coast_clear_m=DEFAULT_COAST_EDGE_CLEAR_M,
        inset_ratio=DEFAULT_PERIMETER_RATIO,
        inset_m=0.0,
        min_band_m=DEFAULT_PERIMETER_MIN_BAND_M,
        min_line_m=DEFAULT_PERIMETER_MIN_LINE_M,
        simplify_m=DEFAULT_CORRIDOR_SIMPLIFY_M,
        min_area_m2=DEFAULT_CORRIDOR_MIN_AREA_M2,
        origin_xy=None,
        radius_m=None):
    """합친 갯벌의 바깥 변만 띠로. 맞닿은 변·해안선 변은 막지 않는다.

    면들을 union 하면 갯벌-갯벌 공유 변은 사라진다.
    남은 둘레 중 C 근처(coast_clear_m)는 철수 통로로 연다.
    그 나머지 변을 갯벌 안으로 밀어 끊기지 않는 띠를 만든다.
    C 에서 고정 거리 고리가 아니다. 폭이 좁은 끝도 바깥 변이 있으면 이어진다.
    """
    if shapely_unary_union is None:
        return []
    mud_src = mudflat_polys
    coast_src = coast_geoms
    clip_box = None
    if origin_xy is not None and radius_m is not None:
        mud_src = clip_geoms_to_window(mudflat_polys, origin_xy, radius_m)
        coast_src = clip_geoms_to_window(coast_geoms, origin_xy, radius_m)
        clip_box = _window_box(origin_xy, radius_m)
    mud = []
    for poly in mud_src or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            mud.append(valid)
    if not mud:
        return []
    mud_union = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
    if mud_union is None or mud_union.is_empty:
        return []
    coast = _coast_union(coast_src)
    if coast is None:
        return []
    try:
        coast = coast.simplify(DEFAULT_SIMPLIFY_M, preserve_topology=True)
    except (TypeError, ValueError):
        pass
    if coast is None or coast.is_empty:
        return []
    clear = _positive_m(coast_clear_m) or DEFAULT_COAST_EDGE_CLEAR_M
    try:
        seaward = mud_union.boundary.difference(coast.buffer(clear))
    except (TypeError, ValueError):
        return []
    if clip_box is not None:
        try:
            seaward = seaward.difference(clip_box.boundary.buffer(2.0))
        except (TypeError, ValueError):
            pass
    lines = _line_parts(seaward)
    min_line = _positive_m(min_line_m) or DEFAULT_PERIMETER_MIN_LINE_M
    lines = [line for line in lines if line.length >= min_line]
    if not lines:
        return []
    band = _positive_m(inset_m)
    if band is None:
        try:
            ratio = float(inset_ratio)
        except (TypeError, ValueError):
            ratio = DEFAULT_PERIMETER_RATIO
        if not (0.0 < ratio < 1.0):
            ratio = DEFAULT_PERIMETER_RATIO
        dists = []
        for line in lines:
            count = max(1, int(line.length / 40.0))
            for index in range(count + 1):
                point = line.interpolate(index / count, normalized=True)
                dists.append(coast.distance(point))
        dists.sort()
        typical = dists[len(dists) // 2] if dists else 0.0
        floor = _positive_m(min_band_m) or DEFAULT_PERIMETER_MIN_BAND_M
        band = max(floor, typical * ratio)
    try:
        fat = shapely_unary_union(lines).buffer(band)
        keep = mud_union.intersection(fat)
    except (TypeError, ValueError):
        return []
    if keep is None or keep.is_empty:
        return []
    simp = _positive_m(simplify_m)
    if simp is not None:
        try:
            keep = keep.simplify(simp, preserve_topology=True)
        except (TypeError, ValueError):
            pass
    rings = _rings_all(keep, min_area=min_area_m2 or 0.0)
    return filter_keepout_rings(rings, min_line)


def filter_keepout_rings(rings, min_line_m=None):
    """짧은 링을 버린다. 기준은 bbox 긴 변."""
    try:
        min_line = float(min_line_m) if min_line_m is not None else (
            DEFAULT_PERIMETER_MIN_LINE_M)
    except (TypeError, ValueError):
        min_line = DEFAULT_PERIMETER_MIN_LINE_M
    if min_line <= 0.0:
        return list(rings or ())
    kept = []
    for ring in rings or ():
        if not ring or len(ring) < 3:
            continue
        xs = [pt[0] for pt in ring]
        ys = [pt[1] for pt in ring]
        if max(max(xs) - min(xs), max(ys) - min(ys)) >= min_line:
            kept.append(ring)
    return kept


def rings_5186_to_map(rings, pin_xy, robot_xy=(0.0, 0.0)):
    """EPSG:5186 링을 map 으로. W_map = robot + (W − P)."""
    pin = _xy_pair(pin_xy)
    robot = _xy_pair(robot_xy) or (0.0, 0.0)
    if pin is None:
        return []
    mapped = []
    for ring in rings or ():
        pts = []
        for raw in ring:
            xy = _xy_pair(raw)
            if xy is None:
                pts = []
                break
            pts.append((
                robot[0] + xy[0] - pin[0],
                robot[1] + xy[1] - pin[1],
            ))
        if len(pts) >= 2:
            mapped.append(pts)
    return mapped


def dump_keepout_geojson(path, rings, properties=None):
    """EPSG:5186 FeatureCollection 으로 링을 쓴다. 피처 수를 반환."""
    import json
    if not path:
        raise ValueError('keepout geojson 경로가 비었다')
    features = []
    props = dict(properties or {})
    for ring in rings or ():
        if len(ring) < 3:
            continue
        coords = [[float(pt[0]), float(pt[1])] for pt in ring]
        if coords[0] != coords[-1]:
            coords.append(list(coords[0]))
        features.append({
            'type': 'Feature',
            'properties': props,
            'geometry': {
                'type': 'Polygon',
                'coordinates': [coords],
            },
        })
    payload = {
        'type': 'FeatureCollection',
        'name': 'tars_keepout_corridor_outer',
        'crs': {
            'type': 'name',
            'properties': {'name': 'urn:ogc:def:crs:EPSG::5186'},
        },
        'features': features,
    }
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle)
    return len(features)


def load_keepout_geojson(path):
    """구운 GeoJSON 을 TideLayer 링 목록으로 읽는다. 외곽만."""
    import json
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f'keepout geojson 이 없다: {path}')
    with open(path, encoding='utf-8') as handle:
        data = json.load(handle)
    features = []
    if isinstance(data, dict):
        if data.get('type') == 'FeatureCollection':
            features = data.get('features') or []
        elif data.get('type') == 'Feature':
            features = [data]
        elif data.get('type') in ('Polygon', 'MultiPolygon'):
            features = [{'geometry': data}]
    rings = []
    for feat in features:
        if not isinstance(feat, dict):
            continue
        geom = feat.get('geometry') or {}
        gtype = geom.get('type')
        coords = geom.get('coordinates') or []
        exteriors = []
        if gtype == 'Polygon' and coords:
            exteriors.append(coords[0])
        elif gtype == 'MultiPolygon':
            for poly in coords:
                if poly:
                    exteriors.append(poly[0])
        for raw in exteriors:
            ring = []
            for pt in raw:
                xy = _xy_pair(pt)
                if xy is not None:
                    ring.append(xy)
            if len(ring) >= 2 and ring[0] == ring[-1]:
                ring = ring[:-1]
            if len(ring) >= 3:
                rings.append(ring)
    return rings


def waterline_steps_filename(site):
    """맵 이름 → 시간대 수위선 파일. waterline_<이름>_steps.json."""
    name = str(site or '').strip()
    if not name or '/' in name or name in ('.', '..'):
        return ''
    return f'waterline_{name}_steps.json'


def resolve_waterline_steps_path(param_path='', site=''):
    """파라미터 → site 이름 → 소스 config 순으로 수위선 스텝 파일을 찾는다."""
    candidates = []
    if param_path:
        candidates.append(str(param_path))
    fname = waterline_steps_filename(site)
    here = os.path.dirname(os.path.abspath(__file__))
    if fname:
        candidates.append(os.path.normpath(os.path.join(
            here, '..', 'config', fname)))
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('eclipse_pkg')
        if fname:
            candidates.append(os.path.join(share, 'config', fname))
    except Exception:
        pass
    seen = set()
    for item in candidates:
        if not item or item in seen:
            continue
        seen.add(item)
        if os.path.isfile(item):
            return item
    return ''


def dump_waterline_steps(path, steps, meta=None):
    """α별 수위선 링을 JSON 으로 저장. 피처(스텝) 수를 반환."""
    import json
    if not path:
        raise ValueError('waterline steps 경로가 비었다')
    payload = dict(meta or {})
    payload['steps'] = []
    for item in steps or ():
        try:
            alpha = round(float(item[0]), 2)
            rings = item[1]
        except (TypeError, ValueError, IndexError):
            continue
        clean = []
        for ring in rings or ():
            pts = []
            for raw in ring:
                xy = _xy_pair(raw)
                if xy is None:
                    pts = []
                    break
                pts.append([xy[0], xy[1]])
            if len(pts) >= 2:
                clean.append(pts)
        payload['steps'].append({'alpha': alpha, 'rings': clean})
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle)
    return len(payload['steps'])


def load_waterline_steps(path):
    """구운 수위선 스텝 [(alpha, rings), ...] 오름차순."""
    import json
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f'waterline steps 가 없다: {path}')
    with open(path, encoding='utf-8') as handle:
        data = json.load(handle)
    raw_steps = []
    if isinstance(data, dict):
        raw_steps = data.get('steps') or []
    out = []
    for item in raw_steps:
        if not isinstance(item, dict):
            continue
        try:
            alpha = round(float(item.get('alpha')), 2)
        except (TypeError, ValueError):
            continue
        rings = []
        for ring in item.get('rings') or ():
            pts = []
            for raw in ring:
                xy = _xy_pair(raw)
                if xy is None:
                    pts = []
                    break
                pts.append(xy)
            if len(pts) >= 2:
                rings.append(pts)
        out.append((alpha, rings))
    out.sort(key=lambda row: row[0])
    return out


def waterline_rings_at_alpha(steps, alpha):
    """가장 가까운 α 스텝의 링."""
    if not steps:
        return []
    try:
        target = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        target = 0.0
    best = steps[0]
    best_d = abs(best[0] - target)
    for item in steps[1:]:
        dist = abs(item[0] - target)
        if dist < best_d:
            best = item
            best_d = dist
    return list(best[1])


WINDOW_BORDER_PAD_M = 24.0
WATERLINE_STITCH_M = 250.0
COAST_NEAR_MUD_M = 80.0
COAST_MIN_LEN_M = 80.0


def _aabb4(aabb):
    """(minx, miny, maxx, maxy). dict 또는 4튜플."""
    if not aabb:
        return None
    if isinstance(aabb, dict):
        try:
            return (
                float(aabb['minx']), float(aabb['miny']),
                float(aabb['maxx']), float(aabb['maxy']))
        except (KeyError, TypeError, ValueError):
            return None
    try:
        minx, miny, maxx, maxy = aabb
        return (float(minx), float(miny), float(maxx), float(maxy))
    except (TypeError, ValueError):
        return None


def _border_sides(x_val, y_val, aabb, pad_m):
    minx, miny, maxx, maxy = aabb
    pad = float(pad_m)
    sides = []
    if x_val <= minx + pad:
        sides.append('W')
    if x_val >= maxx - pad:
        sides.append('E')
    if y_val <= miny + pad:
        sides.append('S')
    if y_val >= maxy - pad:
        sides.append('N')
    return sides


def drop_window_border_segments(rings, aabb, pad_m=WINDOW_BORDER_PAD_M):
    """타일 창 변을 따라가는 선분만 잘라 연다. 상자 유령 변 제거."""
    box = _aabb4(aabb)
    if box is None:
        return [list(ring) for ring in rings or () if ring and len(ring) >= 2]
    try:
        pad = float(pad_m)
    except (TypeError, ValueError):
        pad = WINDOW_BORDER_PAD_M
    if not math.isfinite(pad) or pad < 0.0:
        pad = WINDOW_BORDER_PAD_M
    out = []
    for ring in rings or ():
        if not ring or len(ring) < 2:
            continue
        pts = [xy for xy in (_xy_pair(pt) for pt in ring) if xy is not None]
        if len(pts) < 2:
            continue
        closed = (
            math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
            <= max(1.0, pad))
        edges = list(zip(pts, pts[1:]))
        if closed:
            edges.append((pts[-1], pts[0]))
        current = []
        for start, end in edges:
            share = set(_border_sides(start[0], start[1], box, pad)) & set(
                _border_sides(end[0], end[1], box, pad))
            if share:
                if len(current) >= 2:
                    out.append(current)
                current = []
                continue
            if not current:
                current = [start, end]
            else:
                current.append(end)
        if len(current) >= 2:
            out.append(current)
    return out


def stitch_waterline_rings(rings, join_m=WATERLINE_STITCH_M):
    """끝점이 가까운 수위선 조각을 잇는다."""
    try:
        join = float(join_m)
    except (TypeError, ValueError):
        join = WATERLINE_STITCH_M
    if not math.isfinite(join) or join < 0.0:
        join = WATERLINE_STITCH_M
    clean = []
    for ring in rings or ():
        pts = [xy for xy in (_xy_pair(pt) for pt in ring) if xy is not None]
        if len(pts) >= 2:
            clean.append(pts)
    return _stitch_polylines(clean, join_m=join)


def _line_length(coords):
    length = 0.0
    for a_pt, b_pt in zip(coords, coords[1:]):
        length += math.hypot(b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
    return length


def _iter_lines(geom):
    if geom is None or getattr(geom, 'is_empty', True):
        return
    gtype = getattr(geom, 'geom_type', '')
    if gtype == 'LineString':
        if geom.length > 0.0:
            yield geom
        return
    geoms = getattr(geom, 'geoms', None)
    if geoms is None:
        return
    for part in geoms:
        yield from _iter_lines(part)


def coast_along_mud_rings(
        coast_geoms, mud_polys,
        near_m=COAST_NEAR_MUD_M,
        min_len_m=COAST_MIN_LEN_M,
        join_m=WATERLINE_STITCH_M):
    """α=1: 갯벌 복도와 겹치는 해안만 클립·이어붙임. 조각 keep/drop 아님."""
    if shapely_unary_union is None:
        return []
    mud = []
    for poly in mud_polys or ():
        for part in _explode_polygons(poly) or ():
            valid = _valid_polygon(part)
            if valid is not None and valid.area > 0.0:
                mud.append(valid)
    if not mud:
        return []
    try:
        near = float(near_m)
        min_len = float(min_len_m)
    except (TypeError, ValueError):
        near, min_len = COAST_NEAR_MUD_M, COAST_MIN_LEN_M
    if not math.isfinite(near) or near <= 0.0:
        near = COAST_NEAR_MUD_M
    mud_u = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
    try:
        corridor = mud_u.buffer(near)
    except (TypeError, ValueError):
        return []
    if corridor is None or corridor.is_empty:
        return []
    segs = []
    for raw in coast_geoms or ():
        line = _as_line(raw)
        if line is None or line.is_empty:
            continue
        try:
            hit = line.intersection(corridor)
        except (TypeError, ValueError):
            continue
        for part in _iter_lines(hit):
            coords = [
                xy for xy in (_xy_pair(c) for c in part.coords)
                if xy is not None]
            if len(coords) >= 2:
                segs.append(coords)
    stitched = stitch_waterline_rings(segs, join_m=join_m)
    return [row for row in stitched if _line_length(row) >= min_len]


def cleanup_waterline_rings(
        rings, aabb=None, pad_m=WINDOW_BORDER_PAD_M,
        join_m=WATERLINE_STITCH_M, stitch=True):
    """창 변 유령 선 제거 후 (선택) stitch."""
    cleaned = drop_window_border_segments(rings, aabb, pad_m=pad_m)
    if stitch:
        cleaned = stitch_waterline_rings(cleaned, join_m=join_m)
    return cleaned


def keepout_geojson_filename(site):
    """맵 이름 → config 파일명. keepout_<이름>_perimeter.geojson."""
    name = str(site or '').strip()
    if not name or '/' in name or name in ('.', '..'):
        return ''
    return f'keepout_{name}_perimeter.geojson'


def resolve_keepout_geojson_path(param_path='', site=''):
    """파라미터 → site 이름 → 패키지 share → 소스 config 순으로 구운 파일을 찾는다."""
    candidates = []
    if param_path:
        candidates.append(str(param_path))
    fname = keepout_geojson_filename(site)
    if fname:
        candidates.append(os.path.join(
            '/workspaces/eclipse-test-2', 'src', 'eclipse_pkg', 'config', fname))
        here = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.normpath(os.path.join(
            here, '..', 'config', fname)))
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('eclipse_pkg')
        if fname:
            candidates.append(os.path.join(share, 'config', fname))
        candidates.append(os.path.join(
            share, 'config', DEFAULT_KEEPOUT_GEOJSON_NAME))
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.normpath(os.path.join(
        here, '..', 'config', DEFAULT_KEEPOUT_GEOJSON_NAME)))
    seen = set()
    for item in candidates:
        if not item or item in seen:
            continue
        seen.add(item)
        if os.path.isfile(item):
            return item
    return ''


def rings_to_polygons(rings):
    """링 좌표 → 유효 Polygon 목록."""
    polys = []
    for ring in rings or ():
        if not ring or len(ring) < 3 or ShapelyPolygon is None:
            continue
        try:
            poly = ShapelyPolygon(ring)
        except (TypeError, ValueError):
            continue
        valid = _valid_polygon(poly)
        if valid is not None:
            polys.append(valid)
    return polys


def point_on_mudflat(xy, mud_polys):
    """점이 갯벌 면 안(경계 포함)이면 True. 면이 없으면 None."""
    point_xy = _xy_pair(xy)
    if point_xy is None or ShapelyPoint is None:
        return None
    polys = []
    for item in mud_polys or ():
        valid = _valid_polygon(item)
        if valid is not None:
            polys.append(valid)
    if not polys:
        return None
    query = ShapelyPoint(point_xy[0], point_xy[1])
    return any(poly.covers(query) for poly in polys)


def keepout_grow_span_m(
        keepout_rings, coast_geoms, fallback_m=400.0, max_m=5000.0):
    """keepout 꼭짓점 → 해안 거리의 중앙값. α=1 일 때 성장 거리."""
    try:
        fallback = float(fallback_m)
    except (TypeError, ValueError):
        fallback = 400.0
    try:
        cap = float(max_m)
    except (TypeError, ValueError):
        cap = 5000.0
    coast = _coast_union(coast_geoms)
    if coast is None or getattr(coast, 'is_empty', True):
        return fallback
    dists = []
    for ring in keepout_rings or ():
        if not ring:
            continue
        step = max(1, len(ring) // 40)
        for point in ring[::step]:
            xy = _xy_pair(point)
            if xy is None or ShapelyPoint is None:
                continue
            try:
                dist = float(coast.distance(ShapelyPoint(xy[0], xy[1])))
            except (TypeError, ValueError):
                continue
            if math.isfinite(dist) and 0.0 < dist <= cap:
                dists.append(dist)
    if not dists:
        return fallback
    dists.sort()
    # 넓은 갯벌이 만조에 해안까지 차 보이게 90백분위.
    return dists[int(0.90 * (len(dists) - 1))]


def grow_keepout_geom(
        keepout_rings, mud_polys, alpha, grow_m):
    """구운 keepout 을 갯벌 안에서 키운 shapely 면. 실패면 None.

    wet = K.buffer(α · grow_m) ∩ M.
    """
    if shapely_unary_union is None or ShapelyPolygon is None:
        return None
    try:
        weight = max(0.0, min(1.0, float(alpha)))
        reach = max(0.0, float(grow_m))
    except (TypeError, ValueError):
        return None
    keep_polys = rings_to_polygons(keepout_rings)
    if not keep_polys:
        return None
    keep = keep_polys[0] if len(keep_polys) == 1 else shapely_unary_union(
        keep_polys)
    if keep is None or keep.is_empty:
        return None
    try:
        grown = keep if reach == 0.0 or weight == 0.0 else keep.buffer(
            weight * reach)
    except (TypeError, ValueError):
        return None
    mud = []
    for item in mud_polys or ():
        valid = _valid_polygon(item)
        if valid is not None:
            mud.append(valid)
    if mud:
        mud_union = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
        if mud_union is not None and not mud_union.is_empty:
            try:
                grown = grown.intersection(mud_union)
            except (TypeError, ValueError):
                return None
    if grown is None or getattr(grown, 'is_empty', True):
        return None
    return grown


def keep_grown_off_coast(grown, coast_geoms, gap_m):
    """2026 해안에서 gap_m 안을 비운다. α=1(gap=0)이면 그대로.

    전역 buffer 가 좁은 갯벌에서 해안까지 차는 것을 막는다.
    창 밖 전국 해안은 union 하지 않는다.
    """
    if grown is None or getattr(grown, 'is_empty', True):
        return grown
    gap = _positive_m(gap_m)
    if gap is None:
        return grown
    try:
        search = grown.envelope.buffer(gap + 50.0)
    except (TypeError, ValueError):
        search = None
    nearby = []
    for item in coast_geoms or ():
        line = _as_line(item)
        if line is None or line.is_empty:
            continue
        try:
            if search is None or line.intersects(search):
                nearby.append(line)
        except (TypeError, ValueError):
            continue
    if not nearby:
        return grown
    coast = nearby[0] if len(nearby) == 1 else shapely_unary_union(nearby)
    if coast is None or getattr(coast, 'is_empty', True):
        return grown
    try:
        coast = coast.simplify(20.0, preserve_topology=True)
    except (TypeError, ValueError):
        pass
    try:
        cleared = grown.difference(coast.buffer(gap))
    except (TypeError, ValueError):
        return grown
    if cleared is None or getattr(cleared, 'is_empty', True):
        return None
    return cleared


def _linear_ring_xy(ring):
    """LinearRing / coords → 닫히지 않은 (x,y) 목록."""
    try:
        coords = list(ring.coords)
    except (TypeError, ValueError, AttributeError):
        return None
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) < 3:
        return None
    pts = []
    for raw in coords:
        xy = _xy_pair(raw)
        if xy is None:
            return None
        pts.append(xy)
    return pts if len(pts) >= 3 else None


def waterline_front_rings(geom):
    """해안 쪽 수위선. 구멍이 있으면 구멍만, 없으면 외곽.

    keepout.buffer 의 외곽은 바다 쪽이라 keepout 과 겹쳐 보인다.
    해안(C) 쪽 전선은 보통 interior 구멍이다.
    """
    rings = []
    for part in _explode_polygons(geom):
        if part is None or part.geom_type != 'Polygon':
            continue
        holes = []
        try:
            holes = list(part.interiors)
        except (TypeError, ValueError, AttributeError):
            holes = []
        if holes:
            for hole in holes:
                xy_ring = _linear_ring_xy(hole)
                if xy_ring:
                    rings.append(xy_ring)
            continue
        outer = simple_ring(part)
        if outer:
            rings.append(outer)
    return rings


def _lines_to_xy(geom, min_len_m=40.0):
    """LineString 조각을 (x,y) 폴리라인으로."""
    try:
        floor = float(min_len_m)
    except (TypeError, ValueError):
        floor = 40.0
    lines = []
    for part in _line_parts(geom):
        try:
            if part.length < floor:
                continue
            coords = list(part.coords)
        except (TypeError, ValueError, AttributeError):
            continue
        pts = []
        for raw in coords:
            xy = _xy_pair(raw)
            if xy is None:
                pts = []
                break
            pts.append(xy)
        if len(pts) >= 2:
            lines.append(pts)
    return lines


def waterline_inland_front(keepout_rings, grown, reach_m):
    """keepout 에서 떨어진 grown 경계만. 바다 쪽 겹침·선착장 구멍만 남는 것 방지.

    α=0 이면 keepout 링. 그 외는 grown.boundary − keep.buffer(margin).
    """
    try:
        reach = max(0.0, float(reach_m))
    except (TypeError, ValueError):
        reach = 0.0
    if grown is None or getattr(grown, 'is_empty', True):
        return []
    if reach < 1.0:
        return _rings_all(grown)
    keep_polys = rings_to_polygons(keepout_rings)
    if not keep_polys or shapely_unary_union is None:
        return waterline_front_rings(grown)
    keep = keep_polys[0] if len(keep_polys) == 1 else shapely_unary_union(
        keep_polys)
    if keep is None or keep.is_empty:
        return waterline_front_rings(grown)
    margin = max(12.0, 0.2 * reach)
    try:
        front = grown.boundary.difference(keep.buffer(margin))
    except (TypeError, ValueError):
        return waterline_front_rings(grown)
    if front is None or getattr(front, 'is_empty', True):
        return waterline_front_rings(grown)
    return _lines_to_xy(front, min_len_m=max(30.0, 0.05 * reach))


def _grid_bounds(geoms, pad_m):
    xmin = ymin = xmax = ymax = None
    for geom in geoms or ():
        if geom is None or getattr(geom, 'is_empty', True):
            continue
        try:
            bx0, by0, bx1, by1 = geom.bounds
        except (TypeError, ValueError):
            continue
        xmin = bx0 if xmin is None else min(xmin, bx0)
        ymin = by0 if ymin is None else min(ymin, by0)
        xmax = bx1 if xmax is None else max(xmax, bx1)
        ymax = by1 if ymax is None else max(ymax, by1)
    if xmin is None:
        return None
    pad = float(pad_m)
    return (xmin - pad, ymin - pad, xmax + pad, ymax + pad)


def _rasterize_polys(mask, polys, xmin, ymin, cell_m):
    try:
        from skimage.draw import polygon as draw_polygon
    except ImportError:
        return
    nrows, ncols = mask.shape
    for poly in polys or ():
        for part in _explode_polygons(poly) or ():
            if part is None or part.geom_type != 'Polygon':
                continue
            try:
                ext = list(part.exterior.coords)
            except (TypeError, ValueError, AttributeError):
                continue
            if len(ext) < 3:
                continue
            rows = [(pt[1] - ymin) / cell_m for pt in ext]
            cols = [(pt[0] - xmin) / cell_m for pt in ext]
            rr, cc = draw_polygon(rows, cols, shape=(nrows, ncols))
            mask[rr, cc] = True
            try:
                holes = list(part.interiors)
            except (TypeError, ValueError, AttributeError):
                holes = []
            for hole in holes:
                try:
                    hxy = list(hole.coords)
                except (TypeError, ValueError, AttributeError):
                    continue
                if len(hxy) < 3:
                    continue
                hrows = [(pt[1] - ymin) / cell_m for pt in hxy]
                hcols = [(pt[0] - xmin) / cell_m for pt in hxy]
                hrr, hcc = draw_polygon(hrows, hcols, shape=(nrows, ncols))
                mask[hrr, hcc] = False


def _rasterize_lines(mask, lines, xmin, ymin, cell_m):
    try:
        from skimage.draw import line as draw_line
    except ImportError:
        return
    nrows, ncols = mask.shape
    for item in lines or ():
        line = _as_line(item)
        if line is None:
            continue
        parts = [line] if line.geom_type == 'LineString' else _coast_line_parts(line)
        for part in parts:
            try:
                coords = list(part.coords)
            except (TypeError, ValueError, AttributeError):
                continue
            for start, end in zip(coords, coords[1:]):
                r0 = int(round((start[1] - ymin) / cell_m))
                c0 = int(round((start[0] - xmin) / cell_m))
                r1 = int(round((end[1] - ymin) / cell_m))
                c1 = int(round((end[0] - xmin) / cell_m))
                rr, cc = draw_line(r0, c0, r1, c1)
                keep_r = []
                keep_c = []
                for row, col in zip(rr, cc):
                    if 0 <= row < nrows and 0 <= col < ncols:
                        keep_r.append(row)
                        keep_c.append(col)
                if keep_r:
                    mask[keep_r, keep_c] = True


def _geodesic_dist(mud, seeds, cell_m):
    """씨앗까지 거리(m). 육지 칸은 inf.

    전 창 MCP 는 너무 느려 EDT 를 쓴다. 갯벌이 열린 구간에서는
    해안·기준면이 같은 면에 있어 로컬 폭과 같다.
    """
    import numpy as np
    from scipy.ndimage import binary_dilation, distance_transform_edt
    seed_m = binary_dilation(seeds, iterations=2)
    seed_m = np.logical_and(seed_m, mud)
    if not seed_m.any():
        seed_m = seeds
    if not seed_m.any():
        return np.full(mud.shape, np.inf)
    inv = np.logical_not(seed_m)
    dist = distance_transform_edt(inv) * float(cell_m)
    dist[np.logical_not(mud)] = np.inf
    return dist


def build_local_ratio_grid(
        keepout_rings, mud_polys, coast_geoms, cell_m=12.0):
    """해안·기준면 거리장을 한 번 만든다. α 루프에서 재사용."""
    import numpy as np
    try:
        cell = float(cell_m)
    except (TypeError, ValueError):
        cell = 12.0
    if cell <= 0.0:
        cell = 12.0
    mud = []
    for item in mud_polys or ():
        valid = _valid_polygon(item)
        if valid is not None:
            mud.append(valid)
    keep_polys = rings_to_polygons(keepout_rings)
    if not mud or not keep_polys:
        return []
    bounds = _grid_bounds(mud + keep_polys, cell * 4.0)
    if bounds is None:
        return []
    xmin, ymin, xmax, ymax = bounds
    ncols = int(math.ceil((xmax - xmin) / cell)) + 1
    nrows = int(math.ceil((ymax - ymin) / cell)) + 1
    if nrows < 4 or ncols < 4 or nrows * ncols > 12_000_000:
        return []
    mud_mask = np.zeros((nrows, ncols), dtype=bool)
    keep_mask = np.zeros((nrows, ncols), dtype=bool)
    coast_mask = np.zeros((nrows, ncols), dtype=bool)
    _rasterize_polys(mud_mask, mud, xmin, ymin, cell)
    _rasterize_polys(keep_mask, keep_polys, xmin, ymin, cell)
    _rasterize_lines(coast_mask, coast_geoms, xmin, ymin, cell)
    if not mud_mask.any() or not keep_mask.any() or not coast_mask.any():
        return None
    print('  ratio-grid geodesic (slow once)...', flush=True)
    d_c = _geodesic_dist(mud_mask, coast_mask, cell)
    d_k = _geodesic_dist(mud_mask, keep_mask, cell)
    denom = d_c + d_k
    ratio = np.full(mud_mask.shape, np.nan, dtype=np.float64)
    ok = np.logical_and(mud_mask, denom > cell * 0.5)
    ratio[ok] = d_c[ok] / denom[ok]
    keep_u = (
        keep_polys[0] if len(keep_polys) == 1
        else shapely_unary_union(keep_polys))
    return {
        'mud_mask': mud_mask,
        'ratio': ratio,
        'ok': ok,
        'd_c': d_c,
        'd_k': d_k,
        'xmin': xmin,
        'ymin': ymin,
        'cell': cell,
        'keepout_rings': keepout_rings,
        'coast_geoms': coast_geoms,
        'keep_u': keep_u,
        'coast_u': _coast_union(coast_geoms),
        'mud': mud,
    }


def rings_from_local_ratio_grid(grid, alpha, min_len_m=40.0):
    """거리장에서 α 등고선. α=0 기준면, α=1 해안."""
    import numpy as np
    if not grid:
        return []
    try:
        from skimage.measure import find_contours
    except ImportError:
        return []
    try:
        weight = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        return []
    cell = float(grid['cell'])
    if weight <= 0.01:
        return [
            list(ring) for ring in grid.get('keepout_rings') or ()
            if ring and len(ring) >= 2]
    if weight >= 0.99:
        mud = grid.get('mud') or []
        mud_u = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
        segs = []
        for item in grid.get('coast_geoms') or ():
            line = _as_line(item)
            if line is None:
                continue
            parts = (
                [line] if line.geom_type == 'LineString'
                else _coast_line_parts(line))
            for part in parts:
                try:
                    if mud_u is not None and part.distance(mud_u) <= _ATTACH_M:
                        coords = [
                            xy for xy in (_xy_pair(c) for c in part.coords)
                            if xy is not None]
                        if len(coords) >= 2:
                            length = 0.0
                            for a_pt, b_pt in zip(coords, coords[1:]):
                                length += math.hypot(
                                    b_pt[0] - a_pt[0], b_pt[1] - a_pt[1])
                            if length >= 80.0:
                                segs.append(coords)
                except (TypeError, ValueError):
                    continue
        return segs
    ratio = grid['ratio']
    ok = grid['ok']
    mud_mask = grid['mud_mask']
    d_c = grid['d_c']
    d_k = grid['d_k']
    xmin = grid['xmin']
    ymin = grid['ymin']
    keep_u = grid.get('keep_u')
    coast_u = grid.get('coast_u')
    level = 1.0 - weight
    field = np.where(ok, ratio, 2.0)
    from scipy.ndimage import binary_erosion
    erode_n = max(2, int(round(8.0 / cell)))
    interior = binary_erosion(mud_mask, iterations=erode_n)
    sep = 2.0 * cell
    dc_fill = np.where(np.isfinite(d_c), d_c, 0.0)
    dk_fill = np.where(np.isfinite(d_k), d_k, 0.0)
    gcy, gcx = np.gradient(dc_fill)
    gky, gkx = np.gradient(dk_fill)
    nc = np.hypot(gcx, gcy) + 1.0e-9
    nk = np.hypot(gkx, gky) + 1.0e-9
    oppose = (gcx * gkx + gcy * gky) / (nc * nk) < -0.12
    between = np.logical_and(d_c > sep, d_k > sep)
    corridor = np.logical_and.reduce((ok, interior, between, oppose))
    field = np.where(corridor, ratio, 2.0)
    try:
        contours = find_contours(field, level)
    except (ValueError, TypeError):
        return []
    lines = []
    try:
        min_len = float(min_len_m)
    except (TypeError, ValueError):
        min_len = 40.0

    def _flush(seg):
        if len(seg) < 2:
            return
        coords = seg
        if ShapelyLineString is not None:
            try:
                geom = ShapelyLineString(seg).simplify(
                    cell * 1.5, preserve_topology=False)
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
        if length < min_len:
            return
        lines.append(coords)

    for contour in contours:
        current = []
        for row, col in contour:
            ri = int(round(row))
            ci = int(round(col))
            if (
                0 <= ri < corridor.shape[0]
                and 0 <= ci < corridor.shape[1]
                and corridor[ri, ci]
            ):
                x_val = xmin + (float(col) + 0.5) * cell
                y_val = ymin + (float(row) + 0.5) * cell
                current.append((x_val, y_val))
            else:
                _flush(current)
                current = []
        _flush(current)
    return _stitch_polylines(lines, join_m=max(80.0, 6.0 * cell))


def waterline_local_ratio_rings(
        keepout_rings, mud_polys, coast_geoms, alpha,
        cell_m=12.0, min_len_m=40.0):
    """해안↔기준면(keepout) 로컬 폭의 비율로 수위선.

    α=0 기준면, α=1 해안. 폭이 짧은 곳은 같은 Δα 에 미터 이동이 작다.
    """
    del cell_m
    return waterline_lerp_from_keepout(
        keepout_rings, coast_geoms, alpha, min_len_m=min_len_m,
        mud_polys=mud_polys)


def high_tide_coast_rings(keepout_rings, mud_polys, grow_m):
    """만조 해안. keepout 을 갯벌 끝까지 키운 구멍(섬)."""
    grown = grow_keepout_geom(keepout_rings, mud_polys, 1.0, grow_m)
    if grown is None:
        return []
    return waterline_front_rings(grown)


def _coast_components(coast_geoms):
    """해안을 LineString 조각으로."""
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


def _line_xy_list(line):
    pts = []
    try:
        coords = list(line.coords)
    except (TypeError, ValueError, AttributeError):
        return []
    for raw in coords:
        xy = _xy_pair(raw)
        if xy is not None:
            pts.append(xy)
    return pts if len(pts) >= 2 else []


def _pick_coast_component(keep_pt, components, prev_comp):
    """같은 해안 조각을 유지. 섬↔육지 점프를 막는다."""
    from shapely.ops import nearest_points as shapely_nearest_points
    best = None
    best_d = None
    for comp in components:
        try:
            dist = float(keep_pt.distance(comp))
        except (TypeError, ValueError):
            continue
        if best_d is None or dist < best_d:
            best_d = dist
            best = comp
    if best is None:
        return None, None
    chosen = best
    if prev_comp is not None:
        try:
            d_prev = float(keep_pt.distance(prev_comp))
        except (TypeError, ValueError):
            d_prev = None
        if d_prev is not None and (
                d_prev <= best_d * 1.35 or d_prev - best_d < 120.0):
            chosen = prev_comp
    try:
        coast_pt = shapely_nearest_points(keep_pt, chosen)[1]
    except (TypeError, ValueError, IndexError):
        return chosen, None
    return chosen, coast_pt


def _keepout_lines(keepout_rings):
    lines = []
    for ring in keepout_rings or ():
        if not ring or len(ring) < 2:
            continue
        coords = list(ring)
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        try:
            line = ShapelyLineString(coords)
        except (TypeError, ValueError):
            continue
        if line.length > 0.0:
            lines.append(line)
    return lines


def waterline_lerp_from_keepout(
        keepout_rings, coast_geoms, alpha,
        station_m=30.0, min_span_m=12.0, max_jump_m=1200.0,
        min_len_m=80.0, mud_polys=None):
    """기준면 안쪽 변을 가장 가까운 해안으로 α 보간.

    폭 W 가 작으면 같은 Δα 의 미터 이동이 작다.
    소시지 바깥 변은 안쪽 변이 가리면 버린다.
    """
    del mud_polys
    try:
        weight = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        return []
    station = _positive_m(station_m) or 30.0
    min_span = _positive_m(min_span_m) or 12.0
    try:
        max_jump = float(max_jump_m)
    except (TypeError, ValueError):
        max_jump = 1200.0
    if weight <= 0.01:
        return [
            list(ring) for ring in keepout_rings or ()
            if ring and len(ring) >= 2]
    components = [
        line for line in _coast_components(coast_geoms)
        if line.length >= 80.0]
    if not components or ShapelyLineString is None:
        return []
    if weight >= 0.99:
        out = []
        for line in components:
            pts = _line_xy_list(line)
            if pts:
                out.append(pts)
        return out
    keep_lines = []
    for line in _keepout_lines(keepout_rings):
        try:
            simple = line.simplify(20.0, preserve_topology=False)
        except (TypeError, ValueError):
            simple = line
        if simple is not None and not simple.is_empty and simple.length > 0.0:
            keep_lines.append(simple)
    if not keep_lines:
        return []
    try:
        from shapely.strtree import STRtree
        keep_tree = STRtree(keep_lines)
    except (TypeError, ValueError, ImportError):
        keep_tree = None
    segs = []
    for line in keep_lines:
        count = max(1, int(line.length / station))
        current = []
        prev = None
        prev_comp = None
        for index in range(count + 1):
            keep_pt = line.interpolate(index / count, normalized=True)
            keep_xy = (keep_pt.x, keep_pt.y)
            prev_comp, coast_pt = _pick_coast_component(
                keep_pt, components, prev_comp)
            if coast_pt is None:
                if len(current) >= 2:
                    segs.append(current)
                current = []
                prev = None
                continue
            coast_xy = (float(coast_pt.x), float(coast_pt.y))
            span = math.hypot(
                coast_xy[0] - keep_xy[0], coast_xy[1] - keep_xy[1])
            if span < min_span:
                if len(current) >= 2:
                    segs.append(current)
                current = []
                prev = None
                continue
            if keep_tree is not None and span > 30.0:
                ux = (coast_xy[0] - keep_xy[0]) / span
                uy = (coast_xy[1] - keep_xy[1]) / span
                inner = _first_keepout_along_ray(
                    keep_xy, (ux, uy), keep_tree, keep_lines,
                    15.0, span - 15.0)
                if inner is not None:
                    if len(current) >= 2:
                        segs.append(current)
                    current = []
                    prev = None
                    continue
            water = interpolate_waterline(keep_xy, coast_xy, weight)
            if water is None:
                continue
            if prev is not None:
                jump = math.hypot(water[0] - prev[0], water[1] - prev[1])
                if jump > max_jump:
                    if len(current) >= 2:
                        segs.append(current)
                    current = []
            current.append(water)
            prev = water
        if len(current) >= 2:
            segs.append(current)
    stitched = _stitch_polylines(segs, join_m=max(600.0, max_jump * 0.5))
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


def waterline_coast_offset(
        keepout_rings, mud_polys, alpha, grow_m,
        min_line_m=None, min_area_m2=None):
    """만조 해안 구멍(육지)을 갯벌 쪽으로 (1−α)·grow_m 민다.

    α=1 → 해안. α=0.5 → 해안과 평행한 갯벌 안 전선.
    폭이 밀기 거리보다 좁으면 갯벌 바깥(keepout)에서 막힌다.
    짧은 구멍은 min_line 으로 버리고 밀지 않는다.
    """
    grown_high = grow_keepout_geom(keepout_rings, mud_polys, 1.0, grow_m)
    if grown_high is None:
        return []
    coast_rings = filter_keepout_rings(
        waterline_front_rings(grown_high), min_line_m)
    try:
        weight = max(0.0, min(1.0, float(alpha)))
        reach = max(0.0, float(grow_m))
    except (TypeError, ValueError):
        return coast_rings
    offset = (1.0 - weight) * reach
    if offset < 1.0:
        return coast_rings
    land = rings_to_polygons(coast_rings)
    if not land or shapely_unary_union is None:
        return coast_rings
    land_u = land[0] if len(land) == 1 else shapely_unary_union(land)
    if land_u is None or getattr(land_u, 'is_empty', True):
        return coast_rings
    mud = []
    for item in mud_polys or ():
        valid = _valid_polygon(item)
        if valid is not None:
            mud.append(valid)
    if not mud:
        return coast_rings
    mud_u = mud[0] if len(mud) == 1 else shapely_unary_union(mud)
    if mud_u is None or getattr(mud_u, 'is_empty', True):
        return coast_rings
    try:
        wet = land_u.buffer(offset).intersection(mud_u).difference(land_u)
    except (TypeError, ValueError):
        return coast_rings
    if wet is None or getattr(wet, 'is_empty', True):
        return coast_rings
    try:
        area_floor = float(min_area_m2) if min_area_m2 is not None else (
            DEFAULT_CORRIDOR_MIN_AREA_M2)
    except (TypeError, ValueError):
        area_floor = DEFAULT_CORRIDOR_MIN_AREA_M2
    rings = _rings_all(wet, min_area=area_floor)
    return filter_keepout_rings(rings, min_line_m)


def _stitch_polylines(segs, join_m=160.0):
    """끝점이 가까운 조각을 이어 끊김을 줄인다."""
    try:
        join = float(join_m)
    except (TypeError, ValueError):
        join = 160.0
    unused = [list(seg) for seg in segs if seg and len(seg) >= 2]
    out = []

    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    while unused:
        acc = unused.pop(0)
        changed = True
        while changed:
            changed = False
            a0, a1 = acc[0], acc[-1]
            best_i = None
            best_d = join
            best_how = None
            for index, other in enumerate(unused):
                b0, b1 = other[0], other[-1]
                candidates = (
                    (_dist(a1, b0), 'tail_head'),
                    (_dist(a1, b1), 'tail_tail'),
                    (_dist(a0, b1), 'head_tail'),
                    (_dist(a0, b0), 'head_head'),
                )
                dist, how = min(candidates, key=lambda item: item[0])
                if dist <= best_d:
                    best_d = dist
                    best_i = index
                    best_how = how
            if best_i is None:
                break
            other = unused.pop(best_i)
            if best_how == 'tail_head':
                acc.extend(other)
            elif best_how == 'tail_tail':
                acc.extend(reversed(other))
            elif best_how == 'head_tail':
                acc = other + acc
            else:
                acc = list(reversed(other)) + acc
            changed = True
        out.append(acc)
    return out


def _iter_xy_points(geom):
    """교차 결과에서 점 좌표만 꺼낸다."""
    if geom is None or getattr(geom, 'is_empty', True):
        return
    geom_type = getattr(geom, 'geom_type', '')
    if geom_type == 'Point':
        yield (float(geom.x), float(geom.y))
        return
    if geom_type == 'LineString':
        for coord in geom.coords:
            xy = _xy_pair(coord)
            if xy is not None:
                yield xy
        return
    geoms = getattr(geom, 'geoms', None)
    if geoms is None:
        return
    for part in geoms:
        yield from _iter_xy_points(part)


def _mud_union(mud_polys):
    parts = []
    for poly in mud_polys or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            parts.append(valid)
    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]
    if shapely_unary_union is None:
        return parts[0]
    try:
        merged = shapely_unary_union(parts)
    except (TypeError, ValueError):
        return parts[0]
    return _valid_polygon(merged) or parts[0]


def _first_keepout_along_ray(
        origin_xy, direction_xy, keep_tree, keep_lines, min_span, max_span):
    """C 에서 한 방향으로 가서 처음 만나는 keepout 점."""
    origin = _xy_pair(origin_xy)
    hint = _unit(
        float(direction_xy[0]), float(direction_xy[1])) if direction_xy else None
    if origin is None or hint is None or ShapelyLineString is None:
        return None
    ox, oy = origin
    ux, uy = hint
    try:
        ray = ShapelyLineString([
            (ox, oy),
            (ox + ux * max_span, oy + uy * max_span),
        ])
    except (TypeError, ValueError):
        return None
    try:
        hits = resolve_strtree_query(keep_tree.query(ray), keep_lines)
    except (TypeError, ValueError, IndexError):
        return None
    best_t = None
    best_pt = None
    for line in hits:
        try:
            inter = ray.intersection(line)
        except (TypeError, ValueError):
            continue
        for px, py in _iter_xy_points(inter):
            t = math.hypot(px - ox, py - oy)
            if t < min_span or t > max_span:
                continue
            if best_t is None or t < best_t:
                best_t = t
                best_pt = (px, py)
    return best_pt


def _mud_normal_from_tree(origin_xy, tangent_xy, mud_tree, mud_list):
    """20 m 프로브가 맞는 법선. 전 갯벌 unary_union 은 쓰지 않는다."""
    origin = _xy_pair(origin_xy)
    tang = _unit(
        float(tangent_xy[0]), float(tangent_xy[1])) if tangent_xy else None
    if (
        origin is None or tang is None or mud_tree is None
        or not mud_list or ShapelyPoint is None
    ):
        return None
    nx, ny = -tang[1], tang[0]
    for sign in (1.0, -1.0):
        ux, uy = nx * sign, ny * sign
        probe = ShapelyPoint(
            origin[0] + ux * _PROBE_M, origin[1] + uy * _PROBE_M)
        try:
            hits = resolve_strtree_query(mud_tree.query(probe), mud_list)
        except (TypeError, ValueError, IndexError):
            hits = []
        for geom in hits:
            try:
                if geom.contains(probe) or geom.covers(probe):
                    return (ux, uy)
            except (TypeError, ValueError):
                continue
    return None


def _seaward_keepout_from_coast(
        origin_xy, tangent_xy, mud_tree, mud_list,
        keep_tree, keep_lines, min_span, max_span):
    """접선 법선 중 갯벌(해측) 쪽에서 keepout 을 고른다.

    유클리드 최근접은 섬 옆 keepout 을 집어 주황선이 해안에 붙는다.
    """
    origin = _xy_pair(origin_xy)
    tang = _unit(
        float(tangent_xy[0]), float(tangent_xy[1])) if tangent_xy else None
    if origin is None or tang is None:
        return None
    nx, ny = -tang[1], tang[0]
    mud_n = _mud_normal_from_tree(origin, tang, mud_tree, mud_list)
    if mud_n is not None:
        ordered = (mud_n, (-mud_n[0], -mud_n[1]))
    else:
        ordered = ((nx, ny), (-nx, -ny))
    hits = []
    for index, normal in enumerate(ordered):
        hit = _first_keepout_along_ray(
            origin, normal, keep_tree, keep_lines, min_span, max_span)
        if hit is None:
            continue
        hits.append(hit)
        if mud_n is not None and index == 0:
            return hit
    if not hits:
        return None
    return max(
        hits,
        key=lambda pt: math.hypot(pt[0] - origin[0], pt[1] - origin[1]))


def waterline_local_front(
        keepout_rings, coast_geoms, alpha,
        mud_polys=None,
        station_m=30.0, min_span_m=40.0, max_span_m=8000.0,
        max_jump_m=400.0, stitch_m=250.0):
    """해안을 따라 keepout↔C 를 로컬 보간. α=0 keepout, α=1 해안.

    각 해안 점에서 갯벌 쪽으로 광선을 쏴 keepout 과 교차시킨다.
    최근접 keepout 은 섬 옆 조각을 집어 중간 갯벌을 비운다.
    """
    try:
        weight = max(0.0, min(1.0, float(alpha)))
    except (TypeError, ValueError):
        return []
    station = _positive_m(station_m) or 30.0
    min_span = _positive_m(min_span_m) or 40.0
    max_span = _positive_m(max_span_m) or 8000.0
    try:
        max_jump = float(max_jump_m)
    except (TypeError, ValueError):
        max_jump = 400.0
    if ShapelyPoint is None or ShapelyLineString is None:
        return []
    keep_lines = []
    for ring in keepout_rings or ():
        if not ring or len(ring) < 2:
            continue
        coords = list(ring)
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        try:
            keep_lines.append(ShapelyLineString(coords))
        except (TypeError, ValueError):
            continue
    if not keep_lines:
        return []
    coast_parts = []
    for item in coast_geoms or ():
        line = _as_line(item)
        if line is None:
            continue
        if getattr(line, 'geom_type', '') == 'MultiLineString':
            coast_parts.extend(_coast_line_parts(line))
        elif not line.is_empty and line.length > 0.0:
            coast_parts.append(line)
    try:
        from shapely.strtree import STRtree
        keep_tree = STRtree(keep_lines)
        keep_u = (
            keep_lines[0] if len(keep_lines) == 1
            else shapely_unary_union(keep_lines))
    except (TypeError, ValueError, ImportError):
        return []
    if keep_u is None or getattr(keep_u, 'is_empty', True):
        return []
    mud_list = []
    for poly in mud_polys or ():
        valid = _valid_polygon(poly)
        if valid is not None:
            mud_list.append(valid)
    mud_tree = None
    if mud_list:
        try:
            mud_tree = STRtree(mud_list)
        except (TypeError, ValueError):
            mud_tree = None
    nearby_coast = []
    for line in coast_parts:
        try:
            if line.distance(keep_u) <= max_span:
                nearby_coast.append(line)
        except (TypeError, ValueError):
            continue
    if not nearby_coast:
        return []
    lines_out = []
    for line in nearby_coast:
        count = max(1, int(line.length / station))
        current = []
        prev = None
        for index in range(count + 1):
            coast_pt = line.interpolate(index / count, normalized=True)
            dist = min(line.length, max(0.0, line.project(coast_pt)))
            p0 = line.interpolate(max(0.0, dist - 1.0))
            p1 = line.interpolate(min(line.length, dist + 1.0))
            tang = _unit(p1.x - p0.x, p1.y - p0.y)
            keep_xy = _seaward_keepout_from_coast(
                (coast_pt.x, coast_pt.y), tang, mud_tree, mud_list,
                keep_tree, keep_lines, min_span, max_span)
            if keep_xy is None:
                if len(current) >= 2:
                    lines_out.append(current)
                current = []
                prev = None
                continue
            water = interpolate_waterline(
                keep_xy, (coast_pt.x, coast_pt.y), weight)
            if water is None:
                continue
            if prev is not None:
                jump = math.hypot(water[0] - prev[0], water[1] - prev[1])
                if jump > max_jump:
                    if len(current) >= 2:
                        lines_out.append(current)
                    current = []
            current.append(water)
            prev = water
        if len(current) >= 2:
            lines_out.append(current)
    stitched = _stitch_polylines(lines_out, join_m=stitch_m)
    return [seg for seg in stitched if len(seg) >= 2]


def grow_keepout_toward_coast(
        keepout_rings, mud_polys, alpha, grow_m):
    """구운 keepout 을 갯벌 안에서 해안 쪽으로 α 만큼 키운다.

    wet = K.buffer(α · grow_m) ∩ M.
    TideLayer 입력으로 쓰지 않는다. 수위선·접근 범위 시각화용.
    반환은 외곽 링(면적 판정용). 해안 전선은 waterline_front_rings.
    """
    grown = grow_keepout_geom(keepout_rings, mud_polys, alpha, grow_m)
    if grown is None:
        return []
    return _rings_all(grown)


def fill_triangles_from_rings(rings):
    """닫힌 링을 채울 삼각형 꼭짓점 목록. 3개씩 (x, y).

    Delaunay 뒤 중심이 면 안인 삼각형만 남긴다.
    """
    if shapely_triangulate is None or ShapelyPolygon is None:
        return []
    points = []
    for ring in rings or ():
        if not ring or len(ring) < 3:
            continue
        try:
            poly = ShapelyPolygon(ring)
        except (TypeError, ValueError):
            continue
        valid = _valid_polygon(poly)
        if valid is None:
            continue
        try:
            tol_m = 20.0 if getattr(valid, 'area', 0.0) > 400.0 else 2.0
            simplified = valid.simplify(tol_m, preserve_topology=True)
        except (TypeError, ValueError):
            simplified = valid
        valid = _valid_polygon(simplified) or valid
        parts = [valid]
        if valid.geom_type == 'MultiPolygon':
            parts = [
                part for part in valid.geoms
                if part.geom_type == 'Polygon' and part.area > 0.0]
        for part in parts:
            try:
                triangles = shapely_triangulate(part)
            except (TypeError, ValueError):
                continue
            for tri in triangles:
                if tri is None or tri.is_empty or getattr(tri, 'area', 0.0) <= 0.0:
                    continue
                try:
                    inside = part.covers(tri.centroid)
                except (TypeError, ValueError):
                    continue
                if not inside:
                    continue
                coords = list(tri.exterior.coords)[:3]
                if len(coords) < 3:
                    continue
                for x_val, y_val in coords:
                    points.append((float(x_val), float(y_val)))
    return points
