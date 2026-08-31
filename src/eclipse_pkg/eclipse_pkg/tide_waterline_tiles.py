"""제부 창 크기의 수위선 타일. 전국 한 장이 아니라 폴더 여러 개.

격자 중심은 step 간격, 창은 window 반경. 창이 step 보다 크면 겹친다.
런타임은 GPS→5186 후 그 점을 품은 타일 하나만 고른다.
"""

from __future__ import annotations

import math

DEFAULT_WINDOW_M = 9000.0
DEFAULT_STEP_M = 12000.0
DEFAULT_MIN_MUD_HA = 50.0


def tile_id(cx, cy):
    """타일 폴더 이름. km 단위 동쪽·북쪽."""
    return f'e{int(round(float(cx) / 1000.0)):04d}_n{int(round(float(cy) / 1000.0)):04d}'


def tile_index_xy(x_val, y_val, step_m=DEFAULT_STEP_M):
    """점이 들어가는 격자 인덱스 (ix, iy)."""
    step = float(step_m)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError('step_m 은 양수여야 한다')
    return (int(math.floor(float(x_val) / step)),
            int(math.floor(float(y_val) / step)))


def tile_center(ix, iy, step_m=DEFAULT_STEP_M):
    """격자 칸 중심 EPSG:5186 (x, y)."""
    step = float(step_m)
    return ((float(ix) + 0.5) * step, (float(iy) + 0.5) * step)


def tile_record(ix, iy, window_m=DEFAULT_WINDOW_M, step_m=DEFAULT_STEP_M,
                mud_ha=0.0, lat=None, lon=None):
    cx, cy = tile_center(ix, iy, step_m)
    radius = float(window_m)
    rec = {
        'id': tile_id(cx, cy),
        'ix': int(ix),
        'iy': int(iy),
        'cx': float(cx),
        'cy': float(cy),
        'window_m': radius,
        'step_m': float(step_m),
        'minx': float(cx) - radius,
        'miny': float(cy) - radius,
        'maxx': float(cx) + radius,
        'maxy': float(cy) + radius,
        'mud_ha': float(mud_ha),
    }
    if lat is not None and lon is not None:
        rec['lat'] = float(lat)
        rec['lon'] = float(lon)
    return rec


def point_in_tile(x_val, y_val, tile):
    """점이 타일 창 안에 있으면 True."""
    if not tile:
        return False
    try:
        return (
            float(tile['minx']) <= float(x_val) <= float(tile['maxx'])
            and float(tile['miny']) <= float(y_val) <= float(tile['maxy']))
    except (TypeError, ValueError, KeyError):
        return False


def lookup_tile(tiles, x_val, y_val):
    """점을 품은 타일 중 중심에 가장 가까운 것. 없으면 None."""
    hits = [tile for tile in tiles or () if point_in_tile(x_val, y_val, tile)]
    if not hits:
        return None
    best = None
    best_d = None
    for tile in hits:
        try:
            dist = math.hypot(
                float(x_val) - float(tile['cx']),
                float(y_val) - float(tile['cy']))
        except (TypeError, ValueError, KeyError):
            continue
        if best_d is None or dist < best_d:
            best = tile
            best_d = dist
    return best


def lookup_tile_sticky(tiles, x_val, y_val, current_id=None):
    """현재 칸 창 안에 있으면 유지. 밖으로 나가면 lookup_tile."""
    if current_id:
        for tile in tiles or ():
            if tile.get('id') == current_id and point_in_tile(x_val, y_val, tile):
                return tile
    return lookup_tile(tiles, x_val, y_val)


def load_tile_index(path):
    """index.json 의 tiles 목록. 없거나 깨지면 예외."""
    import json
    import os
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f'수위선 타일 인덱스가 없다: {path}')
    with open(path, encoding='utf-8') as handle:
        data = json.load(handle)
    if isinstance(data, dict):
        raw = data.get('tiles') or []
    elif isinstance(data, list):
        raw = data
    else:
        raw = []
    return [item for item in raw if isinstance(item, dict) and item.get('id')]


def _tile_file_path(tiles_dir, tile_id_val, filename):
    """타일 폴더 안 파일. 이상한 id 는 빈 문자열."""
    import os
    name = str(tile_id_val or '')
    root = str(tiles_dir or '')
    if not root or not name or '/' in name or '\\' in name or name in ('.', '..'):
        return ''
    return os.path.join(root, name, filename)


def tile_steps_path(tiles_dir, tile_id_val):
    """타일 폴더의 waterline_steps.json. 이상한 id 는 빈 문자열."""
    return _tile_file_path(tiles_dir, tile_id_val, 'waterline_steps.json')


def tile_keepout_path(tiles_dir, tile_id_val):
    """타일 폴더의 keepout.geojson. 이상한 id 는 빈 문자열."""
    return _tile_file_path(tiles_dir, tile_id_val, 'keepout.geojson')


def baked_tiles(tiles, tiles_dir):
    """steps JSON 이 있는 타일만."""
    import os
    out = []
    for tile in tiles or ():
        path = tile_steps_path(tiles_dir, tile.get('id'))
        if path and os.path.isfile(path):
            out.append(tile)
    return out


def baked_keepout_tiles(tiles, tiles_dir):
    """keepout.geojson 이 있는 타일만."""
    import os
    out = []
    for tile in tiles or ():
        path = tile_keepout_path(tiles_dir, tile.get('id'))
        if path and os.path.isfile(path):
            out.append(tile)
    return out


def neighbor_indices(ix, iy, ring=1):
    """중심 칸 포함, ring 칸까지 (2*ring+1)^2 인덱스."""
    span = int(ring)
    if span < 0:
        span = 0
    out = []
    for dx in range(-span, span + 1):
        for dy in range(-span, span + 1):
            out.append((int(ix) + dx, int(iy) + dy))
    return out


def list_mud_tiles(mud_polys, window_m=DEFAULT_WINDOW_M, step_m=DEFAULT_STEP_M,
                   min_mud_ha=DEFAULT_MIN_MUD_HA):
    """갯벌과 겹치는 타일 목록. 면적이 min_mud_ha 미만이면 버린다."""
    from shapely.geometry import box as shapely_box

    radius = float(window_m)
    step = float(step_m)
    min_area = float(min_mud_ha) * 10000.0
    polys = [p for p in mud_polys or () if p is not None and not p.is_empty]
    if not polys or radius <= 0.0 or step <= 0.0:
        return []
    minx = min(p.bounds[0] for p in polys)
    miny = min(p.bounds[1] for p in polys)
    maxx = max(p.bounds[2] for p in polys)
    maxy = max(p.bounds[3] for p in polys)
    ix0, iy0 = tile_index_xy(minx, miny, step)
    ix1, iy1 = tile_index_xy(maxx, maxy, step)
    tiles = []
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            cx, cy = tile_center(ix, iy, step)
            win = shapely_box(cx - radius, cy - radius, cx + radius, cy + radius)
            area = 0.0
            for poly in polys:
                pb = poly.bounds
                if (pb[2] < cx - radius or pb[0] > cx + radius
                        or pb[3] < cy - radius or pb[1] > cy + radius):
                    continue
                try:
                    hit = poly.intersection(win)
                except (TypeError, ValueError):
                    continue
                if hit is not None and not hit.is_empty:
                    area += hit.area
            if area < min_area:
                continue
            tiles.append(tile_record(
                ix, iy, window_m=radius, step_m=step, mud_ha=area / 10000.0))
    tiles.sort(key=lambda item: (item['iy'], item['ix']))
    return tiles
