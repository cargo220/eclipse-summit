#!/usr/bin/env python3
"""keepout 을 갯벌 안으로 퍼뜨린 수위선을 α 스텝으로 구워 둔다.

새 핀/지형: progress/TIDE_NEW_SITE.md.
제부 기본 lerp --target holes 는 섬용. 육지 만은 keepout 씨앗이 수 km
남쪽으로 밀린다 → --mode width + 핀 창.
--install 은 named JSON만. launch는 waterline_tiles_dir 칸을 고르므로
GPS 칸에도 같은 파일을 넣는다. Occupancy 500 m 창과 마커 km는 별개.

실시간 buffer 가 아니라 파일을 시간에 맞춰 갈아끼운다.
해안 간격으로 짧은 갯벌을 자르지 않는다. 퍼지면 그 구간은 해안까지 찬다.

  python3 scripts/bake_tide_waterline.py --site jebu
  python3 scripts/bake_tide_waterline.py --site jebu --install
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src' / 'eclipse_pkg'
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

SITES = {
    'jebu': (37.1730, 126.6230),
    'gomso': (35.538593516, 126.551209635),
    'suncheon': (34.8550, 127.5000),
    'incheon': (37.4308576, 126.417128),
}


def _first_existing(paths):
    for item in paths:
        if item and os.path.isfile(item):
            return item
    return ''


def default_coast_path():
    return _first_existing((
        '/home/lee/2026 해안선/2026 해안선.shp',
        str(ROOT / 'datasets' / 'coastline' / '2026 해안선.shp'),
    ))


def default_mud_path(site=''):
    clips = {
        'jebu': ROOT / 'datasets' / 'tidflt' / 'tidflt_jebu_5186.shp',
        'gomso': ROOT / 'datasets' / 'tidflt' / 'tidflt_gomso_5186.shp',
    }
    key = str(site or '').strip().lower()
    if key in clips:
        found = _first_existing((str(clips[key]),))
        if found:
            return found
    return _first_existing((
        '/home/lee/WGIS_TIDFLT.shp',
        str(ROOT / 'datasets' / 'tidflt' / 'WGIS_TIDFLT.shp'),
        str(ROOT / 'datasets' / 'tidflt' / 'tidflt_jebu_5186.shp'),
        str(ROOT / 'datasets' / 'tidflt' / 'tidflt_gomso_5186.shp'),
    ))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='α 스텝 수위선 파일을 뽑는다.')
    parser.add_argument('--site', required=True)
    parser.add_argument(
        '--lat', type=float, default=None,
        help='width 모드 핀 위도. 없으면 SITES[site]')
    parser.add_argument(
        '--lon', type=float, default=None,
        help='width 모드 핀 경도. 없으면 SITES[site]')
    parser.add_argument('--keepout', default='',
                        help='구운 keepout geojson. 기본은 config keepout_<site>_perimeter.geojson')
    parser.add_argument('--mudflat', default='')
    parser.add_argument('--coast', default='')
    parser.add_argument('--step', type=float, default=0.05)
    parser.add_argument('--cell-m', type=float, default=12.0)
    parser.add_argument(
        '--alphas', default='',
        help='쉼표 α 목록. 비우면 --step 전체')
    parser.add_argument(
        '--target', choices=('holes', 'coast'), default='holes',
        help='만조 목표. holes=섬 구멍(제부), coast=2026 해안(육지 만)')
    parser.add_argument(
        '--from-coast', action='store_true',
        help='해안 점에서 갯벌 폭 비율. 기본은 keepout C lerp')
    parser.add_argument(
        '--mode', choices=('lerp', 'width'), default='lerp',
        help='lerp=keepout→C (기본 설치). width=여러 해안 갯벌 거리장')
    parser.add_argument(
        '--window-m', type=float, default=10000.0,
        help='width 모드 핀 창(m). 창 밖은 격자화하지 않음')
    parser.add_argument(
        '--coast-clear-m', type=float, default=40.0,
        help='width 모드: 이 거리 안 갯벌 둘레는 해안으로 보고 L에서 제외')
    parser.add_argument(
        '--min-closed-len-m', type=float, default=2000.0,
        help='width 모드: 이보다 짧은 닫힌 해안(작은 섬)은 C 씨앗에서 제외')
    parser.add_argument(
        '--min-closed-area-ha', type=float, default=5.0,
        help='width 모드: 이보다 작은 닫힌 섬(ha)은 C 씨앗에서 제외')
    parser.add_argument('--out-dir', default='')
    parser.add_argument('--install', action='store_true')
    return parser.parse_args(argv)


def _rings_from_line_geoms(geoms, keepout_rings=None, near_m=2500.0):
    """해안 LineString → (x,y) 링. keepout 근처만."""
    from shapely.ops import unary_union
    from eclipse_pkg.tide_waterline import rings_to_polygons

    keep_u = None
    if keepout_rings:
        polys = rings_to_polygons(keepout_rings)
        if polys:
            keep_u = polys[0] if len(polys) == 1 else unary_union(polys)
    rings = []
    for item in geoms or ():
        if item is None or getattr(item, 'is_empty', True):
            continue
        parts = []
        gtype = getattr(item, 'geom_type', '')
        if gtype == 'LineString':
            parts = [item]
        elif gtype == 'MultiLineString':
            parts = list(item.geoms)
        else:
            continue
        for part in parts:
            try:
                if part.length < 80.0:
                    continue
                if keep_u is not None and part.distance(keep_u) > near_m:
                    continue
            except (TypeError, ValueError):
                continue
            coords = [(float(x), float(y)) for x, y in part.coords]
            if len(coords) >= 2:
                rings.append(coords)
    return rings


def _write_line_geojson(path, rings, alpha):
    import json
    features = []
    for ring in rings or ():
        if not ring or len(ring) < 2:
            continue
        coords = [[float(pt[0]), float(pt[1])] for pt in ring]
        features.append({
            'type': 'Feature',
            'properties': {'alpha': float(alpha)},
            'geometry': {'type': 'LineString', 'coordinates': coords},
        })
    payload = {
        'type': 'FeatureCollection',
        'name': f'tars_waterline_a{float(alpha):.2f}',
        'crs': {
            'type': 'name',
            'properties': {'name': 'urn:ogc:def:crs:EPSG::5186'},
        },
        'features': features,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding='utf-8')
    return len(features)


def _write_wl_plot(
        out_png, origin_xy, keepout_rings, water_rings, title,
        coast_rings=None, mud_polys=None, coast_geoms=None, limits=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely.geometry import box as shapely_box

    px, py = origin_xy
    if limits is None:
        xlim = (px - 3200.0, px + 4500.0)
        ylim = (py - 3500.0, py + 2800.0)
    else:
        xlim = (float(limits[0]), float(limits[1]))
        ylim = (float(limits[2]), float(limits[3]))
    view = shapely_box(xlim[0], ylim[0], xlim[1], ylim[1])
    fig, ax = plt.subplots(figsize=(10.0, 8.4))
    for poly in mud_polys or ():
        try:
            hit = poly.intersection(view)
        except (TypeError, ValueError):
            continue
        if hit is None or hit.is_empty:
            continue
        parts = [hit] if hit.geom_type == 'Polygon' else list(
            getattr(hit, 'geoms', []) or [])
        for part in parts:
            if getattr(part, 'geom_type', '') != 'Polygon':
                continue
            xs, ys = part.exterior.xy
            ax.fill(xs, ys, color='#9ec5e8', alpha=0.40, linewidth=0)
            ax.plot(xs, ys, color='#3d7eea', lw=0.35, alpha=0.7)
    for line in coast_geoms or ():
        try:
            if not line.intersects(view):
                continue
            xs, ys = line.xy
        except (TypeError, ValueError, NotImplementedError):
            continue
        ax.plot(xs, ys, color='#444444', lw=0.7, alpha=0.85)
    for ring in keepout_rings or ():
        if not ring or len(ring) < 2:
            continue
        xs = [pt[0] for pt in ring]
        ys = [pt[1] for pt in ring]
        ax.plot(xs, ys, color='#2e8b57', lw=0.9, alpha=0.85)
    for ring in coast_rings or ():
        if not ring or len(ring) < 2:
            continue
        xs = [pt[0] for pt in ring]
        ys = [pt[1] for pt in ring]
        ax.plot(xs, ys, color='#666666', lw=0.7, ls='--', alpha=0.65)
    for ring in water_rings or ():
        if not ring or len(ring) < 2:
            continue
        xs = [pt[0] for pt in ring]
        ys = [pt[1] for pt in ring]
        ax.plot(xs, ys, color='#f06a12', lw=1.6, alpha=0.95)
    ax.plot(px, py, 'o', color='red', ms=6, label='pin')
    ax.set_aspect('equal')
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(title)
    ax.set_xlabel('EPSG:5186 E (m)')
    ax.set_ylabel('EPSG:5186 N (m)')
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def _write_index(out_dir, name, grow_m, alphas, mode='lerp', window_m=None):
    if mode == 'width':
        extra = (
            f'width 거리장. 창 {float(window_m or 0):.0f} m. '
            '파란=갯벌, 회색=해안, 주황=수위선. 설치하지 않음.')
    else:
        extra = (
            f'grow_m = {grow_m:.1f} m. 파란=keepout, 회색점선=만조 해안, 주황=수위선.\n'
            'keepout C lerp 또는 해안 로컬 폭 비율 (--from-coast).')
    lines = [
        f'# {name} 수위선 스텝 (α 간격 0.05)',
        '',
        extra,
        '',
        '| α | PNG | GeoJSON |',
        '|---:|---|---|',
    ]
    for alpha in alphas:
        tag = f'{alpha:.2f}'
        lines.append(
            f'| {tag} | [preview/a_{tag}.png](preview/a_{tag}.png) | '
            f'[geojson/a_{tag}.geojson](geojson/a_{tag}.geojson) |')
    lines.append('')
    (out_dir / 'README.md').write_text('\n'.join(lines), encoding='utf-8')


def _site_dir(name, out_dir=''):
    if out_dir:
        return Path(out_dir)
    return ROOT / 'progress' / 'waterline_maps' / name


def _install(name, out_dir=''):
    src = _site_dir(name, out_dir) / 'waterline_steps.json'
    if not src.is_file():
        raise SystemExit(f'맵이 없다: {src}')
    dest = (
        ROOT / 'src' / 'eclipse_pkg' / 'config' /
        f'waterline_{name}_steps.json')
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(src.read_bytes())
    print(f'install {src} -> {dest}')
    return 0


def _alpha_list(args):
    try:
        step = float(args.step)
    except (TypeError, ValueError):
        step = 0.05
    if step <= 0.0 or step > 1.0:
        step = 0.05
    raw_alphas = str(args.alphas or '').strip()
    if raw_alphas:
        alphas = []
        for token in raw_alphas.split(','):
            token = token.strip()
            if not token:
                continue
            try:
                alphas.append(round(float(token), 2))
            except (TypeError, ValueError):
                continue
        if not alphas:
            raise SystemExit('--alphas 를 해석할 수 없다')
        return alphas
    count = int(round(1.0 / step))
    alphas = [round(i * step, 2) for i in range(count + 1)]
    if alphas[-1] < 1.0:
        alphas.append(1.0)
    return alphas


def _bake_width(args, name, mud_path, coast_path):
    """여러 해안 거리장 수위선. keepout 을 씨앗으로 쓰지 않는다."""
    import pyproj
    from eclipse_pkg.tide_waterline import (
        dump_waterline_steps,
        load_line_geoms,
        load_mudflat_polygons,
        load_keepout_geojson,
        resolve_keepout_geojson_path,
        rings_to_polygons,
    )
    from eclipse_pkg.tide_waterline_width import waterline_width_steps

    key = name.lower()
    if args.lat is not None and args.lon is not None:
        lat, lon = float(args.lat), float(args.lon)
    elif key in SITES:
        lat, lon = SITES[key]
    else:
        raise SystemExit(
            'width 모드는 SITES 에 있는 --site 이거나 --lat --lon 이 필요하다')
    pin_xy = pyproj.Transformer.from_crs(
        'EPSG:4326', 'EPSG:5186', always_xy=True).transform(lon, lat)
    window_m = float(args.window_m)
    cell_m = float(args.cell_m)
    clear_m = float(args.coast_clear_m)
    min_closed_len = float(args.min_closed_len_m)
    min_closed_area = float(args.min_closed_area_ha) * 10000.0
    mud = load_mudflat_polygons(mud_path)
    coast = load_line_geoms(coast_path, origin_xy=pin_xy, radius_m=window_m)
    alphas = _alpha_list(args)
    print(
        f'width bake site={name} pin={pin_xy[0]:.0f},{pin_xy[1]:.0f} '
        f'window={window_m:.0f} cell={cell_m:.1f} clear={clear_m:.0f} '
        f'closed_min={min_closed_len:.0f}m/{float(args.min_closed_area_ha):.1f}ha '
        f'mud={len(mud)} coast={len(coast)} steps={alphas}')
    baked = waterline_width_steps(
        mud, coast, alphas,
        origin_xy=pin_xy, radius_m=window_m,
        cell_m=cell_m, coast_clear_m=clear_m,
        min_closed_len_m=min_closed_len,
        min_closed_area_m2=min_closed_area)
    if not baked:
        raise SystemExit('width 거리장 실패 (해안이 갯벌에 안 붙었거나 창이 큼)')
    for alpha, rings in baked:
        print(f'  α={alpha:.2f} rings={len(rings)}')

    out_dir = _site_dir(name, args.out_dir)
    if not args.out_dir:
        out_dir = ROOT / 'progress' / 'waterline_maps' / f'{name}_width'
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / 'waterline_steps.json'
    n_steps = dump_waterline_steps(
        str(out_path), baked,
        {
            'site': name,
            'mode': 'width',
            'mudflat': mud_path,
            'coast': coast_path,
            'window_m': window_m,
            'cell_m': cell_m,
            'coast_clear_m': clear_m,
            'min_closed_len_m': min_closed_len,
            'min_closed_area_m2': min_closed_area,
            'step': float(args.step),
            'baked_at': datetime.now(timezone.utc).strftime(
                '%Y-%m-%dT%H:%M:%SZ'),
        })
    print(f'wrote {n_steps} steps {out_path}')
    keepout_path = args.keepout or resolve_keepout_geojson_path(site=name)
    keepout = []
    if keepout_path and os.path.isfile(keepout_path):
        keepout = load_keepout_geojson(keepout_path)
        keep_polys = rings_to_polygons(keepout)
        keep_rings = []
        for poly in keep_polys:
            if poly is None or getattr(poly, 'is_empty', True):
                continue
            coords = [(float(x), float(y)) for x, y in poly.exterior.coords]
            if len(coords) >= 2:
                keep_rings.append(coords)
        keepout = keep_rings
    preview_dir = out_dir / 'preview'
    geo_dir = out_dir / 'geojson'
    for alpha, rings in baked:
        tag = f'{alpha:.2f}'
        _write_line_geojson(geo_dir / f'a_{tag}.geojson', rings, alpha)
        _write_wl_plot(
            preview_dir / f'a_{tag}.png',
            pin_xy, keepout, rings,
            f'{name}  width α={tag}  cell={cell_m:.0f}m  window={window_m:.0f}m',
            mud_polys=mud, coast_geoms=coast,
            limits=(
                pin_xy[0] - window_m, pin_xy[0] + window_m,
                pin_xy[1] - window_m, pin_xy[1] + window_m))
        print(f'  view α={tag}  png+geojson')
    _write_index(
        out_dir, name, 0.0, [row[0] for row in baked],
        mode='width', window_m=window_m)
    print(f'index {out_dir / "README.md"}')
    return 0


def main(argv=None):
    args = parse_args(argv)
    name = str(args.site or '').strip()
    if not name:
        raise SystemExit('--site 에 이름을 넣어라')
    if args.install:
        return _install(name, args.out_dir)

    from eclipse_pkg.tide_waterline import (
        clip_geoms_to_window,
        dump_waterline_steps,
        filter_keepout_rings,
        high_tide_coast_rings,
        keepout_grow_span_m,
        load_keepout_geojson,
        load_line_geoms,
        load_mudflat_polygons,
        resolve_keepout_geojson_path,
        rings_to_polygons,
        waterline_lerp_from_keepout,
        window_from_rings,
    )

    mud_path = args.mudflat or default_mud_path(name)
    coast_path = args.coast or default_coast_path()
    if not mud_path or not os.path.isfile(mud_path):
        raise SystemExit(f'갯벌 shapefile 없음: {mud_path}')
    if not coast_path or not os.path.isfile(coast_path):
        raise SystemExit(f'해안선 shapefile 없음: {coast_path}')

    if str(args.mode) == 'width':
        return _bake_width(args, name, mud_path, coast_path)

    keepout_path = args.keepout or resolve_keepout_geojson_path(site=name)
    if not keepout_path:
        raise SystemExit(f'keepout geojson 없음 (site={name})')

    keepout = filter_keepout_rings(load_keepout_geojson(keepout_path))
    mud = load_mudflat_polygons(mud_path)
    coast = load_line_geoms(coast_path)
    origin, radius = window_from_rings(keepout, 400.0)
    if origin is not None and radius is not None:
        mud = clip_geoms_to_window(mud, origin, radius)
        coast = clip_geoms_to_window(coast, origin, radius)
    key = name.lower()
    if key in SITES:
        import pyproj
        lat, lon = SITES[key]
        pin_xy = pyproj.Transformer.from_crs(
            'EPSG:4326', 'EPSG:5186', always_xy=True).transform(lon, lat)
        local_r = 8000.0
        mud = clip_geoms_to_window(mud, pin_xy, local_r)
        coast = clip_geoms_to_window(coast, pin_xy, local_r)
        keep_p = clip_geoms_to_window(
            rings_to_polygons(keepout), pin_xy, local_r)
        clipped_keep = []
        for poly in keep_p:
            if poly is None or getattr(poly, 'is_empty', True):
                continue
            parts = [poly] if poly.geom_type == 'Polygon' else list(
                getattr(poly, 'geoms', []) or [])
            for part in parts:
                if part.geom_type != 'Polygon' or part.area <= 0.0:
                    continue
                coords = [(float(x), float(y)) for x, y in part.exterior.coords]
                if len(coords) >= 4:
                    clipped_keep.append(coords)
        if clipped_keep:
            keepout = clipped_keep
        origin = pin_xy
        print(f'local window 8 km around pin {pin_xy[0]:.0f},{pin_xy[1]:.0f} '
              f'mud={len(mud)} coast={len(coast)} keepout={len(keepout)}')
    grow_m = keepout_grow_span_m(keepout, coast, fallback_m=400.0)
    target = str(args.target or 'holes').strip().lower()
    if target == 'coast':
        island = _rings_from_line_geoms(coast, keepout, near_m=2500.0)
        print(f'target=coast nearby={len(island)} grow_m={grow_m:.1f}')
    else:
        island = filter_keepout_rings(
            high_tide_coast_rings(keepout, mud, grow_m), 800.0)
        print(f'target=holes island={len(island)} grow_m={grow_m:.1f}')
    try:
        step = float(args.step)
    except (TypeError, ValueError):
        step = 0.05
    if step <= 0.0 or step > 1.0:
        step = 0.05
    raw_alphas = str(args.alphas or '').strip()
    if raw_alphas:
        alphas = []
        for token in raw_alphas.split(','):
            token = token.strip()
            if not token:
                continue
            try:
                alphas.append(round(float(token), 2))
            except (TypeError, ValueError):
                continue
        if not alphas:
            raise SystemExit('--alphas 를 해석할 수 없다')
    else:
        count = int(round(1.0 / step))
        alphas = [round(i * step, 2) for i in range(count + 1)]
        if alphas[-1] < 1.0:
            alphas.append(1.0)
    from_coast = bool(args.from_coast)
    mode = 'coast-width' if from_coast else f'keepout→{target} lerp'
    print(
        f'site={name} keepout={len(keepout)} mud={len(mud)} '
        f'grow_m={grow_m:.1f} steps={alphas} ({mode})')
    baked = []
    for alpha in alphas:
        if from_coast:
            from eclipse_pkg.tide_waterline_island import (
                waterline_from_coast_width)
            if alpha >= 0.99:
                rings = [
                    list(ring) for ring in island if ring and len(ring) >= 2]
            else:
                rings = waterline_from_coast_width(
                    island, mud, alpha, pin_xy=origin)
        elif alpha <= 0.01:
            rings = [list(ring) for ring in keepout if ring and len(ring) >= 2]
        elif alpha >= 0.99:
            rings = [list(ring) for ring in island if ring and len(ring) >= 2]
        else:
            rings = waterline_lerp_from_keepout(
                keepout, island, alpha, mud_polys=mud)
        baked.append((alpha, rings))
        print(f'  α={alpha:.2f} rings={len(rings)}')

    out_dir = _site_dir(name, args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / 'waterline_steps.json'
    n_steps = dump_waterline_steps(
        str(out_path), baked,
        {
            'site': name,
            'keepout': keepout_path,
            'mudflat': mud_path,
            'coast': coast_path,
            'grow_m': grow_m,
            'step': step,
            'from_coast': from_coast,
            'baked_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        })
    print(f'wrote {n_steps} steps {out_path}')
    preview_dir = out_dir / 'preview'
    geo_dir = out_dir / 'geojson'
    origin, _radius = window_from_rings(keepout, 400.0)
    key = name.lower()
    if key in SITES:
        import pyproj
        lonlat = SITES[key]
        origin = pyproj.Transformer.from_crs(
            'EPSG:4326', 'EPSG:5186', always_xy=True).transform(
                lonlat[1], lonlat[0])
    if origin is None:
        origin = (0.0, 0.0)
    for alpha, rings in baked:
        tag = f'{alpha:.2f}'
        _write_line_geojson(geo_dir / f'a_{tag}.geojson', rings, alpha)
        _write_wl_plot(
            preview_dir / f'a_{tag}.png',
            origin, keepout, rings,
            f'{name}  waterline α={tag}  grow_m={grow_m:.0f} m',
            coast_rings=island)
        print(f'  view α={tag}  png+geojson')
    _write_index(out_dir, name, grow_m, [row[0] for row in baked])
    print(f'index {out_dir / "README.md"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
