#!/usr/bin/env python3
"""위치만 주면 그 자리 갯벌 둘레 keepout 맵을 뽑는다.

새 핀/지형: progress/TIDE_NEW_SITE.md.
GPS는 이 파일을 고르지 않는다. start <site> 와 --install 이 짝.
갯벌 shp는 전국본. 사이트 클립이 없으면 제부 클립으로 떨어지니 다른
지형에서 면이 비면 그걸 의심한다.

제부도에서 승인한 규칙과 같다.
  1) 창과 겹치는 2011 갯벌 면을 통째로 모은다 (창으로 자르지 않음)
  2) union 해서 맞닿은 변을 없앤다
  3) 2026 해안선 30 m 안 변은 연다
  4) 남은 바깥 변을 갯벌 안으로 밀어 띠로 만든다

창으로 면을 자르면 창 테두리가 가짜 바깥 변이 된다. 그래서
반경은 '어느 면을 고를지'만 정하고, 고른 면은 자르지 않는다.

두 단계다. 1) 맵을 뽑는다  2) 마음에 들면 install 한다.
install 은 이미 뽑은 폴더를 복사만 한다. 좌표를 다시 넣지 않는다.

  # 1. 맵 뽑기 → progress/keepout_maps/<이름>/
  python3 scripts/bake_tide_keepout.py --site jebu
  python3 scripts/bake_tide_keepout.py --site 제부도서쪽 --lat 37.173 --lon 126.623

  # 2. 설치. 좌표 없음
  python3 scripts/bake_tide_keepout.py --site jebu --install
  python3 scripts/bake_tide_keepout.py --site 제부도서쪽 --install

jebu / gomso 만 좌표를 생략할 수 있다. 다른 이름은 뽑을 때 한 번만 --lat --lon.
"""

from __future__ import annotations

import argparse
import json
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
        '/workspaces/eclipse-test-2/datasets/coastline/2026 해안선.shp',
    ))


def default_mud_path(site=''):
    nationwide = _first_existing((
        '/home/lee/WGIS_TIDFLT.shp',
        str(ROOT / 'datasets' / 'tidflt' / 'WGIS_TIDFLT.shp'),
    ))
    if nationwide:
        return nationwide
    clips = {
        'jebu': ROOT / 'datasets' / 'tidflt' / 'tidflt_jebu_5186.shp',
        'gomso': ROOT / 'datasets' / 'tidflt' / 'tidflt_gomso_5186.shp',
    }
    key = str(site or '').strip().lower()
    if key in clips:
        return _first_existing((str(clips[key]),))
    return _first_existing((
        str(ROOT / 'datasets' / 'tidflt' / 'tidflt_jebu_5186.shp'),
        str(ROOT / 'datasets' / 'tidflt' / 'tidflt_gomso_5186.shp'),
    ))
UTMK = (
    '+proj=tmerc +lat_0=38 +lon_0=127.5 +k=0.9996 '
    '+x_0=1000000 +y_0=2000000 +ellps=WGS84 +units=m +no_defs'
)


def _to_5186():
    import pyproj
    return pyproj.Transformer.from_crs(
        'EPSG:4326', 'EPSG:5186', always_xy=True)


def _to_ll():
    import pyproj
    return pyproj.Transformer.from_crs(
        'EPSG:5186', 'EPSG:4326', always_xy=True)


def _prj_is_5186(path):
    prj = Path(path).with_suffix('.prj')
    if not prj.is_file():
        return False
    text = prj.read_text(encoding='utf-8', errors='replace')
    return 'Central_Belt_2010' in text or '5186' in text


def _iter_shapefile_geoms(path, bbox=None):
    """fiona 가 있으면 bbox 필터, 없으면 pyshp 로 전부 읽는다."""
    from shapely.geometry import shape
    try:
        import fiona
    except ImportError:
        fiona = None
    if fiona is not None:
        with fiona.open(path) as src:
            try:
                recs = src.filter(bbox=bbox) if bbox is not None else src
            except Exception:
                recs = src
            for rec in recs:
                geom = rec.get('geometry')
                if geom:
                    yield shape(geom)
        return
    import shapefile as pyshp
    reader = pyshp.Reader(path)
    try:
        for shp in reader.shapes():
            try:
                yield shape(shp.__geo_interface__)
            except (TypeError, ValueError, AttributeError):
                continue
    finally:
        reader.close()


def _load_mud_faces(path, origin_xy, radius_m):
    """창과 겹치는 면을 통째로. 창으로 자르지 않는다."""
    import pyproj
    from shapely.geometry import box as Box
    from shapely.ops import transform as shp_transform

    from eclipse_pkg.tide_waterline import _valid_polygon

    ox, oy = origin_xy
    if _prj_is_5186(path):
        win = Box(ox - radius_m, oy - radius_m, ox + radius_m, oy + radius_m)
        project = None
    else:
        to_utmk = pyproj.Transformer.from_crs(
            'EPSG:4326', UTMK, always_xy=True)
        to_5186 = pyproj.Transformer.from_crs(
            UTMK, 'EPSG:5186', always_xy=True)
        # origin_xy 는 이미 5186. 창을 UTM-K 로 옮겨 필터한다.
        to_ll = pyproj.Transformer.from_crs(
            'EPSG:5186', 'EPSG:4326', always_xy=True)
        lon, lat = to_ll.transform(ox, oy)
        ux, uy = to_utmk.transform(lon, lat)
        win = Box(ux - radius_m, uy - radius_m, ux + radius_m, uy + radius_m)

        def project(x_val, y_val, z_val=None):
            east, north = to_5186.transform(x_val, y_val)
            if z_val is None:
                return (east, north)
            return (east, north, z_val)

    faces = []
    for g in _iter_shapefile_geoms(path, bbox=win.bounds):
        if g is None or g.is_empty or not g.intersects(win):
            continue
        if project is not None:
            try:
                g = shp_transform(project, g)
            except (TypeError, ValueError):
                continue
        valid = _valid_polygon(g)
        if valid is not None:
            faces.append(valid)
        elif g.geom_type == 'MultiPolygon':
            for part in g.geoms:
                valid = _valid_polygon(part)
                if valid is not None:
                    faces.append(valid)
    return faces


def _load_coast(path, origin_xy, radius_m):
    from eclipse_pkg.tide_waterline import load_line_geoms
    return load_line_geoms(path, origin_xy=origin_xy, radius_m=radius_m)


def _suggested_pin(rings, origin_xy):
    from shapely.geometry import Point, Polygon
    pin = Point(origin_xy[0], origin_xy[1])
    best = None
    best_d = 1e18
    for ring in rings:
        poly = Polygon(ring)
        if not poly.is_valid or poly.is_empty:
            continue
        dist = poly.distance(pin)
        if dist < best_d:
            best_d = dist
            best = poly.centroid
    if best is None:
        return None
    lon, lat = _to_ll().transform(best.x, best.y)
    return {
        'lat': round(lat, 7),
        'lon': round(lon, 7),
        'dist_m': round(best_d, 1),
    }


def _write_plot(out_png, origin_xy, faces, coast, rings, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    import numpy as np

    from shapely.geometry import Point

    px, py = origin_xy
    pin = Point(px, py)
    fig, ax = plt.subplots(figsize=(9.0, 7.8))
    for geom in faces:
        if geom.distance(pin) > 6000.0:
            continue
        if geom.geom_type == 'Polygon':
            xs, ys = geom.exterior.xy
            ax.fill(xs, ys, color='#e6d3b3', alpha=0.55)
    segs = []
    for line in coast:
        if line.geom_type == 'LineString':
            segs.append(np.asarray(line.coords))
        elif line.geom_type == 'MultiLineString':
            for part in line.geoms:
                segs.append(np.asarray(part.coords))
    if segs:
        ax.add_collection(LineCollection(segs, colors='k', linewidths=0.7))
    for ring in rings:
        xs = [pt[0] for pt in ring] + [ring[0][0]]
        ys = [pt[1] for pt in ring] + [ring[0][1]]
        ax.fill(xs, ys, color='tab:red', alpha=0.40)
    ax.plot(px, py, 'o', color='red', ms=7, label='pin')
    ax.set_aspect('equal')
    ax.set_xlim(px - 2800, px + 2800)
    ax.set_ylim(py - 2400, py + 2000)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(title)
    ax.set_xlabel('EPSG:5186 E (m)')
    ax.set_ylabel('EPSG:5186 N (m)')
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='위치의 갯벌 둘레 keepout 맵을 뽑는다.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument(
        '--site', required=True,
        help='맵 이름. 폴더명으로 쓴다. jebu/gomso 는 좌표를 생략할 수 있다')
    parser.add_argument('--lat', type=float, help='위도 WGS84')
    parser.add_argument('--lon', type=float, help='경도 WGS84')
    parser.add_argument('--radius-km', type=float, default=25.0,
                        help='이 반경 안과 겹치는 면을 고른다. 면은 자르지 않음')
    parser.add_argument('--coast-clear-m', type=float, default=30.0)
    parser.add_argument('--inset-ratio', type=float, default=0.10)
    parser.add_argument(
        '--min-line-m', type=float, default=550.0,
        help='이보다 짧은 변은 점으로 남으니 버린다')
    parser.add_argument('--mudflat', default='',
                        help='2011 갯벌 shapefile. 기본은 전국 WGIS, 없으면 사이트 클립')
    parser.add_argument('--coast', default='',
                        help='2026 해안선 shapefile')
    parser.add_argument('--out-dir', default='',
                        help='기본 progress/keepout_maps/<name>')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument(
        '--install', action='store_true',
        help='이미 뽑은 keepout_maps/<site> 를 config 로 복사. 좌표 불필요')
    return parser.parse_args(argv)


def _site_dir(name, out_dir=''):
    if out_dir:
        return Path(out_dir)
    return ROOT / 'progress' / 'keepout_maps' / name


def _install_from_maps(name, out_dir=''):
    src = _site_dir(name, out_dir) / 'keepout.geojson'
    if not src.is_file():
        raise SystemExit(
            f'맵이 없다: {src}\n'
            f'먼저 뽑아라: python3 scripts/bake_tide_keepout.py --site {name} ...')
    dest = (
        ROOT / 'src' / 'eclipse_pkg' / 'config' /
        f'keepout_{name}_perimeter.geojson')
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(src.read_bytes())
    print(f'install {src} -> {dest}')
    print(
        'launch keepout_geojson 를 아래 경로로 바꾸면 된다:\n'
        f'  /workspaces/eclipse-test-2/src/eclipse_pkg/config/'
        f'keepout_{name}_perimeter.geojson')
    return 0


def main(argv=None):
    args = parse_args(argv)
    name = str(args.site or '').strip()
    if not name:
        raise SystemExit('--site 에 이름을 넣어라')
    if args.install and args.lat is None and args.lon is None:
        return _install_from_maps(name, args.out_dir)
    key = name.lower()
    if args.lat is not None and args.lon is not None:
        lat, lon = float(args.lat), float(args.lon)
    elif key in SITES:
        lat, lon = SITES[key]
    else:
        raise SystemExit(
            '맵을 처음 뽑을 때만 좌표가 필요하다.\n'
            f'  python3 scripts/bake_tide_keepout.py --site {name} '
            '--lat <위도> --lon <경도>\n'
            '이미 뽑았으면 설치만:\n'
            f'  python3 scripts/bake_tide_keepout.py --site {name} --install')
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise SystemExit('위경도 범위가 아니다')
    mud_path = args.mudflat or default_mud_path(name)
    coast_path = args.coast or default_coast_path()
    if not mud_path or not os.path.isfile(mud_path):
        raise SystemExit(
            f'갯벌 shapefile 없음: {mud_path or "(기본 경로 전부 없음)"}\n'
            '--mudflat 로 지정하거나 개발 PC의 WGIS_TIDFLT.shp 를 써라')
    if not coast_path or not os.path.isfile(coast_path):
        raise SystemExit(f'해안선 shapefile 없음: {coast_path or "(기본 경로 전부 없음)"}')

    from eclipse_pkg.tide_waterline import (
        dump_keepout_geojson,
        mudflat_perimeter_keepout,
    )

    px, py = _to_5186().transform(lon, lat)
    radius_m = float(args.radius_km) * 1000.0
    print(f'pin {lat:.7f}, {lon:.7f}  5186 {px:.1f} {py:.1f}')
    print(f'select radius {radius_m:.0f} m  mud={mud_path}')
    faces = _load_mud_faces(mud_path, (px, py), radius_m)
    print(f'mud faces {len(faces)} (whole faces, not clipped)')
    if not faces:
        raise SystemExit('그 위치 반경 안에 갯벌 면이 없다')
    coast = _load_coast(coast_path, (px, py), radius_m + 2000.0)
    print(f'coast parts {len(coast)}')
    rings = mudflat_perimeter_keepout(
        faces, coast,
        coast_clear_m=args.coast_clear_m,
        inset_ratio=args.inset_ratio,
        min_line_m=args.min_line_m,
        simplify_m=12.0,
        min_area_m2=100.0,
    )
    area = 0.0
    from shapely.geometry import Polygon
    for ring in rings:
        poly = Polygon(ring)
        if poly.is_valid and not poly.is_empty:
            area += poly.area
    print(f'rings {len(rings)}  area {area/1e4:.1f} ha')

    out_dir = Path(args.out_dir) if args.out_dir else (
        ROOT / 'progress' / 'keepout_maps' / name)
    out_dir.mkdir(parents=True, exist_ok=True)
    geojson_path = out_dir / 'keepout.geojson'
    png_path = out_dir / 'preview.png'
    meta_path = out_dir / 'meta.json'
    count = dump_keepout_geojson(
        str(geojson_path), rings,
        {
            'rule': 'mudflat_perimeter',
            'coast_clear_m': args.coast_clear_m,
            'inset_ratio': args.inset_ratio,
            'site': name,
            'lat': lat,
            'lon': lon,
        })
    print(f'wrote {count} features {geojson_path}')
    if not args.no_plot:
        title = f'{name}  {lat:.5f}, {lon:.5f}  union perimeter'
        _write_plot(png_path, (px, py), faces, coast, rings, title)
        print(f'wrote {png_path}')
    pin = _suggested_pin(rings, (px, py))
    meta = {
        'name': name,
        'lat': lat,
        'lon': lon,
        'radius_km': args.radius_km,
        'mudflat': mud_path,
        'coast': coast_path,
        'faces': len(faces),
        'rings': len(rings),
        'area_ha': round(area / 1e4, 2),
        'coast_clear_m': args.coast_clear_m,
        'inset_ratio': args.inset_ratio,
        'geojson': str(geojson_path),
        'preview': str(png_path) if not args.no_plot else '',
        'suggested_pin': pin,
        'baked_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'launch_hint': (
            'keepout_geojson: '
            f'/workspaces/eclipse-test-2/src/eclipse_pkg/config/'
            f'keepout_{name}_perimeter.geojson'
        ),
    }
    meta_path.write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(f'wrote {meta_path}')
    if pin:
        print(
            f'suggested pin {pin["lat"]}, {pin["lon"]} '
            f'({pin["dist_m"]} m from query)')
    if args.install:
        return _install_from_maps(name, args.out_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
