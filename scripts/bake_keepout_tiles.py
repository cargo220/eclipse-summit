#!/usr/bin/env python3
"""전국 갯벌을 수위선과 같은 12 km 격자로 나눠 keepout 을 굽는다.

Xavier 는 인덱스 + 현재 GPS 칸 1장만 읽는다. OccupancyGrid 를
키우지 않는다.

  # 칸 목록만 (수위선 index 가 있으면 재사용)
  python3 scripts/bake_keepout_tiles.py --list

  # 갯벌 있는 칸 전부. 칸 PNG 없음
  python3 scripts/bake_keepout_tiles.py --all

  # 한 점 주변 3x3
  python3 scripts/bake_keepout_tiles.py --around-lat 37.173 --around-lon 126.623 --ring 1
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
if str(ROOT / 'scripts') not in sys.path:
    sys.path.insert(0, str(ROOT / 'scripts'))

from bake_tide_keepout import (  # noqa: E402
    _load_coast,
    default_coast_path,
    default_mud_path,
)
from bake_waterline_tiles import (  # noqa: E402
    _attach_lonlat,
    _plot_index,
    _select_tiles,
    _write_index_json,
    build_index,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='keepout 타일 인덱스·굽기')
    parser.add_argument('--mudflat', default='')
    parser.add_argument('--coast', default='')
    parser.add_argument('--out-dir', default='')
    parser.add_argument('--from-index', default='',
                        help='수위선 index.json. 있으면 같은 칸 id 를 쓴다')
    parser.add_argument('--window-m', type=float, default=9000.0)
    parser.add_argument('--step-m', type=float, default=12000.0)
    parser.add_argument('--min-mud-ha', type=float, default=50.0)
    parser.add_argument('--coast-clear-m', type=float, default=30.0)
    parser.add_argument('--inset-ratio', type=float, default=0.10)
    parser.add_argument('--min-line-m', type=float, default=550.0)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--all', action='store_true',
                        help='갯벌 칸 전부 굽기')
    parser.add_argument('--around-lat', type=float, default=None)
    parser.add_argument('--around-lon', type=float, default=None)
    parser.add_argument('--ring', type=int, default=1)
    parser.add_argument('--ids', default='')
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--no-plot', action='store_true')
    return parser.parse_args(argv)


def _out_root(args):
    if args.out_dir:
        return Path(args.out_dir)
    return ROOT / 'progress' / 'keepout_tiles'


def _default_index_path():
    return ROOT / 'progress' / 'waterline_tiles' / 'index.json'


def _load_index_tiles(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if isinstance(data, dict):
        raw = data.get('tiles') or []
    elif isinstance(data, list):
        raw = data
    else:
        raw = []
    return [item for item in raw if isinstance(item, dict) and item.get('id')]


def _bake_one(tile, mud, tree, coast_path, args, out_dir):
    from shapely.geometry import box as shapely_box

    from eclipse_pkg.tide_plan import resolve_strtree_query
    from eclipse_pkg.tide_waterline import (
        dump_keepout_geojson,
        mudflat_perimeter_keepout,
    )

    pin = (float(tile['cx']), float(tile['cy']))
    window_m = float(args.window_m)
    dest = Path(out_dir) / 'keepout.geojson'
    if dest.is_file() and not args.force:
        print(f'  skip {tile["id"]}: exists', flush=True)
        return True
    win = shapely_box(
        pin[0] - window_m, pin[1] - window_m,
        pin[0] + window_m, pin[1] + window_m)
    faces = []
    hits = resolve_strtree_query(tree.query(win), mud) if tree is not None else list(mud or ())
    for geom in hits:
        try:
            if geom is None or geom.is_empty or not geom.intersects(win):
                continue
        except (TypeError, ValueError):
            continue
        faces.append(geom)
    print(
        f'tile {tile["id"]} pin={pin[0]:.0f},{pin[1]:.0f} '
        f'mud_ha={tile.get("mud_ha", 0):.0f} faces={len(faces)}',
        flush=True)
    if not faces:
        print(f'  skip {tile["id"]}: no faces', flush=True)
        return False
    coast = _load_coast(coast_path, pin, window_m + 2000.0)
    rings = mudflat_perimeter_keepout(
        faces, coast,
        coast_clear_m=float(args.coast_clear_m),
        inset_ratio=float(args.inset_ratio),
        min_line_m=float(args.min_line_m),
        simplify_m=12.0,
        min_area_m2=100.0,
        origin_xy=pin,
        radius_m=window_m,
    )
    if not rings:
        print(f'  skip {tile["id"]}: empty keepout', flush=True)
        return False
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    count = dump_keepout_geojson(
        str(dest), rings,
        {
            'rule': 'mudflat_perimeter',
            'site': tile['id'],
            'lat': tile.get('lat'),
            'lon': tile.get('lon'),
            'window_m': window_m,
            'step_m': float(args.step_m),
        })
    meta = {
        'id': tile['id'],
        'lat': tile.get('lat'),
        'lon': tile.get('lon'),
        'cx': tile['cx'],
        'cy': tile['cy'],
        'window_m': window_m,
        'rings': count,
        'baked_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
    }
    (Path(out_dir) / 'meta.json').write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(f'  wrote {count} rings {dest}', flush=True)
    return True


def main(argv=None):
    args = parse_args(argv)
    from eclipse_pkg.tide_waterline import load_mudflat_polygons

    mud_path = args.mudflat or default_mud_path()
    coast_path = args.coast or default_coast_path()
    if not mud_path or not os.path.isfile(mud_path):
        raise SystemExit(f'갯벌 shapefile 없음: {mud_path or "(없음)"}')
    if not coast_path or not os.path.isfile(coast_path):
        raise SystemExit(f'해안선 shapefile 없음: {coast_path or "(없음)"}')

    out_root = _out_root(args)
    out_root.mkdir(parents=True, exist_ok=True)
    index_src = args.from_index or (
        str(_default_index_path()) if _default_index_path().is_file() else '')
    if index_src:
        tiles = _load_index_tiles(index_src)
        print(f'reuse index {index_src} tiles={len(tiles)}', flush=True)
    else:
        print(f'load mud {mud_path}', flush=True)
        mud = load_mudflat_polygons(mud_path)
        tiles = build_index(args, mud)
        print(f'tiles {len(tiles)} window={args.window_m:.0f} '
              f'step={args.step_m:.0f}', flush=True)

    meta = {
        'window_m': float(args.window_m),
        'step_m': float(args.step_m),
        'min_mud_ha': float(args.min_mud_ha),
        'mudflat': mud_path,
        'coast': coast_path,
        'count': len(tiles),
        'baked_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'from_index': index_src,
    }
    _write_index_json(out_root / 'index.json', tiles, meta)
    print(f'wrote {out_root / "index.json"}', flush=True)

    if args.list:
        return 0

    mud = load_mudflat_polygons(mud_path)
    from shapely.strtree import STRtree
    tree = STRtree(mud) if mud else None
    if not args.no_plot:
        _plot_index(
            out_root / 'index.png', tiles, mud,
            title=f'keepout tiles n={len(tiles)} window={args.window_m:.0f}m')

    if args.all:
        selected = list(tiles)
    else:
        selected = _select_tiles(tiles, args)
    if not selected:
        print('굽을 칸이 없다. --all 또는 --around-lat/lon 또는 --ids', flush=True)
        return 0
    print(f'bake {len(selected)} tiles', flush=True)
    ok = 0
    for tile in selected:
        if _bake_one(tile, mud, tree, coast_path, args, out_root / tile['id']):
            ok += 1
    print(f'done {ok}/{len(selected)}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
