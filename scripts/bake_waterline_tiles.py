#!/usr/bin/env python3
"""제부 창 크기로 수위선 타일을 굽는다. 로컬 PNG 포함.

새 GPS 칸: progress/TIDE_NEW_SITE.md.
named --install 만 하고 칸 JSON을 안 넣으면 GPS가 제부 타일만 본다.

전국 인덱스:
  python3 scripts/bake_waterline_tiles.py --list

제부 주변 3x3 (PNG 21장):
  python3 scripts/bake_waterline_tiles.py --around-lat 37.173 --around-lon 126.623 --ring 1
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

from bake_tide_waterline import (  # noqa: E402
    _alpha_list,
    _write_index,
    _write_line_geojson,
    _write_wl_plot,
    default_coast_path,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='수위선 타일 인덱스·굽기')
    parser.add_argument('--mudflat', default='/home/lee/WGIS_TIDFLT.shp')
    parser.add_argument('--coast', default='')
    parser.add_argument('--out-dir', default='')
    parser.add_argument('--window-m', type=float, default=9000.0)
    parser.add_argument('--step-m', type=float, default=12000.0)
    parser.add_argument('--cell-m', type=float, default=12.0)
    parser.add_argument('--min-mud-ha', type=float, default=50.0)
    parser.add_argument('--min-closed-len-m', type=float, default=2000.0)
    parser.add_argument('--min-closed-area-ha', type=float, default=5.0)
    parser.add_argument('--coast-clear-m', type=float, default=40.0)
    parser.add_argument('--step', type=float, default=0.05)
    parser.add_argument('--alphas', default='', help='쉼표 α. 비우면 --step 전체')
    parser.add_argument('--list', action='store_true', help='인덱스만 쓰고 굽지 않음')
    parser.add_argument('--around-lat', type=float, default=None)
    parser.add_argument('--around-lon', type=float, default=None)
    parser.add_argument('--ring', type=int, default=1, help='중심 타일 주변 칸. 1=3x3')
    parser.add_argument('--ids', default='', help='쉼표 타일 id. 있으면 그 칸만')
    return parser.parse_args(argv)


def _out_root(args):
    if args.out_dir:
        return Path(args.out_dir)
    return ROOT / 'progress' / 'waterline_tiles'


def _to_lonlat(x_val, y_val):
    import pyproj
    lon, lat = pyproj.Transformer.from_crs(
        'EPSG:5186', 'EPSG:4326', always_xy=True).transform(x_val, y_val)
    return float(lat), float(lon)


def _to_5186(lat, lon):
    import pyproj
    return pyproj.Transformer.from_crs(
        'EPSG:4326', 'EPSG:5186', always_xy=True).transform(lon, lat)


def _write_index_json(path, tiles, meta):
    payload = dict(meta or {})
    payload['tiles'] = tiles
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')


def _plot_index(path, tiles, mud, highlight_ids=None, title=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    highlight = set(highlight_ids or ())
    fig, ax = plt.subplots(figsize=(10.0, 12.0))
    for poly in mud or ():
        try:
            xs, ys = poly.exterior.xy
        except (TypeError, ValueError, AttributeError):
            continue
        ax.fill(xs, ys, color='#9ec5e8', alpha=0.45, linewidth=0)
    for tile in tiles or ():
        color = '#c0392b' if tile['id'] in highlight else '#2c3e50'
        lw = 1.6 if tile['id'] in highlight else 0.4
        ax.add_patch(Rectangle(
            (tile['minx'], tile['miny']),
            tile['maxx'] - tile['minx'],
            tile['maxy'] - tile['miny'],
            fill=False, edgecolor=color, lw=lw, alpha=0.9))
        if tile['id'] in highlight:
            ax.plot(tile['cx'], tile['cy'], 'o', color='#c0392b', ms=4)
    ax.set_aspect('equal')
    ax.set_title(title or 'waterline tiles')
    ax.set_xlabel('EPSG:5186 E (m)')
    ax.set_ylabel('EPSG:5186 N (m)')
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _attach_lonlat(tiles):
    out = []
    for tile in tiles:
        rec = dict(tile)
        lat, lon = _to_lonlat(rec['cx'], rec['cy'])
        rec['lat'] = lat
        rec['lon'] = lon
        out.append(rec)
    return out


def build_index(args, mud):
    from eclipse_pkg.tide_waterline_tiles import list_mud_tiles

    tiles = list_mud_tiles(
        mud,
        window_m=float(args.window_m),
        step_m=float(args.step_m),
        min_mud_ha=float(args.min_mud_ha))
    return _attach_lonlat(tiles)


def _select_tiles(tiles, args):
    from eclipse_pkg.tide_waterline_tiles import (
        neighbor_indices, tile_index_xy)

    if str(args.ids or '').strip():
        wanted = {token.strip() for token in args.ids.split(',') if token.strip()}
        return [tile for tile in tiles if tile['id'] in wanted]
    if args.around_lat is None or args.around_lon is None:
        return []
    x_val, y_val = _to_5186(float(args.around_lat), float(args.around_lon))
    ix, iy = tile_index_xy(x_val, y_val, float(args.step_m))
    cells = set(neighbor_indices(ix, iy, ring=int(args.ring)))
    selected = [
        tile for tile in tiles
        if (int(tile['ix']), int(tile['iy'])) in cells]
    selected.sort(key=lambda item: (item['iy'], item['ix']))
    return selected


def _bake_one(tile, mud, coast_path, args, out_dir):
    from eclipse_pkg.tide_waterline import dump_waterline_steps, load_line_geoms
    from eclipse_pkg.tide_waterline_width import waterline_width_steps

    pin = (float(tile['cx']), float(tile['cy']))
    window_m = float(args.window_m)
    cell_m = float(args.cell_m)
    coast = load_line_geoms(coast_path, origin_xy=pin, radius_m=window_m)
    alphas = _alpha_list(args)
    print(
        f'tile {tile["id"]} pin={pin[0]:.0f},{pin[1]:.0f} '
        f'mud_ha={tile["mud_ha"]:.0f} coast={len(coast)}',
        flush=True)
    baked = waterline_width_steps(
        mud, coast, alphas,
        origin_xy=pin, radius_m=window_m,
        cell_m=cell_m,
        coast_clear_m=float(args.coast_clear_m),
        min_closed_len_m=float(args.min_closed_len_m),
        min_closed_area_m2=float(args.min_closed_area_ha) * 10000.0)
    if not baked:
        print(f'  skip {tile["id"]}: width grid empty', flush=True)
        return False
    out_dir.mkdir(parents=True, exist_ok=True)
    dump_waterline_steps(
        str(out_dir / 'waterline_steps.json'), baked,
        {
            'id': tile['id'],
            'mode': 'width',
            'cx': tile['cx'],
            'cy': tile['cy'],
            'lat': tile.get('lat'),
            'lon': tile.get('lon'),
            'window_m': window_m,
            'step_m': float(args.step_m),
            'cell_m': cell_m,
            'mud_ha': tile['mud_ha'],
            'baked_at': datetime.now(timezone.utc).strftime(
                '%Y-%m-%dT%H:%M:%SZ'),
        })
    preview = out_dir / 'preview'
    geo = out_dir / 'geojson'
    for alpha, rings in baked:
        tag = f'{alpha:.2f}'
        _write_line_geojson(geo / f'a_{tag}.geojson', rings, alpha)
        _write_wl_plot(
            preview / f'a_{tag}.png',
            pin, [], rings,
            f'{tile["id"]}  α={tag}  cell={cell_m:.0f}m',
            mud_polys=mud, coast_geoms=coast,
            limits=(
                pin[0] - window_m, pin[0] + window_m,
                pin[1] - window_m, pin[1] + window_m))
        print(f'  α={tag} rings={len(rings)}', flush=True)
    _write_index(
        out_dir, tile['id'], 0.0, [row[0] for row in baked],
        mode='width', window_m=window_m)
    return True


def main(argv=None):
    args = parse_args(argv)
    from eclipse_pkg.tide_waterline import load_mudflat_polygons

    mud_path = args.mudflat
    if not mud_path or not os.path.isfile(mud_path):
        raise SystemExit(f'갯벌 shapefile 없음: {mud_path}')
    coast_path = args.coast or default_coast_path()
    if not coast_path or not os.path.isfile(coast_path):
        raise SystemExit(f'해안선 shapefile 없음: {coast_path}')
    print(f'load mud {mud_path}', flush=True)
    mud = load_mudflat_polygons(mud_path)
    print(f'mud faces {len(mud)}', flush=True)
    tiles = build_index(args, mud)
    print(f'tiles {len(tiles)} window={args.window_m:.0f} step={args.step_m:.0f}',
          flush=True)
    out_root = _out_root(args)
    _write_index_json(
        out_root / 'index.json', tiles,
        {
            'window_m': float(args.window_m),
            'step_m': float(args.step_m),
            'cell_m': float(args.cell_m),
            'min_mud_ha': float(args.min_mud_ha),
            'mudflat': mud_path,
            'coast': coast_path,
            'count': len(tiles),
            'baked_at': datetime.now(timezone.utc).strftime(
                '%Y-%m-%dT%H:%M:%SZ'),
        })
    _plot_index(
        out_root / 'index.png', tiles, mud,
        title=f'waterline tiles n={len(tiles)} window={args.window_m:.0f}m')
    print(f'index {out_root / "index.json"}', flush=True)
    if args.list:
        return 0
    selected = _select_tiles(tiles, args)
    if not selected:
        raise SystemExit('굽을 타일이 없다. --around-lat/lon 또는 --ids 를 넣어라')
    print(f'bake {len(selected)} tiles: {[t["id"] for t in selected]}', flush=True)
    _plot_index(
        out_root / 'index_selected.png', tiles, mud,
        highlight_ids=[t['id'] for t in selected],
        title=f'selected {len(selected)} / {len(tiles)}')
    ok = 0
    for tile in selected:
        if _bake_one(tile, mud, coast_path, args, out_root / tile['id']):
            ok += 1
    print(f'done ok={ok}/{len(selected)} -> {out_root}', flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
