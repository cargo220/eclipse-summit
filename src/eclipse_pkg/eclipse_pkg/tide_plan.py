"""국립해양조사원 조석예보 API(data.go.kr) 기반 갯벌 활동 윈도우 계획 도구."""

import argparse
import json
import math
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timedelta

try:
    import yaml
except ImportError:  # PyYAML이 없는 환경은 JSON 형식 설정으로 폴백한다.
    yaml = None

try:
    from shapely.geometry import LineString as ShapelyLineString
    from shapely.geometry import Point as ShapelyPoint
    from shapely.geometry import Polygon as ShapelyPolygon
    from shapely.geometry import shape as shapely_shape
    from shapely.ops import unary_union as shapely_unary_union
    from shapely.strtree import STRtree as ShapelySTRtree
except ImportError:
    ShapelyLineString = None
    ShapelyPoint = None
    ShapelyPolygon = None
    shapely_shape = None
    shapely_unary_union = None
    ShapelySTRtree = None

# 연안정보도 갯벌(WGIS_TIDFLT) 원본 CRS. 클립본은 EPSG:5186.
_UTMK = (
    '+proj=tmerc +lat_0=38 +lon_0=127.5 +k=0.9996 '
    '+x_0=1000000 +y_0=2000000 +ellps=WGS84 +units=m +no_defs'
)

# 워크스페이스 기본 설정 경로(CWD 기준). 파일이 없으면 내장 기본값을 사용한다.
DEFAULT_CONFIG_PATH = os.path.join('src', 'eclipse_pkg', 'config', 'tide_ops.yaml')

# 설정 파일이 없거나 키가 빠져 있을 때 쓰는 내장 기본값.
# API 엔드포인트/지점 코드는 조석예보 명세. 임계값은 현장 기준.
DEFAULT_OPS = {
    'station_code': 'DT_0068',
    'station_name': '위도 (곰소만·변산 최근접 예보지점)',
    'api_base_url': 'https://apis.data.go.kr',
    'api_service_path': '/1192136/tideFcstTime/GetTideFcstTimeApiService',
    'interval_min': 60,
    'threshold': {
        'mode': 'absolute',
        'tide_level_m': 0.0,
        'rise_m': 0.0,
    },
    'safety_margin_min': 30,
    'cache_dir': os.path.join('datasets', 'tide_cache'),
    'retreat_topic': '/mission/retreat',
    'status_topic': '/mission/tide_status',
    'polygon_margin_m': 4.0,
    'enable_gps_station_select': True,
    'retreat': {
        'mode': 'dynamic',
        'ground_elevation_m': None,  # 있으면 옛 공식과 공간 시각 중 이른 쪽
        'safety_margin_m': 0.3,
        'retreat_distance_m': 300,  # 지상고 경로 폴백. 공간 모델은 gps_home
        'robot_max_speed_mps': 0.9742,
        'robot_avg_speed_mps': 0.3,
        'time_buffer_min': 10,
        'spatial_margin_m': 20.0,
        'min_work_m': 15.0,
        'fallback_mud_width_m': 400.0,
    },
}

# 플랫폼 선속도 상한. 철수 시간은 이 값을 넘기지 않는다.
PLATFORM_MAX_SPEED_MPS = 0.9742  # D250 · G2.5 · 130 tick

_TIME_FORMATS = (
    '%Y-%m-%d %H:%M',
    '%Y-%m-%d %H:%M:%S',
    '%Y-%m-%dT%H:%M',
    '%Y-%m-%dT%H:%M:%S',
    '%Y%m%d %H:%M',
    '%Y%m%d%H%M',
)

# 활용가이드 명세: numOfRows 최대 300. 하루치(24시간 × 1분 간격 ≤ 1440행)는 페이지네이션 필요 가능.
_PAGE_SIZE = 300


# 전국 조위관측소 목록 (관측소 코드, 이름, 위도, 경도)
# 주요 서해안 갯벌 지역 관측소 위주 + 전국 커버
_TIDE_STATIONS = [
    {"code": "DT_0068", "name": "위도", "lat": 35.6200, "lon": 126.2500},
    {"code": "DT_0018", "name": "군산", "lat": 35.9833, "lon": 126.7000},
    {"code": "DT_0001", "name": "인천", "lat": 37.45194, "lon": 126.59222},
    {"code": "DT_0030", "name": "대천", "lat": 36.3500, "lon": 126.5167},
    {"code": "DT_0043", "name": "목포", "lat": 34.7833, "lon": 126.3667},
    {"code": "DT_0012", "name": "평택", "lat": 36.9833, "lon": 126.8000},
    {"code": "DT_0045", "name": "완도", "lat": 34.3167, "lon": 126.7500},
    {"code": "DT_0060", "name": "여수", "lat": 34.7333, "lon": 127.7333},
    {"code": "DT_0035", "name": "군산외항", "lat": 35.9833, "lon": 126.5500},
    {"code": "DT_0072", "name": "부산", "lat": 35.1000, "lon": 129.0333},
    {"code": "DT_0004", "name": "제주", "lat": 33.5275, "lon": 126.54305},
    {"code": "DT_0011", "name": "안흥", "lat": 36.6667, "lon": 126.1333},
    {"code": "DT_0025", "name": "보령", "lat": 36.3333, "lon": 126.5167},
    {"code": "DT_0055", "name": "통영", "lat": 34.8333, "lon": 128.4167},
    {"code": "DT_0063", "name": "제주", "lat": 33.5167, "lon": 126.5333},
    {"code": "DT_0058", "name": "고흥", "lat": 34.6167, "lon": 127.2833},
    {"code": "DT_0038", "name": "장항", "lat": 36.0000, "lon": 126.6667},
    {"code": "DT_0021", "name": "태안", "lat": 36.7500, "lon": 126.3000},
    {"code": "DT_0028", "name": "서천", "lat": 36.0833, "lon": 126.7000},
    {"code": "DT_0049", "name": "진도", "lat": 34.4833, "lon": 126.2667},
]


def haversine_distance(lat1, lon1, lat2, lon2):
    """두 지점 간 거리를 Haversine 공식으로 계산 (미터)."""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def find_nearest_station(lat, lon):
    """주어진 위도/경도에서 가장 가까운 조위관측소를 반환한다.
    반환: {"code": ..., "name": ..., "lat": ..., "lon": ..., "distance_m": ...}
    """
    best = None
    best_dist = float('inf')
    for station in _TIDE_STATIONS:
        dist = haversine_distance(lat, lon, station['lat'], station['lon'])
        if dist < best_dist:
            best_dist = dist
            best = dict(station)
            best['distance_m'] = round(best_dist, 1)
    return best


def get_stations():
    """관측소 목록 전체를 반환한다."""
    return list(_TIDE_STATIONS)


# keepout_site → 조석예보 지점. 없는 이름은 빈 문자열(yaml station_code).
_SITE_STATION_CODE = {
    'incheon': 'DT_0001',
    'gomso': 'DT_0068',
}


def station_code_for_keepout_site(site):
    """keepout_site 이름에 대응하는 예보지점 코드. 모르면 ''."""
    return _SITE_STATION_CODE.get(str(site or '').strip().lower(), '')


def yaw_from_direction(dx, dy):
    """2D 방향 벡터의 ENU yaw(rad)를 반환한다. 영벡터·비유한이면 None."""
    if not (math.isfinite(dx) and math.isfinite(dy)):
        return None
    if dx == 0.0 and dy == 0.0:
        return None
    return math.atan2(dy, dx)


def yaw_to_quat_zw(yaw):
    """yaw-only quaternion의 (z, w). /imu/mag_heading·/gps/heading_visual 과 동일."""
    half = 0.5 * float(yaw)
    return (math.sin(half), math.cos(half))


def resolve_strtree_nearest(nearest, geometries):
    """STRtree.nearest() 결과를 geometry로 정규화한다.

    Shapely 1.8은 geometry를, Shapely 2.x는 정수 인덱스를 반환한다.
    인덱스 0은 유효한 첫 번째 도형이다 — falsy 검사로 버리면 안 된다.
    """
    if nearest is None:
        return None
    if hasattr(nearest, 'geom_type'):
        return nearest
    try:
        idx = int(nearest)
    except (TypeError, ValueError):
        return None
    if geometries is None or idx < 0 or idx >= len(geometries):
        return None
    return geometries[idx]


def resolve_strtree_query(hits, geometries):
    """STRtree.query() 결과를 geometry 목록으로 정규화한다.

    Shapely 1.8은 geometry 시퀀스, Shapely 2.x는 정수 인덱스를 반환한다.
    인덱스 0은 유효하다.
    """
    if hits is None:
        return []
    resolved = []
    try:
        items = list(hits)
    except TypeError:
        items = [hits]
    for hit in items:
        geom = resolve_strtree_nearest(hit, geometries)
        if geom is not None:
            resolved.append(geom)
    return resolved


def seaward_rectangle(
        origin_x, origin_y, dx, dy,
        length_m, half_width_m, start_offset_m):
    """방향 (dx, dy) 으로 뻗는 사각형 꼭짓점 4개(근좌, 근우, 원우, 원좌).

    origin 은 기준점이다. 로봇에 붙일 용도가 아니다.
    """
    if yaw_from_direction(dx, dy) is None:
        return None
    values = (origin_x, origin_y, length_m, half_width_m, start_offset_m)
    if not all(math.isfinite(float(v)) for v in values):
        return None
    if length_m <= 0.0 or half_width_m <= 0.0 or start_offset_m < 0.0:
        return None
    dist = math.hypot(dx, dy)
    ux, uy = dx / dist, dy / dist
    wx, wy = -uy, ux
    near_x = origin_x + ux * start_offset_m
    near_y = origin_y + uy * start_offset_m
    far_x = origin_x + ux * (start_offset_m + length_m)
    far_y = origin_y + uy * (start_offset_m + length_m)
    return [
        [near_x + wx * half_width_m, near_y + wy * half_width_m],
        [near_x - wx * half_width_m, near_y - wy * half_width_m],
        [far_x - wx * half_width_m, far_y - wy * half_width_m],
        [far_x + wx * half_width_m, far_y + wy * half_width_m],
    ]


def snap_to_coast_offset(px, py, cx, cy, offset_m):
    """점 (px, py)를 해안선점 (cx, cy)에서 같은 쪽으로 offset_m 떨어진 곳으로 옮긴다.

    costmap 창 안에 물이 보이게, 가짜 GPS를 해안선 근처로 당길 때 쓴다.
    """
    if not all(math.isfinite(float(v)) for v in (px, py, cx, cy, offset_m)):
        return None
    if offset_m < 0.0:
        return None
    dx = cx - px
    dy = cy - py
    dist = math.hypot(dx, dy)
    if dist == 0.0:
        return None
    ux, uy = dx / dist, dy / dist
    return (cx - ux * offset_m, cy - uy * offset_m)


def coastline_water_polygon(
        robot_x, robot_y, coast_dx, coast_dy,
        length_m, half_width_m):
    """한 점 기준 사각형. 해안선 띠로 바꾼 뒤에도 테스트용으로 남긴다."""
    if yaw_from_direction(coast_dx, coast_dy) is None:
        return None
    if not (math.isfinite(robot_x) and math.isfinite(robot_y)):
        return None
    coast_x = robot_x + coast_dx
    coast_y = robot_y + coast_dy
    return seaward_rectangle(
        coast_x, coast_y, coast_dx, coast_dy,
        length_m, half_width_m, start_offset_m=0.0)


def ops_config_with_defaults(section):
    """tide_ops 섹션 dict에 내장 기본값을 채워 반환한다."""
    merged = dict(DEFAULT_OPS)
    merged.update(section or {})
    threshold = dict(DEFAULT_OPS['threshold'])
    threshold.update(merged.get('threshold') or {})
    merged['threshold'] = threshold
    retreat = dict(DEFAULT_OPS['retreat'])
    retreat.update(merged.get('retreat') or {})
    merged['retreat'] = retreat
    return merged


def load_ops_config(path):
    """YAML(또는 JSON) 설정 파일을 읽어 tide_ops 섹션을 기본값과 병합해 반환한다."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f'설정 파일을 찾을 수 없습니다: {path}')
    with open(path, 'r', encoding='utf-8') as config_file:
        text = config_file.read()
    if yaml is not None:
        data = yaml.safe_load(text) or {}
    else:
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f'PyYAML이 없어 {path}를 JSON으로만 파싱할 수 있는데 JSON 형식이 아닙니다. '
                'PyYAML을 설치하거나 JSON 형식 설정을 사용하세요.'
            ) from exc
    if not isinstance(data, dict) or not isinstance(data.get('tide_ops'), dict):
        raise ValueError(f'설정 파일에 tide_ops 섹션(dict)이 없습니다: {path}')
    return ops_config_with_defaults(data['tide_ops'])


def resolve_config(explicit_path=None):
    """명시 경로 → 워크스페이스 기본 경로 → 내장 기본값 순으로 설정을 결정한다."""
    if explicit_path:
        return load_ops_config(explicit_path)
    if os.path.isfile(DEFAULT_CONFIG_PATH):
        return load_ops_config(DEFAULT_CONFIG_PATH)
    return ops_config_with_defaults({})


def fetch_tide_series(config, date_from, date_to):
    """조석예보 API를 날짜별·페이지별로 조회해 정규화된 조위 시계열 리스트를 반환한다."""
    # 서비스 키는 환경변수로만 읽고 어떤 파일에도 기록하지 않는다.
    service_key = os.environ.get('DATA_GO_KR_SERVICE_KEY', '').strip()
    if not service_key:
        raise RuntimeError(
            '환경변수 DATA_GO_KR_SERVICE_KEY가 없습니다. 공공데이터포털 서비스 키를 '
            '환경변수로만 설정하세요(파일에 기록 금지).'
        )
    service_path = (config.get('api_service_path') or '').strip()
    if not service_path:
        raise RuntimeError('api_service_path가 비어 있습니다. config/tide_ops.yaml에 엔드포인트를 설정하세요.')

    interval_min = int(config.get('interval_min', 60))
    if not 1 <= interval_min <= 60:
        raise ValueError(f'interval_min은 1~60(분)이어야 합니다: {interval_min}')
    start_day = _validate_date_arg(date_from, 'date_from')
    end_day = _validate_date_arg(date_to, 'date_to')
    if start_day > end_day:
        raise ValueError(f'조회 기간이 잘못되었습니다: {date_from} ~ {date_to}')

    base_url = (config.get('api_base_url') or 'https://apis.data.go.kr').rstrip('/')
    station = config.get('station_code') or 'TBD'

    series = []
    current = datetime.strptime(start_day, '%Y-%m-%d').date()
    last_day = datetime.strptime(end_day, '%Y-%m-%d').date()
    while current <= last_day:
        # 활용가이드 명세: reqDate는 YYYYMMDD 하루 단위 조회라 기간 조회는 날짜별 루프.
        series.extend(_fetch_day(base_url, service_path, service_key, station,
                                 current.strftime('%Y%m%d'), interval_min))
        current += timedelta(days=1)

    # 페이지네이션·날짜 루프에서 겹칠 수 있는 중복 시각 제거.
    unique = {}
    for point in series:
        unique[point['time']] = point
    result = sorted(unique.values(), key=lambda point: point['time'])
    if not result:
        raise RuntimeError(
            '조석예보 API 응답에서 조위 시계열을 찾지 못했습니다(빈 응답이거나 스키마 불일치).'
        )
    return result


def _fetch_day(base_url, service_path, service_key, station, req_date, interval_min):
    """하루치 예보를 totalCount 기준으로 페이지네이션하며 전부 조회한다."""
    day_series = []
    page_no = 1
    while page_no <= 100:  # 안전 상한(300행×100페이지 = 3만행, 하루치 대비 충분)
        payload = _fetch_page(base_url, service_path, service_key, station, req_date, interval_min, page_no)
        body = _response_root(payload).get('body')
        if not isinstance(body, dict):
            body = {}
        day_series.extend(parse_tide_response(payload))

        try:
            total_count = int(body.get('totalCount'))
        except (TypeError, ValueError):
            break  # totalCount가 없으면 첫 페이지 결과를 전체로 간주
        if len(day_series) >= total_count or page_no * _PAGE_SIZE >= total_count:
            break
        page_no += 1
    return day_series


def _fetch_page(base_url, service_path, service_key, station, req_date, interval_min, page_no):
    """한 페이지를 조회해 JSON을 파싱하고 resultCode를 검증한 뒤 반환한다."""
    # 활용가이드 명세: serviceKey는 URL Encode 필요, type=json.
    query = urllib.parse.urlencode({
        'obsCode': station,
        'reqDate': req_date,
        'min': interval_min,
        'numOfRows': _PAGE_SIZE,
        'pageNo': page_no,
        'type': 'json',
    })
    query = f'serviceKey={urllib.parse.quote(service_key, safe="")}&{query}'
    request = urllib.request.Request(f'{base_url}{service_path}?{query}')
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            payload = json.loads(response.read().decode('utf-8'))
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f'조석예보 API HTTP 오류: {exc.code} {exc.reason}') from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f'조석예보 API 접속 실패: {exc.reason}') from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f'조석예보 API 응답이 JSON이 아닙니다: {exc}') from exc

    root = _response_root(payload)
    header = root.get('header')
    if not isinstance(header, dict):
        raise RuntimeError(f'조석예보 API 응답에 header가 없습니다: {str(payload)[:200]}')
    result_code = str(header.get('resultCode', ''))
    if result_code != '00':
        raise RuntimeError(
            f"조석예보 API 오류: resultCode={result_code}, resultMsg={header.get('resultMsg', '')}"
        )
    return payload


def parse_tide_response(raw):
    """조석예보 원시 응답(JSON 파싱 결과)을 정규화 시계열로 변환한다.

    반환 형식: [{"time": "YYYY-MM-DD HH:MM", "tide_level_m": float}, ...] (시각 오름차순)

    활용가이드 확정 명세(1192136/tideFcstTime):
    - header.resultCode "00" = 정상 (fetch에서 검증)
    - body.items.item[]: obsvtrNm(지점명), lot, lat,
      predcDt("YYYY-MM-DD HH:MM"), tdlvHgt(조위높이, cm 단위)
    - item이 1개면 items.item이 리스트가 아닌 dict일 수 있어 둘 다 처리.
    - 실측: JSON 응답은 'response' 래퍼 없이 올 수 있어 둘 다 처리(_response_root).
    """
    items = _find_items(raw)
    series = []
    for item in items:
        if not isinstance(item, dict):
            continue
        predc_dt = item.get('predcDt')
        tdlv_hgt = item.get('tdlvHgt')
        if predc_dt is None or tdlv_hgt is None:
            continue
        parsed_time = _parse_time(str(predc_dt))
        if parsed_time is None:
            continue
        try:
            level_m = float(tdlv_hgt) / 100.0  # 활용가이드 명세: tdlvHgt는 cm 단위
        except (TypeError, ValueError):
            continue
        series.append({
            'time': parsed_time.strftime('%Y-%m-%d %H:%M'),
            'tide_level_m': level_m,
        })
    series.sort(key=lambda point: point['time'])
    return series


def save_cache(series, config, station, date_from, date_to):
    """조위 시계열을 cache_dir 아래 JSON 캐시 파일로 저장하고 경로를 반환한다."""
    cache_dir = config.get('cache_dir') or os.path.join('datasets', 'tide_cache')
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, f'tide_{station}_{date_from}_{date_to}.json')
    payload = {
        'station': station,
        'date_from': date_from,
        'date_to': date_to,
        'saved_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'series': series,
    }
    with open(path, 'w', encoding='utf-8') as cache_file:
        json.dump(payload, cache_file, ensure_ascii=False, indent=2)
    return path


def load_cache(path):
    """캐시 JSON을 읽어 {"series": [...], ...} dict로 반환한다(맨몸 리스트도 허용)."""
    if not path:
        raise ValueError('캐시 파일 경로가 비어 있습니다.')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'캐시 파일을 찾을 수 없습니다: {path}')
    with open(path, 'r', encoding='utf-8') as cache_file:
        data = json.load(cache_file)
    if isinstance(data, list):
        return {'series': data}
    if isinstance(data, dict) and isinstance(data.get('series'), list):
        return data
    raise ValueError(f'캐시 파일 형식이 잘못되었습니다(series 리스트 필요): {path}')


def _as_date(value):
    """datetime/date를 date로 정규화한다."""
    if isinstance(value, datetime):
        return value.date()
    return value


def operational_date_window(today):
    """운용일 캐시 구간: 당일 ~ 다음날 (YYYY-MM-DD, YYYY-MM-DD)."""
    day = _as_date(today)
    return day.strftime('%Y-%m-%d'), (day + timedelta(days=1)).strftime('%Y-%m-%d')


def cache_filename(station, date_from, date_to):
    """save_cache와 같은 파일명 규칙."""
    return f'tide_{station}_{date_from}_{date_to}.json'


def resolve_writable_cache_dir(preferred, fallback='datasets/tide_cache'):
    """preferred가 있거나 부모가 있으면 쓰고, 아니면 fallback.

    Jetson launch는 /workspaces/eclipse-test-2/datasets/tide_cache 를 넘긴다.
    그 트리가 없는 로컬 머신에서는 yaml의 상대 cache_dir로 떨어진다.
    """
    preferred = (preferred or '').strip()
    fallback = (fallback or 'datasets/tide_cache').strip()
    if preferred:
        if os.path.isdir(preferred):
            return preferred
        parent = os.path.dirname(preferred.rstrip(os.sep))
        if parent and os.path.isdir(parent):
            return preferred
    return fallback


def operational_cache_path(config, now, station=None):
    """운용일 캐시 파일 경로와 날짜 구간을 반환한다."""
    date_from, date_to = operational_date_window(now)
    station = station or config.get('station_code') or 'DT_0068'
    cache_dir = config.get('cache_dir') or 'datasets/tide_cache'
    path = os.path.join(cache_dir, cache_filename(station, date_from, date_to))
    return path, date_from, date_to, station


def cache_covers_when(points, when):
    """시계열이 when과 같은 로컬 날짜의 샘플을 하나라도 가지면 True.

    find_nearest는 8/7 23:00을 8/17의 '가장 가까운' 값으로 고른다.
    날짜가 다른 캐시는 오늘 예보가 아니다.
    """
    if not points:
        return False
    target = _as_date(when)
    return any(_as_date(sample[0]) == target for sample in points)


def lookup_tide(points, when):
    """날짜가 커버되면 최근접 샘플, 아니면 (None, None)."""
    if not cache_covers_when(points, when):
        return None, None
    return find_nearest(points, when)


def _same_day_points(points, when):
    """when과 같은 로컬 날짜의 샘플만 시각순으로 반환한다."""
    target = _as_date(when)
    return sorted(
        (sample for sample in points if _as_date(sample[0]) == target),
        key=lambda sample: sample[0],
    )


def interpolate_tide(points, when):
    """같은 날 시계열을 시각에 대해 선형 보간한다. 커버 불가면 None.

    첫 샘플 이전은 첫 값, 마지막 이후는 마지막 값. 다른 날짜로 넘어가지 않는다.
    """
    day_points = _same_day_points(points, when)
    if not day_points:
        return None
    if len(day_points) == 1 or when <= day_points[0][0]:
        return float(day_points[0][1])
    if when >= day_points[-1][0]:
        return float(day_points[-1][1])
    for (t0, l0), (t1, l1) in zip(day_points, day_points[1:]):
        if t0 <= when <= t1:
            span = (t1 - t0).total_seconds()
            if span <= 0.0:
                return float(l1)
            ratio = (when - t0).total_seconds() / span
            return float(l0) + (float(l1) - float(l0)) * ratio
    return float(day_points[-1][1])


def apply_tide_clock_offset(when, offset_hours):
    """조석 조회 시각만 민다. 시스템 시계·ROS clock 과 무관하다."""
    if when is None:
        return None
    try:
        hours = float(offset_hours or 0.0)
    except (TypeError, ValueError):
        return when
    if hours == 0.0:
        return when
    return when + timedelta(hours=hours)


def apply_tide_clock_speed(origin, elapsed_s, speed):
    """origin 에서 elapsed×speed 초 뒤. speed<=0 이면 origin."""
    if origin is None:
        return None
    try:
        elapsed = float(elapsed_s)
        rate = float(speed)
    except (TypeError, ValueError):
        return origin
    if not math.isfinite(elapsed) or elapsed < 0.0:
        elapsed = 0.0
    if not math.isfinite(rate) or rate <= 0.0:
        return origin
    return origin + timedelta(seconds=elapsed * rate)


def wrap_time_to_points(when, points):
    """시계열 첫~끝 안으로 접는다. 미리보기 배속 재생용. 샘플 없으면 when."""
    if when is None or not points:
        return when
    start = points[0][0]
    end = points[-1][0]
    try:
        span = (end - start).total_seconds()
    except (TypeError, ValueError):
        return when
    if not math.isfinite(span) or span <= 0.0:
        return start
    try:
        elapsed = (when - start).total_seconds() % span
    except (TypeError, ValueError):
        return when
    if elapsed < 0.0:
        elapsed += span
    return start + timedelta(seconds=elapsed)


def waterline_sweep_alpha(elapsed_s, period_s):
    """미리보기용 α. period 동안 0→1, 같은 시간 1→0. period<=0 이면 None."""
    try:
        elapsed = float(elapsed_s)
        period = float(period_s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(elapsed) or not math.isfinite(period) or period <= 0.0:
        return None
    cycle = elapsed % (2.0 * period)
    if cycle <= period:
        return max(0.0, min(1.0, cycle / period))
    return max(0.0, min(1.0, 2.0 - cycle / period))


def tide_range_today(points, when):
    """그날 조위 (최저, 최고). 샘플이 없으면 (None, None)."""
    day_points = _same_day_points(points, when)
    if not day_points:
        return None, None
    levels = [float(level) for _, level in day_points]
    return min(levels), max(levels)


def tide_alpha(level_m, t_low, t_high):
    """조위 → [0, 1]. 간조=0, 만조=1. 범위가 없거나 평탄하면 0."""
    if level_m is None or t_low is None or t_high is None:
        return 0.0
    span = float(t_high) - float(t_low)
    if span <= 0.0:
        return 0.0
    alpha = (float(level_m) - float(t_low)) / span
    return max(0.0, min(1.0, alpha))


def keepout_alpha(alpha, alpha_min=0.08):
    """띠용 α. 만조=1 유지, 간조에도 alpha_min 만큼 남겨 띠가 안 사라지게 한다.

    α_draw = α_min + (1 − α_min) · clamp(α).
    상태 JSON 의 원본 α 는 바꾸지 않는다.
    """
    try:
        value = float(alpha)
    except (TypeError, ValueError):
        value = 0.0
    try:
        floor = float(alpha_min)
    except (TypeError, ValueError):
        floor = 0.08
    value = max(0.0, min(1.0, value))
    floor = max(0.0, min(1.0, floor))
    return floor + (1.0 - floor) * value


def interpolate_waterline(low_xy, high_xy, alpha):
    """L(바깥)과 C(해안) 사이를 α로 보간한다. α=0→L, α=1→C."""
    if low_xy is None or high_xy is None:
        return None
    try:
        lx, ly = float(low_xy[0]), float(low_xy[1])
        hx, hy = float(high_xy[0]), float(high_xy[1])
        weight = float(alpha)
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(v) for v in (lx, ly, hx, hy, weight)):
        return None
    weight = max(0.0, min(1.0, weight))
    return (lx + (hx - lx) * weight, ly + (hy - ly) * weight)


def _ray_segment_hit(ox, oy, ux, uy, ax, ay, bx, by, max_dist):
    """원점에서 단위벡터 광선과 선분 AB의 교차 거리 t. 없으면 None."""
    dx = bx - ax
    dy = by - ay
    denom = ux * dy - uy * dx
    if abs(denom) < 1e-12:
        return None
    rx = ax - ox
    ry = ay - oy
    t = (rx * dy - ry * dx) / denom
    s = (rx * uy - ry * ux) / denom
    if t < 0.0 or t > max_dist or s < 0.0 or s > 1.0:
        return None
    return t


def first_hsl0_along_ray(
        origin_xy, seaward_xy, lines, max_dist_m=200.0, min_dist_m=0.0):
    """해안선 점에서 바다 방향 광선이 선과 처음 만나는 점.

    min_dist 안 교차는 건너뛴다(육지 접착).
    lines: [[(x, y), ...], ...]. 교차 없거나 max_dist 밖이면 None.
    """
    try:
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        sx, sy = float(seaward_xy[0]), float(seaward_xy[1])
        max_dist = float(max_dist_m)
        min_dist = float(min_dist_m)
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(v) for v in (ox, oy, sx, sy, max_dist, min_dist)):
        return None
    if max_dist <= 0.0:
        return None
    if min_dist < 0.0:
        min_dist = 0.0
    if min_dist > max_dist:
        return None
    length = math.hypot(sx, sy)
    if length == 0.0:
        return None
    ux, uy = sx / length, sy / length
    best_t = None
    for line in lines or ():
        if not line or len(line) < 2:
            continue
        for start, end in zip(line, line[1:]):
            try:
                ax, ay = float(start[0]), float(start[1])
                bx, by = float(end[0]), float(end[1])
            except (TypeError, ValueError, IndexError):
                continue
            hit = _ray_segment_hit(ox, oy, ux, uy, ax, ay, bx, by, max_dist)
            if hit is None or hit < min_dist:
                continue
            if best_t is None or hit < best_t:
                best_t = hit
    if best_t is None:
        return None
    return (ox + ux * best_t, oy + uy * best_t)


def last_hsl0_along_ray(origin_xy, seaward_xy, lines, max_dist_m=200.0):
    """해안선 점에서 광선이 HSL=0과 마지막으로 만나는 점(바깥).

    첫 교차(갯골)가 아니라 max_dist 안 가장 먼 교차. 없으면 None.
    """
    try:
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        sx, sy = float(seaward_xy[0]), float(seaward_xy[1])
        max_dist = float(max_dist_m)
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(v) for v in (ox, oy, sx, sy, max_dist)):
        return None
    if max_dist <= 0.0:
        return None
    length = math.hypot(sx, sy)
    if length == 0.0:
        return None
    ux, uy = sx / length, sy / length
    best_t = None
    for line in lines or ():
        if not line or len(line) < 2:
            continue
        for start, end in zip(line, line[1:]):
            try:
                ax, ay = float(start[0]), float(start[1])
                bx, by = float(end[0]), float(end[1])
            except (TypeError, ValueError, IndexError):
                continue
            hit = _ray_segment_hit(ox, oy, ux, uy, ax, ay, bx, by, max_dist)
            if hit is None:
                continue
            if best_t is None or hit > best_t:
                best_t = hit
    if best_t is None:
        return None
    return (ox + ux * best_t, oy + uy * best_t)


def last_boundary_along_ray(origin_xy, seaward_xy, lines, max_dist_m=200.0):
    """해안점에서 광선이 경계와 마지막으로 만나는 점.

    갯벌 외곽(구멍 제외)을 lines 로 넘기면 L. last_hsl0_along_ray 와 동일.
    """
    return last_hsl0_along_ray(origin_xy, seaward_xy, lines, max_dist_m)


def _prj_looks_5186(prj_text):
    text = prj_text or ''
    return any(
        token in text
        for token in ('5186', 'KGD2002', 'Korea_2000', 'Central_Belt_2010')
    )


def exterior_rings_from_geometry(geom):
    """폴리곤 외곽 링만 좌표열로. 구멍은 버린다."""
    if geom is None or getattr(geom, 'is_empty', True):
        return []
    rings = []
    if geom.geom_type == 'Polygon':
        coords = list(geom.exterior.coords)
        if len(coords) >= 2:
            rings.append([(float(x), float(y)) for x, y in coords])
    elif geom.geom_type == 'MultiPolygon':
        for part in geom.geoms:
            rings.extend(exterior_rings_from_geometry(part))
    elif geom.geom_type == 'LineString':
        coords = list(geom.coords)
        if len(coords) >= 2:
            rings.append([(float(x), float(y)) for x, y in coords])
    elif geom.geom_type == 'MultiLineString':
        for part in geom.geoms:
            rings.extend(exterior_rings_from_geometry(part))
    return rings


def load_mudflat_exterior_lines(path):
    """갯벌 shapefile 외곽만 EPSG:5186 선으로 읽는다. 구멍 제외.

    원본 UTM-K 이면 변환한다. 클립본(5186)은 그대로.
    """
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f'갯벌 shapefile 이 없습니다: {path}')
    if shapely_shape is None:
        raise RuntimeError('shapely 가 필요합니다')
    prj_path = os.path.splitext(path)[0] + '.prj'
    prj_text = ''
    if os.path.isfile(prj_path):
        with open(prj_path, encoding='utf-8', errors='replace') as prj_file:
            prj_text = prj_file.read()
    transformer = None
    if not _prj_looks_5186(prj_text):
        try:
            import pyproj
        except ImportError as exc:
            raise RuntimeError('UTM-K 변환에 pyproj 가 필요합니다') from exc
        transformer = pyproj.Transformer.from_crs(
            _UTMK, 'EPSG:5186', always_xy=True)

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

    lines = []
    for geom in geoms:
        for ring in exterior_rings_from_geometry(geom):
            if transformer is None:
                lines.append(ring)
                continue
            converted = []
            for x_val, y_val in ring:
                east, north = transformer.transform(x_val, y_val)
                converted.append((float(east), float(north)))
            if len(converted) >= 2:
                lines.append(converted)
    return lines


def seaward_boundary_lines(rings, coast_geoms, min_dist_m=150.0):
    """해안에서 min_dist 이상 떨어진 외곽 구간만 남긴다.

    반대편 해안에 붙는 변은 버리고, 만 안 갯벌 바깥 변만 L 로 쓴다.
    """
    if not rings or not coast_geoms:
        return []
    if ShapelySTRtree is None or ShapelyPoint is None:
        return list(rings)
    try:
        min_dist = float(min_dist_m)
    except (TypeError, ValueError):
        min_dist = 150.0
    tree = ShapelySTRtree(list(coast_geoms))
    kept = []
    for ring in rings:
        if not ring or len(ring) < 2:
            continue
        current = []
        for start, end in zip(ring, ring[1:]):
            try:
                mx = 0.5 * (float(start[0]) + float(end[0]))
                my = 0.5 * (float(start[1]) + float(end[1]))
            except (TypeError, ValueError, IndexError):
                continue
            nearest = resolve_strtree_nearest(
                tree.nearest(ShapelyPoint(mx, my)), coast_geoms)
            if nearest is None:
                dist = None
            else:
                dist = nearest.distance(ShapelyPoint(mx, my))
            if dist is not None and dist >= min_dist:
                if not current:
                    current = [start, end]
                else:
                    current.append(end)
            elif len(current) >= 2:
                kept.append(current)
                current = []
        if len(current) >= 2:
            kept.append(current)
    return kept


def water_keepout_polygon(
        waterline_xy, seaward_xy, length_m, half_width_m):
    """수위선 W에서 바다 쪽으로 뻗는 30×30 상자."""
    if waterline_xy is None:
        return None
    try:
        wx, wy = float(waterline_xy[0]), float(waterline_xy[1])
        sx, sy = float(seaward_xy[0]), float(seaward_xy[1])
    except (TypeError, ValueError, IndexError):
        return None
    return seaward_rectangle(
        wx, wy, sx, sy, length_m, half_width_m, start_offset_m=0.0)


def coast_hsl0_keepout(
        coast_xy, seaward_xy, hsl0_lines, alpha, length_m, half_width_m,
        max_dist_m=200.0):
    """해안선 점 C, 바다 방향, HSL=0 선들로 위험 상자.

    α=0 → HSL=0, α=1 → 해안선. 교차 실패면 None.
    """
    low = first_hsl0_along_ray(coast_xy, seaward_xy, hsl0_lines, max_dist_m)
    waterline = interpolate_waterline(low, coast_xy, alpha)
    if waterline is None:
        return None
    return water_keepout_polygon(waterline, seaward_xy, length_m, half_width_m)


def bbox_of_lines(lines):
    """선 목록의 AABB (minx, miny, maxx, maxy). 점이 없으면 None."""
    minx = miny = maxx = maxy = None
    for line in lines or ():
        if not line:
            continue
        for point in line:
            try:
                x_val = float(point[0])
                y_val = float(point[1])
            except (TypeError, ValueError, IndexError):
                continue
            if not (math.isfinite(x_val) and math.isfinite(y_val)):
                continue
            if minx is None:
                minx = maxx = x_val
                miny = maxy = y_val
            else:
                minx = min(minx, x_val)
                maxx = max(maxx, x_val)
                miny = min(miny, y_val)
                maxy = max(maxy, y_val)
    if minx is None:
        return None
    return (minx, miny, maxx, maxy)


def window_bbox(center_xy, radius_m):
    """중심점 주변 정사각 창. 반경이 없거나 좌표가 깨지면 None."""
    try:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        radius = float(radius_m)
    except (TypeError, ValueError, IndexError):
        return None
    if not all(math.isfinite(v) for v in (cx, cy, radius)) or radius <= 0.0:
        return None
    return (cx - radius, cy - radius, cx + radius, cy + radius)


def intersect_bboxes(first, second):
    """두 AABB의 교집합. 비거나 입력이 None이면 None."""
    if first is None or second is None:
        return None
    try:
        minx = max(float(first[0]), float(second[0]))
        miny = max(float(first[1]), float(second[1]))
        maxx = min(float(first[2]), float(second[2]))
        maxy = min(float(first[3]), float(second[3]))
    except (TypeError, ValueError, IndexError):
        return None
    if minx > maxx or miny > maxy:
        return None
    return (minx, miny, maxx, maxy)


def point_in_bbox(xy, bbox):
    """점이 AABB 안(경계 포함)이면 True."""
    if xy is None or bbox is None:
        return False
    try:
        x_val, y_val = float(xy[0]), float(xy[1])
        minx, miny, maxx, maxy = (float(v) for v in bbox)
    except (TypeError, ValueError, IndexError):
        return False
    return minx <= x_val <= maxx and miny <= y_val <= maxy


def sample_polyline(coords, interval_m):
    """폴리라인을 interval_m 간격으로 샘플한다. 끝점은 항상 포함한다."""
    try:
        step = float(interval_m)
    except (TypeError, ValueError):
        return []
    if step <= 0.0 or not math.isfinite(step):
        return []
    points = []
    for raw in coords or ():
        try:
            x_val, y_val = float(raw[0]), float(raw[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x_val) and math.isfinite(y_val):
            points.append((x_val, y_val))
    if len(points) < 2:
        return list(points)
    sampled = [points[0]]
    traveled = 0.0
    next_at = step
    for start, end in zip(points, points[1:]):
        seg_dx = end[0] - start[0]
        seg_dy = end[1] - start[1]
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len == 0.0:
            continue
        while traveled + seg_len >= next_at:
            ratio = (next_at - traveled) / seg_len
            sampled.append((
                start[0] + ratio * seg_dx,
                start[1] + ratio * seg_dy,
            ))
            next_at += step
        traveled += seg_len
    last = points[-1]
    if math.hypot(sampled[-1][0] - last[0], sampled[-1][1] - last[1]) > 1e-6:
        sampled.append(last)
    return sampled


def _dedupe_points(points, tol_m=1.0):
    """가까운 점을 앞점 기준으로 제거한다."""
    try:
        tol = float(tol_m)
    except (TypeError, ValueError):
        tol = 1.0
    unique = []
    for point in points or ():
        if not unique:
            unique.append(point)
            continue
        if any(
            math.hypot(point[0] - kept[0], point[1] - kept[1]) <= tol
            for kept in unique
        ):
            continue
        unique.append(point)
    return unique


def sample_polylines_in_bbox(polylines, bbox, interval_m):
    """창과 겹치는 폴리라인만 샘플한다. 전국선 전수 순회용 함수가 아니다."""
    if bbox is None:
        return []
    sampled = []
    for line in polylines or ():
        line_bbox = bbox_of_lines([line])
        if intersect_bboxes(line_bbox, bbox) is None:
            continue
        for point in sample_polyline(line, interval_m):
            if point_in_bbox(point, bbox):
                sampled.append(point)
    return _dedupe_points(sampled, tol_m=max(0.5, float(interval_m) * 0.25))


def seaward_hsl0_hit(coast_xy, hint_xy, lines, max_dist_m=200.0):
    """힌트 방향, 실패하면 반대로 HSL=0 교차를 찾는다.

    반환: (L, 사용한 방향) 또는 (None, None).
    """
    try:
        hx, hy = float(hint_xy[0]), float(hint_xy[1])
    except (TypeError, ValueError, IndexError):
        return None, None
    if not (math.isfinite(hx) and math.isfinite(hy)):
        return None, None
    for direction in ((hx, hy), (-hx, -hy)):
        hit = first_hsl0_along_ray(coast_xy, direction, lines, max_dist_m)
        if hit is not None:
            return hit, direction
    return None, None


def waterline_ray_pairs(
        coast_samples, hint_seaward_xy, hsl0_lines, alpha,
        max_dist_m=200.0, min_span_m=0.05):
    """각 C에서 L·W를 만든다. 교차 실패·영두께(α≈0)는 버린다."""
    try:
        min_span = float(min_span_m)
    except (TypeError, ValueError):
        min_span = 0.05
    pairs = []
    for coast_xy in coast_samples or ():
        low_xy, _used = seaward_hsl0_hit(
            coast_xy, hint_seaward_xy, hsl0_lines, max_dist_m)
        if low_xy is None:
            continue
        water_xy = interpolate_waterline(low_xy, coast_xy, alpha)
        if water_xy is None:
            continue
        span = math.hypot(water_xy[0] - low_xy[0], water_xy[1] - low_xy[1])
        if span < min_span:
            continue
        pairs.append({'c': coast_xy, 'l': low_xy, 'w': water_xy})
    return pairs


def sort_pairs_along_shore(pairs, origin_xy, tangent_xy):
    """해안 접선 투영으로 쌍을 정렬한다. 자기교차 띠를 줄이기 위함."""
    try:
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        tx, ty = float(tangent_xy[0]), float(tangent_xy[1])
    except (TypeError, ValueError, IndexError):
        return list(pairs or ())
    if not all(math.isfinite(v) for v in (ox, oy, tx, ty)):
        return list(pairs or ())
    if tx == 0.0 and ty == 0.0:
        return list(pairs or ())

    def _key(pair):
        coast = pair.get('c') or (ox, oy)
        try:
            cx, cy = float(coast[0]), float(coast[1])
        except (TypeError, ValueError, IndexError):
            return 0.0
        return (cx - ox) * tx + (cy - oy) * ty

    return sorted(pairs or (), key=_key)


def waterline_keepout_strip(pairs):
    """W를 이은 뒤 L을 반대로 닫아 바다쪽(W→L) 띠를 만든다.

    점 2개 미만이면 None. 사각형 폴백 없음.
    """
    if not pairs or len(pairs) < 2:
        return None
    waters = [pair['w'] for pair in pairs]
    lows = [pair['l'] for pair in pairs]
    polygon = list(waters) + list(reversed(lows))
    if len(polygon) < 3:
        return None
    return polygon


def waterline_keepout_from_samples(
        coast_samples, hint_seaward_xy, hsl0_lines, alpha,
        origin_xy=None, max_dist_m=200.0):
    """샘플 C_i → W_i/L_i → 연안 정렬 → W→L 띠.

    α=0 근처는 두께 0이라 빈 값. 해안선 C를 별도 윤곽으로 넣지 않는다.
    """
    pairs = waterline_ray_pairs(
        coast_samples, hint_seaward_xy, hsl0_lines, alpha,
        max_dist_m=max_dist_m)
    if len(pairs) < 2:
        return None
    origin = origin_xy if origin_xy is not None else pairs[0]['c']
    try:
        hx, hy = float(hint_seaward_xy[0]), float(hint_seaward_xy[1])
    except (TypeError, ValueError, IndexError):
        return None
    tangent = (-hy, hx)
    ordered = sort_pairs_along_shore(pairs, origin, tangent)
    return waterline_keepout_strip(ordered)


def _xy_pair(raw):
    """좌표 한 점을 (x, y) float 로. 깨지면 None."""
    try:
        x_val, y_val = float(raw[0]), float(raw[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not (math.isfinite(x_val) and math.isfinite(y_val)):
        return None
    return (x_val, y_val)


def _unit_xy(vx, vy):
    length = math.hypot(vx, vy)
    if length == 0.0 or not math.isfinite(length):
        return None
    return (vx / length, vy / length)


def _clean_polyline(coords):
    """중복·비유한 점을 뺀 폴리라인."""
    points = []
    for raw in coords or ():
        xy = _xy_pair(raw)
        if xy is None:
            continue
        if points and math.hypot(
                xy[0] - points[-1][0], xy[1] - points[-1][1]) < 1e-9:
            continue
        points.append(xy)
    return points


def _polyline_length(points):
    total = 0.0
    for start, end in zip(points, points[1:]):
        total += math.hypot(end[0] - start[0], end[1] - start[1])
    return total


def _is_closed_polyline(points, tol_m=1.0):
    if not points or len(points) < 3:
        return False
    return math.hypot(
        points[0][0] - points[-1][0],
        points[0][1] - points[-1][1],
    ) <= float(tol_m)


def is_short_closed_ring(coords, max_length_m=100.0, tol_m=1.0):
    """시작≈끝이고 둘레가 max_length 미만이면 True. 긴 닫힌 본안은 False."""
    points = _clean_polyline(coords)
    if not _is_closed_polyline(points, tol_m):
        return False
    try:
        limit = float(max_length_m)
    except (TypeError, ValueError):
        limit = 100.0
    return _polyline_length(points) < limit


def _point_to_segment(px, py, ax, ay, bx, by):
    """점 P에서 선분 AB 위 최근접. (qx, qy, t, dist)."""
    dx = bx - ax
    dy = by - ay
    length2 = dx * dx + dy * dy
    if length2 == 0.0:
        return ax, ay, 0.0, math.hypot(px - ax, py - ay)
    t_val = ((px - ax) * dx + (py - ay) * dy) / length2
    t_val = max(0.0, min(1.0, t_val))
    qx = ax + t_val * dx
    qy = ay + t_val * dy
    return qx, qy, t_val, math.hypot(px - qx, py - qy)


def nearest_on_polyline(points, origin_xy):
    """한 폴리라인에서 origin 최근접. (seg_idx, qxy, dist) 또는 None."""
    origin = _xy_pair(origin_xy)
    if origin is None or not points or len(points) < 2:
        return None
    best = None
    for seg_idx, (start, end) in enumerate(zip(points, points[1:])):
        qx, qy, _t_val, dist = _point_to_segment(
            origin[0], origin[1], start[0], start[1], end[0], end[1])
        if best is None or dist < best[0]:
            best = (dist, seg_idx, (qx, qy))
    if best is None:
        return None
    return best[1], best[2], best[0]


def select_shore_polyline(
        polylines, origin_xy, min_closed_length_m=100.0):
    """짧은 닫힌 고리(바위 윤곽)를 버리고 P에 가장 가까운 본안을 고른다.

    둘레 < min_closed_length_m 이고 시작≈끝인 조각은 원이 되므로 제외.
    """
    origin = _xy_pair(origin_xy)
    if origin is None:
        return None
    best = None
    for raw in polylines or ():
        points = _clean_polyline(raw)
        if len(points) < 2:
            continue
        if is_short_closed_ring(points, min_closed_length_m):
            continue
        nearest = nearest_on_polyline(points, origin)
        if nearest is None:
            continue
        _seg, qxy, dist = nearest
        if best is None or dist < best[0]:
            best = (dist, points, qxy)
    if best is None:
        return None
    return {'points': best[1], 'c': best[2], 'dist': best[0]}


def walk_one_polyline(
        points, origin_xy, half_length_m=80.0, interval_m=10.0):
    """고른 한 선에서만 C 양쪽 half_length 를 걷는다. 다른 조각과 잇지 않는다.

    같은 선이 긴 닫힌 고리면 끝점에서 감아 같은 선을 이어 걷는다.
    """
    cleaned = _clean_polyline(points)
    origin = _xy_pair(origin_xy)
    try:
        half = float(half_length_m)
        step = float(interval_m)
    except (TypeError, ValueError):
        return []
    if origin is None or len(cleaned) < 2 or half <= 0.0 or step <= 0.0:
        return []
    closed = _is_closed_polyline(cleaned)
    ring = cleaned[:-1] if closed and len(cleaned) > 2 else cleaned
    nearest = nearest_on_polyline(
        ring + [ring[0]] if closed else ring, origin)
    if nearest is None:
        return []
    seg_idx, qxy, _dist = nearest
    n_pts = len(ring)

    def _walk(direction):
        collected = []
        remaining = half
        x_val, y_val = qxy
        if closed:
            # direction +1: toward ring[seg_idx+1], -1: toward ring[seg_idx]
            if direction > 0:
                next_idx = (seg_idx + 1) % n_pts
            else:
                next_idx = seg_idx
        else:
            if direction > 0:
                targets = list(range(seg_idx + 1, n_pts))
            else:
                targets = list(range(seg_idx, -1, -1))
            for target_idx in targets:
                tx, ty = ring[target_idx]
                dx = tx - x_val
                dy = ty - y_val
                seg_len = math.hypot(dx, dy)
                if seg_len < 1e-12:
                    continue
                if seg_len >= remaining:
                    collected.append((
                        x_val + dx / seg_len * remaining,
                        y_val + dy / seg_len * remaining,
                    ))
                    return collected
                collected.append((tx, ty))
                remaining -= seg_len
                x_val, y_val = tx, ty
            return collected
        steps = 0
        while remaining > 1e-6 and steps < n_pts:
            tx, ty = ring[next_idx]
            dx = tx - x_val
            dy = ty - y_val
            seg_len = math.hypot(dx, dy)
            steps += 1
            if seg_len < 1e-12:
                next_idx = (next_idx + direction) % n_pts
                continue
            if seg_len >= remaining:
                collected.append((
                    x_val + dx / seg_len * remaining,
                    y_val + dy / seg_len * remaining,
                ))
                break
            collected.append((tx, ty))
            remaining -= seg_len
            x_val, y_val = tx, ty
            next_idx = (next_idx + direction) % n_pts
        return collected

    backward = _walk(-1)
    forward = _walk(1)
    chain = list(reversed(backward)) + [qxy] + forward
    return sample_polyline(chain, step)


def _tangent_at(points, index):
    """폴리라인 점의 로컬 접선 단위벡터."""
    if not points or len(points) < 2:
        return None
    if index <= 0:
        start, end = points[0], points[1]
    elif index >= len(points) - 1:
        start, end = points[-2], points[-1]
    else:
        start, end = points[index - 1], points[index + 1]
    return _unit_xy(end[0] - start[0], end[1] - start[1])


def local_hsl0_hit(coast_xy, tangent_xy, hsl0_lines, max_dist_m=80.0):
    """접선 수직 양쪽에서 바깥 HSL=0(마지막 교차) 중 더 긴 쪽."""
    tangent = _xy_pair(tangent_xy)
    coast = _xy_pair(coast_xy)
    if tangent is None or coast is None:
        return None, None
    unit = _unit_xy(tangent[0], tangent[1])
    if unit is None:
        return None, None
    best = None
    best_hit = None
    best_dir = None
    for direction in ((-unit[1], unit[0]), (unit[1], -unit[0])):
        hit = last_hsl0_along_ray(
            coast, direction, hsl0_lines, max_dist_m)
        if hit is None:
            continue
        dist = math.hypot(hit[0] - coast[0], hit[1] - coast[1])
        if best is None or dist > best:
            best = dist
            best_hit = hit
            best_dir = direction
    if best_dir is None:
        return None, None
    return best_hit, best_dir


def _fixed_seaward_xy(origin_xy, coast_xy):
    """P−C. 길이가 너무 짧으면 None (해안 위에 있을 때)."""
    origin = _xy_pair(origin_xy)
    coast = _xy_pair(coast_xy)
    if origin is None or coast is None:
        return None
    dx = origin[0] - coast[0]
    dy = origin[1] - coast[1]
    if not (math.isfinite(dx) and math.isfinite(dy)):
        return None
    if math.hypot(dx, dy) < 1e-3:
        return None
    return (dx, dy)


def _closed_mudflat_polygons(lines):
    """닫힌 외곽 링만 면으로. 열린 선은 버린다."""
    if ShapelyPolygon is None or ShapelyPoint is None:
        return []
    polys = []
    for line in lines or ():
        if not line or len(line) < 4:
            continue
        if line[0] != line[-1]:
            continue
        try:
            poly = ShapelyPolygon(line)
        except (TypeError, ValueError):
            continue
        if poly.is_empty or poly.area <= 0.0:
            continue
        if not poly.is_valid:
            poly = poly.buffer(0)
        if getattr(poly, 'geom_type', '') == 'Polygon' and poly.area > 0.0:
            polys.append(poly)
    return polys


def seaward_into_mudflat(origin_xy, coast_xy, lines, probe_m=20.0):
    """C에서 갯벌 면 안으로 가는 방향.

    가까운 교차가 육지인 경우가 있어 P−C와 반대 중
    면 안에 들어가는 쪽만 쓴다.
    """
    hint = _fixed_seaward_xy(origin_xy, coast_xy)
    coast = _xy_pair(coast_xy)
    if hint is None or coast is None:
        return hint
    length = math.hypot(hint[0], hint[1])
    if length < 1e-3:
        return None
    try:
        probe = float(probe_m)
    except (TypeError, ValueError):
        probe = 20.0
    if probe <= 0.0:
        probe = 20.0
    ux, uy = hint[0] / length, hint[1] / length
    polys = _closed_mudflat_polygons(lines)
    if not polys:
        return hint
    for vec in ((ux, uy), (-ux, -uy)):
        test = ShapelyPoint(coast[0] + vec[0] * probe, coast[1] + vec[1] * probe)
        if any(poly.contains(test) for poly in polys):
            return vec
    return hint


def nearest_point_on_lines(origin_xy, lines):
    """선 목록에서 origin 에 가장 가까운 점."""
    origin = _xy_pair(origin_xy)
    if origin is None or ShapelyLineString is None or ShapelyPoint is None:
        return None
    ox, oy = origin
    best = None
    best_d = None
    query = ShapelyPoint(ox, oy)
    for line in lines or ():
        if not line or len(line) < 2:
            continue
        try:
            geom = ShapelyLineString(line)
        except (TypeError, ValueError):
            continue
        if geom.length <= 0.0:
            continue
        point = geom.interpolate(geom.project(query))
        dist = math.hypot(point.x - ox, point.y - oy)
        if best_d is None or dist < best_d:
            best_d = dist
            best = (float(point.x), float(point.y))
    return best


def pair_seaward_point(
        coast_xy, seaward_xy, lines, max_dist_m, min_dist_m=0.0):
    """C에서 넘긴 방향으로 바깥 변과 처음 만나는 점.

    방향은 갯벌 면 안(seaward_into_mudflat)이어야 한다.
    가까운 반대편(육지)을 고르지 않는다.
    """
    return first_hsl0_along_ray(
        coast_xy, seaward_xy, lines,
        max_dist_m=max_dist_m, min_dist_m=min_dist_m)


def waterline_segments_along_coast(
        coast_points, hsl0_lines, alpha, max_dist_m=80.0, min_span_m=0.05,
        alpha_min=0.08, seaward_xy=None, min_dist_m=0.0):
    """해안점을 따라 W·L 을 뽑는다. 실패·너무 먼 점은 구간을 끊는다.

    L 은 갯벌 면 안 방향의 첫 교차(min_dist 이후).
    가까운 육지 쪽을 고르지 않는다.
    """
    try:
        min_span = float(min_span_m)
    except (TypeError, ValueError):
        min_span = 0.05
    draw_alpha = keepout_alpha(alpha, alpha_min)
    fixed = _xy_pair(seaward_xy)
    if fixed is not None and math.hypot(fixed[0], fixed[1]) < 1e-3:
        fixed = None
    segments = []
    current = []
    for index, coast_xy in enumerate(coast_points or ()):
        low_xy = pair_seaward_point(
            coast_xy, fixed, hsl0_lines, max_dist_m, min_dist_m=min_dist_m)
        water_xy = interpolate_waterline(low_xy, coast_xy, draw_alpha)
        span = None
        if water_xy is not None and low_xy is not None:
            span = math.hypot(
                water_xy[0] - low_xy[0], water_xy[1] - low_xy[1])
        if water_xy is None or span is None or span < min_span:
            if current:
                segments.append(current)
                current = []
            continue
        current.append({
            'c': coast_xy,
            'l': low_xy,
            'w': water_xy,
            'width': span,
        })
    if current:
        segments.append(current)
    return [seg for seg in segments if len(seg) >= 2]


def _median_width(values):
    ordered = sorted(float(v) for v in values if v is not None)
    if not ordered:
        return None
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _seaward_buffer_sign(w_points, l_points):
    """W 진행 기준 L이 왼쪽이면 +1, 오른쪽이면 -1."""
    if not w_points or len(w_points) < 2 or not l_points:
        return 1.0
    index = min(len(w_points) // 2, len(w_points) - 1)
    if index <= 0:
        prev_pt, next_pt = w_points[0], w_points[1]
        w_pt = w_points[0]
    elif index >= len(w_points) - 1:
        prev_pt, next_pt = w_points[-2], w_points[-1]
        w_pt = w_points[-1]
    else:
        prev_pt, next_pt = w_points[index - 1], w_points[index + 1]
        w_pt = w_points[index]
    tangent = _unit_xy(next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1])
    if tangent is None:
        return 1.0
    l_pt = l_points[min(index, len(l_points) - 1)]
    to_low = (l_pt[0] - w_pt[0], l_pt[1] - w_pt[1])
    left = (-tangent[1], tangent[0])
    if left[0] * to_low[0] + left[1] * to_low[1] >= 0.0:
        return 1.0
    return -1.0


def _largest_simple_ring(geom):
    """단순 폴리곤의 외곽 링. 자기교차면 None."""
    if geom is None or getattr(geom, 'is_empty', True):
        return None
    if shapely_unary_union is None:
        return None
    work = geom
    if not work.is_valid:
        work = shapely_unary_union(work)
    if work.is_empty:
        return None
    if work.geom_type == 'MultiPolygon':
        work = max(work.geoms, key=lambda item: item.area)
    elif work.geom_type == 'GeometryCollection':
        polys = [
            item for item in work.geoms if item.geom_type == 'Polygon']
        if not polys:
            return None
        work = max(polys, key=lambda item: item.area)
    if work.geom_type != 'Polygon' or work.is_empty or work.area <= 0.0:
        return None
    if not work.is_simple or not work.is_valid:
        work = shapely_unary_union(work)
        if work.geom_type == 'MultiPolygon':
            work = max(work.geoms, key=lambda item: item.area)
        if (
            work.geom_type != 'Polygon'
            or not work.is_simple
            or not work.is_valid
        ):
            return None
    coords = list(work.exterior.coords)
    if len(coords) >= 2 and coords[0] == coords[-1]:
        coords = coords[:-1]
    if len(coords) < 3:
        return None
    return [(float(x_val), float(y_val)) for x_val, y_val in coords]


def waterline_offset_polygon(segment):
    """W와 L 사이를 면으로 닫는다. buffer 하지 않는다."""
    return waterline_wl_polygon(segment)


def waterline_wl_polygon(segment):
    """한 구간의 W와 L 사이. 연속 사다리꼴을 합친다. buffer 없음."""
    if (
        ShapelyPolygon is None
        or shapely_unary_union is None
        or not segment
        or len(segment) < 2
    ):
        return None
    quads = []
    for start, end in zip(segment, segment[1:]):
        ring = [start['w'], end['w'], end['l'], start['l'], start['w']]
        try:
            poly = ShapelyPolygon(ring)
        except (TypeError, ValueError):
            continue
        if poly.is_empty:
            continue
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            continue
        quads.append(poly)
    if not quads:
        return None
    if len(quads) == 1:
        return _largest_simple_ring(quads[0])
    return _largest_simple_ring(shapely_unary_union(quads))


def waterline_keepout_from_coast(
        polylines, origin_xy, hsl0_lines, alpha,
        half_length_m=80.0, interval_m=10.0,
        max_dist_m=80.0, min_closed_length_m=100.0,
        min_span_m=0.05, alpha_min=0.08, min_dist_m=0.0):
    """본안 해안을 따라 W~L 사이만 이동 불가로 만든다.

    L 은 갯벌 면 안 방향의 첫 교차(min_dist 이후).
    가까운 육지 쪽이나 단측 buffer 로 폭을 키우지 않는다.
    """
    shore = select_shore_polyline(
        polylines, origin_xy, min_closed_length_m)
    if shore is None:
        return None
    coast = walk_one_polyline(
        shore['points'], shore['c'], half_length_m, interval_m)
    if len(coast) < 2:
        return None
    seaward_xy = seaward_into_mudflat(origin_xy, shore['c'], hsl0_lines)
    segments = waterline_segments_along_coast(
        coast, hsl0_lines, alpha, max_dist_m, min_span_m,
        alpha_min=alpha_min, seaward_xy=seaward_xy,
        min_dist_m=min_dist_m)
    if not segments:
        return None
    rings = []
    for segment in segments:
        ring = waterline_wl_polygon(segment)
        if ring is not None:
            rings.append(ring)
    if not rings:
        return None
    if len(rings) == 1:
        return rings[0]
    if ShapelyPolygon is None or shapely_unary_union is None:
        return None
    polys = []
    for ring in rings:
        poly = ShapelyPolygon(ring)
        if poly.is_valid and not poly.is_empty:
            polys.append(poly)
    if not polys:
        return None
    return _largest_simple_ring(shapely_unary_union(polys))


def empty_activity_window(session_start, threshold_used=0.0):
    """유효 캐시가 없을 때 쓰는 빈 활동 윈도우."""
    return {
        'session_start': session_start,
        'threshold_mode': 'absolute',
        'threshold_used': float(threshold_used),
        'start_tide_level_m': None,
        'crossing_time': None,
        'retreat_decision_time': None,
        'window_end': None,
        'high_tides': [],
        'low_tides': [],
        'safe_all_day': False,
    }


def ensure_operational_cache(config, now, fetch_fn=None, station=None):
    """당일(+다음날) 캐시를 로드하거나, 없으면 fetch_fn으로 받아 저장한다.

    HTTP는 fetch_fn이 할 일. 기본은 fetch_tide_series.
    오늘 날짜를 커버하지 않는 기존 파일은 쓰지 않는다.
    반환: {cache, path, fetched, ok, reason}
    """
    path, date_from, date_to, station = operational_cache_path(config, now, station)
    if os.path.isfile(path):
        cache = load_cache(path)
        points = series_to_points(cache['series'])
        if cache_covers_when(points, now):
            return {
                'cache': cache,
                'path': path,
                'fetched': False,
                'ok': True,
                'reason': 'existing_covers_today',
            }
    fetch_fn = fetch_fn or fetch_tide_series
    fetch_config = dict(config)
    fetch_config['station_code'] = station
    series = fetch_fn(fetch_config, date_from, date_to)
    saved = save_cache(series, config, station, date_from, date_to)
    cache = load_cache(saved)
    points = series_to_points(cache['series'])
    if not cache_covers_when(points, now):
        return {
            'cache': None,
            'path': saved,
            'fetched': True,
            'ok': False,
            'reason': 'fetch_did_not_cover_today',
        }
    return {
        'cache': cache,
        'path': saved,
        'fetched': True,
        'ok': True,
        'reason': 'fetched',
    }


def compute_window(series, config, session_start):
    """조위 시계열과 임계 설정으로 활동 윈도우를 계산해 dict로 반환한다.

    반환 키: session_start, threshold_mode, threshold_used, start_tide_level_m,
    crossing_time, retreat_decision_time, window_end, high_tides, low_tides, safe_all_day.
    """
    points = series_to_points(series)
    if not points:
        raise ValueError('조위 시계열이 비어 있어 활동 윈도우를 계산할 수 없습니다.')

    threshold_cfg = config.get('threshold') or {}
    mode = str(threshold_cfg.get('mode', 'absolute'))
    # session_start 시점 조위는 보간 없이 가장 가까운 시각의 샘플로 선택한다.
    start_level = find_nearest(points, session_start)[1]
    if mode == 'absolute':
        threshold = float(threshold_cfg.get('tide_level_m', 0.0))
    elif mode == 'rise_from_session_start':
        threshold = start_level + float(threshold_cfg.get('rise_m', 0.0))
    else:
        raise ValueError(f'알 수 없는 threshold.mode입니다: {mode!r} (absolute | rise_from_session_start)')

    crossing_time = _find_crossing(points, session_start, threshold, start_level)
    margin = timedelta(minutes=float(config.get('safety_margin_min', 30)))
    retreat_decision_time = None if crossing_time is None else crossing_time - margin
    window_end = retreat_decision_time if retreat_decision_time is not None else points[-1][0]
    high_tides, low_tides = _local_extrema(points)

    return {
        'session_start': session_start,
        'threshold_mode': mode,
        'threshold_used': threshold,
        'start_tide_level_m': start_level,
        'crossing_time': crossing_time,
        'retreat_decision_time': retreat_decision_time,
        'window_end': window_end,
        'high_tides': high_tides,
        'low_tides': low_tides,
        'safe_all_day': crossing_time is None,
    }


def format_plan(result):
    """compute_window 결과를 사람이 읽는 한국어 요약 문자열로 변환한다."""
    lines = ['=== TARS 조석 활동 윈도우 ===']
    lines.append(f"세션 시작: {_format_time(result['session_start'])}")

    mode = result['threshold_mode']
    detail = f"{result['threshold_used']:.2f} m ({mode}"
    if mode == 'rise_from_session_start':
        detail += f", 시작 조위 {result['start_tide_level_m']:.2f} m 기준 상승량 포함)"
    else:
        detail += ')'
    lines.append(f'임계 기준: {detail}')

    if result['safe_all_day']:
        lines.append(f"예보 구간 내 임계 초과 없음 — 만료 시각까지 안전: {_format_time(result['window_end'])}")
    else:
        lines.append(f"활동 권고 구간: {_format_time(result['session_start'])} ~ {_format_time(result['window_end'])}")
        lines.append(f"임계 초과 예상 시각: {_format_time(result['crossing_time'])}")
        lines.append(f"철수 판단 시각: {_format_time(result['retreat_decision_time'])}")

    for label, entries in (('만조', result['high_tides']), ('간조', result['low_tides'])):
        if entries:
            items = ', '.join(
                f"{_format_time(entry['time'])} ({entry['tide_level_m']:.2f} m)" for entry in entries
            )
            lines.append(f'{label}: {items}')
        else:
            lines.append(f'{label}: 예보 구간 내 검출 없음')
    return '\n'.join(lines)


def series_to_points(series):
    """정규화 시계열을 (datetime, float) 튜플 리스트로 변환해 시각순 정렬한다."""
    points = []
    for index, entry in enumerate(series):
        try:
            when = _parse_time(entry['time'])
            level = float(entry['tide_level_m'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'조위 시계열 항목 #{index}를 파싱할 수 없습니다: {entry!r}') from exc
        if when is None:
            raise ValueError(f'조위 시계열 항목 #{index}의 시각 형식을 인식할 수 없습니다: {entry!r}')
        points.append((when, level))
    points.sort(key=lambda point: point[0])
    return points


def find_nearest(points, when):
    """시각순 (datetime, float) 포인트 중 when에 가장 가까운 포인트를 반환한다."""
    return min(points, key=lambda point: abs((point[0] - when).total_seconds()))


def rise_rate_m_per_hour(samples, at_dt, window_min=60):
    """at_dt ± window_min 구간의 조위 상승률(m/h)을 최소제곱 직선 기울기로 추정한다.

    대칭 창의 최소제곱 기울기는 중앙차분의 일반화다(선형 시계열에서 정확).
    samples: [(datetime, level_m), ...] — 정렬되어 있지 않으면 정렬부터 한다.
    데이터 부족(창 안에 서로 다른 시각 2개 미만)이면 None을 반환한다.
    """
    if not samples:
        return None
    ordered = sorted(samples, key=lambda sample: sample[0])
    window = timedelta(minutes=float(window_min))
    windowed = [(t, lvl) for t, lvl in ordered if abs((t - at_dt).total_seconds()) <= window.total_seconds()]
    if len(windowed) < 2 or windowed[0][0] == windowed[-1][0]:
        return None

    t0 = windowed[0][0]
    xs = [(t - t0).total_seconds() / 3600.0 for t, _ in windowed]
    ys = [lvl for _, lvl in windowed]
    count = len(xs)
    mean_x = sum(xs) / count
    mean_y = sum(ys) / count
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom == 0.0:
        return None
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom


def dynamic_retreat_level(ground_elevation_m, safety_margin_m, distance_m,
                          robot_speed_mps, time_buffer_min, rise_rate):
    """동적 철수 공식을 계산해 결과 dict를 반환한다.

    H_danger = ground - margin, t_return = distance / speed,
    lead = r × (t_return + buffer), H_retreat = H_danger - lead.
    rise_rate가 0 이하(하강/평탄 조위)면 lead는 0으로 클램프한다.
    """
    if float(robot_speed_mps) <= 0.0:
        raise ValueError(f'robot_speed_mps는 0보다 커야 합니다: {robot_speed_mps}')
    danger_level = float(ground_elevation_m) - float(safety_margin_m)
    t_return_s = float(distance_m) / float(robot_speed_mps)
    lead_hours = t_return_s / 3600.0 + float(time_buffer_min) / 60.0
    lead_m = max(0.0, float(rise_rate)) * lead_hours
    return {
        'ground_elevation_m': float(ground_elevation_m),
        'safety_margin_m': float(safety_margin_m),
        'danger_level_m': danger_level,
        'retreat_distance_m': float(distance_m),
        'robot_speed_mps': float(robot_speed_mps),
        'time_buffer_min': float(time_buffer_min),
        'rise_rate_m_per_hour': float(rise_rate),
        't_return_s': t_return_s,
        'lead_m': lead_m,
        'retreat_level_m': danger_level - lead_m,
    }


def crossing_time(samples, level_m, direction='rising', after_dt=None):
    """조위가 level_m을 지정 방향으로 처음 통과하는 시각을 선형 보간으로 반환한다.

    after_dt가 있으면 그 시각 이후의 첫 통과만 찾는다(출발 계획 용도).
    시리즈 밖(시작부터 이미 초과 포함) 교차는 None이다.
    """
    if direction not in ('rising', 'falling'):
        raise ValueError(f'알 수 없는 direction입니다: {direction!r} (rising | falling)')
    ordered = sorted(samples, key=lambda sample: sample[0])
    for (t0, l0), (t1, l1) in zip(ordered, ordered[1:]):
        if direction == 'rising':
            crossed = l0 < level_m <= l1
        else:
            crossed = l0 > level_m >= l1
        if not crossed:
            continue
        span_s = (t1 - t0).total_seconds()
        when = t1 if span_s <= 0.0 else t0 + timedelta(seconds=span_s * (level_m - l0) / (l1 - l0))
        if after_dt is not None and when < after_dt:
            continue
        return when
    return None


def plan_dynamic_retreat(config, points, session_start, ground_override=None, distance_override=None):
    """동적 철수 적용 가능 여부를 판정하고 가능하면 철수 조위·교차 시각을 계산한다.

    반환 dict: enabled(bool), reason(str|None), retreat(결과 dict|None),
    retreat_crossing(철수 조위 상승 통과 시각|None), danger_crossing(위험 조위 도달 시각|None).
    """
    result = {'enabled': False, 'reason': None, 'retreat': None,
              'retreat_crossing': None, 'danger_crossing': None}
    retreat_cfg = config.get('retreat') or {}
    if str(retreat_cfg.get('mode', 'fixed')) != 'dynamic':
        result['reason'] = 'retreat.mode가 dynamic이 아님'
        return result
    ground = ground_override if ground_override is not None else retreat_cfg.get('ground_elevation_m')
    if ground is None:
        result['reason'] = '지상고 캘리브레이션 전'
        return result
    distance = distance_override if distance_override is not None else retreat_cfg.get('retreat_distance_m')
    if distance is None:
        result['reason'] = '복귀 거리(retreat_distance_m) 미설정'
        return result

    rate = rise_rate_m_per_hour(points, session_start)
    if rate is None:
        result['reason'] = '상승률 계산 데이터 부족'
        return result

    retreat = dynamic_retreat_level(
        ground,
        retreat_cfg.get('safety_margin_m', 0.3),
        distance,
        retreat_cfg.get('robot_avg_speed_mps', 0.3),
        retreat_cfg.get('time_buffer_min', 10),
        rate,
    )
    result['enabled'] = True
    result['retreat'] = retreat
    # "최신 출발 시각"은 세션 시작 이후 첫 상승 통과여야 한다.
    result['retreat_crossing'] = crossing_time(points, retreat['retreat_level_m'],
                                               direction='rising', after_dt=session_start)
    result['danger_crossing'] = crossing_time(points, retreat['danger_level_m'],
                                              direction='rising', after_dt=session_start)
    return result


def format_dynamic_retreat(info):
    """plan_dynamic_retreat 결과를 한국어 요약 문자열로 변환한다."""
    retreat = info['retreat']
    lines = ['=== 동적 철수 (dynamic retreat) ===']
    lines.append(
        f"지상고 {retreat['ground_elevation_m']:.2f} m - 안전마진 {retreat['safety_margin_m']:.2f} m"
        f" → 위험 조위 {retreat['danger_level_m']:.2f} m"
    )
    lines.append(
        f"상승률 {retreat['rise_rate_m_per_hour']:.3f} m/h | 복귀 {retreat['retreat_distance_m']:.0f} m"
        f" ÷ {retreat['robot_speed_mps']:.2f} m/s → 복귀 소요 {retreat['t_return_s']:.0f} s"
        f" ({retreat['t_return_s'] / 60.0:.1f} 분)"
    )
    lines.append(
        f"상승 리드 {retreat['lead_m']:.3f} m (철수+버퍼 시간)"
        f" → 철수 조위 {retreat['retreat_level_m']:.3f} m"
    )
    if info['retreat_crossing'] is not None:
        lines.append(f"최신 출발 시각: {_format_time(info['retreat_crossing'])} (철수 조위 상승 통과)")
    else:
        lines.append('최신 출발 시각: 예보 구간 내 철수 조위 미도달')
    if info['danger_crossing'] is not None:
        lines.append(f"위험 도달 시각: {_format_time(info['danger_crossing'])} (위험 조위 상승 통과)")
    else:
        lines.append('위험 도달 시각: 예보 구간 내 위험 조위 미도달')
    return '\n'.join(lines)


def clamp_retreat_speed_mps(avg_mps, max_mps=None):
    """복귀 속도를 플랫폼 상한으로 자른다."""
    avg = float(avg_mps)
    if avg <= 0.0 or not math.isfinite(avg):
        raise ValueError(f'robot_speed_mps는 0보다 커야 합니다: {avg_mps}')
    if max_mps is None:
        cap = PLATFORM_MAX_SPEED_MPS
    else:
        try:
            cap = float(max_mps)
        except (TypeError, ValueError):
            cap = PLATFORM_MAX_SPEED_MPS
    if not math.isfinite(cap) or cap <= 0.0:
        return avg
    return min(avg, cap)


def wet_progress(dist_to_keepout_m, dist_to_coast_m):
    """조수선 성장 좌표. keepout(바다쪽)=0, 해안선=1."""
    try:
        keepout_d = max(0.0, float(dist_to_keepout_m))
        coast_d = max(0.0, float(dist_to_coast_m))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(keepout_d) and math.isfinite(coast_d)):
        return None
    span = keepout_d + coast_d
    if span <= 0.0:
        return None
    return keepout_d / span


def local_mud_span_m(dist_to_keepout_m, dist_to_coast_m, fallback_m=None):
    """그 단면의 C↔keepout 폭. 둘 다 있으면 합, 없으면 폴백."""
    try:
        keepout_d = float(dist_to_keepout_m)
        coast_d = float(dist_to_coast_m)
    except (TypeError, ValueError):
        keepout_d = None
        coast_d = None
    if keepout_d is not None and coast_d is not None:
        if math.isfinite(keepout_d) and math.isfinite(coast_d):
            span = max(0.0, keepout_d) + max(0.0, coast_d)
            if span > 0.0:
                return span
    if fallback_m is None:
        return None
    try:
        fallback = float(fallback_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(fallback) or fallback <= 0.0:
        return None
    return fallback


def dry_width_m(span_m, alpha, spatial_margin_m=0.0):
    """지금 C에서 바다쪽으로 아직 마른 폭."""
    try:
        width = (1.0 - float(alpha)) * float(span_m) - float(spatial_margin_m)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(width):
        return 0.0
    return max(0.0, width)


def water_inland_speed_mps(span_m, rise_rate_m_per_hour, t_low, t_high):
    """조수선이 C 쪽으로 자라는 속도(m/s). 하강·평탄이면 0."""
    try:
        level_span = float(t_high) - float(t_low)
        width = float(span_m)
        rate = float(rise_rate_m_per_hour)
    except (TypeError, ValueError):
        return 0.0
    if level_span <= 0.0 or width <= 0.0 or rate <= 0.0:
        return 0.0
    if not all(math.isfinite(v) for v in (level_span, width, rate)):
        return 0.0
    return (width * rate / level_span) / 3600.0


def accessible_from_coast_m(
        span_m, alpha, robot_speed_mps, water_speed_mps,
        time_buffer_min, spatial_margin_m=0.0):
    """지금 C에서 나가도 육지로 돌아올 수 있는 최대 거리."""
    dry = dry_width_m(span_m, alpha, spatial_margin_m)
    speed = float(robot_speed_mps)
    if speed <= 0.0:
        raise ValueError(
            f'robot_speed_mps는 0보다 커야 합니다: {robot_speed_mps}')
    water = max(0.0, float(water_speed_mps))
    buffer_s = float(time_buffer_min) * 60.0
    if water <= 0.0:
        return dry
    numer = dry - buffer_s * water
    if numer <= 0.0:
        return 0.0
    return numer / (1.0 + water / speed)


def alpha_at_accessible(
        span_m, x_m, robot_speed_mps, water_speed_mps,
        time_buffer_min, spatial_margin_m=0.0):
    """accessible_from_coast_m 이 x_m 이 되는 alpha."""
    try:
        span = float(span_m)
        target = max(0.0, float(x_m))
        margin = float(spatial_margin_m)
        water = max(0.0, float(water_speed_mps))
        buffer_s = float(time_buffer_min) * 60.0
        speed = float(robot_speed_mps)
    except (TypeError, ValueError):
        return None
    if span <= 0.0 or speed <= 0.0 or not math.isfinite(span):
        return None
    need = target * (1.0 + water / speed) + margin + buffer_s * water
    return max(0.0, min(1.0, 1.0 - (need / span)))


def tide_level_from_alpha(alpha, t_low, t_high):
    """alpha → 조위(m)."""
    if alpha is None or t_low is None or t_high is None:
        return None
    try:
        return float(t_low) + float(alpha) * (float(t_high) - float(t_low))
    except (TypeError, ValueError):
        return None


def empty_tide_access(reason='unknown'):
    """GPS/캐시가 없을 때 쓰는 접근 스냅샷."""
    return {
        'phase': 'unknown',
        'reason': reason,
        'should_leave': False,
        'on_mud': False,
        'has_fix': False,
        'span_m': None,
        'alpha': None,
        'wet_progress': None,
        'dry_width_m': None,
        'accessible_from_coast_m': None,
        'distance_to_exit_m': None,
        'remaining_dry_m': None,
        'seconds_to_exit': None,
        'seconds_to_retreat': None,
        'enter_time': None,
        'retreat_decision_time': None,
        'enter_level_m': None,
        'retreat_level_m': None,
        'robot_speed_mps': None,
        'water_inland_mps': None,
        'tide_level_m': None,
        't_low': None,
        't_high': None,
    }


def plan_tide_access(
        points, when,
        span_m,
        dist_to_exit_m,
        on_mud,
        robot_speed_mps,
        time_buffer_min=10.0,
        spatial_margin_m=20.0,
        min_work_m=15.0,
        dist_to_keepout_m=None,
        dist_to_coast_m=None,
        has_fix=True,
        max_speed_mps=None):
    """keepout→C 조수선 성장으로 접근 범위와 철수 시각을 계산한다.

    띠를 옮기지 않는다. 점의 wet_progress = d_K/(d_K+d_C) 가 현재
    alpha 이상이면 아직 마른 쪽이다. 복귀 거리는 gps_home(또는 C).
    """
    info = empty_tide_access('ok')
    info['on_mud'] = bool(on_mud)
    info['has_fix'] = bool(has_fix)
    try:
        speed = clamp_retreat_speed_mps(robot_speed_mps, max_speed_mps)
    except ValueError:
        info['reason'] = 'speed_invalid'
        return info
    info['robot_speed_mps'] = speed

    if not points or when is None:
        info['reason'] = 'no_cache'
        return info

    t_low, t_high = tide_range_today(points, when)
    level = interpolate_tide(points, when)
    alpha = tide_alpha(level, t_low, t_high)
    info['t_low'] = t_low
    info['t_high'] = t_high
    info['tide_level_m'] = level
    info['alpha'] = alpha
    if level is None or t_low is None or t_high is None:
        info['reason'] = 'no_cache'
        return info

    span = local_mud_span_m(dist_to_keepout_m, dist_to_coast_m, span_m)
    if span is None:
        info['reason'] = 'no_span'
        return info
    info['span_m'] = span
    info['wet_progress'] = wet_progress(dist_to_keepout_m, dist_to_coast_m)

    try:
        exit_m = max(0.0, float(dist_to_exit_m))
    except (TypeError, ValueError):
        exit_m = 0.0
    if not math.isfinite(exit_m):
        exit_m = 0.0
    info['distance_to_exit_m'] = exit_m

    rate = rise_rate_m_per_hour(points, when)
    if rate is None:
        rate = 0.0
    water = water_inland_speed_mps(span, rate, t_low, t_high)
    info['water_inland_mps'] = water
    info['dry_width_m'] = dry_width_m(span, alpha, spatial_margin_m)
    xmax = accessible_from_coast_m(
        span, alpha, speed, water, time_buffer_min, spatial_margin_m)
    info['accessible_from_coast_m'] = xmax
    robot_x = exit_m if on_mud else 0.0
    info['remaining_dry_m'] = max(0.0, info['dry_width_m'] - robot_x)
    info['seconds_to_exit'] = exit_m / speed

    leave_alpha = alpha_at_accessible(
        span, robot_x, speed, water, time_buffer_min, spatial_margin_m)
    enter_alpha = alpha_at_accessible(
        span, float(min_work_m), speed, water, time_buffer_min,
        spatial_margin_m)
    info['retreat_level_m'] = tide_level_from_alpha(leave_alpha, t_low, t_high)
    info['enter_level_m'] = tide_level_from_alpha(enter_alpha, t_low, t_high)
    if info['retreat_level_m'] is not None:
        info['retreat_decision_time'] = crossing_time(
            points, info['retreat_level_m'],
            direction='rising', after_dt=when)
        if level >= info['retreat_level_m'] and (
                info['retreat_decision_time'] is None
                or info['retreat_decision_time'] > when):
            info['retreat_decision_time'] = when
    if info['enter_level_m'] is not None:
        info['enter_time'] = crossing_time(
            points, info['enter_level_m'],
            direction='falling', after_dt=when)

    if info['retreat_decision_time'] is not None:
        info['seconds_to_retreat'] = (
            info['retreat_decision_time'] - when).total_seconds()

    if not has_fix:
        info['phase'] = 'unknown'
        info['reason'] = 'no_gps'
        info['should_leave'] = False
        return info

    try:
        work_min = float(min_work_m)
    except (TypeError, ValueError):
        work_min = 15.0
    flooded = info['dry_width_m'] <= 0.0
    past_xmax = bool(on_mud) and exit_m >= xmax
    wet_now = (
        info['wet_progress'] is not None
        and alpha >= info['wet_progress']
        and bool(on_mud)
    )
    if flooded:
        info['phase'] = 'flooded'
        info['should_leave'] = bool(on_mud)
        info['reason'] = 'flooded'
    elif past_xmax or wet_now:
        info['phase'] = 'leave_now'
        info['should_leave'] = True
        info['reason'] = 'past_xmax' if past_xmax else 'wet'
    elif xmax < work_min and on_mud:
        info['phase'] = 'leave_now'
        info['should_leave'] = True
        info['reason'] = 'past_xmax'
    elif xmax < work_min:
        info['phase'] = 'wait_ebb'
        info['should_leave'] = False
        info['reason'] = 'wait_ebb'
    else:
        info['phase'] = 'accessible'
        info['should_leave'] = False
        info['reason'] = 'ok'

    if (
        info['retreat_decision_time'] is not None
        and when >= info['retreat_decision_time']
        and on_mud
        and not info['should_leave']
    ):
        info['phase'] = 'leave_now'
        info['should_leave'] = True
        info['reason'] = 'retreat_time'
    return info


def earlier_retreat_time(access_time, ground_time):
    """공간 모델과 지상고 모델 시각 중 이른 쪽. 둘 다 없으면 None."""
    times = [item for item in (access_time, ground_time) if item is not None]
    if not times:
        return None
    return min(times)


def format_tide_access(info):
    """plan_tide_access 결과를 한국어 요약으로."""
    lines = ['=== 조수선 성장 접근 범위 ===']
    phase = info.get('phase') or 'unknown'
    reason = info.get('reason') or ''
    lines.append(f"단계: {phase} ({reason})")
    if info.get('span_m') is not None:
        lines.append(
            f"단면 폭 {info['span_m']:.0f} m | "
            f"마른 폭 {info.get('dry_width_m') or 0.0:.0f} m | "
            f"접근 {info.get('accessible_from_coast_m') or 0.0:.0f} m"
        )
    if info.get('distance_to_exit_m') is not None:
        lines.append(
            f"복귀 거리 {info['distance_to_exit_m']:.0f} m"
            f" (속도 {info.get('robot_speed_mps') or 0.0:.3f} m/s)"
        )
    if info.get('retreat_decision_time') is not None:
        lines.append(
            f"철수 시각: {_format_time(info['retreat_decision_time'])}"
        )
    else:
        lines.append('철수 시각: 예보 구간 내 없음')
    if info.get('enter_time') is not None:
        lines.append(f"진입 시각: {_format_time(info['enter_time'])}")
    return '\n'.join(lines)


def _find_crossing(points, session_start, threshold, start_level):
    """임계 최초 초과 시각을 찾는다(세션 시작 이후만 대상)."""
    if start_level >= threshold:
        return session_start  # 세션 시작 시점에 이미 임계 이상이면 즉시 철수 판단 대상
    for when, level in points:
        if when >= session_start and level >= threshold:
            return when
    return None


def _local_extrema(points):
    """만조/간조 시각·조위를 (high_tides, low_tides) dict 리스트로 반환한다."""

    # 연속해서 같은 조위는 하나로 묶어 평탄 구간이 극값 판정을 깨는 것을 방지한다.
    reduced = []
    for when, level in points:
        if reduced and reduced[-1][1] == level:
            continue
        reduced.append((when, level))

    highs, lows = [], []
    for index in range(1, len(reduced) - 1):
        prev_level = reduced[index - 1][1]
        when, level = reduced[index]
        next_level = reduced[index + 1][1]
        entry = {'time': when, 'tide_level_m': level}
        if level > prev_level and level > next_level:
            highs.append(entry)
        elif level < prev_level and level < next_level:
            lows.append(entry)
    return highs, lows


def _response_root(raw):
    """header/body를 담은 최상위 dict를 반환한다.

    활용가이드의 XML 예제는 response 래퍼가 있지만, 실측 JSON 응답은 래퍼 없이
    {'header': ..., 'body': ...}로 올 수 있어 둘 다 처리한다.
    """
    if isinstance(raw, dict) and isinstance(raw.get('response'), dict):
        return raw['response']
    if isinstance(raw, dict):
        return raw
    return {}


def _find_items(raw):
    """확정 명세 경로(body.items.item)에서 항목을 찾는다.

    item은 리스트가 일반적이지만 1개뿐이면 dict로 올 수 있어 둘 다 처리한다.
    명세 경로에서 못 찾으면 구조 변경에 버티도록 일반 탐색으로 폴백한다.
    """
    root = _response_root(raw)
    body = root.get('body')
    items = body.get('items') if isinstance(body, dict) else None
    item = items.get('item') if isinstance(items, dict) else None
    if isinstance(item, dict):
        return [item]
    if isinstance(item, list):
        return item
    return _find_item_list(raw)


def _find_item_list(raw):
    """원시 응답에서 시계열 항목 리스트를 일반 탐색으로 찾는다(폴백 전용)."""
    if isinstance(raw, list):
        return raw
    queue = [raw]
    while queue:
        node = queue.pop(0)
        if isinstance(node, list):
            if not node or all(isinstance(item, dict) for item in node):
                return node
            continue
        if not isinstance(node, dict):
            continue
        item = node.get('item')
        if isinstance(item, list):
            return item
        if isinstance(item, dict):
            return [item]
        for key in ('items', 'records', 'list', 'data'):
            if isinstance(node.get(key), (list, dict)):
                queue.append(node[key])
        for key in ('response', 'body', 'result', 'data'):
            if isinstance(node.get(key), (dict, list)):
                queue.append(node[key])
    return []


def _parse_time(text):
    """지원 형식 목록으로 시각 문자열을 파싱한다(실패 시 None)."""
    for time_format in _TIME_FORMATS:
        try:
            return datetime.strptime(text.strip(), time_format)
        except ValueError:
            continue
    return None


def _format_time(value):
    """datetime을 YYYY-MM-DD HH:MM 문자열로 변환한다."""
    if value is None:
        return '없음'
    return value.strftime('%Y-%m-%d %H:%M')


def _validate_date_arg(text, option_name):
    """CLI 날짜 인자를 YYYY-MM-DD로 검증해 정규화한다."""
    try:
        return datetime.strptime(text, '%Y-%m-%d').strftime('%Y-%m-%d')
    except ValueError as exc:
        raise ValueError(f'{option_name}은 YYYY-MM-DD 형식이어야 합니다: {text!r}') from exc


def _parse_session_start(text):
    """--session-start 인자를 파싱한다."""
    parsed = _parse_time(text)
    if parsed is None:
        raise ValueError(f'--session-start 형식이 잘못되었습니다 (YYYY-MM-DD HH:MM): {text!r}')
    return parsed


def _cmd_fetch(args, config):
    station = args.station or config.get('station_code') or 'TBD'
    date_from = _validate_date_arg(args.date_from, '--date-from')
    date_to = _validate_date_arg(args.date_to, '--date-to')
    fetch_config = dict(config)
    fetch_config['station_code'] = station

    series = fetch_tide_series(fetch_config, date_from, date_to)
    cache_path = save_cache(series, config, station, date_from, date_to)
    print(f'캐시 저장: {cache_path}')

    result = compute_window(series, config, datetime.now())
    print(format_plan(result))
    return 0


def _cmd_plan(args, config):
    session_start = _parse_session_start(args.session_start) if args.session_start else datetime.now()
    cache = load_cache(args.cache)
    result = compute_window(cache['series'], config, session_start)
    print(format_plan(result))

    # 동적 철수 정보(확정 지상고 + 상승률 계산 가능 시). 기존 고정 임계 출력 뒤에 붙인다.
    points = series_to_points(cache['series'])
    dyn_info = plan_dynamic_retreat(
        config, points, session_start,
        ground_override=args.ground_elevation, distance_override=args.distance_m,
    )
    if dyn_info['enabled']:
        print(format_dynamic_retreat(dyn_info))
    elif str((config.get('retreat') or {}).get('mode', 'fixed')) == 'dynamic':
        print(f"동적 철수 비활성: {dyn_info['reason']} — 조수선 성장 모델로 계산함")

    retreat_cfg = config.get('retreat') or {}
    access = plan_tide_access(
        points, session_start,
        span_m=args.mud_width if args.mud_width is not None else retreat_cfg.get(
            'fallback_mud_width_m', 400.0),
        dist_to_exit_m=(
            args.distance_m if args.distance_m is not None else 0.0),
        on_mud=bool(args.on_mud),
        robot_speed_mps=retreat_cfg.get('robot_avg_speed_mps', 0.3),
        time_buffer_min=retreat_cfg.get('time_buffer_min', 10),
        spatial_margin_m=retreat_cfg.get('spatial_margin_m', 20.0),
        min_work_m=retreat_cfg.get('min_work_m', 15.0),
        has_fix=True,
        max_speed_mps=retreat_cfg.get('robot_max_speed_mps'),
    )
    if dyn_info['enabled']:
        access['retreat_decision_time'] = earlier_retreat_time(
            access.get('retreat_decision_time'),
            dyn_info.get('retreat_crossing'),
        )
    print(format_tide_access(access))
    return 0


def _build_parser():
    # --config는 메인 파서와 서브커맨드 어디에 와도 동작하도록 parent 파서로 공유한다.
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        '--config',
        default=None,
        help=f'설정 파일 경로 (기본: {DEFAULT_CONFIG_PATH}, 없으면 내장 기본값)',
    )
    parser = argparse.ArgumentParser(
        description='TARS 조석 활동 윈도우 계획 도구 (data.go.kr 조석예보)',
        parents=[config_parser],
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    fetch_parser = subparsers.add_parser(
        'fetch', parents=[config_parser], help='조석예보 조회 → 캐시 저장 → 윈도우 계산',
    )
    fetch_parser.add_argument('--date-from', required=True, help='조회 시작일 YYYY-MM-DD')
    fetch_parser.add_argument('--date-to', required=True, help='조회 종료일 YYYY-MM-DD')
    fetch_parser.add_argument('--station', default=None, help='예보지점 코드 (설정 station_code 덮어쓰기)')

    plan_parser = subparsers.add_parser('plan', parents=[config_parser], help='캐시 파일로 오프라인 윈도우 계산')
    plan_parser.add_argument('--cache', required=True, help='조위 캐시 JSON 경로')
    plan_parser.add_argument(
        '--session-start',
        default=None,
        help='세션 시작 시각 "YYYY-MM-DD HH:MM" (기본: 현재)',
    )
    plan_parser.add_argument(
        '--ground-elevation', type=float, default=None,
        help='지상고(m) 오버라이드 (retreat.ground_elevation_m)',
    )
    plan_parser.add_argument(
        '--distance-m', type=float, default=None,
        help='복귀 거리(m). gps_home 또는 C까지. 공간 모델·지상고 경로 공통',
    )
    plan_parser.add_argument(
        '--mud-width', type=float, default=None,
        help='C↔keepout 단면 폭(m). 없으면 retreat.fallback_mud_width_m',
    )
    plan_parser.add_argument(
        '--on-mud', action='store_true',
        help='로봇이 갯벌 위에 있다고 가정 (기본: 육지)',
    )
    return parser


def main(argv=None):
    """CLI 진입점."""
    args = _build_parser().parse_args(argv)
    try:
        config = resolve_config(args.config)
        if args.command == 'fetch':
            return _cmd_fetch(args, config)
        return _cmd_plan(args, config)
    except (FileNotFoundError, OSError, ValueError, RuntimeError) as exc:
        print(f'오류: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
