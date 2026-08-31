"""캐시된 조석예보 시계열 기반으로 조석 상태를 감시하고 철수 결정을 발행하는 노드."""

import json
import math
import os
import threading
import time
from datetime import datetime

import pyproj
import rclpy
import shapefile as pyshp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener
from shapely.geometry import Point as ShapelyPoint, LineString, box as ShapelyBox
from shapely.ops import nearest_points
from shapely.strtree import STRtree
from std_msgs.msg import Bool, String

from eclipse_pkg.tide_plan import (
    apply_tide_clock_offset,
    apply_tide_clock_speed,
    compute_window,
    crossing_time,
    dynamic_retreat_level,
    earlier_retreat_time,
    empty_activity_window,
    empty_tide_access,
    ensure_operational_cache,
    interpolate_tide,
    intersect_bboxes,
    load_cache,
    plan_tide_access,
    resolve_config,
    resolve_strtree_nearest,
    resolve_strtree_query,
    resolve_writable_cache_dir,
    rise_rate_m_per_hour,
    sample_polylines_in_bbox,
    series_to_points,
    cache_covers_when,
    tide_alpha,
    tide_range_today,
    waterline_sweep_alpha,
    window_bbox,
    wrap_time_to_points,
    yaw_from_direction,
    yaw_to_quat_zw,
)
from eclipse_pkg.tide_waterline import (
    DEFAULT_ALONG_HALF_M,
    DEFAULT_EDGE_INSET_M,
    DEFAULT_EDGE_INSET_RATIO,
    DEFAULT_RADIUS_M,
    DEFAULT_STATION_M,
    bbox_of_polygons,
    clip_geoms_to_window,
    filter_keepout_rings,
    keepout_grow_span_m,
    load_keepout_geojson,
    load_waterline_steps,
    resolve_waterline_steps_path,
    waterline_rings_at_alpha,
    load_mudflat_polygons,
    point_on_mudflat,
    resolve_keepout_geojson_path,
    rings_5186_to_map,
    window_from_rings,
)
from eclipse_pkg.tide_waterline_tiles import (
    baked_keepout_tiles,
    baked_tiles,
    load_tile_index,
    lookup_tile_sticky,
    tile_keepout_path,
    tile_steps_path,
)


class TideWatchNode(Node):
    """조석 상태를 발행하고 수위선 마커를 TideLayer 에 넣는다."""

    def __init__(self):
        super().__init__('tide_watch_node')

        self.declare_parameter('cache_file', '')
        self.declare_parameter('cache_dir', '')
        self.declare_parameter('auto_fetch', True)
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate_hz', 0.1)
        self.declare_parameter('polygon_margin_m', 4.0)  # TideLayer 마진 (inflation_radius 0.5 + inscribed 0.4 ≈ 0.9m → 최종 ~5m)
        self.declare_parameter('enable_gps_station_select', True)
        self.declare_parameter('waterline_window_radius_m', DEFAULT_RADIUS_M)
        self.declare_parameter('waterline_edge_inset_m', DEFAULT_EDGE_INSET_M)
        self.declare_parameter(
            'waterline_edge_inset_ratio', DEFAULT_EDGE_INSET_RATIO)
        self.declare_parameter('waterline_half_length_m', DEFAULT_ALONG_HALF_M)
        self.declare_parameter('waterline_station_m', DEFAULT_STATION_M)
        # 실시간 last-inside 는 만 채움 면에서 수분. 기본 끔.
        self.declare_parameter('waterline_live_compute', False)
        # 오프라인 후보 A. 있으면 실시간 계산 대신 이 링을 TideLayer 에 넣는다.
        self.declare_parameter('keepout_geojson', '')
        self.declare_parameter('keepout_site', '')
        self.declare_parameter('coastline_shapefile', '')
        self.declare_parameter('mudflat_shapefile', '')
        self.declare_parameter('sea_heading_topic', '/tide/sea_heading')
        self.declare_parameter('heading_publish_rate_hz', 0.2)
        # 기본은 실 GPS. 시뮬은 /gps/fix_tide_sim — /gps/fix 에 가짜를 넣으면 EKF가 오염된다.
        self.declare_parameter('gps_topic', '/gps/fix')
        # 조석 조회 시각만 이동. 시스템 시계·use_sim_time·GPS·철수는 그대로.
        self.declare_parameter('tide_clock_offset_hours', 0.0)
        # 1=벽시계. >1 이면 예보 곡선을 그 배속으로 재생(미리보기). 실기동은 1.
        self.declare_parameter('tide_clock_speed', 1.0)
        self.declare_parameter('station_code', '')
        # <0 이면 예보 α. 미리보기에서 조위 API 없이 성장 띠를 볼 때 0~1.
        self.declare_parameter('waterline_alpha_override', -1.0)
        # >0 이면 로컬 미리보기: period 초 동안 0→1, 같은 시간 1→0. 실기동은 0.
        self.declare_parameter('waterline_alpha_sweep_s', 0.0)
        self.declare_parameter('waterline_steps_json', '')
        # 있으면 GPS로 칸 JSON을 고른다. 비우면 waterline_steps_json 한 장.
        self.declare_parameter('waterline_tiles_dir', '')
        # 수위선과 같은 12 km 격자. GPS가 keepout.geojson 1칸만 읽는다.
        self.declare_parameter('keepout_tiles_dir', '')

        config_file = str(self.get_parameter('config_file').value or '')
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn('publish_rate_hz must be > 0. Using 0.1')
            publish_rate_hz = 0.1

        try:
            self.ops_config = resolve_config(config_file or None)
        except (OSError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f'tide_watch 설정 로드 실패: {exc}')
            raise SystemExit(1)

        preferred_dir = str(self.get_parameter('cache_dir').value or '')
        resolved_dir = resolve_writable_cache_dir(
            preferred_dir, self.ops_config.get('cache_dir') or 'datasets/tide_cache')
        self.ops_config = dict(self.ops_config)
        self.ops_config['cache_dir'] = resolved_dir
        station_override = str(
            self.get_parameter('station_code').value or '').strip()
        if station_override:
            self.ops_config['station_code'] = station_override
            self.get_logger().info(f'조위 관측소 지정: {station_override}')

        self._cache_lock = threading.Lock()
        self._fetch_in_progress = False
        self._cache_path = ''
        self._cache_ok = False
        self._cache_reason = 'uninitialized'
        self._ops_date = None
        self.points = []
        self.dynamic_info = None
        self.retreat_sent = False
        self._access = empty_tide_access('uninitialized')
        self.window = empty_activity_window(datetime.now())
        self._tide_interp = {
            'tide_level_m': None,
            't_low': None,
            't_high': None,
            'alpha': 0.0,
        }
        self._clock_t0_wall = datetime.now()
        self._clock_t0_mono = time.monotonic()
        self._load_or_fetch(self._clock_t0_wall)

        self.status_topic = str(self.ops_config.get('status_topic') or '/mission/tide_status')
        self.retreat_topic = str(self.ops_config.get('retreat_topic') or '/mission/retreat')
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        retreat_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.retreat_pub = self.create_publisher(
            Bool, self.retreat_topic, retreat_qos)
        self.retreat_sent = False

        # GPS 구독 (관측소 자동 선택 + 해안선 조회). EKF 입력과 분리 가능.
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.gps_topic = str(self.get_parameter('gps_topic').value or '/gps/fix')
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._gps_callback, gps_qos)
        self._current_lat = None
        self._current_lon = None
        self._current_station_code = self.ops_config.get('station_code', 'DT_0068')
        self._home_x = None
        self._home_y = None
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.create_subscription(
            PoseStamped, '/mission/home_pose', self._home_pose_callback, home_qos)

        # map 자세는 /odometry/filtered 를 매 메시지 역직렬화하지 않는다.
        # 띠·바다방향 타이머에서만 TF 를 조회한다.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._has_map_pose = False
        self._sea_dir_cache = None
        self._sea_dir_lat = None
        self._sea_dir_lon = None

        # 해안선 shapefile 로드 → 공간 인덱스 구축
        coastline_path = str(self.get_parameter('coastline_shapefile').value or '')
        self._coastline = None
        self._coastline_index = None
        self._gps_to_5186 = pyproj.Transformer.from_crs(
            'EPSG:4326', 'EPSG:5186', always_xy=True)
        if coastline_path and os.path.isfile(coastline_path):
            try:
                sf = pyshp.Reader(coastline_path)
                self._coastline = [LineString(shape.points) for shape in sf.shapes()]
                sf.close()
                self._coastline_index = STRtree(self._coastline)
                self.get_logger().info(
                    f'해안선 로드 완료: {len(self._coastline)} segments')
            except Exception as exc:
                self.get_logger().warn(f'해안선 로드 실패: {exc}')
        else:
            self.get_logger().warn(
                'coastline_shapefile 미설정 — 바다 방향 불가, 해측 띠는 비움')

        mudflat_path = str(self.get_parameter('mudflat_shapefile').value or '')
        if mudflat_path and not os.path.isfile(mudflat_path):
            mudflat_path = ''
        if not mudflat_path:
            for candidate in (
                    os.path.join('datasets', 'tidflt', 'tidflt_jebu_5186.shp'),
                    os.path.join('datasets', 'tidflt', 'tidflt_gomso_5186.shp'),
                    '/home/lee/WGIS_TIDFLT.shp'):
                if os.path.isfile(candidate):
                    mudflat_path = candidate
                    break
        self._mudflat_polys = []
        self._mudflat_bbox = None
        if mudflat_path:
            try:
                self._mudflat_polys = load_mudflat_polygons(mudflat_path)
                self._mudflat_bbox = bbox_of_polygons(self._mudflat_polys)
                self.get_logger().info(
                    f'갯벌 면: {len(self._mudflat_polys)} polys '
                    f'({mudflat_path})')
            except Exception as exc:
                self.get_logger().warn(f'갯벌 로드 실패: {exc}')
        else:
            self.get_logger().warn(
                'mudflat_shapefile 미설정 — 해측 띠는 비움')

        self._baked_rings_5186 = []
        fallback_grow = float(
            (self.ops_config.get('retreat') or {}).get(
                'fallback_mud_width_m', 400.0))
        keepout_path = resolve_keepout_geojson_path(
            str(self.get_parameter('keepout_geojson').value or ''),
            site=str(self.get_parameter('keepout_site').value or ''))
        if keepout_path:
            try:
                self._baked_rings_5186 = filter_keepout_rings(
                    load_keepout_geojson(keepout_path))
                self.get_logger().info(
                    f'구운 keepout: {len(self._baked_rings_5186)} rings '
                    f'({keepout_path})')
            except (OSError, ValueError, TypeError) as exc:
                self.get_logger().error(f'keepout geojson 로드 실패: {exc}')
        else:
            self.get_logger().warn('keepout_geojson 없음 — 해측 띠는 라이브/빈 값')

        origin, radius = window_from_rings(
            self._baked_rings_5186, fallback_grow)
        coast_for_span = []
        self._mud_for_grow = list(self._mudflat_polys)
        if origin is not None and radius is not None:
            if self._coastline:
                coast_for_span = clip_geoms_to_window(
                    self._coastline, origin, radius)
            if self._mudflat_polys:
                self._mud_for_grow = clip_geoms_to_window(
                    self._mudflat_polys, origin, radius)
        self._coast_for_front = coast_for_span
        self._grow_m = keepout_grow_span_m(
            self._baked_rings_5186, coast_for_span, fallback_m=fallback_grow)
        self._wl_alpha_key = None
        self._wl_rings_5186 = []
        self._waterline_ring_count = 0
        self._sweep_t0 = time.monotonic()
        self._wl_steps = []
        self._wl_tiles = []
        self._wl_tiles_dir = ''
        self._wl_tile_id = None
        steps_path = resolve_waterline_steps_path(
            str(self.get_parameter('waterline_steps_json').value or ''),
            site=str(self.get_parameter('keepout_site').value or ''))
        if steps_path:
            try:
                self._wl_steps = load_waterline_steps(steps_path)
                self.get_logger().info(
                    f'수위선 스텝 {len(self._wl_steps)}개 ({steps_path})')
            except (OSError, ValueError, TypeError) as exc:
                self.get_logger().error(f'수위선 스텝 로드 실패: {exc}')
        else:
            self.get_logger().warn(
                'waterline steps 없음 — 수위선 마커는 비움. '
                'scripts/bake_tide_waterline.py 로 구워라')
        self._ko_tiles_dir = ''
        self._ko_tiles = []
        self._ko_tile_id = None
        self._load_waterline_tiles()
        self._load_keepout_tiles()
        self.get_logger().info(
            f'수위선 grow_m={self._grow_m:.1f} m '
            f'(fallback={fallback_grow:.1f}, coast={len(coast_for_span)}, '
            f'mud={len(self._mud_for_grow)})')
        if not coast_for_span:
            self.get_logger().warn(
                '해안선이 창에 없음 — grow_m 이 fallback 이라 '
                '만조에도 C 까지 안 찰 수 있다')

        # TideLayer 입력은 /tide/water_polygon_markers.
        # 예전 갯벌 테두리 keepout 대신 구운 수위선(시간에 따라 α).
        water_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.water_markers_pub = self.create_publisher(
            MarkerArray, '/tide/water_polygon_markers', water_qos)
        self.waterline_markers_pub = self.create_publisher(
            MarkerArray, '/tide/waterline_markers', water_qos)

        # Foxglove-only 바다 방향 (PoseWithCovarianceStamped, /imu/mag_heading 과 동일)
        heading_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1)
        self.sea_heading_topic = str(
            self.get_parameter('sea_heading_topic').value or '/tide/sea_heading')
        self.sea_heading_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.sea_heading_topic, heading_qos)

        retreat_at = self.window['retreat_decision_time']
        self.get_logger().info(
            f'tide_watch ready: cache_ok={self._cache_ok} reason={self._cache_reason} '
            f'path={self._cache_path} retreat_decision_time={retreat_at}, '
            f'status={self.status_topic}, retreat={self.retreat_topic}, '
            f'sea_heading={self.sea_heading_topic}, gps={self.gps_topic}'
        )
        self.create_timer(1.0 / publish_rate_hz, self._timer_callback)
        heading_hz = float(self.get_parameter('heading_publish_rate_hz').value)
        if heading_hz <= 0.0:
            self.get_logger().warn('heading_publish_rate_hz must be > 0. Using 0.2')
            heading_hz = 0.2
        self.create_timer(1.0 / heading_hz, self._publish_sea_heading)

    def _apply_cache(self, cache, now, path, reason):
        """유효 캐시로 points/window를 갈아끼운다."""
        self.points = series_to_points(cache['series'])
        self.dynamic_info = self._resolve_dynamic_retreat(now)
        self.window = compute_window(
            cache['series'], self._effective_config(), now)
        self._ops_date = now.date()
        self._cache_path = path
        self._cache_ok = True
        self._cache_reason = reason
        self.retreat_sent = False
        station = self.ops_config.get('station_code', '')
        n_pts = len(self.points)
        first_t = self.points[0][0] if self.points else None
        last_t = self.points[-1][0] if self.points else None
        self.get_logger().info(
            f'조위 캐시 station={station} n={n_pts} '
            f'{first_t} .. {last_t} ({reason})')

    def _clear_cache(self, now, reason):
        """오늘을 커버하지 못하면 빈 상태로 둔다. 옛 날짜 시계열을 쓰지 않는다."""
        threshold = float(
            (self.ops_config.get('threshold') or {}).get('tide_level_m', 0.0))
        self.points = []
        self.dynamic_info = None
        self.window = empty_activity_window(now, threshold)
        self._ops_date = now.date()
        self._cache_ok = False
        self._cache_reason = reason

    def _load_or_fetch(self, now):
        """명시 cache_file이 오늘을 커버하면 쓰고, 아니면 운용일 fetch.

        HTTP는 여기(기동)와 날짜 롤오버 스레드에서만. 0.1 Hz 타이머는 치지 않는다.
        """
        cache_file = str(self.get_parameter('cache_file').value or '')
        auto_fetch = bool(self.get_parameter('auto_fetch').value)
        if cache_file:
            try:
                cache = load_cache(cache_file)
                points = series_to_points(cache['series'])
                if cache_covers_when(points, now):
                    self._apply_cache(
                        cache, now, path=cache_file, reason='explicit_covers_today')
                    return
                self.get_logger().error(
                    f'cache_file이 오늘({now.date()})을 커버하지 않음: {cache_file}'
                )
            except (OSError, ValueError, RuntimeError) as exc:
                self.get_logger().error(f'cache_file 로드 실패: {exc}')
        if auto_fetch:
            try:
                result = ensure_operational_cache(self.ops_config, now)
                if result['ok']:
                    self._apply_cache(
                        result['cache'], now,
                        path=result['path'], reason=result['reason'])
                    self.get_logger().info(
                        f"운용일 캐시 {'수신' if result['fetched'] else '재사용'}: "
                        f"{result['path']}"
                    )
                    return
                self.get_logger().error(
                    f"운용일 캐시 실패: {result['reason']} path={result['path']}"
                )
                self._clear_cache(now, result['reason'])
                return
            except (OSError, ValueError, RuntimeError) as exc:
                self.get_logger().error(f'운용일 캐시 fetch 실패: {exc}')
                self._clear_cache(now, 'fetch_failed')
                return
        self._clear_cache(now, 'no_valid_cache')

    def _start_refresh(self, now):
        """날짜가 바뀌면 백그라운드에서만 fetch. 타이머 콜백을 막지 않는다."""
        if self._fetch_in_progress:
            return
        if not bool(self.get_parameter('auto_fetch').value):
            self._clear_cache(now, 'date_rolled_no_fetch')
            return
        self._fetch_in_progress = True
        thread = threading.Thread(
            target=self._refresh_worker, args=(now,), daemon=True)
        thread.start()

    def _refresh_worker(self, now):
        try:
            result = ensure_operational_cache(self.ops_config, now)
            with self._cache_lock:
                if result['ok']:
                    self._apply_cache(
                        result['cache'], now,
                        path=result['path'], reason=result['reason'])
                    self.get_logger().info(
                        f"운용일 캐시 갱신: {result['path']} ({result['reason']})"
                    )
                else:
                    self._clear_cache(now, result['reason'])
                    self.get_logger().error(
                        f"운용일 캐시 갱신 실패: {result['reason']}"
                    )
        except (OSError, ValueError, RuntimeError) as exc:
            with self._cache_lock:
                self._clear_cache(now, 'fetch_failed')
            self.get_logger().error(f'운용일 캐시 갱신 예외: {exc}')
        finally:
            self._fetch_in_progress = False

    def _gps_callback(self, msg):
        """GPS 위치를 저장하고, 켜져 있으면 최근접 관측소를 고른다."""
        from eclipse_pkg.tide_plan import find_nearest_station
        lat = msg.latitude
        lon = msg.longitude
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return
        self._current_lat = lat
        self._current_lon = lon
        if not self.get_parameter('enable_gps_station_select').value:
            return
        nearest = find_nearest_station(lat, lon)
        if nearest and nearest['code'] != self._current_station_code:
            self._current_station_code = nearest['code']
            self.get_logger().info(
                f'GPS 기반 관측소 변경: {self._current_station_code} '
                f'({nearest["name"]}, {nearest["distance_m"]}m)'
            )
            with self._cache_lock:
                self.ops_config['station_code'] = nearest['code']
            self._start_refresh(datetime.now())
        self._refresh_keepout_tile()

    def _home_pose_callback(self, msg):
        """gps_health_supervisor 가 잡은 gps_home."""
        try:
            self._home_x = float(msg.pose.position.x)
            self._home_y = float(msg.pose.position.y)
        except (TypeError, ValueError, AttributeError):
            return

    def _coast_anchor(self, lat, lon):
        """GPS → EPSG:5186 로봇점 P, 최근접 해안선점 C, 바다 방향 C-P."""
        if self._coastline_index is None or self._gps_to_5186 is None:
            return None
        if lat is None or lon is None:
            return None
        try:
            px, py = self._gps_to_5186.transform(lon, lat)
            query = ShapelyPoint(px, py)
            line = resolve_strtree_nearest(
                self._coastline_index.nearest(query), self._coastline)
            if line is None or getattr(line, 'is_empty', False):
                return None
            nearest_pt = nearest_points(query, line)[1]
            return {
                'px': float(px),
                'py': float(py),
                'cx': float(nearest_pt.x),
                'cy': float(nearest_pt.y),
                'dx': float(nearest_pt.x - px),
                'dy': float(nearest_pt.y - py),
            }
        except Exception as exc:
            self.get_logger().debug(f'해안선 방향 계산 실패: {exc}')
            return None

    def _compute_sea_direction(self, lat, lon):
        """GPS 좌표 → 가장 가까운 해안선까지의 방향 벡터 (map 근사, m)."""
        if (
            lat is not None and lon is not None
            and self._sea_dir_cache is not None
            and self._sea_dir_lat is not None
            and abs(lat - self._sea_dir_lat) < 2e-5
            and abs(lon - self._sea_dir_lon) < 2e-5
        ):
            return self._sea_dir_cache
        anchor = self._coast_anchor(lat, lon)
        if anchor is None:
            return None
        direction = (anchor['dx'], anchor['dy'])
        self._sea_dir_cache = direction
        self._sea_dir_lat = lat
        self._sea_dir_lon = lon
        return direction

    def _refresh_map_pose(self):
        """map→base_link 를 필요할 때만 조회한다."""
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException:
            return self._has_map_pose
        self._robot_x = float(transform.transform.translation.x)
        self._robot_y = float(transform.transform.translation.y)
        self._has_map_pose = True
        return True

    def _resolve_dynamic_retreat(self, session_start):
        """dynamic 모드에서 캐시 시계열로 철수 조위를 계산한다. 불가 시 None(폴백)."""
        retreat_cfg = self.ops_config.get('retreat') or {}
        if str(retreat_cfg.get('mode', 'fixed')) != 'dynamic':
            return None
        ground = retreat_cfg.get('ground_elevation_m')
        if ground is None:
            self.get_logger().warn(
                'retreat.mode=dynamic지만 ground_elevation_m가 null — 고정 임계로 폴백'
            )
            return None
        rate = rise_rate_m_per_hour(self.points, session_start)
        if rate is None:
            self.get_logger().warn('상승률 계산 데이터 부족 — 고정 임계로 폴백')
            return None
        info = dynamic_retreat_level(
            ground,
            retreat_cfg.get('safety_margin_m', 0.3),
            retreat_cfg.get('retreat_distance_m', 300),
            retreat_cfg.get('robot_avg_speed_mps', 0.6),  # 보수적 복귀 속도
            retreat_cfg.get('time_buffer_min', 10),
            rate,
        )
        self.get_logger().info(
            f"dynamic retreat: rise_rate={info['rise_rate_m_per_hour']:.3f} m/h, "
            f"danger={info['danger_level_m']:.3f} m, retreat_level={info['retreat_level_m']:.3f} m"
        )
        return info

    def _effective_config(self):
        """유효 임계 설정: 동적 철수 조위(가능 시), 아니면 기존 threshold 그대로."""
        if self.dynamic_info is None:
            return self.ops_config
        effective = dict(self.ops_config)
        threshold = dict(self.ops_config['threshold'])
        threshold['mode'] = 'absolute'
        threshold['tide_level_m'] = self.dynamic_info['retreat_level_m']
        effective['threshold'] = threshold
        return effective

    def _tide_lookup_when(self, wall_now):
        """조석 조회 시각. 배속 1 은 벽시계, 그 외는 예보 곡선 재생."""
        offset = self.get_parameter('tide_clock_offset_hours').value
        try:
            speed = float(self.get_parameter('tide_clock_speed').value or 1.0)
        except (TypeError, ValueError):
            speed = 1.0
        if not math.isfinite(speed) or speed <= 0.0:
            speed = 1.0
        if abs(speed - 1.0) < 1.0e-9:
            return apply_tide_clock_offset(wall_now, offset)
        origin = apply_tide_clock_offset(self._clock_t0_wall, offset)
        elapsed = time.monotonic() - self._clock_t0_mono
        when = apply_tide_clock_speed(origin, elapsed, speed)
        return wrap_time_to_points(when, self.points)

    def _refresh_tide_interp(self, now):
        """타이머마다 조위 보간값을 갱신한다."""
        level = interpolate_tide(self.points, now)
        t_low, t_high = tide_range_today(self.points, now)
        self._tide_interp = {
            'tide_level_m': level,
            't_low': t_low,
            't_high': t_high,
            'alpha': tide_alpha(level, t_low, t_high),
            'tide_when': now,
        }

    def _timer_callback(self):
        now = datetime.now()
        with self._cache_lock:
            if self._ops_date is not None and now.date() != self._ops_date:
                self._start_refresh(now)
            lookup_when = self._tide_lookup_when(now)
            self._refresh_tide_interp(lookup_when)
            self._access = self._compute_access(lookup_when)
            should_leave = bool(self._access.get('should_leave'))
            self._publish_water_polygon(now)
            self._publish_grown_waterline()
            status_payload = self._build_status(now)

        status_msg = String()
        status_msg.data = json.dumps(status_payload, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        if should_leave and not self.retreat_sent:
            retreat_msg = Bool()
            retreat_msg.data = True
            self.retreat_pub.publish(retreat_msg)
            self.retreat_sent = True
            self.get_logger().warn(
                f'철수: {self.retreat_topic} True '
                f"({self._access.get('reason')})"
            )
        elif (
            not should_leave
            and self.retreat_sent
            and self._access.get('phase') == 'accessible'
        ):
            retreat_msg = Bool()
            retreat_msg.data = False
            self.retreat_pub.publish(retreat_msg)
            self.retreat_sent = False
            self.get_logger().info(
                f'철수 해제: {self.retreat_topic} False (accessible)'
            )

    def _coastline_polylines_near(self, px, py, radius_m):
        """P 창 ∩ 갯벌 외곽 bbox 안 해안선 조각. 전국선은 순회하지 않는다."""
        if self._coastline_index is None or self._mudflat_bbox is None:
            return []
        clipped = intersect_bboxes(
            window_bbox((px, py), radius_m), self._mudflat_bbox)
        if clipped is None:
            return []
        try:
            hits = resolve_strtree_query(
                self._coastline_index.query(ShapelyBox(*clipped)),
                self._coastline)
        except Exception as exc:
            self.get_logger().debug(f'해안선 창 조회 실패: {exc}')
            return []
        polylines = []
        for geom in hits:
            try:
                coords = list(geom.coords)
            except (TypeError, ValueError, AttributeError):
                continue
            if len(coords) >= 2:
                polylines.append(coords)
        return polylines

    def _coastline_samples_near(self, px, py, radius_m, interval_m):
        """창 안 해안선 점 샘플. 발행 경로는 폴리라인 본안을 쓴다."""
        return sample_polylines_in_bbox(
            self._coastline_polylines_near(px, py, radius_m),
            intersect_bboxes(
                window_bbox((px, py), radius_m), self._mudflat_bbox),
            interval_m)

    def _publish_baked_keepout(self):
        """구운 5186 링을 현재 GPS·map 자세로 옮겨 발행한다."""
        self._refresh_keepout_tile()
        if not self._refresh_map_pose():
            self.get_logger().debug(
                'map→base_link 없음 — 구운 띠는 (0,0) 기준')
        if (
            self._current_lat is None or self._current_lon is None
            or self._gps_to_5186 is None
        ):
            self.get_logger().debug('구운 keepout: GPS 없음 — 띠 비움')
            self._publish_edge_markers([])
            return
        try:
            pin_x, pin_y = self._gps_to_5186.transform(
                self._current_lon, self._current_lat)
        except Exception as exc:
            self.get_logger().debug(f'구운 keepout 좌표 변환 실패: {exc}')
            self._publish_edge_markers([])
            return
        rings = rings_5186_to_map(
            self._baked_rings_5186,
            (pin_x, pin_y),
            (self._robot_x, self._robot_y),
        )
        self.get_logger().info(
            f'구운 keepout 조각 {len(rings)}개',
            throttle_duration_sec=30.0)
        self._publish_edge_markers(rings)

    def _publish_water_polygon(self, now):
        """TideLayer 입력은 수위선, 칸 밖이면 keepout 타일."""
        del now

    def _load_waterline_tiles(self):
        """구운 타일 인덱스가 있으면 GPS로 칸을 고른다. 없으면 한 장 모드."""
        tiles_dir = str(
            self.get_parameter('waterline_tiles_dir').value or '').strip()
        if not tiles_dir:
            return
        index_path = os.path.join(tiles_dir, 'index.json')
        try:
            tiles = load_tile_index(index_path)
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'수위선 타일 인덱스 실패 — 한 장 유지: {exc}')
            return
        ready = baked_tiles(tiles, tiles_dir)
        if not ready:
            self.get_logger().warn(
                f'구운 수위선 타일 없음 ({tiles_dir}) — 한 장 유지')
            return
        self._wl_tiles_dir = tiles_dir
        self._wl_tiles = ready
        self._wl_steps = []
        self._wl_tile_id = None
        self._wl_alpha_key = None
        self.get_logger().info(
            f'수위선 타일 {len(ready)}/{len(tiles)} 구움 ({tiles_dir})')

    def _refresh_waterline_tile(self):
        """GPS 칸이 바뀌면 steps JSON 을 갈아끼운다. 파일 I/O 는 여기만."""
        if not self._wl_tiles or self._gps_to_5186 is None:
            return
        if self._current_lat is None or self._current_lon is None:
            return
        try:
            east, north = self._gps_to_5186.transform(
                self._current_lon, self._current_lat)
        except Exception as exc:
            self.get_logger().debug(f'타일 좌표 변환 실패: {exc}')
            return
        tile = lookup_tile_sticky(
            self._wl_tiles, east, north, self._wl_tile_id)
        if tile is None:
            if self._wl_tile_id is not None or self._wl_steps:
                self.get_logger().warn(
                    '수위선 타일 밖 — TideLayer 마커 비움')
                self._wl_tile_id = None
                self._wl_steps = []
                self._wl_alpha_key = None
            return
        if tile.get('id') == self._wl_tile_id and self._wl_steps:
            return
        path = tile_steps_path(self._wl_tiles_dir, tile.get('id'))
        try:
            self._wl_steps = load_waterline_steps(path)
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().error(f'수위선 타일 로드 실패 {path}: {exc}')
            return
        self._wl_tile_id = tile.get('id')
        self._wl_alpha_key = None
        self.get_logger().info(
            f'수위선 타일 {self._wl_tile_id} steps={len(self._wl_steps)}')

    def _load_keepout_tiles(self):
        """구운 keepout 칸 인덱스. GPS가 1칸만 로드한다."""
        tiles_dir = str(
            self.get_parameter('keepout_tiles_dir').value or '').strip()
        if not tiles_dir:
            return
        index_path = os.path.join(tiles_dir, 'index.json')
        try:
            tiles = load_tile_index(index_path)
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'keepout 타일 인덱스 실패 — 한 장 유지: {exc}')
            return
        ready = baked_keepout_tiles(tiles, tiles_dir)
        if not ready:
            self.get_logger().warn(
                f'구운 keepout 타일 없음 ({tiles_dir}) — 한 장 유지')
            return
        self._ko_tiles_dir = tiles_dir
        self._ko_tiles = ready
        self._ko_tile_id = None
        self.get_logger().info(
            f'keepout 타일 {len(ready)}/{len(tiles)} 구움 ({tiles_dir})')

    def _refresh_keepout_tile(self):
        """GPS 칸이 바뀌면 keepout.geojson 을 갈아끼운다."""
        if not self._ko_tiles or self._gps_to_5186 is None:
            return
        if self._current_lat is None or self._current_lon is None:
            return
        try:
            east, north = self._gps_to_5186.transform(
                self._current_lon, self._current_lat)
        except Exception as exc:
            self.get_logger().debug(f'keepout 타일 좌표 변환 실패: {exc}')
            return
        tile = lookup_tile_sticky(
            self._ko_tiles, east, north, self._ko_tile_id)
        if tile is None:
            if self._ko_tile_id is not None:
                self.get_logger().warn('keepout 타일 밖 — 구운 띠 비움')
                self._ko_tile_id = None
                self._baked_rings_5186 = []
            return
        if tile.get('id') == self._ko_tile_id and self._baked_rings_5186:
            return
        path = tile_keepout_path(self._ko_tiles_dir, tile.get('id'))
        try:
            self._baked_rings_5186 = filter_keepout_rings(
                load_keepout_geojson(path))
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().error(f'keepout 타일 로드 실패 {path}: {exc}')
            return
        self._ko_tile_id = tile.get('id')
        self.get_logger().info(
            f'keepout 타일 {self._ko_tile_id} rings={len(self._baked_rings_5186)}')

    def _publish_grown_waterline(self):
        """구운 수위선을 α 로 골라 마커·TideLayer 에 넣는다."""
        self._refresh_waterline_tile()
        empty = ()
        if not self._wl_steps:
            self._waterline_ring_count = 0
            self._publish_line_markers(
                self.waterline_markers_pub, empty, 'tide_waterline',
                (1.0, 0.45, 0.08, 0.95), 10.0)
            if self._ko_tiles or self._baked_rings_5186:
                self._publish_baked_keepout()
            else:
                self._publish_edge_markers(empty)
            return
        if (
            self._current_lat is None or self._current_lon is None
            or self._gps_to_5186 is None
        ):
            self._waterline_ring_count = 0
            self._publish_line_markers(
                self.waterline_markers_pub, empty, 'tide_waterline',
                (1.0, 0.45, 0.08, 0.95), 10.0)
            self._publish_edge_markers(empty)
            return
        self._refresh_map_pose()
        try:
            pin_x, pin_y = self._gps_to_5186.transform(
                self._current_lon, self._current_lat)
        except Exception as exc:
            self.get_logger().debug(f'수위선 좌표 변환 실패: {exc}')
            self._waterline_ring_count = 0
            self._publish_line_markers(
                self.waterline_markers_pub, empty, 'tide_waterline',
                (1.0, 0.45, 0.08, 0.95), 10.0)
            self._publish_edge_markers(empty)
            return
        try:
            sweep_s = float(
                self.get_parameter('waterline_alpha_sweep_s').value or 0.0)
        except (TypeError, ValueError):
            sweep_s = 0.0
        swept = waterline_sweep_alpha(
            time.monotonic() - self._sweep_t0, sweep_s)
        if swept is not None:
            alpha = swept
        else:
            try:
                override = float(
                    self.get_parameter('waterline_alpha_override').value)
            except (TypeError, ValueError):
                override = -1.0
            if math.isfinite(override) and override >= 0.0:
                alpha = override
            else:
                try:
                    alpha = float((self._tide_interp or {}).get('alpha') or 0.0)
                except (TypeError, ValueError):
                    alpha = 0.0
        if not math.isfinite(alpha):
            alpha = 0.0
        key = round(max(0.0, min(1.0, alpha)), 2)
        if key != self._wl_alpha_key:
            self._wl_rings_5186 = waterline_rings_at_alpha(
                self._wl_steps, key)
            self._wl_alpha_key = key
            self.get_logger().info(
                f'수위선 파일 α={key:.2f} '
                f'steps={len(self._wl_steps)} '
                f'rings={len(self._wl_rings_5186)}')
        rings = rings_5186_to_map(
            self._wl_rings_5186,
            (pin_x, pin_y),
            (self._robot_x, self._robot_y),
        )
        self._waterline_ring_count = len(rings)
        self.get_logger().info(
            f'수위선 마커 α={alpha:.3f} grow_m={self._grow_m:.1f} '
            f'rings={len(rings)}',
            throttle_duration_sec=30.0)
        self._publish_line_markers(
            self.waterline_markers_pub, rings, 'tide_waterline',
            (1.0, 0.45, 0.08, 0.95), 10.0)
        self._publish_edge_markers(rings)

    def _publish_edge_markers(self, rings):
        """끊긴 띠를 전부 MarkerArray 로. Foxglove 3D type=MarkerArray."""
        self._publish_line_markers(
            self.water_markers_pub, rings, 'tide_edge',
            (0.25, 0.55, 1.0, 0.95), 6.0)

    def _publish_line_markers(self, publisher, rings, ns, color, scale_x):
        """DELETEALL 뒤 LINE_STRIP. Foxglove 3D type=MarkerArray."""
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        stamp = self.get_clock().now().to_msg()
        red, green, blue, alpha = color
        for index, ring in enumerate(rings or ()):
            if len(ring) < 2:
                continue
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'map'
            marker.ns = str(ns)
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = float(scale_x)
            marker.color.r = float(red)
            marker.color.g = float(green)
            marker.color.b = float(blue)
            marker.color.a = float(alpha)
            marker.pose.orientation.w = 1.0
            for x_val, y_val in ring:
                pt = Point()
                pt.x = float(x_val)
                pt.y = float(y_val)
                pt.z = 0.0
                marker.points.append(pt)
            first = ring[0]
            last = ring[-1]
            if math.hypot(first[0] - last[0], first[1] - last[1]) <= 2.0:
                close = Point()
                close.x = float(first[0])
                close.y = float(first[1])
                close.z = 0.0
                marker.points.append(close)
            arr.markers.append(marker)
        publisher.publish(arr)

    def _publish_sea_heading(self):
        """바다 방향을 PoseWithCovarianceStamped 로 발행한다.

        Foxglove 3D에서 /imu/mag_heading, /gps/heading_visual 과 같이
        type=Arrow 로 붙이면 된다. EKF 에 넣지 않는다.
        """
        if not self._refresh_map_pose():
            return
        sea_dir = self._compute_sea_direction(
            self._current_lat, self._current_lon)
        if sea_dir is None:
            return
        yaw = yaw_from_direction(sea_dir[0], sea_dir[1])
        if yaw is None:
            return
        qz, qw = yaw_to_quat_zw(yaw)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = float(self._robot_x)
        pose.pose.pose.position.y = float(self._robot_y)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.pose.covariance[35] = 1e-3
        self.sea_heading_pub.publish(pose)

    def _distance_to_keepout(self, px, py):
        """EPSG:5186 점에서 수위선(없으면 구운 keepout) 까지 거리."""
        rings = self._wl_rings_5186 or self._baked_rings_5186
        if not rings:
            return None
        query = ShapelyPoint(px, py)
        best = None
        for ring in rings:
            if not ring or len(ring) < 2:
                continue
            try:
                coords = list(ring)
                if coords[0] != coords[-1]:
                    coords.append(coords[0])
                dist = float(query.distance(LineString(coords)))
            except (TypeError, ValueError):
                continue
            if best is None or dist < best:
                best = dist
        return best

    def _compute_access(self, now):
        """조수선 성장 + gps_home 복귀 거리로 접근 범위를 계산한다."""
        retreat_cfg = self.ops_config.get('retreat') or {}
        has_fix = (
            self._current_lat is not None and self._current_lon is not None)
        dist_keepout = None
        dist_coast = None
        on_mud = False
        exit_m = 0.0
        if has_fix and self._gps_to_5186 is not None:
            try:
                px, py = self._gps_to_5186.transform(
                    self._current_lon, self._current_lat)
            except Exception:
                px = py = None
            if px is not None:
                dist_keepout = self._distance_to_keepout(px, py)
                anchor = self._coast_anchor(
                    self._current_lat, self._current_lon)
                if anchor is not None:
                    dist_coast = math.hypot(anchor['dx'], anchor['dy'])
                mud = point_on_mudflat((px, py), self._mudflat_polys)
                on_mud = bool(mud)
        self._refresh_map_pose()
        if (
            self._home_x is not None
            and self._home_y is not None
            and self._has_map_pose
        ):
            exit_m = math.hypot(
                self._robot_x - self._home_x,
                self._robot_y - self._home_y)
        elif on_mud and dist_coast is not None:
            exit_m = dist_coast
        access = plan_tide_access(
            self.points, now,
            span_m=self._grow_m,
            dist_to_exit_m=exit_m,
            on_mud=on_mud,
            robot_speed_mps=retreat_cfg.get('robot_avg_speed_mps', 0.3),
            time_buffer_min=retreat_cfg.get('time_buffer_min', 10),
            spatial_margin_m=retreat_cfg.get('spatial_margin_m', 20.0),
            min_work_m=retreat_cfg.get('min_work_m', 15.0),
            dist_to_keepout_m=dist_keepout,
            dist_to_coast_m=dist_coast,
            has_fix=has_fix,
            max_speed_mps=retreat_cfg.get('robot_max_speed_mps'),
        )
        if self.dynamic_info is not None:
            ground_time = crossing_time(
                self.points, self.dynamic_info['retreat_level_m'],
                direction='rising', after_dt=now)
            merged = earlier_retreat_time(
                access.get('retreat_decision_time'), ground_time)
            access['retreat_decision_time'] = merged
            if merged is not None and merged == ground_time:
                access['reason'] = 'ground_earlier'
        if access.get('retreat_decision_time') is not None:
            access['seconds_to_retreat'] = (
                access['retreat_decision_time'] - now).total_seconds()
        return access

    def _build_status(self, now):
        """현재 시각 기준 조석 상태 JSON dict를 만든다."""
        interp = self._tide_interp
        access = self._access or empty_tide_access()
        crossing = self.window['crossing_time']
        retreat_at = access.get('retreat_decision_time')
        tide_when = interp.get('tide_when')
        offset_h = float(
            self.get_parameter('tide_clock_offset_hours').value or 0.0)
        try:
            clock_speed = float(
                self.get_parameter('tide_clock_speed').value or 1.0)
        except (TypeError, ValueError):
            clock_speed = 1.0

        def _fmt(value):
            return value.strftime('%Y-%m-%d %H:%M') if value else None

        return {
            'stamp': now.strftime('%Y-%m-%d %H:%M:%S'),
            'tide_when': (
                tide_when.strftime('%Y-%m-%d %H:%M:%S') if tide_when else None),
            'clock_offset_hours': offset_h,
            'clock_speed': clock_speed,
            'station_code': self._current_station_code,
            'tide_level_m': interp.get('tide_level_m'),
            't_low': interp.get('t_low'),
            't_high': interp.get('t_high'),
            'alpha': interp.get('alpha'),
            'waterline_alpha': self._wl_alpha_key,
            'waterline_tile_id': self._wl_tile_id,
            'cache_ok': self._cache_ok,
            'cache_reason': self._cache_reason,
            'cache_path': self._cache_path,
            'crossing_time': crossing.strftime('%Y-%m-%d %H:%M') if crossing else None,
            'retreat_decision_time': _fmt(retreat_at),
            'enter_time': _fmt(access.get('enter_time')),
            'seconds_to_retreat': access.get('seconds_to_retreat'),
            'safe_all_day': self.window['safe_all_day'],
            'retreat_published': self.retreat_sent,
            'retreat_level_m': access.get('retreat_level_m'),
            'danger_level_m': (
                self.dynamic_info['danger_level_m']
                if self.dynamic_info else None),
            'rise_rate_m_per_hour': rise_rate_m_per_hour(self.points, now)
            if self.points else None,
            'water_inland_mps': access.get('water_inland_mps'),
            'phase': access.get('phase'),
            'on_mud': access.get('on_mud'),
            'mud_width_m': access.get('span_m'),
            'dry_width_m': access.get('dry_width_m'),
            'accessible_from_coast_m': access.get('accessible_from_coast_m'),
            'distance_to_exit_m': access.get('distance_to_exit_m'),
            'remaining_dry_m': access.get('remaining_dry_m'),
            'seconds_to_exit': access.get('seconds_to_exit'),
            'retreat_reason': access.get('reason'),
            'robot_speed_mps': access.get('robot_speed_mps'),
            'wet_progress': access.get('wet_progress'),
            'grow_m': self._grow_m,
            'waterline_rings': self._waterline_ring_count,
        }


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TideWatchNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
