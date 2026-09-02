import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'eclipse_pkg'

    # Configuration files
    nav2_params_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'nav2_params_mudflat.yaml'
    ])

    probe_sensor_yaml = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'probe_sensor.yaml'
    ])

    # Launch Arguments
    declare_enable_yolo = DeclareLaunchArgument(
        'enable_yolo',
        default_value='false',
        description='Launch YOLOv8 detection node'
    )

    declare_enable_foxglove = DeclareLaunchArgument(
        'enable_foxglove',
        default_value='true',
        description='Launch Foxglove Bridge for remote monitoring'
    )

    declare_tide_gps_topic = DeclareLaunchArgument(
        'tide_gps_topic',
        default_value='/gps/fix',
        description=(
            'GPS topic for tide_watch only. Use /gps/fix_tide_sim to inject '
            'a fake mudflat fix without feeding EKF.'
        ),
    )

    declare_keepout_site = DeclareLaunchArgument(
        'keepout_site',
        default_value='jebu',
        description=(
            'Named waterline_<site>_steps.json and '
            'keepout_<site>_perimeter.geojson. Default jebu is the lerp '
            'keepout→holes bake. Use tiles (or gps / _) for nationwide '
            'GPS tiles. Keepout geojson is not fed to the costmap.'
        ),
    )

    declare_enable_tide_patrol = DeclareLaunchArgument(
        'enable_tide_patrol',
        default_value='false',
        description=(
            'Wander the dry mudflat during the tide access window. '
            'Off by default. Sends /navigation/patrol_goal.'
        ),
    )

    declare_waterline_alpha_override = DeclareLaunchArgument(
        'waterline_alpha_override',
        default_value='-1.0',
        description=(
            'If >= 0, draw that baked waterline alpha instead of the '
            'forecast. Negative (default) uses tide forecast alpha.'
        ),
    )

    # NOTE: enable_height_ai / height_ai_model_path are declared in
    # description.launch.py and applied to eclipse_test_controller, because the
    # height policy runs INSIDE that node rather than as its own process. Pass
    # them straight through on the command line, e.g.
    #   ros2 launch eclipse_pkg tars_autonomy.launch.py \
    #     height_ai_model_path:=/path/to/height_outcome_v1.json

    # 1. Base Hardware & Localization (Includes EKF, GPS, Motors)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'description.launch.py',
            ])
        ]),
        launch_arguments={
            'enable_front_camera': 'false',  # RealSense replaces usb_cam.
            'enable_gamepad_drive': 'true',
            'gamepad_cmd_vel_topic': '/cmd_vel',
        }.items()
    )

    # 2. Perception (RealSense Depth Camera for Costmap)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'realsense_d435.launch.py',
            ])
        ])
    )

    # 3. Nav2 Navigation Stack (Mapless, Phase 2)
    nav2_launch = GroupAction(
        actions=[
            # bt_navigator의 기본 /goal_pose 구독을 격리한다.
            # Foxglove 목표는 gps_waypoint_commander만 검증/변환한다.
            SetRemap(src='goal_pose', dst='/nav2/goal_pose'),
            # 2026-08-09 Lee 승인: Nav2 bond heartbeat(관리 노드 7개 + lifecycle_manager,
            # /bond 토픽 RELIABLE ~90Hz)가 CPU 과부하 상황에서 lifecycle_manager CPU의
            # 상당 부분을 차지하는 것으로 확인되어 비활성화. 관리 노드와 lifecycle_manager
            # 양쪽 다 bond_timeout을 명시적으로 설정하지 않으므로 이 글로벌 기본값이
            # 양쪽 모두에 적용된다(한쪽만 끄면 반대쪽이 "unreachable by bond"로 오판해
            # 전체 shutdown을 유발할 수 있어 반드시 함께 꺼야 함).
            SetParameter(name='bond_timeout', value=0.0),
            # Local fork of nav2_bringup's navigation_launch.py — identical
            # except that waypoint_follower is dropped (unused here; see the
            # header of tars_navigation.launch.py). Re-diff against upstream
            # after any Nav2 upgrade.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch',
                        'tars_navigation.launch.py',
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_path,
                    'autostart': 'true'
                }.items()
            ),
        ]
    )

    # 4. Commander Node (Phase 3: Click-to-Navigate)
    gps_commander_node = Node(
        package=pkg_name,
        executable='gps_waypoint_commander',
        name='gps_waypoint_commander',
        output='screen'
    )

    # 4a. GPS loss W2 + soft /gps/recover + return-to-home (no hard restart).
    # Home: first good-fix map pose, or set home_x/home_y parameters.
    gps_health_supervisor_node = Node(
        package=pkg_name,
        executable='gps_health_supervisor',
        name='gps_health_supervisor',
        output='screen',
        parameters=[{
            'stale_sec': 2.0,
            'wait_sec': 45.0,
            'wait2_sec': 30.0,
            'enable_return_home': True,
            'home_frame': 'map',
        }]
    )

    tide_patrol_node = Node(
        package=pkg_name,
        executable='tide_patrol_node',
        name='tide_patrol',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tide_patrol')),
        parameters=[{
            'enable_patrol': True,
        }],
    )

    def _tide_watch_node(context, *args, **kwargs):
        site_raw = str(
            LaunchConfiguration('keepout_site').perform(context) or '').strip()
        if site_raw.lower() in ('tiles', 'gps', '_'):
            site = ''
        else:
            site = site_raw
        station = {
            'incheon': 'DT_0001',
            'gomso': 'DT_0068',
        }.get(site.lower(), '')
        try:
            wl_override = float(
                LaunchConfiguration('waterline_alpha_override').perform(context)
                or -1.0)
        except (TypeError, ValueError):
            wl_override = -1.0
        return [Node(
            package=pkg_name,
            executable='tide_watch_node',
            name='tide_watch_node',
            output='screen',
            parameters=[{
                'cache_file': '',
                'cache_dir': '/workspaces/eclipse-test-2/datasets/tide_cache',
                'auto_fetch': True,
                'config_file': PathJoinSubstitution([
                    FindPackageShare(pkg_name),
                    'config', 'tide_ops.yaml'
                ]),
                'publish_rate_hz': 0.1,
                'polygon_margin_m': 4.0,
                'waterline_window_radius_m': 20000.0,
                'waterline_edge_inset_m': 0.0,
                'waterline_edge_inset_ratio': 0.10,
                'waterline_half_length_m': 20000.0,
                'waterline_station_m': 60.0,
                'waterline_live_compute': False,
                'waterline_alpha_override': wl_override,
                'keepout_site': site,
                'keepout_geojson': '',
                'waterline_tiles_dir':
                    '/workspaces/eclipse-test-2/progress/waterline_tiles',
                'keepout_tiles_dir':
                    '/workspaces/eclipse-test-2/progress/keepout_tiles',
                'tide_clock_offset_hours': 0.0,
                'enable_gps_station_select': True,
                'station_code': station,
                'heading_publish_rate_hz': 0.2,
                'sea_heading_topic': '/tide/sea_heading',
                'gps_topic': LaunchConfiguration('tide_gps_topic'),
                'coastline_shapefile':
                    '/workspaces/eclipse-test-2/datasets/coastline/2026 해안선.shp',
                'mudflat_shapefile':
                    '/workspaces/eclipse-test-2/datasets/tidflt/tidflt_jebu_5186.shp',
            }],
        )]

    tide_watch_node = OpaqueFunction(function=_tide_watch_node)

    # 4b. EKF yaw is already aligned via pose0 (/imu/mag_heading) in ekf.yaml.
    # GPS heading bootstrap (heading_calibration_bootstrap_node) and
    # one-shot set_pose (heading_calibration_node) are NOT needed —
    # the EKF continuously fuses the magnetometer heading, and
    # navsat_transform_node uses that yaw (use_odometry_yaw: true).
    # gps_fix_gate_node is also removed — navsat gets /gps/fix directly.
    # 2026-08-15: simplified to EKF pose0-only path.

    # 4c. Static TF: camera_link → base_link. RealSense publishes internal
    # camera_link → camera_depth_optical_frame, but the link to base_link
    # is missing. Without this, costmap can't transform camera points to
    # robot frame (Message Filter dropping errors).
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.270', '0.000', '0.050',
                   '0.0', '0.0', '0.0',
                   'base_link', 'camera_link'],
        output='screen'
    )

    # 4d. Probe angle sensor (Arduino). Same wiring as the probe block in
    # description_ai.launch.py (package/executable/parameter loading kept).
    # Parameters come from config/probe_sensor.yaml (/dev/ttyPROBE, 115200);
    # its top-level key is `probe_sensor`, so the node must keep exactly that
    # name for the params to apply (probe_sensor.py's internal default name
    # probe_sensor_node is overridden here on purpose).
    # Publishes /probe/angle, consumed by the tars_recovery_behaviors/MudAssess
    # recovery plugin. If the serial device is absent the node retries on its
    # own reconnect loop; the launch still comes up (verified), so no
    # condition is needed here.
    probe_sensor_node = Node(
        package=pkg_name,
        executable='probe_sensor',
        name='probe_sensor',
        output='screen',
        parameters=[probe_sensor_yaml]
    )

    # NOTE: the GPS heading bootstrap (heading_calibration_bootstrap_node) was
    # REMOVED on 2026-08-13 — its blind creep degraded single-base RTK to
    # DGNSS and is unnecessary now that mag_heading_calibration_node aligns
    # yaw from a stationary magnetometer heading. `max_attempts=0` would NOT
    # have disabled it (the node still runs one creep), so the node itself was
    # removed rather than tuned.

    # 5. Remote UI (Foxglove Bridge)
    # 광고 목록은 config/foxglove_bridge.yaml. 수다/수명주기·원본 카메라 제외.
    foxglove_params = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'foxglove_bridge.yaml',
    ])
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        output='screen',
        parameters=[foxglove_params],
    )

    # 6. Optional: YOLOv8 Integration
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_name),
                'launch',
                'yolo_detect.launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_yolo'))
    )

    pointcloud_filter = Node(
        package=pkg_name,
        executable='pointcloud_filter_node',
        name='pointcloud_filter',
        output='screen',
    )

    ld = LaunchDescription([
        declare_enable_yolo,
        declare_enable_foxglove,
        declare_tide_gps_topic,
        declare_keepout_site,
        declare_enable_tide_patrol,
        declare_waterline_alpha_override,
        base_launch,
        realsense_launch,
        pointcloud_filter,
        nav2_launch,
        gps_commander_node,
        gps_health_supervisor_node,
        tide_watch_node,
        tide_patrol_node,
        camera_tf,
        probe_sensor_node,
        foxglove_bridge_node,
        yolo_launch
    ])
    return ld
