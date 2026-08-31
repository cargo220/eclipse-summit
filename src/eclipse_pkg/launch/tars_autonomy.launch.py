import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
            'Site name for baked waterline steps '
            '(config/waterline_<name>_steps.json). Keepout geojson is not '
            'fed to the costmap.'
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

    # enable_height_ai / height_ai_model_path는 description.launch.py에서
    # eclipse_test_controller로 전달한다. 높이 정책은 그 프로세스 안에서 돈다.

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
            # bond를 끄면 관리 노드와 lifecycle_manager 양쪽에 같이 적용해야 한다.
            # 한쪽만 끄면 반대편이 unreachable로 오판해 전체 shutdown을 유발한다.
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
        site = LaunchConfiguration('keepout_site').perform(context)
        station = {
            'incheon': 'DT_0001',
            'gomso': 'DT_0068',
        }.get(str(site or '').strip().lower(), '')
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
                'keepout_site': LaunchConfiguration('keepout_site'),
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

    # 4b. EKF yaw is pose0 (/imu/mag_heading). navsat uses use_odometry_yaw
    # and /gps/fix directly.

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

    pointcloud_filter = ExecuteProcess(
        cmd=['python3',
             '/workspaces/eclipse-test-2/src/eclipse_pkg/eclipse_pkg/'
             'pointcloud_filter_node.py'],
        output='screen',
    )

    ld = LaunchDescription([
        declare_enable_yolo,
        declare_enable_foxglove,
        declare_tide_gps_topic,
        declare_keepout_site,
        declare_enable_tide_patrol,
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
