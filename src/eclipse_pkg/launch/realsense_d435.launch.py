"""Standalone D435 launch for Docker/Jetson smoke tests.

Default topics (camera_namespace:=camera, camera_name:=camera):
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/depth/color/points (enable_pointcloud:=true by default;
  required by nav2_params_mudflat.yaml's local/global costmap obstacle layers)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    align_depth = LaunchConfiguration('align_depth')
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_sync = LaunchConfiguration('enable_sync')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    config_file = LaunchConfiguration('config_file')
    device_type = LaunchConfiguration('device_type')
    camera_namespace = LaunchConfiguration('camera_namespace')
    camera_name = LaunchConfiguration('camera_name')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        launch_arguments={
            'align_depth.enable': align_depth,
            'enable_color': enable_color,
            'enable_depth': enable_depth,
            'enable_sync': enable_sync,
            # Jetson ARM NEON 빌드(realsense2_camera 4.58.2): 노드가
            # pointcloud 파라미터를 pointcloud__neon_ 접두사로 선언해서
            # rs_launch.py의 표준 이름 pointcloud.enable로는 전달이 무시된다.
            # config_file 경유로 노드에 직접 주입한다.
            'config_file': config_file,
            'device_type': device_type,
            'camera_namespace': camera_namespace,
            'camera_name': camera_name,
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'align_depth',
            default_value='true',
            description='Publish depth aligned to color.',
        ),
        DeclareLaunchArgument(
            'enable_color',
            default_value='true',
            description='Enable color stream.',
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable depth stream.',
        ),
        DeclareLaunchArgument(
            'enable_sync',
            default_value='true',
            description='Enable inter-stream sync when supported.',
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='true',
            description=(
                'Enable PointCloud2 publishing. Required by '
                'nav2_params_mudflat.yaml local/global costmap obstacle layers.'
            ),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('eclipse_pkg'),
                'config',
                'realsense_pointcloud.yaml',
            ),
            description='YAML injecting pointcloud__neon_ params directly into the node.',
        ),
        DeclareLaunchArgument(
            'device_type',
            default_value='d435',
            description='RealSense device type filter (e.g. d435).',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='ROS namespace for the RealSense wrapper.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera name segment used in topic paths.',
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value='320x240x30',
            description='rgb_camera.color_profile (WxHxFPS).',
        ),
        DeclareLaunchArgument(
            'depth_profile',
            # Lower FPS than color: costmap does not need 30 Hz depth clouds.
            # Color stays 320x240x30 for YOLO.
            default_value='424x240x15',
            description='depth_module.depth_profile (WxHxFPS).',
        ),
        realsense_launch,
    ])
