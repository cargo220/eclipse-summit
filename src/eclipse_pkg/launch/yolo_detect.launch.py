"""Standalone thin YOLO 2D detect + optional depth→3D lift.

Default input: /camera/camera/color/image_raw (Best Effort QoS).
Default outputs: /yolo/detections (vision_msgs/Detection2DArray),
                 /yolo/debug_image (optional),
                 /yolo/detections_3d (vision_msgs/Detection3DArray, if enable_3d),
                 /yolo/detections_3d_markers (MarkerArray for RViz, if enable_3d).

Uses eclipse_pkg yolo_detect_node (Ultralytics) + yolo_3d_node.
Does not require yolo_ros. License note: Ultralytics is AGPL.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _maybe_3d(context, *args, **kwargs):
    enabled = LaunchConfiguration('enable_3d').perform(context).lower()
    if enabled not in ('true', '1', 'yes'):
        return []

    return [
        Node(
            package='eclipse_pkg',
            executable='yolo_3d_node',
            name='yolo_3d_node',
            output='screen',
            parameters=[{
                'detections_topic': LaunchConfiguration(
                    'detections_topic'
                ).perform(context),
                'depth_topic': LaunchConfiguration('depth_topic').perform(
                    context
                ),
                'camera_info_topic': LaunchConfiguration(
                    'camera_info_topic'
                ).perform(context),
                'detections_3d_topic': LaunchConfiguration(
                    'detections_3d_topic'
                ).perform(context),
                'publish_markers': LaunchConfiguration(
                    'publish_markers'
                ).perform(context).lower()
                in ('true', '1', 'yes'),
                'markers_topic': LaunchConfiguration('markers_topic').perform(
                    context
                ),
                'depth_window': int(
                    LaunchConfiguration('depth_window').perform(context)
                ),
                'max_depth_dt_sec': float(
                    LaunchConfiguration('max_depth_dt_sec').perform(context)
                ),
                'depth_use_latest_fallback': LaunchConfiguration(
                    'depth_use_latest_fallback'
                ).perform(context).lower()
                in ('true', '1', 'yes'),
                'depth_buffer_size': int(
                    LaunchConfiguration('depth_buffer_size').perform(context)
                ),
                'marker_lifetime_sec': float(
                    LaunchConfiguration('marker_lifetime_sec').perform(context)
                ),
                'depth_bbox_scale': float(
                    LaunchConfiguration('depth_bbox_scale').perform(context)
                ),
                'allowed_classes': LaunchConfiguration(
                    'allowed_classes'
                ).perform(context),
                'denied_classes': LaunchConfiguration(
                    'denied_classes'
                ).perform(context),
            }],
        )
    ]


def generate_launch_description():
    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    image_topic = LaunchConfiguration('image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    debug_image_topic = LaunchConfiguration('debug_image_topic')
    publish_debug = LaunchConfiguration('publish_debug')
    threshold = LaunchConfiguration('threshold')
    imgsz = LaunchConfiguration('imgsz')

    yolo_node = Node(
        package='eclipse_pkg',
        executable='yolo_detect_node',
        name='yolo_detect_node',
        output='screen',
        parameters=[{
            'model': model,
            'device': ParameterValue(device, value_type=str),
            'image_topic': image_topic,
            'detections_topic': detections_topic,
            'debug_image_topic': debug_image_topic,
            'publish_debug': ParameterValue(publish_debug, value_type=bool),
            'publish_debug_compressed': ParameterValue(
                LaunchConfiguration('publish_debug_compressed'),
                value_type=bool,
            ),
            'publish_debug_raw': ParameterValue(
                LaunchConfiguration('publish_debug_raw'),
                value_type=bool,
            ),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'),
                value_type=int,
            ),
            'debug_max_width': ParameterValue(
                LaunchConfiguration('debug_max_width'),
                value_type=int,
            ),
            'debug_max_height': ParameterValue(
                LaunchConfiguration('debug_max_height'),
                value_type=int,
            ),
            'h264_udp_enable': ParameterValue(
                LaunchConfiguration('h264_udp_enable'),
                value_type=bool,
            ),
            'h264_udp_host': LaunchConfiguration('h264_udp_host'),
            'h264_udp_port': ParameterValue(
                LaunchConfiguration('h264_udp_port'),
                value_type=int,
            ),
            'h264_bitrate': ParameterValue(
                LaunchConfiguration('h264_bitrate'),
                value_type=int,
            ),
            'h264_fps': ParameterValue(
                LaunchConfiguration('h264_fps'),
                value_type=int,
            ),
            'threshold': ParameterValue(threshold, value_type=float),
            'imgsz': ParameterValue(imgsz, value_type=int),
            'allowed_classes': LaunchConfiguration('allowed_classes'),
            'denied_classes': LaunchConfiguration('denied_classes'),
            'class_names': LaunchConfiguration('class_names'),
            'class_thresholds': LaunchConfiguration('class_thresholds'),
            'force_cpu_nms': ParameterValue(
                LaunchConfiguration('force_cpu_nms'), value_type=str
            ),
            'infer_rate': ParameterValue(
                LaunchConfiguration('infer_rate'), value_type=float
            ),
            'debug_rate': ParameterValue(
                LaunchConfiguration('debug_rate'), value_type=float
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='yolov8n.pt',
            description='Ultralytics model name/path (n = smoke).',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Inference device (cpu, cuda:0, ...).',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Color image topic from RealSense.',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo/detections',
            description='Detection2DArray output topic.',
        ),
        DeclareLaunchArgument(
            'debug_image_topic',
            default_value='/yolo/debug_image',
            description='Annotated debug image topic.',
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='true',
            description='Publish annotated debug image.',
        ),
        DeclareLaunchArgument(
            'publish_debug_compressed',
            default_value='true',
            description='Publish JPEG CompressedImage at debug_image_topic/compressed.',
        ),
        DeclareLaunchArgument(
            'publish_debug_raw',
            default_value='false',
            description='Also publish raw sensor_msgs/Image (heavy on Wi-Fi).',
        ),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='65',
            description='JPEG quality 1-100 for compressed debug image.',
        ),
        DeclareLaunchArgument(
            'debug_max_width',
            default_value='320',
            description='Max debug width before JPEG (0=no limit). Default 320.',
        ),
        DeclareLaunchArgument(
            'debug_max_height',
            default_value='180',
            description='Max debug height before JPEG (0=no limit). Default 180.',
        ),
        DeclareLaunchArgument(
            'h264_udp_enable',
            default_value='false',
            description='Send annotated debug as H.264 RTP/UDP (Jetson HW enc).',
        ),
        DeclareLaunchArgument(
            'h264_udp_host',
            default_value='',
            description='Laptop IP for H.264 UDP (e.g. 10.10.1.19).',
        ),
        DeclareLaunchArgument(
            'h264_udp_port',
            default_value='5600',
            description='UDP port for H.264 RTP.',
        ),
        DeclareLaunchArgument(
            'h264_bitrate',
            default_value='3000000',
            description='H.264 bitrate bits/sec (nvv4l2h264enc).',
        ),
        DeclareLaunchArgument(
            'h264_fps',
            default_value='30',
            description='H.264 pipeline framerate hint.',
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.3',
            description='Detection confidence threshold.',
        ),
        DeclareLaunchArgument(
            'imgsz',
            default_value='640',
            description='YOLO inference image size.',
        ),
        DeclareLaunchArgument(
            'force_cpu_nms',
            default_value='auto',
            description='Patch torchvision NMS to CPU (auto|true|false; auto for *.engine).',
        ),
        DeclareLaunchArgument(
            'infer_rate',
            default_value='0',
            description='Cap inference Hz (0 = unlimited); independent of debug_rate.',
        ),
        DeclareLaunchArgument(
            'debug_rate',
            default_value='0',
            description='Cap debug image publish Hz (0 = unlimited/camera rate).',
        ),
        DeclareLaunchArgument(
            'allowed_classes',
            default_value='person',
            description=(
                'Comma-separated class names to keep (empty = all). '
                'Rescue default is person. merged5: person,shell '
                'and set class_names.'
            ),
        ),
        DeclareLaunchArgument(
            'denied_classes',
            default_value='',
            description='Comma-separated class names to drop.',
        ),
        DeclareLaunchArgument(
            'class_names',
            default_value='',
            description=(
                'Ordered names for engines without metadata '
                '(empty = COCO-80 fallback). merged5: person,shell.'
            ),
        ),
        DeclareLaunchArgument(
            'class_thresholds',
            default_value='',
            description=(
                'Per-class score cuts, e.g. person:0.65,shell:0.3 '
                '(empty = use threshold for every class).'
            ),
        ),
        DeclareLaunchArgument(
            'enable_3d',
            default_value='true',
            description='Start yolo_3d_node (aligned depth → Detection3DArray).',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Aligned depth image for 3D lift.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='Color camera_info for deprojection.',
        ),
        DeclareLaunchArgument(
            'detections_3d_topic',
            default_value='/yolo/detections_3d',
            description='Detection3DArray output topic.',
        ),
        DeclareLaunchArgument(
            'publish_markers',
            default_value='true',
            description='Publish RViz MarkerArray for 3D detections.',
        ),
        DeclareLaunchArgument(
            'markers_topic',
            default_value='/yolo/detections_3d_markers',
            description='MarkerArray topic for RViz.',
        ),
        DeclareLaunchArgument(
            'depth_window',
            default_value='3',
            description='Min half-window (pixels) for median depth sample.',
        ),
        DeclareLaunchArgument(
            'max_depth_dt_sec',
            default_value='0.35',
            description='Max |detection_stamp - depth_stamp| for 3D lift.',
        ),
        DeclareLaunchArgument(
            'depth_use_latest_fallback',
            default_value='true',
            description='If stamp match fails, use newest depth frame.',
        ),
        DeclareLaunchArgument(
            'depth_buffer_size',
            default_value='60',
            description='Depth frame ring buffer size for time sync.',
        ),
        DeclareLaunchArgument(
            'marker_lifetime_sec',
            default_value='1.0',
            description='RViz Marker lifetime seconds.',
        ),
        DeclareLaunchArgument(
            'depth_bbox_scale',
            default_value='0.35',
            description='Sample window as fraction of 2D box size.',
        ),
        yolo_node,
        OpaqueFunction(function=_maybe_3d),
    ])
