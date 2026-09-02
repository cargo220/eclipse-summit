import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'eclipse_pkg'
    enable_front_camera = LaunchConfiguration('enable_front_camera')
    enable_gamepad_drive = LaunchConfiguration('enable_gamepad_drive')
    enable_joy_node = LaunchConfiguration('enable_joy_node')
    enable_keyboard_teleop = LaunchConfiguration('enable_keyboard_teleop')
    front_camera_device = LaunchConfiguration('front_camera_device')
    joy_device = LaunchConfiguration('joy_device')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    joy_autorepeat_rate = LaunchConfiguration('joy_autorepeat_rate')
    gamepad_cmd_vel_topic = LaunchConfiguration('gamepad_cmd_vel_topic')
    enable_height_ai = LaunchConfiguration('enable_height_ai')
    height_ai_model_path = LaunchConfiguration('height_ai_model_path')

    ekf_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf.yaml',
    )
    navsat_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'navsat_transform_node.yaml',
    )
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'robot.urdf',
    )
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    return LaunchDescription([
        DeclareLaunchArgument(
            'gamepad_cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity topic published by gamepad_drive.',
        ),

        # Height AI policy runs inside eclipse_test_controller (the Dynamixel
        # bus owner), not as its own node — see height_ai_apply_loop.
        DeclareLaunchArgument(
            'enable_height_ai',
            default_value='true',
            description=(
                'Let the height AI policy propose heights inside '
                'eclipse_test_controller. Manual D-pad input always wins '
                'regardless; with no model path the policy is a stub that '
                'only ever holds the current height.'
            ),
        ),

        DeclareLaunchArgument(
            'height_ai_model_path',
            default_value='',
            description=(
                'Height outcome checkpoint JSON (tars-height-outcome-v1). '
                'Empty or invalid = stub hold. A skeleton (zero weights) '
                'loads the grid path but still holds the current height.'
            ),
        ),

        DeclareLaunchArgument(
            'enable_front_camera',
            default_value='false',
            description='Start the front usb_cam image_raw publisher.',
        ),

        DeclareLaunchArgument(
            'front_camera_device',
            default_value='/dev/video4',
            description='Video device path for the front camera.',
        ),

        DeclareLaunchArgument(
            'enable_keyboard_teleop',
            default_value='false',
            description='Start keyboard teleop. Off by default when gamepad_drive publishes /cmd_vel.',
        ),

        DeclareLaunchArgument(
            'enable_gamepad_drive',
            default_value='true',
            description='Start gamepad_drive to convert /joy into /cmd_vel.',
        ),

        DeclareLaunchArgument(
            'enable_joy_node',
            default_value='true',
            description='Start ROS joy_node for a local joystick device.',
        ),

        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js0',
            description='Joystick device path used by joy_node.',
        ),

        DeclareLaunchArgument(
            'joy_deadzone',
            default_value='0.05',
            description='Deadzone passed to joy_node.',
        ),

        DeclareLaunchArgument(
            'joy_autorepeat_rate',
            default_value='20.0',
            description='Autorepeat rate passed to joy_node.',
        ),

        # GPS node handles NGII VRS NTRIP RTCM injection and UBX/NMEA parsing.
        Node(
            package=pkg_name,
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyGPS',
                'baudrate': 38400,
                'frame_id': 'gps_link',
                'fix_topic': '/gps/fix',
                'vel_topic': '/gps/vel',
                # NGII RTS2 VRS. 비밀번호는 TARS_NTRIP_PASS 환경변수로 주입한다.
                'ntrip_host': 'rts2.ngii.go.kr',
                'ntrip_port': 2101,
                'ntrip_user': 'tars260223',
                'ntrip_pass': 'ngii',
                'mountpoint': 'VRS-RTCM32',
                # 조정가능 — GPS 모션 헤딩 최소 속도 (tars_tuning § gps_heading_speed_gates)
                # bootstrap creep 0.18 m/s 보다 낮아야 부팅 헤딩 샘플이 나옴.
                'heading_min_speed_mps': 0.15,
            }],
        ),

        Node(
            package='eclipse_pkg',
            executable='iahrs',
            name='iahrs_driver',
            output='screen',
        ),

        # URDF 고정 TF(base_link->camera_link 등)를 공급한다.
        # camera_link가 없으면 RealSense 포인트클라우드의 TF 조회가
        # 전부 실패해서 Nav2 costmap 마킹이 동작하지 않는다.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
            }],
        ),

        # base_link->imu_link / base_link->gps_link used to be two separate
        # static_transform_publisher processes here. They now live as fixed
        # joints in robot.urdf, so robot_state_publisher above emits them on
        # the same /tf_static — two fewer processes and DDS participants, with
        # the TRANSIENT_LOCAL latch still served by a long-lived publisher.
        # The offsets still need physical verification; see the warning block
        # in robot.urdf.

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config_path],
            remappings=[
                ('/imu', '/imu/data'),
                # navsat reads /gps/fix directly. EKF yaw comes from
                # pose0 (/imu/mag_heading), not a one-shot set_pose.
                ('/odometry/filtered', '/odometry/filtered'),
                ('/odometry/gps', '/odometry/gps_raw'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        # gps_pose_covariance_odom and gps_velocity_odom merged into GPS_node
        # (2026-08-15). GPS_node now publishes /odometry/gps and
        # /odometry/gps_velocity directly.

        Node(
            package='eclipse_pkg',
            executable='eclipse_test_controller',
            name='eclipse_test_controller',
            output='screen',
            parameters=[{
                'enable_height_ai': ParameterValue(
                    enable_height_ai, value_type=bool
                ),
                'height_ai_model_path': ParameterValue(
                    height_ai_model_path, value_type=str
                ),
            }],
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            condition=IfCondition(enable_joy_node),
            output='screen',
            parameters=[{
                'dev': joy_device,
                'deadzone': ParameterValue(joy_deadzone, value_type=float),
                'autorepeat_rate': ParameterValue(joy_autorepeat_rate, value_type=float),
            }],
        ),

        Node(
            package='eclipse_pkg',
            executable='gamepad_drive',
            name='gamepad_drive',
            condition=IfCondition(enable_gamepad_drive),
            output='screen',
            parameters=[{
                'button_linear_step': 0.05,
                'cmd_vel_topic': gamepad_cmd_vel_topic,
            }],
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='front_camera',
            condition=IfCondition(enable_front_camera),
            output='screen',
            parameters=[{
                'video_device': front_camera_device,
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'frame_id': 'camera_link',
                'skip_device_check': True,
            }],
        ),

        # plotjuggler removed: headless Jetson aborts (no DISPLAY) and only
        # produces process-has-died noise; use laptop-side PlotJuggler if needed.

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            condition=IfCondition(enable_keyboard_teleop),
            prefix='xterm -e',
            output='screen',
        ),

    ])
