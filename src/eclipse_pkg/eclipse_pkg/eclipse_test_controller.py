"""Fixed-height wheel test controller for the eclipse-test workspace."""

from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from dynamixel_sdk import COMM_SUCCESS
from eclipse_pkg.current_covariance import (
    CurrentCovarianceState,
    apply_gps_good_wheel_covariance_floor,
    calculate_current_covariance,
    gps_fix_status_is_good,
)
from eclipse_pkg.eclipse_test_config import (
    BAUDRATE,
    BASE_COV,
    BUFFER_SIZE,
    CAMERA_PAN_HOME_TICKS,
    CAMERA_PAN_PROFILE_ACCELERATION,
    CAMERA_PAN_PROFILE_VELOCITY,
    CAMERA_PAN_TORQUE_HOLD_ON_SHUTDOWN,
    camera_pan_limits_are_valid,
    camera_pan_startup_goal,
    clamp_camera_pan_position,
    CMD_VEL_TIMEOUT_SEC,
    CONTROL_DT,
    COV_UPDATE_DT,
    CURRENT_COVARIANCE_CONFIG,
    CURRENT_UNIT_MA,
    DEFAULT_VELOCITY_I_GAIN,
    DEFAULT_VELOCITY_P_GAIN,
    DEVICENAME_FRONT,
    DEVICENAME_REAR,
    DXL_ALL_IDS,
    dxl_bus_name,
    dxl_wheel_goal_sign,
    DXL_CAMERA_PAN_ID,
    DXL_CAMERA_PAN_IDS,
    DXL_FRONT_WHEEL_IDS,
    DXL_HEIGHT_FRONT_ID,
    DXL_HEIGHT_FRONT_IDS,
    DXL_HEIGHT_IDS,
    DXL_HEIGHT_OPTIONAL_IDS,
    DXL_HEIGHT_REAR_ID,
    DXL_HEIGHT_REAR_IDS,
    DXL_LEFT_IDS,
    DXL_REAR_WHEEL_IDS,
    DXL_RIGHT_IDS,
    wheel_bulk_read_is_usable,
    FIXED_HEIGHT_DOWN_MM,
    GPS_FIX_TIMEOUT_SEC,
    GPS_MIN_GOOD_FIX_STATUS,
    GPS_GOOD_WHEEL_ODOM_COV,
    GPS_VELOCITY_TIMEOUT_SEC,
    HEIGHT_AI_APPLY_DT,
    HEIGHT_CURRENT_CLAMP_MA,
    height_current_clamp_is_enabled,
    HEIGHT_TEMPERATURE_STOP_C,
    HEIGHT_AI_DEADBAND_MM,
    HEIGHT_AI_MAX_RATE_MM_PER_S,
    HEIGHT_AI_PROBE_CONTACT_ANGLE,
    HEIGHT_AI_PROBE_CONTACT_TOLERANCE,
    HEIGHT_AI_PROBE_STALE_SEC,
    HEIGHT_INITIALIZE_ATTEMPTS,
    HEIGHT_INITIALIZE_RETRY_DELAY_SEC,
    HEIGHT_INITIALIZE_TIMEOUT_SEC,
    HEIGHT_POSITION_D_GAIN,
    HEIGHT_HOLD_INHIBIT_SEC,
    HEIGHT_POSITION_I_GAIN,
    HEIGHT_POSITION_LIMITS_BY_ID,
    HEIGHT_POSITION_P_GAIN,
    HEIGHT_POSITION_POLL_SEC,
    HEIGHT_PROFILE_ACCELERATION,
    HEIGHT_PROFILE_VELOCITY,
    HEIGHT_POSITION_TOLERANCE_TICKS,
    HEIGHT_SHUTDOWN_TIMEOUT_SEC,
    HEIGHT_STATUS_DT,
    HEIGHT_VOLTAGE_STATUS_DT,
    format_height_voltage_log,
    HEADING_HOLD_CMD_W_THRESHOLD,
    HEADING_HOLD_KP,
    HEADING_HOLD_MAX_W,
    HEIGHT_TORQUE_HOLD_ON_SHUTDOWN,
    HARDWARE_ERROR_NAMES,
    format_hardware_error_status,
    format_height_current_log,
    format_height_hardware_error_log,
    format_wheel_hardware_error_log,
    height_extreme_present,
    height_hold_goal_from_present,
    height_hold_latch_present_for_id,
    height_hold_may_latch,
    height_target_positions,
    height_temperature_is_stop,
    MAX_COV,
    MIN_CMD_VEL,
    MIN_TRACTION_WHEEL_SPEED,
    MOTOR_TO_WHEEL_SPEED_RATIO,
    MOTOR_BUS_WATCHDOG,
    MOTOR_BUS_WATCHDOG_UNIT_SEC,
    MOTOR_INPUT_VOLTAGE_UNIT_V,
    MOTOR_INPUT_VOLTAGE_WARN_MAX_V,
    MOTOR_INPUT_VOLTAGE_WARN_MIN_V,
    MOTOR_PWM_LIMIT,
    MOTOR_PWM_STALL_THRESHOLD,
    MOTOR_PWM_UNIT_PERCENT,
    MOTOR_SAFETY_DT,
    MOTOR_SAFETY_READ_FAILURE_LIMIT,
    MOTOR_SAFETY_WARN_LOG_INTERVAL_SEC,
    MOTOR_SHUTDOWN_MASK,
    MOTOR_STALL_DURATION_SEC,
    MOTOR_STALL_MAX_WHEEL_SPEED_MPS,
    MOTOR_STALL_MIN_CMD_MPS,
    MOTOR_TEMPERATURE_LIMIT_C,
    MOTOR_TEMPERATURE_STOP_C,
    MOTOR_TEMPERATURE_WARN_C,
    MOTOR_TEMP_WARN_SPEED_SCALE,
    NORMAL_STD,
    ODOM_COV_LOG_INTERVAL,
    POS_MAX,
    POS_MIN,
    POSITION_CONTROL_MODE,
    PROTOCOL_VERSION,
    RAD_PER_SEC_TO_DXL_VEL_FACTOR,
    SIGNED_8BIT_MAX,
    SIGNED_16BIT_MAX,
    SIGNED_32BIT_MAX,
    TORQUE_DISABLE_VAL,
    TORQUE_ENABLE_VAL,
    UNSIGNED_8BIT_MAX,
    UNSIGNED_16BIT_MAX,
    UNSIGNED_32BIT_MAX,
    VELOCITY_CONTROL_MODE,
    VELOCITY_UNIT_RPM,
    WHEEL_COMMAND_RADIUS,
    WHEEL_ODOM_RADIUS,
    WHEEL_SEPARATION,
    WHEEL_TORQUE_DISABLE_ON_SHUTDOWN,
)
from eclipse_pkg.heading_hold import (
    calculate_heading_hold_correction,
    quaternion_to_yaw,
)
from eclipse_pkg.drive_control import (
    apply_navigation_stop,
    block_autonomy_reverse,
    scale_to_wheel_limit,
)
from eclipse_pkg.dxl_bus import DxlBus
from eclipse_pkg.height_ai_arbitration import arbitrate_height_ai_command
from eclipse_pkg.height_ai_policy import HeightObservation, load_policy
from eclipse_pkg.height_table import clamp_height_down_mm, height_ticks_for_down_mm
from eclipse_pkg.wheel_odometry import (
    differential_drive_twist,
    integrate_pose,
)
from eclipse_pkg_msgs.msg import PresentCurrent
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import (
    Bool,
    Float32,
    Float32MultiArray,
    Int32,
    Int32MultiArray,
    String,
)
from tf2_ros import TransformBroadcaster


ADDR_OPERATING_MODE = 11
ADDR_TEMPERATURE_LIMIT = 31
ADDR_PWM_LIMIT = 36
ADDR_SHUTDOWN = 63
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_LED = 65
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_VELOCITY_I_GAIN = 76
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_BUS_WATCHDOG = 98
ADDR_PRESENT_PWM = 124
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_VELOCITY_LIMIT = 44

LEN_VELOCITY_GAINS = 4
LEN_GOAL_VELOCITY = 4
LEN_GOAL_POSITION = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_BUS_WATCHDOG = 1
LEN_PRESENT_PWM = 2
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_INPUT_VOLTAGE = 2
LEN_PRESENT_TEMPERATURE = 1
LEN_PRESENT_VOLTAGE_TEMPERATURE = 3
LEN_VELOCITY_LIMIT = 4

HEIGHT_STATE_FIXED = "FIXED_45MM"
HEIGHT_STATE_MANUAL = "MANUAL_HEIGHT"
HEIGHT_STATE_AI = "AI_HEIGHT"
HEIGHT_STATE_TEMP_STOP = "TEMP_STOP"

# Height Dynamixel IDs (1/11) missing or bus noise: repeated read4ByteTxRx
# timeouts flood the serial line and inflate load.
HEIGHT_BUS_FAIL_LATCH_COUNT = 3
HEIGHT_BUS_PROBE_SEC = 5.0
HEIGHT_BUS_WARN_INTERVAL_SEC = 2.0

class EclipseTestController(Node):
    """Wheel drive node with fixed 45 mm height actuator initialization."""

    def __init__(self):
        super().__init__("eclipse_test_controller")

        # Nav2 자율주행 중에는 heading_hold가 경로 추종용 미세 회전 명령을
        # "직진 의도"로 오인해 되돌리려 하므로, 자율주행 중엔 꺼야 한다.
        # /drive/autonomy_active(gps_waypoint_commander가 실시간 발행)가 True인
        # 동안 heading_hold를 끈다: Nav2 자율주행 구간 전체가 여기 해당한다.
        self.autonomy_active = False
        self.autonomy_reverse_blocked = False
        self.navigation_stop_latched = False

        self.velocity_i_gain = DEFAULT_VELOCITY_I_GAIN
        self.velocity_p_gain = DEFAULT_VELOCITY_P_GAIN

        self.target_l = 0.0
        self.target_r = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_vel_time = None
        self.cmd_vel_timeout_active = False
        self.imu_yaw = 0.0
        self.has_imu_yaw = False
        self.heading_target_yaw = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_covariance_log_time = self.last_time

        self.current_buffer_l = deque(maxlen=BUFFER_SIZE)
        self.current_buffer_r = deque(maxlen=BUFFER_SIZE)
        self.std_l = 0.0
        self.std_r = 0.0
        self.current_std_exceed_count = 0
        self.current_rough_count = 0
        self.current_shock_active = False
        self.current_std_baseline = NORMAL_STD
        self.current_baseline_sample_count = 0
        self.prev_current_avg_std = None

        self.fixed_height_down_mm = float(FIXED_HEIGHT_DOWN_MM)
        self.height_state = HEIGHT_STATE_FIXED
        self.height_hold_latched_mm = None
        self.height_hold_inhibit_until = 0.0
        self.height_last_command_down_mm = float(FIXED_HEIGHT_DOWN_MM)
        self.height_move_down_delta = 0.0
        self.height_move_extreme = {dxl_id: None for dxl_id in DXL_HEIGHT_IDS}
        self.height_current_clamp_last = 0.0
        self.height_temp_fault = False
        self.height_temp_fault_id = None
        self.height_temp_fault_c = None
        # Height AI policy runs in-process (see height_ai_apply_loop). Declared
        # as parameters so a trained checkpoint can be swapped in without code
        # changes, and so description_ai.launch.py can disable it while the
        # dataset random-probe FSM owns the height actuator.
        self.declare_parameter('enable_height_ai', True)
        self.declare_parameter('height_ai_model_path', '')
        self.height_ai_enabled = bool(
            self.get_parameter('enable_height_ai').value
        )
        self.height_ai_policy = load_policy(
            str(self.get_parameter('height_ai_model_path').value),
            logger=self.get_logger(),
        )
        self.probe_angle = 0.0
        self.has_probe_angle = False
        self.probe_angle_recv_time = None
        self.height_bus_offline = False
        self.height_bus_fail_streak = 0
        self.height_bus_last_probe_at = 0.0
        self.height_bus_last_warn_at = 0.0
        self.height_bus_probe_active = False
        self.height_last_present = {dxl_id: None for dxl_id in DXL_HEIGHT_IDS}
        self.height_last_goal = {dxl_id: None for dxl_id in DXL_HEIGHT_IDS}
        self.height_active_ids = DXL_HEIGHT_IDS
        self.ekf_speed = 0.0
        self.wheel_odom_speed = 0.0
        self.traction_efficiency = 0.0
        self.has_filtered_odom = False
        self.gps_fix_status = None
        self.last_gps_fix_time = None
        self.last_gps_velocity_time = None
        self.gps_velocity_speed = 0.0
        self.motor_safety_fault = False
        self.motor_safety_state = "OK"
        self.motor_safety_speed_scale = 1.0
        self.motor_safety_read_failures = 0
        self.motor_stall_started_at = None
        self.last_motor_safety_warn_log_at = 0.0
        self.wheel_velocity_limit_ticks = [0 for _ in DXL_ALL_IDS]
        self.last_goal_velocity_ticks = [0 for _ in DXL_ALL_IDS]
        self.last_target_speed_mps = [0.0 for _ in DXL_ALL_IDS]

        self.camera_pan_enabled = False
        self.camera_pan_min = None
        self.camera_pan_max = None
        self.camera_pan_target = None

        self.front_bus = DxlBus(
            "front",
            DEVICENAME_FRONT,
            DXL_FRONT_WHEEL_IDS,
            DXL_HEIGHT_FRONT_IDS,
            PROTOCOL_VERSION,
            extra_ids=DXL_CAMERA_PAN_IDS,
        )
        self.rear_bus = DxlBus(
            "rear",
            DEVICENAME_REAR,
            DXL_REAR_WHEEL_IDS,
            DXL_HEIGHT_REAR_IDS,
            PROTOCOL_VERSION,
        )
        group_addrs = {
            "goal_velocity": (ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY),
            "velocity_gains": (ADDR_VELOCITY_I_GAIN, LEN_VELOCITY_GAINS),
            "goal_position": (ADDR_GOAL_POSITION, LEN_GOAL_POSITION),
            "present_velocity": (ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY),
            "present_current": (ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT),
            "hardware_error": (
                ADDR_HARDWARE_ERROR_STATUS,
                LEN_HARDWARE_ERROR_STATUS,
            ),
            "bus_watchdog": (ADDR_BUS_WATCHDOG, LEN_BUS_WATCHDOG),
            "present_pwm": (ADDR_PRESENT_PWM, LEN_PRESENT_PWM),
            "voltage_temperature": (
                ADDR_PRESENT_INPUT_VOLTAGE,
                LEN_PRESENT_VOLTAGE_TEMPERATURE,
            ),
        }
        self.front_bus.build_groups(**group_addrs)
        self.rear_bus.build_groups(**group_addrs)

        self.open_dynamixel_bus()
        self.configure_dynamixels()
        self.wheel_velocity_limit_ticks = self.read_wheel_velocity_limits()
        # Binding limit for saturation scaling: the slowest wheel governs, since
        # exceeding any one of them is what distorts the arc.
        # read_wheel_velocity_limits stores 0 for a wheel whose register read
        # failed. Dropping those matters because scale_to_wheel_limit treats a
        # 0 limit as "unknown" and skips scaling entirely — so a plain min()
        # would let ONE failed read silently disable the protection for all four
        # wheels. (It would not stop the robot; it would just hand saturation
        # back to the firmware, which clips only the faster wheel.)
        usable_limits = [t for t in self.wheel_velocity_limit_ticks if t > 0]
        self.wheel_limit_ticks = float(min(usable_limits)) if usable_limits else 0.0
        if not usable_limits:
            self.get_logger().warn(
                'no usable Dynamixel Velocity Limit was read; wheel saturation '
                'scaling is disabled and the firmware will clip the faster '
                'wheel on its own (turn radius may drift under saturation)'
            )
        self.create_ros_interfaces()

        self.create_timer(CONTROL_DT, self.feedback_loop)
        self.create_timer(HEIGHT_STATUS_DT, self.publish_height_status)
        self.create_timer(HEIGHT_VOLTAGE_STATUS_DT, self.publish_height_input_voltage)
        self.create_timer(HEIGHT_AI_APPLY_DT, self.height_ai_apply_loop)
        self.create_timer(MOTOR_SAFETY_DT, self.motor_safety_loop)
        self.publish_height_status()
        self.get_logger().info(
            "eclipse-test controller ready: fixed height down=%.1f mm"
            % self.fixed_height_down_mm
        )

    # ------------------------------------------------------------------
    # Startup configuration
    # ------------------------------------------------------------------
    def iter_buses(self):
        return (self.front_bus, self.rear_bus)

    def bus_for(self, dxl_id):
        name = dxl_bus_name(dxl_id)
        if name == "front":
            return self.front_bus
        if name == "rear":
            return self.rear_bus
        raise KeyError(f"Dynamixel ID {dxl_id} is not mapped to a bus")

    def open_dynamixel_bus(self):
        opened = []
        try:
            self.front_bus.open(BAUDRATE)
            opened.append(self.front_bus)
            self.rear_bus.open(BAUDRATE)
            opened.append(self.rear_bus)
        except Exception:
            for bus in opened:
                bus.close()
            raise
        self.get_logger().info(
            f"dynamixel buses open: front={DEVICENAME_FRONT} "
            f"ids={self.front_bus.ids} rear={DEVICENAME_REAR} "
            f"ids={self.rear_bus.ids}"
        )
        self.discover_height_motors()

    def height_command_ids(self):
        return getattr(self, "height_active_ids", DXL_HEIGHT_IDS)

    def ping_dxl(self, dxl_id):
        try:
            return self.bus_for(dxl_id).ping(dxl_id)
        except Exception as exc:
            self.get_logger().warn(f"dxl ping exception for ID {dxl_id}: {exc}")
            return False

    def _sync_write_map(self, group_name, value_by_id, context):
        ok = True
        for bus in self.iter_buses():
            items = [
                (dxl_id, data)
                for dxl_id, data in value_by_id.items()
                if bus.owns(dxl_id)
            ]
            if not items:
                continue
            group = getattr(bus, group_name)
            group.clearParam()
            try:
                bus_ok = True
                for dxl_id, data in items:
                    if not group.addParam(dxl_id, data):
                        self.get_logger().warn(
                            f"{context} addParam failed for ID {dxl_id} "
                            f"on {bus.name}"
                        )
                        bus_ok = False
                        ok = False
                if bus_ok:
                    result = group.txPacket()
                    if result != COMM_SUCCESS:
                        self.get_logger().warn(
                            f"{context} sync write failed on {bus.name}: "
                            f"{bus.tx_result_text(result)}"
                        )
                        ok = False
            except Exception as exc:
                self.get_logger().warn(
                    f"{context} sync write exception on {bus.name}: {exc}"
                )
                ok = False
            finally:
                group.clearParam()
        return ok

    def discover_height_motors(self):
        active = []
        for dxl_id in DXL_HEIGHT_IDS:
            answered = self.ping_dxl(dxl_id)
            if dxl_id in DXL_HEIGHT_OPTIONAL_IDS and not answered:
                self.get_logger().warn(
                    f"height pair ID {dxl_id} did not ping; "
                    "axis will run without this motor until restart"
                )
                continue
            if not answered:
                self.get_logger().error(f"required height ID {dxl_id} did not ping")
            active.append(dxl_id)
        self.height_active_ids = tuple(active)
        self.get_logger().info(
            f"height motors active={self.height_active_ids} "
            f"(optional pair IDs {DXL_HEIGHT_OPTIONAL_IDS})"
        )

    def configure_dynamixels(self):
        for dxl_id in DXL_ALL_IDS:
            self.write_dxl_byte(
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE_VAL,
                "wheel torque disable before mode set",
            )
            self.write_dxl_byte(
                dxl_id,
                ADDR_OPERATING_MODE,
                VELOCITY_CONTROL_MODE,
                "wheel operating mode set",
            )

        height_ids_requiring_reconfigure = []
        for dxl_id in self.height_command_ids():
            height_config_ok = self.height_startup_configuration_matches(dxl_id)
            if height_config_ok:
                self.get_logger().info(
                    f"height ID {dxl_id}: startup config valid; keeping torque state"
                )
                continue

            mode = self.read_dxl_byte(
                dxl_id,
                ADDR_OPERATING_MODE,
                "height operating mode read",
            )
            if (
                HEIGHT_TORQUE_HOLD_ON_SHUTDOWN
                and mode == POSITION_CONTROL_MODE
            ):
                self.get_logger().warn(
                    f"height ID {dxl_id}: startup config not verified; "
                    "preserving torque state and skipping EEPROM repair"
                )
                continue

            if HEIGHT_TORQUE_HOLD_ON_SHUTDOWN:
                self.get_logger().warn(
                    f"height ID {dxl_id}: operating mode {mode} != "
                    f"{POSITION_CONTROL_MODE}; torque off to restore "
                    "position control"
                )

            height_ids_requiring_reconfigure.append(dxl_id)
            self.write_dxl_byte(
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE_VAL,
                "height torque disable before mode/limit repair",
            )
            self.write_height_position_limits(dxl_id)
            self.write_dxl_byte(
                dxl_id,
                ADDR_OPERATING_MODE,
                POSITION_CONTROL_MODE,
                "height operating mode set",
            )

        self.configure_motor_safety_limits(height_ids_requiring_reconfigure)
        self.write_velocity_gains()
        self.write_height_position_gains()
        self.write_height_motion_profile()
        self.write_height_down_mm(self.fixed_height_down_mm)

        for dxl_id in DXL_ALL_IDS + self.height_command_ids():
            self.write_dxl_byte(
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE_VAL,
                "torque enable",
            )

        self.configure_camera_pan()
        self.enable_wheel_bus_watchdog()
        self.move_height_to_down_mm_verified(
            self.fixed_height_down_mm,
            "fixed height startup",
            HEIGHT_INITIALIZE_TIMEOUT_SEC,
        )

    def configure_camera_pan(self):
        """Use Wizard EEPROM limits. Do not write min/max registers."""
        self.camera_pan_enabled = False
        self.camera_pan_min = None
        self.camera_pan_max = None
        dxl_id = DXL_CAMERA_PAN_ID
        if not self.ping_dxl(dxl_id):
            self.get_logger().warn(
                f"camera pan ID {dxl_id} did not ping; pan topic disabled"
            )
            return
        mode = self.read_dxl_byte(
            dxl_id, ADDR_OPERATING_MODE, "camera pan operating mode read"
        )
        if mode != POSITION_CONTROL_MODE:
            self.write_dxl_byte(
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE_VAL,
                "camera pan torque off before mode set",
            )
            if not self.write_dxl_byte(
                dxl_id,
                ADDR_OPERATING_MODE,
                POSITION_CONTROL_MODE,
                "camera pan operating mode set",
            ):
                self.get_logger().error("camera pan: failed to set position mode")
                return
        min_pos = self.read_dxl_4byte(
            dxl_id, ADDR_MIN_POSITION_LIMIT, "camera pan min limit read"
        )
        max_pos = self.read_dxl_4byte(
            dxl_id, ADDR_MAX_POSITION_LIMIT, "camera pan max limit read"
        )
        if not camera_pan_limits_are_valid(min_pos, max_pos):
            self.get_logger().error(
                f"camera pan: invalid EEPROM limits min={min_pos} max={max_pos}"
            )
            return
        self.camera_pan_min = int(min_pos)
        self.camera_pan_max = int(max_pos)
        present = self.read_dxl_4byte(
            dxl_id, ADDR_PRESENT_POSITION, "camera pan present read"
        )
        self.write_dxl_4byte(
            dxl_id,
            ADDR_PROFILE_ACCELERATION,
            CAMERA_PAN_PROFILE_ACCELERATION,
            "camera pan profile acceleration",
        )
        self.write_dxl_4byte(
            dxl_id,
            ADDR_PROFILE_VELOCITY,
            CAMERA_PAN_PROFILE_VELOCITY,
            "camera pan profile velocity",
        )
        if not self.write_dxl_byte(
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE_VAL,
            "camera pan torque enable",
        ):
            self.get_logger().error("camera pan: torque enable failed")
            return
        home = camera_pan_startup_goal(
            self.camera_pan_min, self.camera_pan_max
        )
        if home is None:
            self.get_logger().error("camera pan: home goal clamp failed")
            return
        if home != CAMERA_PAN_HOME_TICKS:
            self.get_logger().warn(
                f"camera pan: home {CAMERA_PAN_HOME_TICKS} clamped to {home} "
                f"[{self.camera_pan_min}, {self.camera_pan_max}]"
            )
        if not self.write_dxl_4byte(
            dxl_id,
            ADDR_GOAL_POSITION,
            home,
            "camera pan home goal",
        ):
            self.get_logger().error("camera pan: home goal write failed")
            return
        self.camera_pan_target = home
        self.camera_pan_enabled = True
        self.get_logger().info(
            f"camera pan ready id={dxl_id} min={self.camera_pan_min} "
            f"max={self.camera_pan_max} present={present} home={home}"
        )

    def height_startup_configuration_matches(self, dxl_id):
        min_pos, max_pos = self.height_position_limits_for_id(dxl_id)
        checks = (
            ("operating mode", ADDR_OPERATING_MODE, 1, POSITION_CONTROL_MODE),
            ("max position limit", ADDR_MAX_POSITION_LIMIT, 4, max_pos),
            ("min position limit", ADDR_MIN_POSITION_LIMIT, 4, min_pos),
            ("temperature limit", ADDR_TEMPERATURE_LIMIT, 1, MOTOR_TEMPERATURE_LIMIT_C),
            ("shutdown mask", ADDR_SHUTDOWN, 1, MOTOR_SHUTDOWN_MASK),
        )

        ok = True
        for label, address, size, expected in checks:
            if size == 1:
                value = self.read_dxl_byte(
                    dxl_id,
                    address,
                    f"height startup {label} read",
                )
            else:
                value = self.read_height_register_4byte(
                    dxl_id,
                    address,
                    f"height startup {label} read",
                )

            if value is None:
                ok = False
                continue

            if int(value) != int(expected):
                self.get_logger().warn(
                    f"height ID {dxl_id}: startup {label} {value} "
                    f"!= expected {expected}; will reconfigure with torque off"
                )
                ok = False

        return ok

    def create_ros_interfaces(self):
        gps_sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(
            Bool,
            "/drive/autonomy_active",
            self.autonomy_active_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/drive/navigation_stop",
            self.navigation_stop_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.filtered_odom_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_fix_callback,
            gps_sensor_qos,
        )
        self.create_subscription(
            Odometry,
            "/odometry/gps_velocity",
            self.gps_velocity_callback,
            10,
        )
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(
            Float32,
            "/drive/height_step_mm",
            self.height_step_callback,
            10,
        )
        self.create_subscription(
            Int32,
            "/camera/pan/command",
            self.camera_pan_command_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/probe/angle",
            self.probe_angle_callback,
            10,
        )

        self.current_pub = self.create_publisher(PresentCurrent, "/present_current", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.odom_tf_broadcaster = TransformBroadcaster(self)
        self.motor_safety_state_pub = self.create_publisher(
            String,
            "/motor/safety_state",
            10,
        )

        self.height_state_pub = self.create_publisher(String, "/height/state", 10)
        self.height_target_pub = self.create_publisher(
            Int32,
            "/height/target_position",
            10,
        )
        self.height_present_pub = self.create_publisher(
            Int32,
            "/height/present_position",
            10,
        )
        self.height_front_target_pub = self.create_publisher(
            Int32,
            "/height/front/target_position",
            10,
        )
        self.height_rear_target_pub = self.create_publisher(
            Int32,
            "/height/rear/target_position",
            10,
        )
        self.height_front_present_pub = self.create_publisher(
            Int32,
            "/height/front/present_position",
            10,
        )
        self.height_rear_present_pub = self.create_publisher(
            Int32,
            "/height/rear/present_position",
            10,
        )
        self.height_front_goal_pub = self.create_publisher(
            Int32,
            "/height/front/goal_position",
            10,
        )
        self.height_rear_goal_pub = self.create_publisher(
            Int32,
            "/height/rear/goal_position",
            10,
        )
        self.height_down_pub = self.create_publisher(Float32, "/height/down_mm", 10)
        self.dxl_height_ids_pub = self.create_publisher(
            Int32MultiArray, "/dxl/height/ids", 10
        )
        self.dxl_height_present_current_ma_pub = self.create_publisher(
            Float32MultiArray, "/dxl/height/present_current_ma", 10
        )
        self.dxl_height_present_temperature_c_pub = self.create_publisher(
            Float32MultiArray, "/dxl/height/present_temperature_c", 10
        )
        self.dxl_height_present_position_pub = self.create_publisher(
            Int32MultiArray, "/dxl/height/present_position", 10
        )
        self.dxl_height_hardware_error_status_pub = self.create_publisher(
            Int32MultiArray, "/dxl/height/hardware_error_status", 10
        )
        self.dxl_wheel_hardware_error_status_pub = self.create_publisher(
            Int32MultiArray, "/dxl/wheel/hardware_error_status", 10
        )
        self.dxl_height_present_input_voltage_v_pub = self.create_publisher(
            Float32MultiArray, "/dxl/height/present_input_voltage_v", 10
        )
        self.dxl_wheel_ids_pub = self.create_publisher(Int32MultiArray, "/dxl/wheel/ids", 10)
        self.dxl_goal_velocity_ticks_pub = self.create_publisher(Int32MultiArray, "/dxl/wheel/goal_velocity_ticks", 10)
        self.dxl_present_velocity_ticks_pub = self.create_publisher(Int32MultiArray, "/dxl/wheel/present_velocity_ticks", 10)
        self.dxl_velocity_limit_ticks_pub = self.create_publisher(Int32MultiArray, "/dxl/wheel/velocity_limit_ticks", 10)
        self.dxl_present_pwm_ticks_pub = self.create_publisher(Int32MultiArray, "/dxl/wheel/present_pwm_ticks", 10)
        self.dxl_target_speed_mps_pub = self.create_publisher(Float32MultiArray, "/dxl/wheel/target_speed_mps", 10)
        self.dxl_present_speed_mps_pub = self.create_publisher(Float32MultiArray, "/dxl/wheel/present_speed_mps", 10)
        self.dxl_present_current_ma_pub = self.create_publisher(Float32MultiArray, "/dxl/wheel/present_current_ma", 10)
        self.camera_pan_present_pub = self.create_publisher(
            Int32, "/camera/pan/present_position", 10
        )
        self.camera_pan_goal_pub = self.create_publisher(
            Int32, "/camera/pan/goal_position", 10
        )
        self.camera_pan_min_pub = self.create_publisher(
            Int32, "/camera/pan/min_position", 10
        )
        self.camera_pan_max_pub = self.create_publisher(
            Int32, "/camera/pan/max_position", 10
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        # if self.motor_safety_fault:
        #     self.reset_drive_command()
        #     return

        # Nav2 and manual commands both use the standard base_link convention:
        # +linear.x is forward and -linear.x is reverse.
        requested_v = float(msg.linear.x)
        requested_w = float(msg.angular.z)
        requested_v, requested_w = apply_navigation_stop(
            requested_v,
            requested_w,
            self.navigation_stop_latched,
        )
        v = block_autonomy_reverse(requested_v, self.autonomy_active)
        reverse_blocked = self.autonomy_active and requested_v < 0.0
        if reverse_blocked and not self.autonomy_reverse_blocked:
            self.get_logger().warning(
                'Blocked reverse /cmd_vel during autonomous navigation.'
            )
        self.autonomy_reverse_blocked = reverse_blocked
        w = requested_w
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout_active = False
        self.cmd_v = v
        self.cmd_w = w
        if abs(w) >= HEADING_HOLD_CMD_W_THRESHOLD or abs(v) < MIN_CMD_VEL:
            self.heading_target_yaw = None

        self.target_l = v - (w * WHEEL_SEPARATION / 2.0)
        self.target_r = v + (w * WHEEL_SEPARATION / 2.0)

    def autonomy_active_callback(self, msg):
        self.autonomy_active = bool(msg.data)

    def navigation_stop_callback(self, msg):
        self.navigation_stop_latched = bool(msg.data)
        if self.navigation_stop_latched:
            self.reset_drive_command()

    def reset_drive_command(self):
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.target_l = 0.0
        self.target_r = 0.0
        self.heading_target_yaw = None

    def enforce_cmd_vel_timeout(self):
        if self.last_cmd_vel_time is None:
            return

        now = self.get_clock().now()
        age_sec = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if age_sec <= CMD_VEL_TIMEOUT_SEC:
            return

        self.reset_drive_command()
        if not self.cmd_vel_timeout_active:
            self.get_logger().warn(
                f"/cmd_vel timeout: no command for {age_sec:.2f}s; "
                "forcing wheel command to zero"
            )
            self.cmd_vel_timeout_active = True

    def filtered_odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.ekf_speed = math.sqrt(vx**2 + vy**2)
        self.has_filtered_odom = True

    def gps_fix_callback(self, msg):
        self.gps_fix_status = int(msg.status.status)
        self.last_gps_fix_time = self.get_clock().now()

    def gps_velocity_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.gps_velocity_speed = math.sqrt(vx**2 + vy**2)
        self.last_gps_velocity_time = self.get_clock().now()

    def is_gps_velocity_good(self, current_time):
        if not gps_fix_status_is_good(
            self.gps_fix_status,
            GPS_MIN_GOOD_FIX_STATUS,
        ):
            return False
        if self.last_gps_fix_time is None or self.last_gps_velocity_time is None:
            return False

        fix_age = self.gps_fix_age_sec(current_time)
        velocity_age = self.gps_velocity_age_sec(current_time)
        return (
            fix_age <= GPS_FIX_TIMEOUT_SEC
            and velocity_age <= GPS_VELOCITY_TIMEOUT_SEC
        )

    def gps_fix_age_sec(self, current_time):
        if self.last_gps_fix_time is None:
            return math.inf
        return (current_time - self.last_gps_fix_time).nanoseconds / 1e9

    def gps_velocity_age_sec(self, current_time):
        if self.last_gps_velocity_time is None:
            return math.inf
        return (current_time - self.last_gps_velocity_time).nanoseconds / 1e9

    def imu_callback(self, msg):
        if msg.orientation_covariance[0] < 0.0:
            return

        q = msg.orientation
        quat_norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        if quat_norm <= 0.0:
            return

        self.imu_yaw = quaternion_to_yaw(
            q.x / quat_norm,
            q.y / quat_norm,
            q.z / quat_norm,
            q.w / quat_norm,
        )
        self.has_imu_yaw = True

    def height_step_callback(self, msg):
        delta_down_mm = float(msg.data)
        if abs(delta_down_mm) <= 1e-6:
            return
        if self.height_temp_fault:
            self.get_logger().warn(
                "manual height step ignored: temperature stop latched "
                f"(ID {self.height_temp_fault_id} {self.height_temp_fault_c}C)",
                throttle_duration_sec=2.0,
            )
            return

        source_down_mm = self.fixed_height_down_mm
        target_down_mm = clamp_height_down_mm(source_down_mm + delta_down_mm)
        actual_delta_mm = target_down_mm - source_down_mm
        if abs(actual_delta_mm) <= 1e-6:
            self.get_logger().info(
                f"manual height step ignored at limit: "
                f"current_down={source_down_mm:.1f} mm "
                f"requested_delta={delta_down_mm:.1f} mm"
            )
            return

        self.fixed_height_down_mm = target_down_mm
        self.height_state = HEIGHT_STATE_MANUAL
        self.write_height_down_mm(target_down_mm)
        self.get_logger().info(
            f"manual height step: source_down={source_down_mm:.1f} mm "
            f"delta={actual_delta_mm:.1f} mm "
            f"target_down={target_down_mm:.1f} mm"
        )

    def camera_pan_command_callback(self, msg):
        if not self.camera_pan_enabled:
            self.get_logger().warn(
                "camera pan command ignored: servo not ready",
                throttle_duration_sec=2.0,
            )
            return
        goal = clamp_camera_pan_position(
            msg.data, self.camera_pan_min, self.camera_pan_max
        )
        if goal is None:
            self.get_logger().warn(
                "camera pan command ignored: limits invalid",
                throttle_duration_sec=2.0,
            )
            return
        if not self.write_dxl_4byte(
            DXL_CAMERA_PAN_ID,
            ADDR_GOAL_POSITION,
            goal,
            "camera pan goal",
        ):
            return
        self.camera_pan_target = goal
        if int(msg.data) != goal:
            self.get_logger().info(
                f"camera pan clamped {int(msg.data)} -> {goal} "
                f"[{self.camera_pan_min}, {self.camera_pan_max}]"
            )

    def publish_camera_pan_status(self):
        if self.camera_pan_min is not None:
            min_msg = Int32()
            min_msg.data = int(self.camera_pan_min)
            self.camera_pan_min_pub.publish(min_msg)
        if self.camera_pan_max is not None:
            max_msg = Int32()
            max_msg.data = int(self.camera_pan_max)
            self.camera_pan_max_pub.publish(max_msg)
        if self.camera_pan_target is not None:
            goal_msg = Int32()
            goal_msg.data = int(self.camera_pan_target)
            self.camera_pan_goal_pub.publish(goal_msg)
        if not self.camera_pan_enabled:
            return
        present = self.read_dxl_4byte(
            DXL_CAMERA_PAN_ID,
            ADDR_PRESENT_POSITION,
            "camera pan present publish",
        )
        if present is None:
            return
        present_msg = Int32()
        present_msg.data = int(present)
        self.camera_pan_present_pub.publish(present_msg)

    def height_ai_observation(self):
        """Build the policy input from this node's own attributes.

        Key names must stay aligned with height_ai_policy.HeightObservation.
        """
        probe_age_sec = (
            time.monotonic() - self.probe_angle_recv_time
            if self.probe_angle_recv_time is not None
            else None
        )
        return {
            # 주행 명령
            "cmd_v": self.cmd_v,
            "cmd_w": self.cmd_w,
            "cmd_v_active": abs(self.cmd_v) >= MIN_CMD_VEL,
            # 높이 상태
            "height_state": self.height_state,
            "height_current_down_mm": self.fixed_height_down_mm,
            "height_bus_offline": self.height_bus_offline,
            "height_temp_fault": self.height_temp_fault,
            # 속도·트랙션
            "ekf_speed": self.ekf_speed,
            "wheel_odom_speed": self.wheel_odom_speed,
            "traction_efficiency": self.traction_efficiency,
            "has_filtered_odom": self.has_filtered_odom,
            # 탐침 (학습 스냅샷과 같은 키)
            "probe_required": True,
            "has_probe_angle": self.has_probe_angle,
            "probe_angle": self.probe_angle,
            "probe_angle_age_sec": probe_age_sec,
            "probe_angle_fresh": self.probe_angle_is_fresh(),
            "probe_angle_stale_sec": HEIGHT_AI_PROBE_STALE_SEC,
            "probe_contact_detected": self.is_probe_contact_detected(),
            # 전류
            "current_std_l": self.std_l,
            "current_std_r": self.std_r,
            "current_std_avg": (self.std_l + self.std_r) / 2.0,
            "current_std_exceed_count": self.current_std_exceed_count,
            "current_rough_count": self.current_rough_count,
            "current_shock_active": self.current_shock_active,
            "current_std_baseline": self.current_std_baseline,
            "current_baseline_sample_count": self.current_baseline_sample_count,
            # 모터 안전
            "motor_safety_fault": self.motor_safety_fault,
            "motor_safety_state": self.motor_safety_state,
            "motor_safety_speed_scale": self.motor_safety_speed_scale,
            # GPS
            "gps_fix_status": self.gps_fix_status,
            "gps_velocity_speed": self.gps_velocity_speed,
        }

    def probe_angle_callback(self, msg):
        self.probe_angle = float(msg.data)
        self.has_probe_angle = True
        self.probe_angle_recv_time = time.monotonic()

    def probe_angle_is_fresh(self):
        if self.probe_angle_recv_time is None:
            return False
        return (time.monotonic() - self.probe_angle_recv_time) <= HEIGHT_AI_PROBE_STALE_SEC

    def is_probe_contact_detected(self):
        if not self.has_probe_angle or not self.probe_angle_is_fresh():
            return False
        diff = abs(self.probe_angle - HEIGHT_AI_PROBE_CONTACT_ANGLE)
        diff = min(diff, 360.0 - diff)
        return diff <= HEIGHT_AI_PROBE_CONTACT_TOLERANCE

    def height_ai_apply_loop(self):
        # Policy output reaches the servo only through arbitrate_height_ai_command():
        # manual height_state always wins; proposal is deadbanded, rate-limited,
        # and mm-clamped twice.
        if not self.height_ai_enabled:
            return
        if self.height_temp_fault:
            return

        try:
            proposal = self.height_ai_policy.propose(
                HeightObservation(
                    current_down_mm=self.fixed_height_down_mm,
                    height_state=self.height_state,
                    stamp_sec=self.get_clock().now().nanoseconds / 1e9,
                    state=self.height_ai_observation(),
                )
            )
            proposal_down_mm = proposal.down_mm
        except Exception as exc:
            # Never let a policy defect take down the Dynamixel bus owner.
            # Holding the current height is the same no-op the stub produces.
            self.get_logger().error(
                f"height AI policy failed ({exc}); holding current height",
                throttle_duration_sec=5.0,
            )
            return

        result = arbitrate_height_ai_command(
            current_down_mm=self.fixed_height_down_mm,
            height_state=self.height_state,
            manual_state=HEIGHT_STATE_MANUAL,
            proposal_down_mm=proposal_down_mm,
            # Always fresh: the proposal was computed from live attributes in
            # this very tick, so there is no transport that could make it stale.
            proposal_fresh=True,
            dt_sec=HEIGHT_AI_APPLY_DT,
            max_rate_mm_per_s=HEIGHT_AI_MAX_RATE_MM_PER_S,
            deadband_mm=HEIGHT_AI_DEADBAND_MM,
        )
        if not result.apply:
            return

        self.fixed_height_down_mm = result.target_down_mm
        self.height_state = HEIGHT_STATE_AI
        self.write_height_down_mm(result.target_down_mm)
        self.get_logger().info(
            f"height AI applied: target_down={result.target_down_mm:.1f} mm "
            f"reason={result.reason} source={proposal.source}",
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------
    # Main wheel loop
    # ------------------------------------------------------------------
    def feedback_loop(self):
        # If either bus bulk-read fails, skip the rest of this tick and zero
        # both boards — do not drive on two of four wheels.
        front_vel = self.front_bus.sync_read_vel.txRxPacket()
        rear_vel = self.rear_bus.sync_read_vel.txRxPacket()
        front_ok = front_vel == COMM_SUCCESS
        rear_ok = rear_vel == COMM_SUCCESS
        if not wheel_bulk_read_is_usable(front_ok, rear_ok):
            if not front_ok:
                self.get_logger().warn(
                    "front wheel velocity sync read failed: "
                    f"{self.front_bus.tx_result_text(front_vel)}",
                    throttle_duration_sec=2.0,
                )
            if not rear_ok:
                self.get_logger().warn(
                    "rear wheel velocity sync read failed: "
                    f"{self.rear_bus.tx_result_text(rear_vel)}",
                    throttle_duration_sec=2.0,
                )
            # Keep odom->base_link TF alive when the motor bus is down: the
            # EKF's map->odom chain needs this transform to bootstrap, so Nav2
            # initialization must not be gated on the motors being powered.
            # Pose and stamp are the last integrated values (unchanged), not a
            # stale pose re-stamped with now() — see publish_odom_tf().
            self.stop_wheels()
            self.publish_odom_tf(self.last_time)
            return
        for bus in self.iter_buses():
            bus.sync_read_pwm.txRxPacket()
            bus.sync_read_current.txRxPacket()

        left_feedback = [
            self.read_wheel_feedback(
                dxl_id, reverse=dxl_wheel_goal_sign(dxl_id) < 0
            )
            for dxl_id in DXL_LEFT_IDS
        ]
        right_feedback = [
            self.read_wheel_feedback(
                dxl_id, reverse=dxl_wheel_goal_sign(dxl_id) < 0
            )
            for dxl_id in DXL_RIGHT_IDS
        ]
        wheel_feedback = left_feedback + right_feedback
        present_pwm_ticks = [
            self.read_wheel_present_pwm(dxl_id)
            for dxl_id in DXL_ALL_IDS
        ]

        v_l = self.average([feedback[0] for feedback in left_feedback])
        v_r = self.average([feedback[0] for feedback in right_feedback])
        cu_l = self.average([feedback[1] for feedback in left_feedback])
        cu_r = self.average([feedback[1] for feedback in right_feedback])

        self.current_buffer_l.append(cu_l)
        self.current_buffer_r.append(cu_r)

        self.enforce_cmd_vel_timeout()
        self.write_wheel_velocity_commands()

        v_linear, v_angular = differential_drive_twist(
            v_l,
            v_r,
            WHEEL_SEPARATION,
        )
        self.wheel_odom_speed = abs(v_linear)
        self.traction_efficiency = self.calculate_traction_efficiency()

        current_time = self.get_clock().now()
        dt_sec = (current_time - self.last_time).nanoseconds / 1e9
        if dt_sec <= 0.0:
            return
        self.last_time = current_time

        self.x, self.y, self.th = integrate_pose(
            self.x,
            self.y,
            self.th,
            v_linear,
            v_angular,
            dt_sec,
        )

        # Covariance recompute at COV_UPDATE_DT (wall time). Current samples
        # still append every CONTROL_DT tick into BUFFER_SIZE (time-windowed).
        now_sec = current_time.nanoseconds / 1e9
        if not hasattr(self, '_last_cov_log_t'):
            self._last_cov_log_t = 0.0
            self._last_dynamic_cov = 0.0
        if now_sec - self._last_cov_log_t >= COV_UPDATE_DT:
            self._last_cov_log_t = now_sec
            self._last_dynamic_cov = (
                self.calculate_and_log_dynamic_covariance(current_time)
            )
        dynamic_cov = self._last_dynamic_cov
        self.publish_odometry(current_time, v_linear, v_angular, dynamic_cov)
        self.publish_odom_tf(current_time)
        self.publish_present_current(cu_l, cu_r)
        self.publish_wheel_debug(
            self.last_goal_velocity_ticks,
            [feedback[2] for feedback in wheel_feedback],
            self.wheel_velocity_limit_ticks,
            present_pwm_ticks,
            self.last_target_speed_mps,
            [feedback[0] for feedback in wheel_feedback],
            [feedback[1] for feedback in wheel_feedback],
        )

    def write_wheel_velocity_commands(self):
        # if self.motor_safety_fault:
        #     return

        if self.autonomy_active:
            heading_correction_w = 0.0
        else:
            heading_correction_w, self.heading_target_yaw = (
                calculate_heading_hold_correction(
                    self.cmd_v,
                    self.cmd_w,
                    self.has_imu_yaw,
                    self.imu_yaw,
                    self.heading_target_yaw,
                    MIN_CMD_VEL,
                    HEADING_HOLD_CMD_W_THRESHOLD,
                    HEADING_HOLD_KP,
                    HEADING_HOLD_MAX_W,
                )
            )
        corrected_target_l = (
            self.target_l - (heading_correction_w * WHEEL_SEPARATION / 2.0)
        )
        corrected_target_r = (
            self.target_r + (heading_correction_w * WHEEL_SEPARATION / 2.0)
        )
        # corrected_target_l *= self.motor_safety_speed_scale
        # corrected_target_r *= self.motor_safety_speed_scale

        raw_cmd_l = (
            (corrected_target_l / WHEEL_COMMAND_RADIUS / MOTOR_TO_WHEEL_SPEED_RATIO)
            * RAD_PER_SEC_TO_DXL_VEL_FACTOR
        )
        raw_cmd_r = (
            (corrected_target_r / WHEEL_COMMAND_RADIUS / MOTOR_TO_WHEEL_SPEED_RATIO)
            * RAD_PER_SEC_TO_DXL_VEL_FACTOR
        )
        # Saturation must be handled HERE, before the firmware clips only the
        # faster wheel and quietly changes the commanded arc. See
        # scale_to_wheel_limit(). The limit is the smallest of the four wheels'
        # own Velocity Limit registers, read at startup.
        scaled_cmd_l, scaled_cmd_r, wheel_scale = scale_to_wheel_limit(
            raw_cmd_l, raw_cmd_r, self.wheel_limit_ticks
        )
        if wheel_scale < 1.0:
            self.get_logger().warn(
                'wheel command saturated: scaled both wheels by '
                f'{wheel_scale:.3f} to keep the commanded turn radius '
                f'(v={self.cmd_v:.3f} w={self.cmd_w:.3f})',
                throttle_duration_sec=2.0,
            )
        dxl_cmd_l = int(scaled_cmd_l)
        dxl_cmd_r = int(scaled_cmd_r)
        self.last_goal_velocity_ticks = [
            dxl_wheel_goal_sign(dxl_id)
            * (dxl_cmd_l if dxl_id in DXL_LEFT_IDS else dxl_cmd_r)
            for dxl_id in DXL_ALL_IDS
        ]
        # Report what was actually commanded, not the pre-saturation intent —
        # otherwise this debug topic (and the traction/covariance work that
        # reads it) would overstate the speed whenever scaling kicked in.
        self.last_target_speed_mps = (
            [corrected_target_l * wheel_scale for _ in DXL_LEFT_IDS]
            + [corrected_target_r * wheel_scale for _ in DXL_RIGHT_IDS]
        )

        payload = {
            dxl_id: self.int32_to_dxl_bytes(tick)
            for dxl_id, tick in zip(DXL_ALL_IDS, self.last_goal_velocity_ticks)
        }
        self._sync_write_map("sync_write_vel", payload, "wheel velocity")

    # ------------------------------------------------------------------
    # Motor safety
    # ------------------------------------------------------------------
    def configure_motor_safety_limits(self, height_ids_for_eeprom=DXL_HEIGHT_IDS):
        height_ids_for_eeprom = tuple(height_ids_for_eeprom)
        for dxl_id in DXL_ALL_IDS + height_ids_for_eeprom:
            self.write_dxl_byte(
                dxl_id,
                ADDR_TEMPERATURE_LIMIT,
                MOTOR_TEMPERATURE_LIMIT_C,
                "motor temperature limit set",
            )
            self.write_dxl_byte(
                dxl_id,
                ADDR_SHUTDOWN,
                MOTOR_SHUTDOWN_MASK,
                "motor shutdown mask set",
            )

        for dxl_id in DXL_ALL_IDS:
            self.write_dxl_2byte(
                dxl_id,
                ADDR_PWM_LIMIT,
                MOTOR_PWM_LIMIT,
                "wheel pwm limit set",
            )
            self.write_dxl_byte(
                dxl_id,
                ADDR_BUS_WATCHDOG,
                0,
                "wheel bus watchdog clear",
            )

        self.get_logger().info(
            "motor safety limits configured: "
            f"temp_limit={MOTOR_TEMPERATURE_LIMIT_C}C "
            f"pwm_limit={MOTOR_PWM_LIMIT} "
            f"height_eeprom_ids={height_ids_for_eeprom}"
        )

    def enable_wheel_bus_watchdog(self):
        watchdog_value = int(MOTOR_BUS_WATCHDOG)
        for dxl_id in DXL_ALL_IDS:
            self.write_dxl_byte(
                dxl_id,
                ADDR_BUS_WATCHDOG,
                watchdog_value,
                "wheel bus watchdog configure",
            )
        if watchdog_value > 0:
            self.get_logger().info(
                "wheel bus watchdog enabled: "
                f"{watchdog_value * MOTOR_BUS_WATCHDOG_UNIT_SEC * 1000:.0f} "
                "ms timeout"
            )
        else:
            self.get_logger().info("wheel bus watchdog disabled and cleared")

    def motor_safety_loop(self):
        # addr 70 로그/토픽은 높이와 같이 남긴다. 토크 차단은 끈 상태.
        self.publish_wheel_hardware_error_status()
        self.motor_safety_fault = False
        self.motor_safety_speed_scale = 1.0
        self.publish_motor_safety_state("OK")
        return
        # if self.motor_safety_fault:
        #     self.publish_motor_safety_state(self.motor_safety_state)
        #     return
        #
        # if not self.refresh_motor_safety_reads():
        #     self.handle_motor_safety_read_failure("WARN_SAFETY_READ_FAIL")
        #     return
        #
        # feedback = []
        # for dxl_id in DXL_ALL_IDS:
        #     if not self.motor_safety_feedback_available(dxl_id):
        #         self.handle_motor_safety_read_failure(
        #             f"WARN_SAFETY_DATA_MISSING_ID_{dxl_id}"
        #         )
        #         return
        #     feedback.append(self.read_motor_safety_feedback(dxl_id))
        #
        # self.motor_safety_read_failures = 0
        # fault_reason = self.find_motor_safety_fault(feedback)
        # if fault_reason:
        #     self.trigger_motor_safety_fault(fault_reason)
        #     return
        #
        # warn_state = self.find_motor_safety_warning(feedback)
        # self.publish_motor_safety_state(warn_state or "OK")
        # self.log_motor_safety_warning(warn_state)

    def refresh_motor_safety_reads(self):
        # present PWM은 feedback_loop가 이미 갱신한다.
        ok = True
        for bus in self.iter_buses():
            reads = (
                ("hardware error", bus.sync_read_hwerr),
                ("bus watchdog", bus.sync_read_watchdog),
                ("voltage/temperature", bus.sync_read_volt_temp),
            )
            for label, group_read in reads:
                dxl_comm_result = group_read.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(
                        f"motor safety {label} sync read failed on "
                        f"{bus.name}: {bus.tx_result_text(dxl_comm_result)}"
                    )
                    ok = False
        return ok

    def handle_motor_safety_read_failure(self, state_prefix):
        self.motor_safety_read_failures += 1
        state = f"{state_prefix}_{self.motor_safety_read_failures}"
        self.publish_motor_safety_state(state)
        if self.motor_safety_read_failures >= MOTOR_SAFETY_READ_FAILURE_LIMIT:
            self.trigger_motor_safety_fault("safety_read_failed")

    def motor_safety_feedback_available(self, dxl_id):
        bus = self.bus_for(dxl_id)
        checks = (
            (
                "hardware error",
                bus.sync_read_hwerr,
                ADDR_HARDWARE_ERROR_STATUS,
                LEN_HARDWARE_ERROR_STATUS,
            ),
            (
                "bus watchdog",
                bus.sync_read_watchdog,
                ADDR_BUS_WATCHDOG,
                LEN_BUS_WATCHDOG,
            ),
            (
                "present pwm",
                bus.sync_read_pwm,
                ADDR_PRESENT_PWM,
                LEN_PRESENT_PWM,
            ),
            (
                "input voltage",
                bus.sync_read_volt_temp,
                ADDR_PRESENT_INPUT_VOLTAGE,
                LEN_PRESENT_INPUT_VOLTAGE,
            ),
            (
                "temperature",
                bus.sync_read_volt_temp,
                ADDR_PRESENT_TEMPERATURE,
                LEN_PRESENT_TEMPERATURE,
            ),
        )
        for label, group_read, address, length in checks:
            if not group_read.isAvailable(dxl_id, address, length):
                self.get_logger().warn(
                    f"motor safety {label} data missing for ID {dxl_id}"
                )
                return False
        return True

    def read_motor_safety_feedback(self, dxl_id):
        bus = self.bus_for(dxl_id)
        hardware_error = bus.sync_read_hwerr.getData(
            dxl_id,
            ADDR_HARDWARE_ERROR_STATUS,
            LEN_HARDWARE_ERROR_STATUS,
        )
        bus_watchdog_raw = bus.sync_read_watchdog.getData(
            dxl_id,
            ADDR_BUS_WATCHDOG,
            LEN_BUS_WATCHDOG,
        )
        present_pwm_raw = bus.sync_read_pwm.getData(
            dxl_id,
            ADDR_PRESENT_PWM,
            LEN_PRESENT_PWM,
        )
        input_voltage_raw = bus.sync_read_volt_temp.getData(
            dxl_id,
            ADDR_PRESENT_INPUT_VOLTAGE,
            LEN_PRESENT_INPUT_VOLTAGE,
        )
        temperature_c = bus.sync_read_volt_temp.getData(
            dxl_id,
            ADDR_PRESENT_TEMPERATURE,
            LEN_PRESENT_TEMPERATURE,
        )

        return {
            "id": dxl_id,
            "hardware_error": hardware_error,
            "hardware_error_text": self.format_hardware_error(hardware_error),
            "bus_watchdog": self.signed_value(
                bus_watchdog_raw,
                SIGNED_8BIT_MAX,
                UNSIGNED_8BIT_MAX,
            ),
            "present_pwm": self.signed_value(
                present_pwm_raw,
                SIGNED_16BIT_MAX,
                UNSIGNED_16BIT_MAX,
            ),
            "input_voltage_v": input_voltage_raw * MOTOR_INPUT_VOLTAGE_UNIT_V,
            "temperature_c": temperature_c,
        }

    def find_motor_safety_fault(self, feedback):
        for item in feedback:
            if item["hardware_error"] != 0:
                return (
                    f"hardware_error_id_{item['id']}_"
                    f"{item['hardware_error_text']}"
                )
            if MOTOR_BUS_WATCHDOG > 0 and item["bus_watchdog"] < 0:
                return f"bus_watchdog_error_id_{item['id']}"
            if item["temperature_c"] >= MOTOR_TEMPERATURE_STOP_C:
                return (
                    f"temperature_stop_id_{item['id']}_"
                    f"{item['temperature_c']}C"
                )

        return self.find_stall_fault(feedback)

    def find_stall_fault(self, feedback):
        # 스톨 물리 임계 (eclipse_test_config):
        #   |cmd| >= MOTOR_STALL_MIN_CMD_MPS (0.05)
        #   wheel_odom_speed <= MOTOR_STALL_MAX_WHEEL_SPEED_MPS (0.02)
        #   max|PWM| >= MOTOR_PWM_STALL_THRESHOLD (500)
        # 후보 유지 중 → find_motor_safety_warning 이 WARN_STALL_PWM_* 발행
        # MOTOR_STALL_DURATION_SEC (1.5s) 이상 → FAULT_stall_... (토크 오프)
        # Recovery StallDetector 는 위 문자열의 "stall" 만 본다.
        max_pwm = max((abs(item["present_pwm"]) for item in feedback), default=0)
        stall_candidate = (
            abs(self.cmd_v) >= MOTOR_STALL_MIN_CMD_MPS
            and self.wheel_odom_speed <= MOTOR_STALL_MAX_WHEEL_SPEED_MPS
            and max_pwm >= MOTOR_PWM_STALL_THRESHOLD
        )
        now = time.monotonic()
        if not stall_candidate:
            self.motor_stall_started_at = None
            return None

        if self.motor_stall_started_at is None:
            self.motor_stall_started_at = now
            return None

        if now - self.motor_stall_started_at >= MOTOR_STALL_DURATION_SEC:
            return (
                "stall_pwm_high_no_motion_"
                f"pwm_{max_pwm}_speed_{self.wheel_odom_speed:.3f}"
            )
        return None

    def find_motor_safety_warning(self, feedback):
        hottest = max(feedback, key=lambda item: item["temperature_c"])
        max_pwm = max((abs(item["present_pwm"]) for item in feedback), default=0)
        voltage_warnings = [
            item
            for item in feedback
            if (
                item["input_voltage_v"] < MOTOR_INPUT_VOLTAGE_WARN_MIN_V
                or item["input_voltage_v"] > MOTOR_INPUT_VOLTAGE_WARN_MAX_V
            )
        ]

        self.motor_safety_speed_scale = 1.0
        for item in feedback:
            if item["bus_watchdog"] < 0:
                return f"WARN_BUS_WATCHDOG_ID_{item['id']}"
        if hottest["temperature_c"] >= MOTOR_TEMPERATURE_WARN_C:
            self.motor_safety_speed_scale = MOTOR_TEMP_WARN_SPEED_SCALE
            return (
                f"WARN_TEMP_ID_{hottest['id']}_"
                f"{hottest['temperature_c']}C"
            )
        # 스톨 후보 구간(DURATION 미달): recovery 가 잡는 "stall" 문자열 (WARN)
        if self.motor_stall_started_at is not None:
            return f"WARN_STALL_PWM_{max_pwm}"
        if voltage_warnings:
            item = voltage_warnings[0]
            return (
                f"WARN_VOLTAGE_ID_{item['id']}_"
                f"{item['input_voltage_v']:.1f}V"
            )
        return None

    def trigger_motor_safety_fault(self, reason):
        self.get_logger().warn(
            f"motor safety fault ignored (wheels 2/3/12/13): {reason}"
        )
        return
        # if self.motor_safety_fault:
        #     return
        #
        # self.motor_safety_fault = True
        # self.motor_safety_state = f"FAULT_{reason}"
        # self.motor_safety_speed_scale = 0.0
        # self.reset_drive_command()
        # self.stop_wheels()
        #
        # for dxl_id in DXL_ALL_IDS:
        #     self.write_dxl_byte(
        #         dxl_id,
        #         ADDR_TORQUE_ENABLE,
        #         TORQUE_DISABLE_VAL,
        #         "wheel safety torque disable",
        #     )
        #
        # self.publish_motor_safety_state(self.motor_safety_state)
        # self.get_logger().error(
        #     f"motor safety fault: {reason}; wheel torque disabled"
        # )

    def publish_motor_safety_state(self, state):
        self.motor_safety_state = state
        if not hasattr(self, "motor_safety_state_pub"):
            return

        msg = String()
        msg.data = state
        self.motor_safety_state_pub.publish(msg)

    def log_motor_safety_warning(self, warn_state):
        if not warn_state:
            return

        now = time.monotonic()
        elapsed = now - self.last_motor_safety_warn_log_at
        if elapsed < MOTOR_SAFETY_WARN_LOG_INTERVAL_SEC:
            return

        self.last_motor_safety_warn_log_at = now
        self.get_logger().warn(f"motor safety warning: {warn_state}")

    def format_hardware_error(self, value):
        names = [name for bit, name in HARDWARE_ERROR_NAMES if value & bit]
        return "none" if not names else "_".join(names)

    def read_hardware_error_status_raw(self, dxl_id):
        """Read addr 70 without check_dxl_result to avoid recursion."""
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, _dxl_error = bus.packet_handler.read1ByteTxRx(
                bus.port_handler,
                dxl_id,
                ADDR_HARDWARE_ERROR_STATUS,
            )
        except Exception:
            return None
        if dxl_comm_result != COMM_SUCCESS:
            return None
        return int(value) & 0xFF

    def publish_height_hardware_error_status(self):
        """Publish Hardware Error Status (addr 70) in ID order 11, 14, 1, 4."""
        values = []
        for dxl_id in DXL_HEIGHT_IDS:
            raw = self.read_hardware_error_status_raw(dxl_id)
            values.append(0 if raw is None else int(raw) & 0xFF)
        msg = Int32MultiArray()
        msg.data = values
        self.dxl_height_hardware_error_status_pub.publish(msg)
        if any(value != 0 for value in values):
            self.get_logger().error(
                format_height_hardware_error_log(DXL_HEIGHT_IDS, values),
                throttle_duration_sec=2.0,
            )

    def publish_wheel_hardware_error_status(self):
        """Publish Hardware Error Status (addr 70) in DXL_ALL_IDS order."""
        values = []
        for dxl_id in DXL_ALL_IDS:
            raw = self.read_hardware_error_status_raw(dxl_id)
            values.append(0 if raw is None else int(raw) & 0xFF)
        msg = Int32MultiArray()
        msg.data = values
        self.dxl_wheel_hardware_error_status_pub.publish(msg)
        if any(value != 0 for value in values):
            self.get_logger().error(
                format_wheel_hardware_error_log(DXL_ALL_IDS, values),
                throttle_duration_sec=2.0,
            )

    def read_height_input_voltage_v(self, dxl_id):
        if self.height_bus_offline and not self.height_bus_probe_active:
            return None
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read2ByteTxRx(
                bus.port_handler,
                dxl_id,
                ADDR_PRESENT_INPUT_VOLTAGE,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"height input voltage read exception for ID {dxl_id}: {exc}"
            )
            return None
        if not self.check_dxl_result(
            dxl_id,
            ADDR_PRESENT_INPUT_VOLTAGE,
            dxl_comm_result,
            dxl_error,
            "height input voltage read",
        ):
            return None
        return float(value) * MOTOR_INPUT_VOLTAGE_UNIT_V

    def publish_height_input_voltage(self):
        """Read Present Input Voltage (addr 144) every HEIGHT_VOLTAGE_STATUS_DT."""
        if self.height_bus_offline and not self.height_bus_probe_active:
            return
        values = []
        for dxl_id in DXL_HEIGHT_IDS:
            volts = self.read_height_input_voltage_v(dxl_id)
            values.append(0.0 if volts is None else float(volts))
        msg = Float32MultiArray()
        msg.data = values
        self.dxl_height_present_input_voltage_v_pub.publish(msg)
        self.get_logger().info(
            format_height_voltage_log(DXL_HEIGHT_IDS, values),
            throttle_duration_sec=5.0,
        )

    # ------------------------------------------------------------------
    # Odometry and covariance
    # ------------------------------------------------------------------
    def calculate_and_log_dynamic_covariance(self, current_time):
        if len(self.current_buffer_l) > 1:
            self.std_l = self.sample_std(self.current_buffer_l)
        if len(self.current_buffer_r) > 1:
            self.std_r = self.sample_std(self.current_buffer_r)

        avg_std = (self.std_l + self.std_r) / 2.0
        result = self.calculate_current_covariance(avg_std)
        wheel_odom_x_cov, gps_velocity_good = self.gps_adjusted_wheel_x_covariance(
            current_time,
            result.dynamic_cov,
        )
        self.log_dynamic_covariance(
            current_time,
            avg_std,
            result,
            gps_velocity_good,
            wheel_odom_x_cov,
        )
        return result.dynamic_cov

    def calculate_current_covariance(self, avg_std):
        state = CurrentCovarianceState(
            baseline=self.current_std_baseline,
            exceed_count=self.current_std_exceed_count,
            rough_count=self.current_rough_count,
            shock_active=self.current_shock_active,
            prev_avg_std=self.prev_current_avg_std,
            baseline_sample_count=self.current_baseline_sample_count,
        )
        result = calculate_current_covariance(
            avg_std=avg_std,
            state=state,
            config=CURRENT_COVARIANCE_CONFIG,
            samples_ready=(
                len(self.current_buffer_l) >= BUFFER_SIZE
                and len(self.current_buffer_r) >= BUFFER_SIZE
            ),
            cmd_v=self.cmd_v,
            cmd_w=self.cmd_w,
            wheel_odom_speed=self.wheel_odom_speed,
        )

        self.current_std_baseline = result.state.baseline
        self.current_std_exceed_count = result.state.exceed_count
        self.current_rough_count = result.state.rough_count
        self.current_shock_active = result.state.shock_active
        self.prev_current_avg_std = result.state.prev_avg_std
        self.current_baseline_sample_count = result.state.baseline_sample_count
        return result

    def gps_adjusted_wheel_x_covariance(self, current_time, dynamic_cov):
        gps_velocity_good = self.is_gps_velocity_good(current_time)
        wheel_odom_x_cov = apply_gps_good_wheel_covariance_floor(
            dynamic_cov,
            gps_velocity_good,
            GPS_GOOD_WHEEL_ODOM_COV,
            MAX_COV,
        )
        return wheel_odom_x_cov, gps_velocity_good

    def publish_odometry(self, current_time, v_linear, v_angular, dynamic_cov):
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = v_angular
        wheel_odom_x_cov, _ = self.gps_adjusted_wheel_x_covariance(
            current_time,
            dynamic_cov,
        )
        odom.twist.covariance[0] = wheel_odom_x_cov
        odom.twist.covariance[7] = BASE_COV
        odom.twist.covariance[35] = dynamic_cov

        self.odom_pub.publish(odom)

    def publish_odom_tf(self, stamp):
        """Publish odom->base_link right after the pose was integrated.

        Called from feedback_loop (CONTROL_DT) with the same stamp as
        publish_odometry. Faster than CONTROL_DT adds nothing: x/y/th only
        change when feedback_loop integrates them.
        """
        odom_tf = TransformStamped()
        odom_tf.header.stamp = stamp.to_msg()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id = "base_link"
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation.z = math.sin(self.th / 2.0)
        odom_tf.transform.rotation.w = math.cos(self.th / 2.0)
        self.odom_tf_broadcaster.sendTransform(odom_tf)

    def publish_present_current(self, cu_l, cu_r):
        curr_msg = PresentCurrent()
        curr_msg.data1 = float(cu_l)
        curr_msg.data2 = float(cu_r)
        self.current_pub.publish(curr_msg)

    def publish_wheel_debug(
        self,
        goal_velocity_ticks,
        present_velocity_ticks,
        velocity_limit_ticks,
        present_pwm_ticks,
        target_speed_mps,
        present_speed_mps,
        present_current_ma,
    ):
        int_topics = (
            (self.dxl_wheel_ids_pub, DXL_ALL_IDS),
            (self.dxl_goal_velocity_ticks_pub, goal_velocity_ticks),
            (self.dxl_present_velocity_ticks_pub, present_velocity_ticks),
            (self.dxl_velocity_limit_ticks_pub, velocity_limit_ticks),
            (self.dxl_present_pwm_ticks_pub, present_pwm_ticks),
        )
        for publisher, values in int_topics:
            msg = Int32MultiArray()
            msg.data = [int(value) for value in values]
            publisher.publish(msg)

        float_topics = (
            (self.dxl_target_speed_mps_pub, target_speed_mps),
            (self.dxl_present_speed_mps_pub, present_speed_mps),
            (self.dxl_present_current_ma_pub, present_current_ma),
        )
        for publisher, values in float_topics:
            msg = Float32MultiArray()
            msg.data = [float(value) for value in values]
            publisher.publish(msg)

    def log_dynamic_covariance(
        self,
        current_time,
        avg_std,
        result,
        gps_velocity_good,
        wheel_odom_x_cov,
    ):
        elapsed = (current_time - self.last_covariance_log_time).nanoseconds / 1e9
        if elapsed < ODOM_COV_LOG_INTERVAL:
            return
        self.last_covariance_log_time = current_time
        self.get_logger().info(
            "odom_cov: "
            f"avg_std={avg_std:.3f}, current_mode={result.current_mode}, "
            f"current_count={result.state.exceed_count}, "
            f"rough_count={result.state.rough_count}, "
            f"baseline={result.state.baseline:.3f}, "
            f"effective_std={result.effective_normal_std:.3f}, "
            f"baseline_updates={result.state.baseline_sample_count}, "
            f"baseline_updated={result.baseline_updated}, "
            f"baseline_reason={result.baseline_update_reason}, "
            f"current_delta={result.current_delta:.3f}, "
            f"current_raw_delta={result.current_raw_delta:.3f}, "
            f"ekf_speed={self.ekf_speed:.3f}, "
            f"wheel_speed={self.wheel_odom_speed:.3f}, "
            f"traction={self.traction_efficiency:.3f}, "
            f"fixed_height_mm={self.fixed_height_down_mm:.1f}, "
            f"dynamic_cov={result.dynamic_cov:.3f}, "
            f"gps_good={gps_velocity_good}, "
            f"gps_fix_status={self.gps_fix_status}, "
            f"gps_fix_age={self.gps_fix_age_sec(current_time):.3f}, "
            f"gps_speed={self.gps_velocity_speed:.3f}, "
            f"gps_velocity_age={self.gps_velocity_age_sec(current_time):.3f}, "
            f"wheel_x_cov={wheel_odom_x_cov:.3f}"
        )

    # ------------------------------------------------------------------
    # Height actuator utilities
    # ------------------------------------------------------------------
    def write_height_goal_map(self, target_positions):
        self.write_height_motion_profile()
        ok = True
        clamped = {
            dxl_id: self.clamp_height_position_for_id(dxl_id, position)
            for dxl_id, position in target_positions.items()
            if dxl_id in self.height_command_ids()
        }

        payload = {
            dxl_id: self.int32_to_dxl_bytes(position)
            for dxl_id, position in clamped.items()
        }
        if not self._sync_write_map(
            "sync_write_height", payload, "height position"
        ):
            ok = False
        return ok

    def write_height_positions(self, front_pos, rear_pos):
        raw_positions = height_target_positions(front_pos, rear_pos)
        return self.write_height_goal_map(raw_positions)

    def write_height_down_mm(self, down_mm):
        if self.height_temp_fault:
            self.get_logger().warn(
                "height command ignored: temperature stop latched "
                f"(ID {self.height_temp_fault_id} {self.height_temp_fault_c}C)",
                throttle_duration_sec=2.0,
            )
            return self.fixed_height_down_mm
        down_mm = clamp_height_down_mm(down_mm)
        prev_cmd = self.height_last_command_down_mm
        self.height_move_down_delta = float(down_mm) - float(prev_cmd)
        self.height_last_command_down_mm = float(down_mm)
        self.height_move_extreme = {
            dxl_id: None for dxl_id in self.height_command_ids()
        }
        self.height_hold_latched_mm = None
        self.height_hold_inhibit_until = time.monotonic() + HEIGHT_HOLD_INHIBIT_SEC
        front_pos, rear_pos = height_ticks_for_down_mm(down_mm)
        self.write_height_positions(front_pos, rear_pos)
        return down_mm

    def note_height_move_extreme(self, presents):
        delta = self.height_move_down_delta
        for dxl_id, pos in presents.items():
            self.height_move_extreme[dxl_id] = height_extreme_present(
                self.height_move_extreme.get(dxl_id),
                pos,
                dxl_id,
                delta,
            )

    def latch_height_hold_if_settled(self, present_positions):
        """If motors have stopped after inhibit, set Goal to each Present.

        Commands still copy the primary tick (4←1, 14←11). After the
        inhibit window, when every height motor |Present Velocity| is at
        the hold cap, each motor latches Goal=its own Present so a pair
        offset does not keep holding current. Window clamp still applies.
        One-shot until the next height command.
        """
        if not present_positions:
            return False
        if self.height_hold_latched_mm == self.fixed_height_down_mm:
            return True
        hold = {}
        velocities = []
        for dxl_id in self.height_command_ids():
            if dxl_id not in present_positions:
                return False
            present = height_hold_latch_present_for_id(
                dxl_id, present_positions
            )
            if present is None:
                return False
            min_pos, max_pos = self.height_position_limits_for_id(dxl_id)
            goal = height_hold_goal_from_present(present, min_pos, max_pos)
            if goal is None:
                return False
            velocity = self.read_height_velocity(dxl_id, "height hold velocity")
            velocities.append(velocity)
            hold[dxl_id] = goal
        if not height_hold_may_latch(
            time.monotonic(), self.height_hold_inhibit_until, velocities
        ):
            return False
        if not self.write_height_goal_map(hold):
            return False
        self.height_hold_latched_mm = self.fixed_height_down_mm
        return True

    def write_height_position_direct(self, dxl_id, position, context):
        try:
            bus = self.bus_for(dxl_id)
            dxl_comm_result, dxl_error = bus.packet_handler.write4ByteTxRx(
                bus.port_handler,
                dxl_id,
                ADDR_GOAL_POSITION,
                self.clamp_height_position_for_id(dxl_id, position),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context}: direct height goal write exception for ID {dxl_id}: "
                f"{exc}"
            )
            return False

        return self.check_dxl_result(
            dxl_id,
            ADDR_GOAL_POSITION,
            dxl_comm_result,
            dxl_error,
            context,
        )

    def read_height_current_ma(self, dxl_id):
        if self.height_bus_offline and not self.height_bus_probe_active:
            return None
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read2ByteTxRx(
                bus.port_handler,
                dxl_id,
                ADDR_PRESENT_CURRENT,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"height current read exception for ID {dxl_id}: {exc}"
            )
            return None
        if not self.check_dxl_result(
            dxl_id,
            ADDR_PRESENT_CURRENT,
            dxl_comm_result,
            dxl_error,
            "height current read",
        ):
            return None
        if value > 32767:
            value -= 65536
        return float(value) * CURRENT_UNIT_MA

    def publish_height_currents(self):
        """Publish current and temperature in ID order 11, 14, 1, 4.

        Returns (currents_ma dict, temps_c dict). Missing reads are 0.0
        on the topics/logs; temps keep None so the 65 C stop can ignore
        a failed read.
        """
        currents_ma = []
        temps_c = []
        raw_temps = {}
        for dxl_id in DXL_HEIGHT_IDS:
            ma = self.read_height_current_ma(dxl_id)
            currents_ma.append(0.0 if ma is None else ma)
            temp = self.read_height_temperature_c(dxl_id)
            raw_temps[dxl_id] = temp
            temps_c.append(0.0 if temp is None else float(temp))
        ids_msg = Int32MultiArray()
        ids_msg.data = [int(dxl_id) for dxl_id in DXL_HEIGHT_IDS]
        self.dxl_height_ids_pub.publish(ids_msg)
        cur_msg = Float32MultiArray()
        cur_msg.data = currents_ma
        self.dxl_height_present_current_ma_pub.publish(cur_msg)
        temp_msg = Float32MultiArray()
        temp_msg.data = temps_c
        self.dxl_height_present_temperature_c_pub.publish(temp_msg)
        self.get_logger().info(
            format_height_current_log(DXL_HEIGHT_IDS, currents_ma),
            throttle_duration_sec=2.0,
        )
        self.publish_height_hardware_error_status()
        return dict(zip(DXL_HEIGHT_IDS, currents_ma)), raw_temps

    def publish_height_presents(self, presents_by_id):
        values = [
            int(presents_by_id[dxl_id]) if presents_by_id.get(dxl_id) is not None else 0
            for dxl_id in DXL_HEIGHT_IDS
        ]
        pos_msg = Int32MultiArray()
        pos_msg.data = values
        self.dxl_height_present_position_pub.publish(pos_msg)

    def snap_height_goal_to_present(self, context):
        presents = {}
        for dxl_id in self.height_command_ids():
            pos = self.read_height_position_strict(dxl_id, context)
            if pos is None:
                return None
            min_pos, max_pos = self.height_position_limits_for_id(dxl_id)
            goal = height_hold_goal_from_present(pos, min_pos, max_pos)
            if goal is None:
                return None
            presents[dxl_id] = goal
        if not self.write_height_goal_map(presents):
            return None
        self.height_hold_latched_mm = self.fixed_height_down_mm
        return presents

    def clamp_height_hold_if_overcurrent(self, currents_ma):
        if not height_current_clamp_is_enabled():
            return
        peak = max((abs(ma) for ma in currents_ma.values()), default=0.0)
        if peak < HEIGHT_CURRENT_CLAMP_MA:
            return
        now = time.monotonic()
        if (now - self.height_current_clamp_last) < 1.0:
            return
        presents = self.snap_height_goal_to_present("height overcurrent hold")
        if not presents:
            return
        self.height_current_clamp_last = now
        self.get_logger().warn(
            f"height overcurrent {peak:.0f} mA >= {HEIGHT_CURRENT_CLAMP_MA:.0f}; "
            f"snapped Goal=Present {self.height_positions_summary(presents)}"
        )

    def read_height_temperature_c(self, dxl_id):
        if self.height_bus_offline and not self.height_bus_probe_active:
            return None
        return self.read_dxl_byte(
            dxl_id,
            ADDR_PRESENT_TEMPERATURE,
            "height temperature read",
        )

    def check_height_temperature_stop(self, temps_c=None):
        if self.height_bus_offline:
            return
        hottest_id = None
        hottest_c = None
        if temps_c is None:
            temps_c = {}
            for dxl_id in self.height_command_ids():
                temps_c[dxl_id] = self.read_height_temperature_c(dxl_id)
        for dxl_id in self.height_command_ids():
            temp_c = temps_c.get(dxl_id)
            if temp_c is None:
                continue
            if hottest_c is None or temp_c > hottest_c:
                hottest_c = temp_c
                hottest_id = dxl_id
        if not height_temperature_is_stop(hottest_c):
            return
        self.trigger_height_temperature_stop(hottest_id, hottest_c)

    def trigger_height_temperature_stop(self, dxl_id, temp_c):
        first = not self.height_temp_fault
        if first:
            self.height_temp_fault = True
            self.height_temp_fault_id = dxl_id
            self.height_temp_fault_c = temp_c
            self.height_state = HEIGHT_STATE_TEMP_STOP
            self.get_logger().error(
                f"height temperature stop: ID {dxl_id} {temp_c}C >= "
                f"{HEIGHT_TEMPERATURE_STOP_C}C; holding present, torque on; "
                "restart to clear"
            )
        presents = self.snap_height_goal_to_present("height temperature hold")
        if presents:
            return
        self.get_logger().error(
            "height temperature stop: failed to snap Goal=Present",
            throttle_duration_sec=2.0,
        )

    def move_height_to_down_mm_verified(self, down_mm, context, timeout_sec):
        if self.height_temp_fault:
            self.get_logger().warn(
                f"{context}: skipped, temperature stop latched "
                f"(ID {self.height_temp_fault_id} {self.height_temp_fault_c}C)"
            )
            return False
        down_mm = float(down_mm)
        front_pos, rear_pos = height_ticks_for_down_mm(down_mm)
        target_positions = {
            dxl_id: pos
            for dxl_id, pos in height_target_positions(front_pos, rear_pos).items()
            if dxl_id in self.height_command_ids()
        }

        present_positions = {}
        for attempt in range(1, HEIGHT_INITIALIZE_ATTEMPTS + 1):
            write_ok = self.write_height_positions(front_pos, rear_pos)
            goal_ok = self.height_goal_positions_match(target_positions, context)
            reached, present_positions = self.wait_for_height_positions(
                target_positions,
                timeout_sec,
                context,
            )
            if write_ok and goal_ok and reached:
                self.get_logger().info(
                    f"{context}: fixed height reached down={down_mm:.1f} mm "
                    f"target {self.height_positions_summary(target_positions)} "
                    f"present {self.height_positions_summary(present_positions)}"
                )
                self.latch_height_hold_if_settled(present_positions)
                return True

            if attempt < HEIGHT_INITIALIZE_ATTEMPTS:
                self.get_logger().warn(
                    f"{context}: retry {attempt + 1}/{HEIGHT_INITIALIZE_ATTEMPTS} "
                    f"for down={down_mm:.1f} mm"
                )
                time.sleep(HEIGHT_INITIALIZE_RETRY_DELAY_SEC)

        self.get_logger().warn(
            f"{context}: failed to verify fixed height down={down_mm:.1f} mm "
            f"target {self.height_positions_summary(target_positions)}"
        )
        self.latch_height_hold_if_settled(present_positions)
        return False

    def height_ids_for_targets(self, target_positions):
        active = set(self.height_command_ids())
        return [dxl_id for dxl_id in target_positions if dxl_id in active]

    def height_goal_positions_match(self, target_positions, context):
        ok = True
        for dxl_id in self.height_ids_for_targets(target_positions):
            target_pos = target_positions[dxl_id]
            goal_pos = self.read_height_goal_position(dxl_id, context)
            if goal_pos is None:
                ok = False
                continue
            if goal_pos != target_pos:
                self.get_logger().warn(
                    f"{context}: ID {dxl_id} goal register {goal_pos} "
                    f"!= target {target_pos}"
                )
                if not self.write_height_position_direct(dxl_id, target_pos, context):
                    ok = False
        return ok

    def wait_for_height_positions(self, target_positions, timeout_sec, context):
        deadline = time.monotonic() + timeout_sec
        last_positions = {}
        while time.monotonic() < deadline:
            reached, present_positions = self.height_positions_reached(
                target_positions,
                context,
            )
            if present_positions:
                last_positions = present_positions
            if reached:
                return True, present_positions
            time.sleep(HEIGHT_POSITION_POLL_SEC)

        self.get_logger().warn(
            f"{context}: timeout waiting for height positions; "
            f"target {self.height_positions_summary(target_positions)} "
            f"present {self.height_positions_summary(last_positions)}"
        )
        return False, last_positions

    def height_positions_reached(self, target_positions, context):
        present_positions = {}
        ok = True
        for dxl_id in self.height_ids_for_targets(target_positions):
            target_pos = target_positions[dxl_id]
            present_pos = self.read_height_position_strict(dxl_id, context)
            if present_pos is None:
                ok = False
                continue
            present_positions[dxl_id] = present_pos
            if abs(present_pos - target_pos) > HEIGHT_POSITION_TOLERANCE_TICKS:
                ok = False
        return ok, present_positions

    def read_height_positions(self):
        front_target, rear_target = height_ticks_for_down_mm(self.fixed_height_down_mm)
        front_pos = self.read_height_position(DXL_HEIGHT_FRONT_ID, front_target)
        rear_pos = self.read_height_position(DXL_HEIGHT_REAR_ID, rear_target)
        return front_pos, rear_pos

    def read_height_position(self, dxl_id, fallback_pos):
        pos = self.read_height_position_strict(dxl_id, "height position read")
        if pos is None:
            if not self.height_bus_offline:
                self.get_logger().warn(
                    f"height position read failed for ID {dxl_id}; using target"
                )
            cached = self.height_last_present.get(dxl_id)
            return int(cached if cached is not None else fallback_pos)
        self.height_last_present[dxl_id] = int(pos)
        return pos

    def read_height_position_strict(self, dxl_id, context):
        pos = self.read_height_register_4byte(dxl_id, ADDR_PRESENT_POSITION, context)
        if pos is None:
            return None
        return self.signed_value(pos, SIGNED_32BIT_MAX, UNSIGNED_32BIT_MAX)

    def read_height_velocity(self, dxl_id, context):
        raw = self.read_height_register_4byte(
            dxl_id, ADDR_PRESENT_VELOCITY, context
        )
        if raw is None:
            return None
        return self.signed_value(raw, SIGNED_32BIT_MAX, UNSIGNED_32BIT_MAX)

    def read_height_goal_position(self, dxl_id, context):
        pos = self.read_height_register_4byte(dxl_id, ADDR_GOAL_POSITION, context)
        if pos is None:
            cached = self.height_last_goal.get(dxl_id)
            return int(cached) if cached is not None else None
        goal = int(pos)
        self.height_last_goal[dxl_id] = goal
        return goal

    def _height_bus_note_failure(self, context):
        self.height_bus_fail_streak += 1
        if (
            not self.height_bus_offline
            and self.height_bus_fail_streak >= HEIGHT_BUS_FAIL_LATCH_COUNT
        ):
            self.height_bus_offline = True
            self.height_bus_last_probe_at = time.monotonic()
            self.get_logger().error(
                f"height bus OFFLINE after {self.height_bus_fail_streak} "
                f"failures ({context}); pausing height serial "
                f"(probe every {HEIGHT_BUS_PROBE_SEC:.0f}s)"
            )
            return
        if self.height_bus_offline:
            now = time.monotonic()
            if (now - self.height_bus_last_warn_at) >= HEIGHT_BUS_WARN_INTERVAL_SEC:
                self.height_bus_last_warn_at = now
                self.get_logger().warn(
                    f"height bus still offline ({context}); next probe in "
                    f"~{HEIGHT_BUS_PROBE_SEC:.0f}s"
                )

    def _height_bus_note_success(self):
        if self.height_bus_offline or self.height_bus_fail_streak:
            if self.height_bus_offline:
                self.get_logger().info("height bus ONLINE again")
            self.height_bus_offline = False
            self.height_bus_fail_streak = 0

    def read_height_register_4byte(self, dxl_id, address, context):
        # Offline: only transmit during an explicit probe cycle.
        if self.height_bus_offline and not self.height_bus_probe_active:
            return None
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read4ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            self._height_bus_note_failure(context)
            return None

        if not self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        ):
            self._height_bus_note_failure(context)
            return None
        self._height_bus_note_success()
        return value

    def publish_height_status(self):
        target_front_pos, target_rear_pos = height_ticks_for_down_mm(
            self.fixed_height_down_mm
        )
        # Latched offline: cache publish; probe all 4 registers every PROBE_SEC.
        if self.height_bus_offline:
            now = time.monotonic()
            if (now - self.height_bus_last_probe_at) < HEIGHT_BUS_PROBE_SEC:
                front_pos = self.height_last_present.get(
                    DXL_HEIGHT_FRONT_ID, target_front_pos
                )
                rear_pos = self.height_last_present.get(
                    DXL_HEIGHT_REAR_ID, target_rear_pos
                )
                front_goal_pos = self.height_last_goal.get(DXL_HEIGHT_FRONT_ID)
                rear_goal_pos = self.height_last_goal.get(DXL_HEIGHT_REAR_ID)
            else:
                self.height_bus_last_probe_at = now
                self.height_bus_probe_active = True
                try:
                    front_pos, rear_pos = self.read_height_positions()
                    front_goal_pos = self.read_height_goal_position(
                        DXL_HEIGHT_FRONT_ID,
                        "height front goal publish",
                    )
                    rear_goal_pos = self.read_height_goal_position(
                        DXL_HEIGHT_REAR_ID,
                        "height rear goal publish",
                    )
                finally:
                    self.height_bus_probe_active = False
        else:
            front_pos, rear_pos = self.read_height_positions()
            front_goal_pos = self.read_height_goal_position(
                DXL_HEIGHT_FRONT_ID,
                "height front goal publish",
            )
            rear_goal_pos = self.read_height_goal_position(
                DXL_HEIGHT_REAR_ID,
                "height rear goal publish",
            )

        state_msg = String()
        state_msg.data = self.height_state
        target_msg = Int32()
        target_msg.data = int(target_rear_pos)
        present_msg = Int32()
        present_msg.data = int(rear_pos)
        front_target_msg = Int32()
        front_target_msg.data = int(target_front_pos)
        rear_target_msg = Int32()
        rear_target_msg.data = int(target_rear_pos)
        front_present_msg = Int32()
        front_present_msg.data = int(front_pos)
        rear_present_msg = Int32()
        rear_present_msg.data = int(rear_pos)
        front_goal_msg = Int32()
        front_goal_msg.data = int(front_goal_pos) if front_goal_pos is not None else -1
        rear_goal_msg = Int32()
        rear_goal_msg.data = int(rear_goal_pos) if rear_goal_pos is not None else -1
        down_msg = Float32()
        down_msg.data = float(self.fixed_height_down_mm)

        self.height_state_pub.publish(state_msg)
        self.height_target_pub.publish(target_msg)
        self.height_present_pub.publish(present_msg)
        self.height_front_target_pub.publish(front_target_msg)
        self.height_rear_target_pub.publish(rear_target_msg)
        self.height_front_present_pub.publish(front_present_msg)
        self.height_rear_present_pub.publish(rear_present_msg)
        self.height_front_goal_pub.publish(front_goal_msg)
        self.height_rear_goal_pub.publish(rear_goal_msg)
        self.height_down_pub.publish(down_msg)

        if not self.height_bus_offline:
            presents = {
                DXL_HEIGHT_FRONT_ID: int(front_pos),
                DXL_HEIGHT_REAR_ID: int(rear_pos),
            }
            targets = height_target_positions(target_front_pos, target_rear_pos)
            for dxl_id in DXL_HEIGHT_IDS:
                if dxl_id in presents:
                    continue
                presents[dxl_id] = self.read_height_position(
                    dxl_id, targets[dxl_id]
                )
            self.note_height_move_extreme(presents)
            self.publish_height_presents(presents)
            currents_ma, temps_c = self.publish_height_currents()
            self.clamp_height_hold_if_overcurrent(currents_ma)
            self.check_height_temperature_stop(temps_c)

        self.publish_camera_pan_status()

        if (
            not self.height_bus_offline
            and self.height_hold_latched_mm != self.fixed_height_down_mm
        ):
            presents = {}
            for dxl_id in self.height_command_ids():
                pos = self.read_height_position_strict(
                    dxl_id, "height hold settle read"
                )
                if pos is None:
                    presents = {}
                    break
                presents[dxl_id] = pos
            if presents:
                self.note_height_move_extreme(presents)
                self.latch_height_hold_if_settled(presents)

    # ------------------------------------------------------------------
    # Dynamixel helpers
    # ------------------------------------------------------------------
    def write_height_motion_profile(self):
        """Write RAM Profile Velocity/Acceleration on height motors only."""
        ok = True
        for dxl_id in self.height_command_ids():
            ok = self.write_dxl_4byte(
                dxl_id,
                ADDR_PROFILE_ACCELERATION,
                HEIGHT_PROFILE_ACCELERATION,
                "height profile acceleration",
            ) and ok
            ok = self.write_dxl_4byte(
                dxl_id,
                ADDR_PROFILE_VELOCITY,
                HEIGHT_PROFILE_VELOCITY,
                "height profile velocity",
            ) and ok
        return ok

    def write_height_position_gains(self):
        """Write RAM Position PID on height motors. Safe with torque on."""
        ok = True
        for dxl_id in self.height_command_ids():
            ok = self.write_dxl_2byte(
                dxl_id,
                ADDR_POSITION_D_GAIN,
                HEIGHT_POSITION_D_GAIN,
                "height position D gain",
            ) and ok
            ok = self.write_dxl_2byte(
                dxl_id,
                ADDR_POSITION_I_GAIN,
                HEIGHT_POSITION_I_GAIN,
                "height position I gain",
            ) and ok
            ok = self.write_dxl_2byte(
                dxl_id,
                ADDR_POSITION_P_GAIN,
                HEIGHT_POSITION_P_GAIN,
                "height position P gain",
            ) and ok
        self.get_logger().info(
            "height position PID: "
            f"P={HEIGHT_POSITION_P_GAIN} I={HEIGHT_POSITION_I_GAIN} "
            f"D={HEIGHT_POSITION_D_GAIN} ok={ok}"
        )
        return ok

    def write_velocity_gains(self):
        gain_data = self.uint16_pair_to_bytes(
            self.velocity_i_gain,
            self.velocity_p_gain,
        )
        payload = {dxl_id: gain_data for dxl_id in DXL_ALL_IDS}
        self._sync_write_map(
            "sync_write_velocity_gains", payload, "velocity gain"
        )

    def read_dxl_byte(self, dxl_id, address, context):
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read1ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            return None

        if not self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        ):
            return None
        return value

    def write_dxl_byte(self, dxl_id, address, value, context):
        try:
            bus = self.bus_for(dxl_id)
            dxl_comm_result, dxl_error = bus.packet_handler.write1ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
                int(value),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            return False

        return self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        )

    def write_dxl_2byte(self, dxl_id, address, value, context):
        try:
            bus = self.bus_for(dxl_id)
            dxl_comm_result, dxl_error = bus.packet_handler.write2ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
                int(value),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            return False

        return self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        )

    def write_dxl_4byte(self, dxl_id, address, value, context):
        try:
            bus = self.bus_for(dxl_id)
            dxl_comm_result, dxl_error = bus.packet_handler.write4ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
                int(value),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            return False

        return self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        )

    def read_dxl_4byte(self, dxl_id, address, context):
        try:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read4ByteTxRx(
                bus.port_handler,
                dxl_id,
                address,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"{context} exception for ID {dxl_id} addr {address}: {exc}"
            )
            return None

        if not self.check_dxl_result(
            dxl_id,
            address,
            dxl_comm_result,
            dxl_error,
            context,
        ):
            return None
        return value

    def check_dxl_result(self, dxl_id, address, dxl_comm_result, dxl_error, context):
        bus = self.bus_for(dxl_id)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().warn(
                f"{context} failed for ID {dxl_id} addr {address}: "
                f"{bus.tx_result_text(dxl_comm_result)}"
            )
            return False
        if dxl_error != 0:
            hw_text = format_hardware_error_status(
                self.read_hardware_error_status_raw(dxl_id)
            )
            self.get_logger().warn(
                f"{context} error for ID {dxl_id} addr {address}: "
                f"{bus.rx_error_text(dxl_error)}; "
                f"hardware_error {hw_text}"
            )
            return False
        return True

    def read_wheel_feedback(self, dxl_id, reverse=False):
        bus = self.bus_for(dxl_id)
        dxl_vel = bus.sync_read_vel.getData(
            dxl_id,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY,
        )
        curr = bus.sync_read_current.getData(
            dxl_id,
            ADDR_PRESENT_CURRENT,
            LEN_PRESENT_CURRENT,
        )

        dxl_vel = self.signed_value(dxl_vel, SIGNED_32BIT_MAX, UNSIGNED_32BIT_MAX)
        curr = self.signed_value(curr, SIGNED_16BIT_MAX, UNSIGNED_16BIT_MAX)
        raw_dxl_vel = dxl_vel

        if reverse:
            dxl_vel = -dxl_vel

        velocity = (
            (dxl_vel * VELOCITY_UNIT_RPM)
            * (math.pi / 30.0)
            * MOTOR_TO_WHEEL_SPEED_RATIO
            * WHEEL_ODOM_RADIUS
        )
        current = curr * CURRENT_UNIT_MA
        return velocity, current, raw_dxl_vel

    def read_wheel_present_pwm(self, dxl_id):
        bus = self.bus_for(dxl_id)
        pwm = bus.sync_read_pwm.getData(
            dxl_id,
            ADDR_PRESENT_PWM,
            LEN_PRESENT_PWM,
        )
        return self.signed_value(pwm, SIGNED_16BIT_MAX, UNSIGNED_16BIT_MAX)

    def read_wheel_velocity_limits(self):
        limits = []
        for dxl_id in DXL_ALL_IDS:
            bus = self.bus_for(dxl_id)
            value, dxl_comm_result, dxl_error = bus.packet_handler.read4ByteTxRx(
                bus.port_handler,
                dxl_id,
                ADDR_VELOCITY_LIMIT,
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().warn(
                    f"Velocity Limit read failed for ID {dxl_id}: "
                    f"{bus.tx_result_text(dxl_comm_result)} "
                    f"{bus.rx_error_text(dxl_error)}"
                )
                limits.append(0)
            else:
                limits.append(int(value))
        return limits

    def stop_wheels(self):
        if not hasattr(self, "front_bus") or self.front_bus.sync_write_vel is None:
            return
        stop_val = self.int32_to_dxl_bytes(0)
        payload = {dxl_id: stop_val for dxl_id in DXL_ALL_IDS}
        self._sync_write_map("sync_write_vel", payload, "wheel stop")

    def shutdown_robot(self):
        self.stop_wheels()
        if not getattr(self, "height_return_on_shutdown", True):
            self.get_logger().info(
                "shutdown: height return disabled; holding current height"
            )
        elif hasattr(self, "front_bus") and self.front_bus.sync_write_height is not None:
            self.move_height_to_down_mm_verified(
                self.fixed_height_down_mm,
                "fixed height shutdown",
                HEIGHT_SHUTDOWN_TIMEOUT_SEC,
            )

        if WHEEL_TORQUE_DISABLE_ON_SHUTDOWN:
            for dxl_id in DXL_ALL_IDS:
                self.write_dxl_byte(
                    dxl_id,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE_VAL,
                    "wheel torque disable",
                )
            self.get_logger().info("shutdown: wheel torque disabled")

        if HEIGHT_TORQUE_HOLD_ON_SHUTDOWN:
            self.get_logger().info(
                "shutdown: height torque kept enabled for mechanical support"
            )
        else:
            for dxl_id in self.height_command_ids():
                self.write_dxl_byte(
                    dxl_id,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE_VAL,
                    "height torque disable",
                )

        if getattr(self, "camera_pan_enabled", False):
            if CAMERA_PAN_TORQUE_HOLD_ON_SHUTDOWN:
                self.get_logger().info("shutdown: camera pan torque kept enabled")
            else:
                self.write_dxl_byte(
                    DXL_CAMERA_PAN_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE_VAL,
                    "camera pan torque disable",
                )

        for dxl_id in DXL_ALL_IDS + self.height_command_ids():
            self.write_dxl_byte(
                dxl_id,
                ADDR_PRESENT_LED,
                0,
                "led off",
            )

        if hasattr(self, "front_bus"):
            self.front_bus.close()
        if hasattr(self, "rear_bus"):
            self.rear_bus.close()

    # ------------------------------------------------------------------
    # Small utilities
    # ------------------------------------------------------------------
    def calculate_traction_efficiency(self):
        if self.wheel_odom_speed < MIN_TRACTION_WHEEL_SPEED:
            return 0.0
        return self.ekf_speed / self.wheel_odom_speed

    def sample_std(self, values):
        mean = sum(values) / len(values)
        variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
        return math.sqrt(variance)

    def average(self, values):
        return sum(values) / len(values) if values else 0.0

    def signed_value(self, value, signed_max, unsigned_max):
        if value > signed_max:
            return value - unsigned_max
        return value

    def clamp_position(self, value):
        return max(POS_MIN, min(int(value), POS_MAX))

    def height_position_limits_for_id(self, dxl_id):
        return HEIGHT_POSITION_LIMITS_BY_ID.get(dxl_id, (POS_MIN, POS_MAX))

    def clamp_height_position_for_id(self, dxl_id, value):
        min_pos, max_pos = self.height_position_limits_for_id(dxl_id)
        return max(min_pos, min(int(value), max_pos))

    def write_height_position_limits(self, dxl_id):
        min_pos, max_pos = self.height_position_limits_for_id(dxl_id)
        ok = self.write_dxl_4byte(
            dxl_id,
            ADDR_MAX_POSITION_LIMIT,
            max_pos,
            "height max position limit set",
        )
        ok = self.write_dxl_4byte(
            dxl_id,
            ADDR_MIN_POSITION_LIMIT,
            min_pos,
            "height min position limit set",
        ) and ok
        return ok

    def height_positions_summary(self, positions):
        parts = []
        for dxl_id in DXL_HEIGHT_IDS:
            role = "front" if dxl_id in DXL_HEIGHT_FRONT_IDS else "rear"
            parts.append(f"{role}{dxl_id}={positions.get(dxl_id, 'unknown')}")
        return " ".join(parts)

    def int32_to_dxl_bytes(self, value):
        value = int(value) & 0xFFFFFFFF
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def uint16_pair_to_bytes(self, first, second):
        first = int(first) & 0xFFFF
        second = int(second) & 0xFFFF
        return [
            first & 0xFF,
            (first >> 8) & 0xFF,
            second & 0xFF,
            (second >> 8) & 0xFF,
        ]


def main():
    rclpy.init()
    node = None
    try:
        node = EclipseTestController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.get_logger().info("shutdown requested; stopping robot")
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(exc)
    finally:
        if node is not None:
            node.shutdown_robot()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
