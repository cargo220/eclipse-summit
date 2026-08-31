"""Runtime constants for the fixed-height eclipse-test controller."""

import math

from eclipse_pkg.current_covariance import CurrentCovarianceConfig


# Dynamixel bus and motor layout.
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DXL_LEFT_IDS = (2, 3)
DXL_RIGHT_IDS = (12, 13)
DXL_ALL_IDS = DXL_LEFT_IDS + DXL_RIGHT_IDS
# Sign applied to that side's robot-frame command before Goal write.
# Also used to invert present-velocity feedback for odom.
# Baseline: left +1, right -1 (right motors mounted opposite).
DXL_WHEEL_GOAL_SIGN_BY_ID = {
    2: 1,
    3: 1,
    12: -1,
    13: -1,
}


def dxl_wheel_goal_sign(dxl_id):
    return int(DXL_WHEEL_GOAL_SIGN_BY_ID.get(int(dxl_id), 1))
# Height: 1/4 front axle, 11/14 rear axle. Pair IDs unused until ping.
# Skid-steer command groups stay left (2, 3) / right (12, 13).
# Dual CM-900:
# FRONT ACM0 = 1,2,4,12 (front height + front wheels) + 5 (camera pan).
# REAR ACM3 = 3,11,13,14 (rear height + rear wheels).
# Board itself answers as ID 200 on both. Left remains 2,3 / right 12,13.
DXL_HEIGHT_FRONT_ID = 1
DXL_HEIGHT_REAR_ID = 11
DXL_HEIGHT_FRONT_PAIR_ID = 4
DXL_HEIGHT_REAR_PAIR_ID = 14
DXL_HEIGHT_FRONT_IDS = (DXL_HEIGHT_FRONT_ID, DXL_HEIGHT_FRONT_PAIR_ID)
DXL_HEIGHT_REAR_IDS = (DXL_HEIGHT_REAR_ID, DXL_HEIGHT_REAR_PAIR_ID)
DXL_HEIGHT_REQUIRED_IDS = (DXL_HEIGHT_REAR_ID, DXL_HEIGHT_FRONT_ID)
DXL_HEIGHT_OPTIONAL_IDS = (DXL_HEIGHT_REAR_PAIR_ID, DXL_HEIGHT_FRONT_PAIR_ID)
# Rear-first order matches the existing SyncWrite convention.
DXL_HEIGHT_IDS = DXL_HEIGHT_REAR_IDS + DXL_HEIGHT_FRONT_IDS
DXL_FRONT_WHEEL_IDS = (2, 12)
DXL_REAR_WHEEL_IDS = (3, 13)
DXL_CAMERA_PAN_ID = 5
DXL_CAMERA_PAN_IDS = (DXL_CAMERA_PAN_ID,)
DXL_FRONT_BUS_IDS = (
    DXL_FRONT_WHEEL_IDS + DXL_HEIGHT_FRONT_IDS + DXL_CAMERA_PAN_IDS
)
DXL_REAR_BUS_IDS = DXL_REAR_WHEEL_IDS + DXL_HEIGHT_REAR_IDS
DEVICENAME_FRONT = '/dev/wttyOPENCM_FRONT'
DEVICENAME_REAR = '/dev/wttyOPENCM_REAR'


def dxl_bus_name(dxl_id):
    """'front', 'rear', or None if the ID is not on either OpenCM."""
    dxl_id = int(dxl_id)
    if dxl_id in DXL_FRONT_BUS_IDS:
        return "front"
    if dxl_id in DXL_REAR_BUS_IDS:
        return "rear"
    return None


def dxl_bus_ids_are_partitioned():
    """True when front/rear IDs are disjoint and cover wheel+height+pan IDs."""
    front = set(DXL_FRONT_BUS_IDS)
    rear = set(DXL_REAR_BUS_IDS)
    expected = set(DXL_ALL_IDS) | set(DXL_HEIGHT_IDS) | set(DXL_CAMERA_PAN_IDS)
    return not (front & rear) and front | rear == expected


def wheel_bulk_read_is_usable(front_ok, rear_ok):
    """False unless both wheel buses returned a successful bulk read."""
    return bool(front_ok) and bool(rear_ok)

# Robot geometry and wheel/motor conversion.
# 조정가능 — 바퀴/기구 튜닝 (상세 목록: robot_specifications/tars_tuning.yaml)
#   반지름·윤거·기어비가 바뀌면 odom·명령 RPM 환산·최대 선속도 한계가 같이 흔들림.
WHEEL_SEPARATION = 0.4788  # 조정가능: 좌우 바퀴 간격(m)
# 패들테스트 선정 D250: 지름 250 mm → 반지름 0.125 m.
# (코드 주석의 "반지름 250"은 지름 숫자. R=0.250 m 가 아님.)
WHEEL_COMMAND_RADIUS = 0.125  # 조정가능: 명령용 바퀴 반지름(m), 직경 250 mm
WHEEL_ODOM_RADIUS = 0.125  # 조정가능: 오도메트리용 바퀴 반지름(m)
# 증속 G=2.5. ω_wheel = G * ω_motor.
# 이빨 수는 비만 표현: 모터 50 / 바퀴 20. 실 풀리 이가 달라도 G가 2.5면 됨.
MOTOR_DRIVE_GEAR_TEETH = 50.0  # 조정가능: 모터측 풀리 이 (증속)
WHEEL_DRIVEN_GEAR_TEETH = 20.0  # 조정가능: 바퀴측 풀리 이
MOTOR_TO_WHEEL_SPEED_RATIO = MOTOR_DRIVE_GEAR_TEETH / WHEEL_DRIVEN_GEAR_TEETH
VELOCITY_UNIT_RPM = 0.229  # Dynamixel 속도 단위 환산 (펌웨어 고정에 가깝)
CURRENT_UNIT_MA = 2.69
RAD_PER_SEC_TO_DXL_VEL_FACTOR = (
    (60.0 / (2.0 * math.pi)) / VELOCITY_UNIT_RPM
)
# 180 mm 실차에서 읽은 XM540 Velocity Limit. 틱 한계는 바퀴가 바뀌어도 같다.
DXL_WHEEL_VELOCITY_LIMIT_TICKS = 130
# v = tick * unit_rpm/60 * 2π * R * G  → 130틱에서 ≈ 0.9742 m/s
PLATFORM_MAX_LINEAR_MPS = (
    DXL_WHEEL_VELOCITY_LIMIT_TICKS
    / RAD_PER_SEC_TO_DXL_VEL_FACTOR
    * WHEEL_COMMAND_RADIUS
    * MOTOR_TO_WHEEL_SPEED_RATIO
)

# Heading hold.
HEADING_HOLD_CMD_W_THRESHOLD = 0.02
HEADING_HOLD_KP = 0.8
HEADING_HOLD_MAX_W = 0.3

# Dynamixel operating modes and default loop settings.
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
TORQUE_ENABLE_VAL = 1
TORQUE_DISABLE_VAL = 0
WHEEL_TORQUE_DISABLE_ON_SHUTDOWN = True
HEIGHT_TORQUE_HOLD_ON_SHUTDOWN = True
# Wheel feedback loop period (GroupSyncRead vel/pwm/current + odom publish).
# Serial bulk reads dominate controller CPU/load — slower is cheaper, but any
# sample-count buffer must scale with CONTROL_DT so wall-time windows stay fixed
# (current std buffer, terrain sample counts, baseline EMA). See helpers below.
CONTROL_DT = 0.125
CONTROL_HZ = 1.0 / CONTROL_DT

# Reference era used only to convert legacy per-sample constants into time-based
# ones when CONTROL_DT changes (do not treat these as live loop rates).
_REF_CONTROL_DT = 0.125
_REF_BUFFER_SIZE = 10  # samples at 8 Hz → 1.25 s current window
_REF_TERRAIN_SAMPLES = 4  # consecutive samples at 8 Hz → 0.5 s
_REF_BASELINE_ALPHA = 0.01  # per-sample EMA at 8 Hz

# 조정가능 — /cmd_vel 침묵 시 휠 정지 (tars_tuning.yaml § controller_cmd_timeouts)
CMD_VEL_TIMEOUT_SEC = 2.0
HEIGHT_STATUS_DT = 0.5
HEIGHT_VOLTAGE_STATUS_DT = 0.5  # addr 144 발행 주기(s)
# Covariance recompute cadence (wall time). Independent of CONTROL_DT; still
# appends current samples every wheel tick into BUFFER_SIZE.
COV_UPDATE_DT = 0.5
DEFAULT_VELOCITY_I_GAIN = 0
DEFAULT_VELOCITY_P_GAIN = 800
# Height XM540 Position PID (RAM). Factory P=800 I=0 D=0.
# I=80 closed 45 mm droop on the bench but overloaded a height motor
# (tick-then-shutdown) on description bringup. Field default is I=0.
HEIGHT_POSITION_P_GAIN = 800  # 조정가능: 높이 Position P (피크 홀드 유지용)
HEIGHT_POSITION_I_GAIN = 0  # 조정가능: 높이 Position I (80은 기동 과부하)
HEIGHT_POSITION_D_GAIN = 0
# Height XM540 Profile (RAM). 33 ≈ 40 mm/s at 45 mm. 0 = unlimited.
# Written at startup and on every height Goal Position write.
HEIGHT_PROFILE_VELOCITY = 33
HEIGHT_PROFILE_ACCELERATION = 8

# Dynamixel motor safety limits.
MOTOR_TEMPERATURE_LIMIT_C = 70
MOTOR_TEMPERATURE_WARN_C = 55
MOTOR_TEMPERATURE_STOP_C = 65
MOTOR_PWM_LIMIT = 885
# --- 모터 스톨 → /motor/safety_state 에 "stall" 문자열을 내는 물리 임계 ---
# 조정가능 — recovery STALL 라벨 원천 (tars_tuning.yaml § recovery_stall)
# Recovery StallDetector 는 문자열만 보고, 아래 숫자는 컨트롤러가 라벨을 붙일 때 사용.
# 조건(동시): |cmd|≥MIN_CMD  and  휠속도≤MAX_WHEEL  and  max|PWM|≥PWM_STALL
#   → 유지 중: WARN_STALL_PWM_*  /  DURATION_SEC 이상: FAULT_stall_...
MOTOR_PWM_STALL_THRESHOLD = 500  # 조정가능: PWM 이 이 이상이면 "힘 많이 줌"
MOTOR_PWM_UNIT_PERCENT = 0.113
MOTOR_SHUTDOWN_MASK = 0x3D
# Hardware Error Status (control table addr 70).
HARDWARE_ERROR_STATUS_ADDR = 70
HARDWARE_ERROR_NAMES = (
    (0x01, "input_voltage"),
    (0x04, "overheating"),
    (0x08, "encoder"),
    (0x10, "electrical_shock"),
    (0x20, "overload"),
)


def format_hardware_error_status(value):
    """Decode Hardware Error Status (addr 70) for logs and topics."""
    if value is None:
        return "unread"
    value = int(value) & 0xFF
    names = [name for bit, name in HARDWARE_ERROR_NAMES if value & bit]
    if not names:
        return f"0x{value:02x}"
    return f"0x{value:02x}({'+'.join(names)})"


def format_hardware_error_log(kind, ids, values):
    parts = []
    for dxl_id, value in zip(ids, values):
        parts.append(f"id{int(dxl_id)}={format_hardware_error_status(value)}")
    return f"{kind} hardware error " + " ".join(parts)


def format_height_hardware_error_log(ids, values):
    return format_hardware_error_log("height", ids, values)


def format_wheel_hardware_error_log(ids, values):
    return format_hardware_error_log("wheel", ids, values)


MOTOR_BUS_WATCHDOG = 0
MOTOR_BUS_WATCHDOG_UNIT_SEC = 0.02
MOTOR_SAFETY_DT = 0.5  # safety_state 발행 주기(초)
MOTOR_SAFETY_READ_FAILURE_LIMIT = 3
MOTOR_SAFETY_WARN_LOG_INTERVAL_SEC = 2.0
MOTOR_TEMP_WARN_SPEED_SCALE = 0.5
MOTOR_INPUT_VOLTAGE_UNIT_V = 0.1
MOTOR_INPUT_VOLTAGE_WARN_MIN_V = 10.0
MOTOR_INPUT_VOLTAGE_WARN_MAX_V = 15.0
MOTOR_STALL_MIN_CMD_MPS = 0.05  # 조정가능: 명령 속도가 이 이상일 때만 스톨 후보
MOTOR_STALL_MAX_WHEEL_SPEED_MPS = 0.02  # 조정가능: 휠이 이 이하면 "거의 안 돎"
MOTOR_STALL_DURATION_SEC = 1.5  # 조정가능: 후보가 이 시간 이상이면 FAULT_stall

# Fixed height actuator setup for wheel-shape tests.
POS_MIN = 0
POS_MAX = 4095
# Camera pan XM430-W350 ID 5 on FRONT. EEPROM min/max come from Wizard;
# the snapshot below is only the last verified window. Runtime clamp
# always uses the live registers, never writes them back.
CAMERA_PAN_WIZARD_MIN_TICKS = 1148
CAMERA_PAN_WIZARD_MAX_TICKS = 2948
CAMERA_PAN_HOME_TICKS = 2048  # XM430 center; Goal at eclipse_test_controller start
CAMERA_PAN_PROFILE_VELOCITY = 50  # 조정가능: RAM Profile Velocity
CAMERA_PAN_PROFILE_ACCELERATION = 8  # 조정가능: RAM Profile Acceleration
CAMERA_PAN_TORQUE_HOLD_ON_SHUTDOWN = True


def camera_pan_limits_are_valid(min_pos, max_pos):
    try:
        lo = int(min_pos)
        hi = int(max_pos)
    except (TypeError, ValueError):
        return False
    return POS_MIN <= lo < hi <= POS_MAX


def clamp_camera_pan_position(tick, min_pos, max_pos):
    """Clamp a pan Goal tick to Wizard EEPROM limits. None if limits bad."""
    if not camera_pan_limits_are_valid(min_pos, max_pos):
        return None
    try:
        value = int(tick)
    except (TypeError, ValueError):
        return None
    return max(int(min_pos), min(value, int(max_pos)))


def camera_pan_startup_goal(min_pos, max_pos):
    """Home Goal at node start, clamped to live EEPROM. None if limits bad."""
    return clamp_camera_pan_position(
        CAMERA_PAN_HOME_TICKS, min_pos, max_pos
    )


# Same-direction remount: each axle shares one window so a pair cannot
# clip at different ticks and twist the shaft.
HEIGHT_POSITION_LIMITS_BY_ID = {
    DXL_HEIGHT_FRONT_ID: (2116, 3246),
    DXL_HEIGHT_FRONT_PAIR_ID: (2116, 3246),
    DXL_HEIGHT_REAR_ID: (824, 1810),
    DXL_HEIGHT_REAR_PAIR_ID: (824, 1810),
}
FIXED_HEIGHT_DOWN_MM = 45.0
HEIGHT_POSITION_TOLERANCE_TICKS = 70
# After a move, rewrite Goal=Present when this close to the table.
# 70 ate a 3 mm step. 15 blocked 26-tick stalls (~1 A). 40 is in between.
HEIGHT_POSITION_HOLD_DEADBAND_TICKS = 40  # 조정가능: 홀드 허용 잔여 틱
# Dynamixel Present Velocity units (addr 128). Do not latch Goal=Present
# while the 4-bar is still moving — that overshoots then pulls back.
HEIGHT_HOLD_MAX_VELOCITY = 3  # 조정가능: 이보다 빠르면 홀드 금지
HEIGHT_HOLD_INHIBIT_SEC = 0.7  # 조정가능: 높이 명령 직후 홀드 금지(초)
# Present a few ticks past EEPROM min/max used to skip stall hold entirely
# (ID 11=821 vs min 824), leaving pair motors on table Goal.
HEIGHT_HOLD_WINDOW_SLACK_TICKS = 32  # 조정가능: 창 밖이어도 홀드 허용(틱)


def height_hold_is_inhibited(now_sec, inhibit_until_sec):
    """True during the post-command window where Goal=Present is forbidden."""
    if now_sec is None or inhibit_until_sec is None:
        return True
    return float(now_sec) < float(inhibit_until_sec)


def height_hold_is_stopped(velocity, max_vel=None):
    """True when Present Velocity is at/under the hold cap."""
    if velocity is None:
        return False
    vmax = HEIGHT_HOLD_MAX_VELOCITY if max_vel is None else max_vel
    return abs(int(velocity)) <= int(vmax)


def height_hold_is_settled(present, table_goal, velocity, deadband=None, max_vel=None):
    """True when a height motor is near the table goal and almost stopped."""
    if present is None or table_goal is None:
        return False
    band = HEIGHT_POSITION_HOLD_DEADBAND_TICKS if deadband is None else deadband
    if abs(int(present) - int(table_goal)) > int(band):
        return False
    return height_hold_is_stopped(velocity, max_vel=max_vel)


def height_hold_may_latch(now_sec, inhibit_until_sec, velocities):
    """After inhibit, latch if every motor is stopped — even off the table.

    Raise stalls ~26 ticks short; waiting for the 15-tick band keeps ~1 A.
    """
    if height_hold_is_inhibited(now_sec, inhibit_until_sec):
        return False
    if not velocities:
        return False
    return all(height_hold_is_stopped(vel) for vel in velocities)


def height_hold_source_id(dxl_id):
    """Primary on that axle for motion commands (4←1, 14←11)."""
    dxl_id = int(dxl_id)
    if dxl_id == DXL_HEIGHT_FRONT_PAIR_ID:
        return DXL_HEIGHT_FRONT_ID
    if dxl_id == DXL_HEIGHT_REAR_PAIR_ID:
        return DXL_HEIGHT_REAR_ID
    return dxl_id


def height_hold_latch_present_for_id(dxl_id, presents):
    """Own Present to write as Goal after |Present Velocity| is at the cap.

    Height *commands* still copy the primary tick (4←1, 14←11). Settle
    hold does not: a pair offset of a few ticks would otherwise keep
    P-gain current on the pair motor forever.
    """
    if not presents:
        return None
    present = presents.get(int(dxl_id))
    if present is None:
        return None
    return int(present)


def height_hold_peak_for_id(dxl_id, extremes, presents):
    """Farthest Present on the primary during a move (not used at settle).

    Settle latch uses height_hold_latch_present_for_id. This keeps the
    commanded-direction extreme for logs and any later pull-back path.
    """
    source = height_hold_source_id(dxl_id)
    if extremes:
        peak = extremes.get(source)
        if peak is not None:
            return int(peak)
    if presents:
        present = presents.get(source)
        if present is not None:
            return int(present)
    return None


def height_extreme_present(prev, present, dxl_id, down_delta):
    """Keep the farthest Present in the commanded down_mm direction.

    Front ticks decrease with down_mm; rear ticks increase.
    """
    if present is None:
        return prev
    pos = int(present)
    if prev is None:
        return pos
    prev = int(prev)
    front = int(dxl_id) in DXL_HEIGHT_FRONT_IDS
    delta = float(down_delta)
    if delta > 0:
        return min(prev, pos) if front else max(prev, pos)
    if delta < 0:
        return max(prev, pos) if front else min(prev, pos)
    return pos


def height_hold_goal_from_present(present, min_pos, max_pos, slack=None):
    """Clamp Present into the window for Goal=Present.

    Returns None if Present is farther than slack outside the EEPROM window
    so a garbage reading cannot become a Goal.
    """
    if present is None or min_pos is None or max_pos is None:
        return None
    lo, hi = int(min_pos), int(max_pos)
    if lo > hi:
        return None
    extra = HEIGHT_HOLD_WINDOW_SLACK_TICKS if slack is None else int(slack)
    pos = int(present)
    if pos < lo - extra or pos > hi + extra:
        return None
    return max(lo, min(hi, pos))


# If any height motor |current| stays at/above this, snap Goal=Present.
# Ampere cap is load-independent; tick deadband is not. XM540 unit 2.69 mA.
# 0 이하면 소프트웨어 클램프 없음. 로컬 기본 2500, 자비에 스테이징 3500.
HEIGHT_CURRENT_CLAMP_MA = 2500.0


def height_current_clamp_is_enabled(limit_ma=None):
    """False when the software height current snap is turned off."""
    if limit_ma is None:
        limit_ma = HEIGHT_CURRENT_CLAMP_MA
    return float(limit_ma) > 0.0
# Software stop for height IDs 1/4/11/14. Wheel software temp path is still
# bypassed. Firmware Temperature Limit stays 70 C as last resort.
HEIGHT_TEMPERATURE_STOP_C = 65  # 조정가능: 높이 Present Temperature 정지(°C)
HEIGHT_INITIALIZE_ATTEMPTS = 3
HEIGHT_INITIALIZE_TIMEOUT_SEC = 4.0
HEIGHT_SHUTDOWN_TIMEOUT_SEC = 4.0
HEIGHT_INITIALIZE_RETRY_DELAY_SEC = 0.2
HEIGHT_POSITION_POLL_SEC = 0.1

# Height AI runs in-process on the controller timer (stub policy holds height).
# HEIGHT_STATE_AI lives in eclipse_test_controller.py with the other FSM labels.
HEIGHT_AI_APPLY_DT = HEIGHT_STATUS_DT  # same cadence as publish_height_status
HEIGHT_AI_MAX_RATE_MM_PER_S = 5.0
HEIGHT_AI_DEADBAND_MM = 0.5
# Serve-side /probe/angle freshness.
HEIGHT_AI_PROBE_STALE_SEC = 0.5
HEIGHT_AI_PROBE_CONTACT_ANGLE = 0.0
HEIGHT_AI_PROBE_CONTACT_TOLERANCE = 3.0


def height_temperature_is_stop(temp_c, stop_c=None):
    """True when a height-motor temperature should latch a software stop."""
    if temp_c is None:
        return False
    limit = HEIGHT_TEMPERATURE_STOP_C if stop_c is None else stop_c
    return float(temp_c) >= float(limit)


def _format_height_id_line(label, ids, values):
    """One log line: 'height current mA id11=.. id14=.. id1=.. id4=..'."""
    parts = []
    for dxl_id, value in zip(ids, values):
        number = 0.0 if value is None else float(value)
        parts.append(f"id{int(dxl_id)}={number:.0f}")
    return f"{label} " + " ".join(parts)


def format_height_current_log(ids, values):
    return _format_height_id_line("height current mA", ids, values)


def format_height_temperature_log(ids, values):
    return _format_height_id_line("height temp C", ids, values)


def format_height_present_log(ids, values):
    return _format_height_id_line("height present tick", ids, values)


def format_height_voltage_log(ids, values):
    parts = []
    for dxl_id, value in zip(ids, values):
        number = 0.0 if value is None else float(value)
        parts.append(f"id{int(dxl_id)}={number:.1f}")
    return "height input voltage V " + " ".join(parts)


# Runtime thresholds shared by wheel odometry and covariance monitoring.
MIN_CMD_VEL = 0.05
MIN_TRACTION_WHEEL_SPEED = 0.02

# Current-variation based odometry covariance.
# Wall-time windows (preserve semantics when CONTROL_DT changes):
CURRENT_BUFFER_WINDOW_SEC = _REF_CONTROL_DT * _REF_BUFFER_SIZE  # 1.25 s
CURRENT_TERRAIN_WINDOW_SEC = _REF_CONTROL_DT * _REF_TERRAIN_SAMPLES  # 0.5 s
# Sample counts filled once per feedback_loop tick:
BUFFER_SIZE = max(4, int(round(CURRENT_BUFFER_WINDOW_SEC / CONTROL_DT)))
CURRENT_TERRAIN_SAMPLES = max(
    2, int(round(CURRENT_TERRAIN_WINDOW_SEC / CONTROL_DT))
)
# Per-sample EMA: keep (updates/sec)*alpha constant in wall time.
# (1/CONTROL_DT)*alpha = (1/_REF_CONTROL_DT)*_REF_BASELINE_ALPHA
CURRENT_BASELINE_ALPHA = min(
    0.25,
    _REF_BASELINE_ALPHA * (CONTROL_DT / _REF_CONTROL_DT),
)
BASE_COV = 0.0018
# GPS_node maps RTK Float/differential to 1 and RTK Fixed to 2.
# Autonomous covariance handling accepts either corrected-GNSS state.
GPS_MIN_GOOD_FIX_STATUS = 1
# Allow RTK Float (1) as the minimum status for starting a goal. RTK Fixed
# (2) remains preferred.
GPS_MIN_NAV_START_FIX_STATUS = 1
GPS_GOOD_WHEEL_ODOM_COV = 0.015
GPS_POSE_COVARIANCE_FLOOR = 0.004
GPS_FIX_TIMEOUT_SEC = 1.0
GPS_VELOCITY_TIMEOUT_SEC = 1.0
NORMAL_STD = 60.0
COV_GAIN = 5.0
CONTINUOUS_TERRAIN_MAX_COV = 0.5
CURRENT_BASELINE_MARGIN = 1.5
CURRENT_BASELINE_MAX_STD = 120.0
CURRENT_BASELINE_STD_STEP_LIMIT = 2.0
CURRENT_BASELINE_MIN_CMD_V = 0.05
CURRENT_BASELINE_MAX_CMD_W = 0.03
CURRENT_BASELINE_MIN_WHEEL_SPEED = 0.03
CURRENT_SHOCK_STD_STEP = 3.0
CURRENT_SHOCK_RATIO = 1.8
ODOM_COV_LOG_INTERVAL = 2.0
MAX_COV = 99.0

CURRENT_COVARIANCE_CONFIG = CurrentCovarianceConfig(
    normal_std=NORMAL_STD,
    cov_gain=COV_GAIN,
    terrain_samples=CURRENT_TERRAIN_SAMPLES,
    continuous_max_cov=CONTINUOUS_TERRAIN_MAX_COV,
    baseline_alpha=CURRENT_BASELINE_ALPHA,
    baseline_margin=CURRENT_BASELINE_MARGIN,
    baseline_max_std=CURRENT_BASELINE_MAX_STD,
    baseline_std_step_limit=CURRENT_BASELINE_STD_STEP_LIMIT,
    baseline_min_cmd_v=CURRENT_BASELINE_MIN_CMD_V,
    baseline_max_cmd_w=CURRENT_BASELINE_MAX_CMD_W,
    baseline_min_wheel_speed=CURRENT_BASELINE_MIN_WHEEL_SPEED,
    shock_std_step=CURRENT_SHOCK_STD_STEP,
    shock_ratio=CURRENT_SHOCK_RATIO,
    base_cov=BASE_COV,
    max_cov=MAX_COV,
)

def height_target_positions(front_pos, rear_pos):
    """Map ID 1 / ID 11 ticks onto all four motors.

    Same 4-bar lengths. After the 2026-08-21 remount, pair IDs 4 and 14
    copy the primary tick on that axle.
    """
    from eclipse_pkg.height_table import (
        height_down_mm_for_ticks,
        height_positions_for_down_mm,
    )

    return height_positions_for_down_mm(
        height_down_mm_for_ticks(front_pos, rear_pos)
    )


# Signed register conversion.
SIGNED_8BIT_MAX = 0x7F
SIGNED_16BIT_MAX = 0x7FFF
SIGNED_32BIT_MAX = 0x7FFFFFFF
UNSIGNED_8BIT_MAX = 256
UNSIGNED_16BIT_MAX = 65536
UNSIGNED_32BIT_MAX = 4294967296
