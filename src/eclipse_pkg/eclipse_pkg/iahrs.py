import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, MagneticField
import serial
import time
import os
import math

# --- 기본 공분산 (최신 분석 결과 반영: 동적 공분산 기반) ---
BASE_AV_COV  = [4.99e-04, 8.86e-04, 1.65e-04]
BASE_LA_COV  = [2.14e-02, 5.92e-03, 1.70e-02]
BASE_ORI_COV = [1.0e-02, 1.0e-02, 5.0e-02]  # Experiment needed.

# 개행이 끝내 오지 않아도 버퍼가 무한히 자라지 않게 한다. 한 줄이 ~80바이트라
# 잡음이 길게 이어져도 이 한도 안에서 복구된다.
MAX_RX_BUFFER_BYTES = 4096
# 장치가 내보내는 필드 수 하한 (자이로3 + 가속도3 + 쿼터니언4).
MIN_IMU_FIELDS = 10
# 매뉴얼 4.2.21. 현재 스트림은 비트 오름차순: gyro + mag + accel-g + quat.
SD_GYRO_MAG_ACCEL_QUAT = 0x00B8
# 하드아이언 중심 (µT). 정지 8방위 LSQ 기준.
# 하드아이언은 위치 의존(처마 -23.98/-2.57 vs 야외 -41.53/-8.35, ~18µT 차이)이라
# 현장마다 재보정 필요. 야외(-41.53) 반영 시 오히려 오차가 커져 처마 값 유지.
MAG_HARDIRON_X_UT = -2.25
MAG_HARDIRON_Y_UT = -2.75
# 소프트아이언 보정 행렬 (정지 8방위 LSQ, 잔차 ~3°).
# 실제 왜곡은 거의 원형(축비 1.044, 회전 -0.043, 노이즈 수준) — 기존 선회 피팅의
# my×1.43은 모터 전류 오염으로 과했고 오히려 왜곡을 만들었다. 단위 행렬로 둔다.
MAG_SOFTIRON_XX = 1.0
MAG_SOFTIRON_XY = 0.0
MAG_SOFTIRON_YX = 0.0
MAG_SOFTIRON_YY = 1.0
# 천안 WMM 편각은 서편각(음수). +0.1396(동편각 8°)은 부호 오류.
MAG_DECLINATION_RAD = -0.14
# asin(6.5/36.18)^2 — 하드아이언만 뺐을 때 잔여 방위 분산 상한.
MAG_YAW_VARIANCE = 0.033
# 천안 지구 수직장(하향, µT). WMM2025 복각 ~56.7°, 총강도 ~49.6µT → 49.6·sin(56.7°).
# mz 실측이 z축 하드아이언 오프셋(+36.8µT)으로 신뢰 불가라, V-모델 투영에서 이 값을 쓴다.
MAG_VERTICAL_UT = 41.4
# 기울임 홀드: roll/pitch 절대값이 이 각도(도)를 넘으면 mag heading 발행을 멈춘다.
# 로봇 자체(모터·철제) 자기장 왜곡이 자세에 따라 변해 큰 기울임에서 mag 는 신뢰
# 불가 — EKF가 마지막 yaw + 각속도 적분으로 유지한다.
# 10°로 시작했으나 실측상 pitch 7.6°에서도 mag 가 19° 튀어 5°로 낮춤.
TILT_HOLD_DEG = 8.0


def parse_imu_lines(buffer, chunk, max_buffer=MAX_RX_BUFFER_BYTES):
    """시리얼 바이트 스트림에서 완전한 IMU 샘플만 뽑고 나머지는 버퍼에 남긴다.

    순수 함수라 시리얼 포트 없이 테스트할 수 있다.

    ``(samples, remaining_buffer)`` 를 돌려준다. ``samples`` 는 파싱된 float
    리스트들이고, ``remaining_buffer`` 는 마지막 개행 뒤에 남은 부분 라인이다 —
    다음 read 로 이어붙여야 하며 지금 파싱하면 안 된다.

    예전 구현은 ``splitlines()[-1]`` 로 마지막 줄만 집었는데, 청크가 줄 중간에서
    끊기면 그 조각이 마지막 줄이 되어 파싱에 실패하고 그 틱을 통째로 버렸다.
    또 그 방식은 앞선 샘플을 전부 버려서, 장치 스트림(>=100Hz)을 20Hz 로 받을 때
    각속도에 에일리어싱을 만들었다 (timer_callback 주석 참고).
    """
    buffer = buffer + chunk
    if len(buffer) > max_buffer:
        cut = buffer.rfind(b'\n')
        buffer = buffer[cut + 1:] if cut >= 0 else b''

    samples = []
    while True:
        index = buffer.find(b'\n')
        if index < 0:
            break
        raw, buffer = buffer[:index], buffer[index + 1:]
        line = raw.decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        try:
            vals = [float(x) for x in line.split(',')]
        except ValueError:
            continue
        if len(vals) >= MIN_IMU_FIELDS:
            samples.append(vals)
    return samples, buffer


def split_sync_sample(vals):
    """Sync 한 줄을 필드 딕셔너리로 나눈다. 레이아웃을 모르면 None.

    10필드: 기존 sd=0x00A8 (gyro, accel-g, quat).
    13필드: sd=0x00B8 (gyro, mag, accel-g, quat). 그 사이 길이는 거부.
    """
    n = len(vals)
    if n == 10:
        return {
            'gyro': vals[0:3],
            'mag': None,
            'accel': vals[3:6],
            'quat': vals[6:10],
        }
    if n == 13:
        return {
            'gyro': vals[0:3],
            'mag': vals[3:6],
            'accel': vals[6:9],
            'quat': vals[9:13],
        }
    return None


def correct_horizontal_mag(mx, my, cx=MAG_HARDIRON_X_UT, cy=MAG_HARDIRON_Y_UT):
    """하드아이언(오프셋)과 소프트아이언(타원→원 스케일)을 함께 보정한다."""
    mx1 = mx - cx
    my1 = my - cy
    mx2 = MAG_SOFTIRON_XX * mx1 + MAG_SOFTIRON_XY * my1
    my2 = MAG_SOFTIRON_YX * mx1 + MAG_SOFTIRON_YY * my1
    return mx2, my2


def magnetic_bearing_rad(mx, my):
    """센서 +X 가 자기북을 볼 때 0, 시계방향 양수.

    바디 ENU(+X 전방, +Y 좌)에서 수평 자기장이 (mx, my)일 때, 북을 보면
    my≈0·mx>0, 동을 보면 my>0·mx≈0 이라 atan2(my, mx) 가 compass bearing 이 된다.
    축 부호는 실차에서 뒤집힐 수 있어 파라미터로 교정한다.
    """
    return math.atan2(my, mx)


def compass_to_enu_yaw(bearing_rad):
    """진북 기준 시계방향 bearing → ROS ENU yaw (동=0, 반시계). GPS_node 와 동일."""
    return math.atan2(
        math.sin((math.pi / 2.0) - bearing_rad),
        math.cos((math.pi / 2.0) - bearing_rad),
    )


def mag_yaw_enu(mx, my, declination_rad=MAG_DECLINATION_RAD,
                cx=MAG_HARDIRON_X_UT, cy=MAG_HARDIRON_Y_UT):
    mx_c, my_c = correct_horizontal_mag(mx, my, cx, cy)
    true_bearing = magnetic_bearing_rad(mx_c, my_c) + declination_rad
    return compass_to_enu_yaw(true_bearing), mx_c, my_c


def quat_rotate_vector(qw, qx, qy, qz, vx, vy, vz):
    """쿼터니언 (w,x,y,z) 로 벡터 (vx,vy,vz) 를 회전한다. body → world.

    Hamilton 곱 v' = q ⊗ v ⊗ q⁻¹ 을 닫힌 형태로 푼 최적화 형태다.
    identity(w=1, x=y=z=0) 면 입력 벡터를 그대로 돌려준다.
    """
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def quat_roll_pitch(quat):
    """벤더 쿼터니언(world→body)의 켤레(body→world)에서 roll/pitch 추출(rad).

    기울임 홀드 게이트와 tilt_project_xy 가 공유하는 자세 추출이다.
    """
    w, x, y, z = quat
    qw, qx, qy, qz = w, -x, -y, -z  # 켤레 (body→world)
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    return roll, pitch


def tilt_hold_active(quat, hold_deg=TILT_HOLD_DEG):
    """기울임이 임계값을 넘으면 True. 이때 mag heading 을 발행하지 않는다."""
    roll, pitch = quat_roll_pitch(quat)
    return (math.degrees(abs(roll)) > hold_deg
            or math.degrees(abs(pitch)) > hold_deg)


def tilt_project_xy(mx, my, quat, V=MAG_VERTICAL_UT):
    """body mag 를 수평면(world xy)에 투영한다. mz 실측은 쓰지 않는다.

    벤더 mz 는 z축 하드아이언 오프셋(로봇 장착 환경)으로 신뢰 불가(평평에서
    -4.6µT, 실제 -40µT). 대신 벤더 쿼터니언(world→body)의 켤레에서 roll/pitch 를
    추출하고, world z 가 지구 수직장 -V 가 되도록 가상 mz 를 역산해 투영한다.
    (mx,my) 가 정확하면 이 역산 mz 는 수학적으로 진짜 mz 와 같다.
    """
    roll, pitch = quat_roll_pitch(quat)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    denom = cp * cr
    if abs(denom) < 1e-6:
        # ±90° 기울임 경계: 수평장 정보가 사라짐. 수평 성분을 그대로 반환.
        return mx, my
    # world z = -sp·mx + cp·(sr·my + cr·mz_v) = -V  →  mz_v 역산.
    mz_v = (sp * mx - cp * sr * my - V) / denom
    # 순방향: Rx(roll) 먼저, Ry(pitch) 그다음.
    y1 = my * cr - mz_v * sr
    z1 = my * sr + mz_v * cr
    x2 = mx * cp + z1 * sp
    y2 = y1
    return x2, y2


def mag_yaw_enu_tilted(mx, my, quat, declination_rad=MAG_DECLINATION_RAD,
                       cx=MAG_HARDIRON_X_UT, cy=MAG_HARDIRON_Y_UT):
    """body mag 를 기울기 보상(roll/pitch 투영, yaw 제외) 후 ENU yaw 로 변환한다.

    하드/소프트아이언은 센서 고정 왜곡이라 body frame 에서 먼저 보정하고,
    그 뒤 V-모델(지구 수직장 역산)로 지평면에 투영한다. mz 실측은 신뢰 불가라
    쓰지 않는다. 이렇게 해야 험지에서 기울어 앉아도 방위가 흔들리지 않는다.
    """
    mx_c, my_c = correct_horizontal_mag(mx, my, cx, cy)
    mx_h, my_h = tilt_project_xy(mx_c, my_c, quat)
    true_bearing = magnetic_bearing_rad(mx_h, my_h) + declination_rad
    return compass_to_enu_yaw(true_bearing), mx_c, my_c


class IahrsDriver(Node):
    def __init__(self):
        super().__init__('iahrs_driver')
        self.declare_parameter('mag_hardiron_x_ut', MAG_HARDIRON_X_UT)
        self.declare_parameter('mag_hardiron_y_ut', MAG_HARDIRON_Y_UT)
        self.declare_parameter('mag_declination_rad', MAG_DECLINATION_RAD)
        self.declare_parameter('mag_yaw_variance', MAG_YAW_VARIANCE)
        self.declare_parameter('tilt_hold_deg', TILT_HOLD_DEG)
        self._mag_cx = float(self.get_parameter('mag_hardiron_x_ut').value)
        self._mag_cy = float(self.get_parameter('mag_hardiron_y_ut').value)
        self._mag_decl = float(self.get_parameter('mag_declination_rad').value)
        self._mag_var = float(self.get_parameter('mag_yaw_variance').value)
        self._tilt_hold_deg = float(self.get_parameter('tilt_hold_deg').value)

        self.accel_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self._pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)
        self._pub_mag_heading = self.create_publisher(
            PoseWithCovarianceStamped, 'imu/mag_heading', 10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.ser = self.find_and_open_port()

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # --- 소프트웨어 필터용 변수 ---
        self.alpha = 0.2
        self.filtered_accel = [0.0, 0.0, 0.0]
        # 틱 사이에 걸친 미완성 라인을 이어붙이기 위한 버퍼.
        self._rx_buffer = b''

    def find_and_open_port(self):
        port = '/dev/ttyIMU'
        if not os.path.exists(port):
            self.get_logger().error(f'{port} 포트가 존재하지 않습니다. udev 연결을 확인하세요.')
            return None
        try:
            ser = serial.Serial(port, 115200, timeout=0.5)
            time.sleep(0.5)
            self.get_logger().info('IMU 센서 설정 및 깨우기 명령 전송 중...')
            ser.reset_input_buffer()
            ser.write(b'\r\n'); time.sleep(0.1)
            settings_commands = [
                b'al=5\r\n', b'av=1.0\r\n', b'zv=0.1\r\n',
                f'sd=0x{SD_GYRO_MAG_ACCEL_QUAT:04X}\r\n'.encode(), b'sp=50\r\n'
            ]
            for cmd in settings_commands:
                ser.write(cmd); time.sleep(0.05)
            ser.write(b'c=5\r\n'); time.sleep(0.2) 
            if ser.in_waiting > 0:
                self.get_logger().info('!!! IAHRS 센서 설정 완료 및 스트리밍 시작 !!!')
                ser.timeout = 0.1; ser.reset_input_buffer()
                return ser
            else:
                ser.close(); return None
        except Exception as e:
            self.get_logger().error(f'포트 연결 오류: {e}'); return None

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open: return
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                # 이 틱에 도착한 완전한 라인을 전부 파싱한다. 부분 라인은 버퍼에
                # 남겨 다음 틱으로 넘긴다(예전에는 splitlines()[-1] 이 잘린 마지막
                # 줄을 집어 그 틱을 통째로 버렸다).
                samples, self._rx_buffer = parse_imu_lines(self._rx_buffer, data)
                if not samples:
                    return

                # 절대 orientation 은 "지금 값"이므로 최신 샘플을 쓴다.
                parsed = [split_sync_sample(s) for s in samples]
                parsed = [p for p in parsed if p is not None]
                if not parsed:
                    return
                latest = parsed[-1]
                current_time = self.get_clock().now().to_msg()
                self.imu_msg.header.stamp = current_time

                deg_to_rad = math.pi / 180.0
                # 각속도는 "구간의 값"이므로 버린 샘플을 무시하지 않고 평균낸다.
                # 장치 스트림(>=100Hz)을 20Hz 타이머로 받으면서 최신 하나만 집으면
                # 나머지가 버려져 바퀴 진동 같은 고주파가 저주파로 접히는(aliasing)
                # 문제가 생긴다. EKF 의 imu0_config 는 yaw 절대값과 함께 yaw
                # 각속도(index 11)도 융합하므로 이 노이즈가 자세 추정에 직접 들어간다.
                n = len(parsed)
                self.imu_msg.angular_velocity.x = (
                    sum(p['gyro'][0] for p in parsed) / n) * deg_to_rad
                self.imu_msg.angular_velocity.y = (
                    sum(p['gyro'][1] for p in parsed) / n) * deg_to_rad
                self.imu_msg.angular_velocity.z = (
                    sum(p['gyro'][2] for p in parsed) / n) * deg_to_rad

                # 가속도는 EKF 가 쓰지 않지만(imu0_config 의 ax/ay/az 전부 false)
                # 토픽 호환을 위해 기존 EMA 를 유지한다.
                accel = latest['accel']
                for i in range(3):
                    self.filtered_accel[i] = (
                        (self.alpha * accel[i])
                        + ((1.0 - self.alpha) * self.filtered_accel[i])
                    )
                self.imu_msg.linear_acceleration.x = self.filtered_accel[0]
                self.imu_msg.linear_acceleration.y = self.filtered_accel[1]
                self.imu_msg.linear_acceleration.z = self.filtered_accel[2]

                quat_w, quat_x, quat_y, quat_z = latest['quat']
                quat_norm = math.sqrt(
                    quat_w**2 + quat_x**2 + quat_y**2 + quat_z**2
                )
                if quat_norm > 0.0:
                    qw = quat_w / quat_norm
                    qx = quat_x / quat_norm
                    qy = quat_y / quat_norm
                    qz = quat_z / quat_norm
                    self.imu_msg.orientation.x = qx
                    self.imu_msg.orientation.y = qy
                    self.imu_msg.orientation.z = qz
                    self.imu_msg.orientation.w = qw
                    self.imu_msg.orientation_covariance[0] = BASE_ORI_COV[0]
                    self.imu_msg.orientation_covariance[4] = BASE_ORI_COV[1]
                    self.imu_msg.orientation_covariance[8] = BASE_ORI_COV[2]
                else:
                    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
                    self.imu_msg.orientation.x = 0.0
                    self.imu_msg.orientation.y = 0.0
                    self.imu_msg.orientation.z = 0.0
                    self.imu_msg.orientation.w = 1.0
                    self.imu_msg.orientation_covariance[0] = -1.0
                    self.imu_msg.orientation_covariance[4] = 0.0
                    self.imu_msg.orientation_covariance[8] = 0.0

                self.imu_msg.angular_velocity_covariance[0] = BASE_AV_COV[0]
                self.imu_msg.angular_velocity_covariance[4] = BASE_AV_COV[1]
                self.imu_msg.angular_velocity_covariance[8] = BASE_AV_COV[2]

                self.imu_msg.linear_acceleration_covariance[0] = BASE_LA_COV[0]
                self.imu_msg.linear_acceleration_covariance[4] = BASE_LA_COV[1]
                self.imu_msg.linear_acceleration_covariance[8] = BASE_LA_COV[2]

                self.accel_publisher_.publish(self.imu_msg)
                if latest['mag'] is not None:
                    self._publish_mag_heading(
                        current_time, latest['mag'], (qw, qx, qy, qz)
                    )
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _publish_mag_heading(self, stamp, mag_xyz, quat):
        mx, my, mz = mag_xyz
        # 기울임 홀드: 큰 기울임에서 로봇 자체 자기장 왜곡이 자세 의존적으로 변해
        # mag 가 신뢰 불가 → 발행을 멈춰 EKF 가 마지막 yaw + 각속도 적분으로 유지.
        if tilt_hold_active(quat, self._tilt_hold_deg):
            return
        yaw, mx_c, my_c = mag_yaw_enu_tilted(
            mx, my, quat, self._mag_decl, self._mag_cx, self._mag_cy
        )
        field = MagneticField()
        field.header.stamp = stamp
        field.header.frame_id = 'imu_link'
        # MagneticField 단위는 tesla. 스트림은 µT.
        field.magnetic_field.x = mx_c * 1e-6
        field.magnetic_field.y = my_c * 1e-6
        field.magnetic_field.z = mz * 1e-6
        self._pub_mag.publish(field)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        # 절대 진북 방위(ENU yaw)이므로 map 프레임. GPS_node 의 /gps/heading 과
        # 같은 frame_id 로 발행해 ekf.yaml 의 pose0 에 그대로 융합되게 한다.
        # imu_link 가 필요한 센서 원시값은 /imu/mag(MagneticField)가 담당한다.
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[35] = self._mag_var
        self._pub_mag_heading.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = IahrsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
