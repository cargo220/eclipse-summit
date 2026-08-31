import base64
import math
import socket
import struct
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger

from eclipse_pkg.eclipse_test_config import GPS_POSE_COVARIANCE_FLOOR


UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62
UBX_CLASS_CFG = 0x06
UBX_ID_CFG_MSG = 0x01
UBX_CLASS_NAV = 0x01
UBX_ID_PVT = 0x07
UBX_ID_STATUS = 0x03
UBX_CLASS_NMEA = 0xF0
UBX_ID_GGA = 0x00

# UBX-NAV-PVT payload offsets.
PVT_ITOW = 0
PVT_YEAR = 4
PVT_MONTH = 6
PVT_DAY = 7
PVT_HOUR = 8
PVT_MIN = 9
PVT_SEC = 10
PVT_VALID = 11
PVT_FIXTYPE = 20
PVT_FLAGS = 21
PVT_NUMSV = 23
PVT_LON = 24
PVT_LAT = 28
PVT_HMSL = 36
PVT_HACC = 40
PVT_VACC = 44
PVT_GSPEED = 60
PVT_HEADMOT = 64
# u-blox UBX-NAV-PVT accuracy estimates for speed and heading-of-motion.
PVT_SACC = 68      # speed accuracy estimate, mm/s
PVT_HEADACC = 72   # heading-of-motion accuracy estimate, 1e-5 deg

# UBX-NAV-STATUS payload offsets.
STA_ITOW = 0
STA_GPSFIX = 4
STA_FLAGS = 5
STA_FIXSTAT = 6
STA_TTFF = 8
STA_MSSS = 12


def _ubx_checksum(data: bytes) -> bytes:
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def _build_ubx(message_class: int, message_id: int, payload: bytes) -> bytes:
    length = len(payload)
    header = bytes(
        [
            UBX_SYNC1,
            UBX_SYNC2,
            message_class,
            message_id,
            length & 0xFF,
            (length >> 8) & 0xFF,
        ]
    )
    return header + payload + _ubx_checksum(header[2:] + payload)


def _nav_status(gnss_fix_ok: int, diff_soln: int, carr_soln: int) -> int:
    if not gnss_fix_ok:
        return NavSatStatus.STATUS_NO_FIX
    if carr_soln == 2:
        return NavSatStatus.STATUS_GBAS_FIX
    if carr_soln == 1:
        return NavSatStatus.STATUS_SBAS_FIX
    if diff_soln:
        return NavSatStatus.STATUS_SBAS_FIX
    return NavSatStatus.STATUS_FIX


def _fix_type_string(fix_type: int) -> str:
    values = {
        0: "No fix",
        1: "Dead reckoning",
        2: "2D-fix",
        3: "3D-fix",
        4: "GNSS+DR",
        5: "Time only",
    }
    return values.get(fix_type, f"Unknown({fix_type})")


def _rtk_state(gnss_fix_ok: int, diff_soln: int, carr_soln: int) -> str:
    if carr_soln == 2:
        return "RTK Fixed"
    if carr_soln == 1:
        return "RTK Float"
    if diff_soln:
        return "DGNSS"
    if gnss_fix_ok:
        return "3D Fix"
    return "No Fix"


def _parse_rtcm3_types(buf: bytes, counter: dict) -> None:
    """RTCM3 프레임의 메시지 유형을 세어 counter 에 누적한다.

    RTCM3 프레임: 0xD3 + 2바이트(길이 10bit) + 2바이트(유형 12bit 상위) +
    payload + 3바이트 CRC24. 손상된 프레임은 건너뛰며 유형만 집계한다.
    """
    i = 0
    n = len(buf)
    while i + 6 <= n:
        if buf[i] != 0xD3:
            i += 1
            continue
        msg_len = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        msg_type = (buf[i + 3] << 4) | (buf[i + 4] >> 4)
        frame = 6 + msg_len
        if i + frame > n:
            break
        counter[msg_type] = counter.get(msg_type, 0) + 1
        i += frame


def _bits(buf: bytes, bit_offset: int, nbits: int) -> int:
    """buf 에서 bit_offset(MSB-first) 부터 nbits 를 읽는다."""
    val = 0
    for k in range(nbits):
        pos = bit_offset + k
        val = (val << 1) | ((buf[pos >> 3] >> (7 - (pos & 7))) & 1)
    return val


def _ecef_to_llh(x_m: float, y_m: float, z_m: float):
    """WGS84 ECEF(m) → (lat_deg, lon_deg, height_m)."""
    a = 6378137.0
    e2 = 6.69437999014e-3
    lon = math.atan2(y_m, x_m)
    p = math.sqrt(x_m * x_m + y_m * y_m)
    lat = math.atan2(z_m, p * (1.0 - e2))
    for _ in range(12):
        n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z_m, p * (1.0 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


def _parse_rtcm_1006_coord(buf: bytes):
    """RTCM3 스트림에서 1006(기준국 좌표)을 찾아 (lat, lon, h, station_id) 반환.

    없으면 None. 1006 payload: msg_type(12) + station_id(12) + itrf_year(6) +
    gps/glo/gal/ref flags(4) + ECEF-X/Y/Z(각 38bit, signed 0.1mm) + antenna_h(16).
    """
    i = 0
    n = len(buf)
    while i + 6 <= n:
        if buf[i] != 0xD3:
            i += 1
            continue
        msg_len = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        msg_type = (buf[i + 3] << 4) | (buf[i + 4] >> 4)
        frame = 6 + msg_len
        if i + frame > n:
            break
        if msg_type == 1006 and msg_len >= 21:
            payload = buf[i + 3:i + 3 + msg_len]
            station_id = _bits(payload, 12, 12)
            bit = 34
            x = _bits(payload, bit, 38)
            y = _bits(payload, bit + 38, 38)
            z = _bits(payload, bit + 76, 38)
            x_m = (x - (1 << 38) if x >= (1 << 37) else x) * 0.0001
            y_m = (y - (1 << 38) if y >= (1 << 37) else y) * 0.0001
            z_m = (z - (1 << 38) if z >= (1 << 37) else z) * 0.0001
            lat, lon, h = _ecef_to_llh(x_m, y_m, z_m)
            return lat, lon, h, station_id, x_m, y_m, z_m
        i += frame
    return None


class StreamParser:
    """Extract UBX NAV-PVT/NAV-STATUS and NMEA GGA from a mixed stream."""

    MAX_BUF = 16384

    def __init__(self, pvt_callback, status_callback, gga_callback):
        self._ubx_buf = bytearray()
        self._mixed_buf = bytearray()
        self._pvt_callback = pvt_callback
        self._status_callback = status_callback
        self._gga_callback = gga_callback

    def feed(self, data: bytes) -> None:
        self._ubx_buf.extend(data)
        self._mixed_buf.extend(data)
        if len(self._ubx_buf) > self.MAX_BUF:
            self._ubx_buf = self._ubx_buf[-self.MAX_BUF :]
        if len(self._mixed_buf) > self.MAX_BUF:
            self._mixed_buf = self._mixed_buf[-self.MAX_BUF :]

    def parse(self) -> None:
        self._parse_ubx()
        self._parse_nmea()

    def _parse_ubx(self) -> None:
        buf = self._ubx_buf
        while len(buf) >= 2:
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == UBX_SYNC1 and buf[i + 1] == UBX_SYNC2:
                    idx = i
                    break

            if idx == -1:
                self._ubx_buf = bytearray(buf[-1:])
                return
            if idx > 0:
                del buf[:idx]
            if len(buf) < 8:
                return

            payload_len = struct.unpack_from("<H", buf, 4)[0]
            total_len = 6 + payload_len + 2
            if len(buf) < total_len:
                return

            packet_body = bytes(buf[2 : 6 + payload_len])
            payload = bytes(buf[6 : 6 + payload_len])
            ck_recv = bytes(buf[6 + payload_len : total_len])

            if _ubx_checksum(packet_body) == ck_recv:
                message_class = packet_body[0]
                message_id = packet_body[1]
                if message_class == UBX_CLASS_NAV and message_id == UBX_ID_PVT:
                    self._pvt_callback(payload)
                elif message_class == UBX_CLASS_NAV and message_id == UBX_ID_STATUS:
                    self._status_callback(payload)

            del buf[:total_len]

    def _parse_nmea(self) -> None:
        buf = self._mixed_buf
        while buf:
            sync_idx = self._find_ubx_sync(buf)
            dollar_idx = buf.find(b"$")

            if sync_idx == -1 and dollar_idx == -1:
                self._mixed_buf = bytearray(buf[-1:])
                return

            if sync_idx != -1 and (dollar_idx == -1 or sync_idx < dollar_idx):
                if sync_idx > 0:
                    del buf[:sync_idx]
                if len(buf) < 6:
                    return
                payload_len = struct.unpack_from("<H", buf, 4)[0]
                total_len = 6 + payload_len + 2
                if len(buf) < total_len:
                    return
                del buf[:total_len]
                continue

            if dollar_idx > 0:
                del buf[:dollar_idx]

            newline_idx = buf.find(b"\n")
            if newline_idx == -1:
                if len(buf) > 256:
                    del buf[:-256]
                return

            line = bytes(buf[:newline_idx]).replace(b"\r", b"")
            del buf[: newline_idx + 1]
            try:
                sentence = line.decode("ascii", errors="ignore").strip()
            except UnicodeDecodeError:
                continue
            if sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA"):
                self._gga_callback(sentence)

    @staticmethod
    def _find_ubx_sync(buf: bytearray) -> int:
        for i in range(len(buf) - 1):
            if buf[i] == UBX_SYNC1 and buf[i + 1] == UBX_SYNC2:
                return i
        return -1


class GpsNode(Node):
    def __init__(self):
        super().__init__("gps_node")

        self.declare_parameter("port", "/dev/ttyGPS")
        self.declare_parameter("baudrate", 38400)
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("fix_topic", "/gps/fix")
        self.declare_parameter("vel_topic", "/gps/vel")
        self.declare_parameter("heading_topic", "/gps/heading")
        # 조정가능 — 모션 헤딩 발행 최소 지면속도
        # description.launch 는 0.15 로 덮어씀. bootstrap creep 0.2 보다 낮아야 헤딩이 나온다.
        self.declare_parameter("heading_min_speed_mps", 0.15)
        # 조정가능 — /gps/heading 신뢰도 게이트 (_heading_is_trustworthy).
        # headAcc는 수신기 heading 정확도(도). speed_acc_factor는 gSpeed가
        # speed accuracy의 몇 배를 넘어야 이동으로 볼지.
        self.declare_parameter("heading_max_acc_deg", 45.0)
        self.declare_parameter("heading_speed_acc_factor", 0.0)
        self.declare_parameter("heading_yaw_variance", 0.05)
        # NGII 네트워크RTK(VRS). 비밀번호는 소스에 저장하지 않고 launch 환경변수로 주입.
        # 계정당 동시접속 1개. TCP가 비정상 종료되면 caster에 유령 세션이 남아 401이 난다.
        self.declare_parameter("ntrip_host", "rts2.ngii.go.kr")
        self.declare_parameter("ntrip_port", 2101)
        self.declare_parameter("ntrip_user", "")
        self.declare_parameter("ntrip_pass", "")
        self.declare_parameter("mountpoint", "VRS-RTCM32")
        self.declare_parameter("reconnect_delay", 30.0)
        self.declare_parameter("gga_interval", 1.0)
        self.declare_parameter("diag_log_interval", 10.0)
        self.declare_parameter("enable_nmea_gga", True)

        self._port = self.get_parameter("port").value
        self._baud = int(self.get_parameter("baudrate").value)
        self._frame = self.get_parameter("frame_id").value
        self._fix_topic = self.get_parameter("fix_topic").value
        self._vel_topic = self.get_parameter("vel_topic").value
        self._heading_topic = self.get_parameter("heading_topic").value
        self._heading_min_speed = float(self.get_parameter("heading_min_speed_mps").value)
        self._heading_max_acc_deg = float(
            self.get_parameter("heading_max_acc_deg").value
        )
        self._heading_speed_acc_factor = float(
            self.get_parameter("heading_speed_acc_factor").value
        )
        self._heading_yaw_variance = float(self.get_parameter("heading_yaw_variance").value)
        self._nhost = self.get_parameter("ntrip_host").value
        self._nport = int(self.get_parameter("ntrip_port").value)
        self._nuser = self.get_parameter("ntrip_user").value
        self._npass = self.get_parameter("ntrip_pass").value
        self._mount = self.get_parameter("mountpoint").value
        self._rdelay = float(self.get_parameter("reconnect_delay").value)
        self._gga_interval = float(self.get_parameter("gga_interval").value)
        self._diag_log_interval = float(self.get_parameter("diag_log_interval").value)
        self._enable_nmea_gga = bool(self.get_parameter("enable_nmea_gga").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_fix = self.create_publisher(NavSatFix, self._fix_topic, qos)
        self._pub_vel = self.create_publisher(TwistStamped, self._vel_topic, qos)
        self._pub_heading = self.create_publisher(
            PoseWithCovarianceStamped, self._heading_topic, qos
        )
        # Foxglove-only visualization topic. This is intentionally separate
        # from /gps/heading so a held/stale heading is never fed back to EKF.
        self._pub_heading_visual = self.create_publisher(
            PoseWithCovarianceStamped, '/gps/heading_visual', qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._visual_odom_callback, 10
        )
        self._visual_timer = self.create_timer(
            0.2, self._publish_visual_heading
        )

        self.declare_parameter("pose_covariance_floor", GPS_POSE_COVARIANCE_FLOOR)
        self._pose_cov_floor = float(
            self.get_parameter("pose_covariance_floor").value
        )
        self._pub_odom_gps = self.create_publisher(
            Odometry, '/odometry/gps', 10
        )
        self._odom_gps_raw_sub = self.create_subscription(
            Odometry, '/odometry/gps_raw', self._covariance_floor_callback, 10
        )

        self.declare_parameter("speed_variance", 0.004)
        self.declare_parameter("lateral_velocity_variance", 10.0)
        self.declare_parameter("angular_velocity_variance", 10.0)
        self.declare_parameter("stationary_speed_threshold", 0.02)
        self.declare_parameter("min_wheel_speed_for_sign", 0.02)
        self.declare_parameter("wheel_odom_timeout", 1.0)
        self._speed_var = float(self.get_parameter("speed_variance").value)
        self._lat_var = float(self.get_parameter("lateral_velocity_variance").value)
        self._ang_var = float(self.get_parameter("angular_velocity_variance").value)
        self._stat_speed = float(self.get_parameter("stationary_speed_threshold").value)
        self._min_wheel_sign = float(self.get_parameter("min_wheel_speed_for_sign").value)
        self._wheel_odom_timeout = float(self.get_parameter("wheel_odom_timeout").value)
        self._pub_odom_vel = self.create_publisher(
            Odometry, '/odometry/gps_velocity', 10
        )
        self._wheel_odom_sub = self.create_subscription(
            Odometry, '/odom', self._wheel_odom_callback, 10
        )
        self._last_wheel_odom_time = None
        self._last_wheel_vx = 0.0

        self._ser = None
        self._stop = threading.Event()
        self._serial_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._ntrip_sock_lock = threading.Lock()
        self._ntrip_sock = None

        self._last_gga_raw = ""
        self._last_gga_quality = None
        self._last_gga_numsv = ""
        self._last_gga_hdop = ""
        self._last_status = {}
        self._last_pvt = {}
        self._last_diag_log = 0.0
        self._ntrip_status = "idle"
        self._ntrip_bytes = 0
        self._ntrip_gga_count = 0
        self._rtcm_types = {}
        self._visual_x = 0.0
        self._visual_y = 0.0
        self._visual_yaw = 0.0
        self._last_gps_heading_yaw = None
        self._last_headmot_raw = None
        self._heading_reject_acc = 0
        self._heading_reject_frozen = 0
        self._heading_reject_speed = 0

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.2)
            self.get_logger().info(f"GPS port connected: {self._port}")
        except serial.SerialException as exc:
            self.get_logger().error(f"GPS port connection failed: {exc}")
            raise RuntimeError("GPS connection failed") from exc

        # Soft recover: NTRIP socket bounce + serial reopen. 노드 재시작 없음.
        self._recover_srv = self.create_service(
            Trigger, "/gps/recover", self._recover_callback
        )

        self._t_reader = threading.Thread(target=self._reader_thread, daemon=True)
        self._t_ntrip = threading.Thread(target=self._ntrip_thread, daemon=True)
        self._t_reader.start()
        self._t_ntrip.start()

        self.get_logger().info(
            f"GPS node started: NTRIP {self._nhost}:{self._nport}/{self._mount}, "
            f"fix_topic={self._fix_topic}, soft recover=/gps/recover"
        )

    def _ntrip_request(self) -> bytes:
        credentials = base64.b64encode(
            f"{self._nuser}:{self._npass}".encode("utf-8")
        ).decode("ascii")
        mountpoint = self._mount if self._mount.startswith("/") else f"/{self._mount}"
        request = (
            f"GET {mountpoint} HTTP/1.0\r\n"
            f"Host: {self._nhost}\r\n"
            "User-Agent: NTRIP ROS2Client/1.0\r\n"
            f"Authorization: Basic {credentials}\r\n"
            "Ntrip-Version: Ntrip/1.0\r\n"
            "Connection: close\r\n\r\n"
        )
        return request.encode("ascii")

    def _ntrip_thread(self) -> None:
        request = self._ntrip_request()
        while not self._stop.is_set():
            sock = None
            try:
                self._set_ntrip_status(f"connecting {self._nhost}")
                sock = socket.create_connection((self._nhost, self._nport), timeout=10)
                with self._ntrip_sock_lock:
                    self._ntrip_sock = sock
                sock.sendall(request)

                header = b""
                while not self._stop.is_set():
                    chunk = sock.recv(1)
                    if not chunk:
                        break
                    header += chunk
                    if b"\r\n\r\n" in header:
                        break
                    if len(header) > 512:
                        break

                lines = header.decode("utf-8", errors="ignore").splitlines()
                first = lines[0] if lines else ""
                if "200" not in first and "ICY" not in first:
                    self._set_ntrip_status(f"auth failed: {first}")
                    self.get_logger().error(f"[NTRIP] authentication failed: {first}")
                    self._stop.wait(timeout=self._rdelay)
                    continue

                self._set_ntrip_status(f"connected {self._mount}")
                self.get_logger().info(f"[NTRIP] connected: {first.strip()}")
                sock.settimeout(0.2)
                last_gga_sent = 0.0

                while not self._stop.is_set():
                    try:
                        rtcm = sock.recv(4096)
                        if rtcm:
                            _parse_rtcm3_types(rtcm, self._rtcm_types)
                            coord = _parse_rtcm_1006_coord(rtcm)
                            if coord is not None:
                                lat, lon, h, sid, x_m, y_m, z_m = coord
                                self.get_logger().info(
                                    f"[NTRIP] VRS base: station={sid} "
                                    f"lat={lat:.6f} lon={lon:.6f} h={h:.2f}m "
                                    f"ecef=({x_m:.1f},{y_m:.1f},{z_m:.1f})"
                                )
                            with self._serial_lock:
                                if self._ser is not None and self._ser.is_open:
                                    self._ser.write(rtcm)
                            with self._state_lock:
                                self._ntrip_bytes += len(rtcm)
                        elif rtcm == b"":
                            self._set_ntrip_status("server closed; reconnecting")
                            break
                    except socket.timeout:
                        pass
                    except OSError:
                        self._set_ntrip_status("socket closed; reconnecting")
                        break

                    now = time.monotonic()
                    if now - last_gga_sent >= self._gga_interval:
                        gga = self._get_gga_raw()
                        if gga:
                            try:
                                sock.sendall(gga.encode("ascii"))
                                last_gga_sent = now
                                with self._state_lock:
                                    self._ntrip_gga_count += 1
                            except OSError:
                                break

                    self._update_ntrip_stream_status()

            except (socket.timeout, ConnectionRefusedError, OSError) as exc:
                if not self._stop.is_set():
                    self._set_ntrip_status(f"connection error: {exc}")
                    self.get_logger().warn(
                        f"[NTRIP] connection error: {exc}; reconnecting in {self._rdelay}s"
                    )
                    self._stop.wait(timeout=self._rdelay)
            except Exception as exc:
                if not self._stop.is_set():
                    self._set_ntrip_status(f"{type(exc).__name__}: {exc}")
                    self.get_logger().error(f"[NTRIP] error: {exc}")
                    self._stop.wait(timeout=self._rdelay)
            finally:
                with self._ntrip_sock_lock:
                    if self._ntrip_sock is sock:
                        self._ntrip_sock = None
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def _reader_thread(self) -> None:
        self._enable_messages()
        parser = StreamParser(
            pvt_callback=self._on_nav_pvt,
            status_callback=self._on_nav_status,
            gga_callback=self._on_gga,
        )

        while not self._stop.is_set():
            try:
                with self._serial_lock:
                    ser = self._ser
                    if ser is None or not ser.is_open:
                        data = b""
                    else:
                        nbytes = ser.in_waiting or 1
                        data = ser.read(nbytes)
                if not data:
                    time.sleep(0.05)
                    continue
                parser.feed(data)
                parser.parse()
            except serial.SerialException as exc:
                if not self._stop.is_set():
                    self.get_logger().error(
                        f"[Reader] serial error: {exc}; waiting for /gps/recover"
                    )
                # 스레드를 유지한다. /gps/recover가 포트를 다시 연다.
                self._stop.wait(timeout=0.5)
            except Exception as exc:
                if not self._stop.is_set():
                    self.get_logger().warn(f"[Reader] error: {exc}")

    def _reopen_serial(self) -> bool:
        """Close and reopen the GNSS serial port (soft recover, no process restart)."""
        with self._serial_lock:
            try:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
            except Exception as exc:
                self.get_logger().warn(f"[Recover] serial close: {exc}")
            self._ser = None
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=0.2)
                self.get_logger().info(f"[Recover] GPS port reopened: {self._port}")
            except serial.SerialException as exc:
                self.get_logger().error(f"[Recover] GPS port reopen failed: {exc}")
                return False
        try:
            self._enable_messages()
        except Exception as exc:
            self.get_logger().warn(f"[Recover] UBX re-enable: {exc}")
        return True

    def _bounce_ntrip_socket(self) -> None:
        """Force NTRIP client loop to drop the current socket and reconnect."""
        with self._ntrip_sock_lock:
            sock = self._ntrip_sock
            self._ntrip_sock = None
        if sock is not None:
            try:
                sock.close()
            except Exception as exc:
                self.get_logger().warn(f"[Recover] NTRIP close: {exc}")
            self._set_ntrip_status("recover: socket bounced")
            self.get_logger().info("[Recover] NTRIP socket closed for reconnect")

    def _recover_callback(self, _request, response):
        """Soft GPS recover: serial reopen + NTRIP bounce. No node restart."""
        ok_serial = self._reopen_serial()
        self._bounce_ntrip_socket()
        response.success = bool(ok_serial)
        response.message = (
            "soft recover: serial "
            + ("ok" if ok_serial else "FAILED")
            + ", NTRIP bounce requested"
        )
        self.get_logger().info(f"[Recover] {response.message}")
        return response
    def _enable_messages(self) -> None:
        self._enable_msg(UBX_CLASS_NAV, UBX_ID_PVT)
        self._enable_msg(UBX_CLASS_NAV, UBX_ID_STATUS)
        if self._enable_nmea_gga:
            self._enable_msg(UBX_CLASS_NMEA, UBX_ID_GGA)

    def _enable_msg(self, message_class: int, message_id: int, rate: int = 1) -> None:
        # CFG-MSG v0 rates: I2C, UART1, UART2, USB, SPI, reserved.
        payload = bytes([message_class, message_id, 0, 0, 0, rate, 0, 0])
        message = _build_ubx(UBX_CLASS_CFG, UBX_ID_CFG_MSG, payload)
        with self._serial_lock:
            self._ser.write(message)
        time.sleep(0.05)

    def _on_nav_pvt(self, payload: bytes) -> None:
        if len(payload) < 84:
            return

        year = struct.unpack_from("<H", payload, PVT_YEAR)[0]
        month = payload[PVT_MONTH]
        day = payload[PVT_DAY]
        hour = payload[PVT_HOUR]
        minute = payload[PVT_MIN]
        sec = payload[PVT_SEC]
        valid = payload[PVT_VALID]
        fix_type = payload[PVT_FIXTYPE]
        flags = payload[PVT_FLAGS]
        numsv = payload[PVT_NUMSV]
        lon_raw = struct.unpack_from("<i", payload, PVT_LON)[0]
        lat_raw = struct.unpack_from("<i", payload, PVT_LAT)[0]
        hmsl = struct.unpack_from("<i", payload, PVT_HMSL)[0]
        hacc = struct.unpack_from("<I", payload, PVT_HACC)[0]
        vacc = struct.unpack_from("<I", payload, PVT_VACC)[0]
        gspeed = struct.unpack_from("<i", payload, PVT_GSPEED)[0]
        heading = struct.unpack_from("<i", payload, PVT_HEADMOT)[0]
        sacc = struct.unpack_from("<I", payload, PVT_SACC)[0]
        headacc = struct.unpack_from("<I", payload, PVT_HEADACC)[0]

        gnss_fix_ok = flags & 0x01
        diff_soln = (flags >> 1) & 0x01
        carr_soln = (flags >> 6) & 0x03

        lat = lat_raw * 1e-7
        lon = lon_raw * 1e-7
        alt_msl = hmsl * 1e-3
        hacc_m = hacc * 1e-3
        vacc_m = vacc * 1e-3
        utc = ""
        if valid & 0x03:
            utc = f"{year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{sec:02d} UTC"

        with self._state_lock:
            self._last_pvt = {
                "utc": utc,
                "fix_type": fix_type,
                "flags": flags,
                "gnss_fix_ok": gnss_fix_ok,
                "diff_soln": diff_soln,
                "carr_soln": carr_soln,
                "numsv": numsv,
                "lat": lat if gnss_fix_ok else None,
                "lon": lon if gnss_fix_ok else None,
                "alt_msl": alt_msl if gnss_fix_ok else None,
                "hacc_m": hacc_m,
                "vacc_m": vacc_m,
            }

        now = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = self._frame
        fix.status.status = _nav_status(gnss_fix_ok, diff_soln, carr_soln)
        fix.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS

        if gnss_fix_ok:
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt_msl
        else:
            fix.latitude = float("nan")
            fix.longitude = float("nan")
            fix.altitude = float("nan")

        h_var = hacc_m * hacc_m
        v_var = vacc_m * vacc_m
        fix.position_covariance = [
            h_var,
            0.0,
            0.0,
            0.0,
            h_var,
            0.0,
            0.0,
            0.0,
            v_var,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub_fix.publish(fix)

        vel = TwistStamped()
        vel.header.stamp = now
        vel.header.frame_id = self._frame
        speed_ms = gspeed * 1e-3
        heading_rad = math.radians(heading * 1e-5)
        vel.twist.linear.x = speed_ms * math.cos(heading_rad)
        vel.twist.linear.y = speed_ms * math.sin(heading_rad)
        self._pub_vel.publish(vel)
        self._publish_velocity_odom(now, speed_ms)

        if self._heading_is_trustworthy(
            gnss_fix_ok, speed_ms, sacc, headacc, heading
        ):
            self._publish_heading(now, heading_rad)

        self._log_diag_if_due()

    def _heading_is_trustworthy(
        self, gnss_fix_ok, speed_ms, sacc_mm_s, headacc_1e5deg, headmot_raw
    ):
        """True when this epoch's heading-of-motion may be published.

        A speed floor alone can republish a frozen headMot while gSpeed noise
        crosses the threshold. The gate uses the receiver's own accuracy
        estimates and rejects an unchanged heading.
        """
        if not gnss_fix_ok:
            return False

        # 1) Speed must beat both the fixed floor and the receiver's own speed
        #    accuracy, so noise alone can never qualify.
        sacc_floor = self._heading_speed_acc_factor * (sacc_mm_s * 1e-3)
        speed_floor = max(self._heading_min_speed, sacc_floor)
        if speed_ms < speed_floor:
            # Only the "moving, but the receiver does not trust its own speed"
            # case is worth counting. Below the fixed floor the robot is simply
            # not driving, and that happens on every stationary epoch — folding
            # it in would make the counter climb forever and say nothing.
            if speed_ms >= self._heading_min_speed:
                self._heading_reject_speed += 1
            return False

        # 2) The receiver's heading accuracy estimate must be usable.
        headacc_deg = headacc_1e5deg * 1e-5
        if headacc_deg <= 0.0 or headacc_deg > self._heading_max_acc_deg:
            self._heading_reject_acc += 1
            return False

        # 3) An unchanged raw headMot means the register is held, not measured.
        #    Compared on the raw integer so no float rounding hides it.
        if headmot_raw == self._last_headmot_raw:
            self._heading_reject_frozen += 1
            return False
        self._last_headmot_raw = headmot_raw
        return True

    def _publish_heading(self, stamp, heading_true_rad: float) -> None:
        # PVT_HEADMOT는 진북(true north) 기준 시계방향 각도(compass bearing)이고
        # ROS/ENU 관례는 동쪽(X축) 기준 반시계방향이므로 변환이 필요하다.
        # 정지/저속 상태에서는 위성 움직임 기반 heading이 노이즈로 흔들리므로
        # heading_min_speed_mps 이상일 때만(호출부에서 게이팅) 발행한다.
        yaw_ros = math.atan2(
            math.sin((math.pi / 2.0) - heading_true_rad),
            math.cos((math.pi / 2.0) - heading_true_rad),
        )
        self._last_gps_heading_yaw = yaw_ros

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.z = math.sin(yaw_ros / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw_ros / 2.0)

        covariance = [0.0] * 36
        covariance[35] = self._heading_yaw_variance
        pose.pose.covariance = covariance

        self._pub_heading.publish(pose)

    def _visual_odom_callback(self, msg: Odometry) -> None:
        self._visual_x = msg.pose.pose.position.x
        self._visual_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self._visual_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z),
            1.0 - 2.0 * (orientation.z * orientation.z),
        )

    def _publish_visual_heading(self) -> None:
        """Publish a Foxglove-only arrow continuously, without EKF feedback."""
        yaw = (
            self._last_gps_heading_yaw
            if self._last_gps_heading_yaw is not None
            else self._visual_yaw
        )
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = self._visual_x
        pose.pose.pose.position.y = self._visual_y
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.covariance[35] = self._heading_yaw_variance
        self._pub_heading_visual.publish(pose)

    def _covariance_floor_callback(self, msg: Odometry) -> None:
        corrected = list(msg.pose.covariance)
        corrected[0] = max(float(corrected[0]), self._pose_cov_floor)
        corrected[7] = max(float(corrected[7]), self._pose_cov_floor)
        msg.pose.covariance = corrected
        self._pub_odom_gps.publish(msg)

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        self._last_wheel_odom_time = self.get_clock().now()
        self._last_wheel_vx = msg.twist.twist.linear.x

    def _publish_velocity_odom(self, stamp, gps_speed_ms: float) -> None:
        signed_speed = gps_speed_ms
        if gps_speed_ms < self._stat_speed:
            signed_speed = 0.0
        else:
            sign = self._wheel_sign()
            if sign == 0:
                return
            signed_speed = sign * gps_speed_ms

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.covariance = [1e6] * 36
        odom.pose.covariance[0] = 1e6
        odom.pose.covariance[7] = 1e6
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6
        odom.twist.twist.linear.x = signed_speed
        odom.twist.covariance[0] = self._speed_var
        odom.twist.covariance[7] = self._lat_var
        odom.twist.covariance[35] = self._ang_var
        self._pub_odom_vel.publish(odom)

    def _wheel_sign(self) -> int:
        if self._last_wheel_odom_time is None:
            return 0
        age = (self.get_clock().now() - self._last_wheel_odom_time).nanoseconds / 1e9
        if age > self._wheel_odom_timeout:
            return 0
        if abs(self._last_wheel_vx) < self._min_wheel_sign:
            return 0
        return 1 if self._last_wheel_vx > 0.0 else -1

    def _on_nav_status(self, payload: bytes) -> None:
        if len(payload) < 16:
            return

        itow = struct.unpack_from("<I", payload, STA_ITOW)[0]
        gpsfix = payload[STA_GPSFIX]
        flags = payload[STA_FLAGS]
        fixstat = payload[STA_FIXSTAT]
        ttff = struct.unpack_from("<I", payload, STA_TTFF)[0]
        msss = struct.unpack_from("<I", payload, STA_MSSS)[0]

        with self._state_lock:
            self._last_status = {
                "itow": itow,
                "gpsfix": gpsfix,
                "flags": flags,
                "fix_ok": flags & 0x01,
                "diff_soln": (flags >> 1) & 0x01,
                "fixstat": fixstat,
                "diff_corr": fixstat & 0x01,
                "ttff": ttff,
                "msss": msss,
            }

    def _on_gga(self, sentence: str) -> None:
        parts = sentence.split(",")
        if len(parts) < 10:
            return

        with self._state_lock:
            self._last_gga_raw = sentence.strip() + "\r\n"
            self._last_gga_quality = parts[6]
            self._last_gga_numsv = parts[7]
            self._last_gga_hdop = parts[8]

    def _get_gga_raw(self) -> str:
        with self._state_lock:
            return self._last_gga_raw

    def _set_ntrip_status(self, status: str) -> None:
        with self._state_lock:
            self._ntrip_status = status

    def _update_ntrip_stream_status(self) -> None:
        with self._state_lock:
            self._ntrip_status = (
                f"RTCM {self._ntrip_bytes} bytes, GGA sent {self._ntrip_gga_count}"
            )

    def _log_diag_if_due(self) -> None:
        now = time.monotonic()
        if now - self._last_diag_log < self._diag_log_interval:
            return
        self._last_diag_log = now

        with self._state_lock:
            pvt = dict(self._last_pvt)
            sta = dict(self._last_status)
            gga_quality = self._last_gga_quality
            gga_numsv = self._last_gga_numsv
            gga_hdop = self._last_gga_hdop
            ntrip_status = self._ntrip_status
            ntrip_bytes = self._ntrip_bytes
            ntrip_gga_count = self._ntrip_gga_count
            rtcm_types = ",".join(
                f"{k}:{v}" for k, v in sorted(self._rtcm_types.items())
            )

        if not pvt:
            return

        rtk = _rtk_state(
            pvt.get("gnss_fix_ok", 0),
            pvt.get("diff_soln", 0),
            pvt.get("carr_soln", 0),
        )
        status_bits = ""
        if sta:
            status_bits = (
                f", status_fix={_fix_type_string(sta.get('gpsfix', 0))}, "
                f"status_diff={sta.get('diff_soln')}, diff_corr={sta.get('diff_corr')}"
            )

        self.get_logger().info(
            "GPS diag: "
            f"pvt={rtk}, fix={_fix_type_string(pvt.get('fix_type', 0))}, "
            f"numsv={pvt.get('numsv')}, hacc={pvt.get('hacc_m'):.3f}m, "
            f"vacc={pvt.get('vacc_m'):.3f}m, gga_quality={gga_quality}, "
            f"gga_numsv={gga_numsv}, gga_hdop={gga_hdop}, "
            f"ntrip='{ntrip_status}', rtcm_bytes={ntrip_bytes}, "
            f"gga_sent={ntrip_gga_count}, rtcm_types=[{rtcm_types}]{status_bits}, "
            # 창정렬이 왜 진행되지 않는지 필드에서 즉시 보이도록 노출한다.
            # hdg_rej_acc: headAcc 초과로 버린 횟수, hdg_rej_frozen: headMot 이
            # 직전과 동일(수신기 홀드)해서 버린 횟수, hdg_rej_speed: 고정 하한은
            # 넘겼지만 sAcc 기준 하한에 못 미쳐 버린 횟수(= 움직이긴 했는데
            # 수신기가 자기 속도를 못 믿는 상태. 손으로 흔들면 여기가 오른다).
            f"hdg_rej_acc={self._heading_reject_acc}, "
            f"hdg_rej_frozen={self._heading_reject_frozen}, "
            f"hdg_rej_speed={self._heading_reject_speed}"
        )

    def destroy_node(self):
        self.get_logger().info("Stopping GPS node...")
        self._stop.set()
        if hasattr(self, "_t_ntrip"):
            self._t_ntrip.join(timeout=2)
        if hasattr(self, "_t_reader"):
            self._t_reader.join(timeout=2)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"[gps_node] error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
