import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    import serial
except ImportError:  # pragma: no cover - handled at runtime on the robot.
    serial = None


# Drop a partial line that never terminates rather than growing forever. One
# probe line is ~30 bytes, so this tolerates a long burst of noise before
# resetting.
MAX_RX_BUFFER_BYTES = 4096


def extract_complete_lines(buffer, chunk, max_buffer=MAX_RX_BUFFER_BYTES):
    """Split a serial byte stream into whole lines, keeping the remainder.

    Pure function so the framing can be tested without a serial port.

    Returns ``(lines, remaining_buffer)`` where ``lines`` are decoded and
    stripped, and ``remaining_buffer`` holds the bytes after the last newline
    — i.e. a partial line still in flight, which must be carried into the next
    read instead of being parsed.

    This exists because ``Serial.readline()`` with a short timeout returns
    whatever arrived when the timeout expires, newline or not. With
    serial_timeout=0.02 that split single probe lines in two: the head was
    rejected as malformed and the tail showed up on the next tick as its own
    "malformed" line (e.g. "4,114.68,OK"). Live logs showed ~1% of lines lost
    this way, which matters because probe_angle is a training-dataset feature.
    """
    buffer = buffer + chunk
    if len(buffer) > max_buffer:
        # Keep only the tail after the last newline; if there is none, drop it.
        cut = buffer.rfind(b'\n')
        buffer = buffer[cut + 1:] if cut >= 0 else b''

    lines = []
    while True:
        index = buffer.find(b'\n')
        if index < 0:
            break
        raw, buffer = buffer[:index], buffer[index + 1:]
        lines.append(raw.decode('utf-8', errors='ignore').strip())
    return lines, buffer


class ProbeSensorNode(Node):
    """Read the Arduino probe angle stream and publish ROS topics."""

    def __init__(self):
        super().__init__('probe_sensor_node')

        self.declare_parameter('port', '/dev/ttyACM3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.02)
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('publish_prefix', '/probe')
        self.declare_parameter('ground_max_deg', 25.0)
        self.declare_parameter('mudflat_max_deg', 70.0)
        self.declare_parameter('water_min_deg', 100.0)
        self.declare_parameter('connected_publish_period', 1.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.publish_prefix = self.get_parameter('publish_prefix').value.rstrip('/')
        self.ground_max_deg = float(self.get_parameter('ground_max_deg').value)
        self.mudflat_max_deg = float(self.get_parameter('mudflat_max_deg').value)
        self.water_min_deg = float(self.get_parameter('water_min_deg').value)
        self.connected_publish_period = float(
            self.get_parameter('connected_publish_period').value
        )
        self._validate_parameters()

        self.raw_pub = self.create_publisher(Float32, self._topic('raw'), 10)
        self.voltage_pub = self.create_publisher(Float32, self._topic('voltage'), 10)
        self.angle_pub = self.create_publisher(Float32, self._topic('angle'), 10)
        # 전환 시점과 주기로 연결 상태를 발행한다.
        self.connected_pub = self.create_publisher(Bool, self._topic('connected'), 10)

        self._ser = None
        self._connected = None
        self._last_connect_attempt = 0.0
        self._last_connected_publish_at = 0.0
        # Bytes of an incomplete line carried between timer ticks.
        self._rx_buffer = b''

        self._connect_serial()

        self.create_timer(self.timer_period, self._timer_callback)

    def _topic(self, name):
        return f'{self.publish_prefix}/{name}'

    def _validate_parameters(self):
        if not self.publish_prefix:
            self.get_logger().warn('publish_prefix is empty; using /probe')
            self.publish_prefix = '/probe'

        if self.serial_timeout < 0.0:
            self.get_logger().warn('serial_timeout must be >= 0. Using 0.02')
            self.serial_timeout = 0.02

        if self.timer_period <= 0.0:
            self.get_logger().warn('timer_period must be > 0. Using 0.05')
            self.timer_period = 0.05

        if self.reconnect_period <= 0.0:
            self.get_logger().warn('reconnect_period must be > 0. Using 2.0')
            self.reconnect_period = 2.0

        if self.connected_publish_period <= 0.0:
            self.get_logger().warn('connected_publish_period must be > 0. Using 1.0')
            self.connected_publish_period = 1.0

        if not self.ground_max_deg <= self.mudflat_max_deg < self.water_min_deg:
            self.get_logger().warn(
                'Expected ground_max_deg <= mudflat_max_deg < water_min_deg. '
                'Using defaults: 25.0, 70.0, 100.0'
            )
            self.ground_max_deg = 25.0
            self.mudflat_max_deg = 70.0
            self.water_min_deg = 100.0

    def _connect_serial(self):
        self._last_connect_attempt = time.monotonic()

        if serial is None:
            self.get_logger().error(
                'pyserial is not available. Install python3-serial or pyserial.'
            )
            self._set_connected(False)
            return

        try:
            self._ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.serial_timeout,
            )
            self._set_connected(True)
            self.get_logger().info(
                f'Probe Arduino connected: {self.port} @ {self.baudrate}'
            )
        except serial.SerialException as exc:
            self._ser = None
            self._set_connected(False)
            self.get_logger().warn(f'Probe Arduino not connected yet: {exc}')

    def _timer_callback(self):
        self._maybe_publish_connected()

        if self._ser is None:
            self._set_connected(False)
            if time.monotonic() - self._last_connect_attempt >= self.reconnect_period:
                self._connect_serial()
            return

        try:
            # Read what is buffered rather than readline(): with a 20 ms
            # timeout readline() hands back unterminated fragments, which is
            # what produced the "malformed probe line" losses. Fall back to a
            # 1-byte read so an idle port still honours serial_timeout instead
            # of spinning.
            pending = getattr(self._ser, 'in_waiting', 0) or 1
            chunk = self._ser.read(pending)
        except Exception as exc:
            self.get_logger().warn(f'Probe serial read failed: {exc}')
            self._close_serial()
            return

        lines, self._rx_buffer = extract_complete_lines(self._rx_buffer, chunk)

        for line in lines:
            if not line or line.startswith('#'):
                continue

            sample = self._parse_line(line)
            if sample is None:
                self.get_logger().warn(f'Ignoring malformed probe line: {line}')
                continue

            raw, voltage, angle_deg, _arduino_state = sample
            self._publish_sample(raw, voltage, angle_deg)

    def _parse_line(self, line):
        parts = [part.strip() for part in line.split(',')]
        if len(parts) < 4:
            return None

        try:
            raw = float(parts[0])
            voltage = float(parts[1])
            angle_deg = float(parts[2])
        except ValueError:
            return None

        arduino_state = parts[3].upper()
        if not self._is_valid_sample(raw, voltage, angle_deg, arduino_state):
            return None

        return raw, voltage, angle_deg, arduino_state

    def _is_valid_sample(self, raw, voltage, angle_deg, arduino_state):
        if not all(math.isfinite(value) for value in (raw, voltage, angle_deg)):
            return False

        if raw < 0.0 or raw > 1023.0:
            return False

        if voltage < 0.0 or voltage > 5.5:
            return False

        if angle_deg < 0.0 or angle_deg >= 360.0:
            return False

        allowed_states = {
            'OK',
            'CALIBRATING',
            'ERROR',
            'GROUND',
            'MUDFLAT',
            'WATER',
        }
        return arduino_state in allowed_states

    def _classify_angle(self, angle_deg):
        if angle_deg >= self.water_min_deg:
            return 'WATER'
        if self.ground_max_deg < angle_deg <= self.mudflat_max_deg:
            return 'MUDFLAT'
        if self.mudflat_max_deg < angle_deg < self.water_min_deg:
            return 'TRANSITION'
        return 'GROUND'

    def _publish_sample(self, raw, voltage, angle_deg):
        self._set_connected(True)

        raw_msg = Float32()
        raw_msg.data = float(raw)
        self.raw_pub.publish(raw_msg)

        voltage_msg = Float32()
        voltage_msg.data = float(voltage)
        self.voltage_pub.publish(voltage_msg)

        angle_msg = Float32()
        angle_msg.data = float(angle_deg)
        self.angle_pub.publish(angle_msg)

    def _set_connected(self, connected):
        if self._connected == connected:
            return

        self._connected = connected
        if self._connected:
            self.get_logger().info('Probe connected status: True')
        else:
            self.get_logger().warn('Probe connected status: False')
        self._publish_connected()

    def _publish_connected(self):
        msg = Bool()
        msg.data = bool(self._connected)
        self.connected_pub.publish(msg)
        self._last_connected_publish_at = time.monotonic()

    def _maybe_publish_connected(self):
        """주기 상태 재발행: 전환이 없어도 구독자가 최신 상태를 받게 한다."""
        if time.monotonic() - self._last_connected_publish_at >= self.connected_publish_period:
            self._publish_connected()

    def _close_serial(self, publish_status=True):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        # Drop any half-received line: after a reconnect it would splice onto
        # the first fresh bytes and corrupt that sample.
        self._rx_buffer = b''
        if publish_status:
            self._set_connected(False)
        else:
            self._connected = False

    def destroy_node(self):
        self._close_serial(publish_status=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ProbeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
