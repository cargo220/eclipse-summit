"""One Dynamixel serial bus (one CM-900 / OpenCM)."""

from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)


class DxlBus:
    """Port + packet handler + GroupSync objects for the IDs on one board."""

    def __init__(
        self,
        name,
        device,
        wheel_ids,
        height_ids,
        protocol_version,
        extra_ids=(),
    ):
        self.name = str(name)
        self.device = str(device)
        self.wheel_ids = tuple(int(dxl_id) for dxl_id in wheel_ids)
        self.height_ids = tuple(int(dxl_id) for dxl_id in height_ids)
        self.extra_ids = tuple(int(dxl_id) for dxl_id in extra_ids)
        self.ids = self.wheel_ids + self.height_ids + self.extra_ids
        self._id_set = set(self.ids)
        self.port_handler = PortHandler(self.device)
        self.packet_handler = PacketHandler(protocol_version)
        self.sync_write_vel = None
        self.sync_write_velocity_gains = None
        self.sync_write_height = None
        self.sync_read_vel = None
        self.sync_read_current = None
        self.sync_read_hwerr = None
        self.sync_read_watchdog = None
        self.sync_read_pwm = None
        self.sync_read_volt_temp = None

    def owns(self, dxl_id):
        return int(dxl_id) in self._id_set

    def build_groups(self, **pairs):
        """Create GroupSync objects. Each value is (address, length)."""
        port = self.port_handler
        packet = self.packet_handler
        self.sync_write_vel = GroupSyncWrite(port, packet, *pairs["goal_velocity"])
        self.sync_write_velocity_gains = GroupSyncWrite(
            port, packet, *pairs["velocity_gains"]
        )
        self.sync_write_height = GroupSyncWrite(
            port, packet, *pairs["goal_position"]
        )
        self.sync_read_vel = GroupSyncRead(port, packet, *pairs["present_velocity"])
        self.sync_read_current = GroupSyncRead(
            port, packet, *pairs["present_current"]
        )
        self.sync_read_hwerr = GroupSyncRead(port, packet, *pairs["hardware_error"])
        self.sync_read_watchdog = GroupSyncRead(port, packet, *pairs["bus_watchdog"])
        self.sync_read_pwm = GroupSyncRead(port, packet, *pairs["present_pwm"])
        self.sync_read_volt_temp = GroupSyncRead(
            port, packet, *pairs["voltage_temperature"]
        )
        for dxl_id in self.wheel_ids:
            self.sync_read_vel.addParam(dxl_id)
            self.sync_read_current.addParam(dxl_id)
            self.sync_read_hwerr.addParam(dxl_id)
            self.sync_read_watchdog.addParam(dxl_id)
            self.sync_read_pwm.addParam(dxl_id)
            self.sync_read_volt_temp.addParam(dxl_id)

    def open(self, baudrate, timeout=0.01):
        if not self.port_handler.openPort():
            raise RuntimeError(
                f"Failed to open Dynamixel {self.name} port {self.device}"
            )
        if not self.port_handler.setBaudRate(int(baudrate)):
            raise RuntimeError(
                f"Failed to set Dynamixel {self.name} baudrate {baudrate}"
            )
        self.port_handler.ser.timeout = timeout

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass

    def ping(self, dxl_id):
        try:
            _model, comm, err = self.packet_handler.ping(
                self.port_handler,
                int(dxl_id),
            )
        except Exception:
            return False
        return comm == COMM_SUCCESS and err == 0

    def tx_result_text(self, comm_result):
        return self.packet_handler.getTxRxResult(comm_result)

    def rx_error_text(self, dxl_error):
        return self.packet_handler.getRxPacketError(dxl_error)
