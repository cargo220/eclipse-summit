#!/usr/bin/env python3
"""Ping Dynamixel IDs on front/rear OpenCM. No torque or Goal writes.

Do not run while eclipse_test_controller holds the ports.
"""

import sys

from dynamixel_sdk import PacketHandler, PortHandler

from eclipse_pkg.eclipse_test_config import (
    BAUDRATE,
    DEVICENAME_FRONT,
    DEVICENAME_REAR,
    DXL_FRONT_BUS_IDS,
    DXL_REAR_BUS_IDS,
    PROTOCOL_VERSION,
    dxl_bus_name,
)


def ping_port(device, ids):
    port = PortHandler(device)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        print(f"OPEN FAIL {device}")
        return {}
    if not port.setBaudRate(BAUDRATE):
        print(f"BAUD FAIL {device}")
        port.closePort()
        return {}
    found = {}
    for dxl_id in ids:
        _model, comm, err = packet.ping(port, int(dxl_id))
        found[int(dxl_id)] = comm == 0 and err == 0
        print(
            f"{device} id={dxl_id} expected={dxl_bus_name(dxl_id)} "
            f"ok={found[int(dxl_id)]}"
        )
    port.closePort()
    return found


def main():
    print(f"front {DEVICENAME_FRONT} expect {DXL_FRONT_BUS_IDS}")
    front = ping_port(DEVICENAME_FRONT, DXL_FRONT_BUS_IDS + DXL_REAR_BUS_IDS)
    print(f"rear {DEVICENAME_REAR} expect {DXL_REAR_BUS_IDS}")
    rear = ping_port(DEVICENAME_REAR, DXL_FRONT_BUS_IDS + DXL_REAR_BUS_IDS)
    mismatch = False
    for dxl_id in DXL_FRONT_BUS_IDS:
        if front.get(int(dxl_id)) and rear.get(int(dxl_id)):
            print(f"BOTH buses answered id={dxl_id}")
            mismatch = True
        if not front.get(int(dxl_id)):
            print(f"MISSING on front id={dxl_id}")
            mismatch = True
    for dxl_id in DXL_REAR_BUS_IDS:
        if front.get(int(dxl_id)) and rear.get(int(dxl_id)):
            print(f"BOTH buses answered id={dxl_id}")
            mismatch = True
        if not rear.get(int(dxl_id)):
            print(f"MISSING on rear id={dxl_id}")
            mismatch = True
    return 1 if mismatch else 0


if __name__ == "__main__":
    sys.exit(main())
