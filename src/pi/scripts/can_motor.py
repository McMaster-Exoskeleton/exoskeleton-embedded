"""
can_motor.py

Motor CAN messages: torque commands and motor status.
Mirrors the C API in can_motor.h.
"""

import struct
import can
import can_common


def send_torque_cmd(bus: can.Bus, dest_node: int, torque_nm: float,
                    src_node: int = can_common.NODE_PI):
    """
    Send a torque command to a specific STM32 node.
    torque_nm in Nm. Range: +/-32 Nm.
    src_node defaults to NODE_PI (0) but can be overridden for testing.
    """
    can_id = can_common.build_can_id(can_common.MSG_TORQUE_CMD, src_node, dest_node)
    raw = int(torque_nm * 1000)
    data = struct.pack('<h', raw)
    print(f"  [DEBUG] CAN ID=0x{can_id:03X} data={data.hex()} raw={raw}")
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"  [DEBUG] send OK")


def parse_torque_cmd(msg: can.Message) -> float:
    """Parse a torque command. Returns torque in Nm."""
    raw = struct.unpack('<h', msg.data[0:2])[0]
    return raw / 1000.0


def send_motor_status(bus: can.Bus, src_node: int, position: float, speed: float,
                      current: float, temperature: int, error: int):
    """
    Send motor status to the Pi.
    position: degrees, speed: ERPM, current: amps,
    temperature: degrees C, error: MotorErrorCode.
    """
    can_id = can_common.build_can_id(can_common.MSG_MOTOR_STATUS, src_node, can_common.NODE_PI)
    raw_pos = int(position * 10)
    raw_spd = int(speed / 10)
    raw_cur = int(current * 100)
    data = struct.pack('<hhhbB', raw_pos, raw_spd, raw_cur, temperature, error)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def parse_motor_status(msg: can.Message) -> tuple[float, float, float, int, int]:
    """
    Parse motor status. Returns (position, speed, current, temperature, error).
    position in degrees, speed in ERPM, current in amps.
    """
    raw_pos, raw_spd, raw_cur, temp, err = struct.unpack('<hhhbB', msg.data[0:8])
    return raw_pos / 10.0, raw_spd * 10.0, raw_cur / 100.0, temp, err
