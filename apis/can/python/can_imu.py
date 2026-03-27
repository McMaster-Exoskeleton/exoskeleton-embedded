"""
can_imu.py

IMU CAN messages: accelerometer and gyroscope data.
Mirrors the C API in can_imu.h.
All values are little-endian int16 scaled by 100.
"""

import struct
import can
from . import can_common


def send_imu_accel(bus: can.Bus, src_node: int, ax: float, ay: float, az: float):
    """Send IMU accelerometer data. ax, ay, az in m/s^2."""
    can_id = can_common.build_can_id(can_common.MSG_IMU_ACCEL, src_node, can_common.NODE_PI)
    data = struct.pack('<hhh', int(ax * 100), int(ay * 100), int(az * 100))
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def send_imu_gyro(bus: can.Bus, src_node: int, gx: float, gy: float, gz: float):
    """Send IMU gyroscope data. gx, gy, gz in deg/s."""
    can_id = can_common.build_can_id(can_common.MSG_IMU_GYRO, src_node, can_common.NODE_PI)
    data = struct.pack('<hhh', int(gx * 100), int(gy * 100), int(gz * 100))
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def parse_imu_accel(msg: can.Message) -> tuple[float, float, float]:
    """Parse IMU accelerometer data. Returns (ax, ay, az) in m/s^2."""
    ax, ay, az = struct.unpack('<hhh', msg.data[0:6])
    return ax / 100.0, ay / 100.0, az / 100.0


def parse_imu_gyro(msg: can.Message) -> tuple[float, float, float]:
    """Parse IMU gyroscope data. Returns (gx, gy, gz) in deg/s."""
    gx, gy, gz = struct.unpack('<hhh', msg.data[0:6])
    return gx / 100.0, gy / 100.0, gz / 100.0
