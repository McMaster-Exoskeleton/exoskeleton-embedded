"""
can_imu.py

IMU CAN message parsing for the Raspberry Pi.
Mirrors the encoding in can_imu.c on the STM32.

The STM32 sends two frames per cycle at 500 Hz:
  MSG_IMU_ACCEL: ax, ay, az (little-endian int16 * 100) + uint16 timestamp
  MSG_IMU_GYRO:  gx, gy, gz (little-endian int16 * 100) + uint16 timestamp

Encoding:
  Bytes 0-1: X axis — int16, value = reading * 100  (e.g. 9.81 m/s² → 981)
  Bytes 2-3: Y axis — int16, value = reading * 100
  Bytes 4-5: Z axis — int16, value = reading * 100
  Bytes 6-7: timestamp — uint16, value = HAL_GetTick() & 0xFFFF (ms, wraps ~65s)

Units:
  Accel: m/s²
  Gyro:  dps (degrees per second)
"""

from typing import Tuple, Optional
import struct
import can
import can_common


def parse_imu_accel(msg: can.Message) -> Optional[Tuple[float, float, float, int]]:
    """
    Parse an IMU_ACCEL frame from an STM32 node.
    Returns (ax, ay, az, timestamp_ms) or None if the frame is not IMU_ACCEL.

    ax, ay, az in m/s².
    timestamp_ms is the STM32 HAL tick when the reading was taken (wraps at 65535).
    """
    msg_type, src_node, dest = can_common.parse_can_id(msg.arbitration_id)

    if msg_type != can_common.MSG_IMU_ACCEL:
        return None
    if len(msg.data) < 8:
        return None

    raw_ax, raw_ay, raw_az, ts = struct.unpack('<hhhH', msg.data[0:8])
    return raw_ax / 100.0, raw_ay / 100.0, raw_az / 100.0, ts


def parse_imu_gyro(msg: can.Message) -> Optional[Tuple[float, float, float, int]]:
    """
    Parse an IMU_GYRO frame from an STM32 node.
    Returns (gx, gy, gz, timestamp_ms) or None if the frame is not IMU_GYRO.

    gx, gy, gz in dps (degrees per second).
    timestamp_ms is the STM32 HAL tick when the reading was taken (wraps at 65535).
    """
    msg_type, src_node, dest = can_common.parse_can_id(msg.arbitration_id)

    if msg_type != can_common.MSG_IMU_GYRO:
        return None
    if len(msg.data) < 8:
        return None

    raw_gx, raw_gy, raw_gz, ts = struct.unpack('<hhhH', msg.data[0:8])
    return raw_gx / 100.0, raw_gy / 100.0, raw_gz / 100.0, ts


def get_src_node(msg: can.Message) -> int:
    """Extract which STM32 node sent this IMU frame."""
    _, src_node, _ = can_common.parse_can_id(msg.arbitration_id)
    return src_node
