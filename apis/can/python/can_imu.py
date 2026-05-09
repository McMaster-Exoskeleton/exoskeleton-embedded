"""
can_imu.py

IMU CAN messages: accelerometer and gyroscope+position data.
Mirrors the C API in can_imu.h.

Wire format:
  IMU_ACCEL (DLC 6): ax, ay, az -> int16 * 100  (m/s^2)
  IMU_GYRO  (DLC 8): gx, gy, gz -> int16 * 100  (deg/s)
                     motor_pos  -> int16 *  10  (degrees)

The motor position is co-located in the gyro frame so the IMU sample and
the joint angle share the same on-wire timestamp.
"""

import struct
import can

try:
    from . import can_common
except ImportError:
    import can_common


def send_imu_accel(bus: can.Bus, src_node: int, ax: float, ay: float, az: float):
    """Send IMU accelerometer data. ax, ay, az in m/s^2."""
    can_id = can_common.build_can_id(can_common.MSG_IMU_ACCEL, src_node, can_common.NODE_PI)
    data = struct.pack('<hhh', int(ax * 100), int(ay * 100), int(az * 100))
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def send_imu_gyro(bus: can.Bus, src_node: int, gx: float, gy: float, gz: float,
                  motor_position: float = 0.0):
    """Send IMU gyroscope data + motor position. gx,gy,gz in deg/s, position in deg."""
    can_id = can_common.build_can_id(can_common.MSG_IMU_GYRO, src_node, can_common.NODE_PI)
    data = struct.pack('<hhhh',
                       int(gx * 100), int(gy * 100), int(gz * 100),
                       int(motor_position * 10))
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def parse_imu_accel(msg: can.Message) -> tuple[float, float, float]:
    """Parse IMU accelerometer data. Returns (ax, ay, az) in m/s^2."""
    ax, ay, az = struct.unpack('<hhh', msg.data[0:6])
    return ax / 100.0, ay / 100.0, az / 100.0


def parse_imu_gyro(msg: can.Message) -> tuple[float, float, float]:
    """Parse IMU gyroscope data only. Returns (gx, gy, gz) in deg/s.
    Backward-compatible: ignores the motor_position byte if present."""
    gx, gy, gz = struct.unpack('<hhh', msg.data[0:6])
    return gx / 100.0, gy / 100.0, gz / 100.0


def parse_imu_gyro_pos(msg: can.Message) -> tuple[float, float, float, float | None]:
    """Parse IMU gyroscope + motor position. Returns (gx, gy, gz, position).
    position is None if the frame is the legacy DLC 6 form."""
    if len(msg.data) >= 8:
        gx, gy, gz, raw_pos = struct.unpack('<hhhh', msg.data[0:8])
        return gx / 100.0, gy / 100.0, gz / 100.0, raw_pos / 10.0
    gx, gy, gz = struct.unpack('<hhh', msg.data[0:6])
    return gx / 100.0, gy / 100.0, gz / 100.0, None
