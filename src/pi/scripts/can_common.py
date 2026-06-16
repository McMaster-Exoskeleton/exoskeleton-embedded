"""
can_common.py

CAN API transport layer for the Raspberry Pi.
Constants, ID helpers, bus setup, and receive function.
Mirrors the C API in can_common.h.
"""

import can

# ── Node IDs ──
NODE_PI         = 0
NODE_LEFT_HIP   = 1
NODE_RIGHT_HIP  = 2
NODE_LEFT_KNEE  = 3
NODE_RIGHT_KNEE = 4

# ── Message Type IDs ──
MSG_ESTOP        = 0x0
MSG_TORQUE_CMD   = 0x1
MSG_MOTOR_STATUS = 0x2
MSG_IMU_ACCEL    = 0x3
MSG_IMU_GYRO     = 0x4
MSG_HEARTBEAT    = 0x5


def build_can_id(msg_type: int, src_node: int, dest: int) -> int:
    """Build an 11-bit CAN ID from message type, source node, and destination."""
    return ((msg_type & 0x0F) << 7) | ((src_node & 0x07) << 4) | (dest & 0x0F)


def parse_can_id(can_id: int) -> tuple[int, int, int]:
    """Extract (msg_type, src_node, dest) from an 11-bit CAN ID."""
    msg_type = (can_id >> 7) & 0x0F
    src_node = (can_id >> 4) & 0x07
    dest = can_id & 0x0F
    return msg_type, src_node, dest


def create_bus(channel: str = "can1") -> can.Bus:
    """
    Create a SocketCAN bus interface.
    Bitrate must be configured at the OS level before calling this:
        sudo ip link set can0 up type can bitrate 1000000
    """
    return can.interface.Bus(channel=channel, interface="socketcan")


def recv(bus: can.Bus, timeout: float = 0.01) -> can.Message | None:
    """
    Receive a CAN message with timeout.
    Returns a can.Message or None if no message arrived within timeout.
    """
    return bus.recv(timeout=timeout)
