"""
can_system.py

System-level CAN messages: ESTOP.
Mirrors the C API in can_system.h.
"""

import struct
import can
import can_common

# ── ESTOP Reason Codes ──
ESTOP_MANUAL      = 0
ESTOP_COMM_LOSS   = 1
ESTOP_MOTOR_ERROR = 2
ESTOP_SOFTWARE    = 3


def send_estop(bus: can.Bus, src_node: int, reason: int = ESTOP_MANUAL):
    """Send an ESTOP broadcast."""
    can_id = can_common.build_can_id(can_common.MSG_ESTOP, src_node, 0)
    data = struct.pack('<B', reason)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)


def parse_estop(msg: can.Message) -> int:
    """Parse an ESTOP message. Returns the reason code."""
    return struct.unpack('<B', msg.data[0:1])[0]
