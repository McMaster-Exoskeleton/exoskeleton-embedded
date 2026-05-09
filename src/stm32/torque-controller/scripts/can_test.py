#!/usr/bin/env python3
"""
Minimal CAN test — single file, no imports from other project modules.
Tests sending a standard CAN frame on can1.
"""
import can
import struct

CHANNEL = "can1"

print(f"Opening {CHANNEL}...")
bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
print(f"Bus open: {bus}")

# Build a TORQUE_CMD to node 1 (Left Hip):
#   msg_type=1, src=0 (Pi), dest=1 → CAN ID = (1<<7)|(0<<4)|1 = 0x081
can_id = 0x081
torque_nm = 5.0
raw = int(torque_nm * 1000)  # 5000
data = struct.pack('<h', raw)

print(f"Sending: ID=0x{can_id:03X} data={data.hex()} ({torque_nm} Nm)")

msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)

try:
    bus.send(msg)
    print("Send returned OK")
except can.CanError as e:
    print(f"Send FAILED: {e}")

bus.shutdown()
print("Done")
