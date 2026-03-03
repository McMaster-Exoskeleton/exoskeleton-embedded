import can
import time

BUS = can.interface.Bus(channel="can0", interface="socketcan")

MSG_ID = 0x123

print("Sending on can0...")
counter = 0
while True:
    payload = bytes([counter & 0xFF, 0xAA, 0x55, 0x00, 0x11, 0x22, 0x33, 0x44])
    msg = can.Message(arbitration_id=MSG_ID, data=payload, is_extended_id=False)
    BUS.send(msg)
    print(f"Sent ID=0x{MSG_ID:X} DATA={payload.hex().upper()}")
    counter += 1
    time.sleep(0.1)
    
## NEED TO TEST AND CONFIGURE WHEN CONNECTED##