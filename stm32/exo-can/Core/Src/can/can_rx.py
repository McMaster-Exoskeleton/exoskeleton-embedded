import can

BUS = can.interface.Bus(channel="can0", interface="socketcan")

print("Listening on can0...")
for msg in BUS:
    data_hex = msg.data.hex().upper()
    print(f"ID=0x{msg.arbitration_id:X} DLC={msg.dlc} DATA={data_hex}")
