import can
import struct

# Initialize bus
bus = can.interface.Bus(channel="can0", interface="socketcan")

print("Exoskeleton Telemetry Started...")

for msg in bus:
    if msg.arbitration_id == 0x123:
        # data[0:6] contains AX, AY, AZ packed as 16-bit signed integers (Big Endian)
        # '>hhh' tells Python to unpack 3 signed shorts
        try:
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', msg.data[0:6])
            
            # Divide by 100 because we multiplied by 100 in main.c
            ax = ax_raw / 100.0
            ay = ay_raw / 100.0
            az = az_raw / 100.0
            
            print(f"Accel -> X: {ax:6.2f} | Y: {ay:6.2f} | Z: {az:6.2f} m/s²")
        except Exception as e:
            print(f"Data error: {e}")
