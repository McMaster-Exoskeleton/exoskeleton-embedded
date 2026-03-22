import can
import struct

# Initialize bus
bus = can.interface.Bus(channel="can0", interface="socketcan")

print("Exoskeleton Telemetry Started...")

telemetry = {
    "ax": 0.0, 
    "ay": 0.0, 
    "az": 0.0, 
    "gx": 0.0, 
    "gy": 0.0, 
    "gz": 0.0
}

for msg in bus:
    if msg.arbitration_id == 0x123:
        # data[0:6] contains data triplet packed as 16-bit signed integers (Big Endian)
        # '>hhh' tells Python to unpack 3 signed shorts
        try:
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', msg.data[0:6])

            # Divide by 100 because we multiplied by 100 in main.c
            telemetry["ax"] = ax_raw / 100.0
            telemetry["ay"] = ay_raw / 100.0
            telemetry["az"] = az_raw / 100.0
        except Exception as e:
            print(f"Error in retrieving acceleration data: {e}")

    elif msg.arbitration_id == 0x124:
        try:
            gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', msg.data[0:6])

            telemetry["gx"] = gx_raw / 100.0
            telemetry["gy"] = gy_raw / 100.0
            telemetry["gz"] = gz_raw / 100.0
        except Exception as e:
            print(f"Error in retrieving gyroscope data: {e}")

    print(f"Accel -> X: {telemetry['ax']:6.2f} | Y: {telemetry['ay']:6.2f} | Z: {telemetry['az']:6.2f} m/s^2")
    print(f"Gyro -> X: {telemetry['gx']:6.2f} | Y: {telemetry['gy']:6.2f} | Z: {telemetry['gz']:6.2f} deg/s")