import can
import struct

# CAN ID layout helpers
CAN_MSG_IMU_ACCEL = 0x3
CAN_MSG_IMU_GYRO = 0x4
CAN_NODE_PI = 0


def can_get_msg_type(can_id: int) -> int:
    return (can_id >> 7) & 0x0F


def can_get_src_node(can_id: int) -> int:
    return (can_id >> 4) & 0x07


def can_get_dest(can_id: int) -> int:
    return can_id & 0x0F


# Initialize bus
bus = can.interface.Bus(channel="can1", interface="socketcan")

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
    msg_type = can_get_msg_type(msg.arbitration_id)
    src_node = can_get_src_node(msg.arbitration_id)
    dest = can_get_dest(msg.arbitration_id)

    if dest != CAN_NODE_PI:
        continue

    if msg_type == CAN_MSG_IMU_ACCEL:
        try:
            ax_raw, ay_raw, az_raw = struct.unpack('<hhh', msg.data[0:6])

            # Divide by 100 because STM32 sends int16 values scaled by 100
            telemetry["ax"] = ax_raw / 100.0
            telemetry["ay"] = ay_raw / 100.0
            telemetry["az"] = az_raw / 100.0
        except Exception as e:
            print(f"Error in retrieving acceleration data: {e}")

    elif msg_type == CAN_MSG_IMU_GYRO:
        try:
            gx_raw, gy_raw, gz_raw = struct.unpack('<hhh', msg.data[0:6])

            telemetry["gx"] = gx_raw / 100.0
            telemetry["gy"] = gy_raw / 100.0
            telemetry["gz"] = gz_raw / 100.0
        except Exception as e:
            print(f"Error in retrieving gyroscope data: {e}")
    else:
        continue

    print(f"Source Node: {src_node} | CAN ID: 0x{msg.arbitration_id:03X}")
    print(f"Accel -> X: {telemetry['ax']:6.2f} | Y: {telemetry['ay']:6.2f} | Z: {telemetry['az']:6.2f} m/s^2")
    print(f"Gyro -> X: {telemetry['gx']:6.2f} | Y: {telemetry['gy']:6.2f} | Z: {telemetry['gz']:6.2f} deg/s")