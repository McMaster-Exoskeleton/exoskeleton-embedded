import can
from collections import deque

hz = 500
target = 187 # // request by dilly

# format: IMU#_GYRO/ACCEL_ID
imu1_a_id = 0x123
imu1_g_id = 0x124
''' for the future
imu2_a_id = 0x125
imu2_g_id = 0x126

imu3_a_id = 0x127
imu3_g_id = 0x128

imu4_a_id = 0x129
imu4_g_id = 0x130
'''

imus = {
    "imu1": {
        'accumulator': 0,
        'queue': deque(maxlen=target),
        'temp_accel': None,
        'temp_gyro': None,
        'timestampe': 0.0
        # // encoder reading to be added?
    }
    # // future IMUS, add below
}

print("buffering joint data...")

def process_msg(msg: can.Message):
    can_id = msg.arbitration_id

    # // if the id is something like the power readings for the IMU, ignore
    if can_id == imu1_a_id:
        imu_key = "imu1"
        sensor_type = "accel"
    elif can_id == imu1_g_id:
        imu_key = "imu1"
        sensor_type = "gyro"
    else:
        return

    # // access a specific imus data for appending
    stream_state = imus[imu_key]

    try:
        x_raw, y_raw, z_raw = struct.unpack('<hhh', msg.data[0:6])
        val = (x_raw / 100.0, y_raw / 100.0, z_raw / 100.0)

        if sensor_type == "accel":
            stream_state['temp_accel'] = val
            stream_state['timestamp'] = msg.timestamp
        else:
            stream_state['temp_gyro'] = val
            # // the id before this one provided the timestamp (id's are contiguous)
    
    except Exception as e:
        print("noo, cannot unpack!")
        return

    # // its crucial that the contiguous data (accel + gyro) is a snychronized pair
    if stream_state['temp_accel'] is not None and stream_state['temp_gyro'] is not None:

        # // accumulate :p
        stream_state['accumulator'] += hz

        if stream_state['accumulator'] >= hz:
            # // mettre la donnees dans le file d'attente (+horodatage)
            data_tuple = (
                stream_state['timestamp'], 
                stream_state['temp_accel'], 
                stream_state['temp_gyro']
            )
            
            stream_state['queue'].append(data_tuple)
            
            # // reset accumulator :p
            stream_state['accumulator'] -= hz

        # // clear the temporary slots to wait for the next pair from the bus
        stream_state['temp_accel'] = None
        stream_state['temp_gyro'] = None

def main():
    print("exo telemetry, i choose you")

    try:
        bus = can.interface.Bus(channel='can1', interface='socketcan')

        for msg in bus:
            process_msg(msg)

    except can.CanError as e:
        print(f"CAN error: {e}")
    except KeyboardInterrupt:
        print("you stopped it")
    finally:
        # // this might actually remove that annoying wall of
        # // text we get when we CTRL + C the python file lol
        if 'bus' in locals():
            bus.shutdown()

if __name__ == "__main__":
    main()