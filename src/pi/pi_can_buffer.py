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

# // hold state (accumulator, queue) for each IMU. queue has ('time', '(x,y,z)')
can_streams = {
    imu1_a_id: {'accumulator': 0, 'queue': deque(maxlen=target)},
    imu1_g_id: {'accumulator': 0, 'queue': deque(maxlen=target)}
    # add rest
}

print("buffering joint data...")

def process_msg(msg: can.Message):
    can_id = msg.arbitration_id

    # // if the id is something like the power readings for the IMU, ignore
    if can_id not in can_streams:
        return

    # // get accumulator, and queue for the ID -> accumulate
    stream_state = can_streams[can_id]
    stream_state['accumulator'] += target

    if stream_state['accumulator'] >= hz:
        try:
            x_raw, y_raw, z_raw = struct.unpack('<hhh', msg.data[0:6])

            x = x_raw / 100.0
            y = y_raw / 100.0
            z = z_raw / 100.0

            # // mettre la donnees dans le file d'attente (+horodatage)
            data_tuple = (msg.timestamp, (x, y, z))

            stream_state['queue'].append(data_tuple)
        
        except Exception as e:
            print("Oh no, cannot unpack")
        
        # // reset accumulator :p
        stream_state['accumulator'] -= hz

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